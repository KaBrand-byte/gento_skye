#!/usr/bin/env python3
"""locmatch.py [check|search R DEG|global] [--apply]
Scores the live lidar scan against map_2d.png at the reported pose; optionally searches for a better pose
and (with --apply) relocates via slam/Relocation (vendor convention: orientation.w = yaw rad)."""
import os, re, sys, time, math, json, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from slam.msg import LocationData

MAP_ROOT = '/home/sunrise/robot/config/map/'
SERVER_CFG = '/home/sunrise/robot/config/param/robot_server.json'    # {"map_id": N}: the vendor's current map
LOC_LOG = '/mnt/ssd/log/location.log'                                # "加载地图成功<path>": map the localisation loaded


def loaded_map_id():
    """Map id the vendor localisation last loaded (from the end of its log), or None."""
    try:
        with open(LOC_LOG, 'rb') as f:
            f.seek(0, 2); f.seek(max(0, f.tell() - 4000000))
            tail = f.read().decode('utf-8', 'ignore')
        hits = re.findall(r'加载地图成功\S*/config/map/(\d+)/', tail)
        return int(hits[-1]) if hits else None
    except OSError:
        return None


def map_dir():
    """Directory of the map in use: GENTO_MAP_DIR, else the vendor's current map (robot_server.json map_id), else
    the map the localisation last loaded.  Re-mapping therefore needs no change here."""
    if os.environ.get('GENTO_MAP_DIR'):
        return os.environ['GENTO_MAP_DIR'].rstrip('/') + '/', 'GENTO_MAP_DIR'
    try:
        mid = int(json.load(open(SERVER_CFG))['map_id'])
        if os.path.exists(MAP_ROOT + '%d/floor1/map_2d.bmap' % mid):
            return MAP_ROOT + '%d/floor1/' % mid, 'map_id %d' % mid
    except (OSError, ValueError, KeyError, TypeError):
        pass
    mid = loaded_map_id()
    if mid is not None and os.path.exists(MAP_ROOT + '%d/floor1/map_2d.bmap' % mid):
        return MAP_ROOT + '%d/floor1/' % mid, 'map %d (location.log)' % mid
    raise SystemExit('ABORT: cannot tell which vendor map is in use (%s, %s)' % (SERVER_CFG, LOC_LOG))


D, D_SRC = map_dir()
RES = 0.05
_b = json.load(open(D + 'map_2d.bmap'))
_o = np.array([[q['x'], q['y']] for q in _b['normalPosList']])
_lid = loaded_map_id()
MAP_NOTE = None
if D_SRC.startswith('map_id') and _lid is not None and _lid != int(D_SRC.split()[1]):
    MAP_NOTE = 'vendor current map is %s but the localisation last loaded map %d' % (D_SRC, _lid)
OX, OY = _o[:, 0].min() - 3.0, _o[:, 1].min() - 3.0
W = int((_o[:, 0].max() + 3.0 - OX) / RES) + 1; H = int((_o[:, 1].max() + 3.0 - OY) / RES) + 1
occ = np.zeros((H, W), bool)
occ[(H - 1 - np.round((_o[:, 1] - OY) / RES)).astype(int), np.round((_o[:, 0] - OX) / RES).astype(int)] = True
dt = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3) * RES   # metres to nearest obstacle
rclpy.init(); n = Node('claude_locmatch'); C = []; L = [None]
n.create_subscription(PointCloud2, '/driver/lidar/point_cloud/Data', lambda m: C.append(m) if len(C) < 5 else None, qos_profile_sensor_data)
n.create_subscription(LocationData, 'slam/LocationData', lambda m: L.__setitem__(0, m), qos_profile_sensor_data)
t = time.time()
while (len(C) < 5 or L[0] is None) and time.time() - t < 6:
    rclpy.spin_once(n, timeout_sec=0.05)
# both come from the vendor programs (linkhou_robot.service): a clear message instead of a traceback (2026-11-18)
_missing = [w for w, ok in (('lidar point cloud', C), ('localisation pose (slam/LocationData)', L[0] is not None)) if not ok]
if _missing:
    raise SystemExit('ABORT: no %s in 6 s - the vendor chassis programs are not running or not publishing '
                     '(linkhou_robot.service): run "gento doctor"; power-cycle the chassis (or: sudo systemctl '
                     'restart linkhou_robot)' % ' and no '.join(_missing))
a = np.concatenate([np.stack([s['x'], s['y'], s['z']], 1).astype(float)
                    for s in (pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) for m in C)])
a = a[np.isfinite(a).all(1)]
a = a[(a[:, 2] > 0.15) & (a[:, 2] < 1.0) & (np.hypot(a[:, 0], a[:, 1]) > 0.5) & (np.hypot(a[:, 0], a[:, 1]) < 12)]
# thin to one point per 5 cm cell
k = np.unique(np.floor(a[:, :2] / 0.05).astype(int), axis=0, return_index=True)[1]
P = a[k, :2]
p = L[0].pose; X0, Y0, T0 = p.position.x, p.position.y, p.orientation.z


def score(x, y, th, pts=P):
    c, s = math.cos(th), math.sin(th)
    mx = x + c * pts[:, 0] - s * pts[:, 1]; my = y + s * pts[:, 0] + c * pts[:, 1]
    u = np.round((mx - OX) / RES).astype(int); v = (H - 1 - np.round((my - OY) / RES)).astype(int)
    ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    d = np.full(len(pts), 1.0); d[ok] = dt[v[ok], u[ok]]
    return float((d < 0.10).mean())


def refine(b):
    for step, ang in ((0.05, 1.0), (0.02, 0.4), (0.01, 0.2)):
        improved = True
        while improved:
            improved = False
            for dx, dy, dth in ((step, 0, 0), (-step, 0, 0), (0, step, 0), (0, -step, 0), (0, 0, math.radians(ang)), (0, 0, -math.radians(ang))):
                sc = score(b[1] + dx, b[2] + dy, b[3] + dth)
                if sc > b[0] + 1e-4:
                    b = (sc, b[1] + dx, b[2] + dy, b[3] + dth); improved = True
    return b


s0 = score(X0, Y0, T0)
res = dict(reported=[round(X0, 3), round(Y0, 3), round(math.degrees(T0), 1)], conf=L[0].confidence, pts=len(P), match=round(s0, 3),
           map=D_SRC)
if MAP_NOTE:
    res['map_warning'] = MAP_NOTE
mode = sys.argv[1] if len(sys.argv) > 1 else 'check'
best = (s0, X0, Y0, T0)
cands = []
if mode in ('search', 'global'):
    sub = P[np.random.default_rng(0).choice(len(P), min(250, len(P)), replace=False)]
    if mode == 'search':
        R = float(sys.argv[2]); DG = float(sys.argv[3])
        xs = np.arange(X0 - R, X0 + R + 1e-9, 0.1); ys = np.arange(Y0 - R, Y0 + R + 1e-9, 0.1)
        ths = T0 + np.radians(np.arange(-DG, DG + 1e-9, 2.0))
    else:
        # the whole mapped area (inside the outermost walls), not a fixed box of one particular map
        x0, y0 = np.percentile(_o, 1, axis=0); x1, y1 = np.percentile(_o, 99, axis=0)
        xs = np.arange(x0, x1, 0.15); ys = np.arange(y0, y1, 0.15); ths = np.radians(np.arange(-180, 180, 4.0))
        res['search_box'] = [round(float(v), 2) for v in (x0, x1, y0, y1)]
    for th in ths:
        for x in xs:
            for y in ys:
                sc = score(x, y, th, sub)
                if sc > 0.6:
                    cands.append((sc, x, y, th))
                if sc > best[0]:
                    best = (sc, x, y, th)
    best = refine(best)
    res['best'] = [round(best[1], 3), round(best[2], 3), round(math.degrees(best[3]), 2)]
    res['best_match'] = round(best[0], 3)
    # ambiguity: refine the strongest clearly-different candidates with all points and compare like with like
    def far(c, b):
        return math.hypot(c[1] - b[1], c[2] - b[2]) > 0.5 or abs((c[3] - b[3] + math.pi) % (2 * math.pi) - math.pi) > math.radians(20)
    alt = sorted([c for c in cands if far(c, best)], reverse=True); picked = []
    for c in alt:
        if all(far(c, p) for p in picked):
            picked.append(c)
        if len(picked) == 3:
            break
    ref = [refine((score(c[1], c[2], c[3]), c[1], c[2], c[3])) for c in picked]
    ref = [r for r in ref if far(r, best)]
    sec = max(ref, default=(0.0, 0, 0, 0))
    res['second_best'] = round(sec[0], 3)
    if sec[0] > 0:
        res['second_pose'] = [round(sec[1], 2), round(sec[2], 2), round(math.degrees(sec[3]), 1)]
    res['ambiguous'] = res['second_best'] > best[0] - 0.10
elif mode == 'at':
    best = refine((score(float(sys.argv[2]), float(sys.argv[3]), math.radians(float(sys.argv[4]))),
                   float(sys.argv[2]), float(sys.argv[3]), math.radians(float(sys.argv[4]))))
    res['best'] = [round(best[1], 3), round(best[2], 3), round(math.degrees(best[3]), 2)]
    res['best_match'] = round(best[0], 3)
print(json.dumps(res), flush=True)
if '--apply' in sys.argv and mode != 'check' and (res.get('ambiguous') or best[0] < 0.85):
    print('NOT APPLIED: match too low or ambiguous; relocate by hand on the web page', flush=True)
elif '--apply' in sys.argv and mode != 'check':
    from slam.srv import Relocation
    c = n.create_client(Relocation, 'slam/Relocation'); c.wait_for_service(timeout_sec=5)
    r = Relocation.Request()
    fields = r.get_fields_and_field_types()
    pose = r.pose if 'pose' in fields else getattr(r, [k for k, v in fields.items() if 'Pose' in v][0])
    pose.position.x = best[1]; pose.position.y = best[2]; pose.orientation.w = best[3]
    f = c.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=10); print('relocation ->', f.result())
    t = time.time()
    while time.time() - t < 4:
        rclpy.spin_once(n, timeout_sec=0.05)
    p = L[0].pose; print('after: %.3f %.3f %.1fdeg conf %d match %.3f' % (p.position.x, p.position.y, math.degrees(p.orientation.z), L[0].confidence, score(p.position.x, p.position.y, p.orientation.z)))

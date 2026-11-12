#!/usr/bin/env python3
"""Precise docking for Gento (body-frame lidar, no reliance on map pose for the last stretch).
  dock.py measure [near]                     fit the face in front (body frame), no motion
  dock.py approach GAP [--dry]               at staging: fix heading, then drive STRAIGHT until chassis front is GAP m from the face
  dock.py retreat DIST                       drive STRAIGHT back DIST m (rear ultrasonic checked)
  dock.py run STAGING_STATION_ID GAP [--dry] navigate to staging (turns happen only there), then approach
Rules: every turn / straight move is checked against the live 3-D lidar with the arm-aware robot model
(robot_model.py): turns back off first if anything is inside the swept circle of chassis+torso+arms, the final
approach slides along the face (within the station's lat_window) to the nearest offset where the arms' path and
work zone are clear, and never moves forward more than MAX_FWD.
"""
import os, sys, time, math, json, subprocess, numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from move.msg import State
from move.srv import SetControlMode, SetParkingMode
from move.action import MoveRelative
HERE = os.path.dirname(os.path.abspath(__file__))      # install dir (stations.json, logs, helpers)
sys.path.insert(0, HERE)
import robot_model as RM
CFG = RM.load()

FRONT = 0.338          # chassis centre -> front edge
GRIP = 0.33            # gripper tip beyond chassis front (worst case)
MAX_FWD = 0.80
ANG_TOL = math.radians(0.4)
YAW_OFF = [0.0]                  # station fine-tune (rad), set from stations.json yaw_offset
ROT_SHORT = math.radians(0.77)   # measured rotation shortfall of MoveRelative
FWD_SHORT = 0.011                # measured: straight moves stop ~11 mm short
YLIM = [0.9]
ZB = [0.10, 0.70]      # height band used for the face fit (override: --z LO HI)

rclpy.init()
n = Node('claude_dock')
S = [None]; U = [None]; C = []
n.create_subscription(State, 'move/State', lambda m: S.__setitem__(0, m), 10)
try:
    from driver.msg import RadarsData
    n.create_subscription(RadarsData, 'driver/radar/Data',
                          lambda m: U.__setitem__(0, {d.id: d.range for d in m.data}), qos_profile_sensor_data)
except Exception:
    pass
n.create_subscription(PointCloud2, '/driver/lidar/point_cloud/Data',
                      lambda m: C.append(m) if len(C) < 8 else None, qos_profile_sensor_data)


def spin(t):
    e = time.time() + t
    while time.time() < e:
        rclpy.spin_once(n, timeout_sec=0.05)


T0 = time.time()
FRAMES = 6                     # lidar frames per snapshot (10 Hz)
NAV_SPEED = float(os.environ.get('GENTO_NAV_SPEED', '0.35'))   # vendor max_speed.linear is 0.35 m/s
MOVES = [0]                    # incremented by every relative move: invalidates the cloud cache
_CACHE = {'moves': -1, 't': 0.0, 'a': None}


def log(*a):
    print('[%6.1fs]' % (time.time() - T0), *a, flush=True)


def pub(m):
    return json.dumps({k: v for k, v in m.items() if k not in ('err', 'runs')}) if m else 'no face'


def cloud(fresh=False):
    """Lidar snapshot (base_link).  Reused while the robot has not moved since and it is < 2 s old."""
    if not fresh and _CACHE['moves'] == MOVES[0] and time.time() - _CACHE['t'] < 2.0:
        return _CACHE['a']
    C.clear(); spin(0.15); C.clear(); t = time.time()
    while len(C) < FRAMES and time.time() - t < 4:
        rclpy.spin_once(n, timeout_sec=0.05)
    a = np.concatenate([np.stack([s['x'], s['y'], s['z']], 1).astype(float)
                        for s in (pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) for m in list(C))])
    a = a[np.isfinite(a).all(1)]
    _CACHE.update(moves=MOVES[0], t=time.time(), a=a)
    return a


def need_clear(h, what):
    if h:
        raise SystemExit('ABORT: %s blocked by obstacle %s' % (what, json.dumps(h)))


def move_to_turn_spot(max_d=0.30):
    """If turning here would hit something (chassis, torso, arms or tray), make the shortest collision-free
    straight move (back / sideways / diagonal-back) after which turning is clear.  Re-checks after moving."""
    for _ in range(3):
        p = cloud(); mv = RM.leave_move(p, CFG, max_d)      # exact shortest move, computed in one pass
        if mv is None and max_d >= 0.6:
            # e.g. docked at a station with something beside the approach: no single straight move reaches a
            # turning spot, but reversing straight out first (the way the robot came in) and then one more
            # straight move does
            for back in (0.3, 0.45, 0.6, 0.75, 0.9):
                if RM.sweep_hits(p, CFG, -back, 0.0):
                    break
                if RM.leave_move(p + np.array([back, 0.0, 0.0]), CFG, max_d) is not None:
                    log('no single move reaches a turning spot: reverse straight %.2f m first' % back)
                    rel(-back, 0.0, rear_check=True)
                    mv = 'moved'
                    break
            if mv == 'moved':
                continue
        if mv is None:
            raise SystemExit('ABORT: no clear spot to turn within %.2f m: %s' % (max_d, json.dumps(RM.rotation_hits(p, CFG))))
        if mv == (0.0, 0.0):
            return
        log('turning here would hit %s: move straight x=%.2f y=%.2f first' % (json.dumps(RM.rotation_hits(p, CFG)), mv[0], mv[1]))
        rel(mv[0], 0.0, rear_check=mv[0] < 0, y=mv[1])     # never more than the checked move; re-checked
    need_clear(RM.rotation_hits(cloud(), CFG), 'turning (still blocked after moving)')


def safe_rotate(yaw):
    """Small alignment turns are checked over the arc actually swept; larger ones need a clear full circle."""
    if abs(yaw) <= math.radians(12) and not RM.turn_hits(cloud(), CFG, yaw):
        rel(0.0, yaw)
        return
    move_to_turn_spot()
    rel(0.0, yaw)


def measure(near=False):
    a = cloud(fresh=True)
    if near:   # exclude the gripper zone only if the fit band reaches gripper height
        sel = (a[:, 0] > FRONT + 0.02) & (a[:, 0] < 1.3) & (np.abs(a[:, 1]) < YLIM[0])
        if ZB[1] > 0.75:
            sel &= ~((a[:, 0] < FRONT + GRIP + 0.05) & (np.abs(a[:, 1]) < 0.55))
    else:
        sel = (a[:, 0] > FRONT + GRIP + 0.08) & (a[:, 0] < 2.2) & (np.abs(a[:, 1]) < YLIM[0])
    sel &= (a[:, 2] >= ZB[0]) & (a[:, 2] < ZB[1])
    xy = a[sel][:, :2]
    if len(xy) < 40:
        return None
    rng = np.random.default_rng(1); best = (None, None)
    sub = xy if len(xy) <= 3000 else xy[rng.choice(len(xy), 3000, replace=False)]   # RANSAC on a sample
    for _ in range(300):
        i, j = rng.choice(len(sub), 2, replace=False); d = sub[j] - sub[i]; L = np.linalg.norm(d)
        if L < 0.15:
            continue
        d /= L
        if abs(d[1]) < 0.85:      # want a face roughly across the heading
            continue
        nr = np.array([-d[1], d[0]]); inl = np.abs((sub - sub[i]) @ nr) < 0.012
        sc = inl.sum() - 50 * abs(sub[i] @ nr) * len(sub) / 3000.0   # prefer the nearest face
        if best[0] is None or sc > best[0]:
            best = (sc, (sub[i].copy(), nr))
    if best[1] is None:
        return None
    p0, nr0 = best[1]
    pts = xy[np.abs((xy - p0) @ nr0) < 0.012]               # all points of the chosen line, full resolution
    if len(pts) < 30:
        return None
    cen = pts.mean(0); _, _, vt = np.linalg.svd(pts - cen); d = vt[0]
    nr = np.array([d[1], -d[0]])
    if nr[0] < 0:
        nr = -nr                   # normal pointing forward (+x)
    dist = float(cen @ nr); err = math.atan2(nr[1], nr[0])   # +ve: face normal to the left -> turn left
    rms = float(np.sqrt((((pts - cen) @ nr) ** 2).mean())); t_ = np.sort((pts - cen) @ d)
    cut = np.where(np.diff(t_) > 0.04)[0]; runs = np.split(t_, cut + 1)
    t0 = -(cen @ d)                          # where the robot's centre line meets the face
    run = min(runs, key=lambda r: 0 if r[0] <= t0 <= r[-1] else min(abs(r[0] - t0), abs(r[-1] - t0)))
    ends = [cen + d * run[0], cen + d * run[-1]]
    return dict(dist=round(dist, 4), gap=round(dist - FRONT, 4), ang_deg=round(math.degrees(err), 2), err=err,
                n=int(len(pts)), rms=round(rms, 4), span=round(float(t_.max() - t_.min()), 2),
                ends=[[round(float(e[0]), 3), round(float(e[1]), 3)] for e in ends],
                runs=[[round(float((cen + d * r[0])[1]), 4), round(float((cen + d * r[-1])[1]), 4)] for r in runs if len(r) > 15])


_CLIENTS = {}


def _client(kind, srv, name):
    if name not in _CLIENTS:
        _CLIENTS[name] = (n.create_client(srv, name) if kind == 'srv' else ActionClient(n, srv, name))
    return _CLIENTS[name]


def _wait_state(pred, timeout):
    t = time.time()
    while time.time() - t < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        if S[0] is not None and pred(S[0]):
            return True
    return False


def setmode(v):
    if v == 0:
        _PARK_OK[0] = False                     # in auto mode the chassis parks by itself
    c = _client('srv', SetControlMode, 'move/SetControlMode'); c.wait_for_service(timeout_sec=5)
    r = SetControlMode.Request(); r.mode = v; f = c.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    _wait_state(lambda s: s.is_soft_auto == (v == 0), 1.0)     # instead of a fixed 0.8 s pause


_PARK_OK = [False]   # parking released by us and nothing (navigation / auto mode) re-engaged it since


def unpark(force=False):
    """Release parking.  The chassis re-engages parking by itself after navigation, and the release waits for
    the wheels to zero (~0.7 s; a relative move sent earlier is rejected), so release unless we did it right
    before the previous relative move and nothing else has run since."""
    if not force and _PARK_OK[0] and S[0] is not None and not S[0].is_parking_mode and not S[0].is_soft_auto:
        return
    c = _client('srv', SetParkingMode, 'move/SetParkingMode'); c.wait_for_service(timeout_sec=5)
    r = SetParkingMode.Request(); r.enable = False; f = c.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    _wait_state(lambda s: not s.is_parking_mode, 2.0)
    spin(0.8)                                   # wheels to zero ("Parking release completed")
    _PARK_OK[0] = True


def rel(x=0.0, yaw=0.0, rear_check=False, y=0.0):
    if S[0].is_soft_auto:
        setmode(1)
    unpark()
    ac = _client('action', MoveRelative, 'move/MoveRelativeCmd')
    if not ac.wait_for_server(timeout_sec=5):
        raise SystemExit('ABORT: no MoveRelative server')
    MOVES[0] += 1
    g = MoveRelative.Goal()
    g.relative_pose.position.x = float(x); g.relative_pose.position.y = float(y)
    g.relative_pose.orientation.w = float(yaw)        # vendor convention: w = yaw (rad)
    for attempt in range(4):
        gf = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, gf, timeout_sec=5); gh = gf.result()
        if gh and gh.accepted:
            break
        # typically "parking mode enabled": releasing parking waits for the wheels to zero (~0.7 s)
        log('  relative move rejected (attempt %d) - releasing parking and retrying' % (attempt + 1))
        spin(0.5); unpark(force=True); spin(0.3)
    else:
        raise SystemExit('ABORT: relative move rejected (check parking / soft mode)')
    rf = gh.get_result_async(); t0 = time.time()
    while not rf.done() and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05)
        if rear_check and U[0] and min(U[0].get(5, 9), U[0].get(6, 9)) < 0.25:
            log('SAFETY: rear < 0.25 m, cancel'); cf = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(n, cf, timeout_sec=3); raise SystemExit(1)
    r = rf.result().result if rf.done() else None; spin(0.4)      # let the chassis settle before measuring
    log('  rel move x=%.3f y=%.3f yaw=%.2fdeg -> %s' % (x, y, math.degrees(yaw), (r.error_code, r.error_msg) if r else 'TIMEOUT'))
    if not r or r.error_code != 0:
        raise SystemExit('ABORT: relative move failed')


ST_FILE = os.path.join(HERE, 'stations.json')
LOG_FILE = os.path.join(HERE, 'dock_log.csv')
LAT_GAIN = 1.0          # small sideways moves are erratic (under- and overshoot): no gain
CLOSEST_SLACK = 0.01   # margin kept when stopping 'as close as possible'
# The final approach is lidar-relative (edge distance, angle, end point), so the staging point only needs to be
# roughly reached: within this distance and standing still counts as arrived (the vendor server keeps its task
# open beyond ~5 cm, e.g. when its localisation corrected itself on the way).
STAGING_TOL = 0.35
LOC_MIN_MATCH = 0.60   # scan-to-map match below this before navigating = localisation clearly wrong -> stop
LAT_TOL = 0.010         # the chassis cannot place sideways moves below ~1 cm reliably
LAT_MAX = 0.08
STAGING_MARGIN = 0.10  # teach-pose: turning must also be clear this far off the staging spot
LAT_PLAN_MAX = 1.50   # planned sideways shift from staging to the dock line (staging may be offset to turn)


def load_st():
    try:
        return json.load(open(ST_FILE))
    except Exception:
        return {}


def end_y(m, side, target=None):
    # end of a contiguous face segment on the given side; with a target, the segment end nearest to it
    if target is None or not m.get('runs'):
        ys = [e[1] for e in m['ends']]
        return max(ys) if side == 'left' else min(ys)
    cands = [max(r) if side == 'left' else min(r) for r in m['runs']]
    return min(cands, key=lambda y: abs(y - target))


def align_heading(m, dry=False):
    """Turn until the face normal is straight ahead (plus the station's yaw_offset fine-tune)."""
    for k in range(4):
        e = m['err'] + YAW_OFF[0]                 # yaw_offset > 0: stop turned that much to the left of square
        if abs(e) <= ANG_TOL:
            break
        if abs(e) > math.radians(25):          # > 12 deg is checked over a full circle by safe_rotate
            raise SystemExit('ABORT: heading error too large (%.1f deg)' % math.degrees(e))
        log('rotate %.2f deg' % math.degrees(e))
        if dry:                                 # same decision as safe_rotate, without moving
            p = cloud()
            if abs(e) <= math.radians(12) and not RM.turn_hits(p, CFG, e):
                log('  (dry) turn of %.2f deg is clear' % math.degrees(e))
            else:
                mv = RM.leave_move(p, CFG, 0.30)
                if mv is None:
                    need_clear(RM.rotation_hits(p, CFG), 'rotation (dry run)')
                log('  (dry) would first move x=%.2f y=%.2f, then turn %.2f deg' % (mv[0], mv[1], math.degrees(e)))
            break
        safe_rotate(e + math.copysign(ROT_SHORT, e)); m = measure(); log('  re-measure', pub(m))
    return m


def align_lateral(m, side, target, dry=False):
    for k in range(3):
        dy = end_y(m, side, target) - target
        log('lateral: %s end at y=%.4f, target %.4f, error %.4f' % (side, end_y(m, side, target), target, dy))
        if abs(dy) <= LAT_TOL:
            break
        if abs(dy) > LAT_PLAN_MAX:
            raise SystemExit('ABORT: lateral error %.3f m too large (check localisation / station)' % dy)
        if dry:
            break
        step = dy * LAT_GAIN if abs(dy) < 0.05 else dy          # big planned shift: plain command, then refine
        rel(0.0, 0.0, y=step); m = measure(); log('  re-measure', pub(m))
    return m


def plan_approach(m, st, gap):
    """Check the final approach to the TAUGHT pose (the arm motion is hard-coded, so the base never picks
    another spot): sideways move to the taught lateral position, straight forward to the taught gap, and -
    for a place station - the spot under the tray must be empty.  Any problem -> stop and report."""
    p = RM.remove_self(cloud(), CFG)                          # own points out BEFORE the cloud is shifted below
    dy = end_y(m, st['side'], st['end_y']) - st['end_y']          # +: robot must move left
    fwd = m['dist'] - FRONT - gap; sz = RM.surface_height(p, m['dist'])
    log('plan: sideways %.3f m, forward %.3f m, surface top %s m' % (dy, fwd, None if sz is None else round(sz, 3)))
    need_clear(RM.sweep_hits(p, CFG, 0.0, dy), 'sideways move to the taught position')
    need_clear(RM.sweep_hits(p - np.array([0.0, dy, 0.0]), CFG, fwd, 0.0), 'straight approach')
    if st.get('place'):
        if sz is None:
            raise SystemExit('ABORT: cannot see the surface the tray goes on')
        need_clear(RM.place_hits(p - np.array([0.0, dy, 0.0]), CFG, fwd, sz), 'place spot under the tray')
        need_clear(RM.place_motion_hits(p - np.array([0.0, dy, 0.0]), CFG, fwd, sz), 'place motion after docking')
    return dict(dy=round(dy, 4), fwd=round(fwd, 4), surface=sz)


def approach(gap, dry, align_only=False):
    m = measure()
    if not m:
        raise SystemExit('ABORT: no face found ahead')
    log('staging measure', pub(m))
    for k in range(4):
        if abs(m['err']) <= ANG_TOL:
            break
        if abs(m['err']) > math.radians(12):
            raise SystemExit('ABORT: heading error too large (%.1f deg)' % math.degrees(m['err']))
        log('rotate %.2f deg' % math.degrees(m['err']))
        if dry:
            break
        safe_rotate(m['err'] + math.copysign(ROT_SHORT, m['err'])); m = measure(); log('  re-measure', pub(m))
    if align_only:
        return m
    fwd = m['dist'] - FRONT - gap
    log('forward %.3f m (target gap %.3f)' % (fwd, gap))
    if fwd < 0 or fwd > MAX_FWD:
        raise SystemExit('ABORT: forward %.3f out of range' % fwd)
    p = cloud(); need_clear(RM.sweep_hits(p, CFG, fwd, 0.0), 'straight approach')
    if dry:
        return
    rel(fwd + FWD_SHORT, 0.0)
    f = measure(near=True); log('FINAL', pub(f))
    json.dump(dict(fwd=fwd, gap=gap, t=time.time()), open('/tmp/claude_dock_last.json', 'w'))


def check_localisation():
    """Scan-to-map match of the current pose (locmatch.py).  Navigating with a clearly wrong pose drives to the
    wrong place, so stop early with a clear message."""
    try:
        out = subprocess.run(['python3', os.path.join(HERE, 'locmatch.py'), 'check'], capture_output=True,
                             text=True, timeout=60).stdout.strip().splitlines()
        res = json.loads(out[-1])
    except Exception as e:
        log('localisation check skipped (%s)' % e); return
    log('localisation: pose %s, scan-to-map match %.2f' % (res.get('reported'), res.get('match', 0)))
    if res.get('match', 0) < LOC_MIN_MATCH:
        raise SystemExit('ABORT: localisation looks wrong (match %.2f < %.2f) - relocalise first '
                         '(locmatch.py global --apply, or relocate on the web page)' % (res.get('match', 0), LOC_MIN_MATCH))


def nav_task(st):
    """Submit the vendor navigation task to the station's staging point (in-process, NAV_SPEED)."""
    _PARK_OK[0] = False                         # navigation ends in parking mode
    from server.srv import CreateTaskById, CreateTaskByPose
    from server.msg import TargetInfoWithId, TargetInfoWithPose
    if 'staging_pose' in st:
        sx, sy, sh = st['staging_pose']
        t = TargetInfoWithPose(); t.target_pose.position.x = float(sx); t.target_pose.position.y = float(sy)
        t.target_pose.orientation.w = math.radians(sh)              # vendor convention: w = yaw (rad)
        t.action_type = 0; t.action_param = 0.0
        c = _client('srv', CreateTaskByPose, 'server/CreateTaskByPose'); r = CreateTaskByPose.Request()
    else:
        t = TargetInfoWithId(); t.target_id = int(st['staging']); t.action_type = 0; t.action_param = 0.0
        c = _client('srv', CreateTaskById, 'server/CreateTaskById'); r = CreateTaskById.Request(); r.navigation_mode = 0
    r.max_speed = NAV_SPEED; r.max_loop_times = 1; r.target_list = [t]
    if not c.wait_for_service(timeout_sec=5):
        raise SystemExit('ABORT: navigation service not available')
    f = c.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=10); res = f.result()
    if res is None:
        raise SystemExit('ABORT: navigation service did not answer')
    return 'task %d %d %s (speed %.2f m/s)' % (res.task_id, res.error_code, res.error_msg, NAV_SPEED)


_POSE = [None]
_POSE_SUB = []


def map_pose():
    """Latest map pose (x, y, yaw) from a persistent subscription."""
    if not _POSE_SUB:
        from slam.msg import LocationData
        _POSE_SUB.append(n.create_subscription(LocationData, 'slam/LocationData',
                                               lambda m: _POSE.__setitem__(0, m), qos_profile_sensor_data))
    t = time.time()
    while _POSE[0] is None and time.time() - t < 3:
        rclpy.spin_once(n, timeout_sec=0.05)
    rclpy.spin_once(n, timeout_sec=0.0)
    p = _POSE[0].pose
    return p.position.x, p.position.y, p.orientation.z


def turn_to(h):
    for _ in range(3):
        x, y, yaw = map_pose(); d = (h - yaw + math.pi) % (2 * math.pi) - math.pi
        log('turn to %.1f deg: at (%.3f, %.3f) yaw %.1f, delta %.1f' % (math.degrees(h), x, y, math.degrees(yaw), math.degrees(d)))
        if abs(d) < math.radians(3):
            return
        safe_rotate(d + math.copysign(ROT_SHORT, d))


def cancel_all_tasks():
    from server.srv import CancelAllTask
    c = n.create_client(CancelAllTask, 'server/CancelAllTask')
    if c.wait_for_service(timeout_sec=3):
        f = c.call_async(CancelAllTask.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=3)


def wait_task(tmo=150, near=None, tol=STAGING_TOL):
    """Wait until the navigation task is gone.  The vendor server keeps a station task open when the robot
    stops a few cm off the node ("Current Pos Without Node"); so if we are within `tol` of the staging point
    and have not moved for 1.5 s, treat it as arrived and cancel the dangling task ourselves."""
    import urllib.request
    t0 = time.time(); spin(0.5); last = None; still = time.time()
    while time.time() - t0 < tmo:
        q = urllib.request.Request('http://127.0.0.1:8080/api/Task/GetAllTask', data=b'{}',
                                   headers={'Content-Type': 'application/json'})
        if not json.load(urllib.request.urlopen(q, timeout=5))['Content']:
            return True
        if near:
            x, y, yaw = map_pose()
            if last is None or math.hypot(x - last[0], y - last[1]) > 0.005 or abs(yaw - last[2]) > 0.005:
                still = time.time(); last = (x, y, yaw)
            if math.hypot(x - near[0], y - near[1]) < tol and time.time() - still > 1.5:
                log('arrived %.3f m from staging but the server keeps the task open: cancelling it' % math.hypot(x - near[0], y - near[1]))
                cancel_all_tasks(); return True
        spin(0.5)
    return False


cmd = sys.argv[1]; args = sys.argv[2:]; dry = '--dry' in args; args = [a for a in args if a != '--dry']
if '--wide' in args:
    YLIM[0] = 2.5; args.remove('--wide')
if '--z' in args:
    i = args.index('--z'); ZB[0], ZB[1] = float(args[i + 1]), float(args[i + 2]); args = args[:i] + args[i + 3:]
args = [a for a in args if not a.startswith('--')]          # flags (--leave, --stage-only, --save) are read from sys.argv
spin(1.0)
if cmd == 'measure':
    nr = args[:1] == ['near']
    for b in ([0.05, 0.25], [0.25, 0.45], [0.45, 0.70], [0.70, 0.90], [0.90, 1.10]):
        ZB[0], ZB[1] = b; log('z %.2f-%.2f' % tuple(b), pub(measure(near=nr)))
elif cmd == 'approach':
    approach(float(args[0]), dry)
elif cmd == 'teach':
    # teach NAME STAGING_ID SIDE GAP : robot must already be at the staging point facing the face
    name, sid, side, gap = args[0], int(args[1]), args[2], float(args[3]); YLIM[0] = 2.5
    m = align_heading(measure(), dry)
    st = load_st(); old = st.get(name, {})
    st[name] = dict(old, staging=sid, side=side, gap=gap, end_y=round(end_y(m, side), 4), z=list(ZB))
    for k in ('untaught', 'lat_offset', 'yaw_offset', 'note'):       # a fresh teach replaces old fine-tunes
        st[name].pop(k, None)
    json.dump(st, open(ST_FILE, 'w'), indent=1); log('taught', name, json.dumps(st[name]))
elif cmd == 'teach-pose':
    # teach-pose NAME [BACK] [SIDE] : the robot stands EXACTLY at the final pose (driven there by hand, arms in
    # the carry pose).  NO MOTION.  Records that pose relative to the face - distance (gap), heading relative to
    # square (yaw_offset) and the lateral position of the face end (end_y) - plus a staging pose BACK m straight
    # behind it on the map, so docking = navigate there, then reproduce the recorded lidar-relative pose.
    name = args[0]; back = float(args[1]) if len(args) > 1 else 0.5; YLIM[0] = 2.5
    st = load_st(); old = st.get(name, {})
    side = args[2] if len(args) > 2 else old.get('side', 'left')
    assert side in ('left', 'right'), 'SIDE must be left or right'
    ZB[0], ZB[1] = old.get('z', ZB)
    check_localisation()                                      # the staging pose is stored in map coordinates
    m = measure(near=True)
    if not m:
        raise SystemExit('ABORT: no face found ahead (z band %.2f-%.2f m)' % tuple(ZB))
    log('face at the final pose', pub(m))
    if m['dist'] + back < FRONT + GRIP + 0.10 or m['dist'] + back > 2.1 or not 0.25 <= back <= MAX_FWD:
        raise SystemExit('ABORT: BACK %.2f m gives a staging distance of %.2f m to the face (need 0.77-2.1, BACK 0.25-%.2f)'
                         % (back, m['dist'] + back, MAX_FWD))
    ey = end_y(m, side)
    if abs(ey) > YLIM[0] - 0.15:
        log('WARNING: the %s end of the face is at y=%.2f m, near the edge of the view - consider SIDE %s'
            % (side, ey, 'right' if side == 'left' else 'left'))
    x, y, yaw = map_pose()
    p = RM.remove_self(cloud(), CFG)
    notes = []
    here = RM.max_forward(p, CFG, 0.0)
    if here is None:
        notes.append('the robot model already touches something here (%s): docking with --closest stops '
                     'a little short of this pose' % json.dumps(RM.sweep_hits(p, CFG, 0.0, 0.0)))
    if old.get('place'):
        sz = RM.surface_height(p, m['dist'])
        h = RM.place_motion_hits(p, CFG, 0.0, sz) if sz is not None else {'what': 'surface not seen'}
        if h:
            notes.append('place motion from here would hit %s' % json.dumps(h))
    # staging: b m behind and s m to the right of the final pose, where the robot + arms + tray can turn in
    # place, the sideways move back onto the final line and the straight approach are clear.  Nearest such
    # spot (smallest sideways offset, then smallest distance) from BACK on; else BACK straight behind.
    def staging_ok(b, s):
        q = p + np.array([b, s, 0.0])                         # the cloud as seen from that staging spot
        f = RM.max_forward(p + np.array([b, 0.0, 0.0]), CFG, b)
        if RM.sweep_hits(q, CFG, 0.0, s) or f is None or f < b - 0.005:
            return False
        # turning must stay clear if navigation stops up to STAGING_MARGIN off the spot
        return not any(RM.rotation_hits(p + np.array([b + db, s + ds, 0.0]), CFG)
                       for db, ds in ((0, 0), (STAGING_MARGIN, 0), (-STAGING_MARGIN, 0), (0, STAGING_MARGIN), (0, -STAGING_MARGIN)))
    bs = [b for b in np.arange(back, MAX_FWD + 0.001, 0.05) if m['dist'] + b <= 2.1]
    ss = sorted(np.arange(-LAT_PLAN_MAX + 0.1, LAT_PLAN_MAX - 0.1 + 0.001, 0.05), key=abs)
    pick = next(((float(b), float(s)) for s in ss for b in bs if staging_ok(b, s)), None)
    st[name] = dict(old, side=side, gap=round(m['gap'], 4), end_y=round(ey, 4),
                    yaw_offset=round(-m['ang_deg'], 2), z=list(ZB),
                    pose=[round(x, 3), round(y, 3), round(math.degrees(yaw), 2)], taught=time.strftime('%Y-%m-%d %H:%M:%S'))
    for k in ('untaught', 'lat_offset', 'note', 'staging_pose', 'back', 'staging_right'):  # fresh teach
        st[name].pop(k, None)
    if pick is not None:
        # a map pose: the vendor navigation arrives there with an arbitrary heading, the robot turns in place
        b, s = pick
        log('staging spot: %.2f m back, %.2f m to the %s of the final pose (turning clear there)'
            % (b, abs(s), 'right' if s >= 0 else 'left'))
        st[name].update(staging_pose=[round(x - b * math.cos(yaw) + s * math.sin(yaw), 3),
                                      round(y - b * math.sin(yaw) - s * math.cos(yaw), 3), round(math.degrees(yaw), 2)],
                        back=round(b, 3), staging_right=round(s, 3))
    elif 'staging' in old:
        # no room to turn in place near the station: keep the vendor station, whose navigation arrives already
        # facing the right way (only small alignment turns there)
        notes.append('no spot within %.1f m back / %.1f m sideways where the arms can turn in place (%s): keeping '
                     'vendor station %s as staging (it arrives facing the face)'
                     % (MAX_FWD, LAT_PLAN_MAX - 0.1, json.dumps(RM.rotation_hits(p + np.array([back, 0, 0]), CFG)), old['staging']))
    else:
        raise SystemExit('ABORT: no staging spot where the arms can turn in place and no vendor station for %s - '
                         'record a station on the web page first' % name)
    json.dump(st, open(ST_FILE, 'w'), indent=1)
    for t in notes:
        log('NOTE:', t)
    log('taught', name, json.dumps(st[name]))
elif cmd in ('dock', 'redock'):
    # dock NAME [GAP]: navigate to staging (unless redock), align heading, lateral, heading, straight in
    name = args[0]; st = load_st()[name]; gap = float(args[1]) if len(args) > 1 else st['gap']; YLIM[0] = 2.5
    ZB[0], ZB[1] = st['z']
    # fine-tune: lat_offset (m, + = robot further left) shifts the lateral target; yaw_offset (deg, + = left)
    st['end_y'] = st['end_y'] - float(st.get('lat_offset', 0.0)); YAW_OFF[0] = math.radians(float(st.get('yaw_offset', 0.0)))
    if st.get('lat_offset') or st.get('yaw_offset'):
        log('fine-tune: lat_offset %.3f m, yaw_offset %.2f deg' % (st.get('lat_offset', 0.0), st.get('yaw_offset', 0.0)))
    if cmd == 'dock' and dry:
        p0 = cloud()
        log('DRY RUN (no motion): turn at start:', json.dumps(RM.rotation_hits(p0, CFG)) or 'clear',
            '| move needed before turning (x, y):', RM.leave_move(p0, CFG, 0.6),
            '| would navigate to', st.get('staging_pose') or st.get('staging'), '- skipped; checking from the current pose')
    elif cmd == 'dock':
        # the vendor navigation turns in place at the start and only sees obstacles below 0.8 m: first make
        # sure the robot + arms + tray can turn here.  With --leave (e.g. starting at the table edge with the
        # tray over the table) reverse straight out until turning is clear; otherwise stop.
        if not S[0].is_hard_auto:
            raise SystemExit('ABORT: hard switch is on MANUAL - switch it to AUTO first')
        check_localisation()
        if RM.rotation_hits(cloud(), CFG):
            if '--leave' not in sys.argv:
                need_clear(RM.rotation_hits(cloud(), CFG), 'turning at the start position (navigation turns in place '
                           'first; use --leave to move out first)')
            move_to_turn_spot(0.6)
        if not S[0].is_soft_auto:
            setmode(0)
        out = nav_task(st)                                      # in-process service call (no extra process)
        log(out)
        if ' 0 Succ' not in out:
            raise SystemExit('ABORT: task not created')
        sxy = st['staging_pose'][:2] if 'staging_pose' in st else st.get('staging_xy')
        if not wait_task(near=sxy):
            raise SystemExit('ABORT: staging not reached in time')
        spin(0.3)
        # the task list also empties when the task is cancelled or fails: check we really are at the staging point
        if sxy:
            x, y, _ = map_pose(); dd = math.hypot(x - sxy[0], y - sxy[1])
            if dd > STAGING_TOL + 0.05:
                raise SystemExit('ABORT: navigation ended %.2f m from the staging point (task cancelled or failed?)' % dd)
        if 'staging_pose' in st:   # pose tasks keep the start heading: turn to the face here (staging is clear all round)
            turn_to(math.radians(st['staging_pose'][2]))
    m = measure()
    if not m:
        raise SystemExit('ABORT: no face found ahead')
    log('staging measure', pub(m))
    m = align_heading(m, dry)
    if '--stage-only' in sys.argv:                             # e.g. place pose not taught yet: stop at staging
        log('STAGE-ONLY: stopped at staging, facing the face', pub(m))
        if not dry:
            setmode(0)
        raise SystemExit(0)
    if '--here' in sys.argv:
        # untaught station (demo): no lateral target - drive straight in from the current lateral position,
        # up to the requested gap or, with --closest, as far as the robot + arms + tray stay clear
        fwd = m['dist'] - FRONT - gap
        pw = RM.remove_self(cloud(), CFG); sz = RM.surface_height(pw, m['dist'])
        place = bool(st.get('place'))
        # largest forward distance for which the straight approach AND (place stations) the place motion run
        # afterwards are clear - computed directly, not by stepping
        ok = RM.max_forward(pw, CFG, max(fwd, 0.0))
        if ok is not None and place:
            ok = RM.max_place_forward(pw, CFG, ok, sz)
        reached = ok is not None and ok >= fwd - 0.005
        if not reached:
            why = RM.sweep_hits(pw, CFG, fwd, 0.0) or (RM.place_motion_hits(pw, CFG, fwd, sz) if place else None)
            if '--closest' not in sys.argv:
                need_clear(why or {'what': 'approach/place motion'}, 'straight approach / place motion')
            if ok is None:
                raise SystemExit('ABORT: no position on this line where the approach and the place motion are '
                                 'clear: %s' % json.dumps(why))
            ok = max(0.0, ok - CLOSEST_SLACK)
            log('requested gap %.3f is blocked by %s -> stopping early, forward %.3f m (gap %.3f)'
                % (gap, json.dumps(why), ok, m['dist'] - FRONT - ok))
            fwd = ok
        log('forward %.3f m from the current lateral position (surface top %s m)' % (fwd, None if sz is None else round(sz, 3)))
        if fwd > MAX_FWD:
            raise SystemExit('ABORT: forward %.3f too long' % fwd)
        if not dry and fwd > 0.005:
            rel(fwd + FWD_SHORT, 0.0)
        if not dry:
            f = measure(near=True)
            log('FINAL', json.dumps(dict(name=name, mode='here', gap_target=gap, gap=f['gap'] if f else None,
                                          ang_deg=f['ang_deg'] if f else None, fwd=round(fwd, 4),
                                          reached_gap=reached, place_motion_clear=place)))
            json.dump(dict(fwd=fwd + FWD_SHORT, gap=gap, t=time.time()), open('/tmp/claude_dock_last.json', 'w'))
        if not dry:
            setmode(0)
        raise SystemExit(0)
    if '--closest' in sys.argv:
        # arm motion not fixed yet: go to the requested gap, or as close as the robot + arms + tray stay clear
        pw = RM.remove_self(cloud(), CFG); dy = end_y(m, st['side'], st['end_y']) - st['end_y']
        need_clear(RM.sweep_hits(pw, CFG, 0.0, dy), 'sideways move to the taught position')
        f0 = m['dist'] - FRONT - gap
        ok = RM.max_forward(pw - np.array([0.0, dy, 0.0]), CFG, max(f0, 0.0))
        if ok is None:
            raise SystemExit('ABORT: already touching something ahead after the sideways move')
        if ok < f0 - 0.005:
            ok = max(0.0, ok - CLOSEST_SLACK)      # never plan exactly at the limit: the sideways move shifts things
            log('requested gap %.3f is blocked -> will stop at gap %.3f (closest clear)' % (gap, m['dist'] - FRONT - ok))
            gap = m['dist'] - FRONT - ok
    plan_approach(m, st, gap)                                  # stops here if the taught approach is blocked
    m = align_lateral(m, st['side'], st['end_y'], dry)
    m = align_heading(m, dry)
    fwd = m['dist'] - FRONT - gap
    log('forward %.3f m (target gap %.3f)' % (fwd, gap))
    if fwd < 0 or fwd > MAX_FWD:
        raise SystemExit('ABORT: forward %.3f out of range' % fwd)
    if '--closest' in sys.argv:
        # re-evaluate with the fresh cloud after the sideways/heading corrections; back off if needed
        ok = RM.max_forward(RM.remove_self(cloud(), CFG), CFG, max(fwd, 0.0))
        if ok is None:
            raise SystemExit('ABORT: already touching something ahead after aligning')
        if ok < fwd - 0.002:
            ok = max(0.0, ok - CLOSEST_SLACK)
            log('after aligning the requested gap is blocked -> forward %.3f m instead of %.3f (gap %.3f)'
                % (ok, fwd, m['dist'] - FRONT - ok))
            fwd = ok; gap = m['dist'] - FRONT - ok
    plan_approach(m, st, gap)                                  # re-check right before driving in
    if not dry:
        rel(fwd + FWD_SHORT, 0.0)
        YLIM[0] = 2.5; f = measure(near=True)
        res = dict(name=name, gap_target=gap, gap=f['gap'], ang_deg=f['ang_deg'], lat_err=round(end_y(f, st['side'], st['end_y']) - st['end_y'], 4), fwd=round(fwd, 4))
        log('FINAL', json.dumps(res))
        json.dump(dict(fwd=fwd + FWD_SHORT, gap=gap, t=time.time()), open('/tmp/claude_dock_last.json', 'w'))
        new = not __import__('os').path.exists(LOG_FILE)
        with open(LOG_FILE, 'a') as fh:
            if new:
                fh.write('time,name,gap_target,gap,ang_deg,lat_err,fwd' + chr(10))
            fh.write('%s,%s,%.3f,%.4f,%.2f,%.4f,%.4f' %  (time.strftime('%H:%M:%S'), name, gap, f['gap'], f['ang_deg'], res['lat_err'], fwd) + chr(10))
elif cmd == 'undock':
    # undock [MIN] : reverse straight out along the last docking approach (at least MIN m, default 0.30)
    d = json.load(open('/tmp/claude_dock_last.json'))['fwd']; assert 0 <= d <= MAX_FWD + 0.05
    d = max(d, float(args[0]) if args else 0.30)
    pb = cloud(); pb = pb[pb[:, 0] < -0.30]                  # only what is behind: the arms may be in any pose
    need_clear(RM.sweep_hits(pb, CFG, -d, 0.0), 'reversing out')
    rel(-d, 0.0, rear_check=True); setmode(0)
    log('FINAL', json.dumps(dict(name='undock', back=round(d, 4))))
elif cmd == 'over':
    # over [XMAX]: list everything above 0.75 m in front (body frame) in 10 cm cells
    xm = float(args[0]) if args else 2.0; a = cloud()
    a = a[(a[:, 0] > FRONT + GRIP + 0.03) & (a[:, 0] < xm) & (np.abs(a[:, 1]) < 1.5) & (a[:, 2] > 0.75)]
    k, cnt = np.unique(np.floor(a[:, :2] / 0.1).astype(int), axis=0, return_counts=True)
    for (i, j), c in sorted(zip(map(tuple, k), cnt)):
        z = a[(np.floor(a[:, 0] / 0.1) == i) & (np.floor(a[:, 1] / 0.1) == j), 2]
        if c >= 5:
            log('x %.1f y %+.1f  n %4d  z %.2f..%.2f' % (i / 10, j / 10, c, z.min(), z.max()))
elif cmd == 'check':
    # check NAME [GAP] : at the current pose, report what a docking from here would do. NO MOTION.
    name = args[0]; st = load_st()[name]; gap = float(args[1]) if len(args) > 1 else st['gap']; YLIM[0] = 2.5
    ZB[0], ZB[1] = st['z']; p = cloud()
    log('rotation here:', json.dumps(RM.rotation_hits(p, CFG)) or 'clear', '| move needed before turning (x, y):', RM.leave_move(p, CFG, 0.6))
    m = measure()
    if not m:
        raise SystemExit('no face ahead')
    log('face', pub(m)); plan = plan_approach(m, st, gap); log('plan', json.dumps(plan))
elif cmd == 'selfscan':
    # selfscan [--save] : measure the arms with the lidar (needs nothing else within ~1 m in front). NO MOTION.
    r = RM.selfscan(cloud()); log(json.dumps(r), '| tray box in model:', RM.tray_box(CFG))
    if r and '--save' in sys.argv:
        log('saved to', RM.CFG_FILE, json.dumps(RM.save_selfscan(r)))
elif cmd == 'stop':
    # software stop: cancel relative moves and navigation tasks, switch to (soft) manual. The red e-stop
    # button is still the primary stop.
    from action_msgs.srv import CancelGoal
    from server.srv import CancelAllTask
    c = n.create_client(CancelGoal, 'move/MoveRelativeCmd/_action/cancel_goal')
    if c.wait_for_service(timeout_sec=3):
        f = c.call_async(CancelGoal.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=3)   # empty = cancel all
        log('relative moves cancelled:', f.result().return_code if f.result() else 'no reply')
    c = n.create_client(CancelAllTask, 'server/CancelAllTask')
    if c.wait_for_service(timeout_sec=3):
        f = c.call_async(CancelAllTask.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=3)
        log('navigation tasks cancelled:', f.result().error_msg if f.result() else 'no reply')
    setmode(1); log('soft manual:', not S[0].is_soft_auto)
elif cmd == 'side':
    dy = float(args[0]); assert abs(dy) <= 0.10
    YLIM[0] = 2.5; log('before', pub(measure())); rel(0.0, 0.0, y=dy); log('after ', pub(measure()))
elif cmd == 'align':
    approach(0.0, dry, align_only=True)
elif cmd == 'retreat':
    d = float(args[0]); assert 0 < d <= 1.0
    rel(-d, 0.0, rear_check=True); setmode(0)
elif cmd == 'run':
    st = args[0]; gap = float(args[1])
    if not S[0].is_soft_auto:
        setmode(0)
    out = subprocess.run(['python3', os.path.join(HERE, 'nav.py'), 'node', st, '--speed', '0.25'],
                         capture_output=True, text=True).stdout.strip()
    log(out)
    if ' 0 Succ' not in out:
        raise SystemExit('ABORT: task not created')
    if not wait_task():
        raise SystemExit('ABORT: staging not reached in time')
    spin(1.5); approach(gap, dry)

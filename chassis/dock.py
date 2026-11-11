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


def log(*a):
    print(*a, flush=True)


def pub(m):
    return json.dumps({k: v for k, v in m.items() if k not in ('err', 'runs')}) if m else 'no face'


def cloud():
    C.clear(); spin(0.3); C.clear(); t = time.time()
    while len(C) < 8 and time.time() - t < 4:
        rclpy.spin_once(n, timeout_sec=0.05)
    a = np.concatenate([np.stack([s['x'], s['y'], s['z']], 1).astype(float)
                        for s in (pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) for m in C)])
    return a[np.isfinite(a).all(1)]


def need_clear(h, what):
    if h:
        raise SystemExit('ABORT: %s blocked by obstacle %s' % (what, json.dumps(h)))


def move_to_turn_spot(max_d=0.30):
    """If turning here would hit something (chassis, torso, arms or tray), make the shortest collision-free
    straight move (back / sideways / diagonal-back) after which turning is clear.  Re-checks after moving."""
    for _ in range(3):
        p = cloud(); mv = RM.leave_move(p, CFG, max_d)
        if mv is None:
            raise SystemExit('ABORT: no clear spot to turn within %.2f m: %s' % (max_d, json.dumps(RM.rotation_hits(p, CFG))))
        if mv == (0.0, 0.0):
            return
        log('turning here would hit %s: move straight x=%.2f y=%.2f first' % (json.dumps(RM.rotation_hits(p, CFG)), mv[0], mv[1]))
        rel(mv[0], 0.0, rear_check=mv[0] < 0, y=mv[1])     # never more than the checked move; loop re-checks
    need_clear(RM.rotation_hits(cloud(), CFG), 'turning (still blocked after moving)')


def safe_rotate(yaw):
    move_to_turn_spot()
    rel(0.0, yaw)


def measure(near=False):
    C.clear(); spin(0.3); C.clear(); t = time.time()
    while len(C) < 8 and time.time() - t < 4:
        rclpy.spin_once(n, timeout_sec=0.05)
    a = np.concatenate([np.stack([s['x'], s['y'], s['z']], 1).astype(float)
                        for s in (pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) for m in C)])
    a = a[np.isfinite(a).all(1)]
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
    for _ in range(600):
        i, j = rng.choice(len(xy), 2, replace=False); d = xy[j] - xy[i]; L = np.linalg.norm(d)
        if L < 0.15:
            continue
        d /= L
        if abs(d[1]) < 0.85:      # want a face roughly across the heading
            continue
        nr = np.array([-d[1], d[0]]); inl = np.abs((xy - xy[i]) @ nr) < 0.012
        sc = inl.sum() - 50 * abs(xy[i] @ nr)   # prefer the nearest face
        if best[0] is None or sc > best[0]:
            best = (sc, inl)
    if best[1] is None:
        return None
    pts = xy[best[1]]
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


def setmode(v):
    c = n.create_client(SetControlMode, 'move/SetControlMode'); c.wait_for_service(timeout_sec=5)
    r = SetControlMode.Request(); r.mode = v; f = c.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5); spin(0.8)


def unpark():
    c = n.create_client(SetParkingMode, 'move/SetParkingMode'); c.wait_for_service(timeout_sec=5)
    r = SetParkingMode.Request(); r.enable = False; f = c.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5); spin(0.5)


def rel(x=0.0, yaw=0.0, rear_check=False, y=0.0):
    if S[0].is_soft_auto:
        setmode(1)
    unpark()
    ac = ActionClient(n, MoveRelative, 'move/MoveRelativeCmd')
    if not ac.wait_for_server(timeout_sec=5):
        raise SystemExit('ABORT: no MoveRelative server')
    g = MoveRelative.Goal()
    g.relative_pose.position.x = float(x); g.relative_pose.position.y = float(y)
    g.relative_pose.orientation.w = float(yaw)        # vendor convention: w = yaw (rad)
    gf = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, gf, timeout_sec=5); gh = gf.result()
    if not gh or not gh.accepted:
        raise SystemExit('ABORT: relative move rejected')
    rf = gh.get_result_async(); t0 = time.time()
    while not rf.done() and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05)
        if rear_check and U[0] and min(U[0].get(5, 9), U[0].get(6, 9)) < 0.25:
            log('SAFETY: rear < 0.25 m, cancel'); cf = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(n, cf, timeout_sec=3); raise SystemExit(1)
    r = rf.result().result if rf.done() else None; spin(1.0)
    log('  rel move x=%.3f y=%.3f yaw=%.2fdeg -> %s' % (x, y, math.degrees(yaw), (r.error_code, r.error_msg) if r else 'TIMEOUT'))
    if not r or r.error_code != 0:
        raise SystemExit('ABORT: relative move failed')


ST_FILE = os.path.join(HERE, 'stations.json')
LOG_FILE = os.path.join(HERE, 'dock_log.csv')
LAT_GAIN = 1.3          # sideways moves achieve ~75 % of the command
LAT_TOL = 0.003
LAT_MAX = 0.08
LAT_PLAN_MAX = 0.60   # planned sideways shift from staging to the dock line


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
        if abs(e) > math.radians(12):
            raise SystemExit('ABORT: heading error too large (%.1f deg)' % math.degrees(e))
        log('rotate %.2f deg' % math.degrees(e))
        if dry:
            need_clear(RM.rotation_hits(cloud(), CFG, 'carry'), 'rotation (dry run)')
            break
        safe_rotate(e + math.copysign(ROT_SHORT, e)); m = measure(); log('  re-measure', pub(m))
    return m


def align_lateral(m, side, target, dry=False):
    for k in range(5):
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


def map_pose():
    from slam.msg import LocationData
    P = [None]; sub = n.create_subscription(LocationData, 'slam/LocationData', lambda m: P.__setitem__(0, m), qos_profile_sensor_data)
    t = time.time()
    while P[0] is None and time.time() - t < 3:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_subscription(sub); p = P[0].pose
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


def wait_task(tmo=150, near=None, tol=0.20):
    """Wait until the navigation task is gone.  The vendor server keeps a station task open when the robot
    stops a few cm off the node ("Current Pos Without Node"); so if we are within `tol` of the staging point
    and have not moved for 3 s, treat it as arrived and cancel the dangling task ourselves."""
    import urllib.request
    t0 = time.time(); time.sleep(2); last = None; still = time.time()
    while time.time() - t0 < tmo:
        q = urllib.request.Request('http://127.0.0.1:8080/api/Task/GetAllTask', data=b'{}',
                                   headers={'Content-Type': 'application/json'})
        if not json.load(urllib.request.urlopen(q, timeout=5))['Content']:
            return True
        if near:
            x, y, yaw = map_pose()
            if last is None or math.hypot(x - last[0], y - last[1]) > 0.005 or abs(yaw - last[2]) > 0.005:
                still = time.time(); last = (x, y, yaw)
            if math.hypot(x - near[0], y - near[1]) < tol and time.time() - still > 3.0:
                log('arrived %.3f m from staging but the server keeps the task open: cancelling it' % math.hypot(x - near[0], y - near[1]))
                cancel_all_tasks(); return True
        time.sleep(1)
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
        if RM.rotation_hits(cloud(), CFG):
            if '--leave' not in sys.argv:
                need_clear(RM.rotation_hits(cloud(), CFG), 'turning at the start position (navigation turns in place '
                           'first; use --leave to move out first)')
            move_to_turn_spot(0.6)
        if not S[0].is_soft_auto:
            setmode(0)
        if 'staging_pose' in st:
            sx, sy, sh = st['staging_pose']
            out = subprocess.run(['python3', os.path.join(HERE, 'nav.py'), 'goto', str(sx), str(sy), str(math.radians(sh)), '--speed', '0.25'],
                                 capture_output=True, text=True).stdout.strip()
        else:
            out = subprocess.run(['python3', os.path.join(HERE, 'nav.py'), 'node', str(st['staging']), '--speed', '0.25'],
                                 capture_output=True, text=True).stdout.strip()
        log(out)
        if ' 0 Succ' not in out:
            raise SystemExit('ABORT: task not created')
        sxy = st.get('staging_xy') or (st['staging_pose'][:2] if 'staging_pose' in st else None)
        if not wait_task(near=sxy):
            raise SystemExit('ABORT: staging not reached in time')
        spin(1.5)
        # the task list also empties when the task is cancelled or fails: check we really are at the staging point
        if sxy:
            x, y, _ = map_pose(); dd = math.hypot(x - sxy[0], y - sxy[1])
            if dd > 0.25:
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
        pw = RM.remove_self(cloud(), CFG); blocked = RM.sweep_hits(pw, CFG, fwd, 0.0)
        if blocked:
            if '--closest' not in sys.argv:
                need_clear(blocked, 'straight approach')
            ok = 0.0
            for f_ in np.arange(0.01, fwd + 1e-9, 0.01):
                if RM.sweep_hits(pw, CFG, float(f_), 0.0):
                    break
                ok = float(f_)
            log('requested gap %.3f is blocked by %s -> stopping early, forward %.3f m (gap %.3f)'
                % (gap, json.dumps(blocked), ok, m['dist'] - FRONT - ok))
            fwd = ok
        log('forward %.3f m from the current lateral position' % fwd)
        if fwd > MAX_FWD:
            raise SystemExit('ABORT: forward %.3f too long' % fwd)
        sz = RM.surface_height(pw, m['dist'])
        if st.get('place') and sz is not None:
            log('place spot check at this pose:', json.dumps(RM.place_hits(pw, CFG, fwd, sz)) or 'clear (surface %.2f m)' % sz)
        if not dry and fwd > 0.005:
            rel(fwd + FWD_SHORT, 0.0)
            f = measure(near=True)
            log('FINAL', json.dumps(dict(name=name, mode='here', gap_target=gap, gap=f['gap'] if f else None,
                                          ang_deg=f['ang_deg'] if f else None, fwd=round(fwd, 4))))
            json.dump(dict(fwd=fwd + FWD_SHORT, gap=gap, t=time.time()), open('/tmp/claude_dock_last.json', 'w'))
        if not dry:
            setmode(0)
        raise SystemExit(0)
    if '--closest' in sys.argv:
        # arm motion not fixed yet: go to the requested gap, or as close as the robot + arms + tray stay clear
        pw = RM.remove_self(cloud(), CFG); dy = end_y(m, st['side'], st['end_y']) - st['end_y']
        need_clear(RM.sweep_hits(pw, CFG, 0.0, dy), 'sideways move to the taught position')
        f0 = m['dist'] - FRONT - gap; ok = f0
        if RM.sweep_hits(pw - np.array([0.0, dy, 0.0]), CFG, f0, 0.0):
            ok = 0.0
            for f_ in np.arange(0.01, f0 + 1e-9, 0.01):
                if RM.sweep_hits(pw - np.array([0.0, dy, 0.0]), CFG, float(f_), 0.0):
                    break
                ok = float(f_)
            log('requested gap %.3f is blocked -> will stop at gap %.3f (closest clear)' % (gap, m['dist'] - FRONT - ok))
            gap = m['dist'] - FRONT - ok
    plan_approach(m, st, gap)                                  # stops here if the taught approach is blocked
    m = align_lateral(m, st['side'], st['end_y'], dry)
    m = align_heading(m, dry)
    fwd = m['dist'] - FRONT - gap
    log('forward %.3f m (target gap %.3f)' % (fwd, gap))
    if fwd < 0 or fwd > MAX_FWD:
        raise SystemExit('ABORT: forward %.3f out of range' % fwd)
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
    d = json.load(open('/tmp/claude_dock_last.json'))['fwd']; assert 0 < d <= MAX_FWD + 0.05
    rel(-d, 0.0, rear_check=True); setmode(0)
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

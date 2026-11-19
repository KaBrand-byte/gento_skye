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
import route as RT
CFG = RM.load()
WP_FILE = os.path.join(HERE, RT.WAYPOINT_FILE)
WAYPOINT_TOL = 0.35     # m: a waypoint only shapes the route - close enough is reached
WAYPOINT_TMO = 90       # s: not reached by then -> cancel and drive straight to the destination
VIA = ['random']        # --via none|random|NAME: goto picks W1 / W2 / ... / direct at random (default)
JITTER = [0.0]          # --jitter R: navigate to a random spot within R m of the taught safe point

FRONT = 0.338          # chassis centre -> front edge
GRIP = 0.33            # gripper tip beyond chassis front (worst case)
MAX_FWD = 1.20          # longest straight approach (checked against the live lidar the whole way)
ANG_TOL = math.radians(1.0)       # turns below ~1 deg are not executed at all (dead band); the residual heading
                                  # is taken into account by lat_error (drive drift)
YAW_OFF = [0.0]                  # station fine-tune (rad), set from stations.json yaw_offset
ROT_SHORT = math.radians(0.77)   # measured rotation shortfall of MoveRelative
FWD_SHORT = 0.011                # measured: straight moves stop ~11 mm short
FWD_SHORT_LAST = 0.005           # measured on the ~0.3 m last stage at D: 11 mm overshot by 3-8 mm
YLIM = [0.9]
REAR_MIN = float(os.environ.get('GENTO_REAR_MIN', '0.10'))   # m: rear ultrasonic stop while reversing
                                 # (was 0.25; T has fixed structure ~16 cm behind the chassis)
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


# the vendor programs (linkhou_robot.service: driver_node, localisation, ...) publish the lidar, the chassis state and
# the pose; 2026-11-16 and 11-18 the service was not running after a boot (Restart=no) and every check failed
VENDOR_DOWN = ('no %s - the vendor chassis programs are not running or not publishing (linkhou_robot.service): '
               'run "gento doctor"; power-cycle the chassis (or: sudo systemctl restart linkhou_robot)')


def need_state(timeout=5.0):
    """The chassis state (move/State); a clear ABORT instead of a traceback when the vendor programs are down."""
    t = time.time()
    while S[0] is None and time.time() - t < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    if S[0] is None:
        raise SystemExit('ABORT: %s' % VENDOR_DOWN % ('chassis state (move/State), %.0f s' % timeout))
    return S[0]


def spin(t):
    e = time.time() + t
    while time.time() < e:
        rclpy.spin_once(n, timeout_sec=0.05)


T0 = time.time()
FRAMES = 6                     # lidar frames per snapshot (10 Hz)
NAV_SPEED = float(os.environ.get('GENTO_NAV_SPEED', '0.35'))   # vendor max_speed.linear is 0.35 m/s
MOVES = [0]                    # incremented by every relative move: invalidates the cloud cache
_CACHE = {'moves': -1, 't': 0.0, 'a': None}
PLAN_LIMIT_MS = 1200           # requirement: data input -> planning done (command issued) below this
INPUT_T = [None]               # when the newest sensor input used for planning became available
CMD_INPUT_T = [None]           # input time of the command being executed (for the motion-start time)
_FROZEN = [None]               # bench: plan on one fixed snapshot
MOTION_JOB = [False]           # dock / undock (not dry): switch mode + release parking BEFORE capturing input
LAT_FILE = os.path.join(HERE, 'latency_log.csv')
# Timing events in the log (parsed by latency_report.py; every line carries the wall-clock time to the ms):
#   LIDAR scan start | INPUT ready: ... | PLAN start: ... | PLAN done: ... | input -> plan N ms
#   COMMAND accepted: ... | input -> command N ms | MOTION started: ... | input -> motion N ms


def stamp(t=None):
    t = time.time() if t is None else t
    return time.strftime('%H:%M:%S', time.localtime(t)) + '.%03d' % int((t % 1.0) * 1000)


RESP_FILE = os.path.join(HERE, 'response_log.csv')   # append-only: one line per job
FIRST = {'scan': None, 'last_scan': None, 'input': None, 'done': False}   # response time of this job


def full_stamp(t):
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)) + '.%03d' % int((t % 1.0) * 1000)


def input_ready(what):
    INPUT_T[0] = time.time(); log('INPUT ready: %s' % what)
    if what.startswith('lidar snapshot') and FIRST['input'] is None:          # first point cloud of this job
        FIRST['input'], FIRST['scan'] = INPUT_T[0], FIRST['last_scan']


def response_time(step):
    """Evaluation metric: the first environment input (lidar point cloud) of this job -> the first chassis motion
    command issued.  Required < PLAN_LIMIT_MS.  Logged once per job with both system timestamps (epoch and
    local time) and appended to response_log.csv (never overwritten)."""
    if FIRST['done'] or FIRST['input'] is None:
        return
    FIRST['done'] = True
    t_in, t_cmd = FIRST['input'], time.time(); ms = 1000.0 * (t_cmd - t_in)
    scan_ms = 1000.0 * (t_cmd - FIRST['scan']) if FIRST['scan'] else float('nan')
    ok = 'OK' if ms < PLAN_LIMIT_MS else 'OVER'
    log('RESPONSE: first lidar input %.3f (%s) -> first chassis command %.3f (%s) = %.0f ms (limit %d ms) %s | '
        'command: %s | from the scan start %.0f ms' % (t_in, full_stamp(t_in), t_cmd, full_stamp(t_cmd), ms,
                                                       PLAN_LIMIT_MS, ok, step, scan_ms))
    try:
        new = not os.path.exists(RESP_FILE)
        with open(RESP_FILE, 'a') as fh:
            if new:
                fh.write('job,first_input_epoch,first_input_time,first_command_epoch,first_command_time,ms,result,command,scan_start_ms' + chr(10))
            fh.write('%s,%.3f,%s,%.3f,%s,%.0f,%s,%s,%.0f' % (' '.join(sys.argv[1:3]), t_in, full_stamp(t_in), t_cmd,
                                                            full_stamp(t_cmd), ms, ok, step.replace(',', ' '), scan_ms) + chr(10))
    except OSError:
        pass


def plan_start(what):
    if INPUT_T[0] is not None:
        log('PLAN start: %s' % what)


def _since(t0):
    return 1000.0 * (time.time() - t0)


def plan_done(step):
    """Planning of `step` finished (command about to be issued): log the time since the newest sensor input."""
    if INPUT_T[0] is None:
        CMD_INPUT_T[0] = None
        return
    ms = _since(INPUT_T[0]); CMD_INPUT_T[0] = INPUT_T[0]; INPUT_T[0] = None
    log('PLAN done: %s | input -> plan %.0f ms%s' % (step, ms, '  OVER %d ms' % PLAN_LIMIT_MS if ms > PLAN_LIMIT_MS else ''))
    try:
        new = not os.path.exists(LAT_FILE)
        with open(LAT_FILE, 'a') as fh:
            if new:
                fh.write('time,step,ms' + chr(10))
            fh.write('%s,%s,%.0f' % (time.strftime('%m-%d %H:%M:%S'), step.replace(',', ' '), ms) + chr(10))
    except OSError:
        pass


def log(*a):
    print('[%s %6.1fs]' % (stamp(), time.time() - T0), *a, flush=True)


def command_sent(step):
    response_time(step)
    if CMD_INPUT_T[0] is not None:
        log('COMMAND accepted: %s | input -> command %.0f ms' % (step, _since(CMD_INPUT_T[0])))


class MotionWatch:
    """Logs 'MOTION started' once the map pose has moved (> 1 cm or > 0.5 deg) after a command."""
    def __init__(self, step):
        self.step = step; self.t_in = CMD_INPUT_T[0]; self.p0 = _POSE[0]; self.done = self.t_in is None

    def poll(self):
        if self.done or _POSE[0] is None:
            return
        if self.p0 is None:
            self.p0 = _POSE[0]
            return
        a, b = self.p0.pose, _POSE[0].pose
        if (math.hypot(b.position.x - a.position.x, b.position.y - a.position.y) > 0.01
                or abs((b.orientation.z - a.orientation.z + math.pi) % (2 * math.pi) - math.pi) > math.radians(0.5)):
            self.done = True
            log('MOTION started: %s | input -> motion %.0f ms' % (self.step, _since(self.t_in)))


def pub(m):
    return json.dumps({k: v for k, v in m.items() if k not in ('err', 'runs')}) if m else 'no face'


def cloud(fresh=False):
    """Lidar snapshot (base_link).  Reused while the robot has not moved since and it is < 2 s old."""
    if _FROZEN[0] is not None:
        return _FROZEN[0]
    if not fresh and _CACHE['moves'] == MOVES[0] and time.time() - _CACHE['t'] < 2.0:
        return _CACHE['a']
    ready_to_move()
    log('LIDAR scan start'); FIRST['last_scan'] = time.time(); C.clear(); spin(0.15); C.clear(); t = time.time()
    while len(C) < FRAMES and time.time() - t < 4:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not C:
        raise SystemExit('ABORT: %s' % VENDOR_DOWN % 'lidar point cloud (/driver/lidar/point_cloud/Data), 4 s')
    a = np.concatenate([np.stack([s['x'], s['y'], s['z']], 1).astype(float)
                        for s in (pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) for m in list(C))])
    a = a[np.isfinite(a).all(1)]
    _CACHE.update(moves=MOVES[0], t=time.time(), a=a)
    input_ready('lidar snapshot %d points, %d frames' % (len(a), len(C)))
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


FACE_KEEP = 0.04       # m: re-measuring after a turn in place keeps the face at this distance (+-) from the centre


def measure(near=False, expect=None):
    """Fit the face ahead.  expect: a previous fit of the SAME face (the robot only turned in place since): the
    distance from the robot centre to the face line does not change with a turn, so only lines within FACE_KEEP of
    it count - otherwise a second surface nearby can win the fit (2026-11-16 at D: after a 1.9 deg turn the fit
    jumped from the conveyor at 0.75 m to a low structure at 0.84 m and the heading followed it by 8 deg)."""
    a = cloud(fresh=True)
    if near:   # close to the face: leave out only the robot's own points (measured arm shape), not a whole zone
        sel = (a[:, 0] > FRONT + 0.02) & (a[:, 0] < 1.3) & (np.abs(a[:, 1]) < YLIM[0])
        if ZB[1] > 0.75:
            sel &= ~RM.self_mask(a, CFG, 0.03)
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
        nr = np.array([-d[1], d[0]])
        if expect is not None and abs(abs(sub[i] @ nr) - expect['dist']) > FACE_KEEP:
            continue
        inl = np.abs((sub - sub[i]) @ nr) < 0.012
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
    if expect is not None and abs(dist - expect['dist']) > FACE_KEEP:
        return None
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
    # SAFETY: the vendor MoveRelative does NOT move a combined forward + sideways goal in a straight line
    # (2026-11-12: x 0.75 / y 1.00 ended turned 16 deg and 0.38 m off, and hit the table next to the belt).
    # Every collision check here assumes straight pure moves: refuse combined goals.
    if abs(x) > 0.001 and abs(y) > 0.001:
        raise SystemExit('ABORT: combined forward + sideways move (x %.3f, y %.3f) refused - split it' % (x, y))
    step = 'move x %.3f y %.3f yaw %.1f' % (x, y, math.degrees(yaw))
    plan_done(step)
    if not _POSE_SUB:
        map_pose()                              # pose subscription for the motion-start time
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
    command_sent(step); mw = MotionWatch(step)
    rf = gh.get_result_async(); t0 = time.time()
    while not rf.done() and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.05); mw.poll()
        if rear_check and U[0] and min(U[0].get(5, 9), U[0].get(6, 9)) < REAR_MIN:
            log('SAFETY: rear < %.2f m, cancel' % REAR_MIN); cf = gh.cancel_goal_async()
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
        # small turns: no shortfall compensation (measured: left turns are exact, right ones scatter +-0.8 deg)
        safe_rotate(e if abs(e) < math.radians(5) else e + math.copysign(ROT_SHORT, e))
        prev = m
        m = measure(expect=prev) or measure(near=True, expect=prev)          # the same face as before the turn
        if not m:           # safe_rotate may have had to move first: then the distance changed
            log('  the face at %.3f m not found again after the turn - taking the face ahead' % prev['dist'])
            m = measure() or measure(near=True)
            if not m:
                raise SystemExit('ABORT: no face found ahead after turning')
        log('  re-measure', pub(m))
    return m


def _face_lateral(err, dist, ye):
    """Lateral offset of a face point (robot-frame y = ye, on the face line) from the robot centre, measured
    ALONG the face: unchanged when the robot turns in place."""
    x = (dist - ye * math.sin(err)) / math.cos(err)
    return -x * math.sin(err) + ye * math.cos(err)


def expected_end(st):
    """Where the reference end should appear now (robot frame y), from the map pose relative to the station's
    final pose (good to ~10 cm): picks the right piece when the face is split into several runs."""
    if 'pose' not in st:
        return st['end_y']
    try:
        x, y, _ = map_pose()
    except Exception:
        return st['end_y']
    fx, fy, fh = st['pose']; h = math.radians(fh)
    yr = -math.sin(h) * (x - fx) + math.cos(h) * (y - fy)     # robot left (+) of the station's line
    return st['end_y'] - yr


def lat_error(m, st, gap):
    """Sideways move (m, + = left) that puts the station's reference face end at its taught lateral position at
    the final pose: measured along the face (independent of the current heading) and including the drift of
    the straight drive still to come at the current heading."""
    ye = end_y(m, st['side'], expected_end(st))
    taught = _face_lateral(-YAW_OFF[0], FRONT + st['gap'], st['end_y'])
    now = _face_lateral(m['err'], m['dist'], ye) + (m['dist'] - FRONT - gap) * math.sin(m['err'])
    return (now - taught) / math.cos(m['err']) + LAT_BIAS[0]


# Stations with a reference scan: at the safe point the face-end lateral is calibrated against the reference
# match (seeded by the map pose), so the first sideways move already goes to the right place.  Measured at D on
# the new map: the face end gave -0.04 m, the reference +0.26 m (and the reference was right) - the robot went
# back and forth.
LAT_BIAS = [0.0]


STAGE2 = 0.30          # m: the last part of a long approach is driven after a fresh measurement
SIDE_MIN = 0.025       # m: the chassis does not execute sideways moves much below ~2 cm on their own


def sideways_small(dy, st, fwd):
    """Pure sideways correction, also below the chassis' ~2 cm dead band: out by dy + 4 cm, back by 4 cm (both
    moves are big enough to be executed).  Both checked against the live cloud (incl. accepted structure)."""
    if abs(dy) < 0.008:
        return
    steps = [dy] if abs(dy) >= SIDE_MIN else [dy + math.copysign(0.04, dy), -math.copysign(0.04, dy)]
    p = accepted_filter(RM.remove_self(cloud(), CFG), st, fwd, dy)
    off = 0.0
    for s_ in steps:
        need_clear(RM.sweep_hits(p - np.array([0.0, off, 0.0]), CFG, 0.0, s_), 'sideways correction')
        off += s_
    log('sideways correction %.3f m (%s)' % (dy, ' then '.join('%.3f' % s_ for s_ in steps)))
    for s_ in steps:
        rel(0.0, 0.0, y=s_)


SLANT_TOL = 0.015      # m: lateral offsets up to this are left alone (no move); larger ones by slant_move


def _turned(p, yaw):
    """Points of the robot frame seen from the same spot after turning by yaw."""
    c, s_ = math.cos(-yaw), math.sin(-yaw); q = p.copy()
    q[:, 0] = c * p[:, 0] - s_ * p[:, 1]; q[:, 1] = s_ * p[:, 0] + c * p[:, 1]
    return q


def slant_plan(dy, dx, p, what):
    """Check (no motion) reaching the point dx ahead / dy to the side WITHOUT a sideways move: turn towards it,
    drive straight, turn back.  p: robot-frame cloud (own points and accepted structure removed).  Raises if any
    of the three moves is blocked; returns (turn, distance)."""
    a = math.atan2(dy, dx); d = math.hypot(dx, dy)
    need_clear(RM.turn_hits(p, CFG, a), what + ' (turn towards the line)')
    q = _turned(p, a)
    need_clear(RM.sweep_hits(q, CFG, d, 0.0), what + ' (straight, slanted)')
    need_clear(RM.turn_hits(q - np.array([d, 0.0, 0.0]), CFG, -a), what + ' (turn back)')
    return a, d


def slant_move(a, d, short, what):
    """Run a checked slant_plan: turn a, straight d (+ short: the chassis' stop shortfall), turn back."""
    log('%s: turn %.1f deg, straight %.3f m, turn back (no sideways move)' % (what, math.degrees(a), d))
    comp = (lambda e: e if abs(e) < math.radians(5) else e + math.copysign(ROT_SHORT, e))
    rel(0.0, comp(a)); rel(d + short, 0.0); rel(0.0, comp(-a))


FINAL_X_TOL = 0.008     # m: remaining distance to the taught pose (reference scan) corrected after the last stage
FINAL_X_MAX = 0.05      # m: larger remainders are not "a short last move" - left alone (reported in FINAL)


def final_forward_fix(name, st):
    """After the last stage: the chassis stops short by ~1-1.5 cm on some floors (D, new map: 3 runs, 0.9-1.5 cm
    left by the reference scan).  Drive the remainder; below ~2 cm the chassis ignores a move, so back off 3 cm
    first and drive 3 cm + the remainder.  Too close is left alone (no extra move towards the face).  Returns the
    net forward distance added."""
    try:
        rx, ry, ra = ref_offset(name)
    except SystemExit as e:
        log('final check: no reference match (%s) - no correction' % e)
        return 0.0
    if not FINAL_X_TOL < rx <= FINAL_X_MAX:
        log('final check: %.3f m to the taught pose - no correction' % rx)
        return 0.0
    p = accepted_filter(RM.remove_self(cloud(), CFG), st, 0.0, 0.0)
    steps = [rx] if rx >= SIDE_MIN else [-0.03, 0.03 + rx]
    off = 0.0
    for d in steps:
        if d > 0:
            need_clear(RM.sweep_hits(p - np.array([off, 0.0, 0.0]), CFG, d, 0.0), 'final forward correction')
        off += d
    log('final forward correction %.3f m (%s)' % (rx, ' then '.join('%.3f' % d for d in steps)))
    for d in steps:
        rel(d, 0.0, rear_check=d < 0)
    return rx


def measure_station(st, gap, expect_dist):
    """Fit the station's face: far and near fits can find different faces (e.g. a table leg close by); take the
    one closest to where the station's face should be."""
    cands = [c for c in (measure(near=True), measure()) if c]
    if not cands:
        raise SystemExit('ABORT: station face lost during the approach')
    return min(cands, key=lambda c: abs(lat_error(c, st, gap)) + abs(c['dist'] - expect_dist))


def align_lateral(m, st, gap, dry=False):
    side, target = st['side'], st['end_y']
    for k in range(3):
        dy = lat_error(m, st, gap)
        log('lateral: %s end at y=%.4f, target %.4f, error %.4f (along the face, incl. drive drift)' % (side, end_y(m, side, target), target, dy))
        if abs(dy) <= LAT_TOL:
            break
        if abs(dy) > LAT_PLAN_MAX:
            raise SystemExit('ABORT: lateral error %.3f m too large (check localisation / station)' % dy)
        if dry:
            break
        step = dy * LAT_GAIN if abs(dy) < 0.05 else dy          # big planned shift: plain command, then refine
        rel(0.0, 0.0, y=step); m = measure() or measure(near=True); log('  re-measure', pub(m))
    return m


ACC_VOX = 0.02     # accepted-obstacle voxels (m); a live point within one voxel of a recorded one is ignored


def _vox_hash(k):
    k = k.astype(np.int64) + 4096
    return (k[:, 0] << 26) | (k[:, 1] << 13) | k[:, 2]


SURF_BAND = (0.04, 0.02)   # m below / above the measured work-surface top that counts as the surface itself


def surface_filter(p, st):
    """Place stations: drop the work surface itself (its top, +-noise) ahead of the robot.  The carried arms run
    level with the belt top (model: lowest arm part 0.818 m, belt 0.82 m; verified on site that they clear it) and
    the arms + tray are MEANT to go over it; with the model's 3 cm margin every belt point was an obstacle, and the
    belt seen from further back is not covered by the accepted points (2026-11-16: stops at gap 0.18 / 0.32 with no
    real obstacle).  Anything standing on the surface (> top + 2 cm) still counts."""
    sz = SURF[0]
    if not st.get('place') or sz is None or not len(p):
        return p
    drop = (p[:, 0] > FRONT + 0.02) & (p[:, 2] > sz - SURF_BAND[0]) & (p[:, 2] <= sz + SURF_BAND[1])
    return p[~drop]


def accepted_filter(p, st, fwd, dy=0.0):
    """Drop the points that were recorded at this station with `accept` (static structure next to the final
    pose that the conservative arm model touches but the real arms do not - verified on site) and, at place
    stations, the work surface itself (surface_filter).  p: robot frame, the final pose is (fwd, dy) ahead."""
    p = surface_filter(p, st)
    acc = st.get('accepted')
    if not acc or not len(p):
        return p
    a = np.array(acc, dtype=np.int64); lo, hi = a.min(0) - 1, a.max(0) + 1
    k = np.floor((p - np.array([fwd, dy, 0.0])) / ACC_VOX).astype(np.int64)
    near = np.all((k >= lo) & (k <= hi), axis=1)
    if not near.any():
        return p
    known = _vox_hash(a); kn = k[near]; hit = np.zeros(len(kn), bool)
    for d in np.array(np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])).T.reshape(-1, 3):
        hit |= np.isin(_vox_hash(kn + d), known)
    drop = np.zeros(len(p), bool); drop[np.where(near)[0][hit]] = True
    return p[~drop]


def model_hits_mask(p, fwd, sz, place):
    """Points (own points already removed, robot frame) that the robot model touches on the straight approach
    of `fwd` m and - place stations - at the place spot / during the place motion."""
    ms, mg = CFG.get('margin_straight', 0.03), CFG['margin']; hit = np.zeros(len(p), bool)
    for _, b in RM.boxes(CFG):
        hit |= RM._inside(p, RM._swept(b, fwd, 0.0), ms)
    if place and sz is not None:
        q = p - np.array([fwd, 0.0, 0.0]); t = RM.tray_box(CFG); pm = CFG['place_motion']
        hit |= ((q[:, 0] > t[0] - mg) & (q[:, 0] < t[1] + mg) & (q[:, 1] > t[2] - mg) & (q[:, 1] < t[3] + mg)
                & (q[:, 2] > sz + 0.02) & (q[:, 2] < t[4]))
        for name, b in RM.boxes(CFG):
            if name in ('arms', 'tray'):
                env = [b[0], b[1] + pm['forward'], b[2] - pm['side'], b[3] + pm['side'], b[4] - pm['down'], b[5]]
                floor = max(env[4], sz + 0.02)
                hit |= ((q[:, 0] > env[0] - ms) & (q[:, 0] < env[1] + ms) & (q[:, 1] > env[2] - ms)
                        & (q[:, 1] < env[3] + ms) & (q[:, 2] > floor) & (q[:, 2] < env[5] + ms))
    return hit


SURF = [None]          # top of the work surface for this docking (see plan_approach)


def plan_approach(m, st, gap, dy=None, sideways=True):
    """Check the final approach to the TAUGHT pose (the arm motion is hard-coded, so the base never picks
    another spot): sideways move to the taught lateral position, straight forward to the taught gap, and -
    for a place station - the spot under the tray must be empty.  Any problem -> stop and report."""
    p = RM.remove_self(cloud(), CFG)                          # own points out BEFORE the cloud is shifted below
    if dy is None:
        dy = lat_error(m, st, gap)                             # +: robot must move left (face edge)
    fwd = m['dist'] - FRONT - gap
    if SURF[0] is None:
        # measured once, from the first (farthest) view: close to the edge the belt is seen at a grazing angle
        # and a lower frame layer can come out as "the surface" (0.74 instead of 0.82 m, measured)
        SURF[0] = RM.surface_height(p, m['dist'])
    sz = SURF[0]
    p = accepted_filter(p, st, fwd, dy)
    log('plan: sideways %.3f m, forward %.3f m, surface top %s m' % (dy, fwd, None if sz is None else round(sz, 3)))
    if sideways:      # stations with a reference scan get to the line by slant_move (checked there), not sideways
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


NAV_WATCH = [None]


def nav_task(st, pose=None):
    """Submit the vendor navigation task to the station's staging point - or to `pose` [x, y, yaw_deg], e.g. a
    waypoint or a jittered safe point - (in-process, NAV_SPEED)."""
    _PARK_OK[0] = False                         # navigation ends in parking mode
    from server.srv import CreateTaskById, CreateTaskByPose
    from server.msg import TargetInfoWithId, TargetInfoWithPose
    if pose is not None or 'staging_pose' in st:
        sx, sy, sh = pose if pose is not None else st['staging_pose']
        t = TargetInfoWithPose(); t.target_pose.position.x = float(sx); t.target_pose.position.y = float(sy)
        t.target_pose.orientation.w = math.radians(sh)              # vendor convention: w = yaw (rad)
        t.action_type = 0; t.action_param = 0.0
        c = _client('srv', CreateTaskByPose, 'server/CreateTaskByPose'); r = CreateTaskByPose.Request()
    else:
        t = TargetInfoWithId(); t.target_id = int(st['staging']); t.action_type = 0; t.action_param = 0.0
        c = _client('srv', CreateTaskById, 'server/CreateTaskById'); r = CreateTaskById.Request(); r.navigation_mode = 0
    r.max_speed = NAV_SPEED; r.max_loop_times = 1; r.target_list = [t]
    plan_done('navigation task')
    if not c.wait_for_service(timeout_sec=5):
        raise SystemExit('ABORT: navigation service not available')
    f = c.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=10); res = f.result()
    if res is None:
        raise SystemExit('ABORT: navigation service did not answer')
    command_sent('navigation task'); NAV_WATCH[0] = MotionWatch('navigation task (vendor path planning + start)')
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


# ---- map-positioned stations (mode "map"): no edge to dock against; the final pose comes from the map ----
MAP_TOL = 0.04                  # m: near T the map pose jumps by up to ~10 cm after small moves (measured), so
MAP_ANG = math.radians(1.5)     # corrections below this only chase localisation noise
MAP_CORRECTIONS = 1             # after the main translation
MAP_TURNS = 3                   # heading corrections, counted apart from the translations
MAP_SETTLE = 8.0                # s: wait this long for the localisation to settle at a map station (4 s once
                                # was not enough after a 144 deg turn at T's safe point, 2026-11-17)
MAP_REF_MAX = 0.30              # m: further than this from the station by the map -> no scan match, ABORT
MAP_MAX_MOVE = 2.5              # m: longest straight translation from the safe point to the station


def ready_to_move():
    """Motion jobs: soft manual mode + parking released (~1-1.5 s, wheels to zero) BEFORE the input of the next
    plan is captured, so 'input -> command / motion' only contains planning.  No-op when already ready."""
    if not MOTION_JOB[0] or S[0] is None:
        return
    if S[0].is_soft_auto or S[0].is_parking_mode or not _PARK_OK[0]:
        t = time.time()
        if S[0].is_soft_auto:
            setmode(1)
        unpark()
        log('chassis ready to move (soft manual, parking released): %.0f ms, before capturing input' % (1000 * (time.time() - t)))


def loc_state():
    """The vendor localisation's own confidence (0-100) and relocating flag of the latest pose message."""
    m = _POSE[0]
    return getattr(m, 'confidence', None), getattr(m, 'is_relocating', None)


def settled_pose(timeout=4.0):
    """Map pose once the localisation has caught up after a move (measured: it keeps correcting for ~1 s after
    the robot stops; with a fixed short wait the corrections chased stale poses by up to 10 cm).  Not settled
    while the vendor localisation says it is relocating.  Logs its confidence (2026-11-17: once, after a 144 deg
    turn at T's safe point, the pose kept moving for > 4 s and then jumped 7 cm / 3 deg - cause unknown)."""
    ready_to_move()
    spin(0.4); first = last = map_pose(); t0 = time.time()
    while time.time() - t0 < timeout:
        spin(0.4); cur = map_pose(); conf, reloc = loc_state()
        if (not reloc and math.hypot(cur[0] - last[0], cur[1] - last[1]) < 0.003
                and abs((cur[2] - last[2] + math.pi) % (2 * math.pi) - math.pi) < math.radians(0.2)):
            input_ready('pose settled (%.3f, %.3f, %.1f deg) conf %s' % (cur[0], cur[1], math.degrees(cur[2]), conf))
            return cur
        last = cur
    conf, reloc = loc_state()
    log('  localisation still moving after %.0f s - using the latest pose (moved %.3f m / %.1f deg while waiting, '
        'conf %s, relocating %s)' % (timeout, math.hypot(last[0] - first[0], last[1] - first[1]),
                                     math.degrees((last[2] - first[2] + math.pi) % (2 * math.pi) - math.pi), conf, reloc))
    input_ready('pose (still moving) (%.3f, %.3f, %.1f deg)' % (last[0], last[1], math.degrees(last[2])))
    return last


def rel_target(pose_deg, settle=4.0):
    """(dx, dy, dyaw) from the current (settled) map pose to a map pose [x, y, yaw_deg], in the robot frame."""
    x, y, yaw = settled_pose(settle); tx, ty, th = pose_deg; th = math.radians(th)
    c, s = math.cos(yaw), math.sin(yaw); wx, wy = tx - x, ty - y
    return c * wx + s * wy, -s * wx + c * wy, (th - yaw + math.pi) % (2 * math.pi) - math.pi


BLOCKED_DIR = os.path.join(HERE, 'blocked')   # lidar snapshots that blocked a straight move (diagnosis)
BLOCKED_KEEP = 10                             # newest files kept (~3 MB each)
TRANSLATE_WAIT = [None]                       # s: wait for a blocked straight move (None: BLOCK_WAIT)


def save_blocked(p, tag, hits):
    """Keep the snapshot that blocked a move (float32, base_link) so it can be looked at afterwards."""
    try:
        os.makedirs(BLOCKED_DIR, exist_ok=True)
        f = os.path.join(BLOCKED_DIR, 'blocked-%s-%s.npy' % (time.strftime('%Y%m%d-%H%M%S'), tag))
        np.save(f, np.asarray(p, np.float32))
        for old in sorted(x for x in os.listdir(BLOCKED_DIR) if x.startswith('blocked-'))[:-BLOCKED_KEEP]:
            os.remove(os.path.join(BLOCKED_DIR, old))
        log('  blocking snapshot saved: %s (%s)' % (f, json.dumps(hits)))
    except Exception as e:
        log('  blocking snapshot not saved: %s' % e)


def clear_path(p, dx, dy):
    """The first of: sideways then forward, forward then sideways, whose swept volume is clear in p; else None."""
    for path in ([(0.0, dy), (dx, 0.0)], [(dx, 0.0), (0.0, dy)]):     # pure moves only (see rel)
        q, ok = p, True
        for mx, my in path:
            if (abs(mx) > 1e-3 or abs(my) > 1e-3) and RM.sweep_hits(q, CFG, mx, my):
                ok = False; break
            q = q - np.array([mx, my, 0.0])
        if ok:
            return path
    return None


def translate(dx, dy):
    """Straight translation without turning by (dx, dy) in the robot frame: sideways then forward, or forward
    then sideways - the first whose swept volume is clear in the live cloud.  Blocked: fresh scans for up to
    BLOCK_WAIT s before giving up (2026-11-17: 24 lidar returns 2 mm inside the margin behind T, in one snapshot
    only - nothing was there, the next approach from the same side was clear - aborted the return to T)."""
    if abs(dx) <= 0.005 and abs(dy) <= 0.005:
        return            # nothing to move: no lidar snapshot (a needless one delayed the first command by 1.7 s)
    wait = BLOCK_WAIT if TRANSLATE_WAIT[0] is None else TRANSLATE_WAIT[0]
    p = cloud(); t0 = time.time(); k = 0
    while True:
        path = clear_path(p, dx, dy)
        if path:
            break
        hits = RM.sweep_hits(p, CFG, dx, dy)
        if k == 0:
            save_blocked(p, 'first', hits)
        left = wait - (time.time() - t0)
        if left <= 0 or _FROZEN[0] is not None:                 # a frozen snapshot never changes
            if k > 0:
                save_blocked(p, 'last', hits)
            # BLOCKED (after waiting): the task on the Orin asks to clear it and press Enter to retry
            raise SystemExit('ABORT: %sno clear straight path to the station (x %.2f, y %.2f): %s%s'
                             % ('BLOCKED: ' if k else '', dx, dy, json.dumps(hits),
                                (' - still there after %.0f s' % wait) if k else ''))
        if k % 5 == 0:
            log('WAITING: straight path (x %.2f, y %.2f) blocked by %s - fresh scans for %.0f s more'
                % (dx, dy, json.dumps(hits), left))
        k += 1; spin(1.0); p = cloud(fresh=True)
    if k:
        log('  straight path clear on a fresh scan after %d rescans (%.1f s)' % (k, time.time() - t0))
    for mx, my in path:
        if abs(mx) > 0.005 or abs(my) > 0.005:
            rel(mx + (math.copysign(FWD_SHORT, mx) if abs(mx) > 0.05 else 0.0), 0.0, rear_check=mx < 0, y=my)


# ---- reference-scan positioning (map stations with a recorded scan): ~1 cm instead of the map's ~10 cm ----
REF_DIR = os.path.join(HERE, 'refs')
REF_TOL, REF_ANG = 0.02, math.radians(1.0)   # map stations (handover): the chassis ignores smaller moves anyway


def scan2d(p=None):
    """Fixed surroundings as 3-D points (robot frame) at heights with vertical structure only - legs and cabinet
    sides (0.35-0.65 m), walls / boards / frames (1.0-1.7 m); table and belt tops (flat areas) carry no position
    information and would hold ICP at its start.  Own points out, within 3.5 m, 2 cm grid."""
    p = RM.remove_self(cloud(fresh=True) if p is None else p, CFG)
    z = p[:, 2]
    p = p[(((z > 0.35) & (z < 0.65)) | ((z > 1.0) & (z < 1.7))) & (np.hypot(p[:, 0], p[:, 1]) < 3.5)]
    return np.unique(np.round(p / 0.02), axis=0) * 0.02


def _normals_xy(ref, radius=0.06):
    """Unit surface normal in the horizontal plane per reference point (PCA of its 3-D neighbours' x, y) and
    whether that point lies on a clear vertical surface (thin, elongated neighbourhood)."""
    from scipy.spatial import cKDTree
    # vectorised: the K nearest neighbours within `radius` (a per-point Python loop took ~4 s for the D scan)
    K = 32
    d, idx = cKDTree(ref).query(ref, k=min(K, len(ref)), distance_upper_bound=radius)
    ok = np.isfinite(d); cnt = ok.sum(1)
    P = ref[np.where(ok, idx, 0), :2] * ok[..., None]
    mean = P.sum(1) / np.maximum(cnt, 1)[:, None]
    Q = (P - mean[:, None, :]) * ok[..., None]
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', Q, Q))
    good = (cnt >= 5) & (w[:, 1] > 1e-6) & (w[:, 0] < 0.1 * w[:, 1])
    nrm = np.where(good[:, None], v[:, :, 0], 0.0)
    return nrm, good


ICP_MAX_SRC = 6000      # scan points used by the ICP (random subset): enough for 3 parameters, ~3x faster


def icp2d(src, ref, max_d=0.30, iters=30, nrm=None, tree=None, seed=None):
    """Trimmed point-to-plane ICP on 3-D points, solving only the planar motion (x, y, yaw):
    (R, t, rms, inlier fraction) with ref_xy ~ R @ src_xy + t, or None.  seed: (dx, dy, dyaw) still to go to
    the reference pose (robot frame), e.g. from the map, as the starting estimate."""
    from scipy.spatial import cKDTree
    if nrm is None:
        nrm = _normals_xy(ref)
    nv, good = nrm
    if len(src) > ICP_MAX_SRC:
        src = src[np.random.default_rng(0).choice(len(src), ICP_MAX_SRC, replace=False)]
    tree = tree if tree is not None else cKDTree(ref); th = 0.0; t = np.zeros(2); rms = None; frac = 0.0
    if seed is not None:
        th = -float(seed[2]); c0, s0 = math.cos(th), math.sin(th)
        t = -np.array([[c0, -s0], [s0, c0]]) @ np.array([float(seed[0]), float(seed[1])])
    for _ in range(iters):
        c, s_ = math.cos(th), math.sin(th); R = np.array([[c, -s_], [s_, c]])
        sx = src[:, :2] @ R.T + t
        d, idx = tree.query(np.column_stack([sx, src[:, 2]]), distance_upper_bound=max_d)
        ok = np.isfinite(d)
        if ok.sum() < 100:
            return None
        ok &= d <= np.percentile(d[ok], 80)                    # trim: people, moved chairs, the robot's shadow
        frac = float(ok.sum()) / len(src); rms = float(np.sqrt((d[ok] ** 2).mean()))
        ok &= good[np.where(ok, idx, 0)]
        if ok.sum() < 50:
            return None
        P, B, N = sx[ok], ref[idx[ok], :2], nv[idx[ok]]
        r = ((P - B) * N).sum(1)                               # point-to-plane residuals
        J = np.column_stack([-P[:, 1] * N[:, 0] + P[:, 0] * N[:, 1], N[:, 0], N[:, 1]])
        dx = -np.linalg.lstsq(J, r, rcond=None)[0]
        dth, dt = float(dx[0]), dx[1:]
        cd, sd = math.cos(dth), math.sin(dth)
        t = np.array([[cd, -sd], [sd, cd]]) @ t + dt; th += dth
        max_d = max(0.06, max_d * 0.8)
        if abs(dth) < 2e-5 and np.hypot(*dt) < 2e-4:
            break
    c, s_ = math.cos(th), math.sin(th)
    return np.array([[c, -s_], [s_, c]]), t, rms, frac


_REF_NRM = {}


def ref_data(name):
    """Reference scan of `name` with its normals and search tree, prepared once per file (docking calls this at
    the start, so the planning steps later only run the matching)."""
    from scipy.spatial import cKDTree
    f = os.path.join(REF_DIR, name + '.npy')
    key = (f, os.path.getmtime(f))
    if _REF_NRM.get(name, (None,))[0] != key:
        ref = np.load(f); _REF_NRM[name] = (key, ref, _normals_xy(ref), cKDTree(ref))
    return _REF_NRM[name]


def ref_offset(name, seed=None):
    """(dx, dy, dyaw) in the robot frame from here to the pose where the reference scan of `name` was taken."""
    _, ref, nrm, tree = ref_data(name)
    r = icp2d(scan2d(), ref, nrm=nrm, tree=tree, seed=seed)
    if r is None:
        raise SystemExit('ABORT: the scan does not match the reference of %s (too few common points)' % name)
    R, t, rms, frac = r; th = math.atan2(R[1, 0], R[0, 0])
    d = R.T @ (-t)
    if not (rms > 0.04 or frac < 0.5):
        LAST_ICP[name] = (R, t, MOVES[0])
    log('  scan match: rms %.3f m, %.0f%% matched, to go x %.3f y %.3f yaw %.2f deg' % (rms, 100 * frac, d[0], d[1], -math.degrees(th)))
    if rms > 0.04 or frac < 0.5:
        raise SystemExit('ABORT: poor match with the reference of %s (rms %.3f m, %.0f%%) - surroundings changed? '
                         'teach-ref again' % (name, rms, 100 * frac))
    return float(d[0]), float(d[1]), -th


LAST_ICP = {}           # name -> (R, t, moves): robot frame -> reference-pose frame of the last good match
BG_VOX = 0.03           # m: background voxels; a live point within one voxel of a recorded one is "already there"
BG_AHEAD = 0.05         # background is recorded only ahead of the robot's front edge - 5 cm (the person guarding
                        # the robot stands beside / behind it and must never become "background")
_BG = {}


def bg_file(name):
    return os.path.join(REF_DIR, name + '_bg.npy')


def _hash3(k):
    k = np.asarray(k).astype(np.int64) + 32768
    return (k[:, 0] << 34) | (k[:, 1] << 17) | k[:, 2]


def bg_data(name):
    """Recorded background of station `name` (voxels in its reference-pose frame), dilated by one voxel and
    sorted for a fast lookup; None if none recorded."""
    f = bg_file(name)
    if not os.path.exists(f):
        return None
    key = os.path.getmtime(f)
    if _BG.get(name, (None,))[0] != key:
        v = np.load(f).astype(np.int64)
        offs = np.array(np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])).T.reshape(-1, 3)
        _BG[name] = (key, len(v), np.unique(np.concatenate([_hash3(v + o) for o in offs])))
    return _BG[name]


def to_station(p, R, t):
    q = p.copy(); q[:, :2] = p[:, :2] @ R.T + t
    return q


def station_transform(name, st):
    """(R, t): robot frame -> frame of the reference pose of `name`, for where the robot stands now."""
    c = LAST_ICP.get(name)
    if c is None or c[2] != MOVES[0]:
        ref_offset(name, seed=rel_target(st['pose']))
        c = LAST_ICP[name]
    return c[0], c[1]


def background_filter(p, name, st):
    """Leave out what was already there when the background of `name` was recorded (conveyor, belt, frames,
    cabinets ...): only NEW things - a tray left on the belt, a box, a person - can block the docking."""
    bg = bg_data(name)
    if bg is None or not len(p):
        return p
    R, t = station_transform(name, st)
    h = _hash3(np.floor(to_station(p, R, t) / BG_VOX))
    dil = bg[2]; i = np.minimum(np.searchsorted(dil, h), len(dil) - 1)
    return p[dil[i] != h]


def clear_cloud(name, st, fwd, dy):
    """Points that can block (robot frame): own points, accepted structure, the work surface and the recorded
    background left out.  fwd, dy: where the final pose is, as for accepted_filter."""
    return background_filter(accepted_filter(RM.remove_self(cloud(), CFG), st, fwd, dy), name, st)


def add_background(name, st, where):
    """Record what is around now (ahead of the robot) as background of `name`.  The place zone above the work
    surface is never recorded: whatever stands there must always block (e.g. the last tray still on the belt)."""
    R, t = station_transform(name, st)
    p = RM.remove_self(cloud(), CFG)
    p = p[(p[:, 0] > FRONT - BG_AHEAD) & (np.hypot(p[:, 0], p[:, 1]) < 3.5) & (p[:, 2] > 0.02) & (p[:, 2] < 2.0)]
    q = to_station(p, R, t)
    face = FRONT + float(st.get('ref_gap', st['gap']))           # the face in the reference-pose frame
    sz = RM.surface_height(q, face)
    if st.get('place'):
        if sz is None:
            raise SystemExit('ABORT: cannot see the surface of %s - background not recorded' % name)
        q = q[~((q[:, 0] > face - 0.02) & (q[:, 0] < face + 0.8) & (np.abs(q[:, 1]) < 0.5) & (q[:, 2] > sz + 0.02))]
    v = np.unique(np.floor(q / BG_VOX).astype(np.int64), axis=0)
    f = bg_file(name)
    old = np.load(f).astype(np.int64) if os.path.exists(f) else np.zeros((0, 3), np.int64)
    allv = np.unique(np.concatenate([old, v]), axis=0)
    np.save(f, allv.astype(np.int32))
    log('background of %s recorded at %s: %d voxels here, %d new, %d in all (place zone above the surface %s m '
        'left out)' % (name, where, len(v), len(allv) - len(old), len(allv), None if sz is None else round(sz, 3)))


BLOCK_WAIT = float(os.environ.get('GENTO_BLOCK_WAIT', '30'))     # s: wait this long for a blocking object to go


def guarded(fn, what):
    """Run a no-motion check that raises "... blocked by obstacle ..." (need_clear); while it is blocked, wait
    (fresh scans) up to BLOCK_WAIT s for the object to go - a person walking past, a tray being taken off -
    then give up with ABORT: BLOCKED (the task on the Orin then asks to clear it and retry)."""
    t0 = time.time(); k = 0
    while True:
        try:
            return fn()
        except SystemExit as e:
            if 'blocked by obstacle' not in str(e):
                raise
            left = BLOCK_WAIT - (time.time() - t0)
            if left <= 0:
                raise SystemExit('ABORT: BLOCKED: %s - still there after %.0f s' % (str(e).replace('ABORT: ', ''), BLOCK_WAIT))
            if k % 5 == 0:
                log('WAITING: %s - waiting for it to clear (%.0f s left)' % (str(e).replace('ABORT: ', ''), left))
            k += 1; spin(1.0); _CACHE['t'] = 0.0                  # next check on a fresh scan


def ref_approach(name):
    for it in range(4):
        dx, dy, da = ref_offset(name)
        if math.hypot(dx, dy) <= REF_TOL and abs(da) <= REF_ANG:
            break
        if math.hypot(dx, dy) > 0.5 or abs(da) > math.radians(12):
            raise SystemExit('ABORT: reference match says %.2f m / %.1f deg away - not trusted' % (math.hypot(dx, dy), math.degrees(da)))
        if abs(da) > math.radians(1.0):                         # below ~1 deg the chassis does not turn at all
            need_clear(RM.turn_hits(cloud(), CFG, da), 'heading correction')
            rel(0.0, da)
            continue
        translate(dx, dy)
    dx, dy, da = ref_offset(name)
    return dx, dy, da


def map_approach(name, st, dry):
    """From the safe point: turn in place to the station heading (clear there), translate straight to the
    station, then correct with the map pose until within MAP_TOL / MAP_ANG."""
    turn_to(math.radians(st['pose'][2]))
    has_ref = os.path.exists(os.path.join(REF_DIR, name + '.npy'))
    # with a reference scan only the main move uses the map: map corrections near T chase ~10 cm jumps and the
    # scan match then has to undo them (measured: +9 cm by the map, -8 cm back by the scan)
    # heading corrections are counted on their own: two in a row once used up the whole budget and the scan match
    # then ran 0.7 m short of T (4% matched -> ABORT, 2026-11-17)
    moves, turns, it = 0, 0, 0
    while True:
        dx, dy, da = rel_target(st['pose'], settle=MAP_SETTLE)
        log('map approach %d: to go x %.3f y %.3f m, yaw %.2f deg' % (it, dx, dy, math.degrees(da))); it += 1
        if math.hypot(dx, dy) <= MAP_TOL and abs(da) <= MAP_ANG:
            break
        if math.hypot(dx, dy) > MAP_MAX_MOVE:
            raise SystemExit('ABORT: station %.2f m away from the safe point (check localisation)' % math.hypot(dx, dy))
        if dry:
            log('  (dry) path clear:', not RM.sweep_hits(cloud(), CFG, dx, dy)); return
        if abs(da) > math.radians(12):
            raise SystemExit('ABORT: heading %.1f deg off before translating' % math.degrees(da))
        if abs(da) > MAP_ANG and turns < MAP_TURNS:
            need_clear(RM.turn_hits(cloud(), CFG, da), 'heading correction')
            rel(0.0, da + math.copysign(min(ROT_SHORT, abs(da) / 2), da)); turns += 1
            continue                            # the frame turned: recompute the translation
        if math.hypot(dx, dy) <= MAP_TOL:
            break                               # only the heading is left (turns used up): the scan match / final
        translate(dx, dy); moves += 1
        if has_ref or moves > MAP_CORRECTIONS:
            break                               # with a reference scan the scan match does the rest
    if not has_ref and moves > MAP_CORRECTIONS:
        log('map approach: stopping after %d corrections' % (moves - 1))
    if has_ref:
        dx, dy, da = rel_target(st['pose'], settle=MAP_SETTLE)
        if math.hypot(dx, dy) > MAP_REF_MAX:
            raise SystemExit('ABORT: still %.2f m from %s by the map before the scan match - the approach did not '
                             'get there (localisation?)' % (math.hypot(dx, dy), name))
        # the map is only good to ~10 cm here: finish against the recorded reference scan
        dx, dy, da = ref_approach(name)
        log('FINAL', json.dumps(dict(name=name, mode='map+ref', err_x=round(dx, 4), err_y=round(dy, 4),
                                     err_yaw_deg=round(math.degrees(da), 2))))
        return
    dx, dy, da = rel_target(st['pose'])
    log('FINAL', json.dumps(dict(name=name, mode='map', err_x=round(dx, 4), err_y=round(dy, 4),
                                 err_yaw_deg=round(math.degrees(da), 2), pose=[round(v, 3) for v in map_pose()[:2]])))


LEAVE_MIN = 0.06       # m: the safe point is only a place to turn (STAGING_TOL 0.35): smaller pieces are skipped


def rough(dx, dy):
    """The way back to a safe point without its tiny components (each one is a separate stop-and-go move)."""
    rx_, ry_ = (dx if abs(dx) >= LEAVE_MIN else 0.0), (dy if abs(dy) >= LEAVE_MIN else 0.0)
    if (rx_, ry_) != (dx, dy):
        log('  leaving: skipping the small part (x %.3f, y %.3f) - the safe point needs no precision' % (dx - rx_, dy - ry_))
    return rx_, ry_


def leave_map_station():
    """At a map station (robot within 0.3 m / 20 deg of its pose): go back to its safe point the way the docking
    came in (straight translation), so the next navigation starts where turning is clear."""
    for nm, st in load_st().items():
        if st.get('mode') != 'map' or 'staging_pose' not in st:
            continue
        dx, dy, da = rel_target(st['pose'])
        if math.hypot(dx, dy) < 0.3 and abs(da) < math.radians(20):
            sx, sy, _ = rel_target(st['staging_pose'])
            log('at map station %s: back to its safe point first (x %.2f, y %.2f)' % (nm, sx, sy))
            translate(*rough(sx, sy))
            return True
    # lidar station after undock (e.g. D): on its approach line, facing it, within reach of its safe point -
    # go back to the safe point (the reverse of the approach's sideways move): room to turn there, unlike in
    # front of the station where the vendor's turn creeps past table legs (measured: 38-54 s for the T leg)
    for nm, st in load_st().items():
        if st.get('mode') == 'map' or 'staging_pose' not in st or 'pose' not in st:
            continue
        x, y, yaw = settled_pose()
        fx, fy, fh = st['pose']; h = math.radians(fh)
        ahead = math.cos(h) * (fx - x) + math.sin(h) * (fy - y)        # distance to the final pose along its line
        side = -math.sin(h) * (x - fx) + math.cos(h) * (y - fy)
        dh = abs((yaw - h + math.pi) % (2 * math.pi) - math.pi)
        if 0.2 < ahead < MAX_FWD + 0.3 and abs(side) < 0.4 and dh < math.radians(20):
            sx, sy, _ = rel_target(st['staging_pose'])
            if math.hypot(sx, sy) > 1.6:
                continue
            log('in front of %s: to its safe point first (x %.2f, y %.2f)' % (nm, sx, sy))
            translate(*rough(sx, sy))
            return True
    return False


def face_towards(xy, min_deg=15.0):
    """Before handing over to the vendor navigation: turn in place towards the next destination, if that turn
    is clear with the arms + tray (the vendor's own start turn only sees obstacles below 0.8 m and creeps near
    table legs).  Not clear -> leave the turn to the vendor, as before."""
    x, y, yaw = settled_pose()
    if math.hypot(xy[0] - x, xy[1] - y) < 0.5:
        return                  # already there: its direction means nothing (a 2 cm target gave a 60 deg turn)
    d = (math.atan2(xy[1] - y, xy[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi
    if abs(d) < math.radians(min_deg):
        return
    h = RM.rotation_hits(cloud(fresh=True), CFG)
    if h:
        log('pre-turn towards the destination skipped (not clear: %s)' % json.dumps(h))
        return
    log('pre-turn %.1f deg towards the destination' % math.degrees(d))
    rel(0.0, d)


def goto_waypoint(name, wp):
    """Route variation: drive through a taught waypoint on the way.  A waypoint only shapes the route: if the
    navigation does not get there in time it is cancelled and the trip continues straight to the destination."""
    pose = wp['pose']
    log('route: via waypoint %s (%.2f, %.2f)' % (name, pose[0], pose[1]))
    if RM.rotation_hits(cloud(), CFG):
        move_to_turn_spot(0.6)
    face_towards(pose[:2])
    if not S[0].is_soft_auto:
        setmode(0)
    out = nav_task({}, pose); log(out)
    if ' 0 Succ' not in out:
        log('route: waypoint task not created - going straight to the destination'); return False
    if not wait_task(tmo=WAYPOINT_TMO, near=pose[:2], tol=WAYPOINT_TOL):
        cancel_all_tasks(); spin(1.0)
        log('route: waypoint %s not reached in %d s - going straight to the destination' % (name, WAYPOINT_TMO))
        return False
    spin(0.3); x, y, _ = map_pose()
    log('route: at waypoint %s (%.2f m off)' % (name, math.hypot(x - pose[0], y - pose[1])))
    return True


def unjitter(st, why):
    """Back from a jittered spot to the taught safe point (straight translation, checked)."""
    dx, dy, _ = rel_target(st['staging_pose'])
    log('jittered safe point: %s - back to the taught safe point (x %.3f, y %.3f)' % (why, dx, dy))
    translate(dx, dy)


def cancel_all_tasks():
    from server.srv import CancelAllTask
    c = n.create_client(CancelAllTask, 'server/CancelAllTask')
    if c.wait_for_service(timeout_sec=3):
        f = c.call_async(CancelAllTask.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=3)


class NavTrace:
    """Pose samples during a vendor navigation: logs a line every 2 s and, at the end, the pauses (standing
    still > 0.6 s on the way) and reversals (driving backwards), to see where the navigation hesitates."""
    def __init__(self):
        self.s = []; self.t_log = 0.0

    def add(self):
        if _POSE[0] is None:
            return
        p = _POSE[0].pose; t = time.time(); self.s.append((t, p.position.x, p.position.y, p.orientation.z))
        if t - self.t_log >= 2.0:
            self.t_log = t
            log('NAV pose (%.2f, %.2f, %.0f deg)' % (p.position.x, p.position.y, math.degrees(p.orientation.z)))

    def summary(self):
        s = self.s
        if len(s) < 3:
            return
        pauses = []; rev = 0; dist = 0.0; still_from = None; moved = False
        for (t0, x0, y0, h0), (t1, x1, y1, h1) in zip(s, s[1:]):
            dt = max(t1 - t0, 1e-3); dx, dy = x1 - x0, y1 - y0; d = math.hypot(dx, dy); dist += d
            dh = abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
            if d / dt < 0.02 and dh / dt < math.radians(3):
                still_from = still_from or t0
            else:
                if moved and still_from and t0 - still_from > 0.6:
                    pauses.append((still_from, t0 - still_from))
                still_from = None; moved = True
                if d > 0.01 and dx * math.cos(h0) + dy * math.sin(h0) < -0.005:
                    rev += 1
        log('NAV summary: %.1f s, %.2f m driven, %d pauses on the way%s, %d backwards samples' % (
            s[-1][0] - s[0][0], dist, len(pauses),
            (' (' + ', '.join('%s %.1f s' % (stamp(a), b) for a, b in pauses[:6]) + ')') if pauses else '', rev))


NEAR_STOP = 0.15        # m: this close to a safe point / waypoint the navigation is stopped (arrived)
NEAR_DWELL = 3.0        # s: this long within STAGING_TOL (manoeuvring around it) also counts as arrived


def wait_task(tmo=150, near=None, tol=STAGING_TOL):
    """Wait until the navigation task is gone.  The vendor server keeps a station task open when the robot
    stops a few cm off the node ("Current Pos Without Node"); so if we are within `tol` of the staging point
    and have not moved for 1.5 s, treat it as arrived and cancel the dangling task ourselves."""
    import urllib.request
    t0 = time.time(); last = None; still = time.time(); mw = NAV_WATCH[0]; NAV_WATCH[0] = None
    tr = NavTrace()
    while time.time() - t0 < 0.5:
        rclpy.spin_once(n, timeout_sec=0.05)
        if mw:
            mw.poll()
    zone = None
    while time.time() - t0 < tmo:
        q = urllib.request.Request('http://127.0.0.1:8080/api/Task/GetAllTask', data=b'{}',
                                   headers={'Content-Type': 'application/json'})
        if not json.load(urllib.request.urlopen(q, timeout=5))['Content']:
            tr.summary(); return True
        if mw:
            mw.poll()
        tr.add()
        if near:
            x, y, yaw = map_pose()
            if last is None or math.hypot(x - last[0], y - last[1]) > 0.005 or abs(yaw - last[2]) > 0.005:
                still = time.time(); last = (x, y, yaw)
            dd = math.hypot(x - near[0], y - near[1])
            if dd < tol and time.time() - still > 1.5:
                log('arrived %.3f m from staging but the server keeps the task open: cancelling it' % dd)
                tr.summary(); cancel_all_tasks(); return True
            # the safe point is only a place to turn: the vendor otherwise keeps manoeuvring onto the exact pose
            # (2026-11-16 at D: 22 s of reversing and turning within 0.3 m of it)
            if dd < NEAR_STOP:
                log('within %.2f m of the safe point: navigation stopped there' % dd)
                cancel_all_tasks(); spin(0.5); tr.summary(); return True
            zone = (zone or time.time()) if dd < tol else None
            if zone and time.time() - zone > NEAR_DWELL:
                log('%.0f s within %.2f m of the safe point (%.2f m now): navigation stopped there' % (NEAR_DWELL, tol, dd))
                cancel_all_tasks(); spin(0.5); tr.summary(); return True
        for _ in range(4):
            rclpy.spin_once(n, timeout_sec=0.05)
            if mw:
                mw.poll()
    return False


def find_staging(p, dist, back):
    """p: cloud (own points removed) in the frame of a station's FINAL pose; dist: centre -> face there.
    Staging spot b m behind and s m to the right of the final pose, where the robot + arms + tray can turn in
    place (also STAGING_MARGIN around it: navigation stops a few cm off), and the sideways move back onto the
    final line and the straight approach are clear.  The nearest one (smallest sideways offset, then smallest
    distance) from `back` on, as (b, s); None if there is none."""
    def ok(b, s):
        if any(RM.rotation_hits(p + np.array([b + db, s + ds, 0.0]), CFG)        # cheapest test first
               for db, ds in ((0, 0), (STAGING_MARGIN, 0), (-STAGING_MARGIN, 0), (0, STAGING_MARGIN), (0, -STAGING_MARGIN))):
            return False
        if RM.sweep_hits(p + np.array([b, s, 0.0]), CFG, 0.0, s):
            return False
        f = RM.max_forward(p + np.array([b, 0.0, 0.0]), CFG, b)
        return f is not None and f >= b - 0.005
    # from `back` outwards first; then closer spots (down to right beside the pose: slide in sideways)
    bs = [b for b in np.arange(back, MAX_FWD + 0.001, 0.05) if dist + b <= 2.1] + list(np.arange(0.0, back - 0.001, 0.05))
    ss = sorted(np.arange(-LAT_PLAN_MAX + 0.1, LAT_PLAN_MAX - 0.1 + 0.001, 0.05), key=abs)
    return next(((round(float(b), 3), round(float(s), 3)) for s in ss for b in bs if ok(b, s)), None)


def staging_pose_of(pose_deg, b, s):
    x, y, h = pose_deg; yaw = math.radians(h)
    return [round(x - b * math.cos(yaw) + s * math.sin(yaw), 3), round(y - b * math.sin(yaw) - s * math.cos(yaw), 3),
            round(h, 2)]


def cloud_in_pose_frame(pose_deg):
    """Current cloud (own points removed) expressed in the frame of a map pose [x, y, yaw_deg] (map localisation)."""
    xc, yc, yc_yaw = map_pose(); p = RM.remove_self(cloud(fresh=True), CFG)
    c, s = math.cos(yc_yaw), math.sin(yc_yaw)
    mx, my = c * p[:, 0] - s * p[:, 1] + xc, s * p[:, 0] + c * p[:, 1] + yc
    x, y, h = pose_deg; c, s = math.cos(-math.radians(h)), math.sin(-math.radians(h))
    return np.stack([c * (mx - x) - s * (my - y), s * (mx - x) + c * (my - y), p[:, 2]], 1)


def _planner(fn, what):
    def wrapped(*a, **k):
        plan_start(what)
        return fn(*a, **k)
    wrapped.__name__ = fn.__name__; wrapped.__doc__ = fn.__doc__
    return wrapped


for _fn, _what in (('leave_map_station', 'leave the station (back to its safe point)'),
                   ('move_to_turn_spot', 'move to a spot where turning is clear'),
                   ('face_towards', 'turn towards the destination'), ('safe_rotate', 'turn in place'),
                   ('align_heading', 'heading correction'), ('align_lateral', 'sideways correction'),
                   ('sideways_small', 'small sideways correction'), ('plan_approach', 'approach plan'),
                   ('translate', 'straight path'), ('ref_offset', 'reference scan match'),
                   ('nav_task', 'navigation task to the safe point')):
    globals()[_fn] = _planner(globals()[_fn], _what)

def _after_move(d, dx, dy, dyaw):
    """Predicted (to go x, y, yaw) to the reference pose after moving (dx, dy) and then turning dyaw."""
    x, y = d[0] - dx, d[1] - dy; c, s_ = math.cos(-dyaw), math.sin(-dyaw)
    return (c * x - s_ * y, s_ * x + c * y, d[2] - dyaw)


def _comp(a):
    return a if abs(a) < math.radians(5) else a + math.copysign(ROT_SHORT, a)


def dock_by_ref(name, st, gap, record=False):
    """Docking at a station with a reference scan, from its safe point, WITHOUT the face fit: heading, sideways
    and forward all come from the reference scan (~1 cm; the face fit can lock onto another surface), and only
    NEW objects block (background_filter).  The final pose is `off` = ref_gap - gap beyond the reference pose.
    No pure sideways moves (slant_move).  Blocked checks wait for the object to go (guarded).  record: also
    record the background here, at stage 2 and at the final pose (supervised run, place zone empty)."""
    off = float(st['ref_gap']) - gap
    d = ref_offset(name, seed=rel_target(st['pose']))
    if record:
        add_background(name, st, 'the safe point')
    log('docking %s by the reference scan only: final pose %.3f m beyond the reference pose (gap %.4f, reference '
        'gap %.4f)%s' % (name, off, gap, st['ref_gap'], '' if bg_data(name) else ' - NO background recorded yet'))
    for k in range(3):                                       # heading first, from the scan
        if abs(d[2]) <= ANG_TOL:
            break
        a = d[2]
        log('heading from the reference scan: turn %.2f deg' % math.degrees(a))
        guarded(lambda: need_clear(RM.turn_hits(clear_cloud(name, st, d[0] + off, d[1]), CFG, a), 'heading correction'),
                'heading correction')
        rel(0.0, _comp(a)); d = ref_offset(name, seed=_after_move(d, 0.0, 0.0, a))
    fwd, dy = d[0] + off, d[1]
    if not 0.0 <= fwd <= MAX_FWD or abs(dy) > LAT_PLAN_MAX:
        raise SystemExit('ABORT: the reference scan puts the final pose at x %.3f y %.3f - out of reach from here' % (fwd, dy))
    SURF[0] = RM.surface_height(RM.remove_self(cloud(), CFG), fwd + FRONT + gap)
    log('plan: final pose %.3f m ahead, %.3f m to the side, surface top %s m'
        % (fwd, dy, None if SURF[0] is None else round(SURF[0], 3)))

    def final_checks(fwd, dy):
        p = clear_cloud(name, st, fwd, dy) - np.array([0.0, dy, 0.0])
        need_clear(RM.sweep_hits(p, CFG, fwd, 0.0), 'straight approach on the station line')
        if st.get('place'):
            if SURF[0] is None:
                raise SystemExit('ABORT: cannot see the surface the tray goes on')
            need_clear(RM.place_hits(p, CFG, fwd, SURF[0]), 'place spot under the tray')
            need_clear(RM.place_motion_hits(p, CFG, fwd, SURF[0]), 'place motion after docking')

    guarded(lambda: final_checks(fwd, dy), 'final approach / place spot')
    total = 0.0
    if fwd > STAGE2 + 0.15:
        first = fwd - STAGE2
        if abs(dy) > SLANT_TOL:
            a, dist = guarded(lambda: slant_plan(dy, first, clear_cloud(name, st, fwd, dy), 'approach to stage 2'),
                              'approach to stage 2')
            slant_move(a, dist, FWD_SHORT, 'approach to stage 2')
            d = ref_offset(name, seed=_after_move(d, first, dy, 0.0))
        else:
            guarded(lambda: need_clear(RM.sweep_hits(clear_cloud(name, st, fwd, dy), CFG, first, 0.0), 'approach to stage 2'),
                    'approach to stage 2')
            log('approach to stage 2: forward %.3f m (sideways %.3f m left for the last stage)' % (first, dy))
            rel(first + FWD_SHORT, 0.0)
            d = ref_offset(name, seed=_after_move(d, first, 0.0, 0.0))
        total += first + FWD_SHORT
        if record:
            add_background(name, st, 'stage 2')
        if abs(d[2]) > ANG_TOL:
            a = d[2]
            guarded(lambda: need_clear(RM.turn_hits(clear_cloud(name, st, d[0] + off, d[1]), CFG, a), 'heading correction'),
                    'heading correction')
            rel(0.0, _comp(a)); d = ref_offset(name, seed=_after_move(d, 0.0, 0.0, a))
        fwd, dy = d[0] + off, d[1]
        log('  stage 2 (reference scan): final pose %.3f m ahead, %.3f m to the side, heading %.2f deg'
            % (fwd, dy, math.degrees(d[2])))
        guarded(lambda: final_checks(fwd, dy), 'last stage / place spot')
    slant = None
    if abs(dy) > SLANT_TOL:
        try:
            slant = guarded(lambda: slant_plan(dy, fwd, clear_cloud(name, st, fwd, dy), 'last stage'), 'last stage')
        except SystemExit as e:
            log('last stage: slanted path not clear (%s) - sideways correction instead' % e)
            sideways_small(dy, st, fwd)
    elif abs(dy) >= 0.008:
        log('last stage: sideways %.3f m left alone (<= %.3f)' % (dy, SLANT_TOL))
    if slant:
        slant_move(slant[0], slant[1], FWD_SHORT_LAST, 'last stage')
    else:
        guarded(lambda: need_clear(RM.sweep_hits(clear_cloud(name, st, fwd, 0.0), CFG, fwd, 0.0), 'last stage'),
                'last stage')
        log('last stage: forward %.3f m' % fwd)
        rel(fwd + FWD_SHORT_LAST, 0.0)
    total += fwd + FWD_SHORT_LAST
    # remainder from the scan: the chassis stops ~1-1.5 cm short on some floors (see final_forward_fix)
    d = ref_offset(name, seed=_after_move(d, fwd, dy, 0.0)); rem = d[0] + off
    if FINAL_X_TOL < rem <= FINAL_X_MAX:
        steps = [rem] if rem >= SIDE_MIN else [-0.03, 0.03 + rem]
        guarded(lambda: need_clear(RM.sweep_hits(clear_cloud(name, st, rem, 0.0), CFG, rem, 0.0), 'final forward correction'),
                'final forward correction')
        log('final forward correction %.3f m (%s)' % (rem, ' then '.join('%.3f' % x for x in steps)))
        for x in steps:
            rel(x, 0.0, rear_check=x < 0)
        total += rem; d = ref_offset(name, seed=_after_move(d, rem, 0.0, 0.0)); rem = d[0] + off
    else:
        log('final check: %.3f m to the target - no correction' % rem)
    YLIM[0] = 2.5; f = measure(near=True)
    if record:
        add_background(name, st, 'the final pose')
        if f:
            rg = f['gap'] - d[0]                          # face gap at the reference pose (d[0] still to go)
            if abs(rg - st['ref_gap']) <= 0.04:
                all_st = load_st(); all_st[name]['ref_gap'] = round(rg, 4); json.dump(all_st, open(ST_FILE, 'w'), indent=1)
                log('reference gap of %s re-measured: %.4f (was %.4f)' % (name, rg, st['ref_gap']))
            else:
                log('reference gap re-measured as %.4f, far from %.4f - not stored (face fit?)' % (rg, st['ref_gap']))
    res = dict(name=name, mode='ref', gap_target=gap, gap=f['gap'] if f else None, ang_deg=f['ang_deg'] if f else None,
               err_x=round(rem, 4), err_y=round(d[1], 4), err_yaw_deg=round(math.degrees(d[2]), 2), fwd=round(total, 4))
    log('FINAL', json.dumps(res))
    json.dump(dict(fwd=total, gap=gap, t=time.time()), open('/tmp/claude_dock_last.json', 'w'))
    raise SystemExit(0)


cmd = sys.argv[1]; args = sys.argv[2:]; dry = '--dry' in args; args = [a for a in args if a != '--dry']
if '--wide' in args:
    YLIM[0] = 2.5; args.remove('--wide')
if '--z' in args:
    i = args.index('--z'); ZB[0], ZB[1] = float(args[i + 1]), float(args[i + 2]); args = args[:i] + args[i + 3:]
if '--via' in args:
    i = args.index('--via'); VIA[0] = args[i + 1]; args = args[:i] + args[i + 2:]
if '--jitter' in args:
    i = args.index('--jitter'); JITTER[0] = float(args[i + 1]); args = args[:i] + args[i + 2:]
    assert 0.0 <= JITTER[0] <= 0.20, '--jitter must be 0-0.20 m'
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
    if old.get('mode') == 'map':
        # map station: only the map pose matters; its safe point (teach-staging) is kept
        x, y, yaw = map_pose()
        old.update(pose=[round(x, 3), round(y, 3), round(math.degrees(yaw), 2)], taught=time.strftime('%Y-%m-%d %H:%M:%S'))
        st[name] = old; json.dump(st, open(ST_FILE, 'w'), indent=1)
        log('taught', name, '(map station)', json.dumps(old)); raise SystemExit(0)
    m = measure(near=True)
    if not m:
        raise SystemExit('ABORT: no face found ahead (z band %.2f-%.2f m)' % tuple(ZB))
    log('face at the final pose', pub(m))
    if m['dist'] + back > 2.1 or not 0.0 <= back <= MAX_FWD:
        raise SystemExit('ABORT: BACK %.2f m gives a staging distance of %.2f m to the face (need <= 2.1, BACK 0-%.2f)'
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
    pick = find_staging(p, m['dist'], back)
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
        st[name].update(staging_pose=staging_pose_of(st[name]['pose'], b, s), back=b, staging_right=s)
    else:
        # a vendor station may face another edge (e.g. an older station): never fall back to it silently
        st[name]['no_staging'] = True
        notes.append('NO STAGING SPOT: nowhere within %.1f m back / %.1f m sideways can the arms turn in place and '
                     'then slide / drive straight to this pose; docking at %s will stop until there is one (move '
                     'what is in the way, then run restage %s)' % (MAX_FWD, LAT_PLAN_MAX - 0.1, name, name))
    if pick is not None:
        st[name].pop('no_staging', None)
    json.dump(st, open(ST_FILE, 'w'), indent=1)
    for t in notes:
        log('NOTE:', t)
    log('taught', name, json.dumps(st[name]))
elif cmd == 'teach-staging':
    # teach-staging NAME [map] : NO MOTION.  The robot stands at a safe point near station NAME (room to turn
    # with the arms + tray).  Records the current map pose as NAME's staging point; with "map" the station is
    # docked by map position: navigate here, turn to the station heading, translate straight to its pose.
    name = args[0]; st = load_st()
    if name not in st or 'pose' not in st[name]:
        raise SystemExit('ABORT: teach the final pose of %s first (teach-pose)' % name)
    check_localisation()
    h = RM.rotation_hits(cloud(fresh=True), CFG)
    if h:
        raise SystemExit('ABORT: the arms cannot turn in place here: %s - choose a spot with more room' % json.dumps(h))
    x, y, yaw = map_pose(); s0 = st[name]
    s0.update(staging_pose=[round(x, 3), round(y, 3), round(math.degrees(yaw), 2)])
    for k in ('no_staging', 'back', 'staging_right', 'staging_xy'):
        s0.pop(k, None)
    if 'map' in args[1:]:
        s0['mode'] = 'map'
    if s0.get('mode') != 'map':
        # lidar docking from here: the station's approach line must be within reach (sideways, then straight in)
        fx, fy, fh = s0['pose']; c, sn = math.cos(-math.radians(fh)), math.sin(-math.radians(fh))
        xr, yr = c * (x - fx) - sn * (y - fy), sn * (x - fx) + c * (y - fy)      # safe point in the final-pose frame
        if not (0.0 <= -xr <= MAX_FWD and abs(yr) <= LAT_PLAN_MAX - 0.1):
            raise SystemExit('ABORT: this point is %.2f m behind and %.2f m to the %s of %s - for lidar docking it must '
                             'be 0-%.1f m behind and at most %.1f m to the side' % (-xr, abs(yr), 'left' if yr > 0 else 'right',
                                                                                  name, MAX_FWD, LAT_PLAN_MAX - 0.1))
    dx, dy, da = rel_target(s0['pose'])
    json.dump(st, open(ST_FILE, 'w'), indent=1)
    log('safe point of %s: (%.3f, %.3f); the station is %.2f m away; mode %s' % (name, x, y, math.hypot(dx, dy), s0.get('mode', 'face')))
    log('taught', name, json.dumps(s0))
elif cmd == 'teach-waypoint':
    # teach-waypoint NAME [clear] : NO MOTION.  The robot (arms in the carry pose, holding the tray) stands in an
    # open area that the route may pass through: record the map pose as waypoint NAME (waypoints.json; the
    # stations are not touched).  "clear" removes it.  Used by goto with --via random|NAME.
    name = args[0]; w = RT.load_waypoints(WP_FILE)
    if name in load_st():
        raise SystemExit('ABORT: %s is a station name - use another name for a waypoint (e.g. W1)' % name)
    if 'clear' in args[1:]:
        w.pop(name, None); RT.save_waypoints(WP_FILE, w); log('waypoint', name, 'removed; left:', ', '.join(sorted(w)) or 'none')
        raise SystemExit(0)
    if 'norandom' in args[1:] or 'random' in args[1:]:
        # keep the waypoint, only take it out of (or back into) the random route choice; --via NAME still works
        if name not in w:
            raise SystemExit('ABORT: no waypoint %s (known: %s)' % (name, ', '.join(sorted(w)) or 'none'))
        if 'norandom' in args[1:]:
            w[name]['random'] = False
        else:
            w[name].pop('random', None)
        RT.save_waypoints(WP_FILE, w)
        log('waypoint', name, 'is %s random routes' % ('NOT in' if w[name].get('random') is False else 'in'),
            '| random routes use:', ', '.join(k for k, v in sorted(w.items()) if v.get('random') is not False) or 'none')
        raise SystemExit(0)
    check_localisation()
    h = RM.rotation_hits(cloud(fresh=True), CFG)
    if h and 'force' not in args[1:]:
        raise SystemExit('ABORT: the arms cannot turn in place here: %s - a waypoint needs room to turn '
                         '(verified clear on site: teach-waypoint %s force)' % (json.dumps(h), name))
    if h:
        log('WARNING: the model says turning here would touch %s - recorded anyway (force: verified on site); '
            'leaving the waypoint, the robot only turns itself where that is clear, otherwise the vendor navigation '
            'turns' % json.dumps(h))
    x, y, yaw = map_pose()
    w[name] = dict(pose=[round(x, 3), round(y, 3), round(math.degrees(yaw), 2)], taught=time.strftime('%Y-%m-%d %H:%M:%S'))
    if h:
        w[name]['forced'] = True
    RT.save_waypoints(WP_FILE, w)
    for nm, s0 in load_st().items():
        if 'staging_pose' in s0:
            log('  %.2f m from the safe point of %s' % (math.hypot(x - s0['staging_pose'][0], y - s0['staging_pose'][1]), nm))
    log('taught waypoint', name, json.dumps(w[name]), '| all:', ', '.join(sorted(w)))
elif cmd == 'waypoints':
    # waypoints : NO MOTION.  List the waypoints and which of them a random route between the stations may use
    w = RT.load_waypoints(WP_FILE); st = load_st()
    for k, v in sorted(w.items()):
        log('%-6s %s  (taught %s)%s' % (k, v['pose'], v.get('taught', '?'),
                                        '  NOT in random routes (only --via %s)' % k if v.get('random') is False else ''))
    sp = {k: s['staging_pose'] for k, s in st.items() if 'staging_pose' in s}
    for a in sorted(sp):
        for b in sorted(sp):
            if a != b:
                log('%s -> %s: --via random may use %s' % (a, b, ', '.join(RT.eligible(sp[a], sp[b], w)) or 'none (always direct)'))
    if not w:
        log('no waypoints taught (teach-waypoint NAME)')
elif cmd == 'accept':
    # accept NAME [clear] : NO MOTION.  The robot is lined up on station NAME's approach line (e.g. where a goto
    # stopped before driving in) or stands at its final pose.  Records the lidar points that the (conservative)
    # robot model touches on the rest of the approach and - place stations - in the place motion, as accepted
    # static structure: verified ON SITE that the real arms clear it.  Docking at NAME then ignores exactly
    # those points (+- one 2 cm voxel); anything else (a hand, an item, something moved) still stops it.
    name = args[0]; st = load_st(); s0 = st[name]
    if 'clear' in args[1:]:
        s0.pop('accepted', None); json.dump(st, open(ST_FILE, 'w'), indent=1); log('accepted points of', name, 'cleared')
        raise SystemExit(0)
    ZB[0], ZB[1] = s0['z']; YLIM[0] = 2.5; YAW_OFF[0] = math.radians(float(s0.get('yaw_offset', 0.0)))
    target = s0['end_y'] - float(s0.get('lat_offset', 0.0))
    # far and near fits can find different faces (e.g. a table leg nearby): take the station's face
    cands = [c for c in (measure(), measure(near=True)) if c]
    if not cands:
        raise SystemExit('ABORT: no face found ahead')
    m = min(cands, key=lambda c: abs(lat_error(c, dict(s0, end_y=target), s0['gap'])))
    dy = lat_error(m, dict(s0, end_y=target), s0['gap']); e = m['err'] + YAW_OFF[0]
    fwd = m['dist'] - FRONT - s0['gap']
    if os.path.exists(os.path.join(REF_DIR, name + '.npy')) and 'pose' in s0:
        # stations whose face edge is not reliable (D): sideways / heading from the reference scan
        try:
            rx, ry, ra = ref_offset(name, seed=rel_target(s0['pose']))
            if abs(rx - fwd) <= 0.10:
                log('reference scan: sideways %.3f m, heading %.2f deg (face edge said %.3f m)' % (ry, math.degrees(ra), dy))
                dy, e = ry, ra
        except SystemExit as ex:
            log('reference scan not usable here (%s) - face edge' % ex)
    log('face', pub(m), '| sideways %.3f m, heading %.2f deg, forward %.3f m to the final pose' % (dy, math.degrees(e), fwd))
    if abs(dy) > 0.03 or abs(e) > math.radians(1.5) or not -0.02 <= fwd <= MAX_FWD:
        raise SystemExit('ABORT: not lined up on the approach line of %s - run goto (it lines up before driving in)' % name)
    if fwd < 0.25:
        raise SystemExit('ABORT: too close to the final pose (%.2f m): structure inside the arm model here looks like '
                         'the robot itself - run accept where goto lines up before driving in' % fwd)
    p = RM.remove_self(cloud(), CFG); sz = RM.surface_height(p, m['dist'])
    p = p - np.array([0.0, dy, 0.0])
    q = p[model_hits_mask(p, max(fwd, 0.0), sz, s0.get('place'))] - np.array([max(fwd, 0.0), 0.0, 0.0])
    if not len(q):
        s0.pop('accepted', None); json.dump(st, open(ST_FILE, 'w'), indent=1)
        log('nothing to accept: the model is clear at', name); raise SystemExit(0)
    vox = np.unique(np.floor(q / ACC_VOX).astype(int), axis=0)
    s0['accepted'] = vox.tolist(); json.dump(st, open(ST_FILE, 'w'), indent=1)
    log('accepted at %s: %d points in %d voxels, relative to the final pose x %.2f..%.2f, y %.2f..%.2f, z %.2f..%.2f'
        % (name, len(q), len(vox), q[:, 0].min(), q[:, 0].max(), q[:, 1].min(), q[:, 1].max(), q[:, 2].min(), q[:, 2].max()))
    if len(q) > 3000:
        log('WARNING: that is a lot - make sure no person or loose item was in front of the robot')
elif cmd == 'teach-ref':
    # teach-ref NAME : NO MOTION.  The robot stands EXACTLY at the final pose of map station NAME.  Records the
    # surroundings as a reference scan (and the map pose); docking at NAME then finishes against this scan.
    name = args[0]; st = load_st()
    if name not in st:
        raise SystemExit('ABORT: unknown station %s' % name)
    check_localisation()
    ref = scan2d()
    if len(ref) < 300:
        raise SystemExit('ABORT: too little fixed structure around (%d points) for a reference scan' % len(ref))
    r = icp2d(scan2d(), ref)                                # repeatability: a second scan against the first
    os.makedirs(REF_DIR, exist_ok=True); np.save(os.path.join(REF_DIR, name + '.npy'), ref)
    x, y, yaw = settled_pose()
    st[name].update(pose=[round(x, 3), round(y, 3), round(math.degrees(yaw), 2)], ref=True,
                    taught=time.strftime('%Y-%m-%d %H:%M:%S'))
    json.dump(st, open(ST_FILE, 'w'), indent=1)
    log('reference scan of %s: %d points; repeat scan matches with rms %s m, offset %s m' % (
        name, len(ref), None if r is None else round(r[2], 4), None if r is None else round(float(np.hypot(*r[1])), 4)))
    log('taught', name, json.dumps(st[name]))
elif cmd == 'restage':
    # restage NAME [BACK] : NO MOTION.  Recompute the staging spot of a station taught with teach-pose, from
    # the current scan (robot anywhere near the station, localisation good), e.g. after the robot model changed.
    name = args[0]; back = float(args[1]) if len(args) > 1 else 0.5
    st = load_st(); s0 = st[name]
    if 'pose' not in s0:
        raise SystemExit('ABORT: %s has no taught final pose (use teach-pose)' % name)
    check_localisation()
    p = cloud_in_pose_frame(s0['pose'])
    log('model at the final pose:', json.dumps(RM.sweep_hits(p, CFG, 0.0, 0.0)) or 'clear')
    pick = find_staging(p, s0['gap'] + FRONT, back)
    if pick is None:
        raise SystemExit('ABORT: no staging spot where the arms can turn in place (within %.1f m back, %.1f m '
                         'sideways); %s unchanged' % (MAX_FWD, LAT_PLAN_MAX - 0.1, name))
    b, s = pick
    s0.update(staging_pose=staging_pose_of(s0['pose'], b, s), back=b, staging_right=s); s0.pop('no_staging', None)
    json.dump(st, open(ST_FILE, 'w'), indent=1)
    log('staging spot: %.2f m back, %.2f m to the %s of the final pose' % (b, abs(s), 'right' if s >= 0 else 'left'))
    log('restaged', name, json.dumps(s0))
elif cmd in ('dock', 'redock'):
    # dock NAME [GAP]: navigate to staging (unless redock), align heading, lateral, heading, straight in
    name = args[0]; st = load_st()[name]; gap = float(args[1]) if len(args) > 1 else st['gap']; YLIM[0] = 2.5
    ZB[0], ZB[1] = st['z']
    jittered = False                            # navigated to a random spot near the safe point (--jitter)
    if os.path.exists(os.path.join(REF_DIR, name + '.npy')):
        ref_data(name)                          # prepare the reference now, not inside a planning step
    # fine-tune: lat_offset (m, + = robot further left) shifts the lateral target; yaw_offset (deg, + = left)
    st['end_y'] = st.get('end_y', 0.0) - float(st.get('lat_offset', 0.0)); YAW_OFF[0] = math.radians(float(st.get('yaw_offset', 0.0)))
    if st.get('no_staging') and cmd == 'dock':
        raise SystemExit('ABORT: station %s has no staging spot where the arms can turn in place - move what is in '
                         'the way and run restage %s, or teach the station somewhere with more room' % (name, name))
    if st.get('lat_offset') or st.get('yaw_offset'):
        log('fine-tune: lat_offset %.3f m, yaw_offset %.2f deg' % (st.get('lat_offset', 0.0), st.get('yaw_offset', 0.0)))
    if cmd == 'dock' and dry:
        p0 = cloud()
        log('DRY RUN (no motion): turn at start:', json.dumps(RM.rotation_hits(p0, CFG)) or 'clear',
            '| move needed before turning (x, y):', RM.leave_move(p0, CFG, 0.6),
            '| would navigate to', st.get('staging_pose') or st.get('staging'), '- skipped; checking from the current pose')
    elif cmd == 'dock':
        MOTION_JOB[0] = True
        # the vendor navigation turns in place at the start and only sees obstacles below 0.8 m: first make
        # sure the robot + arms + tray can turn here.  With --leave (e.g. starting at the table edge with the
        # tray over the table) reverse straight out until turning is clear; otherwise stop.
        if not need_state().is_hard_auto:
            raise SystemExit('ABORT: hard switch is on MANUAL - switch it to AUTO first')
        check_localisation()
        if st.get('mode') == 'map' and 'pose' in st:
            dx, dy, da = rel_target(st['pose'])
            if math.hypot(dx, dy) < 0.15 and abs(da) < math.radians(10):
                # already standing at this station (e.g. a new run starts where the last one ended): only the
                # fine correction, no round trip over the safe point
                log('already at %s (%.3f m, %.1f deg): fine correction only' % (name, math.hypot(dx, dy), math.degrees(da)))
                if os.path.exists(os.path.join(REF_DIR, name + '.npy')):
                    dx, dy, da = ref_approach(name)
                log('FINAL', json.dumps(dict(name=name, mode='map+ref', err_x=round(dx, 4), err_y=round(dy, 4),
                                             err_yaw_deg=round(math.degrees(da), 2), note='already there')))
                setmode(0)
                raise SystemExit(0)
        if '--leave' in sys.argv:
            leave_map_station()                 # at / in front of a station: back to its safe point first
        if RM.rotation_hits(cloud(), CFG):
            if '--leave' not in sys.argv:
                need_clear(RM.rotation_hits(cloud(), CFG), 'turning at the start position (navigation turns in place '
                           'first; use --leave to move out first)')
            move_to_turn_spot(0.6)
        target = st.get('staging_pose')
        if target and JITTER[0] > 0:
            # route variation 2: a random spot near the taught safe point (checked again on arrival)
            target = RT.jitter_pose(st['staging_pose'], st, JITTER[0], front=FRONT, max_fwd=MAX_FWD,
                                    lat_max=LAT_PLAN_MAX - 0.1, map_max=MAP_MAX_MOVE)
            jittered = target != list(st['staging_pose'])
            log('route: safe point %s (%.3f, %.3f), %.2f m from the taught one'
                % ('jittered to' if jittered else 'kept at', target[0], target[1],
                   math.hypot(target[0] - st['staging_pose'][0], target[1] - st['staging_pose'][1])))
        if target and VIA[0] != 'none' and '--lineup' not in sys.argv:
            # route variation 1: through a taught waypoint (random or named)
            x0, y0, _ = map_pose(); wps = RT.load_waypoints(WP_FILE)
            try:
                wn, why = RT.choose_via((x0, y0), target[:2], wps, VIA[0])
            except ValueError as e:
                raise SystemExit('ABORT: %s' % e)
            log('route:', why if wn is None else 'via %s (%s)' % (wn, why))
            if wn:
                goto_waypoint(wn, wps[wn])
        dest = target[:2] if target else st.get('staging_xy')
        if dest:
            face_towards(dest)                  # our checked turn instead of the vendor's start turn
        if not S[0].is_soft_auto:
            setmode(0)
        out = nav_task(st, target)                              # in-process service call (no extra process)
        log(out)
        if ' 0 Succ' not in out:
            raise SystemExit('ABORT: task not created')
        sxy = dest
        if not wait_task(near=sxy):
            raise SystemExit('ABORT: staging not reached in time')
        spin(0.3)
        # the task list also empties when the task is cancelled or fails: check we really are at the staging point
        if sxy:
            x, y, _ = map_pose(); dd = math.hypot(x - sxy[0], y - sxy[1])
            if dd > STAGING_TOL + 0.05:
                raise SystemExit('ABORT: navigation ended %.2f m from the staging point (task cancelled or failed?)' % dd)
        if jittered and RM.rotation_hits(cloud(fresh=True), CFG):
            unjitter(st, 'turning is not clear here'); jittered = False
        if st.get('mode') == 'map':
            if jittered:
                TRANSLATE_WAIT[0] = 3.0     # one or two rescans (noise), then the taught safe point instead
            try:
                map_approach(name, st, dry)
            except SystemExit as e:
                # translate() checks before moving: from the jittered spot the straight path may be blocked
                if not (jittered and 'no clear straight path' in str(e)):
                    raise
                TRANSLATE_WAIT[0] = None
                unjitter(st, 'no clear straight path from here'); jittered = False
                map_approach(name, st, dry)
            TRANSLATE_WAIT[0] = None
            setmode(0)
            raise SystemExit(0)
        if 'staging_pose' in st:   # pose tasks keep the start heading: turn to the face here (staging is clear all round)
            turn_to(math.radians(st.get('pose', st['staging_pose'])[2]))     # face the way the final pose faces
        if (cmd == 'dock' and 'ref_gap' in st and 'pose' in st and os.path.exists(os.path.join(REF_DIR, name + '.npy'))
                and not any(a in sys.argv for a in ('--lineup', '--stage-only', '--here'))):
            dock_by_ref(name, st, gap, record='--record-bg' in sys.argv)
    m = measure() or measure(near=True)          # near: staging spots right beside the final pose
    if not m:
        raise SystemExit('ABORT: no face found ahead')
    log('staging measure', pub(m))
    m = align_heading(m, dry)
    ref_lat = None                              # sideways to the taught pose from the reference scan (if trusted)
    if (cmd == 'dock' and not dry and '--here' not in sys.argv and 'pose' in st
            and os.path.exists(os.path.join(REF_DIR, name + '.npy'))):
        face_dy = lat_error(m, st, gap)
        try:
            sd = rel_target(st['pose'])                              # map: good to a few cm here
            rx, ry, ra = ref_offset(name, seed=sd)
            fwd0 = m['dist'] - FRONT - gap
            # the map is good to ~5 cm here; the face is not (it can be another surface, and the taught pose need
            # not sit at the face gap): reference and map must agree
            if abs(rx - sd[0]) <= 0.10 and abs(ry - sd[1]) <= 0.10:
                LAT_BIAS[0] = ry - face_dy; ref_lat = ry
                log('lateral from the reference scan: %.3f m (face edge: %.3f, map: %.3f) - using the reference'
                    % (ry, face_dy, sd[1]))
            else:
                log('reference match at the safe point not consistent with the map (x %.3f vs %.3f, y %.3f vs %.3f; '
                    'face x %.3f) - lateral from the face edge' % (rx, sd[0], ry, sd[1], fwd0))
        except SystemExit as e:
            log('reference match at the safe point failed (%s) - lateral from the face edge' % e)
    if jittered and '--lineup' not in sys.argv:
        # from the jittered spot the approach may be blocked where it is clear from the taught one: check
        # (no motion) and fall back to the taught safe point instead of stopping
        try:
            plan_approach(m, st, gap)
        except SystemExit as e:
            unjitter(st, str(e).replace('ABORT: ', '')); jittered = False
            turn_to(math.radians(st.get('pose', st['staging_pose'])[2]))
            m = measure() or measure(near=True)
            if not m:
                raise SystemExit('ABORT: no face found ahead')
            m = align_heading(m, dry)
    if '--lineup' in sys.argv and ref_lat is not None:
        # the face edge is not trusted here (see LAT_BIAS): line up with the reference scan, one sideways move
        fwd0 = m['dist'] - FRONT - gap
        sideways_small(ref_lat, st, fwd0)                       # checked against the live cloud
        sd = rel_target(st['pose'])
        try:
            rx, ry, ra = ref_offset(name, seed=sd)
            log('lined up by the reference scan: still x %.3f y %.3f yaw %.2f deg to the taught pose'
                % (rx, ry, math.degrees(ra)))
        except SystemExit as e:
            log('after lining up: %s' % e)
        log('STAGE-ONLY: lined up in front of %s (reference scan), %.3f m before the final pose (not driven in)'
            % (name, m['dist'] - FRONT - gap))
        setmode(0)
        raise SystemExit(0)
    if '--stage-only' in sys.argv:                            # e.g. place pose not taught yet: stop at staging
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
    has_ref = os.path.exists(os.path.join(REF_DIR, name + '.npy')) and not dry
    if '--closest' in sys.argv:
        # arm motion not fixed yet: go to the requested gap, or as close as the robot + arms + tray stay clear
        pw = RM.remove_self(cloud(), CFG); dy = lat_error(m, st, gap)
        if SURF[0] is None:                   # from the farthest view (see plan_approach); surface_filter needs it
            SURF[0] = RM.surface_height(pw, m['dist'])
        if not has_ref:
            need_clear(RM.sweep_hits(pw, CFG, 0.0, dy), 'sideways move to the taught position')
        f0 = m['dist'] - FRONT - gap
        pw = accepted_filter(pw, st, f0, dy)
        ok = RM.max_forward(pw - np.array([0.0, dy, 0.0]), CFG, max(f0, 0.0))
        if ok is None:
            raise SystemExit('ABORT: already touching something ahead after the sideways move')
        if ok < f0 - 0.005:
            ok = max(0.0, ok - CLOSEST_SLACK)      # never plan exactly at the limit: the sideways move shifts things
            why = RM.sweep_hits(pw - np.array([0.0, dy, 0.0]), CFG, f0, 0.0)
            log('requested gap %.3f is blocked by %s -> will stop at gap %.3f (closest clear)'
                % (gap, json.dumps(why), m['dist'] - FRONT - ok))
            gap = m['dist'] - FRONT - ok
    if '--lineup' in sys.argv:                                 # only the sideways move must be clear here
        need_clear(RM.sweep_hits(RM.remove_self(cloud(), CFG), CFG, 0.0, lat_error(m, st, gap)),
                   'sideways move to the taught position')
    else:
        plan_approach(m, st, gap, sideways=not has_ref)        # stops here if the taught approach is blocked
    if os.path.exists(os.path.join(REF_DIR, name + '.npy')) and not dry and '--lineup' not in sys.argv:
        # Fast path with a reference scan, NO pure sideways moves: turn / straight / turn back to STAGE2 before the
        # final pose on the station line, re-measure, then the last stage the same way with the lateral from the scan
        # match (on its own the chassis ignores sideways moves below ~2 cm - measured at D)
        dy = lat_error(m, st, gap); fwd = m['dist'] - FRONT - gap; total = 0.0
        if fwd < 0 or fwd > MAX_FWD:
            raise SystemExit('ABORT: forward %.3f out of range' % fwd)
        if fwd > STAGE2 + 0.15:
            first = fwd - STAGE2
            p = accepted_filter(RM.remove_self(cloud(), CFG), st, fwd, dy)
            if abs(dy) > SLANT_TOL:
                # no sideways move (2026-11-16, user): turn towards the stage-2 point on the line, straight, turn back
                a, d = slant_plan(dy, first, p, 'approach to stage 2')
                slant_move(a, d, FWD_SHORT, 'approach to stage 2')
            else:
                need_clear(RM.sweep_hits(p, CFG, first, 0.0), 'straight approach')
                log('approach: forward %.3f m (sideways %.3f m left alone)' % (first, dy))
                rel(first + FWD_SHORT, 0.0)
            total += first + FWD_SHORT
            m = measure_station(st, gap, m['dist'] - first)
            log('  stage 2 measure', pub(m))
        # from here on the sideways position comes from the reference scan only: close to D the face end jumps
        # between pieces (2026-11-16: -0.06 m at the safe point, +0.20 m at stage 2 -> a planned 0.47 m sideways
        # move, stopped by the clearance check)
        # seeded with the map: without the first stage (short approach, e.g. a gap cut back by --closest) the
        # robot is still ~0.4 m out and an unseeded match lands in a wrong minimum (2026-11-16: 9% matched)
        seed = rel_target(st['pose']) if 'pose' in st else None
        rx, ry, ra = ref_offset(name, seed=seed)
        plan_approach(m, st, gap, dy=ry, sideways=False)
        if abs(ra) > math.radians(1.0):
            need_clear(RM.turn_hits(accepted_filter(RM.remove_self(cloud(), CFG), st, rx, ry), CFG, ra), 'heading correction')
            rel(0.0, ra)
            m = measure_station(st, gap, m['dist']); rx, ry, ra = ref_offset(name)
        fwd = m['dist'] - FRONT - gap
        slant = None
        if abs(ry) > SLANT_TOL:
            try:
                slant = slant_plan(ry, fwd, accepted_filter(RM.remove_self(cloud(), CFG), st, fwd, ry), 'last stage')
            except SystemExit as e:
                log('last stage: slanted path not clear (%s) - sideways correction instead' % e)
                sideways_small(ry, st, fwd)
        elif abs(ry) >= 0.008:
            log('last stage: sideways %.3f m left alone (<= %.3f)' % (ry, SLANT_TOL))
        if slant:
            slant_move(slant[0], slant[1], FWD_SHORT_LAST, 'last stage')
        else:
            need_clear(RM.sweep_hits(accepted_filter(RM.remove_self(cloud(), CFG), st, fwd, 0.0), CFG, fwd, 0.0), 'last stage')
            log('last stage: forward %.3f m' % fwd)
            rel(fwd + FWD_SHORT_LAST, 0.0)
        total += fwd + FWD_SHORT_LAST
        total += final_forward_fix(name, st)
        YLIM[0] = 2.5; f = measure(near=True)
        res = dict(name=name, gap_target=gap, gap=f['gap'] if f else None, ang_deg=f['ang_deg'] if f else None, fwd=round(fwd, 4))
        try:
            rx, ry, ra = ref_offset(name)
            res.update(ref_err_x=round(rx, 4), ref_err_y=round(ry, 4), ref_err_yaw_deg=round(math.degrees(ra), 2))
        except SystemExit as e:
            res.update(ref_err=str(e))
        log('FINAL', json.dumps(res))
        json.dump(dict(fwd=total, gap=gap, t=time.time()), open('/tmp/claude_dock_last.json', 'w'))
        raise SystemExit(0)
    m = align_lateral(m, st, gap, dry)
    m = align_heading(m, dry)
    m = align_lateral(m, st, gap, dry)                         # again: the drift term depends on the final heading
    fwd = m['dist'] - FRONT - gap
    if '--lineup' in sys.argv:
        # lined up on the approach line, not driven in: e.g. to run "accept NAME" here
        log('STAGE-ONLY: lined up in front of %s, %.3f m before the final pose (not driven in)' % (name, fwd))
        if not dry:
            setmode(0)
        raise SystemExit(0)
    log('forward %.3f m (target gap %.3f)' % (fwd, gap))
    if fwd < 0 or fwd > MAX_FWD:
        raise SystemExit('ABORT: forward %.3f out of range' % fwd)
    if '--closest' in sys.argv:
        # re-evaluate with the fresh cloud after the sideways/heading corrections; back off if needed
        ok = RM.max_forward(accepted_filter(RM.remove_self(cloud(), CFG), st, fwd), CFG, max(fwd, 0.0))
        if ok is None:
            raise SystemExit('ABORT: already touching something ahead after aligning')
        if ok < fwd - 0.002:
            ok = max(0.0, ok - CLOSEST_SLACK)
            log('after aligning the requested gap is blocked -> forward %.3f m instead of %.3f (gap %.3f)'
                % (ok, fwd, m['dist'] - FRONT - ok))
            fwd = ok; gap = m['dist'] - FRONT - ok
    plan_approach(m, st, gap)                                  # re-check right before driving in
    if not dry:
        total = 0.0
        if fwd > STAGE2 + 0.15:
            # long straight drives drift sideways (~2 cm over 1.1 m, measured): stop STAGE2 short, measure again,
            # correct sideways / heading (re-checked against the live cloud), then drive the last part
            first = fwd - STAGE2
            rel(first + FWD_SHORT, 0.0); total += first + FWD_SHORT
            m = measure_station(st, gap, m['dist'] - first)
            log('  stage 2 measure', pub(m))
            plan_approach(m, st, gap)
            if os.path.exists(os.path.join(REF_DIR, name + '.npy')):
                # close to the edge its end flickers by ~5 cm (measured at D): sideways / heading from the
                # reference scan of the final pose instead (all surrounding structure, ~1 cm)
                for _ in range(3):
                    rx, ry, ra = ref_offset(name)
                    if abs(ra) > math.radians(1.0):
                        need_clear(RM.turn_hits(accepted_filter(RM.remove_self(cloud(), CFG), st, rx, ry), CFG, ra),
                                   'heading correction'); rel(0.0, ra); continue
                    if abs(ry) <= REF_TOL:
                        break
                    need_clear(RM.sweep_hits(accepted_filter(RM.remove_self(cloud(), CFG), st, rx, ry), CFG, 0.0, ry),
                               'sideways correction'); rel(0.0, 0.0, y=ry)
                m = measure_station(st, gap, m['dist'])
            else:
                m = align_lateral(m, st, gap)
                m = align_heading(m)
                m = align_lateral(m, st, gap)
            fwd = m['dist'] - FRONT - gap
            log('forward %.3f m (last stage)' % fwd)
            plan_approach(m, st, gap)
        short = FWD_SHORT if fwd > STAGE2 + 0.15 else FWD_SHORT_LAST
        rel(fwd + short, 0.0); total += fwd + short
        YLIM[0] = 2.5; f = measure(near=True)
        res = dict(name=name, gap_target=gap, gap=f['gap'], ang_deg=f['ang_deg'], lat_err=round(lat_error(f, st, f['gap']), 4), fwd=round(fwd, 4))
        if os.path.exists(os.path.join(REF_DIR, name + '.npy')):
            try:
                rx, ry, ra = ref_offset(name)
                res.update(ref_err_x=round(rx, 4), ref_err_y=round(ry, 4), ref_err_yaw_deg=round(math.degrees(ra), 2))
            except SystemExit as e:
                res.update(ref_err=str(e))
        log('FINAL', json.dumps(res))
        json.dump(dict(fwd=total, gap=gap, t=time.time()), open('/tmp/claude_dock_last.json', 'w'))
        new = not __import__('os').path.exists(LOG_FILE)
        with open(LOG_FILE, 'a') as fh:
            if new:
                fh.write('time,name,gap_target,gap,ang_deg,lat_err,fwd' + chr(10))
            fh.write('%s,%s,%.3f,%.4f,%.2f,%.4f,%.4f' %  (time.strftime('%H:%M:%S'), name, gap, f['gap'], f['ang_deg'], res['lat_err'], fwd) + chr(10))
elif cmd == 'bench':
    # bench [NAME] : NO MOTION.  Planning time on ONE lidar snapshot (taken first, not counted): every check and
    # computation the docking runs between receiving data and issuing a command.  Requirement: < PLAN_LIMIT_MS.
    name = args[0] if args else 'D'; st = load_st()[name]; YLIM[0] = 2.5; ZB[0], ZB[1] = st['z']
    YAW_OFF[0] = math.radians(float(st.get('yaw_offset', 0.0)))
    st['end_y'] = st.get('end_y', 0.0) - float(st.get('lat_offset', 0.0))
    has_ref = os.path.exists(os.path.join(REF_DIR, name + '.npy'))
    if has_ref:
        t = time.time(); ref_data(name); log('prepare reference (start of a docking, not timed): %.0f ms' % (1000 * (time.time() - t)))
    _FROZEN[0] = cloud(fresh=True); log('snapshot: %d points' % len(_FROZEN[0]))
    res = []

    def tm(label, f):
        t = time.time()
        try:
            r = f()
        except SystemExit as e:
            r = 'stops: %s' % e
        ms = 1000 * (time.time() - t); res.append((label, ms)); log('%-44s %7.1f ms   %s' % (label, ms, str(r)[:90]))
        return r
    tm('turn in place clear? (rotation_hits)', lambda: RM.rotation_hits(cloud(), CFG))
    tm('move to a turning spot (leave_move)', lambda: RM.leave_move(cloud(), CFG, 0.6))
    tm('turn 45 deg (turn_hits)', lambda: RM.turn_hits(cloud(), CFG, math.radians(45)))
    tm('straight back 0.6 m (sweep_hits)', lambda: RM.sweep_hits(cloud(), CFG, -0.6, 0.0))
    tm('sideways 0.5 m (sweep_hits)', lambda: RM.sweep_hits(cloud(), CFG, 0.0, 0.5))
    m = tm('face fit (measure)', lambda: measure() or measure(near=True))
    if isinstance(m, dict) and st.get('mode') != 'map':
        tm('approach plan (plan_approach)', lambda: plan_approach(m, st, st['gap']))
    if has_ref:
        tm('reference scan match (ref_offset)', lambda: ref_offset(name))
    worst = max(ms for _, ms in res)
    log('BENCH %s: slowest step %.0f ms, all steps together %.0f ms (limit %d ms per planning step)'
        % (name, worst, sum(ms for _, ms in res), PLAN_LIMIT_MS))
    _FROZEN[0] = None
elif cmd == 'undock':
    # undock [MIN] : reverse straight out along the last docking approach (at least MIN m, default 0.30)
    d = json.load(open('/tmp/claude_dock_last.json'))['fwd']; assert 0 <= d <= MAX_FWD + 0.05
    d = max(d, float(args[0]) if args else 0.30)
    MOTION_JOB[0] = True
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
    YAW_OFF[0] = math.radians(float(st.get('yaw_offset', 0.0))); st['end_y'] = st['end_y'] - float(st.get('lat_offset', 0.0))
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

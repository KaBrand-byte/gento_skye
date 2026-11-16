"""Route variation for the demo (no ROS here: pure geometry, tested offline in chassis/tests/test_route.py).

The vendor navigation plans the same shortest path every time.  Two ways to make trips differ:
  1. waypoints  - points taught by hand (waypoints.json, `teach-waypoint NAME`); every trip picks with EQUAL
                  probability one of the qualifying waypoints or the direct route (2 waypoints: 1/3 each).  This
                  is the default of every goto (--via none switches it off).  Only waypoints that do not make the
                  trip much longer and are not right next to the start or the destination qualify.
  2. jitter     - the vendor navigation drives to a random spot within JITTER m of the taught safe point; the
                  final docking is unchanged (lidar / reference scan).  For lidar (face) stations the spot stays
                  inside the envelope from which the docking works (behind the final pose, within reach).
Everything here only PICKS targets; all motion and every clearance check stays in dock.py.
"""
import json, math, os, random

WAYPOINT_FILE = 'waypoints.json'
VIA_MAX_DETOUR = 1.5     # via-path length <= direct * this + VIA_EXTRA
VIA_EXTRA = 0.8         # m
VIA_MIN_DIST = 0.8       # m: a waypoint this close to the start or the destination adds nothing
JITTER = 0.08            # m: default radius around the taught safe point


def load_waypoints(path):
    try:
        w = json.load(open(path))
        return {k: v for k, v in w.items() if isinstance(v, dict) and 'pose' in v}
    except Exception:
        return {}


def save_waypoints(path, w):
    tmp = path + '.tmp'
    json.dump(w, open(tmp, 'w'), indent=1); os.replace(tmp, path)


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def eligible(start, dest, waypoints):
    """Waypoint names that make sense between start and dest (map x, y), sorted by name."""
    direct = _d(start, dest); out = []
    for k, v in sorted(waypoints.items()):
        p = v['pose']
        if _d(p, start) < VIA_MIN_DIST or _d(p, dest) < VIA_MIN_DIST:
            continue
        if _d(start, p) + _d(p, dest) > direct * VIA_MAX_DETOUR + VIA_EXTRA:
            continue
        out.append(k)
    return out


def choose_via(start, dest, waypoints, mode, rng=random):
    """mode: 'none' | 'random' | a waypoint name.  Returns (name or None, reason)."""
    if not mode or mode == 'none':
        return None, 'direct (no waypoint requested)'
    if mode != 'random':
        if mode not in waypoints:
            raise ValueError('unknown waypoint %s (known: %s)' % (mode, ', '.join(sorted(waypoints)) or 'none'))
        return mode, 'requested'
    ok = eligible(start, dest, waypoints)
    if not ok:
        return None, 'direct (no suitable waypoint between start and destination)'
    pick = rng.choice(ok + [None])                    # each waypoint and the direct route equally likely
    return pick, 'random choice among %s and direct' % ', '.join(ok)


def in_face_envelope(xy, st, front, max_fwd, lat_max, max_dist=2.1):
    """For a lidar (face) station: may the docking start from map point xy?  Same limits as teach-staging:
    behind the final pose by 0..max_fwd, at most lat_max to the side, face within max_dist ahead."""
    fx, fy, fh = st['pose']; h = math.radians(fh)
    c, s = math.cos(-h), math.sin(-h)
    xr, yr = c * (xy[0] - fx) - s * (xy[1] - fy), s * (xy[0] - fx) + c * (xy[1] - fy)
    behind = -xr
    return 0.0 <= behind <= max_fwd and abs(yr) <= lat_max and behind + st.get('gap', 0.0) + front <= max_dist


def jitter_pose(base, st, radius, rng=random, front=0.338, max_fwd=1.20, lat_max=1.40, map_max=2.5):
    """A random point within `radius` of the taught safe point `base` [x, y, yaw_deg] (uniform over the disc),
    kept where the station's docking still works from; `base` itself when radius <= 0 or nothing fits."""
    if radius <= 0:
        return list(base)
    face = st.get('mode') != 'map' and 'pose' in st
    for _ in range(30):
        a = rng.uniform(-math.pi, math.pi); r = radius * math.sqrt(rng.random())
        p = [round(base[0] + r * math.cos(a), 3), round(base[1] + r * math.sin(a), 3), base[2]]
        if face and not in_face_envelope(p, st, front, max_fwd, lat_max):
            continue
        if st.get('mode') == 'map' and 'pose' in st and _d(p, st['pose']) > map_max:
            continue
        return p
    return list(base)

#!/usr/bin/env python3
"""Collision model of Gento carrying a tray (chassis + torso + arms + tray), body frame (x fwd, y left, z up, m).

Task (demo): the robot starts at the table edge already holding the tray, drives to the place position at the
conveyor and the (hard-coded) arm motion puts the tray down.  The tray is carried at >= 1.2 m so it clears the
table (0.9 m) and the belt (0.84 m).  Note: that height does NOT clear frames above the belt (e.g. the glass
tunnel, 0.9-1.52 m) - the place position must be taught so the tray footprint is beside such frames.

Only the ROBOT's geometry lives here; obstacles always come from the live 3-D lidar cloud (base_link frame).
The robot never shifts its docking pose by itself: the arm motion is hard-coded, so the base must arrive at the
taught pose or stop.

Geometry (vendor three-view, mm): chassis 676.7 x 547.1, centre -> front 338; shoulders 461 apart, ~49 ahead of
centre; upper arm 287, forearm 314, wrist+gripper 165; lift: shoulder ~0.78 .. 1.69 m.

Config robot_model.json next to this file (optional, overrides DEFAULT):
  shoulder_z   shoulder height while carrying (m)
  tray         {"front": tray front edge beyond chassis front, "depth", "width", "z_bottom", "height"}
  phases.carry list of extra boxes [x0,x1,y0,y1,z0,z1] for arms (z may be "s+0.1" = relative to shoulder_z)
  `dock.py selfscan --save` measures arms+tray with the lidar and stores them as phases.carry.
"""
import json, math, os
import numpy as np

CFG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot_model.json')

FRONT = 0.338
HALF_W = 0.274

DEFAULT = {
    'shoulder_z': 1.45,                       # tray >= 1.2 m means the forearms are at ~1.2 m -> shoulder ~1.45 m
    'tray': {'front': 0.45, 'depth': 0.35, 'width': 0.45, 'z_bottom': 1.20, 'height': 0.10},
    'phases': {
        'carry': [[0.00, 0.70, -0.44, 0.44, 's-0.35', 's+0.12']],   # arms holding the tray (measured width +-0.41)
    },
    'margin': 0.05,           # clearance for turning (the arms/tray swing out)
    'margin_straight': 0.03,  # clearance for slow straight moves (lidar accuracy ~1 cm)
    # the (hard-coded) place motion after docking: forward, down, then each hand sideways out (skye_task
    # forward_mm / down_mm / left_exit_mm / right_exit_mm, in metres)
    'place_motion': {'forward': 0.10, 'down': 0.10, 'side': 0.10},
}


def load():
    cfg = json.loads(json.dumps(DEFAULT))
    try:
        user = json.load(open(CFG_FILE))
        for k, v in user.items():
            if k == 'phases':
                cfg['phases'].update(v)
            elif k == 'tray':
                cfg['tray'].update(v)
            else:
                cfg[k] = v
    except Exception:
        pass
    return cfg


def _z(v, sz):
    if isinstance(v, str) and v.startswith('s'):
        return sz + float(v[1:])
    return float(v)


def tray_box(cfg):
    t = cfg['tray']; x1 = FRONT + t['front']
    return [x1 - t['depth'], x1, -t['width'] / 2, t['width'] / 2, t['z_bottom'], t['z_bottom'] + t['height']]


def boxes(cfg, phase='carry'):
    sz = float(cfg['shoulder_z'])
    out = [('chassis', [-FRONT, FRONT, -HALF_W, HALF_W, 0.02, 0.45]),
           ('torso', [-0.30, 0.18, -0.29, 0.29, 0.45, sz + 0.39]),
           ('tray', tray_box(cfg))]
    for b in cfg['phases'].get(phase, []):
        z0, z1 = _z(b[4], sz), _z(b[5], sz)
        out.append(('arms', [b[0], b[1], b[2], b[3], z0, z1]))
        # upper arms / shoulders beside the torso (the lidar cannot see them well): same width and heights,
        # from behind the shoulder axis to where the measured forearms start
        out.append(('upper_arms', [-0.15, max(b[0], 0.0), b[2], b[3], z0, max(z1, sz + 0.12)]))
    return out


def _inside(p, b, m):
    return ((p[:, 0] > b[0] - m) & (p[:, 0] < b[1] + m) & (p[:, 1] > b[2] - m) & (p[:, 1] < b[3] + m)
            & (p[:, 2] > b[4] - m) & (p[:, 2] < b[5] + m))


def remove_self(p, cfg, phase='carry'):
    """Drop lidar points that are the robot itself (inside its own boxes at the current pose) and the floor."""
    keep = np.ones(len(p), bool)
    for _, b in boxes(cfg, phase):
        keep &= ~_inside(p, b, cfg['margin'] + 0.02)   # must exceed the collision margin, or the robot "hits itself"
    return p[keep & (p[:, 2] > 0.04)]


def _report(p, sel, what):
    hits = p[sel]
    if len(hits) < 10:                          # isolated returns are noise
        return None
    i = np.argmin(np.hypot(hits[:, 0], hits[:, 1]))
    return dict(what=what, n=int(len(hits)), x=round(float(hits[i, 0]), 3), y=round(float(hits[i, 1]), 3),
                z=round(float(hits[i, 2]), 3))


def rotation_hits(p, cfg, phase='carry'):
    """Turning in place sweeps each box through a circle: any obstacle inside that circle, in that box's
    height band, is a collision.  None when clear, else the nearest offending point."""
    m = cfg['margin']; p = remove_self(p, cfg, phase); r = np.hypot(p[:, 0], p[:, 1]); worst = None
    for name, b in boxes(cfg, phase):
        R = max(math.hypot(x, y) for x in (b[0], b[1]) for y in (b[2], b[3])) + m
        h = _report(p, (r < R) & (p[:, 2] > b[4] - m) & (p[:, 2] < b[5] + m), 'rotate:' + name)
        if h and (worst is None or math.hypot(h['x'], h['y']) < math.hypot(worst['x'], worst['y'])):
            worst = h
    return worst


NOISE = 10        # fewer offending lidar returns than this are treated as noise (same rule as _report)


def _swept(b, dx, dy):
    """Exact volume swept by an axis-aligned box translated along x or y (no rotation)."""
    return [min(b[0], b[0] + dx), max(b[1], b[1] + dx), min(b[2], b[2] + dy), max(b[3], b[3] + dy), b[4], b[5]]


def sweep_hits(p, cfg, dx, dy=0.0, phase='carry'):
    """Straight move by (dx, dy) without turning.  Pure forward/back or sideways moves are checked exactly in
    one pass (the swept volume of a box is a longer box); diagonal moves are sampled every 2 cm."""
    m = cfg.get('margin_straight', 0.03); p = remove_self(p, cfg, phase); hit = np.zeros(len(p), bool)
    if abs(dx) < 1e-9 or abs(dy) < 1e-9:
        for _, b in boxes(cfg, phase):
            hit |= _inside(p, _swept(b, dx, dy), m)
    else:
        k = max(1, int(math.hypot(dx, dy) / 0.02))
        for i in range(k + 1):
            q = p - np.array([dx * i / k, dy * i / k, 0.0])
            for _, b in boxes(cfg, phase):
                hit |= _inside(q, b, m)
    return _report(p, hit, 'sweep dx=%.2f dy=%.2f' % (dx, dy))


def _kth(values, k=NOISE):
    """k-th smallest value (noise-robust minimum); +inf when fewer than k values."""
    if len(values) < k:
        return float('inf')
    return float(np.partition(values, k - 1)[k - 1])


def max_forward(p, cfg, limit, phase='carry'):
    """Largest straight forward distance <= limit that the robot (all boxes) can drive without contact."""
    m = cfg.get('margin_straight', 0.03); p = remove_self(p, cfg, phase); best = float(limit)
    for _, b in boxes(cfg, phase):
        band = (p[:, 1] > b[2] - m) & (p[:, 1] < b[3] + m) & (p[:, 2] > b[4] - m) & (p[:, 2] < b[5] + m) & (p[:, 0] > b[0] - m)
        best = min(best, _kth(p[band, 0] - b[1] - m))
    return max(best, 0.0) if best > -1e-9 else None      # None: already touching something ahead


def _free_points(intervals, lo, hi, k=NOISE):
    """Allowed sub-ranges of [lo, hi]: places covered by fewer than k of the blocked intervals [start, end]."""
    if len(intervals) == 0:
        return [(lo, hi)]
    starts, ends = np.sort(intervals[:, 0]), np.sort(intervals[:, 1])
    cuts = np.unique(np.concatenate([[lo, hi], starts[(starts > lo) & (starts < hi)], ends[(ends > lo) & (ends < hi)]]))
    mids = (cuts[:-1] + cuts[1:]) / 2
    cover = np.searchsorted(starts, mids, side='right') - np.searchsorted(ends, mids, side='left')
    free = []
    for a, b, ok in zip(cuts[:-1], cuts[1:], cover < k):
        if not ok:
            continue
        if free and abs(free[-1][1] - a) < 1e-12:
            free[-1] = (free[-1][0], float(b))
        else:
            free.append((float(a), float(b)))
    return free


def max_place_forward(p, cfg, limit, surface_z, clear=0.02):
    """Largest forward distance <= limit for which the place motion run afterwards (forward, down, hands out)
    is clear; None if there is none.  The surface itself (up to surface_z + clear) is expected."""
    pm = cfg['place_motion']; m = cfg.get('margin_straight', 0.03); p = remove_self(p, cfg); iv = []
    for name, b in boxes(cfg):
        if name not in ('arms', 'tray'):
            continue
        env = [b[0], b[1] + pm['forward'], b[2] - pm['side'], b[3] + pm['side'], b[4] - pm['down'], b[5]]
        floor = max(env[4], (surface_z + clear) if surface_z is not None else env[4])
        band = (p[:, 1] > env[2] - m) & (p[:, 1] < env[3] + m) & (p[:, 2] > floor) & (p[:, 2] < env[5] + m)
        x = p[band, 0]
        iv.append(np.stack([x - env[1] - m, x - env[0] + m], 1))    # f values for which the point is inside
    iv = np.concatenate(iv) if iv else np.zeros((0, 2))
    free = _free_points(iv, 0.0, float(limit))
    return max(f[1] for f in free) if free else None


def place_hits(p, cfg, dx, surface_z, clear=0.02):
    """At the docked pose (after moving dx forward) the tray is lowered straight down onto the surface.
    The column under the tray footprint, from just above the surface up to the tray, must be empty
    (no item already on the belt, no frame, no hand)."""
    m = cfg['margin']; p = remove_self(p, cfg); q = p - np.array([dx, 0.0, 0.0]); t = tray_box(cfg)
    sel = ((q[:, 0] > t[0] - m) & (q[:, 0] < t[1] + m) & (q[:, 1] > t[2] - m) & (q[:, 1] < t[3] + m)
           & (q[:, 2] > surface_z + clear) & (q[:, 2] < t[4]))      # no margin below: the surface itself is expected
    return _report(p, sel, 'place spot (surface %.2f m)' % surface_z)


def place_motion_hits(p, cfg, dx, surface_z, clear=0.02):
    """At the docked pose (after moving dx forward) the arms + tray run the place motion: forward, down, and
    the hands out to the sides.  The space this sweeps must be free, except the work surface itself (anything
    standing more than `clear` above it - a frame over the belt, an item, a hand - counts)."""
    pm = cfg['place_motion']; m = cfg.get('margin_straight', 0.03)
    p = remove_self(p, cfg); q = p - np.array([dx, 0.0, 0.0])
    hit = np.zeros(len(p), bool)
    for name, b in boxes(cfg):
        if name not in ('arms', 'tray'):
            continue
        env = [b[0], b[1] + pm['forward'], b[2] - pm['side'], b[3] + pm['side'], b[4] - pm['down'], b[5]]
        floor = max(env[4], (surface_z + clear) if surface_z is not None else env[4])
        hit |= ((q[:, 0] > env[0] - m) & (q[:, 0] < env[1] + m) & (q[:, 1] > env[2] - m) & (q[:, 1] < env[3] + m)
                & (q[:, 2] > floor) & (q[:, 2] < env[5] + m))
    return _report(p, hit, 'place motion (forward %.2f, down %.2f, sides %.2f m)' % (pm['forward'], pm['down'], pm['side']))


def surface_height(p, face_dist, depth=0.35):
    """Top of the table / belt just behind the measured face: the densest 2 cm layer."""
    sel = (p[:, 0] > face_dist + 0.02) & (p[:, 0] < face_dist + depth) & (np.abs(p[:, 1]) < 0.6) & (p[:, 2] > 0.3) & (p[:, 2] < 1.3)
    z = p[sel, 2]
    if len(z) < 30:
        return None
    hist, edges = np.histogram(z, bins=np.arange(0.3, 1.31, 0.02))
    dense = np.where(hist >= 0.25 * hist.max())[0]           # the TOP surface is the highest dense layer
    return float(edges[dense.max()] + 0.02)


def _turn_clear_distance(q, R, u, max_d):
    """Smallest d in [0, max_d] such that every point of q (2-D, robot frame) is farther than R from d*u
    (noise rule applied); None if none."""
    t = q @ u; perp2 = (q ** 2).sum(1) - t ** 2
    close = perp2 < R * R
    s = np.sqrt(np.maximum(R * R - perp2[close], 0.0))
    iv = np.stack([t[close] - s, t[close] + s], 1)
    free = _free_points(iv, 0.0, float(max_d))
    return free[0][0] if free else None


def leave_move(p, cfg, max_d=0.6, phase='carry'):
    """Shortest straight move (reverse, sideways or diagonal-back; no turning) that is itself collision-free
    and after which turning in place is clear - computed exactly, not by trial.  Returns (dx, dy) in the body
    frame, (0, 0) if turning is already clear, None if nothing within max_d works."""
    if not rotation_hits(p, cfg, phase):
        return (0.0, 0.0)
    p = remove_self(p, cfg, phase); m = cfg['margin']; k = math.sqrt(0.5); best = None
    for ux, uy in ((-1.0, 0.0), (0.0, -1.0), (0.0, 1.0), (-k, -k), (-k, k)):
        u = np.array([ux, uy]); need = 0.0
        for _, b in boxes(cfg, phase):
            R = max(math.hypot(x, y) for x in (b[0], b[1]) for y in (b[2], b[3])) + m
            band = (p[:, 2] > b[4] - m) & (p[:, 2] < b[5] + m)
            d = _turn_clear_distance(p[band, :2], R, u, max_d)
            if d is None:
                need = None; break
            need = max(need, d)
        if need is None:
            continue
        need = min(max_d, need + 0.02)                  # 2 cm extra: moves stop a little short
        # all boxes together may need slightly more than the per-box minimum: verify, then extend if needed
        for _ in range(5):
            if not rotation_hits(p - np.array([ux * need, uy * need, 0.0]), cfg, phase):
                break
            need += 0.03
        else:
            continue
        if need > max_d or sweep_hits(p, cfg, ux * need, uy * need, phase):
            continue
        if best is None or need < best[0]:
            best = (need, (float(ux * need), float(uy * need)))
    return best[1] if best else None


def leave_distance(p, cfg, max_back=0.8, phase='carry'):
    """Shortest straight reverse after which turning in place is clear (kept for callers of the old API)."""
    if not rotation_hits(p, cfg, phase):
        return 0.0
    mv = leave_move(p, cfg, max_back, phase)
    return -mv[0] if mv and abs(mv[1]) < 1e-9 else None


def selfscan(p, face_min=0.95):
    """With nothing else within ~1 m in front, points above the chassis and ahead of the torso are the robot's
    own arms (and tray): report their extent."""
    sel = (p[:, 0] > 0.0) & (p[:, 0] < min(face_min, 1.2)) & (np.abs(p[:, 1]) < 0.6) & (p[:, 2] > 0.5) & (p[:, 2] < 2.2)
    s = p[sel]
    if len(s) < 20:
        return None
    x = [float(np.percentile(s[:, 0], 0.5)), float(np.percentile(s[:, 0], 99.5))]
    y = [float(np.percentile(s[:, 1], 0.5)), float(np.percentile(s[:, 1], 99.5))]
    z = [float(np.percentile(s[:, 2], 1)), float(np.percentile(s[:, 2], 99))]
    hw = max(abs(y[0]), abs(y[1]))          # one arm is usually hidden from the lidar: mirror the wider side
    box = [round(max(0.0, x[0] - 0.02), 3), round(x[1] + 0.03, 3), round(-hw - 0.03, 3), round(hw + 0.03, 3),
           round(z[0] - 0.03, 3), round(z[1] + 0.05, 3)]
    return dict(n=int(len(s)), x=[round(v, 3) for v in x], y=[round(v, 3) for v in y], z=[round(v, 3) for v in z],
                carry_box=box)


def save_selfscan(res):
    """Store the measured arms(+tray) envelope as phases.carry (absolute heights)."""
    try:
        user = json.load(open(CFG_FILE))
    except Exception:
        user = {}
    user.setdefault('phases', {})['carry'] = [res['carry_box']]
    user['carry_measured'] = {k: res[k] for k in ('n', 'x', 'y', 'z')}
    json.dump(user, open(CFG_FILE, 'w'), indent=1)
    return user

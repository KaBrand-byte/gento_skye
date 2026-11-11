"""Offline check of robot_model.py on synthetic clouds built from the measured room geometry. No robot needed."""
import math, sys
import numpy as np
sys.path.insert(0, '.')
import robot_model as RM

cfg = RM.load(); rng = np.random.default_rng(0)
print('tray box', RM.tray_box(cfg), '| boxes', [n for n, _ in RM.boxes(cfg)])


def block(x0, x1, y0, y1, z0, z1, n):
    return np.stack([rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)], 1)


# ---- 1. docked at the table, tray over the table (front gap 0.24 m) ----
f = RM.FRONT + 0.24                                   # table front edge, body x
table = np.concatenate([block(f, f + 0.70, -1.0, 1.0, 0.88, 0.90, 4000),        # top
                        block(f, f + 0.03, -1.0, 1.0, 0.75, 0.90, 800),          # apron
                        block(f, f + 0.05, -1.0, -0.95, 0.0, 0.9, 200), block(f, f + 0.05, 0.95, 1.0, 0.0, 0.9, 200),
                        block(f + 0.65, f + 0.68, -1.0, 1.0, 0.90, 1.75, 4000)])  # pegboard at the back
print('\n[table start] turn here:', RM.rotation_hits(table, cfg), '| reverse needed before turning:', RM.leave_distance(table, cfg))

# ---- 2. conveyor, robot at the old D1 staging (2.75,-2.31, 170.8 deg) ----
x0, y0, h = 2.75, -2.31, math.radians(170.8)
def body(X, Y):
    dX, dY = X - x0, Y - y0
    return dX * math.cos(h) + dY * math.sin(h), -dX * math.sin(h) + dY * math.cos(h)
pts = []
t = rng.uniform(-0.7, 1.6, 4000); z = rng.uniform(0.1, 0.7, 4000)
X = 1.715 + t * math.cos(math.radians(80.4)); Y = -2.12 + t * math.sin(math.radians(80.4))
bx, by = body(X, Y); pts.append(np.stack([bx, by, z], 1))
t = rng.uniform(-0.7, 1.6, 3000); dp = rng.uniform(0.0, 0.45, 3000)
X = 1.715 + t * math.cos(math.radians(80.4)) - dp * math.cos(math.radians(-9.6)); Y = -2.12 + t * math.sin(math.radians(80.4)) - dp * math.sin(math.radians(-9.6))
bx, by = body(X, Y); pts.append(np.stack([bx, by, np.full(3000, 0.84)], 1))
X = rng.uniform(1.2, 1.8, 3000); Y = rng.uniform(-2.0, -1.45, 3000); Z = rng.uniform(0.9, 1.52, 3000)   # glass tunnel
bx, by = body(X, Y); pts.append(np.stack([bx, by, Z], 1))
p = np.concatenate(pts)
face = 1.05; gap = 0.24; fwd = face - RM.FRONT - gap; sz = RM.surface_height(p, face)
print('\n[conveyor] surface top', sz, '| turn at staging:', RM.rotation_hits(p, cfg))
for dy in (0.0, 0.20, 0.36, 0.50):
    q = p - np.array([0, dy, 0])
    r = RM.sweep_hits(p, cfg, 0.0, dy) or RM.sweep_hits(q, cfg, fwd, 0.0) or RM.place_hits(q, cfg, fwd, sz)
    print('  taught pose %.2f m left of old D: %s' % (dy, r or 'clear'))
q = p - np.array([fwd, 0, 0])
print('[old collision] turn at old D pose:', RM.rotation_hits(q, cfg))

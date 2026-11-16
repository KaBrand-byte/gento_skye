"""Offline tests of the collision model's fast paths (chassis/robot_model.py): the point prefilters in turn_hits
and sweep_hits must give exactly the same answer as checking every point.  Needs numpy only.

Run from the repository root:  python3 -m unittest discover -s chassis/tests -v
"""

import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_model as RM  # noqa: E402


def turn_hits_all_points(p, cfg, yaw, phase='carry', step_deg=0.5):
    m = cfg['margin']; p = RM.remove_self(p, cfg, phase); hit = np.zeros(len(p), bool)
    k = max(1, int(math.ceil(abs(math.degrees(yaw)) / step_deg)))
    for i in range(k + 1):
        a = -yaw * i / k
        c, s = math.cos(a), math.sin(a)
        q = np.stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1], p[:, 2]], 1)
        for _, b in RM.boxes(cfg, phase):
            hit |= RM._inside(q, b, m)
    return RM._report(p, hit, 'turn %.1f deg' % math.degrees(yaw))


def sweep_hits_all_points(p, cfg, dx, dy=0.0, phase='carry'):
    m = cfg.get('margin_straight', 0.03); p = RM.remove_self(p, cfg, phase); hit = np.zeros(len(p), bool)
    if abs(dx) < 1e-9 or abs(dy) < 1e-9:
        for _, b in RM.boxes(cfg, phase):
            hit |= RM._inside(p, RM._swept(b, dx, dy), m)
    else:
        k = max(1, int(math.hypot(dx, dy) / 0.02))
        for i in range(k + 1):
            q = p - np.array([dx * i / k, dy * i / k, 0.0])
            for _, b in RM.boxes(cfg, phase):
                hit |= RM._inside(q, b, m)
    return RM._report(p, hit, 'sweep dx=%.2f dy=%.2f' % (dx, dy))


class PrefilterTests(unittest.TestCase):
    def setUp(self):
        self.cfg = RM.load(); self.rng = np.random.default_rng(7)

    def scene(self):
        # clutter all around the robot at every height, densest close in where the decisions are made
        n = 20000
        r = np.abs(self.rng.normal(0.0, 1.0, n)) + 0.2; a = self.rng.uniform(-math.pi, math.pi, n)
        return np.column_stack([r * np.cos(a), r * np.sin(a), self.rng.uniform(0.0, 2.2, n)])

    def test_turns_match_the_full_check(self):
        for _ in range(8):
            p = self.scene()
            for deg in (2, 10, -30, 45, -120, 180):
                self.assertEqual(RM.turn_hits(p, self.cfg, math.radians(deg)),
                                 turn_hits_all_points(p, self.cfg, math.radians(deg)), deg)

    def test_straight_moves_match_the_full_check(self):
        for _ in range(8):
            p = self.scene()
            for dx, dy in ((0.5, 0.0), (-0.6, 0.0), (0.0, 0.4), (0.0, -0.3), (0.2, 0.15), (-0.1, -0.2)):
                self.assertEqual(RM.sweep_hits(p, self.cfg, dx, dy), sweep_hits_all_points(p, self.cfg, dx, dy), (dx, dy))

    def test_clear_scene_stays_clear(self):
        far = np.column_stack([self.rng.uniform(3, 5, 2000), self.rng.uniform(-1, 1, 2000), self.rng.uniform(0, 2, 2000)])
        self.assertIsNone(RM.turn_hits(far, self.cfg, math.radians(90)))
        self.assertIsNone(RM.sweep_hits(far, self.cfg, 0.5, 0.0))



class RemoveSelfCacheTests(unittest.TestCase):
    def test_cache_returns_the_result_for_that_array_only(self):
        cfg = RM.load(); rng = np.random.default_rng(3)
        a = np.column_stack([rng.uniform(-1, 1, 5000), rng.uniform(-1, 1, 5000), rng.uniform(0, 2, 5000)])
        b = a + np.array([0.3, 0.0, 0.0])
        ra = RM.remove_self(a, cfg)
        self.assertIs(RM.remove_self(a, cfg), ra)                       # same snapshot: computed once
        np.testing.assert_array_equal(RM.remove_self(b, cfg), RM._remove_self(b, cfg))
        np.testing.assert_array_equal(RM.remove_self(a, cfg), RM._remove_self(a, cfg))
        self.assertEqual(len(RM.remove_self(a, cfg, 'carry')), len(ra))


if __name__ == "__main__":
    unittest.main()

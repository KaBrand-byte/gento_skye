"""Offline tests of the route variation (chassis/route.py): waypoint choice and safe-point jitter.

Run from the repository root:  python3 -m unittest discover -s chassis/tests -v
"""

import math
from pathlib import Path
import random
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import route as RT  # noqa: E402

# roughly the real layout: T safe point, D safe point (face station, final pose facing +x... here -170 deg)
T = {"mode": "map", "pose": [1.40, 1.70, 0.0], "staging_pose": [1.905, 1.684, 0.0]}
D = {"pose": [2.70, -1.60, 169.3], "gap": 0.09, "staging_pose": [3.225, -1.691, 169.3]}
WPS = {"W1": {"pose": [2.6, 0.0, 0.0]},      # between T and D, small detour
       "W2": {"pose": [5.5, 0.5, 0.0]},      # far off to the side: too long a detour
       "W3": {"pose": [2.1, 1.3, 0.0]}}      # right next to T's safe point: adds nothing


class WaypointTests(unittest.TestCase):
    def test_only_sensible_waypoints_qualify(self):
        self.assertEqual(RT.eligible(T["staging_pose"], D["staging_pose"], WPS), ["W1"])

    def test_none_and_named(self):
        self.assertIsNone(RT.choose_via((0, 0), (1, 1), WPS, "none")[0])
        self.assertEqual(RT.choose_via((0, 0), (1, 1), WPS, "W2")[0], "W2")
        with self.assertRaises(ValueError):
            RT.choose_via((0, 0), (1, 1), WPS, "W9")

    def test_random_mixes_direct_and_via(self):
        rng = random.Random(1)
        picks = [RT.choose_via(T["staging_pose"], D["staging_pose"], WPS, "random", rng)[0] for _ in range(200)]
        self.assertEqual(set(picks), {None, "W1"})
        self.assertTrue(0.4 < picks.count("W1") / 200 < 0.6)          # W1 or direct, equally likely

    def test_site_layout_three_equal_choices_both_ways(self):
        # the taught site (2026-11-16, map 3): W1 / W2 on either side of an obstacle between the safe points
        t_safe, d_safe = (1.263, 1.808), (3.054, -0.852)
        site = {"W1": {"pose": [3.799, 0.502, 118.4]}, "W2": {"pose": [2.437, -0.08, -64.2]}}
        rng = random.Random(4)
        for a, b in ((t_safe, d_safe), (d_safe, t_safe)):
            self.assertEqual(RT.eligible(a, b, site), ["W1", "W2"])
            picks = [RT.choose_via(a, b, site, "random", rng)[0] for _ in range(3000)]
            for k in ("W1", "W2", None):
                self.assertTrue(0.30 < picks.count(k) / 3000 < 0.37, (k, picks.count(k)))

    def test_random_without_waypoints_is_direct(self):
        self.assertIsNone(RT.choose_via((0, 0), (3, 0), {}, "random")[0])

    def test_file_round_trip_and_bad_file(self):
        with tempfile.TemporaryDirectory() as d:
            p = str(Path(d) / "waypoints.json")
            self.assertEqual(RT.load_waypoints(p), {})
            RT.save_waypoints(p, WPS)
            self.assertEqual(RT.load_waypoints(p), WPS)
            Path(p).write_text("not json")
            self.assertEqual(RT.load_waypoints(p), {})


class JitterTests(unittest.TestCase):
    def test_zero_radius_keeps_the_taught_point(self):
        self.assertEqual(RT.jitter_pose(D["staging_pose"], D, 0.0), D["staging_pose"])

    def test_within_radius_and_heading_kept(self):
        rng = random.Random(2)
        for st in (T, D):
            b = st["staging_pose"]
            pts = [RT.jitter_pose(b, st, 0.08, rng) for _ in range(300)]
            self.assertTrue(all(math.hypot(p[0] - b[0], p[1] - b[1]) <= 0.0805 for p in pts))
            self.assertTrue(all(p[2] == b[2] for p in pts))
            self.assertGreater(len({tuple(p) for p in pts}), 250)          # really varies

    def test_face_station_stays_in_the_docking_envelope(self):
        rng = random.Random(3)
        for _ in range(300):
            p = RT.jitter_pose(D["staging_pose"], D, 0.20, rng)
            self.assertTrue(RT.in_face_envelope(p, D, 0.338, 1.20, 1.40))

    def test_point_outside_envelope_rejected(self):
        # 0.5 m in FRONT of the final pose (heading 169.3 deg): never a start for the lidar docking
        h = math.radians(D["pose"][2])
        ahead = [D["pose"][0] + 0.5 * math.cos(h), D["pose"][1] + 0.5 * math.sin(h)]
        self.assertFalse(RT.in_face_envelope(ahead, D, 0.338, 1.20, 1.40))
        self.assertTrue(RT.in_face_envelope(D["staging_pose"], D, 0.338, 1.20, 1.40))

    def test_no_fit_falls_back_to_the_taught_point(self):
        # safe point right at the limit behind the pose: half of all jitters fall outside; never returns one
        h = math.radians(D["pose"][2])
        b = [D["pose"][0] - 1.20 * math.cos(h), D["pose"][1] - 1.20 * math.sin(h), D["pose"][2]]
        rng = random.Random(4)
        for _ in range(200):
            p = RT.jitter_pose(b, D, 0.08, rng)
            self.assertTrue(p == b or RT.in_face_envelope(p, D, 0.338, 1.20, 1.40))


if __name__ == "__main__":
    unittest.main()

"""Offline tests of the chassis navigation client (no ROS, no SSH, no robot).

Run from the repository root:  python3 -m unittest discover -s chassis/tests -v
"""

import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))
from gento_navigation import GentoNavigation, NavigationError  # noqa: E402


class SshTransportTests(unittest.TestCase):
    def patch_run(self, outputs):
        calls = []

        def fake_run(argv, **kwargs):
            calls.append(argv[-1])
            return subprocess.CompletedProcess(argv, 0, stdout=outputs.pop(0), stderr="")
        return calls, mock.patch("gento_navigation.subprocess.run", side_effect=fake_run)

    def test_matches_the_skye_task_navigation_protocol(self):
        for name in ("start", "is_arrived_and_stopped", "stop"):
            self.assertTrue(callable(getattr(GentoNavigation, name)))

    def test_goto_with_gap_and_arrival_requires_this_job(self):
        calls, patch = self.patch_run([
            "started job D (123): timeout 500 ...\n",
            json.dumps({"state": "done", "job": "T"}) + "\n",
            json.dumps({"state": "running", "job": "D"}) + "\n",
            json.dumps({"state": "done", "job": "D", "final": "FINAL {}"}) + "\n",
        ])
        with patch:
            nav = GentoNavigation(transport="ssh", ssh_target="chassis", gaps={"D": 0.3})
            nav.start("D")
            self.assertFalse(nav.is_arrived_and_stopped())   # an older job's "done" does not count
            self.assertFalse(nav.is_arrived_and_stopped())
            self.assertTrue(nav.is_arrived_and_stopped())
        self.assertTrue(calls[0].endswith("run.sh goto D 0.300"))

    def test_refused_job_raises(self):
        _, patch = self.patch_run(["BUSY: job 42 still running - use stop first\n"])
        with patch, self.assertRaises(NavigationError):
            GentoNavigation(transport="ssh").start("D")

    def test_failed_job_reports_the_robot_reason(self):
        _, patch = self.patch_run(["started job D (1)\n",
                                   json.dumps({"state": "failed", "job": "D", "error": "ABORT: straight approach blocked"})])
        with patch:
            nav = GentoNavigation(transport="ssh")
            nav.start("D")
            with self.assertRaisesRegex(NavigationError, "straight approach blocked"):
                nav.raise_if_failed()

    def test_stale_stop_from_the_previous_run_is_ignored_but_a_real_stop_is_reported(self):
        stale = json.dumps({"state": "failed", "job": "stopped", "error": "stopped by user"})
        _, patch = self.patch_run([
            "started job T (7)\n",
            stale,                                                     # left by the previous run
            json.dumps({"state": "running", "job": "T"}),
            stale,                                                     # stopped while OUR job ran
        ])
        with patch:
            nav = GentoNavigation(transport="ssh")
            nav.start("T")
            nav.raise_if_failed()                                      # stale: must not raise
            self.assertFalse(nav.is_arrived_and_stopped())
            with self.assertRaisesRegex(NavigationError, "stopped by user"):
                nav.raise_if_failed()

    def test_undock_is_a_job_polled_like_a_destination(self):
        calls, patch = self.patch_run([
            "started job undock (9)\n",
            json.dumps({"state": "done", "job": "D"}),                 # the docking before it: ignored
            json.dumps({"state": "done", "job": "undock", "final": "FINAL {}"}),
        ])
        with patch:
            nav = GentoNavigation(transport="ssh")
            nav.start_undock()
            self.assertFalse(nav.is_arrived_and_stopped())
            self.assertTrue(nav.is_arrived_and_stopped())
        self.assertTrue(calls[0].endswith("run.sh undock"))

    def test_stop_and_tune_commands(self):
        calls, patch = self.patch_run(["stopped\n", "D {\"gap\": 0.3}\n"])
        with patch:
            nav = GentoNavigation(transport="ssh")
            nav.stop()
            nav.tune("D", "gap", 0.3)
        self.assertTrue(calls[0].endswith("run.sh stop"))
        self.assertTrue(calls[1].endswith("run.sh tune D gap 0.3"))

    def test_invalid_transport_rejected(self):
        with self.assertRaises(ValueError):
            GentoNavigation(transport="udp")


class RobotModelTests(unittest.TestCase):
    """The collision model on synthetic clouds (needs numpy)."""
    def setUp(self):
        try:
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest("numpy not installed")
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        import robot_model
        self.rm, self.np = robot_model, numpy
        self.cfg = robot_model.load()
        self.rng = numpy.random.default_rng(0)

    def block(self, x0, x1, y0, y1, z0, z1, n):
        r = self.rng
        return self.np.stack([r.uniform(x0, x1, n), r.uniform(y0, y1, n), r.uniform(z0, z1, n)], 1)

    def test_turning_next_to_a_frame_at_tray_height_is_a_collision(self):
        frame = self.block(0.55, 0.65, -0.6, -0.1, 0.9, 1.5, 2000)        # like the glass tunnel beside the belt
        self.assertIsNotNone(self.rm.rotation_hits(frame, self.cfg))

    def test_reversing_until_the_turn_is_clear(self):
        frame = self.block(0.80, 0.85, -0.3, 0.3, 1.1, 1.5, 2000)
        self.assertIsNotNone(self.rm.rotation_hits(frame, self.cfg))
        dx, dy = self.rm.leave_move(frame, self.cfg, 0.6)
        self.assertLess(dx, 0)
        self.assertIsNone(self.rm.rotation_hits(frame - self.np.array([dx, dy, 0]), self.cfg))

    def test_surface_under_the_tray_is_not_an_obstacle_but_an_item_on_it_is(self):
        belt = self.block(0.60, 1.10, -0.5, 0.5, 0.83, 0.84, 4000)
        self.assertIsNone(self.rm.place_hits(belt, self.cfg, 0.3, 0.84))
        item = self.block(0.95, 1.00, -0.05, 0.05, 0.85, 0.95, 300)
        self.assertIsNotNone(self.rm.place_hits(self.np.concatenate([belt, item]), self.cfg, 0.3, 0.84))

    def test_measured_arm_shape_passes_over_a_table_top(self):
        # arms seen by the lidar: forearms at 1.25-1.4 m, grippers down to 1.0 m at x 0.5-0.6 (right side only)
        arm = self.np.concatenate([self.block(0.25, 0.50, -0.45, -0.30, 1.25, 1.40, 400),
                                   self.block(0.50, 0.62, -0.25, -0.15, 1.00, 1.30, 300)])
        r = self.rm.selfscan(arm)
        cfg = dict(self.cfg, phases=dict(self.cfg["phases"], carry=[r["carry_box"]], carry_slices=r["carry_slices"]))
        cfg["tray"] = dict(cfg["tray"], z_bottom=1.25)
        names = [n for n, _ in self.rm.boxes(cfg)]
        self.assertGreater(names.count("arms"), 3)
        widths = [b[3] for n, b in self.rm.boxes(cfg) if n == "arms"]
        self.assertTrue(all(abs(w) >= 0.25 for w in widths))                    # mirrored: both arms covered
        table = self.block(0.55, 1.20, -0.6, 0.6, 0.88, 0.90, 3000)               # table top 0.9 m, edge at 0.55
        self.assertIsNone(self.rm.sweep_hits(table, cfg, 0.10, 0.0))             # grippers (>= 0.97 m) pass over it
        box = dict(self.cfg, phases=dict(self.cfg["phases"], carry=[[0.0, 0.885, -0.468, 0.468, 0.66, 1.424]]))
        self.assertIsNotNone(self.rm.sweep_hits(table, box, 0.10, 0.0))          # the old single box did not

    def test_fast_rotation_check_matches_the_box_circles(self):
        frame = self.block(0.55, 0.65, -0.6, -0.1, 0.9, 1.5, 2000)
        self.assertIsNotNone(self.rm.rotation_hits(frame, self.cfg))
        far = self.block(1.6, 1.7, -0.6, 0.6, 0.9, 1.5, 2000)
        self.assertIsNone(self.rm.rotation_hits(far, self.cfg))


if __name__ == "__main__":
    unittest.main()

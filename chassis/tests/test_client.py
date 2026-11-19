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

    def test_route_options_reach_run_sh(self):
        calls, patch = self.patch_run(["started job D (1): ...\n", "started job T (2): ...\n"])
        with patch:
            GentoNavigation(transport="ssh", route={"via": "random", "jitter": 0.08}).start("D")
            GentoNavigation(transport="ssh", route={"via": None, "jitter": 0}).start("T")
        self.assertTrue(calls[0].endswith("run.sh goto D --via random --jitter 0.080"), calls[0])
        self.assertTrue(calls[1].endswith("run.sh goto T"), calls[1])

    def test_previous_result_of_the_same_station_is_not_taken_as_arrival(self):
        # ROS transport: the bridge status can still show the previous "done T" right after goto T is accepted
        nav = GentoNavigation(transport="ssh")
        nav.transport = "ros"
        nav._status_at = 1e18                                   # never stale in this test
        nav._status = {"job": {"state": "done", "job": "T", "t": 100}}
        nav._begin("T", nav._peek_job())
        self.assertFalse(nav.is_arrived_and_stopped())         # the old result
        nav._status = {"job": {"state": "running", "job": "T", "t": 103}}
        self.assertFalse(nav.is_arrived_and_stopped())
        nav._status = {"job": {"state": "done", "job": "T", "t": 112}}
        self.assertTrue(nav.is_arrived_and_stopped())

    def test_a_new_result_without_seeing_running_still_counts(self):
        nav = GentoNavigation(transport="ssh")
        nav.transport = "ros"; nav._status_at = 1e18
        nav._status = {"job": {"state": "done", "job": "T", "t": 100}}
        nav._begin("T", nav._peek_job())
        nav._status = {"job": {"state": "done", "job": "T", "t": 104}}
        self.assertTrue(nav.is_arrived_and_stopped())
        nav._status = {"job": {"state": "done", "job": "D", "t": 90}}
        nav._begin("T", nav._peek_job())                       # different previous job: first T result is ours
        nav._status = {"job": {"state": "done", "job": "T", "t": 95}}
        self.assertTrue(nav.is_arrived_and_stopped())

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


class SilentBridgeTests(unittest.TestCase):
    """ROS transport while the bridge is silent (2026-11-17: > 3 s without status on the loaded chassis; the task
    aborted and its ROS stop never arrived - the chassis drove on alone): SSH fallback for status and stop."""

    def nav(self, status_age):
        nav = GentoNavigation(transport="ssh")
        nav.transport = "ros"
        nav._publisher = mock.Mock(); nav._publisher.get_subscription_count.return_value = 0
        nav._string = lambda data: data
        nav._status = {"job": {"state": "running", "job": "D", "t": 100}}
        nav._status_at = __import__("time").monotonic() - status_age
        nav.active, nav._seen_active = "D", True
        return nav

    def ssh(self, outputs):
        calls = []

        def fake_run(argv, **kwargs):
            calls.append(argv)
            out = outputs.pop(0)
            if isinstance(out, Exception):
                raise out
            return subprocess.CompletedProcess(argv, 0, stdout=out, stderr="")
        return calls, mock.patch("gento_navigation.subprocess.run", side_effect=fake_run)

    def test_fresh_ros_status_needs_no_ssh(self):
        calls, patch = self.ssh([])
        with patch:
            self.assertEqual(self.nav(0.5).job_status()["state"], "running")
        self.assertEqual(calls, [])

    def test_stale_ros_status_is_read_over_ssh(self):
        calls, patch = self.ssh([json.dumps({"state": "done", "job": "D", "t": 200}) + "\n"])
        with patch:
            nav = self.nav(4.0)
            self.assertTrue(nav.is_arrived_and_stopped())       # the arrival came over SSH
        self.assertEqual(calls[0][-2], "sunrise@6.6.7.6")
        self.assertIn("job_status.json", calls[0][-1])

    def test_link_drop_of_half_a_minute_is_ridden_out(self):
        # 2026-11-17: eth1 dropped for seconds at a time; the chassis job drove on - the task must wait, not abort
        calls, patch = self.ssh([subprocess.TimeoutExpired("ssh", 5)])
        with patch:
            nav = self.nav(30.0)
            self.assertEqual(nav.job_status()["state"], "running")
        calls, patch = self.ssh([json.dumps({"state": "done", "job": "D", "t": 200}) + "\n"])
        with patch:                                              # the link is back: the result is picked up
            nav._ssh_tried = 0.0
            self.assertTrue(nav.is_arrived_and_stopped())

    def test_short_silence_without_ssh_keeps_the_last_status(self):
        calls, patch = self.ssh([subprocess.TimeoutExpired("ssh", 5)])
        with patch:
            nav = self.nav(5.0)
            self.assertEqual(nav.job_status()["state"], "running")   # not an abort yet
            nav._status_at -= 70.0; nav._ssh_tried = 0.0
        calls, patch = self.ssh([subprocess.TimeoutExpired("ssh", 5)])
        with patch, self.assertRaises(NavigationError) as cm:
            nav.job_status()
        self.assertIn("ROS 与 ssh 都已 60 s 无应答", str(cm.exception)); self.assertIn("gento doctor", str(cm.exception))

    def test_stop_goes_over_ssh_when_the_bridge_is_silent(self):
        calls, patch = self.ssh([""])
        with patch:
            nav = self.nav(6.0)
            nav.stop()
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0][-1].endswith("run.sh stop"))
        self.assertIsNone(nav.active)

    def test_ros_stop_failure_falls_back_to_ssh(self):
        calls, patch = self.ssh([""])
        with patch, mock.patch.object(GentoNavigation, "_ros_command", side_effect=NavigationError("未发现底盘导航桥")):
            self.nav(0.5).stop()
        self.assertTrue(calls[0][-1].endswith("run.sh stop"))


class BridgeJobStatusTests(unittest.TestCase):
    def test_reads_the_status_file_like_run_sh(self):
        import importlib.util
        import os
        import tempfile
        for m in ("rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg", "slam", "slam.msg"):
            sys.modules.setdefault(m, mock.MagicMock())
        spec = importlib.util.spec_from_file_location("dock_bridge", str(Path(__file__).resolve().parents[1] / "dock_bridge.py"))
        bridge = importlib.util.module_from_spec(spec); spec.loader.exec_module(bridge)
        with tempfile.TemporaryDirectory() as tmp:
            bridge.JS, bridge.PIDF = os.path.join(tmp, "job_status.json"), os.path.join(tmp, "run.pid")
            self.assertEqual(bridge.read_job(), {"state": "idle"})
            Path(bridge.JS).write_text(json.dumps({"state": "running", "job": "D"}))
            Path(bridge.PIDF).write_text(str(os.getpid()))
            self.assertEqual(bridge.read_job()["state"], "running")             # the job process is alive
            Path(bridge.PIDF).write_text("999999999")
            self.assertEqual(bridge.read_job(), {"state": "failed", "error": "job died"})
            Path(bridge.JS).write_text(json.dumps({"state": "done", "job": "D", "final": "FINAL {}"}))
            self.assertEqual(bridge.read_job()["state"], "done")


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

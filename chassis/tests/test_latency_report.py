"""Offline tests of the planning-latency report (chassis/latency_report.py) on a log in dock.py's format.

Run from the repository root:  python3 -m unittest discover -s chassis/tests -v
"""

import io
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import latency_report as LR  # noqa: E402

LOG = """===== 20:49:59 timeout 500 python3 dock.py dock D --leave --closest
[20:50:00.120    1.0s] fine-tune: lat_offset 0.000 m, yaw_offset -1.35 deg
[20:50:01.610    2.5s] localisation: pose [1.307, 2.39, -94.8], scan-to-map match 0.92
[20:50:01.620    2.5s] PLAN start: leave the station (back to its safe point)
[20:50:02.430    3.3s] INPUT ready: pose settled (1.307, 2.390, -94.8 deg)
[20:50:02.431    3.3s] at map station T: back to its safe point first (x 0.58, y -0.04)
[20:50:02.432    3.3s] LIDAR scan start
[20:50:03.180    4.1s] INPUT ready: lidar snapshot 118734 points, 6 frames
[20:50:03.181    4.1s] PLAN start: straight path
[20:50:03.262    4.2s] PLAN done: move x 0.000 y -0.040 yaw 0.0 | input -> plan 82 ms
[20:50:04.050    4.9s] COMMAND accepted: move x 0.000 y -0.040 yaw 0.0 | input -> command 870 ms
[20:50:04.410    5.3s] MOTION started: move x 0.000 y -0.040 yaw 0.0 | input -> motion 1230 ms
[20:50:06.000    6.9s]   rel move x=0.000 y=-0.040 yaw=0.00deg -> (0, '完成相对移动')
[20:50:06.010    6.9s] PLAN done: move x 0.591 y 0.000 yaw 0.0 | input -> plan 2829 ms  OVER 1200 ms
[20:50:06.100    7.0s] COMMAND accepted: move x 0.591 y 0.000 yaw 0.0 | input -> command 2919 ms
[20:50:06.300    7.2s] MOTION started: move x 0.591 y 0.000 yaw 0.0 | input -> motion 3119 ms
[20:50:30.000   30.9s] FINAL {"name": "D"}
===== 20:52:10 timeout 90 python3 dock.py undock
[20:52:11.000    1.0s] LIDAR scan start
[20:52:11.700    1.7s] INPUT ready: lidar snapshot 120000 points, 6 frames
[20:52:11.760    1.8s] PLAN done: move x -0.317 y 0.000 yaw 0.0 | input -> plan 60 ms
[20:52:11.900    1.9s] COMMAND accepted: move x -0.317 y 0.000 yaw 0.0 | input -> command 200 ms
[20:52:12.100    2.1s] MOTION started: move x -0.317 y 0.000 yaw 0.0 | input -> motion 400 ms
===== 20:53:00 timeout 90 python3 dock.py teach-ref T
[20:53:02.000    2.0s] reference scan of T: 17895 points
"""


class LatencyReportTests(unittest.TestCase):
    def setUp(self):
        self.jobs = LR.parse(io.StringIO(LOG))

    def test_commands_are_paired_with_their_inputs(self):
        d = self.jobs[0]['cmds']
        self.assertEqual(len(d), 2)
        self.assertEqual(d[0]['scan'], '20:50:02.432')
        self.assertEqual(d[0]['input'], '20:50:03.180')
        self.assertEqual(d[0]['plan_start'], '20:50:03.181')
        self.assertEqual((d[0]['plan_ms'], d[0]['cmd_ms'], d[0]['motion_ms']), (82, 870, 1230))
        self.assertIsNone(d[1]['input'])                 # planned on older data: no fresh input of its own
        self.assertEqual(d[1]['plan_ms'], 2829)

    def test_jobs_without_timed_commands_have_none(self):
        self.assertEqual([len(j['cmds']) for j in self.jobs], [2, 1, 0])
        self.assertTrue(self.jobs[0]['end'].startswith('FINAL'))

    def test_report_marks_start_up_and_over_limit(self):
        out = io.StringIO(); LR.report(self.jobs, out); text = out.getvalue()
        self.assertIn('START-UP (* first command): input (lidar snapshot 118734 points, 6 frames) -> plan done 82 ms [OK]', text)
        self.assertIn('motion started 1230 ms [OVER]', text)
        self.assertIn('input -> plan max 2829 ms (move x 0.591 y 0.000 yaw 0.0) [OVER]', text)
        self.assertNotIn('teach-ref', text)


if __name__ == "__main__":
    unittest.main()

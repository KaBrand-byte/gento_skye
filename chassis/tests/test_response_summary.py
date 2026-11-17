"""Response-time metric: dock.py logs it per job (first lidar input -> first chassis command), response_summary.py
puts the run's figure on top of the run log and appends it to a CSV."""
import io
import os
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'client'))
import response_summary as RS  # noqa: E402
from test_dock_ref import load_dock  # noqa: E402

OLD_LOG = """===== 2026-11-17 11:22:38 gento task 1/1:
===== chassis ~/gento_dock/run.log from line 7842
===== 20:39:34 timeout 650 python3 dock.py dock D --leave --closest
[20:39:40.719    5.3s] INPUT ready: pose settled (1.315, 2.389, -95.1 deg) conf 85
[20:39:41.523    6.1s] LIDAR scan start
[20:39:42.235    6.8s] INPUT ready: lidar snapshot 240000 points, 6 frames
[20:39:42.327    6.9s] PLAN done: move x 0.594 y 0.000 yaw 0.0 | input -> plan 93 ms
[20:39:42.349    6.9s] COMMAND accepted: move x 0.594 y 0.000 yaw 0.0 | input -> command 115 ms
[20:39:51.084   15.6s] COMMAND accepted: move x 0.000 y 0.000 yaw 67.6 | input -> command 62 ms
===== 20:41:31 timeout 90 python3 dock.py undock
[20:41:34.200    2.0s] LIDAR scan start
[20:41:34.933    2.7s] INPUT ready: lidar snapshot 240000 points, 6 frames
[20:41:34.992    2.8s] COMMAND accepted: move x -0.767 y 0.000 yaw 0.0 | input -> command 59 ms
===== 20:41:48 timeout 650 python3 dock.py dock T --leave --closest
[20:41:57.144    7.7s] LIDAR scan start
[20:41:57.829    8.3s] INPUT ready: lidar snapshot 240000 points, 6 frames
[20:41:59.527   10.0s] COMMAND accepted: move x 0.000 y 0.000 yaw -47.4 | input -> command 98 ms
"""


class ResponseSummaryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_old_chassis_log_is_reconstructed(self):
        res = RS.parse(OLD_LOG.splitlines(True))
        self.assertEqual([(r['job'], round(r['ms'])) for r in res], [('dock D', 114), ('undock', 59), ('dock T', 1698)])
        text = RS.summary(res)
        self.assertIn('114 ms  →  合格', text)                              # only the leg to D counts
        self.assertIn('20:39:42.235', text); self.assertIn('20:39:42.349', text)
        self.assertNotIn('1698', text)                                       # the later legs are not listed

    def test_handover_drive_to_t_first_does_not_count(self):
        head = '===== 2026-11-17 11:22:38 gento task 1/1:\n'
        log = OLD_LOG.replace(head, head + '===== 20:30:00 timeout 650 python3 dock.py dock T --leave --closest\n'
                              '[20:30:05.000    5.0s] INPUT ready: lidar snapshot 240000 points, 6 frames\n'
                              '[20:30:07.000    7.0s] COMMAND accepted: move x 0.1 y 0.0 yaw 0.0 | input -> command 2000 ms\n')
        text = RS.summary(RS.parse(log.splitlines(True)))
        self.assertIn('114 ms  →  合格', text); self.assertNotIn('2000', text)

    def test_new_dock_log_line_round_trip_and_csv_appends(self):
        ns = load_dock(self.tmp)
        ns['RESP_FILE'] = os.path.join(self.tmp, 'response_log.csv')
        out = io.StringIO()
        with redirect_stdout(out):
            ns['input_ready']('pose settled (1, 2, 3 deg)')                  # not a point cloud: no start
            ns['command_sent']('navigation task')                            # before any point cloud: ignored
            ns['cloud'] = None
            ns['FIRST']['last_scan'] = 1000.0
            ns['input_ready']('lidar snapshot 240000 points, 6 frames')
            ns['command_sent']('move x 0.594 y 0.000 yaw 0.0')
            ns['input_ready']('lidar snapshot 240000 points, 6 frames')       # later inputs / commands: no change
            ns['command_sent']('move x 0.000 y 0.000 yaw 67.6')
        lines = [l for l in out.getvalue().splitlines() if 'RESPONSE:' in l]
        self.assertEqual(len(lines), 1)
        self.assertIn('command: move x 0.594', lines[0]); self.assertIn(' OK |', lines[0])
        self.assertEqual(len(open(ns['RESP_FILE']).read().splitlines()), 2)  # header + this job
        log = os.path.join(self.tmp, 'run.log')
        with open(log, 'w', encoding='utf-8') as fh:
            fh.write('===== 20:39:34 timeout 650 python3 dock.py dock D --leave --closest\n' + lines[0] + '\n')
        csv = os.path.join(self.tmp, 'response_times.csv')
        for _ in range(2):                                                   # two runs: two lines, nothing lost
            with redirect_stdout(io.StringIO()) as o:
                RS.main([log, '--csv', csv, '--offset', '-33000'])
        self.assertIn('epoch', o.getvalue()); self.assertIn('Orin 时间', o.getvalue()); self.assertIn('合格', o.getvalue())
        self.assertEqual(len(open(csv, encoding='utf-8').read().splitlines()), 3)

    def test_no_chassis_log(self):
        self.assertIn('无法统计', RS.summary(RS.parse(['===== 2026-11-17 gento task\n', 'hello\n'])))


if __name__ == '__main__':
    unittest.main()

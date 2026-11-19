"""dock.py waiting for the vendor navigation (wait_task) and its safety net.  Regression of 2026-11-19: the vendor
task list took > 5 s to answer on the loaded chassis, the timeout killed the job and the navigation drove on
unsupervised - the robot ended short of T, not turned."""
import io
import os
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_dock_ref import load_dock  # noqa: E402


class Trace:
    def add(self):
        pass

    def summary(self):
        pass


class WaitTaskTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        ns = self.ns = load_dock(self.tmp)
        self.cancelled = []
        self.poses = iter([])
        ns.update(NavTrace=Trace, spin=lambda t: None, cancel_all_tasks=lambda: self.cancelled.append('tasks'),
                  map_pose=lambda: next(self.poses))
        ns['NAV_ACTIVE'][0] = True

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_wait(self, answers, poses, **kw):
        self.poses = iter(poses)
        it = iter(answers)

        def urlopen(q, timeout=5):
            a = next(it)
            if isinstance(a, Exception):
                raise a
            return io.BytesIO(a.encode())
        out = io.StringIO()
        with mock.patch('urllib.request.urlopen', urlopen), redirect_stdout(out):
            try:
                return self.ns['wait_task'](**kw)
            finally:
                self.log = out.getvalue()

    def test_slow_vendor_answer_does_not_end_the_job(self):
        far = [(3.0 - 0.1 * i, 0.0, 0.0) for i in range(20)]
        ok = self.run_wait([TimeoutError('timed out'), TimeoutError('timed out'), '{"Content": [1]}', '{"Content": []}'],
                           far, near=(0.0, 0.0))
        self.assertTrue(ok)
        self.assertIn('did not answer', self.log); self.assertIn('answers again', self.log)
        self.assertFalse(self.ns['NAV_ACTIVE'][0])                         # seen finished

    def test_arrival_by_position_while_the_server_is_silent(self):
        poses = [(1.0, 0.0, 0.0), (0.5, 0.0, 0.0), (0.1, 0.0, 0.0)]
        ok = self.run_wait([TimeoutError('timed out')] * 5, poses, near=(0.0, 0.0))
        self.assertTrue(ok); self.assertEqual(self.cancelled, ['tasks']); self.assertFalse(self.ns['NAV_ACTIVE'][0])

    def test_server_silent_too_long_aborts_and_leaves_the_navigation_to_be_cancelled(self):
        self.ns['HTTP_LOST'] = 0.0
        with self.assertRaises(SystemExit) as cm:
            self.run_wait([TimeoutError('timed out')] * 5, [(3.0, 0.0, 0.0)] * 5, near=(0.0, 0.0))
        self.assertIn('has not answered', str(cm.exception))
        self.assertTrue(self.ns['NAV_ACTIVE'][0])                          # the exit handler cancels it


class SafetyNetTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.ns = load_dock(self.tmp)
        self.stops = []
        self.ns['safe_stop'] = lambda why, moves=True: self.stops.append((why, moves))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_crash_during_a_motion_job_stops_everything(self):
        self.ns['MOTION_JOB'][0] = True
        with redirect_stdout(io.StringIO()) as out, mock.patch('traceback.print_exception'):
            self.ns['_crash'](TimeoutError, TimeoutError('timed out'), None)
        self.assertIn('ABORT: internal error TimeoutError', out.getvalue())
        self.ns['_at_exit']()
        self.assertEqual(self.stops, [('internal error', True)])

    def test_abort_during_the_navigation_cancels_it(self):
        self.ns['NAV_ACTIVE'][0] = True
        self.ns['_at_exit']()
        self.assertEqual(self.stops, [('job ended during the navigation', False)])

    def test_normal_end_touches_nothing(self):
        self.ns['_at_exit']()
        self.assertEqual(self.stops, [])


if __name__ == '__main__':
    unittest.main()

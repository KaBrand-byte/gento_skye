"""Straight moves to a map station (dock.py translate): a blocked path is checked again on fresh scans before the
job gives up, and the snapshots that blocked it are kept.  Regression of 2026-11-17: 24 lidar returns 2 mm inside
the margin behind T, in one snapshot only (nothing was there), aborted the return to T at once."""
import io
import os
import shutil
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_dock_ref import grid, load_dock  # noqa: E402

WALL = grid(-3.0, -3.0, -1.0, 1.0, 0.1, 1.5, step=0.1)                  # far behind: never in the way
GHOST = np.vstack([WALL, grid(-1.15, -1.15, -0.06, 0.06, 0.74, 0.82)])   # the 2026-11-17 returns behind T


class TranslateBlockedTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        ns = self.ns = load_dock(self.tmp)
        self.moves, self.scans = [], []
        ns.update(rel=lambda x=0.0, yaw=0.0, rear_check=False, y=0.0: self.moves.append((round(x, 3), round(y, 3))),
                  spin=lambda t: time.sleep(0.05))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_translate(self, clouds, dx=-0.83, dy=0.0):
        it = iter(clouds)

        def cloud(fresh=False):
            self.scans.append(fresh)
            return next(it)
        self.ns['cloud'] = cloud
        out = io.StringIO()
        try:
            with redirect_stdout(out):
                self.ns['translate'](dx, dy)
        finally:
            self.log = out.getvalue()

    def saved(self):
        d = self.ns['BLOCKED_DIR']
        return sorted(os.listdir(d)) if os.path.isdir(d) else []

    def test_one_noisy_snapshot_rescans_and_drives(self):
        self.run_translate([GHOST, WALL])
        self.assertEqual(self.scans, [False, True])                         # the second one is a fresh scan
        self.assertEqual(self.moves, [(round(-0.83 - self.ns['FWD_SHORT'], 3), 0.0)])
        self.assertIn('WAITING: straight path', self.log)
        self.assertIn('clear on a fresh scan after 1 rescans', self.log)
        files = self.saved()
        self.assertEqual(len(files), 1); self.assertIn('-first.npy', files[0])
        self.assertEqual(np.load(os.path.join(self.ns['BLOCKED_DIR'], files[0])).shape, GHOST.shape)

    def test_clear_path_takes_no_extra_scan(self):
        self.run_translate([WALL])
        self.assertEqual(self.scans, [False]); self.assertEqual(len(self.moves), 1); self.assertEqual(self.saved(), [])

    def test_still_blocked_after_waiting_is_blocked_for_the_orin(self):
        self.ns['BLOCK_WAIT'] = 0.3
        with self.assertRaises(SystemExit) as cm:
            self.run_translate([GHOST] * 100)
        msg = str(cm.exception)
        self.assertTrue(msg.startswith('ABORT: BLOCKED: no clear straight path'), msg)   # the runner offers a retry
        self.assertIn('still there after 0 s', msg)
        self.assertEqual(self.moves, [])
        self.assertEqual([f.rsplit('-', 1)[1] for f in self.saved()], ['first.npy', 'last.npy'])

    def test_no_wait_aborts_at_once_like_before(self):
        self.ns['TRANSLATE_WAIT'][0] = 0.0                                  # jittered spot: the taught one instead
        with self.assertRaises(SystemExit) as cm:
            self.run_translate([GHOST])
        self.assertTrue(str(cm.exception).startswith('ABORT: no clear straight path'), str(cm.exception))
        self.assertEqual(self.scans, [False])

    def test_only_the_newest_snapshots_are_kept(self):
        d = self.ns['BLOCKED_DIR']; os.makedirs(d)
        for i in range(12):
            open(os.path.join(d, 'blocked-20260101-0000%02d-first.npy' % i), 'w').close()
        self.run_translate([GHOST, WALL])
        files = self.saved()
        self.assertEqual(len(files), self.ns['BLOCKED_KEEP'])
        self.assertNotIn('blocked-20260101-000000-first.npy', files)


class VendorDownTest(unittest.TestCase):
    """2026-11-18: the vendor programs were not running; dock.py and locmatch.py died with tracebacks."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.ns = load_dock(self.tmp)
        self.ns.update(ready_to_move=lambda: None, spin=lambda t: None)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_no_chassis_state_is_a_clear_abort(self):
        self.ns['S'][0] = None
        with self.assertRaises(SystemExit) as cm:
            self.ns['need_state'](timeout=0.2)
        self.assertIn('move/State', str(cm.exception)); self.assertIn('linkhou_robot', str(cm.exception))
        self.assertIn('gento doctor', str(cm.exception))

    def test_no_lidar_is_a_clear_abort(self):
        del self.ns['C'][:]
        with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as cm:
            self.ns['cloud'](fresh=True)
        self.assertIn('lidar point cloud', str(cm.exception)); self.assertIn('linkhou_robot', str(cm.exception))


if __name__ == '__main__':
    unittest.main()

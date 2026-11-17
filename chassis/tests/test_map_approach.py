"""Map station approach (dock.py map_approach), scripted: what the localisation reports in turn, what the chassis
is told to do.  Regression of 2026-11-17: two heading corrections in a row used up the approach and the scan
match ran 0.7 m short of T (4% matched -> ABORT)."""
import math
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_dock_ref import load_dock  # noqa: E402


class MapApproachTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        ns = self.ns = load_dock(self.tmp)
        os.makedirs(ns['REF_DIR'], exist_ok=True)
        np.save(os.path.join(ns['REF_DIR'], 'T.npy'), np.zeros((10, 3)))      # T has a reference scan
        self.calls = []
        ns.update(turn_to=lambda h: None, cloud=lambda fresh=False: np.zeros((0, 3)), need_clear=lambda hits, what: None,
                  rel=lambda x=0.0, yaw=0.0, rear_check=False, y=0.0: self.calls.append(('turn', round(math.degrees(yaw), 2))),
                  translate=lambda dx, dy: self.calls.append(('translate', round(dx, 3), round(dy, 3))),
                  ref_approach=lambda name: (self.calls.append(('scan match',)), (0.004, 0.003, 0.001))[1])
        patcher = mock.patch.object(ns['RM'], 'turn_hits', lambda p, cfg, da: None)
        patcher.start(); self.addCleanup(patcher.stop)
        self.st = dict(pose=[1.307, 2.39, -94.75], staging_pose=[1.263, 1.808, -95.28], mode='map')

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def approach(self, readings):
        """readings: (dx, dy, dyaw_deg) the map reports, one per rel_target call."""
        it = iter([(dx, dy, math.radians(da)) for dx, dy, da in readings])
        self.ns['rel_target'] = lambda pose, settle=4.0: next(it)
        self.ns['map_approach']('T', self.st, False)
        self.assertEqual(list(it), [])                                       # every reading was used

    def test_two_heading_corrections_still_translate_before_the_scan_match(self):
        # the 2026-11-17 case: 5.2 deg, then -3.85 deg after the first turn, then within 1.5 deg
        self.approach([(-0.629, -0.199, 5.20), (-0.697, -0.151, -3.85), (-0.700, -0.150, -1.0), (0.01, 0.01, 0.3)])
        self.assertEqual([c[0] for c in self.calls], ['turn', 'turn', 'translate', 'scan match'])
        self.assertEqual(self.calls[2], ('translate', -0.7, -0.15))

    def test_turns_used_up_the_translation_still_happens(self):
        self.approach([(-0.70, -0.15, 4.0), (-0.70, -0.15, -3.0), (-0.70, -0.15, 2.5), (-0.70, -0.15, -2.0),
                       (0.02, 0.0, -2.0)])
        self.assertEqual([c[0] for c in self.calls], ['turn'] * 3 + ['translate', 'scan match'])

    def test_still_far_from_the_station_by_the_map_is_a_clear_abort(self):
        with self.assertRaises(SystemExit) as cm:
            self.approach([(-0.70, -0.15, 0.5), (-0.45, -0.10, 0.5)])        # after the move: still 0.46 m away
        self.assertIn('still 0.46 m from T', str(cm.exception))
        self.assertNotIn(('scan match',), self.calls)

    def test_large_heading_error_still_aborts(self):
        with self.assertRaises(SystemExit) as cm:
            self.approach([(-0.70, -0.15, 14.0)])
        self.assertIn('heading 14.0 deg off', str(cm.exception))


if __name__ == '__main__':
    unittest.main()

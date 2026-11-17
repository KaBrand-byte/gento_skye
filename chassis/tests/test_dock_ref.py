"""Docking by the reference scan with a recorded background (dock.py dock_by_ref), simulated: ROS is stubbed, the
robot moves exactly as commanded and the lidar sees a conveyor (front face, belt top) from wherever it stands."""
import math
import os
import shutil
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CHASSIS = os.path.dirname(HERE)


def load_dock(tmp):
    """dock.py's definitions (everything before the command dispatch) with ROS stubbed, installed in `tmp`."""
    for m in ('rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg', 'sensor_msgs_py',
              'move', 'move.msg', 'move.srv', 'move.action'):
        sys.modules.setdefault(m, mock.MagicMock())
    src = open(os.path.join(CHASSIS, 'dock.py'), encoding='utf-8').read().split('\ncmd = sys.argv[1]')[0]
    for f in ('robot_model.py', 'route.py'):
        shutil.copy(os.path.join(CHASSIS, f), tmp)
    sys.path.insert(0, CHASSIS)
    ns = {'__file__': os.path.join(tmp, 'dock.py'), '__name__': 'dock_sim'}
    exec(compile(src, 'dock.py', 'exec'), ns)
    return ns


def grid(x0, x1, y0, y1, z0, z1, step=0.02):
    xs, ys, zs = np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step), np.arange(z0, z1 + 1e-9, step)
    return np.array(np.meshgrid(xs, ys, zs)).reshape(3, -1).T


class Sim:
    """Station frame = the reference pose.  Conveyor face at x = FRONT + ref_gap, belt top 0.82 m."""

    def __init__(self, ns, ref_gap=0.123):
        self.ns = ns; self.face = ns['FRONT'] + ref_gap
        f = self.face
        self.world = np.vstack([grid(f, f, -0.8, 1.6, 0.10, 0.80),            # front face of the conveyor
                                grid(f, f + 0.6, -0.8, 1.6, 0.82, 0.82),      # belt top
                                grid(f + 0.62, f + 0.62, -0.8, 1.6, 0.30, 0.80)])
        self.pose = [0.0, 0.0, 0.0]; self.moves = []

    def add(self, pts):
        self.world = np.vstack([self.world, pts])

    def robot_points(self):
        x, y, a = self.pose; c, s = math.cos(-a), math.sin(-a)
        q = self.world.copy(); dx, dy = q[:, 0] - x, q[:, 1] - y
        q[:, 0], q[:, 1] = c * dx - s * dy, s * dx + c * dy
        return q

    def to_go(self):
        x, y, a = self.pose; c, s = math.cos(-a), math.sin(-a)
        return (c * -x - s * -y, s * -x + c * -y, -a)

    # stand-ins for the dock.py functions that talk to the robot
    def ref_offset(self, name, seed=None):
        x, y, a = self.pose; c, s = math.cos(a), math.sin(a)
        self.ns['LAST_ICP'][name] = (np.array([[c, -s], [s, c]]), np.array([x, y]), self.ns['MOVES'][0])
        return self.to_go()

    def rel(self, x=0.0, yaw=0.0, rear_check=False, y=0.0):
        assert not (abs(x) > 0.001 and abs(y) > 0.001)
        px, py, a = self.pose
        self.pose = [px + math.cos(a) * x - math.sin(a) * y, py + math.sin(a) * x + math.cos(a) * y, a + yaw]
        self.ns['MOVES'][0] += 1; self.moves.append((round(x, 3), round(y, 3), round(math.degrees(yaw), 1)))

    def measure(self, near=False):
        return dict(gap=round(self.face - self.ns['FRONT'] - self.pose[0], 4), ang_deg=0.0, dist=self.face - self.pose[0])


class DockByRefTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        ns = self.ns = load_dock(self.tmp)
        os.makedirs(ns['REF_DIR'], exist_ok=True)
        np.save(os.path.join(ns['REF_DIR'], 'D.npy'), np.zeros((10, 3)))
        sim = self.sim = Sim(ns)
        ns.update(ref_offset=sim.ref_offset, rel=sim.rel, measure=sim.measure, cloud=lambda fresh=False: sim.robot_points(),
                  rel_target=lambda pose: sim.to_go(), spin=lambda t=0: None)
        patcher = mock.patch.object(ns['RM'], 'remove_self', lambda p, cfg, phase='carry': p)   # no own points here
        patcher.start(); self.addCleanup(patcher.stop)
        real_open = open
        ns['open'] = lambda f, *a, **k: real_open(os.path.join(self.tmp, os.path.basename(f)) if str(f).startswith('/tmp/') else f, *a, **k)
        self.st = dict(gap=0.0505, ref_gap=0.123, place=True, pose=[0, 0, 0], side='left', end_y=0.0, z=[0.1, 0.7])
        ns['ST_FILE'] = os.path.join(self.tmp, 'stations.json')
        import json
        json.dump({'D': self.st}, open(ns['ST_FILE'], 'w'))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def dock(self, start, record=False):
        self.sim.pose = list(start); self.sim.moves = []; self.ns['SURF'][0] = None
        with self.assertRaises(SystemExit) as cm:
            self.ns['dock_by_ref']('D', dict(self.st), self.st['gap'], record=record)
        return cm.exception

    def test_docks_from_an_offset_safe_point_without_sideways_moves(self):
        e = self.dock([-0.60, 0.11, math.radians(3)], record=True)
        self.assertEqual(e.code, 0)
        x, y, a = self.sim.pose
        self.assertAlmostEqual(x, 0.123 - 0.0505, delta=0.02)          # final pose 7.25 cm beyond the reference
        self.assertLess(abs(y), 0.016); self.assertLess(abs(math.degrees(a)), 1.2)
        self.assertFalse([m for m in self.sim.moves if abs(m[1]) > 0.001], self.sim.moves)   # no sideways move
        self.assertTrue(os.path.exists(self.ns['bg_file']('D')))

    def test_only_new_objects_block_and_the_place_zone_is_never_background(self):
        self.assertEqual(self.dock([-0.60, 0.05, 0.0], record=True).code, 0)
        # a tray left on the belt where the next one goes: new -> blocked (waits, then BLOCKED)
        self.sim.add(grid(self.sim.face + 0.10, self.sim.face + 0.40, -0.15, 0.15, 0.84, 0.90))
        self.ns['BLOCK_WAIT'] = 0.0
        e = self.dock([-0.60, 0.05, 0.0])
        self.assertIn('BLOCKED', str(e))
        # recording the background with it there does not make it background
        self.ns['add_background']('D', self.st, 'test')
        e = self.dock([-0.60, 0.05, 0.0])
        self.assertIn('BLOCKED', str(e))

    def test_static_structure_next_to_the_station_does_not_block_once_recorded(self):
        # fixed structure the model touches (here: a low frame part right in front of the conveyor, beside the
        # line): blocks without a background, not once it was recorded as background
        leg = grid(self.sim.face - 0.04, self.sim.face - 0.02, 0.24, 0.28, 0.10, 0.40)
        self.sim.add(leg)
        self.ns['BLOCK_WAIT'] = 0.0
        self.assertIn('BLOCKED', str(self.dock([-0.60, 0.0, 0.0])))
        self.sim.pose = [-0.60, 0.0, 0.0]; self.ns['MOVES'][0] += 1
        self.ns['add_background']('D', self.st, 'test')
        self.assertEqual(self.dock([-0.60, 0.0, 0.0]).code, 0)

    def test_guarded_waits_for_the_object_to_go(self):
        calls = []

        def check():
            calls.append(1)
            if len(calls) < 3:
                raise SystemExit('ABORT: x blocked by obstacle {}')
            return 'ok'
        self.ns['BLOCK_WAIT'] = 60.0
        self.assertEqual(self.ns['guarded'](check, 'x'), 'ok'); self.assertEqual(len(calls), 3)

    def test_prediction_after_a_move(self):
        d = self.ns['_after_move']((0.5, 0.1, 0.0), 0.2, 0.1, math.radians(10))
        self.assertAlmostEqual(d[0], 0.3 * math.cos(math.radians(10)), places=6)
        self.assertAlmostEqual(d[1], -0.3 * math.sin(math.radians(10)), places=6)


if __name__ == '__main__':
    unittest.main()

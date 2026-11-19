"""Chassis health check (doctor.py): the verdict for each failure mode seen so far."""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import doctor  # noqa: E402

HEALTHY = dict(uptime=900.0, load=9.0, ncpu=4, service='active', bridge='active', missing=[], carrier_changes=1,
               job='idle', lidar=50, state=40, pose=50, hard_auto=True, conf=85, relocating=False, match=0.95)


def levels(**changes):
    f = dict(HEALTHY); f.update(changes)
    return [(lv, text) for lv, text, _ in doctor.evaluate(f)]


def fails(**changes):
    return [t for lv, t in levels(**changes) if lv == 'fail']


def warns(**changes):
    return [t for lv, t in levels(**changes) if lv == 'warn']


class DoctorTest(unittest.TestCase):
    def test_healthy_chassis_passes(self):
        self.assertEqual(fails(), []); self.assertEqual(warns(), [])

    def test_vendor_service_down_like_2026_11_18(self):
        # ping and ssh worked, but no lidar, no chassis state, no pose
        f = fails(service='inactive', missing=list(doctor.VENDOR), lidar=0, state=0, pose=0, match=None,
                  hard_auto=None, conf=None)
        self.assertTrue(any('linkhou_robot' in t for t in f))
        self.assertTrue(any('激光' in t for t in f) and any('move/State' in t for t in f))
        fix = [x for lv, t, x in doctor.evaluate(dict(HEALTHY, service='inactive')) if 'linkhou_robot' in t][0]
        self.assertIn('断电重启', fix)

    def test_eth1_drops_like_2026_11_17_are_a_warning(self):
        w = warns(carrier_changes=79, tx_hang=2)
        self.assertTrue(any('eth1 掉线 39 次' in t and '卡死 2 次' in t for t in w), w)
        self.assertEqual(fails(carrier_changes=79), [])

    def test_hard_switch_manual_and_bad_localisation_block_the_start(self):
        self.assertTrue(any('MANUAL' in t for t in fails(hard_auto=False)))
        self.assertTrue(any('匹配度 0.36' in t for t in fails(match=0.36)))
        self.assertTrue(any('匹配度 0.70' in t for t in warns(match=0.70)))

    def test_just_booted_and_bridge_not_up(self):
        self.assertTrue(any('开机才 40 s' in t for t in warns(uptime=40.0)))
        self.assertTrue(any('gento-dock-bridge' in t for t in fails(bridge='inactive')))

    def test_few_messages_or_overload_are_warnings(self):
        self.assertTrue(any('只收到 3 条' in t for t in warns(lidar=3)))
        self.assertTrue(any('负载很高' in t for t in warns(load=14.0)))
        self.assertTrue(any('作业正在运行' in t for t in warns(job='running')))


if __name__ == '__main__':
    unittest.main()

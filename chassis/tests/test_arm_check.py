"""Read-only arm / lift check (client/arm_check.py): the verdicts, with the controller's feedback faked."""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'client'))
import arm_check  # noqa: E402
from skye_task.config import load_config  # noqa: E402

CONFIG = load_config(os.path.join(ROOT, 'config', 'default.json'))
OK_STATE = {'err': 0, 'cur': 1}


def facts(**changes):
    f = dict(states={'左臂': OK_STATE, '右臂': OK_STATE, 'body': OK_STATE, 'head': OK_STATE, 'lift': OK_STATE},
             lift=CONFIG.receive_lift[0], lift_moving=False, body=list(CONFIG.body), head=list(CONFIG.head),
             joints=[list(a) for a in CONFIG.receive], traj_states=[0, 0])
    f.update(changes)
    return f


def verdict(**changes):
    return [(lv, t) for lv, t, _ in arm_check.evaluate(facts(**changes), CONFIG)]


class ArmCheckTest(unittest.TestCase):
    def test_ready_for_a_run(self):
        res = verdict()
        self.assertTrue(all(lv == 'ok' for lv, _ in res), res)
        self.assertTrue(any('升降高度 700.0 mm' in t for _, t in res))

    def test_lift_away_from_the_carry_height_fails_like_the_task_start(self):
        res = verdict(lift=CONFIG.receive_lift[0] - 35.0)
        self.assertTrue(any(lv == 'fail' and '升降高度 665.0 mm' in t for lv, t in res), res)
        self.assertFalse(any(lv == 'fail' for lv, _ in verdict(lift=CONFIG.receive_lift[0] + 0.6)))

    def test_moving_lift_and_axis_error(self):
        self.assertTrue(any(lv == 'warn' and '升降正在运动' in t for lv, t in verdict(lift_moving=True)))
        res = verdict(states={'左臂': OK_STATE, '右臂': OK_STATE, 'body': OK_STATE, 'head': OK_STATE,
                              'lift': {'err': 5, 'cur': 1}})
        self.assertTrue(any(lv == 'fail' and 'lift(err=5' in t for lv, t in res), res)

    def test_arm_poses(self):
        ready = [list(a) for a in CONFIG.ready]
        self.assertTrue(any(lv == 'warn' and '准备位' in t for lv, t in verdict(joints=ready)))
        odd = [[a + 20 for a in arm] for arm in CONFIG.receive]
        self.assertTrue(any(lv == 'fail' and '任务会拒绝开始' in t for lv, t in verdict(joints=odd)))
        wrist = [list(arm) for arm in CONFIG.receive]; wrist[0][6] += 5
        self.assertTrue(any(lv == 'ok' and '只转动腕部' in t for lv, t in verdict(joints=wrist)))


if __name__ == '__main__':
    unittest.main()

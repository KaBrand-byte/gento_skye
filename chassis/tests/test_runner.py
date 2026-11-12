"""Offline test of the full-trial runner with skye_task's own test double (no robot, ROS or SSH).

Run from the repository root:  python3 -m unittest discover -s chassis/tests -v
"""

from dataclasses import dataclass
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "chassis" / "client"))

import run_task_with_navigation as runner  # noqa: E402
runner.LINK_RELEASE_S = 0.0
from gento_navigation import NavigationError  # noqa: E402
from test_sequence import Device  # noqa: E402  (skye_task's offline robot double)


class FakeNav:
    log = []

    def __init__(self, fail_on=None, **kwargs):
        self.fail_on, self.active, self.count = fail_on, None, 0
        FakeNav.log.append(("init", kwargs.get("gaps")))

    def start(self, destination):
        FakeNav.log.append(("start", destination)); self.active, self.count = destination, 0

    def is_arrived_and_stopped(self):
        self.count += 1
        return self.count > 2 and self.active != self.fail_on

    def raise_if_failed(self):
        if self.active == self.fail_on and self.count > 1:
            raise NavigationError("ABORT: blocked")

    def start_undock(self):
        FakeNav.log.append(("start", "undock")); self.active, self.count = "undock", 0

    def job_status(self):
        return {"state": "done", "job": self.active, "final": "FINAL {}"}

    def pose(self):
        return [0.0, 0.0, 0.0]

    def stop(self):
        FakeNav.log.append(("stop", self.active))

    def selfscan(self):
        FakeNav.log.append(("selfscan",)); return "ok"

    def close(self):
        pass


@dataclass
class FakeGripperConfig:
    def action_positions(self, action, side):
        return {"left": 0.0 if action == "open" else 0.94, "right": 0.0 if action == "open" else 0.94}


class FakeGripper:
    """Records gripper actions into the same log as the navigation calls (order matters)."""
    worker_running = False

    def __init__(self):
        self.config = FakeGripperConfig()

    def reserve(self):
        pass

    def connect(self):
        pass

    def control(self, target, action, checkpoint=lambda: None):
        checkpoint()
        FakeNav.log.append(("grip", getattr(action, "value", action)))
        return {}

    def check_health(self):
        pass

    def freeze(self):
        FakeNav.log.append(("grip", "freeze"))

    def snapshot(self):
        return {}

    def close(self):
        pass


def make_sequence_with_fake_gripper(self, sdk, config, label=""):
    self.gripper, self._sdk = FakeGripper(), sdk
    return runner.Sequence(sdk, config, gripper=self.gripper)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg = Path(self.tmp.name) / "fast.json"
        self.cfg.write_text(json.dumps({"stable_seconds": 0.04, "start_timeout_seconds": 0.15,
                                        "execution_timeout_seconds": 0.5, "poll_seconds": 0.02,
                                        "gripper_enabled": False, "handover_wait_seconds": 0.05,
                                        "gripper_wait_seconds": 0.05}))
        self.result = Path(self.tmp.name) / "result.json"
        FakeNav.log = []

    def tearDown(self):
        self.tmp.cleanup()

    def run_trial(self, fail_on=None, extra=(), arms=None):
        def device(config):
            d = Device(config)
            d.close = lambda: None                                           # the real GentoSDK has close()
            for i, arm in enumerate(d.rt["arms"]):
                arm["fb"]["fb_pos"] = list((arms or config.ready)[i])
            return d
        with mock.patch.object(runner, "GentoSDK", side_effect=device), \
             mock.patch.object(runner, "GentoNavigation", side_effect=lambda **kw: FakeNav(fail_on=fail_on, **kw)), \
             mock.patch.object(runner, "wait_for_start", lambda seq: None):
            code = runner.main(["--config", str(self.cfg), "--result", str(self.result), "--poll", "0.01",
                                "--place-gap", "0.3", *extra])
        return code, json.loads(self.result.read_text(encoding="utf-8"))

    def test_full_trial_order(self):
        code, report = self.run_trial(extra=["--selfscan"])
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["navigation"], "arrived")
        self.assertEqual([l["destination"] for l in report["metadata"]["navigation_legs"]], ["T", "D", "undock", "T"])
        self.assertEqual(FakeNav.log, [("init", {"D": 0.3}), ("selfscan",), ("start", "T"), ("start", "D"),
                                       ("start", "undock"), ("start", "T")])
        names = [s["name"] for s in report["steps"]]
        self.assertLess(names.index("receive_lift"), names.index("forward"))   # carry pose held while driving
        self.assertEqual(names[-2:], ["back_to_carry", "finished"])   # straight back into the carry pose, then drive

    def test_grippers_close_at_the_start_station_before_driving_to_the_place_station(self):
        data = json.loads(self.cfg.read_text()); data["gripper_enabled"] = True
        self.cfg.write_text(json.dumps(data))
        with mock.patch.object(runner.Trial, "make_sequence", make_sequence_with_fake_gripper):
            code, report = self.run_trial()
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        events = [e for e in FakeNav.log if e[0] in ("grip", "start")]
        self.assertEqual(events, [("grip", "open"), ("start", "T"), ("grip", "close"), ("start", "D"),
                                  ("grip", "open"), ("start", "undock"), ("start", "T")])
        names = [s["name"] for s in report["steps"]]
        placing = [n for n in names if n in ("forward", "down", "release", "up", "back_to_carry")]
        self.assertEqual(list(dict.fromkeys(placing)), ["forward", "down", "release", "up", "back_to_carry"])
        self.assertNotIn("return_transition", names)                     # no detour over transition / ready

    def test_starting_in_the_carry_pose_does_not_move_the_arms(self):
        from skye_task.config import load_config
        receive = load_config(self.cfg).receive
        code, report = self.run_trial(arms=receive)
        self.assertEqual(code, 0)
        names = [s["name"] for s in report["steps"]]
        self.assertNotIn("ready", names)
        self.assertNotIn("receive", names)
        self.assertEqual(FakeNav.log[1:3], [("start", "T"), ("start", "D")])

    def test_refuses_to_start_with_the_arms_somewhere_else(self):
        code, report = self.run_trial(arms=[[10.0] * 7, [-10.0] * 7])
        self.assertEqual(code, 1)
        self.assertNotIn(("start", "T"), FakeNav.log)                     # nothing moved

    def test_down_distance_can_be_set_for_one_run(self):
        code, report = self.run_trial(extra=["--down-mm", "120"])
        self.assertEqual(code, 0)
        self.assertEqual(report["metadata"]["place_motion_mm"]["down"], 120)
        self.assertEqual(report["metadata"]["place_motion_mm"]["up"], 120)
        down = [s for s in report["steps"] if s["name"] == "down_target"][0]
        self.assertEqual(down["robot_displacements_mm"], [[0, 0, -120], [0, 0, -120]])

    def test_descent_beyond_the_arms_reach_is_finished_by_the_lift(self):
        orig = Device.plan_linear

        def limited(dev, feedback, starts, targets, motion):          # like the real arm: no deep reach
            if max(abs(targets[a][i][3] - starts[a][i][3]) for a in range(2) for i in range(3)) > 140:
                raise runner.MotionError("Target is out of workspace")
            return orig(dev, feedback, starts, targets, motion)
        with mock.patch.object(Device, "plan_linear", limited):
            code, report = self.run_trial(extra=["--down-mm", "150"])
        self.assertEqual(code, 0)
        plan = report["metadata"]["place_motion_mm"]
        self.assertEqual(plan["down"], 150)
        self.assertEqual(plan["arm_down"] + plan["lift_down"], 150)
        self.assertGreater(plan["lift_down"], 0)
        names = list(dict.fromkeys(s["name"] for s in report["steps"]))
        i = names.index
        self.assertLess(i("down"), i("place_lift_down"))
        self.assertLess(i("place_lift_down"), i("release"))
        self.assertLess(i("release"), i("place_lift_up"))
        self.assertLess(i("place_lift_up"), i("up"))

    def test_no_return(self):
        code, report = self.run_trial(extra=["--no-return"])
        self.assertEqual(code, 0)
        self.assertEqual([l["destination"] for l in report["metadata"]["navigation_legs"]], ["T", "D"])

    def test_failure_on_the_way_back_keeps_the_placement_result(self):
        code, report = self.run_trial(fail_on="undock")
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["metadata"]["return_status"], "failed")
        self.assertIn(("stop", "undock"), FakeNav.log)

    def test_failure_at_the_place_station_stops_chassis_and_arms_without_placing(self):
        code, report = self.run_trial(fail_on="D")
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "failed")
        self.assertIn(("stop", "D"), FakeNav.log)
        self.assertNotIn("forward", [s["name"] for s in report["steps"]])
        self.assertEqual(report["metadata"]["navigation_legs"][-1]["status"], "failed")


if __name__ == "__main__":
    unittest.main()

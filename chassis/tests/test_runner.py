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
        FakeNav.log.append(("init", kwargs.get("gaps"))); FakeNav.route = kwargs.get("route")

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


class BlockedOnceNav(FakeNav):
    """The first docking at D ends BLOCKED (an object in the way, the chassis gave up waiting); the retry works."""
    def __init__(self, **kw):
        super().__init__(**kw); self.blocked = False

    def raise_if_failed(self):
        if self.active == "D" and not self.blocked and self.count > 1:
            self.blocked = True
            raise NavigationError("ABORT: BLOCKED: place spot under the tray blocked by obstacle {} - still there after 30 s")

    def is_arrived_and_stopped(self):
        self.count += 1
        return self.count > 2 and (self.active != "D" or self.blocked)


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

    def run_trial(self, fail_on=None, extra=(), arms=None, nav_cls=None, answers=("", "1"), tty=True, lift=None):
        """`answers`: what the operator types, in order (handover: o / c / Enter; after placing: 1 / 0, r ...)."""
        typed = iter(answers)
        self.typed = typed

        def read_answer(seq):
            seq.checked_feedback()
            return next(typed)
        def device(config):
            d = Device(config)
            d.close = lambda: None                                           # the real GentoSDK has close()
            for i, arm in enumerate(d.rt["arms"]):
                arm["fb"]["fb_pos"] = list((arms or config.ready)[i])
            if lift is not None:
                d.rt["lift"]["fb_pos"] = [lift]
            return d
        with mock.patch.object(runner, "GentoSDK", side_effect=device), \
             mock.patch.object(runner, "GentoNavigation", side_effect=lambda **kw: (nav_cls or FakeNav)(fail_on=fail_on, **kw)), \
             mock.patch.object(runner, "wait_for_start", lambda seq: None),              mock.patch.object(runner, "read_answer", read_answer),              mock.patch.object(runner.sys.stdin, "isatty", lambda: tty):
            code = runner.main(["--config", str(self.cfg), "--result", str(self.result), "--poll", "0.01",
                                "--place-gap", "0.3", *extra])
        return code, json.loads(self.result.read_text(encoding="utf-8"))

    def test_full_trial_order(self):
        code, report = self.run_trial(extra=["--selfscan"])
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["navigation"], "arrived")
        # the handover happens where the robot stands (T): no driving to T first, straight to D
        self.assertEqual([l["destination"] for l in report["metadata"]["navigation_legs"]], ["D", "undock", "T"])
        self.assertEqual(FakeNav.log, [("init", {"D": 0.3}), ("selfscan",), ("start", "D"),
                                       ("start", "undock"), ("start", "T")])
        names = [s["name"] for s in report["steps"]]
        self.assertLess(names.index("receive_lift"), names.index("forward"))   # carry pose held while driving
        self.assertEqual(names[-2:], ["back_to_carry", "finished"])   # straight back into the carry pose, then drive

    def with_grippers(self, **kw):
        from skye_task.config import load_config
        data = json.loads(self.cfg.read_text()); data["gripper_enabled"] = True
        self.cfg.write_text(json.dumps(data))
        kw.setdefault("arms", load_config(self.cfg).receive)
        with mock.patch.object(runner.Trial, "make_sequence", make_sequence_with_fake_gripper):
            code, report = self.run_trial(**kw)
        return code, report, [e for e in FakeNav.log if e[0] in ("grip", "start")]

    def test_handover_enter_closes_and_1_goes(self):
        code, report, events = self.with_grippers(answers=("", "1"))
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(events, [("grip", "close"), ("start", "D"), ("grip", "open"), ("start", "undock"),
                                  ("start", "T")])
        self.assertEqual(report["metadata"]["handover_feedback"], ["close", "1"])
        self.assertNotIn("ready", [s["name"] for s in report["steps"]])       # arms not moved for the handover

    def test_handover_0_opens_the_grippers_to_put_the_tray_right(self):
        # 0: not held right -> grippers open; Enter closes again; a stray Enter while closed does nothing; 1 goes
        code, report, events = self.with_grippers(answers=("", "0", "", "", "1"))
        self.assertEqual(code, 0)
        self.assertEqual(report["metadata"]["handover_feedback"], ["close", "0", "close", "1"])
        self.assertEqual(events, [("grip", "close"), ("grip", "open"), ("grip", "close"), ("start", "D"),
                                  ("grip", "open"), ("start", "undock"), ("start", "T")])
        self.assertEqual(list(self.typed), [])                             # every answer was read
        done = [s["name"] for s in report["steps"] if s.get("status") == "completed"]
        self.assertEqual(done.count("forward"), 1)                         # placing at D is unchanged: once

    def test_handover_manual_keys_and_a_tray_closed_before_the_run(self):
        code, report, events = self.with_grippers(answers=("o", "c", "1"))
        self.assertEqual(events[:3], [("grip", "open"), ("grip", "close"), ("start", "D")])
        FakeNav.log = []
        code, report, events = self.with_grippers(answers=("1",))           # gento grip close done before
        self.assertEqual(code, 0)
        self.assertEqual(events[:2], [("grip", "close"), ("start", "D")])   # closed again: hold torque on the way

    def test_stop_during_the_handover_does_not_drive(self):
        code, report, events = self.with_grippers(answers=("", "0"))     # then Ctrl-C (no more input)
        self.assertNotEqual(code, 0)
        self.assertNotIn(("start", "D"), FakeNav.log)

    def test_no_terminal_asks_nothing(self):
        code, report = self.run_trial(answers=(), tty=False)
        self.assertEqual(code, 0)
        self.assertNotIn("handover_feedback", report["metadata"])

    def test_handover_option_drives_to_the_start_station_first(self):
        data = json.loads(self.cfg.read_text()); data["gripper_enabled"] = True
        self.cfg.write_text(json.dumps(data))
        with mock.patch.object(runner.Trial, "make_sequence", make_sequence_with_fake_gripper):
            code, report = self.run_trial(extra=["--handover"])
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
        self.assertEqual(FakeNav.log[1:3], [("start", "D"), ("start", "undock")])

    def test_refuses_to_start_with_the_arms_somewhere_else(self):
        code, report = self.run_trial(arms=[[10.0] * 7, [-10.0] * 7])
        self.assertEqual(code, 1)
        self.assertNotIn(("start", "D"), FakeNav.log)                     # nothing moved

    def test_lift_lowers_the_tray_after_the_arms(self):
        code, report = self.run_trial(extra=["--down-mm", "120", "--place-lift-mm", "50"])
        self.assertEqual(code, 0)
        plan = report["metadata"]["place_motion_mm"]
        self.assertEqual((plan["arm_down"], plan["lift_down"], plan["total_down"]), (120, 50, 170))
        lift = [s for s in report["steps"] if s["name"] == "place_lift_down" and s.get("status") == "started"][0]
        self.assertEqual(lift["target_mm"], runner.load_config(self.cfg).receive_lift[0] - 50)
        code, report = self.run_trial(extra=["--down-mm", "120", "--place-lift-mm", "0"])
        self.assertEqual(report["metadata"]["place_motion_mm"]["lift_down"], 0)
        self.assertFalse(any(s["name"] == "place_lift_down" for s in report["steps"]))

    def test_basket_descent_by_default(self):
        code, report = self.run_trial()
        self.assertEqual(code, 0)
        plan = report["metadata"]["place_motion_mm"]
        self.assertEqual((plan["down"], plan["up"], plan["total_down"], plan["lift_down"]),
                         (runner.PLACE_DOWN_MM, runner.PLACE_DOWN_MM, 135, 0))   # 5 cm less than the tray's 185
        self.assertFalse(any(s["name"] == "place_lift_down" for s in report["steps"]))

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
        self.assertEqual(plan["lift_down"], max(150 - plan["arm_down"], runner.PLACE_LIFT_MM))
        self.assertEqual(plan["total_down"], plan["arm_down"] + plan["lift_down"])
        self.assertEqual(plan["arm_up"], plan["arm_down"])                 # arms back up by what they went down
        names = list(dict.fromkeys(s["name"] for s in report["steps"]))
        i = names.index
        self.assertLess(i("down"), i("place_lift_down"))
        self.assertLess(i("place_lift_down"), i("release"))
        self.assertLess(i("release"), i("place_lift_up"))
        self.assertLess(i("place_lift_up"), i("up"))

    def test_start_in_the_carry_pose_opens_the_grippers_only_at_the_start_station(self):
        from skye_task.config import load_config
        data = json.loads(self.cfg.read_text()); data["gripper_enabled"] = True
        self.cfg.write_text(json.dumps(data))
        receive = load_config(self.cfg).receive
        with mock.patch.object(runner.Trial, "make_sequence", make_sequence_with_fake_gripper):
            code, report = self.run_trial(arms=receive, extra=["--handover"])
        self.assertEqual(code, 0)
        events = [e for e in FakeNav.log if e[0] in ("grip", "start")]
        self.assertEqual(events[:3], [("start", "T"), ("grip", "open"), ("grip", "close")])

    def test_holding_never_opens_before_the_place_station(self):
        from skye_task.config import load_config
        data = json.loads(self.cfg.read_text()); data["gripper_enabled"] = True
        self.cfg.write_text(json.dumps(data))
        receive = load_config(self.cfg).receive
        with mock.patch.object(runner.Trial, "make_sequence", make_sequence_with_fake_gripper):
            code, report = self.run_trial(arms=receive, extra=["--holding"])
        self.assertEqual(code, 0)
        events = [e for e in FakeNav.log if e[0] in ("grip", "start")]
        # closed once more (same position, the hold torque stays on while driving), never opened before D
        self.assertEqual(events, [("grip", "close"), ("start", "D"), ("grip", "open"), ("start", "undock"),
                                  ("start", "T")])

    def test_older_carry_pose_only_turns_the_wrists(self):
        from skye_task.config import load_config
        receive = load_config(self.cfg).receive
        old = [list(arm[:5]) + [arm[5] + 6.2, arm[6] + 1.8 * (1 if i == 0 else -1)] for i, arm in enumerate(receive)]
        code, report = self.run_trial(arms=old)
        self.assertEqual(code, 0)
        names = [s["name"] for s in report["steps"]]
        self.assertIn("carry_wrists", names)
        self.assertNotIn("ready", names)                                   # no prepare(): not via the ready pose
        self.assertLess(names.index("carry_wrists"), names.index("forward"))
        moved = [s for s in report["steps"] if s["name"] == "carry_wrists" and "targets" in s]
        for step in moved:
            self.assertEqual(step["targets"], [list(a) for a in receive])
        # another joint off as well: refused, nothing moves
        FakeNav.log = []
        bad = [list(a) for a in old]; bad[0][1] += 3.0
        code, report = self.run_trial(arms=bad)
        self.assertEqual(code, 1)
        self.assertNotIn("carry_wrists", [s["name"] for s in report["steps"]])
        self.assertNotIn(("start", "D"), FakeNav.log)

    def interrupted_place_joints(self):
        """Run a trial whose place stops at the lift step (like the basket pressing against the belt); return
        the arm joints it stopped with (arms forward and down)."""
        devices = []
        orig_device = runner.GentoSDK

        def failing_lift(seq, name, target):
            raise runner.MotionError("升降调整期间双臂未保持静止")
        with mock.patch.object(runner.Sequence, "_move_lift", failing_lift):
            real = Device

            def capture(config):
                d = real(config); devices.append(d); return d
            with mock.patch(__name__ + ".Device", side_effect=capture):
                code, report, _ = self.with_grippers(extra=["--place-lift-mm", "50"])   # the tray's lift step
        self.assertEqual(code, 1)
        self.assertIn("未保持静止", report["error"])
        self.arm_down = report["metadata"]["place_motion_mm"]["arm_down"]
        return [list(arm["fb"]["fb_pos"]) for arm in devices[0].rt["arms"]]   # the sequence's link (created first)

    def test_finish_an_interrupted_place_releases_retracts_and_returns(self):
        down = self.interrupted_place_joints()
        FakeNav.log = []
        lowered = runner.load_config(self.cfg).receive_lift[0] - 33         # the lift stopped on its way down
        code, report, events = self.with_grippers(arms=down, extra=["--finish-place", str(self.arm_down)],
                                                  answers=(), lift=lowered)
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(events, [("grip", "open"), ("start", "undock"), ("start", "T")])   # released first
        done = [s["name"] for s in report["steps"] if s.get("status") == "completed"]
        self.assertLess(done.index("release"), done.index("place_lift_up"))
        self.assertLess(done.index("place_lift_up"), done.index("up"))
        self.assertLess(done.index("up"), done.index("back_to_carry"))
        self.assertNotIn("forward", done)

    def test_finish_place_is_refused_unless_it_ends_at_the_carry_pose(self):
        down = self.interrupted_place_joints()
        FakeNav.log = []
        code, report, events = self.with_grippers(arms=down, extra=["--finish-place", "0"], answers=())
        self.assertEqual(code, 1)
        self.assertIn("只读规划不通过", report["error"])
        self.assertEqual([e for e in events if e != ("grip", "freeze")], [])   # nothing opened, nothing driven

    def test_holding_requires_the_carry_pose(self):
        code, report = self.run_trial(extra=["--holding"])                  # arms at ready
        self.assertEqual(code, 1)
        self.assertNotIn(("start", "D"), FakeNav.log)

    def test_route_is_fixed_unless_asked(self):
        self.run_trial()
        self.assertFalse(any(FakeNav.route.values()))

    def test_random_route_asks_for_a_random_waypoint_and_jitter(self):
        code, _ = self.run_trial(extra=["--random-route"])
        self.assertEqual(code, 0)
        self.assertEqual(FakeNav.route, {"via": "random", "jitter": runner.RANDOM_ROUTE_JITTER})
        self.run_trial(extra=["--random-route", "--via", "W2", "--jitter", "0"])
        self.assertEqual(FakeNav.route, {"via": "W2", "jitter": 0.0})

    def test_no_return(self):
        code, report = self.run_trial(extra=["--no-return"])
        self.assertEqual(code, 0)
        self.assertEqual([l["destination"] for l in report["metadata"]["navigation_legs"]], ["D"])

    def test_failure_on_the_way_back_keeps_the_placement_result(self):
        code, report = self.run_trial(fail_on="undock")
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["metadata"]["return_status"], "failed")
        self.assertIn(("stop", "undock"), FakeNav.log)

    def test_blocked_docking_is_retried_after_enter_on_a_terminal(self):
        code, report = self.run_trial(nav_cls=BlockedOnceNav)
        self.assertEqual(code, 0)
        self.assertEqual([e for e in FakeNav.log if e[0] == "start"],
                         [("start", "D"), ("start", "D"), ("start", "undock"), ("start", "T")])

    def test_blocked_docking_without_a_terminal_fails(self):
        code, report = self.run_trial(nav_cls=BlockedOnceNav, tty=False)
        self.assertEqual(code, 1)

    def test_failure_at_the_place_station_stops_chassis_and_arms_without_placing(self):
        code, report = self.run_trial(fail_on="D")
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "failed")
        self.assertIn(("stop", "D"), FakeNav.log)
        self.assertNotIn("forward", [s["name"] for s in report["steps"]])
        self.assertEqual(report["metadata"]["navigation_legs"][-1]["status"], "failed")


if __name__ == "__main__":
    unittest.main()

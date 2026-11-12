"""Task integration tests; no SDK library or physical gripper is used."""

from contextlib import redirect_stdout
from dataclasses import replace
from io import StringIO
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

from skye_task.__main__ import main
from skye_task.config import Config
from skye_task.dm_gripper import DMGripperConfig, load_gripper_config
from skye_task.dm_protocol import DISABLE, ENABLE
from skye_task.interfaces import GripperAction, GripperTarget
from skye_task.sdk import MotionError
from skye_task.sequence import Sequence, Stopped
from skye_task.task_gripper import TaskGripper
from test_dm_gripper import MotorBus
from test_sequence import Clock, Device


class TaskGripperTests(unittest.TestCase):
    def setUp(self):
        # Keep drift samples independent of user tuning in the runtime JSON.
        self.config = replace(load_gripper_config("config/gripper-can.json"),
                              position_tolerance_rad=.08, hold_position_tolerance_rad=.15,
                              hold_error_seconds=.5,
                              stable_seconds=.03, motion_timeout_seconds=1,
                              feedback_timeout_seconds=.3, startup_timeout_seconds=.15)
        self.sdk = MagicMock()
        self.sdk.config = Config()
        self.sdk.connected = True
        self.buses = {}
        self.gripper = TaskGripper(self.sdk, self.config)
        def transport(side):
            bus = MotorBus(side)
            self.buses[side.interface] = bus
            return bus
        self.gripper.connection.transport = transport
        self.gripper.connect()
        self.addCleanup(self.gripper.close)

    def test_task_uses_existing_sdk_keeps_sending_and_closes_without_release(self):
        self.gripper.config = replace(self.config,
                                      left=replace(self.config.left, open_position_rad=.45),
                                      right=replace(self.config.right, open_position_rad=.45))
        self.gripper.control(GripperTarget.BOTH, GripperAction.OPEN)
        before = sum(len(bus.sent) for bus in self.buses.values())
        # Represents a blocking arm planning/upload call that releases the GIL.
        time.sleep(.05)
        self.gripper.check_health()
        self.assertGreater(sum(len(bus.sent) for bus in self.buses.values()), before)
        self.gripper.close()
        self.sdk.connect.assert_not_called()
        self.sdk.close.assert_not_called()
        self.sdk.robot.unlink.assert_not_called()
        self.assertTrue(all(DISABLE not in [p for _, p in bus.sent] for bus in self.buses.values()))

    def hold_sample(self, left, stamp, right=.5):
        return {"targets_rad": {"left": .5, "right": .5}, "feedback": {
            "left": {"position_rad": left, "received_at": stamp},
            "right": {"position_rad": right, "received_at": stamp}}}

    def test_sustained_holding_angle_drift_is_a_failure(self):
        self.gripper.device.enable()
        self.gripper.settled = True
        with patch.object(self.gripper.device, "snapshot", side_effect=[
                self.hold_sample(.3, 10), self.hold_sample(.3, 10.6)]):
            self.gripper.check_health()
            with self.assertRaisesRegex(MotionError, "持续偏离保持角度"):
                self.gripper.check_health()

    def test_holding_allows_small_offsets_and_resets_after_transient_excursion(self):
        self.gripper.device.enable()
        self.gripper.settled = True
        with patch.object(self.gripper.device, "snapshot", side_effect=[
                self.hold_sample(.4, 10), self.hold_sample(.4, 11),  # Within hold tolerance.
                self.hold_sample(.2, 11.1), self.hold_sample(.2, 11.4),  # Brief large excursion.
                self.hold_sample(.4, 11.5),  # Recovery resets the timer.
                self.hold_sample(.2, 11.6), self.hold_sample(.2, 11.9)]):
            for _ in range(7):
                self.gripper.check_health()

    def test_rereading_same_bad_feedback_does_not_accumulate_hold_error_time(self):
        self.gripper.device.enable()
        self.gripper.settled = True
        with patch.object(self.gripper.device, "snapshot", return_value=self.hold_sample(.2, 10)):
            for _ in range(5):
                self.gripper.check_health()

    def test_holding_requires_each_sides_own_continuous_excursion(self):
        self.gripper.device.enable()
        self.gripper.settled = True
        with patch.object(self.gripper.device, "snapshot", side_effect=[
                self.hold_sample(.2, 10), self.hold_sample(.5, 10.4, right=.2),
                self.hold_sample(.5, 10.6, right=.2), self.hold_sample(.5, 11, right=.2)]):
            for _ in range(3):
                self.gripper.check_health()
            with self.assertRaisesRegex(MotionError, "right.*持续偏离"):
                self.gripper.check_health()

    def test_freeze_preserves_current_command_without_disable(self):
        self.gripper.device.enable()
        with self.gripper.device.lock:
            self.gripper.device.targets = {"left": .94, "right": .94}
            commands = dict(self.gripper.device.commands)
            self.gripper.freeze()
            self.assertEqual(self.gripper.device.targets, commands)
        self.assertTrue(all(DISABLE not in [p for _, p in bus.sent] for bus in self.buses.values()))

    def test_incomplete_endpoints_rejected_before_connect(self):
        with self.assertRaisesRegex(ValueError, "尚未设置"):
            TaskGripper(self.sdk, DMGripperConfig())


class TaskGripperDouble:
    """Immediate motor positions solely for testing arm/gripper sequencing."""
    def __init__(self, device):
        self.device = device
        self.config = load_gripper_config("config/gripper-can.json")
        self.actions = []
        self.error = None
        self.fail_action = None
        self.on_control = lambda: None
        self.checks = 0
        self.targets = {}
        self.frozen = False
        self.connected = False

    def reserve(self):
        self.device.calls.append(("gripper_reserve",))
        if self.device.connected:
            raise AssertionError("reserve must happen before SDK connect")

    def connect(self):
        self.connected = True

    def control(self, target, action, checkpoint):
        self.actions.append((target, action))
        self.device.calls.append(("gripper", action.value))
        self.on_control()
        checkpoint()
        if action == self.fail_action:
            raise MotionError("right gripper not at target")
        self.targets = self.config.action_positions(action.value, "both")
        return {"feedback": {side: {"position_rad": value} for side, value in self.targets.items()}}

    def check_health(self):
        self.checks += 1
        if self.error is not None:
            raise MotionError(self.error)

    def freeze(self):
        self.frozen = True

    def snapshot(self):
        return {"targets_rad": self.targets}


class IntegratedSequenceTests(unittest.TestCase):
    def setUp(self):
        self.config = replace(Config(), stable_seconds=.04, start_timeout_seconds=.15,
                              execution_timeout_seconds=.5, poll_seconds=.02)
        self.clock = Clock()
        self.device = Device(self.config)
        self.gripper = TaskGripperDouble(self.device)
        self.sequence = Sequence(self.device, self.config, gripper=self.gripper,
                                 clock=self.clock, wait=self.clock.wait)

    def test_open_receive_trigger_close_place_open_up_return_order_and_waits(self):
        self.sequence.prepare()
        self.assertEqual(self.gripper.actions, [(GripperTarget.BOTH, GripperAction.OPEN)])
        self.assertEqual(self.sequence.result.handover, "skipped")
        self.sequence.start()
        self.assertEqual([action for _, action in self.gripper.actions],
                         [GripperAction.OPEN, GripperAction.CLOSE, GripperAction.OPEN])
        completed = [step["name"] for step in self.sequence.result.steps if step["status"] == "completed"]
        expected = ["ready_lift", "open_before_receive", "open_before_receive_wait", "transition",
                    "receive", "receive_lift",
                    "handover_and_close", "handover_and_close_wait", "handover_wait",
                    "forward", "down", "release", "release_wait", "up",
                    "return_transition", "return_ready", "return_ready_lift"]
        self.assertEqual([step for step in completed if step in expected], expected)
        for name in ("open_before_receive_wait", "handover_and_close_wait", "handover_wait", "release_wait"):
            rows = [step for step in self.sequence.result.steps if step["name"] == name]
            self.assertGreaterEqual(rows[1]["elapsed_seconds"] - rows[0]["elapsed_seconds"], 1 - 1e-9)
        result = self.sequence.result
        self.assertEqual(result.mode, "fixed_base_with_grippers")
        self.assertEqual(result.gripper_actions, "completed")
        self.assertEqual(result.handover, "completed")
        self.assertIsNone(result.grasped)
        self.assertIsNone(result.placed)
        self.assertEqual(result.grasp_check, "skipped")
        self.assertFalse(self.gripper.frozen)

    def test_close_failure_stops_before_any_linear_movement(self):
        self.sequence.prepare()
        self.gripper.fail_action = GripperAction.CLOSE
        with self.assertRaisesRegex(MotionError, "right gripper"):
            self.sequence.start()
        self.assertEqual(self.device.linear_deltas, [])
        self.assertEqual(self.device.calls[-1], ("stop", 3))
        self.assertEqual(self.sequence.result.gripper_actions, "incomplete")
        self.assertTrue(self.gripper.frozen)
        self.assertEqual(len(self.gripper.actions), 2)  # No automatic reopening.

    def test_release_failure_prevents_up_and_return(self):
        self.sequence.prepare()
        self.gripper.fail_action = GripperAction.OPEN
        with self.assertRaisesRegex(MotionError, "right gripper"):
            self.sequence.start()
        self.assertEqual(len(self.device.linear_deltas), 2)
        self.assertNotIn("return_transition", [s["name"] for s in self.sequence.result.steps])
        self.assertNotIn("return_ready", [s["name"] for s in self.sequence.result.steps])

    def test_motor_fault_while_arms_move_stops_arms_without_release(self):
        self.sequence.prepare()
        def fault():
            if self.device.linear_deltas:
                self.gripper.error = "motor feedback timeout"
        self.device.on_read = fault
        with self.assertRaisesRegex(MotionError, "motor feedback timeout"):
            self.sequence.start()
        self.assertTrue(self.gripper.frozen)
        self.assertEqual(self.device.calls[-1], ("stop", 3))
        self.assertEqual(len(self.gripper.actions), 2)

    def test_cancel_during_close_aborts_without_release(self):
        self.sequence.prepare()
        self.gripper.on_control = self.sequence.request_stop
        with self.assertRaises(Stopped):
            self.sequence.start()
        self.assertEqual(self.sequence.result.status, "stopped")
        self.assertTrue(self.gripper.frozen)
        self.assertEqual(self.device.linear_deltas, [])

    def test_arm_drift_during_gripper_action_is_rejected(self):
        self.sequence.prepare()
        self.device.running = None
        self.gripper.on_control = lambda: self.device.rt["arms"][0]["fb"]["fb_pos"].__setitem__(0, 999)
        with self.assertRaisesRegex(MotionError, "未保持静止"):
            self.sequence.start()
        self.assertEqual(self.device.linear_deltas, [])

    def test_missing_enabled_driver_fails_before_connection_or_motion(self):
        sequence = Sequence(self.device, self.config)
        with self.assertRaisesRegex(MotionError, "未提供实际驱动"):
            sequence.prepare()
        self.assertFalse(self.device.connected)
        self.assertEqual(self.device.calls, [])

    def test_existing_gripper_owner_prevents_sdk_connection(self):
        self.gripper.reserve = MagicMock(side_effect=MotionError("SDK already owned"))
        with self.assertRaisesRegex(MotionError, "SDK already owned"):
            self.sequence.prepare()
        self.assertFalse(self.device.connected)
        self.assertEqual(self.device.calls, [])

    def test_real_driver_runs_three_actions_with_mock_motors_and_one_arm_link(self):
        config = replace(self.gripper.config, slew_rate_rad_s=20, stable_seconds=.02,
                         motion_timeout_seconds=2)
        self.device.robot = MagicMock()
        gripper = TaskGripper(self.device, config)
        buses = {}
        def transport(side):
            buses[side.interface] = MotorBus(side)
            return buses[side.interface]
        gripper.connection.transport = transport
        self.addCleanup(gripper.close)
        sequence = Sequence(self.device, self.config, gripper=gripper,
                            clock=self.clock, wait=self.clock.wait)
        sequence.prepare()
        sequence.start()
        self.assertEqual(sequence.result.status, "completed")
        rows = [step for step in sequence.result.steps
                if step["status"] == "completed" and "targets_rad" in step]
        self.assertEqual([step["targets_rad"] for step in rows],
                         [{"left": 0., "right": 0.}, {"left": .94, "right": .94},
                          {"left": 0., "right": 0.}])
        for bus in buses.values():
            self.assertEqual([p for _, p in bus.sent].count(ENABLE), 1)
            self.assertNotIn(DISABLE, [p for _, p in bus.sent])
            self.assertAlmostEqual(bus.position, 0., delta=.001)
        gripper.close()
        self.assertTrue(self.device.connected)  # SDK ownership remains with the arm task.


class IntegratedCLITests(unittest.TestCase):
    def test_cli_uses_one_sdk_and_closes_motor_worker_before_sdk(self):
        events = []
        with tempfile.TemporaryDirectory() as folder, \
             patch("skye_task.__main__.GentoSDK") as sdk_factory, \
             patch("skye_task.task_gripper.TaskGripper") as gripper_factory, \
             patch("skye_task.__main__.Sequence") as sequence_factory, \
             patch("skye_task.__main__.wait_for_start"), redirect_stdout(StringIO()):
            from skye_task.sequence import Result
            sdk, gripper = sdk_factory.return_value, gripper_factory.return_value
            sdk.close.side_effect = lambda: events.append("sdk_close")
            gripper.close.side_effect = lambda: events.append("gripper_close")
            gripper.snapshot.return_value = {}
            sequence_factory.return_value.result = Result(status="completed")
            self.assertEqual(main(["run", "--result", str(Path(folder) / "result.json")]), 0)
        sdk_factory.assert_called_once()
        self.assertIs(gripper_factory.call_args.args[0], sdk)
        self.assertIs(sequence_factory.call_args.kwargs["gripper"], gripper)
        self.assertEqual(events, ["gripper_close", "sdk_close"])

    def test_missing_endpoints_are_rejected_before_sdk_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / "gripper.json"
            config.write_text("{}")
            with patch("skye_task.__main__.GentoSDK") as sdk, patch("sys.stderr", new_callable=StringIO):
                with self.assertRaises(SystemExit):
                    main(["run", "--gripper-config", str(config)])
                sdk.assert_not_called()

    def test_gripper_cleanup_failure_does_not_unlink_a_running_worker(self):
        with tempfile.TemporaryDirectory() as folder, \
             patch("skye_task.__main__.GentoSDK") as sdk_factory, \
             patch("skye_task.task_gripper.TaskGripper") as gripper_factory, \
             patch("skye_task.__main__.Sequence") as sequence_factory, \
             patch("skye_task.__main__.wait_for_start"), redirect_stdout(StringIO()):
            from skye_task.sequence import Result
            gripper = gripper_factory.return_value
            gripper.snapshot.return_value = {}
            gripper.close.side_effect = MotionError("worker still running")
            gripper.worker_running = True
            sequence_factory.return_value.result = Result(status="completed")
            path = Path(folder) / "result.json"
            self.assertEqual(main(["run", "--result", str(path)]), 1)
            report = json.loads(path.read_text())
            self.assertEqual(report["status"], "failed")
            self.assertEqual(report["error"], "worker still running")
            sdk_factory.return_value.close.assert_not_called()

    def test_unloaded_config_does_not_construct_gripper(self):
        with tempfile.TemporaryDirectory() as folder, \
             patch("skye_task.__main__.GentoSDK"), \
             patch("skye_task.task_gripper.TaskGripper") as gripper_factory, \
             patch("skye_task.__main__.Sequence") as sequence_factory, \
             patch("skye_task.__main__.wait_for_start"), redirect_stdout(StringIO()):
            from skye_task.sequence import Result
            sequence_factory.return_value.result = Result(status="completed")
            path = Path(folder) / "config.json"
            path.write_text('{"gripper_enabled": false}')
            self.assertEqual(main(["run", "--config", str(path),
                                   "--result", str(Path(folder) / "result.json")]), 0)
            gripper_factory.assert_not_called()

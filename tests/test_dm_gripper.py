"""Protocol vectors and physical-feedback orchestration; no real CAN sockets are opened."""

from collections import deque
from contextlib import redirect_stdout
from dataclasses import replace
from io import StringIO
import importlib.util
import json
import math
from pathlib import Path
import socket
import struct
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

from skye_task.__main__ import main
from skye_task.dm_gripper import (CANSide, DMGripper, DMGripperConfig, SocketCAN,
                                  load_gripper_config)
from skye_task.dm_protocol import (CAN_FRAME, DISABLE, ENABLE, decode_feedback,
                                   encode_mit, pack_frame)
from skye_task.sdk import MotionError
from skye_task.sequence import Stopped


def feedback_frame(motor_id=1, can_id=0x11, status=1, position=0.5, velocity=0):
    q = int((position + 12.5) / 25 * 65535)
    dq = int((velocity + 30) / 60 * 4095)
    # Independently encode velocity and zero torque (0x7ff), temperatures 30/31.
    return CAN_FRAME.pack(can_id, 8, bytes((status << 4 | motor_id, q >> 8, q & 255,
                                          dq >> 4, (dq & 15) << 4 | 7, 0xFF, 30, 31)))


class ProtocolTests(unittest.TestCase):
    def test_documented_mit_vectors(self):
        vectors = [
            (0.0, 3, .12, "7FFF7FF0180627FF"),
            (.4, 3, .12, "84187FF0180627FF"),
            (.8, 3, .12, "88307FF0180627FF"),
            (1.2, 3, .12, "8C497FF0180627FF"),
            (1.44, 3, .12, "8EBE7FF0180627FF"),
            (1.6, 3, .12, "90617FF0180627FF"),
            (0.0, 1, .10, "7FFF7FF0080517FF"),
            (.5, 1, .10, "851E7FF0080517FF"),
            (1.0, 3, .12, "8A3C7FF0180627FF"),
            (1.5, 3, .12, "8F5B7FF0180627FF"),
        ]
        for q, kp, kd, expected in vectors:
            with self.subTest(q=q, kp=kp):
                self.assertEqual(encode_mit(q, kp=kp, kd=kd).hex().upper(), expected)

    def test_linux_frame_matches_documented_16_bytes(self):
        self.assertEqual(pack_frame(1, encode_mit(0)).hex(),
                         "01000000080000007fff7ff0180627ff")
        self.assertEqual(ENABLE.hex(), "fffffffffffffffc")
        self.assertEqual(DISABLE.hex(), "fffffffffffffffd")

    def test_feedback_slave_master_and_zero_ids_and_quantization(self):
        for can_id in (0, 1, 0x11):
            with self.subTest(can_id=can_id):
                feedback = decode_feedback(feedback_frame(can_id=can_id), slave_id=1,
                                           master_id=17, received_at=123)
                self.assertEqual((feedback.status, feedback.motor_id), (1, 1))
                self.assertAlmostEqual(feedback.position_rad, .5, delta=25/65535)
                self.assertAlmostEqual(feedback.velocity_rad_s, 0, delta=30/4095)
                self.assertAlmostEqual(feedback.torque_nm, 0, delta=10/4095)
                self.assertEqual((feedback.mos_temperature_c, feedback.motor_temperature_c), (30, 31))
                self.assertEqual(feedback.received_at, 123)

    def test_wrong_motor_ids_commands_and_flagged_frames_are_ignored(self):
        frames = [feedback_frame(motor_id=2), feedback_frame(can_id=0x12),
                  feedback_frame(can_id=0x80000011), feedback_frame(can_id=0x40000011),
                  feedback_frame(can_id=0x20000011), pack_frame(1, ENABLE),
                  pack_frame(1, DISABLE), pack_frame(1, encode_mit(0))]
        for frame in frames:
            with self.subTest(frame=frame.hex()):
                self.assertIsNone(decode_feedback(frame, slave_id=1, master_id=17, received_at=0))

    def test_malformed_feedback_is_rejected(self):
        for frame in (b"short", CAN_FRAME.pack(17, 7, bytes(8))):
            with self.assertRaises(MotionError):
                decode_feedback(frame, slave_id=1, master_id=17, received_at=0)

    def test_fault_codes_preserve_raw_bytes_instead_of_becoming_success(self):
        for status in (0, 8, 9, 10, 11, 12, 13, 14, 15):
            frame = feedback_frame(status=status)
            feedback = decode_feedback(frame, slave_id=1, master_id=17, received_at=0)
            self.assertEqual(feedback.status, status)
            self.assertEqual(feedback.raw_hex, frame.hex())

    def test_out_of_range_and_nonfinite_values_are_rejected_without_clamping(self):
        for kwargs in ({"position_rad": 12.6}, {"position_rad": math.nan},
                       {"position_rad": 0, "kp": 501}, {"position_rad": 0, "kd": -1},
                       {"position_rad": 0, "torque_nm": 11}):
            with self.assertRaises(ValueError):
                encode_mit(**kwargs)


class MotorBus:
    """A feedback-generating transport double, solely for offline tests."""
    def __init__(self, config):
        self.config = config
        self.queue = deque()
        self.sent = []
        self.position = .5
        self.position_offset = 0
        self.velocity = 0
        self.claimed = False
        self.closed = False
        self.feedback_enabled = True
        self.track_target = True
        self.fail_disable = False
        self.status = 0

    def claim(self):
        self.claimed = True

    def emit(self, status=None):
        self.queue.append(feedback_frame(self.config.slave_id, self.config.master_id,
                                         self.status if status is None else status, self.position, self.velocity))

    def send(self, payload):
        self.sent.append((time.monotonic(), payload))
        if payload == ENABLE:
            self.status = 1
        elif payload == DISABLE:
            if self.fail_disable:
                raise OSError("disable failed")
            self.status = 0
        elif self.track_target:
            self.position = (payload[0] << 8 | payload[1]) / 65535 * 25 - 12.5 + self.position_offset
        if self.feedback_enabled:
            self.emit()

    def receive(self):
        return self.queue.popleft() if self.queue else None

    def close(self):
        self.closed = True


class DriverTests(unittest.TestCase):
    def setUp(self):
        self.config = replace(DMGripperConfig(), stable_seconds=.03,
                              position_tolerance_rad=.02, velocity_tolerance_rad_s=.1,
                              motion_timeout_seconds=.6, feedback_timeout_seconds=.3,
                              startup_timeout_seconds=.15)
        self.buses = {}
        def factory(config):
            bus = MotorBus(config)
            self.buses[config.interface] = bus
            return bus
        self.driver = DMGripper(self.config, transport_factory=factory)
        self.driver.connect()
        self.addCleanup(self.driver.close)

    def wait_for_error(self):
        deadline = time.monotonic() + 1
        while self.driver.error is None and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertIsNotNone(self.driver.error)

    def test_status_is_passive_and_reports_off_and_faults(self):
        self.buses["vcan0"].emit(0)
        self.buses["vcan1"].emit(10)
        report = self.driver.read_status(.1)
        self.assertEqual(report["feedback"]["left"]["status"], 0)
        self.assertEqual(report["feedback"]["right"]["status"], 10)
        self.assertTrue(all(not bus.sent and not bus.claimed for bus in self.buses.values()))

    def test_no_feedback_is_failure_and_sends_nothing(self):
        with self.assertRaisesRegex(MotionError, "未收到新鲜"):
            self.driver.read_status(.02)
        self.assertTrue(all(not bus.sent for bus in self.buses.values()))

    def test_both_enable_order_slew_and_actual_arrival(self):
        self.driver.enable()
        before = self.driver.snapshot()
        self.assertAlmostEqual(before["targets_rad"]["left"], .5, delta=.001)
        report = self.driver.step(.05)
        self.assertEqual(set(report["feedback"]), {"left", "right"})
        self.assertGreater(report["mit_frames_sent"], 10)
        left_fc = self.buses["vcan0"].sent[0]
        right_fc = self.buses["vcan1"].sent[0]
        self.assertEqual((left_fc[1], right_fc[1]), (ENABLE, ENABLE))
        self.assertGreaterEqual(right_fc[0] - left_fc[0], .095)
        for bus in self.buses.values():
            positions = [(p[0] << 8 | p[1]) / 65535 * 25 - 12.5
                         for _, p in bus.sent if p not in (ENABLE, DISABLE)]
            self.assertGreater(positions[-1], .53)
            self.assertTrue(all(abs(b-a) <= .002 + 25/65535
                                for a, b in zip(positions, positions[1:])))
        self.driver.disable()
        self.assertTrue(all(bus.sent[-1][1] == DISABLE for bus in self.buses.values()))

    def test_open_decreases_and_close_increases_both_installed_grippers(self):
        self.driver.enable()
        before = self.driver.snapshot()["feedback"]
        opened = self.driver.move_to({"left": .45, "right": .46}, action="open")
        for name in ("left", "right"):
            self.assertLess(opened["feedback"][name]["position_rad"],
                            before[name]["position_rad"] - .03)
        closed = self.driver.move_to({"left": .5, "right": .51}, action="close")
        for name in ("left", "right"):
            self.assertGreater(closed["feedback"][name]["position_rad"],
                               opened["feedback"][name]["position_rad"] + .04)
        self.assertEqual(closed["targets_rad"], {"left": .5, "right": .51})

    def test_absolute_target_does_not_accumulate_on_repeated_commands(self):
        self.driver.enable()
        targets = {"left": .55, "right": .55}
        self.driver.move_to(targets, action="close")
        repeated = self.driver.move_to(targets, action="close")
        self.assertEqual(repeated["targets_rad"], targets)
        for feedback in repeated["feedback"].values():
            self.assertAlmostEqual(feedback["position_rad"], .55, delta=.001)

    def test_absolute_target_can_exceed_relative_step_limit(self):
        self.driver.enable()
        self.driver.config = replace(self.config, motion_timeout_seconds=1.5)
        result = self.driver.move_to({"left": .65, "right": .65}, action="close")
        self.assertEqual(result["targets_rad"], {"left": .65, "right": .65})
        self.assertGreater(result["feedback"]["left"]["position_rad"], .64)

    def test_open_close_respects_each_sides_configured_direction(self):
        self.driver.config = replace(self.config, right=replace(self.config.right, close_direction=-1))
        self.driver.enable()
        closed = self.driver.move_to({"left": .55, "right": .45}, action="close")
        self.assertGreater(closed["feedback"]["left"]["position_rad"], .54)
        self.assertLess(closed["feedback"]["right"]["position_rad"], .46)

    def test_invalid_open_close_request_does_not_change_targets(self):
        self.driver.enable()
        targets = self.driver.snapshot()["targets_rad"]
        for action, request in (("toggle", {"left": .5, "right": .5}),
                                ("open", {"left": .4}),
                                ("close", {"left": .55, "right": 1.7}),
                                ("close", {"left": .55, "right": math.nan}),
                                ("open", {"left": True, "right": .45})):
            with self.subTest(action=action, request=request), self.assertRaises(ValueError):
                self.driver.move_to(request, action=action)
        for action, request in (("close", {"left": .55, "right": .4}),
                                ("open", {"left": .45, "right": .6})):
            with self.subTest(action=action), self.assertRaisesRegex(MotionError, "方向相反"):
                self.driver.move_to(request, action=action)
        self.assertEqual(self.driver.snapshot()["targets_rad"], targets)

    def test_feedback_loss_stops_target_updates_without_reset_or_disable(self):
        self.driver.enable()
        with self.driver.lock:
            self.buses["vcan0"].feedback_enabled = False
        self.wait_for_error()
        with self.assertRaisesRegex(MotionError, "反馈超时"):
            self.driver.check_health()
        sent = [len(bus.sent) for bus in self.buses.values()]
        time.sleep(.03)
        self.assertEqual(sent, [len(bus.sent) for bus in self.buses.values()])
        self.assertTrue(all(DISABLE not in [p for _, p in bus.sent] for bus in self.buses.values()))

    def test_motor_fault_aborts_and_preserves_raw_frame(self):
        self.driver.enable()
        with self.driver.lock:
            self.buses["vcan1"].status = 10
            self.buses["vcan1"].emit()
        self.wait_for_error()
        with self.assertRaisesRegex(MotionError, "过电流.*原始帧"):
            self.driver.check_health()

    def test_a_fault_followed_by_on_in_the_same_batch_is_not_hidden(self):
        self.driver.enable()
        with self.driver.lock:
            self.buses["vcan0"].emit(10)
            self.buses["vcan0"].emit(1)
        self.wait_for_error()
        with self.assertRaisesRegex(MotionError, "过电流"):
            self.driver.check_health()

    def test_known_motor_fault_is_rejected_before_enabling(self):
        self.buses["vcan0"].emit(11)
        with self.assertRaisesRegex(MotionError, "MOS 过温"):
            self.driver.enable()
        self.assertTrue(all(not bus.sent for bus in self.buses.values()))

    def test_single_side_never_opens_or_commands_the_other_motor(self):
        buses = []
        def factory(config):
            bus = MotorBus(config)
            buses.append(bus)
            return bus
        driver = DMGripper(self.config, "right", transport_factory=factory)
        self.addCleanup(driver.close)
        driver.connect()
        driver.enable()
        driver.step(-.05)
        self.assertEqual(len(buses), 1)
        self.assertEqual(buses[0].config.interface, "vcan1")
        self.assertEqual(set(driver.snapshot()["feedback"]), {"right"})

    def test_one_side_not_at_target_cannot_complete_both(self):
        self.driver.enable()
        self.buses["vcan1"].track_target = False
        with self.assertRaisesRegex(MotionError, "执行超时"):
            self.driver.step(.05)

    def test_relaxed_arrival_accepts_measured_offset_and_velocity_noise(self):
        self.driver.config = replace(self.config, position_tolerance_rad=.08,
                                     velocity_tolerance_rad_s=.3, stable_seconds=.1)
        self.driver.enable()
        for bus in self.buses.values():
            bus.position_offset = .047
            bus.velocity = .2
        # Matches the measured opening offset (~0.041..0.047 rad).
        self.driver.config = replace(self.driver.config, slew_rate_rad_s=20)
        result = self.driver.move_to({"left": 0., "right": 0.}, action="open")
        for value in result["feedback"].values():
            self.assertAlmostEqual(value["position_rad"], .047, delta=.001)
        self.assertTrue(all(abs(command) < 1e-9 for command in result["commands_rad"].values()))

    def test_relaxed_arrival_still_rejects_a_stuck_motor_far_from_target(self):
        self.driver.config = replace(self.config, position_tolerance_rad=.08,
                                     velocity_tolerance_rad_s=.3, stable_seconds=.1,
                                     slew_rate_rad_s=20)
        self.driver.enable()
        self.buses["vcan1"].track_target = False
        with self.assertRaisesRegex(MotionError, "执行超时.*right 目标.*允许误差 0.080"):
            self.driver.move_to({"left": .94, "right": .94}, action="close")

    def test_step_outside_limits_does_not_change_either_target(self):
        self.driver.enable()
        targets = self.driver.snapshot()["targets_rad"]
        for delta in (0, .11, math.nan):
            with self.assertRaises(ValueError):
                self.driver.step(delta)
        self.assertEqual(self.driver.snapshot()["targets_rad"], targets)

    def test_startup_without_on_feedback_never_sends_mit(self):
        for bus in self.buses.values():
            bus.feedback_enabled = False
        with self.assertRaisesRegex(MotionError, "使能后未收到"):
            self.driver.enable()
        self.assertTrue(all([p for _, p in bus.sent] == [ENABLE] for bus in self.buses.values()))

    def test_disable_attempts_both_sides_after_one_send_fails(self):
        self.buses["vcan0"].fail_disable = True
        with self.assertRaisesRegex(MotionError, "失能发送失败"):
            self.driver.disable()
        self.assertEqual(self.buses["vcan1"].sent[-1][1], DISABLE)

    def test_disable_is_not_confirmed_from_send_success_alone(self):
        self.buses["vcan1"].feedback_enabled = False
        with self.assertRaisesRegex(MotionError, "无法确认物理失能"):
            self.driver.disable()

    def test_repeated_disable_polls_follow_manual_positions_without_enable_or_mit(self):
        self.driver.disable()
        self.buses["vcan0"].position = .8
        self.buses["vcan1"].position = .3
        self.driver.disable()
        feedback = self.driver.snapshot()["feedback"]
        self.assertAlmostEqual(feedback["left"]["position_rad"], .8, delta=.001)
        self.assertAlmostEqual(feedback["right"]["position_rad"], .3, delta=.001)
        self.assertTrue(all(value["status"] == 0 for value in feedback.values()))
        self.assertTrue(all([payload for _, payload in bus.sent] == [DISABLE, DISABLE]
                            for bus in self.buses.values()))

    def test_monitor_poll_cannot_reuse_previous_off_as_fresh_confirmation(self):
        self.driver.disable()
        self.buses["vcan1"].feedback_enabled = False
        with self.assertRaisesRegex(MotionError, "无法确认物理失能"):
            self.driver.disable()

    def test_close_never_sends_automatic_disable(self):
        self.driver.enable()
        self.driver.close()
        self.assertTrue(all(bus.closed for bus in self.buses.values()))
        self.assertTrue(all(DISABLE not in [p for _, p in bus.sent] for bus in self.buses.values()))

    def test_stuck_worker_does_not_have_its_transports_closed(self):
        with patch.object(self.driver, "_stop_worker", side_effect=MotionError("worker stuck")):
            with self.assertRaisesRegex(MotionError, "worker stuck"):
                self.driver.close()
        self.assertTrue(all(not bus.closed for bus in self.buses.values()))

    def test_cancelled_step_does_not_report_success(self):
        self.driver.enable()
        def cancel():
            raise Stopped("cancel")
        with self.assertRaises(Stopped):
            self.driver.step(.05, cancel)


class TransportAndCLITests(unittest.TestCase):
    def test_socket_setup_standard_filters_no_own_echo_and_nonblocking(self):
        sock = MagicMock()
        sock.send.return_value = 16
        sock.recv.side_effect = BlockingIOError
        with patch("skye_task.dm_gripper.socket.socket", return_value=sock):
            bus = SocketCAN(CANSide())
            sock.bind.assert_called_once_with(("vcan0",))
            sock.setblocking.assert_called_once_with(False)
            sock.setsockopt.assert_any_call(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            self.assertIsNone(bus.receive())
            bus.send(ENABLE)
            sock.send.assert_called_once_with(pack_frame(1, ENABLE))
            bus.close()
            sock.close.assert_called_once()

    def test_failed_bind_closes_socket(self):
        sock = MagicMock()
        sock.bind.side_effect = OSError("no such device")
        with patch("skye_task.dm_gripper.socket.socket", return_value=sock):
            with self.assertRaises(OSError):
                SocketCAN(CANSide())
        sock.close.assert_called_once()

    def test_default_config_and_invalid_overrides(self):
        loaded = load_gripper_config("config/gripper-can.json")
        for side in (loaded.left, loaded.right):
            self.assertEqual(side.open_position_rad, 0.0)
            self.assertEqual(side.close_position_rad, .94)
        invalid = [{"left": {"slave_id": 16}}, {"left": {"master_id": 2048}},
                   {"left": {"interface": "vcan1"}}, {"kp": 501}, {"kd": math.nan},
                   {"max_step_rad": -1}, {"control_rate_hz": 0}, {"unknown": 1},
                   {"min_position_rad": 2}, {"control_rate_hz": True},
                   {"left": {"close_direction": 0}}, {"right": {"close_direction": True}},
                   {"left": {"open_position_rad": math.nan}},
                   {"left": {"close_position_rad": True}},
                   {"left": {"open_position_rad": -.1}},
                   {"right": {"close_position_rad": 1.7}},
                   {"left": {"open_position_rad": .7, "close_position_rad": .5}},
                   {"hold_position_tolerance_rad": .01}, {"hold_error_seconds": 0},
                   {"hold_error_seconds": math.nan}]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            for values in invalid:
                with self.subTest(values=values):
                    path.write_text(json.dumps(values))
                    with self.assertRaises(ValueError):
                        load_gripper_config(path)

    def test_cli_step_and_requested_disable_cleanup_never_connect_arm_sdk(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            with patch("skye_task.gripper_cli.DMGripper") as factory, \
                 patch("skye_task.__main__.GentoSDK") as arm_sdk, redirect_stdout(StringIO()):
                driver = factory.return_value
                driver.enabled_sides = ["left"]
                driver.snapshot.return_value = {"feedback": {}}
                driver.step.return_value = {"feedback": {}}
                result = main(["gripper", "step", "--side", "left", "--delta-rad", ".05",
                               "--hold-seconds", "0", "--disable-on-exit", "--result", str(path)])
            self.assertEqual(result, 0)
            driver.enable.assert_called_once()
            driver.step.assert_called_once()
            self.assertEqual(driver.step.call_args.args[0], .05)
            driver.disable.assert_called_once()
            driver.close.assert_called_once()
            arm_sdk.assert_not_called()
            report = json.loads(path.read_text())
            self.assertEqual(report["disable_on_exit"], "confirmed_off")
            self.assertEqual(report["status"], "completed")

    def test_cli_control_failure_preserves_error_and_runs_requested_cleanup(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            with patch("skye_task.gripper_cli.DMGripper") as factory, redirect_stdout(StringIO()):
                driver = factory.return_value
                driver.enabled_sides = ["left"]
                driver.snapshot.return_value = {"feedback": {}}
                driver.step.side_effect = MotionError("feedback stale")
                driver.disable.side_effect = MotionError("Off not confirmed")
                result = main(["gripper", "step", "--side", "left", "--delta-rad", ".05",
                               "--disable-on-exit", "--result", str(path)])
            self.assertEqual(result, 1)
            driver.close.assert_called_once()
            report = json.loads(path.read_text())
            self.assertEqual(report["error"], "feedback stale")
            self.assertEqual(report["cleanup_error"], "Off not confirmed")
            self.assertEqual(report["disable_on_exit"], "not_confirmed")

    def test_cli_inspect_never_opens_can_or_connects_arm_sdk(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            with patch("skye_task.gripper_cli.socket.if_nameindex", return_value=[(1, "lo")]), \
                 patch("skye_task.gripper_cli.DMGripper") as driver, \
                 patch("skye_task.__main__.GentoSDK") as arm_sdk, redirect_stdout(StringIO()):
                self.assertEqual(main(["gripper", "inspect", "--result", str(path)]), 1)
            driver.assert_not_called()
            arm_sdk.assert_not_called()
            report = json.loads(path.read_text())
            self.assertFalse(report["interfaces"]["left"]["exists"])
            self.assertEqual(report["arm_motion"], "not_called")

    def test_cli_invalid_step_is_rejected_before_constructing_driver(self):
        with patch("skye_task.gripper_cli.DMGripper") as driver, \
             patch("sys.stderr", new_callable=StringIO):
            with self.assertRaises(SystemExit):
                main(["gripper", "step", "--delta-rad", "1.0"])
        driver.assert_not_called()

    def test_cli_open_close_mapping_and_requested_disable(self):
        with tempfile.TemporaryDirectory() as folder:
            for action, target in (("open", .4), ("close", .6)):
                with self.subTest(action=action):
                    path = Path(folder) / f"{action}.json"
                    with patch("skye_task.gripper_cli.DMGripper") as factory, redirect_stdout(StringIO()):
                        driver = factory.return_value
                        driver.enabled_sides = ["left", "right"]
                        driver.snapshot.return_value = {"feedback": {}}
                        driver.move_to.return_value = {"feedback": {}}
                        result = main(["gripper", action, "--side", "both", "--position-rad", str(target),
                                       "--hold-seconds", "0", "--disable-on-exit", "--result", str(path)])
                    self.assertEqual(result, 0)
                    driver.step.assert_not_called()
                    self.assertEqual(driver.move_to.call_args.args[0], {"left": target, "right": target})
                    self.assertEqual(driver.move_to.call_args.kwargs, {"action": action})
                    driver.disable.assert_called_once()
                    driver.close.assert_called_once()
                    report = json.loads(path.read_text())
                    self.assertEqual(report["motion_scope"], "absolute_position")
                    self.assertEqual(report["requested_targets_rad"], {"left": target, "right": target})
                    self.assertEqual(report["gripper_actions"], {"left": action, "right": action})
                    self.assertEqual(report["disable_on_exit"], "confirmed_off")

    def test_cli_open_close_failure_runs_disable_and_preserves_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            with patch("skye_task.gripper_cli.DMGripper") as factory, redirect_stdout(StringIO()):
                driver = factory.return_value
                driver.enabled_sides = ["left"]
                driver.snapshot.return_value = {"feedback": {}}
                driver.move_to.side_effect = MotionError("not at target")
                result = main(["gripper", "close", "--side", "left", "--position-rad", ".6",
                               "--disable-on-exit", "--result", str(path)])
            self.assertEqual(result, 1)
            driver.disable.assert_called_once()
            self.assertEqual(json.loads(path.read_text())["error"], "not at target")

    def test_snapshot_failure_does_not_prevent_disable_and_connection_close(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            with patch("skye_task.gripper_cli.DMGripper") as factory, redirect_stdout(StringIO()):
                driver = factory.return_value
                driver.enabled_sides = ["left"]
                driver.snapshot.side_effect = RuntimeError("snapshot failed")
                result = main(["gripper", "close", "--side", "left", "--position-rad", ".6",
                               "--disable-on-exit", "--result", str(path)])
            self.assertEqual(result, 1)
            driver.disable.assert_called_once()
            driver.close.assert_called_once()
            report = json.loads(path.read_text())
            self.assertEqual(report["error"], "snapshot failed")
            self.assertEqual(report["disable_on_exit"], "confirmed_off")

    def test_brief_reports_active_angle_before_disable_relaxes_motor(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            output = StringIO()
            with patch("skye_task.gripper_cli.DMGripper") as factory, redirect_stdout(output):
                driver = factory.return_value
                driver.enabled_sides = ["left"]
                driver.snapshot.side_effect = [
                    {"feedback": {"left": {"position_rad": .5}}},
                    {"feedback": {"left": {"position_rad": .6}}},
                    {"feedback": {"left": {"position_rad": .6, "status_name": "On"}}},
                    {"feedback": {"left": {"position_rad": .55, "status_name": "Off"}}},
                ]
                driver.move_to.return_value = {"feedback": {}}
                result = main(["gripper", "close", "--side", "left", "--position-rad", ".6",
                               "--hold-seconds", "0", "--disable-on-exit", "--brief", "--result", str(path)])
            self.assertEqual(result, 0)
            self.assertIn("起始 0.500000 rad，目标 0.600000 rad，实际 0.600000 rad", output.getvalue())
            self.assertIn("退出状态 Off", output.getvalue())

    def test_cli_invalid_open_close_arguments_never_construct_driver(self):
        invalid = [["open"], ["close"], ["move"], ["close", "--position-rad", "-.1"],
                   ["open", "--position-rad", "1.7"], ["close", "--delta-rad", ".1"],
                   ["open", "--position-rad", "nan"],
                   ["open", "--position-rad", ".4", "--delta-rad", "-.1"],
                   ["step", "--position-rad", ".6", "--delta-rad", ".1"],
                   ["status", "--position-rad", ".6"]]
        for arguments in invalid:
            with self.subTest(arguments=arguments), \
                 patch("skye_task.gripper_cli.DMGripper") as driver, \
                 patch("sys.stderr", new_callable=StringIO):
                with self.assertRaises(SystemExit):
                    main(["gripper", *arguments])
                driver.assert_not_called()

    def test_cli_uses_individually_configured_absolute_positions(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / "config.json"
            config.write_text(json.dumps({"left": {"close_position_rad": .6},
                                          "right": {"close_position_rad": .7}}))
            with patch("skye_task.gripper_cli.DMGripper") as factory, redirect_stdout(StringIO()):
                driver = factory.return_value
                driver.snapshot.return_value = {"feedback": {}}
                driver.move_to.return_value = {"feedback": {}}
                self.assertEqual(main(["gripper", "close", "--config", str(config),
                                       "--hold-seconds", "0", "--result", str(Path(folder) / "out.json")]), 0)
            self.assertEqual(driver.move_to.call_args.args[0], {"left": .6, "right": .7})

    def test_standalone_script_forwards_absolute_target_to_sdk_and_disables(self):
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location("gripper_test_script", root / "scripts/gripper_test.py")
        script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(script)
        with patch.object(script, "gripper_main", return_value=0) as run:
            self.assertEqual(script.main(["close", "--side", "right", "--position-rad", ".8"]), 0)
        args = run.call_args.args[0]
        self.assertEqual(args[0], "close")
        for option, value in (("--transport", "sdk"), ("--side", "right"), ("--position-rad", "0.8"),
                              ("--config", str(root / "config/gripper-can.json")),
                              ("--robot-config", str(root / "config/default.json")),
                              ("--result", str(root / "runs/gripper-test-right-close.json"))):
            self.assertEqual(args[args.index(option) + 1], value)
        self.assertIn("--disable-on-exit", args)
        self.assertIn("--brief", args)


if __name__ == "__main__":
    unittest.main()

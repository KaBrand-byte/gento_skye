"""Verify the native SDK envelope and connection ownership without hardware."""

from collections import deque
from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from skye_task.__main__ import main
from skye_task.config import Config
from skye_task.dm_gripper import DMGripperConfig
from skye_task.dm_protocol import CAN_FRAME, ENABLE
from skye_task.dm_terminal import (GentoTerminalConnection, pack_terminal,
                                   unpack_terminal)
from skye_task.sdk import MotionError


class TerminalTests(unittest.TestCase):
    def setUp(self):
        self.sdk = MagicMock()
        self.sdk.connect.return_value = {"sdk_version": "00040701"}
        self.sdk.robot.terminal_set.return_value = (0, 0)
        self.connection = GentoTerminalConnection(Config(), DMGripperConfig(),
                                                 sdk_factory=lambda _: self.sdk)
        self.connection.connect()
        self.addCleanup(self.connection.close)
        self.left = self.connection.transport(DMGripperConfig().left)
        self.right = self.connection.transport(DMGripperConfig().right)

    def mock_receive(self, packets):
        packets = deque(packets)
        def receive(obj, timeout, channel, buffer, elapsed):
            expected_obj, count, channel_value, raw = packets.popleft()
            self.assertEqual((obj, timeout), (expected_obj, 0))
            channel._obj.value = channel_value
            for i, value in enumerate(raw):
                buffer[i] = value
            return count
        self.sdk.robot.dll.FX_L1_Terminal_GetData.side_effect = receive

    def test_vendor_envelope_is_id_then_data_without_socketcan_header(self):
        raw = pack_terminal(1, ENABLE)
        self.assertEqual(raw.hex(), "01000000fffffffffffffffc")
        self.assertEqual(unpack_terminal(raw), CAN_FRAME.pack(1, 8, ENABLE))
        for raw in (b"", b"\x01" * 8, CAN_FRAME.pack(1, 8, ENABLE), b"\x00" * 13):
            with self.assertRaises(MotionError):
                unpack_terminal(raw)

    def test_left_and_right_send_use_canfd_and_correct_arm(self):
        self.left.send(ENABLE)
        self.right.send(ENABLE)
        calls = self.sdk.robot.terminal_set.call_args_list
        self.assertEqual(calls[0].args, (0, 1, bytes.fromhex("01000000fffffffffffffffc")))
        self.assertEqual(calls[1].args, (1, 1, bytes.fromhex("02000000fffffffffffffffc")))
        self.assertEqual(calls[0].kwargs, {"timeout": 0})
        self.assertEqual(self.connection.frames_sent, 2)

    def test_send_error_does_not_report_success(self):
        self.sdk.robot.terminal_set.return_value = (-27, 0)
        with self.assertRaisesRegex(MotionError, "-27"):
            self.left.send(ENABLE)
        self.assertEqual(self.connection.frames_sent, 0)

    def test_receive_preserves_can_id_and_raw_terminal_telemetry(self):
        raw = bytes.fromhex("120000001288307ff7ff1e1f")
        self.mock_receive([(1, 12, 1, raw)])
        self.assertEqual(self.right.receive(), CAN_FRAME.pack(18, 8, raw[4:]))
        snapshot = self.connection.snapshot()
        self.assertEqual(snapshot["recent_terminal_frames"][0]["raw_hex"], raw.hex())
        self.assertEqual(snapshot["recent_terminal_frames"][0]["arm"], 1)

    def test_receive_only_timeout_and_empty_are_no_data(self):
        self.mock_receive([(0, -10, 0, b""), (0, 0, 0, b""), (0, -27, 0, b"")])
        self.assertIsNone(self.left.receive())
        self.assertIsNone(self.left.receive())
        with self.assertRaisesRegex(MotionError, "-27"):
            self.left.receive()

    def test_wrong_channel_and_invalid_length_fail_with_raw_data(self):
        raw = bytes.fromhex("110000001188307ff7ff1e1f")
        self.mock_receive([(0, 12, 2, raw), (0, 8, 1, raw[:8])])
        with self.assertRaisesRegex(MotionError, "其他末端通道"):
            self.left.receive()
        with self.assertRaisesRegex(MotionError, "长度异常"):
            self.left.receive()

    def test_single_sdk_connection_and_close_never_move_or_disable_arms(self):
        self.connection.close()
        self.connection.close()
        self.sdk.connect.assert_called_once()
        self.sdk.close.assert_called_once()
        self.sdk.initialize.assert_not_called()
        self.sdk.stop.assert_not_called()
        self.sdk.robot.terminal_set.assert_not_called()
        with self.assertRaises(MotionError):
            self.left.send(ENABLE)

    def test_failed_connection_closes_sdk_and_releases_lock(self):
        self.connection.close()
        self.sdk.connect.side_effect = MotionError("no link")
        connection = GentoTerminalConnection(Config(), DMGripperConfig(),
                                              sdk_factory=lambda _: self.sdk)
        with self.assertRaisesRegex(MotionError, "no link"):
            connection.connect()
        self.assertIsNone(connection.sdk)
        self.assertIsNone(connection.ownership)
        self.assertEqual(self.sdk.close.call_count, 2)

    def test_borrowed_link_never_connects_or_closes_owner_sdk(self):
        self.connection.close()
        self.sdk.reset_mock()
        self.sdk.connected = True
        connection = GentoTerminalConnection(Config(), DMGripperConfig(), sdk=self.sdk)
        self.addCleanup(connection.close)
        connection.connect()
        self.assertTrue(connection.snapshot()["connection"]["shared_with_arm_sequence"])
        connection.close()
        self.sdk.connect.assert_not_called()
        self.sdk.close.assert_not_called()

    def test_borrowing_disconnected_sdk_fails_and_releases_lock(self):
        self.connection.close()
        self.sdk.connected = False
        connection = GentoTerminalConnection(Config(), DMGripperConfig(), sdk=self.sdk)
        with self.assertRaisesRegex(MotionError, "尚未连接"):
            connection.connect()
        self.assertIsNone(connection.ownership)


class TerminalCLITests(unittest.TestCase):
    def test_sdk_inspect_is_local_only(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            with patch("skye_task.gripper_cli.GentoTerminalConnection") as connection, \
                 redirect_stdout(StringIO()):
                result = main(["gripper", "inspect", "--transport", "sdk", "--result", str(path)])
            connection.assert_not_called()
            self.assertEqual(result, 0)
            self.assertFalse(json.loads(path.read_text())["requires_vcan"])

    def test_sdk_status_closes_connection_even_on_feedback_timeout(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            with patch("skye_task.gripper_cli.GentoTerminalConnection") as factory, \
                 patch("skye_task.gripper_cli.DMGripper") as device_factory, \
                 redirect_stdout(StringIO()):
                connection, device = factory.return_value, device_factory.return_value
                connection.snapshot.return_value = {"terminal_frames_sent": 0}
                device.snapshot.return_value = {"feedback": {}}
                device.read_status.side_effect = MotionError("no feedback")
                result = main(["gripper", "status", "--transport", "sdk", "--result", str(path)])
            self.assertEqual(result, 1)
            connection.connect.assert_called_once()
            connection.close.assert_called_once()
            device.enable.assert_not_called()
            device.disable.assert_not_called()
            device.close.assert_called_once()
            self.assertEqual(json.loads(path.read_text())["error"], "no feedback")


if __name__ == "__main__":
    unittest.main()

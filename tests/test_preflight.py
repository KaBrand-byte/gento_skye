"""A failed read-only plan check must never change robot control state."""

from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from skye_task.__main__ import main
from skye_task.config import Config
from skye_task.preflight import check_current_plan
from skye_task.sdk import MotionError
from test_sequence import Device


class PreflightTests(unittest.TestCase):
    def device_at_receive(self):
        device = Device(Config())
        device.rt["lift"]["fb_pos"][0] = Config().receive_lift[0]
        return device

    def test_full_plan_from_feedback_has_no_enable_upload_run_or_stop(self):
        device = self.device_at_receive()
        for arm in range(2):
            device.rt["arms"][arm]["fb"]["fb_pos"] = list(Config().receive[arm])
        device.enter_position = MagicMock(side_effect=AssertionError("must not enable"))
        report = {}
        self.assertTrue(check_current_plan(device, Config(), report))
        self.assertEqual(report["status"], "planned")
        self.assertEqual([r["name"] for r in report["segments"]],
                         ["forward", "down", "up", "return_transition", "return_ready"])
        self.assertEqual(device.calls, [("plan_joint", 0), ("plan_joint", 1)] * 2)
        forward, down, up, transition, ready = report["segments"]
        self.assertEqual(up["robot_displacements_mm"], [[0, 0, 100], [0, 0, 100]])
        self.assertEqual(up["arm_base_target_matrices"], down["arm_base_start_matrices"])
        self.assertEqual(transition["reference_joints_deg"], forward["endpoint_joints_deg"])
        self.assertEqual(ready["reference_joints_deg"], Config().transition)
        self.assertEqual(ready["endpoint_joints_deg"], Config().ready)
        device.enter_position.assert_not_called()

    def test_failed_return_transition_does_not_plan_return_ready_or_send_control(self):
        device = self.device_at_receive()
        device.plan_joint = MagicMock(side_effect=MotionError("return transition unreachable"))
        report = {}
        self.assertFalse(check_current_plan(device, Config(), report))
        self.assertEqual([r["name"] for r in report["segments"]],
                         ["forward", "down", "up", "return_transition"])
        self.assertEqual(report["segments"][-1]["status"], "failed")
        self.assertEqual(report["error"], "return transition unreachable")
        self.assertEqual(device.calls, [])
        device.plan_joint.assert_called_once()

    def test_failed_plan_retains_inputs_without_stop_or_upload(self):
        device = self.device_at_receive()
        device.plan_linear = MagicMock(side_effect=MotionError("planning failed: -1020"))
        report = {}
        self.assertFalse(check_current_plan(device, Config(), report))
        self.assertEqual(device.calls, [])
        self.assertEqual(report["segments"][0]["status"], "failed")
        self.assertEqual(len(report["segments"][0]["arm_base_target_matrices"]), 2)
        self.assertEqual(report["error"], "planning failed: -1020")

    def test_error_feedback_is_rejected_without_enabling_or_stopping(self):
        device = Device(Config())
        device.rt["arms"][1]["state"]["cur"] = 100
        report = {}
        self.assertFalse(check_current_plan(device, Config(), report))
        self.assertEqual(device.calls, [])
        self.assertEqual(report["segments"], [])

    def test_wrong_lift_height_is_rejected_without_control_commands(self):
        device = Device(Config())
        report = {}
        self.assertFalse(check_current_plan(device, Config(), report))
        self.assertIn("lift 偏离", report["error"])
        self.assertEqual(device.calls, [])
        self.assertEqual(report["segments"], [])

    def test_cli_plan_check_uses_separate_result_and_does_not_enable(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "plan-check.json"
            device = self.device_at_receive()
            device.close = MagicMock()
            device.plan_linear = MagicMock(side_effect=MotionError("planning failed: -1020"))
            with patch("skye_task.__main__.GentoSDK", return_value=device), redirect_stdout(StringIO()):
                self.assertEqual(main(["plan-check", "--result", str(path)]), 1)
            report = json.loads(path.read_text())
            self.assertEqual(report["mode"], "read_only_plan_check")
            self.assertEqual(report["status"], "failed")
            self.assertEqual(device.calls, [])
            device.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()

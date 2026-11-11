"""Verify SDK call contracts using a transport double, without a hardware connection."""

from copy import deepcopy
from ctypes import Structure, c_double, c_int32, c_ubyte
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, call

from skye_task.config import Config
from skye_task.sdk import Feedback, GentoSDK, MotionError, Trajectory
from test_sequence import Clock, Device, IDENTITY


class PlanningParams(Structure):
    _fields_ = [("Max_Body_Start_PRR", c_double * 3)] + [
        item for arm in range(2) for item in (
            (f"Arm{arm}_Start_XYZABC", c_double * 6),
            (f"Arm{arm}_End_XYZABC", c_double * 6),
            (f"Arm{arm}_Ref_Joints", c_double * 7),
            (f"Arm{arm}_ZSP_Type", c_int32),
            (f"Arm{arm}_ZSP_Para", c_double * 6))
    ] + [("Freq", c_int32), ("Vel", c_double), ("Acc", c_double),
         ("Sync_Type", c_int32), ("World_Co_Flag", c_ubyte)]


class SDKTests(unittest.TestCase):
    def setUp(self):
        self.config = Config()
        self.clock = Clock()
        self.robot = MagicMock()
        self.robot.max_point = 5000
        self.robot.get_sdk_version.return_value = "00040701"
        self.robot.get_controller_version.return_value = "00040703"
        self.robot.get_robot_type.return_value = "GentoSkye"
        self.robot.get_ctrl_obj_dof.side_effect = {0: 7, 1: 7, 2: 2, 3: 2, 4: 1}.get
        self.robot.link.return_value = 42  # Positive link return is success, not an error.
        self.robot.check_link_state.return_value = 1
        self.robot.init_single_arm_config.return_value = 0
        self.robot.set_tool.return_value = 0
        self.robot.config_set_traj.return_value = 0
        self.robot.runtime_run_traj.return_value = 3
        self.robot.runtime_stop_traj.return_value = 3
        self.robot.switch_to_position_mode.return_value = 0
        self.robot.runtime_set_joint_pos_cmd.return_value = 0
        self.robot.runtime_set_speed_ratio.return_value = 0
        self.robot.xyzabc2matrix.return_value = deepcopy(IDENTITY)
        self.robot.matrix2xyzabc.side_effect = lambda m: [m[i][3] for i in range(3)] + [0] * 3
        device = Device(self.config)
        self.feedback = Feedback(device.rt, device.sg)
        self.sdk = GentoSDK(self.config, module=SimpleNamespace(ArmsSynchronousPlanningParams=PlanningParams),
                            robot=self.robot, clock=self.clock)

    def test_positive_link_result_and_patch_difference_are_accepted(self):
        metadata = self.sdk.connect()
        self.assertTrue(self.sdk.connected)
        self.assertEqual(metadata["effective_dofs"], {"left": 7, "right": 7, "head": 2,
                                                      "body": 2, "lift": 1})
        self.robot.link.assert_called_once_with(6, 6, 7, 190, log_level=14)

    def test_wrong_sdk_version_does_not_connect(self):
        self.robot.get_sdk_version.return_value = "00040601"
        with self.assertRaisesRegex(MotionError, "版本"):
            self.sdk.connect()
        self.robot.link.assert_not_called()

    def test_connection_error_does_not_enable(self):
        self.robot.link.return_value = -4
        with self.assertRaisesRegex(MotionError, "连接失败"):
            self.sdk.connect()
        self.robot.switch_to_position_mode.assert_not_called()

    def test_controller_version_is_checked_before_enabling(self):
        self.robot.get_controller_version.return_value = "00040501"
        with self.assertRaisesRegex(MotionError, "版本"):
            self.sdk.connect()
        self.robot.switch_to_position_mode.assert_not_called()
        self.sdk.close()
        self.robot.unlink.assert_called_once()

    def test_effective_axes_mismatch_does_not_enable(self):
        self.robot.get_ctrl_obj_dof.side_effect = {0: 7, 1: 7, 2: 3, 3: 2, 4: 1}.get
        with self.assertRaisesRegex(MotionError, "有效自由度"):
            self.sdk.connect()
        self.robot.switch_to_position_mode.assert_not_called()

    def test_partial_start_does_not_count_as_dual_success(self):
        for mask in (0, 1, 2, -1):
            with self.subTest(mask=mask):
                self.robot.runtime_run_traj.return_value = mask
                with self.assertRaises(MotionError):
                    self.sdk.run()
        self.assertEqual(self.robot.runtime_run_traj.call_args, call(3))
        self.robot.runtime_run_traj.return_value = 7
        self.sdk.run()  # A returned superset contains both required arms.

    def test_partial_stop_is_reported(self):
        self.robot.runtime_stop_traj.return_value = 1
        with self.assertRaisesRegex(MotionError, "停止双臂"):
            self.sdk.stop()

    def test_fixed_hold_pads_sdk_arrays_without_extra_axes(self):
        self.sdk.enter_position(self.feedback, lambda: None)
        self.robot.runtime_set_joint_pos_cmd.assert_any_call(2, [0, 0, 0.])
        self.robot.runtime_set_joint_pos_cmd.assert_any_call(3, [0, 0, 0., 0., 0., 0.])
        self.robot.runtime_set_joint_pos_cmd.assert_any_call(4, [500, 0.])
        self.robot.runtime_set_speed_ratio.assert_has_calls([call(0, 100, 100), call(1, 100, 100)])

    def test_body_fk_argument_order_is_lift_pitch_yaw(self):
        self.robot.skye_body_forward_kinematics.return_value = (IDENTITY, IDENTITY)
        self.sdk.shoulders(self.feedback)
        self.robot.skye_body_forward_kinematics.assert_called_once_with([500, 0, 0])

    def test_lift_command_uses_position_mm_and_padded_second_slot(self):
        self.sdk.move_lift(450)
        self.robot.runtime_set_speed_ratio.assert_called_once_with(4, 10, 10)
        self.robot.runtime_set_joint_pos_cmd.assert_called_once_with(4, [450, 0.])

    def test_lift_hold_uses_fresh_measured_position_and_keeps_enabled(self):
        self.feedback.rt["lift"]["state"]["cur"] = 1
        self.feedback.rt["lift"]["fb_pos"][0] = 477.3
        self.robot.get_rt_dict.return_value = self.feedback.rt
        self.robot.get_sg_dict.return_value = self.feedback.sg
        self.sdk.hold_lift()
        self.robot.runtime_set_joint_pos_cmd.assert_called_once_with(4, [477.3, 0.])
        self.robot.switch_to_position_mode.assert_not_called()

    def test_lift_fault_during_stop_does_not_send_an_invalid_hold_command(self):
        self.feedback.rt["lift"]["state"] = {"cur": 100, "err": 42}
        self.robot.get_rt_dict.return_value = self.feedback.rt
        self.robot.get_sg_dict.return_value = self.feedback.sg
        with self.assertRaisesRegex(MotionError, "需要人工处理"):
            self.sdk.hold_lift()
        self.robot.runtime_set_joint_pos_cmd.assert_not_called()

    def test_movej_reinitializes_selected_arm_and_uses_fraction_ratios(self):
        self.sdk.tools = [[0] * 6, [0] * 6]
        points = [0.] * 7 + [1.] * 7
        self.robot.plan_joints.return_value = (points, 2)
        result = self.sdk.plan_joint(0, [0.] * 7, [1.] * 7)
        self.assertEqual(result.endpoint, [1.] * 7)
        self.robot.init_single_arm_config.assert_called_once_with(0)
        self.robot.plan_joints.assert_called_once_with(0, [0.] * 7, [1.] * 7, .1, .1, 50)

    def test_linear_planning_uses_local_frames_current_joints_and_fixed_orientation(self):
        targets = [deepcopy(IDENTITY), deepcopy(IDENTITY)]
        targets[0][0][3], targets[1][0][3] = 150., 150.
        self.robot.plan_linear_synchronous.return_value = ([0.] * 14, [0.] * 14, 2)
        self.sdk.plan_linear(self.feedback, [IDENTITY, IDENTITY], targets, self.config.forward)
        params = self.robot.plan_linear_synchronous.call_args.args[0]
        self.assertEqual(params.World_Co_Flag, 0)
        self.assertEqual(list(params.Arm0_End_XYZABC), [150, 0, 0, 0, 0, 0])
        self.assertEqual(list(params.Arm1_End_XYZABC), [150, 0, 0, 0, 0, 0])
        self.assertEqual(list(params.Arm0_Ref_Joints), self.feedback.joints[0])
        self.assertEqual((params.Freq, params.Vel, params.Acc), (50, 100, 100))
        self.assertEqual(params.Arm0_ZSP_Type, 0)
        self.assertEqual(params.Arm1_ZSP_Type, 0)
        self.assertEqual(list(params.Arm0_ZSP_Para), [0] * 6)
        self.assertEqual(list(params.Arm1_ZSP_Para), [0] * 6)

    def test_stationary_trajectory_is_converted_to_sdk_ctypes_buffer(self):
        self.sdk.upload(0, Trajectory([0.] * 35, 5))
        arm, points, count = self.robot.config_set_traj.call_args.args
        self.assertEqual((arm, count), (0, 5))
        self.assertIsInstance(points, c_double * 35)

    def test_short_planned_trajectories_keep_all_points_and_hold_endpoint_to_five(self):
        for count in range(1, 5):
            with self.subTest(count=count):
                points = [float(i) for i in range(count * 7)]
                trajectory = self.sdk._trajectory(points, count)
                self.assertEqual(trajectory.count, 5)
                self.assertEqual(trajectory.points[:count * 7], points)
                self.assertEqual(trajectory.points[count * 7:], points[-7:] * (5 - count))
                self.assertEqual(trajectory.startpoint, points[:7])
                self.assertEqual(trajectory.endpoint, points[-7:])
                self.assertEqual(len(points), count * 7)

    def test_upload_rejects_invalid_point_counts_before_calling_native_sdk(self):
        for count in (0, 1, 2, 3, 4, 5001):
            with self.subTest(count=count):
                with self.assertRaisesRegex(MotionError, "轨迹点数.*拒绝上传"):
                    self.sdk.upload(0, Trajectory([0.] * (count * 7), count))
        self.robot.config_set_traj.assert_not_called()

    def test_stale_rt_or_sg_frames_are_rejected(self):
        for source in ("rt", "sg"):
            with self.subTest(source=source):
                self.sdk._frames.clear()
                rt, sg = deepcopy(self.feedback.rt), deepcopy(self.feedback.sg)
                self.robot.get_rt_dict.side_effect = lambda: deepcopy(rt)
                self.robot.get_sg_dict.side_effect = lambda: deepcopy(sg)
                self.sdk.read()
                self.clock.wait(1.1)
                (sg if source == "rt" else rt)["frame_serial"] += 1
                with self.assertRaisesRegex(MotionError, source + " 反馈帧"):
                    self.sdk.read()

    def test_link_loss_is_rejected_before_reading(self):
        self.robot.check_link_state.return_value = -1
        with self.assertRaisesRegex(MotionError, "连接中断"):
            self.sdk.read()
        self.robot.get_rt_dict.assert_not_called()

    def test_invalid_and_overflow_trajectories_are_rejected(self):
        for points, count in (([], 0), ([0] * 7, 2), ([float("nan")] * 7, 1), ([], 5001)):
            with self.subTest(count=count):
                with self.assertRaises(MotionError):
                    self.sdk._trajectory(points, count)


if __name__ == "__main__":
    unittest.main()

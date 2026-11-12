"""Offline control tests. These doubles never load a robot library or control grippers."""

from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest

from skye_task.config import Config, load_config
from skye_task.geometry import local_displacement, pose_error, translated
from skye_task.interfaces import Gripper, GripperAction, GripperTarget
from skye_task.sdk import Feedback, MotionError, Trajectory
from skye_task.sequence import Sequence, Stopped


IDENTITY = [[1., 0., 0., 0.], [0., 1., 0., 0.],
            [0., 0., 1., 0.], [0., 0., 0., 1.]]
LEFT_SHOULDER = [[1, 0, 0, 0], [0, 0, 1, 230.5],
                 [0, -1, 0, 933.333], [0, 0, 0, 1]]
RIGHT_SHOULDER = [[1, 0, 0, 0], [0, 0, -1, -230.5],
                  [0, 1, 0, 933.333], [0, 0, 0, 1]]


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now

    def wait(self, seconds):
        self.now += seconds


class Device:
    """Joint 1..3 encode XYZ solely to exercise orchestration against feedback."""
    def __init__(self, config):
        self.config = config
        self.connected = False
        self.tools = None
        self.rt = {"frame_serial": 0, "arms": [self.arm(), self.arm()]}
        for name in ("body", "head", "lift"):
            self.rt[name] = {"state": {"cur": 0, "err": 0},
                             "fb_pos": list(config.ready_lift if name == "lift"
                                            else getattr(config, name)) + [999] * 4}
        self.sg = {"frame_serial": 0, "arms": [
            {"set": {"tool_kine": [0] * 6, "vel_ratio": 100, "acc_ratio": 100},
             "get": {"traj_state": 0}} for _ in range(2)]}
        self.sg.update({name: {"get": {"traj_state": 0}} for name in ("body", "lift")})
        self.calls = []
        self.uploads = {}
        self.running = None
        self.tick = 0
        self.never_start = False
        self.never_finish = False
        self.wrong_endpoint = False
        self.omit_running_state = False
        self.fail_upload = None
        self.partial_start = False
        self.stop_failure = False
        self.plan_failure = False
        self.on_read = lambda: None
        self.linear_deltas = []
        self.lift_target = None
        self.lift_stuck = False

    @staticmethod
    def arm():
        return {"state": {"cur": 0, "err": 0},
                "fb": {"fb_pos": [0.] * 7, "fb_vel": [0.] * 7}}

    def connect(self):
        self.connected = True
        return {"sdk_version": "00040701", "controller_version": "00040701"}

    def read(self):
        self.rt["frame_serial"] += 1
        self.sg["frame_serial"] += 1
        if self.lift_target is not None and not self.lift_stuck:
            self.rt["lift"]["fb_pos"][0] = self.lift_target
        if self.running is not None:
            self.tick += 1
            for arm in range(2):
                state = 0 if self.never_start or self.tick == 1 else 3
                if not self.never_start and self.tick >= 2:
                    self.rt["arms"][arm]["fb"]["fb_pos"] = self.running[arm].endpoint
                if self.wrong_endpoint:
                    self.rt["arms"][arm]["fb"]["fb_pos"] = [0.] * 7
                if (self.tick >= 3 and not self.never_finish) or self.omit_running_state:
                    state = 0
                self.sg["arms"][arm]["get"]["traj_state"] = state
        self.on_read()
        return Feedback(deepcopy(self.rt), deepcopy(self.sg))

    def enter_position(self, feedback, checkpoint):
        for arm in self.rt["arms"]:
            arm["state"]["cur"] = 1
        for name in ("head", "body", "lift"):
            self.rt[name]["state"]["cur"] = 1
        self.tools = [[0] * 6 for _ in range(2)]

    def initialize_kinematics(self, feedback):
        self.tools = [[0] * 6 for _ in range(2)]

    def move_lift(self, target):
        self.calls.append(("move_lift", target))
        self.lift_target = target

    def hold_lift(self):
        self.calls.append(("hold_lift", self.rt["lift"]["fb_pos"][0]))
        self.lift_target = self.rt["lift"]["fb_pos"][0]

    def forward(self, arm, joints):
        pose = deepcopy(IDENTITY)
        for i in range(3):
            pose[i][3] = joints[i]
        return pose

    def shoulders(self, feedback):
        return LEFT_SHOULDER, RIGHT_SHOULDER

    def plan_joint(self, arm, start, target):
        self.calls.append(("plan_joint", arm))
        if self.plan_failure:
            raise MotionError("planning failure")
        return Trajectory(list(start) + list(target) * 4, 5)

    def plan_linear(self, feedback, starts, targets, motion):
        result = []
        self.linear_deltas.append([[targets[a][i][3] - starts[a][i][3] for i in range(3)]
                                   for a in range(2)])
        for arm in range(2):
            joints = list(feedback.joints[arm])
            for i in range(3):
                joints[i] = targets[arm][i][3]
            result.append(Trajectory(list(feedback.joints[arm]) + joints * 4, 5))
        return result

    def upload(self, arm, trajectory):
        if not 5 <= trajectory.count <= 5000:
            raise MotionError("SDK Runtime_InitTraj rejects point count: -19")
        self.calls.append(("upload", arm))
        if arm == self.fail_upload:
            raise MotionError("upload failure")
        self.uploads[arm] = trajectory
        self.running = None
        self.sg["arms"][arm]["get"]["traj_state"] = 2

    def run(self):
        self.calls.append(("run", 3))
        self.running = [self.uploads[0], self.uploads[1]]
        self.tick = 0
        if self.partial_start:
            raise MotionError("partial start mask=1")

    def stop(self):
        self.calls.append(("stop", 3))
        self.running = None
        if self.stop_failure:
            raise MotionError("stop failed")


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.config = replace(Config(), stable_seconds=.04, start_timeout_seconds=.15,
                              execution_timeout_seconds=.5, poll_seconds=.02, gripper_enabled=False)
        self.clock = Clock()
        self.device = Device(self.config)
        self.sequence = Sequence(self.device, self.config, clock=self.clock, wait=self.clock.wait)

    def test_full_unloaded_sequence_and_opposite_arm_frames(self):
        self.sequence.prepare()
        self.assertEqual(self.sequence.result.status, "running")
        self.sequence.start()
        result = self.sequence.result
        self.assertTrue(result.motion_completed)
        self.assertEqual(result.status, "completed")
        self.assertIsNone(result.grasped)
        self.assertIsNone(result.placed)
        self.assertEqual(result.gripper_actions, "not_executed")
        self.assertEqual(result.navigation, "not_called")
        self.assertEqual(self.device.linear_deltas, [
            [[100, 0, 0], [100, 0, 0]],
            [[0, 100, 0], [0, -100, 0]],
            [[0, -100, 0], [0, 100, 0]],
        ])
        completed = [s["name"] for s in result.steps if s["status"] == "completed"]
        self.assertEqual(completed, ["initialized", "ready", "ready_lift", "transition", "receive",
                                     "receive_lift", "forward", "down", "up", "return_transition", "return_ready",
                                     "return_ready_lift", "finished"])
        self.assertEqual([c for c in self.device.calls if c[0] == "move_lift"],
                         [("move_lift", 500), ("move_lift", 450), ("move_lift", 500)])
        self.assertEqual(self.sequence.fixed["lift"], [500])
        for i, call in enumerate(self.device.calls):
            if call == ("run", 3):
                self.assertEqual(self.device.calls[i-2:i], [("upload", 0), ("upload", 1)])
        self.assertNotIn(("stop", 3), self.device.calls)
        json.dumps(result.to_dict(), allow_nan=False)

    def test_transition_is_used_on_outbound_and_return_paths_at_fixed_lift_heights(self):
        moves = []
        original_run = self.device.run

        def capture():
            moves.append(([list(self.device.uploads[arm].endpoint) for arm in range(2)],
                          self.device.rt["lift"]["fb_pos"][0]))
            original_run()

        self.device.run = capture
        self.sequence.prepare()
        self.assertEqual(moves, [(self.config.ready, 500), (self.config.transition, 500),
                                 (self.config.receive, 500)])
        self.assertEqual(self.device.rt["lift"]["fb_pos"][0], 450)
        self.sequence.start()
        # Up reverses down; both return poses are reached before lifting the column.
        self.assertEqual(len(moves), 8)
        self.assertEqual(moves[3][0], moves[5][0])
        self.assertTrue(all(height == 450 for _, height in moves[3:]))
        self.assertEqual(moves[-2:], [(self.config.transition, 450), (self.config.ready, 450)])
        self.assertEqual(sum(targets == self.config.transition for targets, _ in moves), 2)
        self.assertEqual(self.device.rt["lift"]["fb_pos"][0], 500)

    def test_up_must_finish_before_any_return_joint_motion(self):
        self.sequence.prepare()

        def stall(name, detail):
            if name == "up" and detail["status"] == "started":
                self.device.never_finish = True

        self.sequence.on_event = stall
        with self.assertRaisesRegex(MotionError, "up 执行超时"):
            self.sequence.start()
        names = [step["name"] for step in self.sequence.result.steps]
        self.assertNotIn("return_transition", names)
        self.assertNotIn("return_ready", names)
        self.assertEqual(self.device.calls[-1], ("stop", 3))
        self.assertEqual(self.device.rt["lift"]["fb_pos"][0], 450)

    def test_return_transition_must_finish_before_return_ready(self):
        self.sequence.prepare()

        def stall(name, detail):
            if name == "return_transition" and detail["status"] == "started":
                self.device.never_finish = True

        self.sequence.on_event = stall
        with self.assertRaisesRegex(MotionError, "return_transition 执行超时"):
            self.sequence.start()
        names = [step["name"] for step in self.sequence.result.steps]
        self.assertNotIn("return_ready", names)
        self.assertNotIn("return_ready_lift", names)
        self.assertEqual(self.device.calls[-1], ("stop", 3))
        self.assertEqual(self.device.rt["lift"]["fb_pos"][0], 450)

    def test_transition_must_finish_before_receive_or_lift_descent(self):
        def stall(name, detail):
            if name == "transition" and detail["status"] == "started":
                self.device.never_finish = True

        self.sequence.on_event = stall
        with self.assertRaisesRegex(MotionError, "transition 执行超时"):
            self.sequence.prepare()
        self.assertFalse(self.sequence.prepared)
        self.assertEqual(self.sequence.result.status, "failed")
        self.assertEqual(self.device.calls.count(("run", 3)), 2)
        self.assertNotIn("receive", [step["name"] for step in self.sequence.result.steps])
        self.assertNotIn(("move_lift", 450), self.device.calls)
        self.assertEqual(self.device.calls[-1], ("stop", 3))

    def test_cancellation_at_transition_stops_before_receive(self):
        def cancel(name, detail):
            if name == "transition" and detail["status"] == "started":
                self.sequence.request_stop()

        self.sequence.on_event = cancel
        with self.assertRaises(Stopped):
            self.sequence.prepare()
        self.assertFalse(self.sequence.prepared)
        self.assertEqual(self.sequence.result.status, "stopped")
        self.assertEqual(self.device.calls.count(("run", 3)), 2)
        self.assertNotIn("receive", [step["name"] for step in self.sequence.result.steps])
        self.assertNotIn(("move_lift", 450), self.device.calls)
        self.assertEqual(self.device.calls[-1], ("stop", 3))

    def test_consecutive_runs_from_final_ready_pose_upload_valid_hold_trajectories(self):
        self.sequence.prepare()
        self.sequence.start()
        self.assertEqual(self.sequence.result.status, "completed")
        for arm in range(2):
            self.assertEqual(self.device.rt["arms"][arm]["fb"]["fb_pos"], self.config.ready[arm])
        second = Sequence(self.device, self.config, clock=self.clock, wait=self.clock.wait)
        uploads = []
        original_upload = self.device.upload
        def capture(arm, trajectory):
            uploads.append((arm, trajectory))
            original_upload(arm, trajectory)
        self.device.upload = capture
        second.prepare()
        second.start()
        self.assertEqual(second.result.status, "completed")
        for arm, trajectory in uploads[:2]:
            self.assertEqual(trajectory.count, 5)
            self.assertEqual(trajectory.points, self.config.ready[arm] * 5)
        ready_start = next(s for s in second.result.steps
                           if s["name"] == "ready" and s["status"] == "started")
        self.assertEqual(ready_start["point_counts"], [5, 5])
        self.assertNotIn(("stop", 3), self.device.calls)

    def test_idle_zero_before_start_is_not_completion(self):
        self.device.never_start = True
        with self.assertRaisesRegex(MotionError, "未确认双臂开始"):
            self.sequence.prepare()
        self.assertGreaterEqual(self.clock.now, .15)
        self.assertEqual(self.sequence.result.status, "failed")
        self.assertEqual(self.device.calls[-1], ("stop", 3))

    def test_short_motion_with_missed_running_state_uses_actual_movement(self):
        self.device.omit_running_state = True
        self.sequence.prepare()
        self.sequence.start()
        self.assertTrue(self.sequence.result.motion_completed)

    def test_actual_position_required_even_after_trajectory_finishes(self):
        self.device.wrong_endpoint = True
        with self.assertRaisesRegex(MotionError, "执行超时"):
            self.sequence.prepare()
        self.assertFalse(self.sequence.result.motion_completed)

    def test_executing_trajectory_at_target_is_not_finished(self):
        self.device.never_finish = True
        with self.assertRaisesRegex(MotionError, "执行超时"):
            self.sequence.prepare()

    def test_upload_failure_never_starts_and_stops_both(self):
        self.device.fail_upload = 1
        with self.assertRaisesRegex(MotionError, "upload failure"):
            self.sequence.prepare()
        self.assertNotIn(("run", 3), self.device.calls)
        self.assertEqual(self.device.calls[-1], ("stop", 3))

    def test_partial_start_stops_both_and_does_not_continue(self):
        self.device.partial_start = True
        with self.assertRaisesRegex(MotionError, "partial start"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls.count(("run", 3)), 1)
        self.assertEqual(self.device.calls[-1], ("stop", 3))

    def test_trajectory_first_point_jump_is_rejected_before_upload(self):
        original = self.device.plan_joint
        def discontinuous(arm, start, target):
            trajectory = original(arm, start, target)
            trajectory.points[0] += 76.2
            return trajectory
        self.device.plan_joint = discontinuous
        with self.assertRaisesRegex(MotionError, "轨迹首点偏离"):
            self.sequence.prepare()
        self.assertNotIn(("upload", 0), self.device.calls)
        self.assertNotIn(("run", 3), self.device.calls)

    def test_failed_linear_plan_preserves_both_input_poses(self):
        self.sequence.prepare()
        def unreachable(*args):
            raise MotionError("双臂同步直线规划失败: -1020")
        self.device.plan_linear = unreachable
        with self.assertRaisesRegex(MotionError, "-1020"):
            self.sequence.start()
        record = self.sequence.result.steps[-1]
        self.assertEqual(record["name"], "forward_target")
        self.assertEqual(record["status"], "planning")
        self.assertEqual(len(record["arm_base_start_matrices"]), 2)
        self.assertEqual(len(record["arm_base_target_matrices"]), 2)
        self.assertEqual(record["reference_joints_deg"], self.config.receive)

    def test_planning_failure_stops_without_retry(self):
        self.device.plan_failure = True
        with self.assertRaisesRegex(MotionError, "planning failure"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls, [("plan_joint", 0), ("stop", 3)])

    def test_stop_failure_keeps_original_failure(self):
        self.device.plan_failure = True
        self.device.stop_failure = True
        with self.assertRaisesRegex(MotionError, "planning failure"):
            self.sequence.prepare()
        self.assertEqual(self.sequence.result.error, "planning failure")
        self.assertEqual(self.sequence.result.stop_error, "stop failed")

    def test_fixed_axis_drift_aborts(self):
        def drift():
            if self.device.running:
                self.device.rt["lift"]["fb_pos"][0] += 2
        self.device.on_read = drift
        with self.assertRaisesRegex(MotionError, "lift"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls[-1], ("stop", 3))

    def test_active_body_trajectory_rejected_before_enabling(self):
        self.device.sg["body"]["get"]["traj_state"] = 3
        with self.assertRaisesRegex(MotionError, "body 正在执行轨迹"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls, [("stop", 3)])

    def test_error_state_aborts(self):
        def fault():
            if self.device.running:
                self.device.rt["arms"][1]["state"]["err"] = 42
        self.device.on_read = fault
        with self.assertRaisesRegex(MotionError, "状态异常"):
            self.sequence.prepare()

    def test_tcp_change_aborts(self):
        def change_tool():
            if self.device.running:
                self.device.sg["arms"][0]["set"]["tool_kine"][0] = 5
        self.device.on_read = change_tool
        with self.assertRaisesRegex(MotionError, "TCP 定义发生变化"):
            self.sequence.prepare()

    def test_cancellation_stops_without_return_motion(self):
        def cancel():
            if self.device.running:
                self.sequence.request_stop()
        self.device.on_read = cancel
        with self.assertRaises(Stopped):
            self.sequence.prepare()
        self.assertEqual(self.sequence.result.status, "stopped")
        self.assertEqual(self.device.calls[-1], ("stop", 3))

    def test_task_trigger_revalidates_receive_pose(self):
        self.sequence.prepare()
        self.device.running = None
        self.device.rt["arms"][0]["fb"]["fb_pos"][0] += 2
        with self.assertRaisesRegex(MotionError, "偏离接盘位"):
            self.sequence.start()

    def test_receive_height_is_reached_before_trigger_and_held_for_linear_work(self):
        self.sequence.prepare()
        self.assertEqual(self.device.rt["lift"]["fb_pos"][0], 450)
        self.assertEqual(self.sequence.fixed["lift"], [450])
        seen = []
        original = self.device.plan_linear
        def plan(feedback, *args):
            seen.append(feedback.rt["lift"]["fb_pos"][0])
            return original(feedback, *args)
        self.device.plan_linear = plan
        self.sequence.start()
        self.assertEqual(seen, [450, 450, 450])
        self.assertEqual(self.sequence.result.metadata["initial_fixed_feedback"]["lift"], [500])

    def test_rerun_at_receive_height_first_reaches_full_ready_pose(self):
        self.device.rt["lift"]["fb_pos"][0] = 450
        self.sequence.prepare()
        steps = [s for s in self.sequence.result.steps if s["status"] == "completed"]
        self.assertEqual(next(s for s in steps if s["name"] == "ready_lift")["actual_mm"], 500)
        self.assertEqual(self.device.rt["lift"]["fb_pos"][0], 450)

    def test_lift_stuck_stops_and_holds_without_task_trigger(self):
        self.device.lift_stuck = True
        with self.assertRaisesRegex(MotionError, "receive_lift 未确认升降开始"):
            self.sequence.prepare()
        self.assertFalse(self.sequence.prepared)
        self.assertEqual(self.device.calls[-2:], [("stop", 3), ("hold_lift", 500)])

    def test_lift_progress_without_reaching_target_times_out_and_holds_actual_height(self):
        def stall_halfway():
            if self.sequence.lift_command_active and self.device.lift_target == 450:
                self.device.rt["lift"]["fb_pos"][0] = 475
        self.device.on_read = stall_halfway
        with self.assertRaisesRegex(MotionError, "receive_lift 执行超时"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls[-2:], [("stop", 3), ("hold_lift", 475)])
        self.assertFalse(self.sequence.prepared)

    def test_arm_stop_failure_still_attempts_lift_hold(self):
        self.device.lift_stuck = True
        self.device.stop_failure = True
        with self.assertRaisesRegex(MotionError, "receive_lift 未确认升降开始"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls[-1], ("hold_lift", 500))
        self.assertEqual(self.sequence.result.stop_error, "stop failed")

    def test_lift_completion_requires_both_feedback_streams_to_advance(self):
        def freeze_sg_during_lift():
            if self.sequence.lift_command_active:
                self.device.sg["frame_serial"] -= 1
        self.device.on_read = freeze_sg_during_lift
        with self.assertRaisesRegex(MotionError, "ready_lift 执行超时"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls[-1], ("hold_lift", 500))

    def test_cancel_during_lift_holds_measured_position(self):
        def cancel():
            if self.sequence.lift_command_active:
                self.sequence.request_stop()
        self.device.on_read = cancel
        with self.assertRaises(Stopped):
            self.sequence.prepare()
        self.assertEqual(self.sequence.result.status, "stopped")
        self.assertEqual(self.device.calls[-2:], [("stop", 3), ("hold_lift", 500)])
        self.assertEqual(self.device.calls.count(("run", 3)), 1)

    def test_arm_movement_during_lift_aborts_and_holds_lift(self):
        def drift():
            if self.sequence.lift_command_active:
                self.device.rt["arms"][0]["fb"]["fb_pos"][0] += 2
        self.device.on_read = drift
        with self.assertRaisesRegex(MotionError, "升降调整期间双臂"):
            self.sequence.prepare()
        self.assertEqual(self.device.calls[-1][0], "hold_lift")

    def test_lift_drift_before_task_start_is_rejected(self):
        self.sequence.prepare()
        self.device.lift_target = 460
        with self.assertRaisesRegex(MotionError, "lift 偏离"):
            self.sequence.start()
        self.assertEqual(self.device.linear_deltas, [])

    def test_feedback_padding_is_not_an_effective_axis(self):
        self.sequence.prepare()
        self.assertEqual(len(self.sequence.fixed["body"]), 2)
        self.assertEqual(len(self.sequence.fixed["head"]), 2)
        self.assertEqual(len(self.sequence.fixed["lift"]), 1)

    def test_stationary_arm_participates_in_dual_mask(self):
        self.device.rt["arms"][0]["fb"]["fb_pos"] = list(self.config.ready[0])
        self.sequence.prepare()
        self.assertEqual(self.device.calls.count(("run", 3)), 3)
        ready_start = next(s for s in self.sequence.result.steps
                           if s["name"] == "ready" and s["status"] == "started")
        self.assertEqual(ready_start["point_counts"][0], 5)

    def test_unstable_feedback_resets_stability_timer(self):
        self.config = replace(self.config, stable_seconds=.08)
        self.sequence.config = self.config
        def disturb():
            if self.device.running and self.device.tick == 4:
                self.device.rt["arms"][0]["fb"]["fb_vel"][0] = 2
            else:
                self.device.rt["arms"][0]["fb"]["fb_vel"][0] = 0
        self.device.on_read = disturb
        self.sequence.prepare()
        completions = [s for s in self.sequence.result.steps
                       if s.get("execution_seconds") and "target_mm" not in s]
        self.assertTrue(all(s["execution_seconds"] >= .16 for s in completions))


class GeometryAndConfigTests(unittest.TestCase):
    def test_rotated_waist_preserves_robot_vector(self):
        # 90-degree waist yaw plus shoulder mounting; R R^T delta = delta.
        shoulder = [[0, 0, -1, 1], [1, 0, 0, 2], [0, -1, 0, 3], [0, 0, 0, 1]]
        delta = [150, 0, -150]
        local = local_displacement(shoulder, delta)
        restored = [sum(shoulder[i][j] * local[j] for j in range(3)) for i in range(3)]
        self.assertEqual(restored, delta)

    def test_translation_preserves_rotation_and_does_not_modify_start(self):
        start = deepcopy(LEFT_SHOULDER)
        target = translated(start, [1, 2, 3])
        self.assertEqual(start, LEFT_SHOULDER)
        self.assertEqual([r[:3] for r in start], [r[:3] for r in target])
        distance, angle = pose_error(start, target)
        self.assertAlmostEqual(distance, math.sqrt(14))
        self.assertAlmostEqual(angle, 0)

    def test_gripper_entry_is_unimplemented_for_all_targets_and_actions(self):
        gripper = Gripper()
        for target in GripperTarget:
            for action in GripperAction:
                with self.assertRaises(NotImplementedError):
                    gripper.control(target, action)
        self.assertIsNone(gripper.is_grasped())
        self.assertIsNone(gripper.is_placed())

    def test_default_and_partial_config(self):
        self.assertEqual(load_config(), Config())
        configured = json.loads(Path("config/default.json").read_text(encoding="utf-8"))
        self.assertEqual(load_config("config/default.json").receive, configured["receive"])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            path.write_text('{"forward": {"velocity_mm_s": 30}}')
            config = load_config(path)
            self.assertEqual(config.forward.velocity_mm_s, 30)
            self.assertEqual(config.forward.frequency_hz, 50)

    def test_invalid_config_is_rejected_offline(self):
        invalid = [
            {"forward_mm": -1}, {"poll_seconds": 0}, {"stable_seconds": float("nan")},
            {"joint": {"velocity_ratio": 10}}, {"head": [0, 0, 0]},
            {"joint": {"frequency_hz": 100}}, {"controller_ip": "example.com"},
            {"execution_timeout_seconds": .5}, {"mode_timeout_ms": 2.5},
            {"unknown": 1}, {"forward": {"frequency_hz": 50.5}},
            {"up_mm": True}, {"up_mm": 0}, {"up": {"frequency_hz": 100}},
            {"forward_mm": "150"},
            {"ready_lift": [500, 0]}, {"receive_lift": [float("nan")]},
            {"transition": []}, {"transition": [[0] * 7]},
            {"transition": [[0] * 6, [0] * 7]},
            {"transition": [[float("nan")] * 7, [0] * 7]},
            {"gripper_enabled": 1}, {"gripper_enabled": "true"},
        ]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            for override in invalid:
                with self.subTest(override=override):
                    path.write_text(json.dumps(override))
                    with self.assertRaises(ValueError):
                        load_config(path)


if __name__ == "__main__":
    unittest.main()

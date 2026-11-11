"""Fixed-base arm and gripper task with feedback-based completion and stopping."""

from dataclasses import asdict, dataclass, field
from copy import deepcopy
from datetime import datetime, timezone
from functools import wraps
import math
import threading
import time

from .geometry import local_displacement, pose_error, translated
from .interfaces import GripperAction, GripperTarget
from .sdk import MIN_TRAJECTORY_POINTS, MotionError, Trajectory


FIXED_AXES = {"body": 2, "head": 2, "lift": 1}


class Stopped(MotionError):
    pass


def abort_on_failure(method):
    @wraps(method)
    def guarded(self, *args, **kwargs):
        try:
            return method(self, *args, **kwargs)
        except (Exception, KeyboardInterrupt) as error:
            self.fail(error)
            raise
    return guarded


@dataclass
class Result:
    started_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    finished_at: str = ""
    status: str = "running"
    mode: str = "unloaded_fixed_base"
    motion_completed: bool = False
    grasped: object = None
    placed: object = None
    grasp_check: str = "skipped"
    placement_check: str = "skipped"
    gripper_actions: str = "not_executed"
    handover: str = "skipped"
    navigation: str = "not_called"
    steps: list = field(default_factory=list)
    metadata: dict = field(default_factory=dict)
    error: str = ""
    stop_error: str = ""

    def to_dict(self):
        return asdict(self)


class Sequence:
    def __init__(self, sdk, config, *, gripper=None, clock=time.monotonic, wait=None, on_event=None):
        self.sdk, self.config = sdk, config
        self.gripper = gripper if config.gripper_enabled else None
        self.clock = clock
        self.stop_requested = threading.Event()
        self.wait = wait or self.stop_requested.wait
        self.on_event = on_event or (lambda name, details: None)
        self.result = Result()
        if config.gripper_enabled:
            self.result.mode = "fixed_base_with_grippers"
        self.fixed = None
        self.lift_motion = None
        self.lift_command_active = False
        self.prepared = False
        self.started = False

    def request_stop(self):
        """May be called from a signal handler or a UI thread; never resumes a sequence."""
        self.stop_requested.set()

    def _checkpoint(self):
        if self.stop_requested.is_set():
            raise Stopped("收到停止请求")

    def _record(self, name, **details):
        self.result.steps.append({"name": name, "elapsed_seconds": self.clock() - self.begin,
                                  **details})
        self.on_event(name, details)

    @abort_on_failure
    def prepare(self):
        """Connect, enter position control, reach each arm pose and its lift height in turn."""
        if self.prepared or self.started or self.result.status != "running":
            raise MotionError("流程不可重复初始化")
        self.begin = self.clock()
        self._checkpoint()
        if self.config.gripper_enabled and self.gripper is None:
            raise MotionError("夹爪已启用但未提供实际驱动，拒绝执行双臂任务")
        if self.gripper is not None:
            self.gripper.reserve()
        self.result.metadata = self.sdk.connect()
        feedback = self.sdk.read()
        self._guard(feedback, require_position=False)
        if any(state >= 3 for state in feedback.trajectory_states):
            raise MotionError("已有双臂轨迹正在执行，需人工处理")
        self.fixed = {name: list(feedback.rt[name]["fb_pos"][:size])
                      for name, size in FIXED_AXES.items()}
        self.sdk.enter_position(feedback, self._checkpoint)
        self.result.metadata["tool_xyzabc_mm_deg"] = self.sdk.tools
        self.result.metadata["initial_fixed_feedback"] = deepcopy(self.fixed)
        self.result.metadata["lift_targets_mm"] = {"ready": self.config.ready_lift,
                                                   "receive": self.config.receive_lift}
        if self.gripper is not None:
            self.gripper.connect()
            self.result.metadata["gripper_config"] = asdict(self.gripper.config)
        self._record("initialized", status="completed")
        self._move_joint("ready", self.config.ready)
        self._move_lift("ready_lift", self.config.ready_lift[0])
        if self.gripper is not None:
            self._gripper_action("open_before_receive", GripperAction.OPEN)
        else:
            self._record("open_before_receive", status="skipped", reason="配置关闭夹爪，空载验证")
        self._move_joint("receive", self.config.receive)
        self._move_lift("receive_lift", self.config.receive_lift[0])
        if self.gripper is None:
            self._record("handover_and_close", status="skipped", reason="空载验证，无交接等待")
        self.prepared = True
        self._record("awaiting_task_start", status="waiting")

    @abort_on_failure
    def start(self):
        """Task trigger at receive pose; all following steps run without manual nodes."""
        if not self.prepared or self.started or self.result.status != "running":
            raise MotionError("任务只能在接盘位触发一次")
        self._checkpoint()
        self.started = True
        feedback = self.checked_feedback()
        if any(not self._near(actual, target) for actual, target in
               zip(feedback.joints, self.config.receive)):
            raise MotionError("触发任务时双臂已偏离接盘位")
        if any(feedback.trajectory_states):
            raise MotionError("触发任务时双臂轨迹尚未空闲")
        if self.gripper is not None:
            self.result.handover = "started"
            self._gripper_action("handover_and_close", GripperAction.CLOSE)
            self._wait_checked("handover_wait", self.config.handover_wait_seconds)
            self.result.handover = "completed"
        self._record("navigation", status="skipped", reason="固定底盘，未调用导航")
        self._move_linear("forward", [[self.config.forward_mm, 0, 0]] * 2, self.config.forward)
        self._move_linear("down", [[0, 0, -self.config.down_mm]] * 2, self.config.down)
        if self.gripper is not None:
            self._gripper_action("release", GripperAction.OPEN)
        else:
            self._record("release", status="skipped", reason="配置关闭夹爪，无开合指令或等待")
        self._move_linear("exit", [[0, self.config.left_exit_mm, 0],
                                   [0, -self.config.right_exit_mm, 0]], self.config.exit)
        self._move_joint("return_ready", self.config.ready)
        self._move_lift("return_ready_lift", self.config.ready_lift[0])
        self.result.motion_completed = True
        if self.gripper is not None:
            self.result.gripper_actions = "completed"
        self.result.status = "completed"
        self.result.finished_at = datetime.now(timezone.utc).isoformat()
        self._record("finished", status="completed", grasped=None, placed=None)

    def fail(self, error):
        """Stop arms and any active lift command, retaining position control."""
        if self.result.status in ("failed", "stopped"):
            return
        self.result.status = "stopped" if isinstance(error, (Stopped, KeyboardInterrupt)) else "failed"
        self.result.error = str(error) or type(error).__name__
        self.result.finished_at = datetime.now(timezone.utc).isoformat()
        if self.gripper is not None:
            try:
                self.gripper.freeze()
            except Exception as stop_error:
                self.result.metadata["gripper_freeze_error"] = str(stop_error)
            if self.result.gripper_actions == "started":
                self.result.gripper_actions = "incomplete"
            if self.result.handover == "started":
                self.result.handover = "incomplete"
        if self.sdk.connected:
            stop_errors = []
            try:
                self.sdk.stop()
            except Exception as stop_error:
                stop_errors.append(str(stop_error))
            if self.lift_command_active:
                try:
                    self.sdk.hold_lift()
                except Exception as stop_error:
                    stop_errors.append(str(stop_error))
            self.result.stop_error = "; ".join(stop_errors)
        self.on_event("aborted", {"status": self.result.status, "error": self.result.error,
                                  "stop_error": self.result.stop_error})

    def checked_feedback(self):
        self._checkpoint()
        feedback = self.sdk.read()
        self._checkpoint()
        self._guard(feedback)
        if self.gripper is not None:
            self.gripper.check_health()
        return feedback

    def _stationary_checkpoint(self, joints):
        feedback = self.checked_feedback()
        if (any(feedback.trajectory_states)
                or any(not self._near(actual, target) for actual, target in zip(feedback.joints, joints))
                or any(abs(v) > self.config.joint_velocity_tolerance_deg_s
                       for arm in feedback.rt["arms"] for v in arm["fb"]["fb_vel"])):
            raise MotionError("夹爪动作或交接等待期间双臂未保持静止")

    def _wait_checked(self, name, seconds):
        joints = self.checked_feedback().joints
        self._record(name, status="started", wait_seconds=seconds)
        until = self.clock() + seconds
        while True:
            self._stationary_checkpoint(joints)
            remaining = until - self.clock()
            if remaining <= 0:
                break
            self.wait(min(self.config.poll_seconds, remaining))
        self._record(name, status="completed", wait_seconds=seconds)

    def _gripper_action(self, name, action):
        joints = self.checked_feedback().joints
        self.result.gripper_actions = "started"
        targets = self.gripper.config.action_positions(action.value, "both")
        self._record(name, status="started", action=action.value, targets_rad=targets)
        started = self.clock()
        try:
            sample = self.gripper.control(GripperTarget.BOTH, action,
                                          lambda: self._stationary_checkpoint(joints))
        except (Exception, KeyboardInterrupt) as error:
            try:
                snapshot = self.gripper.snapshot()
            except Exception as snapshot_error:
                snapshot = {"snapshot_error": str(snapshot_error)}
            self._record(name, status="failed", action=action.value, targets_rad=targets,
                         error=str(error), feedback=snapshot)
            raise
        self._record(name, status="completed", action=action.value, targets_rad=targets,
                     feedback=sample, execution_seconds=self.clock() - started)
        self._wait_checked(name + "_wait", self.config.gripper_wait_seconds)

    def _guard(self, feedback, *, require_position=True):
        for arm in feedback.rt["arms"]:
            self._state(arm, require_position)
            values = arm["fb"]["fb_pos"]
            if len(values) != 7 or any(not math.isfinite(v) for v in values):
                raise MotionError("机械臂关节反馈无效")
            velocities = arm["fb"]["fb_vel"]
            if len(velocities) != 7 or any(not math.isfinite(v) for v in velocities):
                raise MotionError("机械臂速度反馈无效")
        if len(feedback.rt["arms"]) != 2:
            raise MotionError("缺少双臂反馈")
        if len(feedback.trajectory_states) != 2 or any(s < 0 for s in feedback.trajectory_states):
            raise MotionError("双臂轨迹状态无效")
        for arm in feedback.sg["arms"]:
            tool = arm["set"]["tool_kine"]
            if len(tool) != 6 or any(not math.isfinite(v) for v in tool):
                raise MotionError("TCP 配置反馈无效")
        for name, size in FIXED_AXES.items():
            component = feedback.rt[name]
            self._state(component, require_position)
            if name in ("body", "lift") and feedback.sg[name]["get"]["traj_state"] >= 3:
                raise MotionError(f"{name} 正在执行轨迹，不能进行固定躯干双臂规划")
            actual = component["fb_pos"][:size]
            tolerance = (self.config.fixed_lift_tolerance_mm if name == "lift"
                         else self.config.fixed_angle_tolerance_deg)
            if len(actual) != size or any(not math.isfinite(v) for v in actual):
                raise MotionError(f"{name} 有效轴反馈无效")
            if name == "lift":
                if self.lift_motion:
                    lower, upper = sorted((self.lift_motion["start"], self.lift_motion["target"]))
                elif self.fixed:
                    lower = upper = self.fixed["lift"][0]
                else:
                    # Accept a rerun at either taught height or between them.
                    lower, upper = sorted((self.config.ready_lift[0], self.config.receive_lift[0]))
                if not lower - tolerance <= actual[0] <= upper + tolerance:
                    raise MotionError(f"lift 偏离当前阶段允许高度: {actual}")
            else:
                if any(abs(a - b) > tolerance for a, b in zip(actual, getattr(self.config, name))):
                    raise MotionError(f"{name} 偏离示教时固定姿态: {actual}")
                if self.fixed and any(abs(a - b) > tolerance for a, b in zip(actual, self.fixed[name])):
                    raise MotionError(f"{name} 在动作期间发生位移")
        if self.lift_motion:
            if (any(feedback.trajectory_states)
                    or any(not self._near(a, b) for a, b in
                           zip(feedback.joints, self.lift_motion["joints"]))
                    or any(abs(v) > self.config.joint_velocity_tolerance_deg_s
                           for arm in feedback.rt["arms"] for v in arm["fb"]["fb_vel"])):
                raise MotionError("升降调整期间双臂未保持静止")
        if self.sdk.tools is not None:
            for arm in range(2):
                tool = feedback.sg["arms"][arm]["set"]["tool_kine"]
                if len(tool) != 6 or any(not math.isfinite(v) for v in tool):
                    raise MotionError("TCP 配置反馈无效")
                if any(abs(a - b) > 1e-3 for a, b in zip(tool, self.sdk.tools[arm])):
                    raise MotionError("动作期间控制器 TCP 定义发生变化")

    @staticmethod
    def _state(component, require_position):
        state = component["state"]
        if state["err"] != 0 or state["cur"] not in ((1,) if require_position else (0, 1)):
            raise MotionError(f"控制对象状态异常: {state}")

    def _near(self, actual, target):
        return all(abs(a - b) <= self.config.joint_tolerance_deg for a, b in zip(actual, target))

    def _move_lift(self, name, target):
        feedback = self.checked_feedback()
        if any(feedback.trajectory_states):
            raise MotionError("双臂轨迹未空闲，不能调整升降")
        initial = feedback.rt["lift"]["fb_pos"][0]
        self.lift_motion = {"start": initial, "target": target, "joints": feedback.joints}
        self._guard(feedback)
        # Mark active before calling: a transport error may occur after the command was accepted.
        self.lift_command_active = True
        self.sdk.move_lift(target)
        self._record(name, status="started", start_mm=initial, target_mm=target)
        started = previous_time = self.clock()
        previous_position = initial
        previous_frames = (feedback.rt["frame_serial"], feedback.sg["frame_serial"])
        progress = abs(initial - target) <= self.config.fixed_lift_tolerance_mm
        stable_since = None
        while True:
            feedback = self.checked_feedback()
            now = self.clock()
            elapsed = now - started
            actual = feedback.rt["lift"]["fb_pos"][0]
            frames = (feedback.rt["frame_serial"], feedback.sg["frame_serial"])
            fresh = all(a != b for a, b in zip(frames, previous_frames))
            if elapsed >= self.config.execution_timeout_seconds:
                raise MotionError(f"{name} 执行超时")
            if fresh:
                progress |= abs(actual - initial) > 0.01
                # The SDK exposes no lift velocity; estimate from fresh position frames.
                velocity = (abs(actual - previous_position) / (now - previous_time)
                            if now > previous_time else math.inf)
                settled = (abs(actual - target) <= self.config.fixed_lift_tolerance_mm
                           and velocity <= self.config.lift_velocity_tolerance_mm_s)
                if settled:
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= self.config.stable_seconds:
                        self.fixed["lift"] = [target]
                        self.lift_motion = None
                        self.lift_command_active = False
                        self._record(name, status="completed", target_mm=target,
                                     actual_mm=actual, execution_seconds=elapsed)
                        return
                else:
                    stable_since = None
                previous_time, previous_position, previous_frames = now, actual, frames
            if elapsed >= self.config.start_timeout_seconds and not progress:
                raise MotionError(f"{name} 未确认升降开始执行")
            self.wait(self.config.poll_seconds)

    def _move_joint(self, name, targets):
        feedback = self.checked_feedback()
        trajectories = []
        # A stationary arm still participates in the single dual-arm start mask.
        for arm in range(2):
            if self._near(feedback.joints[arm], targets[arm]):
                trajectories.append(Trajectory(list(feedback.joints[arm]) * MIN_TRAJECTORY_POINTS,
                                               MIN_TRAJECTORY_POINTS))
            else:
                trajectories.append(self.sdk.plan_joint(arm, feedback.joints[arm], targets[arm]))
            self._checkpoint()
        for trajectory, target in zip(trajectories, targets):
            if not self._near(trajectory.endpoint, target):
                raise MotionError("MoveJ 规划终点与目标关节值不一致")
        self._execute(name, feedback, trajectories, targets)

    def _move_linear(self, name, deltas, motion):
        feedback = self.checked_feedback()
        starts = [self.sdk.forward(arm, feedback.joints[arm]) for arm in range(2)]
        shoulders = self.sdk.shoulders(feedback)
        targets = [translated(starts[arm], local_displacement(shoulders[arm], deltas[arm]))
                   for arm in range(2)]
        # Preserve inputs even when the native planner fails before producing any points.
        self._record(name + "_target", status="planning", robot_displacements_mm=deltas,
                     reference_joints_deg=feedback.joints, arm_base_start_matrices=starts,
                     arm_base_target_matrices=targets,
                     arm_base_displacements_mm=[local_displacement(shoulders[i], deltas[i])
                                                for i in range(2)],
                     null_space_strategy="nearest_reference_joints")
        trajectories = self.sdk.plan_linear(feedback, starts, targets, motion)
        self._checkpoint()
        for arm, trajectory in enumerate(trajectories):
            if not self._pose_near(self.sdk.forward(arm, trajectory.endpoint), targets[arm]):
                raise MotionError(f"{name}: arm{arm} 规划终点未达到目标位姿")
        self._record(name + "_target", status="planned", robot_displacements_mm=deltas,
                     arm_base_target_matrices=targets)
        self._execute(name, feedback, trajectories, [t.endpoint for t in trajectories], targets)

    def _pose_near(self, actual, target):
        distance, angle = pose_error(actual, target)
        return (distance <= self.config.position_tolerance_mm
                and angle <= self.config.orientation_tolerance_deg)

    def _execute(self, name, planned_from, trajectories, joint_targets, pose_targets=None):
        for arm, trajectory in enumerate(trajectories):
            if not self._near(trajectory.startpoint, planned_from.joints[arm]):
                raise MotionError(f"{name}: arm{arm} 轨迹首点偏离当前关节值，拒绝上传")
        latest = self.checked_feedback()
        if any(s >= 3 for s in latest.trajectory_states):
            raise MotionError("规划期间出现其他正在执行的轨迹")
        if any(not self._near(a, b) for a, b in zip(latest.joints, planned_from.joints)):
            raise MotionError("规划后起点已变化，停止执行；不自动重新规划")
        for arm, trajectory in enumerate(trajectories):
            self.sdk.upload(arm, trajectory)
            self.checked_feedback()
        # Both uploads must finish before the one combined runtime_run_traj call.
        before_run = self.checked_feedback()
        if any(arm["set"][key] != 100 for arm in before_run.sg["arms"]
               for key in ("vel_ratio", "acc_ratio")):
            raise MotionError("运行时速度/加速度比例未达到 100%，轨迹执行时基可能改变")
        if any(not self._near(a, b) for a, b in zip(before_run.joints, planned_from.joints)):
            raise MotionError("上传期间双臂起点已变化")
        self.sdk.run()
        self._record(name, status="started", point_counts=[t.count for t in trajectories])
        start_time = self.clock()
        progress = [False, False]
        stationary = [self._near(before_run.joints[i], joint_targets[i]) for i in range(2)]
        duration = [(t.count - 1) / self.config.controller_trajectory_hz for t in trajectories]
        stable_since = None
        last_frames = (before_run.rt["frame_serial"], before_run.sg["frame_serial"])
        while True:
            feedback = self.checked_feedback()
            now = self.clock()
            elapsed = now - start_time
            fresh_rt = feedback.rt["frame_serial"] != before_run.rt["frame_serial"]
            fresh_sg = feedback.sg["frame_serial"] != before_run.sg["frame_serial"]
            for arm in range(2):
                moving = any(abs(a - b) > 0.01 for a, b in
                             zip(feedback.joints[arm], before_run.joints[arm]))
                progress[arm] |= ((fresh_sg and feedback.trajectory_states[arm] >= 3)
                                  or (fresh_rt and moving)
                                  or (stationary[arm] and fresh_sg and elapsed >= duration[arm]))
            if elapsed >= self.config.execution_timeout_seconds:
                raise MotionError(f"{name} 执行超时")
            if elapsed >= self.config.start_timeout_seconds and not all(progress):
                raise MotionError(f"{name} 未确认双臂开始执行: {progress}")
            at_target = all(self._near(a, b) for a, b in zip(feedback.joints, joint_targets))
            if at_target and pose_targets is not None:
                at_target = all(self._pose_near(self.sdk.forward(i, feedback.joints[i]), pose_targets[i])
                                for i in range(2))
            complete = (all(progress) and fresh_rt and fresh_sg and at_target
                        and feedback.trajectory_states == [0, 0] and elapsed >= max(duration)
                        and all(abs(v) <= self.config.joint_velocity_tolerance_deg_s
                                for arm in feedback.rt["arms"] for v in arm["fb"]["fb_vel"]))
            frames = (feedback.rt["frame_serial"], feedback.sg["frame_serial"])
            new_feedback = all(a != b for a, b in zip(frames, last_frames))
            if complete:
                if stable_since is None:
                    stable_since = now
                if new_feedback and now - stable_since >= self.config.stable_seconds:
                    self._record(name, status="completed", execution_seconds=elapsed)
                    return
            else:
                stable_since = None
            last_frames = frames
            self.wait(self.config.poll_seconds)

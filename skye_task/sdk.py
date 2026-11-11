"""Thin adapter for the locally installed GentoRobot 0407 wrapper."""

from dataclasses import dataclass
from ctypes import c_double
import importlib.util
from ipaddress import IPv4Address
import math
import os
from pathlib import Path
import time


# SDK 0407 RobotCtrl::{Arm0,Arm1}_Runtime_InitTraj accepts 5..5000 points.
MIN_TRAJECTORY_POINTS = 5


class MotionError(RuntimeError):
    pass


@dataclass
class Trajectory:
    points: object
    count: int

    @property
    def startpoint(self):
        return list(self.points[:7])

    @property
    def endpoint(self):
        return list(self.points[(self.count - 1) * 7:self.count * 7])


@dataclass
class Feedback:
    rt: dict
    sg: dict

    @property
    def joints(self):
        return [arm["fb"]["fb_pos"] for arm in self.rt["arms"]]

    @property
    def trajectory_states(self):
        return [arm["get"]["traj_state"] for arm in self.sg["arms"]]


class GentoSDK:
    ARM_MASK = 3
    OBJECTS = {"left": 0, "right": 1, "head": 2, "body": 3, "lift": 4}
    WIDTHS = {"left": 7, "right": 7, "head": 3, "body": 6, "lift": 2}
    EFFECTIVE_DOFS = {"left": 7, "right": 7, "head": 2, "body": 2, "lift": 1}

    def __init__(self, config, *, module=None, robot=None, clock=time.monotonic):
        self.config = config
        self.clock = clock
        self.connected = False
        self.tools = None
        self._frames = {}
        if module is None:
            root = Path(os.environ.get("GENTO_SDK_ROOT", config.sdk_root)).expanduser()
            wrapper = root / "PYTHON_SDK" / "GentoRobot.py"
            if not wrapper.is_file():
                raise MotionError(f"找不到 SDK: {wrapper}")
            spec = importlib.util.spec_from_file_location("skye_gento_sdk_0407", wrapper)
            module = importlib.util.module_from_spec(spec)
            # Avoid generating bytecode in the external SDK directory.
            source = wrapper.read_bytes()
            exec(compile(source, str(wrapper), "exec"), module.__dict__)
        self.module = module
        self.robot = robot if robot is not None else module.GentoRobot(precision=6)

    @staticmethod
    def _ok(result, operation):
        if result != 0:
            raise MotionError(f"{operation} 失败，SDK 返回 {result}")

    def connect(self):
        sdk_version = self.robot.get_sdk_version()
        self._version(sdk_version, "SDK")
        result = self.robot.link(*IPv4Address(self.config.controller_ip).packed, log_level=14)
        if result < 0:
            raise MotionError(f"连接失败，SDK 返回 {result}")
        self.connected = True
        controller_version = self.robot.get_controller_version()
        self._version(controller_version, "控制器")
        if sdk_version[:6].upper() != controller_version[:6].upper():
            raise MotionError("SDK 与控制器 MAJOR/MINOR 不一致")
        if self.robot.get_robot_type() != "GentoSkye":
            raise MotionError("控制器机器人类型不是 GentoSkye")
        dofs = {}
        for name, obj in self.OBJECTS.items():
            expected = self.EFFECTIVE_DOFS[name]
            dofs[name] = self.robot.get_ctrl_obj_dof(obj)
            if dofs[name] != expected:
                raise MotionError(f"{name} 有效自由度不匹配: SDK={dofs[name]}, 配置={expected}")
        return {"sdk_version": sdk_version, "controller_version": controller_version,
                "robot_type": "GentoSkye", "effective_dofs": dofs,
                "controller_trajectory_hz": self.config.controller_trajectory_hz}

    def _version(self, version, name):
        if not isinstance(version, str) or not version.upper().startswith(self.config.expected_version_prefix):
            raise MotionError(f"{name} 版本不符合 0407: {version}")

    def read(self):
        if self.robot.check_link_state() != 1:
            raise MotionError("控制器连接中断或无法接收反馈")
        rt, sg = self.robot.get_rt_dict(), self.robot.get_sg_dict()
        now = self.clock()
        for name, data in (("rt", rt), ("sg", sg)):
            if "error" in data:
                raise MotionError(f"{name} 反馈失败: {data['error']}")
            frame = data["frame_serial"]
            last_frame, last_change = self._frames.get(name, (None, now))
            if frame != last_frame:
                self._frames[name] = (frame, now)
            elif now - last_change >= self.config.feedback_timeout_seconds:
                raise MotionError(f"{name} 反馈帧停止更新")
        return Feedback(rt, sg)

    def enter_position(self, feedback, checkpoint):
        # Refuse automatic error resets and transitions out of other control modes.
        for name, obj in self.OBJECTS.items():
            checkpoint()
            data = feedback.rt["arms"][obj] if obj < 2 else feedback.rt[name]
            state = data["state"]["cur"]
            if state not in (0, 1):
                raise MotionError(f"{name} 当前模式 {state}，需人工处理后再运行")
            if state == 0:
                self._ok(self.robot.switch_to_position_mode(
                    obj, self.config.mode_timeout_ms, self.config.mode_velocity_percent,
                    self.config.mode_acceleration_percent), f"{name} 进入位置模式")
            checkpoint()
            values = data["fb"]["fb_pos"] if obj < 2 else data["fb_pos"]
            effective = self.EFFECTIVE_DOFS[name]
            hold = list(values[:effective]) + [0.0] * (self.WIDTHS[name] - effective)
            self._ok(self.robot.runtime_set_joint_pos_cmd(obj, hold), f"{name} 保持当前位置")
            if obj < 2:
                # Planner ratios are fractions, whereas these runtime limits are percentages.
                self._ok(self.robot.runtime_set_speed_ratio(obj, 100, 100), f"{name} 设置运行比例")
        self.initialize_kinematics(feedback, checkpoint)

    def move_lift(self, target_mm):
        # Direct position commands use controller interpolation, independently of arm trajectories.
        self._ok(self.robot.runtime_set_speed_ratio(
            self.OBJECTS["lift"], self.config.mode_velocity_percent,
            self.config.mode_acceleration_percent), "升降设置运行比例")
        self._ok(self.robot.runtime_set_joint_pos_cmd(self.OBJECTS["lift"], [target_mm, 0.0]),
                 "升降位置指令")

    def hold_lift(self):
        """Replace an active lift target with its fresh position; keep position mode enabled."""
        feedback = self.read()
        lift = feedback.rt["lift"]
        if lift["state"]["cur"] != 1 or lift["state"]["err"] != 0:
            raise MotionError("升降状态异常，无法保持当前位置，需要人工处理")
        position = lift["fb_pos"][0]
        if not math.isfinite(position):
            raise MotionError("升降反馈无效，无法保持当前位置")
        self._ok(self.robot.runtime_set_joint_pos_cmd(self.OBJECTS["lift"], [position, 0.0]),
                 "升降停止并保持当前位置")

    def initialize_kinematics(self, feedback, checkpoint=lambda: None):
        """Read controller model parameters and set local planning tools; no motion commands."""
        self.tools = [list(arm["set"]["tool_kine"]) for arm in feedback.sg["arms"]]
        for arm in range(2):
            checkpoint()
            self._init_arm(arm)

    def _init_arm(self, arm):
        # 0407's MoveJ planner uses the most recently initialized shared arm model.
        self._ok(self.robot.init_single_arm_config(arm), f"arm{arm} 初始化运动学")
        self._ok(self.robot.set_tool(arm, self.robot.xyzabc2matrix(self.tools[arm])),
                 f"arm{arm} 设置规划 TCP")

    def forward(self, arm, joints):
        result = self.robot.forward_kinematics(arm, joints)
        self._matrix(result, "正运动学")
        return result

    def shoulders(self, feedback):
        # SDK CFxKineMAX takes [lift_mm, waist_pitch_deg, waist_yaw_deg].
        body = feedback.rt["body"]["fb_pos"]
        lift = feedback.rt["lift"]["fb_pos"][0]
        result = self.robot.skye_body_forward_kinematics([lift, body[0], body[1]])
        if not isinstance(result, tuple) or len(result) != 2:
            raise MotionError(f"躯干正运动学失败: {result}")
        for matrix in result:
            self._matrix(matrix, "肩部参考系")
        return result

    @staticmethod
    def _matrix(value, operation):
        if (not isinstance(value, (tuple, list)) or len(value) != 4
                or any(len(row) != 4 or any(not math.isfinite(v) for v in row) for row in value)):
            raise MotionError(f"{operation} 返回无效矩阵: {value}")

    def plan_joint(self, arm, start, target):
        self._init_arm(arm)
        motion = self.config.joint
        result = self.robot.plan_joints(arm, start, target, motion.velocity_ratio,
                                        motion.acceleration_ratio, motion.frequency_hz)
        if not isinstance(result, tuple) or len(result) != 2:
            raise MotionError(f"arm{arm} MoveJ 规划失败: {result}")
        return self._trajectory(*result)

    def plan_linear(self, feedback, starts, targets, motion):
        params = self.module.ArmsSynchronousPlanningParams()
        params.World_Co_Flag = 0  # Explicitly converted to each arm's base by the caller.
        params.Sync_Type = 0
        params.Freq = motion.frequency_hz
        params.Vel = motion.velocity_mm_s
        params.Acc = motion.acceleration_mm_s2
        for arm in range(2):
            getattr(params, f"Arm{arm}_Ref_Joints")[:] = feedback.joints[arm]
            getattr(params, f"Arm{arm}_Start_XYZABC")[:] = self.robot.matrix2xyzabc(starts[arm])
            getattr(params, f"Arm{arm}_End_XYZABC")[:] = self.robot.matrix2xyzabc(targets[arm])
            # Follow the measured joint configuration. The vendor example's type 1
            # imposes a new elbow plane and can change joints at the very first point.
            setattr(params, f"Arm{arm}_ZSP_Type", 0)
            getattr(params, f"Arm{arm}_ZSP_Para")[:] = [0] * 6
        result = self.robot.plan_linear_synchronous(params)
        if not isinstance(result, tuple) or len(result) != 3:
            raise MotionError(f"双臂同步直线规划失败: {result}")
        return [self._trajectory(result[arm], result[2]) for arm in range(2)]

    def _trajectory(self, points, count):
        if type(count) is not int or not 1 <= count <= self.robot.max_point:
            raise MotionError(f"无效轨迹点数: {count}")
        if len(points) < count * 7 or any(not math.isfinite(points[i]) for i in range(count * 7)):
            raise MotionError("轨迹包含无效关节值")
        if count < MIN_TRAJECTORY_POINTS:
            # Preserve every planned point; hold the endpoint for the remaining samples.
            endpoint = list(points[(count - 1) * 7:count * 7])
            points = list(points[:count * 7]) + endpoint * (MIN_TRAJECTORY_POINTS - count)
            count = MIN_TRAJECTORY_POINTS
        return Trajectory(points, count)

    def upload(self, arm, trajectory):
        if (type(trajectory.count) is not int
                or not MIN_TRAJECTORY_POINTS <= trajectory.count <= self.robot.max_point):
            raise MotionError(f"arm{arm} 轨迹点数 {trajectory.count} 不满足 SDK 要求 "
                              f"{MIN_TRAJECTORY_POINTS}..{self.robot.max_point}，拒绝上传")
        points = trajectory.points
        if isinstance(points, list):
            points = (c_double * len(points))(*points)
        self._ok(self.robot.config_set_traj(arm, points, trajectory.count),
                 f"arm{arm} 上传轨迹")

    def run(self):
        mask = self.robot.runtime_run_traj(self.ARM_MASK)
        if mask < 0 or mask & self.ARM_MASK != self.ARM_MASK:
            raise MotionError(f"双臂轨迹启动失败或部分启动: 掩码={mask:#x}")

    def stop(self):
        mask = self.robot.runtime_stop_traj(self.ARM_MASK)
        if mask < 0 or mask & self.ARM_MASK != self.ARM_MASK:
            raise MotionError(f"停止双臂轨迹未完全成功: 掩码={mask:#x}，需要人工处理")

    def close(self):
        try:
            if self.connected:
                self.robot.unlink()
        finally:
            self.connected = False
            self.robot.cleanup()

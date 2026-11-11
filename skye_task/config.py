"""Configuration in SDK units: degrees, millimetres and seconds."""

from dataclasses import asdict, dataclass, field, fields, is_dataclass, replace
from ipaddress import IPv4Address
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class JointMotion:
    velocity_ratio: float = 0.1
    acceleration_ratio: float = 0.1
    frequency_hz: int = 50


@dataclass(frozen=True)
class LinearMotion:
    velocity_mm_s: float = 100.0
    acceleration_mm_s2: float = 100.0
    frequency_hz: int = 50


@dataclass(frozen=True)
class Config:
    sdk_root: str = "/home/gento/robot_sdk/tianji-robot-SDK"
    controller_ip: str = "6.6.7.190"
    expected_version_prefix: str = "000407"
    # The upload API carries points only, with no sampling-frequency field.
    # Set this to the controller's verified execution frequency before hardware use.
    controller_trajectory_hz: int = 50
    ready: list = field(default_factory=lambda: [
        [90, -64, -67, -89, -24, -8, 17],
        [-90, -64, 67, -89, 24, -8, -17],
    ])
    receive: list = field(default_factory=lambda: [
        [85, -70, -60, -90, 15, 0, 10],
        [-85, -70, 60, -90, -15, 0, -10],
    ])
    # Only these explicitly listed axes are effective; remaining SDK slots are padding.
    body: list = field(default_factory=lambda: [0, 0])
    head: list = field(default_factory=lambda: [0, 0])
    ready_lift: list = field(default_factory=lambda: [500])
    receive_lift: list = field(default_factory=lambda: [450])
    forward_mm: float = 100.0
    down_mm: float = 100.0
    left_exit_mm: float = 100.0
    right_exit_mm: float = 100.0
    joint: JointMotion = field(default_factory=JointMotion)
    forward: LinearMotion = field(default_factory=LinearMotion)
    down: LinearMotion = field(default_factory=LinearMotion)
    exit: LinearMotion = field(default_factory=LinearMotion)
    mode_velocity_percent: float = 10.0
    mode_acceleration_percent: float = 10.0
    mode_timeout_ms: int = 2000
    joint_tolerance_deg: float = 0.5
    joint_velocity_tolerance_deg_s: float = 0.5
    position_tolerance_mm: float = 2.0
    orientation_tolerance_deg: float = 1.0
    fixed_angle_tolerance_deg: float = 0.2
    fixed_lift_tolerance_mm: float = 1.0
    lift_velocity_tolerance_mm_s: float = 1.0
    stable_seconds: float = 1.0
    start_timeout_seconds: float = 3.0
    execution_timeout_seconds: float = 120.0
    feedback_timeout_seconds: float = 1.0
    poll_seconds: float = 0.02
    gripper_enabled: bool = True
    gripper_wait_seconds: float = 1.0
    handover_wait_seconds: float = 1.0

    def validate(self):
        IPv4Address(self.controller_ip)
        if not self.sdk_root or self.expected_version_prefix != "000407":
            raise ValueError("本实现要求 SDK/控制器版本前缀 000407")
        for name in ("ready", "receive"):
            points = getattr(self, name)
            if not isinstance(points, list) or len(points) != 2:
                raise ValueError(f"{name} 必须包含左右臂两个点位")
            for point in points:
                _vector(point, 7, name)
        for name, size in (("body", 2), ("head", 2), ("ready_lift", 1), ("receive_lift", 1)):
            _vector(getattr(self, name), size, name)
        if type(self.gripper_enabled) is not bool:
            raise ValueError("gripper_enabled 必须为布尔值")
        for name, value in asdict(self).items():
            if name != "gripper_enabled" and isinstance(value, (float, int)):
                _positive(value, name)
        for name in ("controller_trajectory_hz", "mode_timeout_ms"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} 必须是整数")
        for name in ("mode_velocity_percent", "mode_acceleration_percent"):
            if getattr(self, name) > 100:
                raise ValueError(f"{name} 不能大于 100")
        for name in ("joint", "forward", "down", "exit"):
            motion = getattr(self, name)
            for key, value in asdict(motion).items():
                _positive(value, f"{name}.{key}")
            if type(motion.frequency_hz) is not int:
                raise ValueError(f"{name}.frequency_hz 必须是整数")
            if motion.frequency_hz != self.controller_trajectory_hz:
                raise ValueError(f"{name} 采样频率必须与控制器轨迹执行频率一致")
        if not 0 < self.joint.velocity_ratio <= 1 or not 0 < self.joint.acceleration_ratio <= 1:
            raise ValueError("MoveJ 的速度、加速度比例必须在 (0, 1] 内")
        if self.execution_timeout_seconds <= self.stable_seconds:
            raise ValueError("执行超时必须大于稳定时间")
        if self.start_timeout_seconds > self.execution_timeout_seconds:
            raise ValueError("启动超时不能大于执行超时")
        if self.poll_seconds >= self.feedback_timeout_seconds:
            raise ValueError("反馈超时必须大于轮询周期")
        return self


def _positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须是有限正数")


def _vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} 必须有 {size} 个有效轴")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f"{name} 含无效数值")


def _merge(default, overrides):
    if not isinstance(overrides, dict):
        raise ValueError("配置必须是 JSON 对象")
    known = {f.name for f in fields(default)}
    if overrides.keys() - known:
        raise ValueError(f"未知配置项: {sorted(overrides.keys() - known)}")
    updates = {}
    for key, value in overrides.items():
        old = getattr(default, key)
        if is_dataclass(old):
            updates[key] = _merge(old, value)
        else:
            if isinstance(old, str) and not isinstance(value, str):
                raise ValueError(f"{key} 必须是字符串")
            if type(old) in (int, float) and type(value) not in (int, float):
                raise ValueError(f"{key} 必须是数值")
            updates[key] = value
    return replace(default, **updates)


def load_config(path=None):
    config = Config()
    if path is not None:
        with Path(path).open(encoding="utf-8") as stream:
            config = _merge(config, json.load(stream))
    return config.validate()

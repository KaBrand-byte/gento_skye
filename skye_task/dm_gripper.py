"""Independent DM4310 diagnostics with configured open/close directions."""

from dataclasses import dataclass, field
import fcntl
import json
import math
import os
from pathlib import Path
import re
import socket
import struct
import threading
import time
from typing import Optional

from .config import _merge, _positive
from .dm_protocol import (DISABLE, ENABLE, STATUS_NAMES, decode_feedback, encode_mit,
                          pack_frame)
from .sdk import MotionError


@dataclass(frozen=True)
class CANSide:
    interface: str = "vcan0"
    slave_id: int = 1
    master_id: int = 0x11
    # User verified on both installed grippers: increasing angle closes the jaws.
    close_direction: int = 1
    open_position_rad: Optional[float] = None
    close_position_rad: Optional[float] = None

    def validate(self):
        if not re.fullmatch(r"[a-zA-Z0-9_.-]{1,15}", self.interface):
            raise ValueError("CAN 接口名称无效")
        if type(self.slave_id) is not int or not 1 <= self.slave_id <= 15:
            raise ValueError("DM4310 slave_id 必须为 1..15，以匹配反馈的 4 bit ID")
        if type(self.master_id) is not int or not 0 <= self.master_id <= 0x7FF:
            raise ValueError("master_id 必须为 11 bit 标准 CAN ID")
        if type(self.close_direction) is not int or self.close_direction not in (-1, 1):
            raise ValueError("close_direction 必须为 +1 或 -1")
        for name in ("open_position_rad", "close_position_rad"):
            value = getattr(self, name)
            if value is not None and (type(value) not in (int, float) or not math.isfinite(value)):
                raise ValueError(f"{name} 必须为有限数值或 null")
        if self.open_position_rad is not None and self.close_position_rad is not None:
            if (self.close_position_rad - self.open_position_rad) * self.close_direction <= 0:
                raise ValueError("开合目标角度的顺序与 close_direction 不一致")


@dataclass(frozen=True)
class DMGripperConfig:
    left: CANSide = field(default_factory=CANSide)
    right: CANSide = field(default_factory=lambda: CANSide("vcan1", 2, 0x12))
    control_rate_hz: int = 100
    kp: float = 3.0
    kd: float = 0.12
    min_position_rad: float = 0.0
    max_position_rad: float = 1.6
    max_step_rad: float = 0.1
    slew_rate_rad_s: float = 0.2
    position_tolerance_rad: float = 0.08
    velocity_tolerance_rad_s: float = 0.3
    stable_seconds: float = 0.1
    hold_position_tolerance_rad: float = 0.15
    hold_error_seconds: float = 0.5
    feedback_timeout_seconds: float = 0.5
    startup_timeout_seconds: float = 3.0
    motion_timeout_seconds: float = 15.0

    def validate(self):
        self.left.validate()
        self.right.validate()
        if self.left.interface == self.right.interface:
            raise ValueError("左右夹爪必须使用独立 CAN 接口")
        if type(self.control_rate_hz) is not int or not 1 <= self.control_rate_hz <= 1000:
            raise ValueError("control_rate_hz 必须为 1..1000 的整数")
        for name in ("max_step_rad", "slew_rate_rad_s", "position_tolerance_rad",
                     "velocity_tolerance_rad_s", "stable_seconds", "hold_position_tolerance_rad",
                     "hold_error_seconds", "feedback_timeout_seconds",
                     "startup_timeout_seconds", "motion_timeout_seconds"):
            _positive(getattr(self, name), name)
        encode_mit(0, kp=self.kp, kd=self.kd)
        for value in (self.min_position_rad, self.max_position_rad):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("夹爪位置边界必须为有限数值")
        if not -12.5 <= self.min_position_rad < self.max_position_rad <= 12.5:
            raise ValueError("夹爪位置边界必须在 DM4310 的 -12.5..12.5 rad 量程内")
        for side in (self.left, self.right):
            for value in (side.open_position_rad, side.close_position_rad):
                if value is not None and not self.min_position_rad <= value <= self.max_position_rad:
                    raise ValueError("开合目标角度超出配置位置范围")
        if self.max_step_rad > self.max_position_rad - self.min_position_rad:
            raise ValueError("max_step_rad 不能超过配置的位置范围")
        if self.hold_position_tolerance_rad < self.position_tolerance_rad:
            raise ValueError("保持角度容差不能小于开合到位容差")
        if self.motion_timeout_seconds <= self.stable_seconds:
            raise ValueError("夹爪动作超时必须大于稳定时间")
        if self.feedback_timeout_seconds <= 1 / self.control_rate_hz:
            raise ValueError("夹爪反馈超时必须大于控制周期")
        return self

    def action_positions(self, action, side):
        if action not in ("open", "close"):
            raise ValueError("夹爪动作必须为 open 或 close")
        targets = {name: getattr(getattr(self, name), f"{action}_position_rad")
                   for name in selected_sides(side)}
        missing = [name for name, value in targets.items() if value is None]
        if missing:
            raise ValueError(f"尚未设置 {missing} 的 {action}_position_rad；"
                             "请配置实测角度，或用 --position-rad 指定本次绝对目标")
        return targets

    def validate_positions(self, targets, sides):
        if not isinstance(targets, dict) or set(targets) != set(sides):
            raise ValueError("必须为全部所选夹爪分别提供绝对目标角度")
        for name, value in targets.items():
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not self.min_position_rad <= value <= self.max_position_rad):
                raise ValueError(f"{name} 绝对目标角度必须在 "
                                 f"{self.min_position_rad}..{self.max_position_rad} rad 范围内")
        return dict(targets)


def load_gripper_config(path=None):
    config = DMGripperConfig()
    if path is not None:
        with Path(path).open(encoding="utf-8") as stream:
            config = _merge(config, json.load(stream))
    return config.validate()


def selected_sides(side):
    if side == "both":
        return ("left", "right")
    if side in ("left", "right"):
        return (side,)
    raise ValueError(f"未知夹爪: {side}")


class SocketCAN:
    """One nonblocking classic CAN socket per physical end channel."""

    def __init__(self, config):
        self.config = config
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.ownership = None
        try:
            # Exact standard data-frame filters include flag bits in the mask.
            filters = b"".join(struct.pack("=II", value, 0xE00007FF)
                               for value in sorted({0, config.slave_id, config.master_id}))
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            self.socket.bind((config.interface,))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise

    def claim(self):
        # Prevent two instances of this tool from commanding the same motor.
        # ApexTool/cansend do not take this lock and must be stopped separately.
        if self.ownership is not None:
            return
        lock_path = Path("/tmp") / f"skye-dm-{os.getuid()}-{self.config.interface}-{self.config.slave_id}.lock"
        ownership = lock_path.open("a")
        try:
            fcntl.flock(ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            ownership.close()
            raise MotionError(f"夹爪 {self.config.interface} 已被另一检测进程占用")
        self.ownership = ownership

    def send(self, payload):
        frame = pack_frame(self.config.slave_id, payload)
        if self.socket.send(frame) != len(frame):
            raise MotionError("SocketCAN 未发送完整帧")

    def receive(self):
        try:
            return self.socket.recv(16)
        except (BlockingIOError, InterruptedError):
            return None

    def close(self):
        self.socket.close()
        if self.ownership is not None:
            self.ownership.close()
            self.ownership = None


class DMGripper:
    def __init__(self, config, side="both", *, transport_factory=SocketCAN,
                 clock=time.monotonic):
        self.config = config.validate()
        self.sides = selected_sides(side)
        self.transport_factory = transport_factory
        self.clock = clock
        self.transports = {}
        self.feedback = {}
        self.serials = {name: 0 for name in self.sides}
        self.targets = {}
        self.commands = {}
        self.goal_serials = {}
        self.error = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.enabled_sides = []
        self.send_count = 0
        self.max_command_interval_seconds = 0.0
        self.last_send = None

    def connect(self):
        try:
            for name in self.sides:
                self.transports[name] = self.transport_factory(getattr(self.config, name))
        except Exception as error:
            self.close()
            raise MotionError(f"无法打开夹爪通信链路: {error}") from error

    def _receive(self, *, check_faults=False, require_on=False):
        for name, bus in self.transports.items():
            # Bound each cycle's work so a busy interface cannot starve the other side.
            for _ in range(256):
                frame = bus.receive()
                if frame is None:
                    break
                side = getattr(self.config, name)
                feedback = decode_feedback(frame, slave_id=side.slave_id,
                                           master_id=side.master_id, received_at=self.clock())
                if feedback is not None:
                    self.feedback[name] = feedback
                    self.serials[name] += 1
                    if check_faults and feedback.status not in (0, 1):
                        detail = STATUS_NAMES.get(feedback.status, "未知状态")
                        raise MotionError(f"{name} DM4310 状态 {feedback.status:#x}: {detail}，"
                                          f"原始帧 {feedback.raw_hex}")
                    if require_on and feedback.status != 1:
                        raise MotionError(f"{name} 夹爪离开 On 状态: {feedback.status:#x}，"
                                          f"原始帧 {feedback.raw_hex}")

    def _validate(self, *, require_on, require_fresh=True):
        for name in self.sides:
            feedback = self.feedback.get(name)
            if feedback is None:
                if require_fresh:
                    raise MotionError(f"{name} 尚未收到夹爪反馈")
                continue
            if feedback.status not in (0, 1):
                detail = STATUS_NAMES.get(feedback.status, "未知状态")
                raise MotionError(f"{name} DM4310 状态 {feedback.status:#x}: {detail}，"
                                  f"原始帧 {feedback.raw_hex}")
            if require_on and feedback.status != 1:
                raise MotionError(f"{name} 夹爪未使能，状态={feedback.status}")
            if require_fresh and self.clock() - feedback.received_at > self.config.feedback_timeout_seconds:
                raise MotionError(f"{name} 夹爪反馈超时，停止更新目标")

    def read_status(self, timeout, checkpoint=lambda: None):
        """Passive read: no enable, MIT command, reset or disable frames."""
        deadline = self.clock() + timeout
        while True:
            checkpoint()
            self._receive()
            if all(name in self.feedback for name in self.sides):
                # Passive diagnostics retain Off and fault codes as information.
                if all(self.clock() - self.feedback[name].received_at <= self.config.feedback_timeout_seconds
                       for name in self.sides):
                    return self.snapshot()
            if self.clock() >= deadline:
                missing = [name for name in self.sides if name not in self.feedback]
                raise MotionError(f"未收到新鲜夹爪反馈: {missing or list(self.sides)}；被动读取不会发送控制帧")
            self.stop_event.wait(0.01)

    def enable(self, checkpoint=lambda: None):
        if self.thread is not None:
            raise MotionError("夹爪周期控制已启动")
        if len(self.transports) != len(self.sides):
            raise MotionError("夹爪尚未连接")
        for bus in self.transports.values():
            bus.claim()
        # Ignore any buffered pre-enable feedback for the startup confirmation.
        self._receive(check_faults=True)
        self.feedback.clear()
        for name in self.sides:
            checkpoint()
            self.enabled_sides.append(name)
            self.transports[name].send(ENABLE)
            until = self.clock() + 0.1  # Protocol startup ordering: left, 100 ms, right, 100 ms.
            while self.clock() < until:
                checkpoint()
                self._receive(check_faults=True)
                self._validate(require_on=False, require_fresh=False)
                self.stop_event.wait(0.005)
        deadline = self.clock() + self.config.startup_timeout_seconds
        while True:
            checkpoint()
            self._receive(check_faults=True)
            self._validate(require_on=False, require_fresh=False)
            if all(name in self.feedback and self.feedback[name].status == 1
                   and self.clock() - self.feedback[name].received_at <= self.config.feedback_timeout_seconds
                   for name in self.sides):
                break
            if self.clock() >= deadline:
                raise MotionError("使能后未收到两侧所选夹爪的新鲜 On 反馈，不发送位置指令")
            self.stop_event.wait(0.01)
        # Start at the actual measured position, never at an assumed zero/open position.
        self.targets = {name: self.feedback[name].position_rad for name in self.sides}
        self.commands = dict(self.targets)
        self.last_send = self.clock()
        self.thread = threading.Thread(target=self._run, name="dm4310-can", daemon=True)
        self.thread.start()

    def _cycle(self):
        with self.lock:
            self._receive(check_faults=True, require_on=True)
            self._validate(require_on=True)
            now = self.clock()
            interval = now - self.last_send
            self.max_command_interval_seconds = max(self.max_command_interval_seconds, interval)
            # A delayed scheduler must not turn a rate limit into a large target jump.
            max_delta = self.config.slew_rate_rad_s * min(interval, 1 / self.config.control_rate_hz)
            for name in self.sides:
                delta = self.targets[name] - self.commands[name]
                self.commands[name] += max(-max_delta, min(max_delta, delta))
                self.transports[name].send(encode_mit(self.commands[name], kp=self.config.kp,
                                                     kd=self.config.kd))
                self.send_count += 1
                if abs(self.commands[name] - self.targets[name]) < 1e-9 and name not in self.goal_serials:
                    self.goal_serials[name] = self.serials[name]
            self.last_send = now

    def _run(self):
        while not self.stop_event.is_set():
            try:
                self._cycle()
            except Exception as error:
                with self.lock:
                    self.error = error
                return  # Do not advance targets or send reset/disable frames on a fault.
            self.stop_event.wait(1 / self.config.control_rate_hz)

    def check_health(self):
        with self.lock:
            if self.error is not None:
                raise MotionError(f"夹爪周期控制失败: {self.error}") from self.error
            self._validate(require_on=True)

    def step(self, delta_rad, checkpoint=lambda: None):
        if (type(delta_rad) not in (float, int) or not math.isfinite(delta_rad)
                or not 0 < abs(delta_rad) <= self.config.max_step_rad):
            raise ValueError(f"相对移动量必须满足 0 < |delta_rad| <= {self.config.max_step_rad}")
        return self._step({name: delta_rad for name in self.sides}, checkpoint)

    def _step(self, deltas, checkpoint):
        checkpoint()
        if self.thread is None:
            raise MotionError("夹爪周期控制未启动")
        self.check_health()
        with self.lock:
            targets = {name: self.feedback[name].position_rad + deltas[name] for name in self.sides}
        return self.move_to(targets, checkpoint)

    def move_to(self, targets, checkpoint=lambda: None, *, action=None):
        """Move to fixed motor angles, independent of previous command count."""
        targets = self.config.validate_positions(targets, self.sides)
        if action is not None and action not in ("open", "close"):
            raise ValueError("夹爪动作必须为 open 或 close")
        checkpoint()
        if self.thread is None:
            raise MotionError("夹爪周期控制未启动")
        self.check_health()
        with self.lock:
            if action is not None:
                for name in self.sides:
                    change = targets[name] - self.feedback[name].position_rad
                    sign = getattr(self.config, name).close_direction * (1 if action == "close" else -1)
                    if change * sign < -self.config.position_tolerance_rad:
                        raise MotionError(f"{name} 的 {action} 目标 {targets[name]:.6f} rad 与动作方向相反；"
                                          f"当前反馈 {self.feedback[name].position_rad:.6f} rad")
            self.targets = targets  # Validate both sides before changing either command.
            self.goal_serials.clear()
        deadline = self.clock() + self.config.motion_timeout_seconds
        stable_since = None
        previous_serials = None
        while True:
            checkpoint()
            self.check_health()
            now = self.clock()
            with self.lock:
                serials = tuple(self.serials[name] for name in self.sides)
                complete = all(
                    name in self.goal_serials and self.serials[name] > self.goal_serials[name]
                    and abs(self.feedback[name].position_rad - targets[name]) <= self.config.position_tolerance_rad
                    and abs(self.feedback[name].velocity_rad_s) <= self.config.velocity_tolerance_rad_s
                    for name in self.sides)
            if complete:
                if stable_since is None:
                    stable_since = now
                    previous_serials = serials
                if (now - stable_since >= self.config.stable_seconds
                        and all(a != b for a, b in zip(serials, previous_serials))):
                    return self.snapshot()
            else:
                stable_since = None
            if now >= deadline:
                with self.lock:
                    detail = "; ".join(
                        f"{name} 目标 {targets[name]:.6f} rad，实际 {self.feedback[name].position_rad:.6f} rad"
                        for name in self.sides)
                raise MotionError(f"夹爪运动未持续到位，执行超时；{detail}；"
                                  f"允许误差 {self.config.position_tolerance_rad:.3f} rad；不推断已夹到物体")
            self.stop_event.wait(0.01)

    def snapshot(self):
        with self.lock:
            return {"feedback": {name: {**value.to_dict(),
                                         "age_seconds": self.clock() - value.received_at,
                                         "status_name": STATUS_NAMES.get(value.status, "未知状态")}
                                 for name, value in self.feedback.items()},
                    "targets_rad": dict(self.targets), "commands_rad": dict(self.commands),
                    "mit_frames_sent": self.send_count,
                    "max_command_interval_seconds": self.max_command_interval_seconds}

    def freeze(self):
        """Stop increasing setpoints without releasing a potentially held load."""
        with self.lock:
            self.targets = dict(self.commands)
            self.goal_serials.clear()

    def disable(self):
        """Explicit operator action. Attempt both sides even when one send fails."""
        self._stop_worker()
        try:
            self._receive()
        except Exception:
            # Receiving a malformed/faulty frame must not prevent explicit disable sends.
            pass
        before_serials = dict(self.serials)
        errors = []
        for name, bus in self.transports.items():
            try:
                bus.claim()
                bus.send(DISABLE)
            except Exception as error:
                errors.append(f"{name}: {error}")
        if errors:
            raise MotionError("失能发送失败: " + "; ".join(errors))
        deadline = self.clock() + self.config.startup_timeout_seconds
        while self.clock() < deadline:
            self._receive()
            if all(name in self.feedback and self.serials[name] > before_serials[name]
                   and self.feedback[name].status == 0
                   and self.clock() - self.feedback[name].received_at <= self.config.feedback_timeout_seconds
                   for name in self.sides):
                return
            time.sleep(0.01)
        raise MotionError("失能帧已发送，但未收到全部所选夹爪的新鲜 Off 反馈；无法确认物理失能")

    def _stop_worker(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise MotionError("夹爪控制线程未停止，不能确认清理完成")

    def close(self):
        """Close sockets only; consistent with Apex disable_on_shutdown=false."""
        # Do not close a transport that a stuck worker could still be using.
        self._stop_worker()
        for bus in self.transports.values():
            bus.close()
        self.transports.clear()

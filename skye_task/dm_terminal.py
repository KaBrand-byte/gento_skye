"""Gento L1 Terminal CAN transport, using the vendor's 4-byte ID envelope.

Source: Gento-Teleoperation-Apex/gento_ros2_packages,
marvin_ros2_control/src/marvin_robot.cpp, sdk_{read,write}_can{A,B}.
This standalone connection never changes arm modes or sends arm commands.
"""

from collections import deque
from ctypes import byref, c_int32, c_ubyte, c_uint32
import fcntl
import os
from pathlib import Path
import struct
import threading
import time

from .dm_protocol import CAN_FRAME, pack_frame
from .sdk import GentoSDK, MotionError


def pack_terminal(can_id, payload):
    pack_frame(can_id, payload)  # Apply the same standard-ID / DLC validation.
    return struct.pack("<I", can_id) + payload


def unpack_terminal(payload):
    if len(payload) != 12:
        raise MotionError(f"DM4310 Terminal CAN 报文长度异常: {len(payload)}，要求 12；"
                          f"原始数据 {payload.hex()}")
    can_id, = struct.unpack("<I", payload[:4])
    return CAN_FRAME.pack(can_id, 8, payload[4:])


class GentoTerminalConnection:
    """One SDK connection, serialized across both end channels."""

    def __init__(self, robot_config, gripper_config, *, sdk_factory=GentoSDK, sdk=None):
        self.robot_config = robot_config
        self.routes = {gripper_config.left: 0, gripper_config.right: 1}
        self.sdk_factory = sdk_factory
        self.sdk = sdk
        self.owns_sdk = sdk is None
        self.ownership = None
        self.lock = threading.RLock()
        self.connection_info = {}
        self.raw_received = deque(maxlen=32)
        self.frames_sent = 0

    def reserve(self):
        # This lock covers all end channels, including passive readers, because the
        # controller allows one SDK owner and readers consume a shared receive queue.
        if self.ownership is not None:
            return
        path = Path('/tmp') / f"skye-dm-sdk-{os.getuid()}-{self.robot_config.controller_ip}.lock"
        ownership = path.open('a')
        try:
            fcntl.flock(ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception as error:
            ownership.close()
            raise MotionError("另一夹爪检测进程正在使用该控制器 SDK") from error
        self.ownership = ownership

    def connect(self):
        self.reserve()
        try:
            if self.owns_sdk:
                self.sdk = self.sdk_factory(self.robot_config)
                self.connection_info = self.sdk.connect()
            elif self.sdk is None or not self.sdk.connected:
                raise MotionError("共享的双臂 SDK 尚未连接")
            else:
                self.connection_info = {"shared_with_arm_sequence": True}
            # L1 logs every empty nonblocking receive as a timeout. Keep these
            # expected polls quiet; actual negative return codes are checked below.
            self.sdk.robot.set_log_level(0)
        except Exception:
            self.close()
            raise

    def transport(self, config):
        if self.sdk is None:
            raise MotionError("Gento SDK 尚未连接")
        return GentoTerminalCAN(self, config, self.routes[config])

    def snapshot(self):
        with self.lock:
            return {"connection": dict(self.connection_info),
                    "controller_ip": self.robot_config.controller_ip,
                    "channel": 1, "envelope": "uint32_le_can_id_plus_8_data_bytes",
                    "terminal_frames_sent": self.frames_sent,
                    "recent_terminal_frames": list(self.raw_received)}

    def close(self):
        with self.lock:
            try:
                if self.sdk is not None:
                    sdk, self.sdk = self.sdk, None
                    if self.owns_sdk:
                        sdk.close()
            finally:
                if self.ownership is not None:
                    self.ownership.close()
                    self.ownership = None


class GentoTerminalCAN:
    def __init__(self, connection, config, obj_type):
        self.connection = connection
        self.config = config
        self.obj_type = obj_type
        self.closed = False

    def claim(self):
        if self.closed or self.connection.sdk is None:
            raise MotionError("末端 CAN 连接已关闭")

    def send(self, payload):
        raw = pack_terminal(self.config.slave_id, payload)
        with self.connection.lock:
            self.claim()
            # timeout=0 prevents a failed transfer from stalling the control loop.
            result, _ = self.connection.sdk.robot.terminal_set(self.obj_type, 1, raw, timeout=0)
            if result != 0:
                raise MotionError(f"arm{self.obj_type} Terminal CAN 发送失败，SDK 返回 {result}")
            self.connection.frames_sent += 1

    def receive(self):
        with self.connection.lock:
            self.claim()
            channel, elapsed = c_int32(), c_uint32()
            buffer = (c_ubyte * 64)()
            # The Python terminal_get wrapper collapses errors into empty data.
            # Preserve the native error; only -10 is an expected empty poll.
            count = self.connection.sdk.robot.dll.FX_L1_Terminal_GetData(
                self.obj_type, 0, byref(channel), buffer, byref(elapsed))
            if count in (0, -10):
                return None
            if not 0 < count <= 64:
                raise MotionError(f"arm{self.obj_type} Terminal CAN 接收失败，SDK 返回 {count}")
            raw = bytes(buffer[:count])
            self.connection.raw_received.append({"arm": self.obj_type, "channel": channel.value,
                                                 "received_at": time.monotonic(), "raw_hex": raw.hex()})
            if channel.value != 1:
                raise MotionError(f"arm{self.obj_type} 收到其他末端通道 {channel.value}，"
                                  f"要求 CANFD=1；原始数据 {raw.hex()}")
            return unpack_terminal(raw)

    def close(self):
        # The CLI owns the shared SDK connection and releases it after both sides.
        self.closed = True

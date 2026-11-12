"""DM4310 MIT protocol from the Apex DM CAN document (2026-08-11 baseline)."""

from dataclasses import asdict, dataclass
import math
import struct

from .sdk import MotionError


CAN_FRAME = struct.Struct("<IB3x8s")
ENABLE = bytes.fromhex("FF FF FF FF FF FF FF FC")
DISABLE = bytes.fromhex("FF FF FF FF FF FF FF FD")
# No zeroing, parameter writes or motor-mode changes are implemented.


def _uint(value, low, high, bits):
    if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"DM4310 参数 {value!r} 超出 [{low}, {high}]")
    return int((value - low) / (high - low) * ((1 << bits) - 1))


def _float(value, low, high, bits):
    return value / ((1 << bits) - 1) * (high - low) + low


def encode_mit(position_rad, *, velocity_rad_s=0, kp=3.0, kd=0.12, torque_nm=0):
    q = _uint(position_rad, -12.5, 12.5, 16)
    dq = _uint(velocity_rad_s, -30, 30, 12)
    p = _uint(kp, 0, 500, 12)
    d = _uint(kd, 0, 5, 12)
    tau = _uint(torque_nm, -10, 10, 12)
    return bytes((q >> 8, q & 255, dq >> 4, (dq & 15) << 4 | p >> 8,
                  p & 255, d >> 4, (d & 15) << 4 | tau >> 8, tau & 255))


def pack_frame(can_id, payload):
    if type(can_id) is not int or not 0 <= can_id <= 0x7FF or len(payload) != 8:
        raise ValueError("DM4310 要求 11 bit 标准 CAN ID 和 8 字节数据")
    return CAN_FRAME.pack(can_id, 8, payload)


@dataclass(frozen=True)
class MotorFeedback:
    can_id: int
    motor_id: int
    status: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    mos_temperature_c: int
    motor_temperature_c: int
    received_at: float
    raw_hex: str

    def to_dict(self):
        return asdict(self)


def decode_feedback(frame, *, slave_id, master_id, received_at):
    if len(frame) != CAN_FRAME.size:
        raise MotionError(f"SocketCAN 帧长度异常: {len(frame)}，要求 16")
    can_id, dlc, data = CAN_FRAME.unpack(frame)
    # Reject EFF, RTR and ERR frames. The document specifies standard data frames.
    if can_id & ~0x7FF or can_id not in (0, slave_id, master_id):
        return None
    if dlc != 8:
        raise MotionError(f"DM4310 反馈 DLC={dlc}，要求 8，原始帧 {frame.hex()}")
    motor_id = data[0] & 15
    # Match the embedded ID even when the CAN ID is the master or zero. This also
    # excludes locally echoed enable/disable and the documented MIT command frames.
    if motor_id != slave_id:
        return None
    return MotorFeedback(
        can_id, motor_id, data[0] >> 4,
        _float(data[1] << 8 | data[2], -12.5, 12.5, 16),
        _float(data[3] << 4 | data[4] >> 4, -30, 30, 12),
        _float((data[4] & 15) << 8 | data[5], -10, 10, 12),
        data[6], data[7], received_at, frame.hex(),
    )


STATUS_NAMES = {0: "Off", 1: "On", 8: "超压", 9: "欠压", 10: "过电流",
                11: "MOS 过温", 12: "电机过温", 13: "通讯丢失", 14: "过载"}

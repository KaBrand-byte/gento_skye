"""Dependency-free observation/action logic; not a physical safety controller."""

from dataclasses import dataclass
import math
from numbers import Real


STATE_DIM = 16
MODEL_DIM = 32
JOINT_INDICES = (*range(7), *range(8, 15))
GRIPPER_INDICES = (7, 15)
IMAGE_KEYS = ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb")


class ContractError(ValueError):
    pass


def vector(values, size, label):
    if len(values) != size:
        raise ContractError(f"{label}: expected {size} values, got {len(values)}")
    if any(isinstance(x, bool) or not isinstance(x, Real) or not math.isfinite(x) for x in values):
        raise ContractError(f"{label}: values must be finite numbers")
    return tuple(float(x) for x in values)


@dataclass(frozen=True)
class RGBFrame:
    source: str
    stamp_ns: int
    rgb: bytes
    width: int = 224
    height: int = 224

    def validate(self):
        if (self.width, self.height) != (224, 224):
            raise ContractError("frame must already be resized to 224x224")
        if not isinstance(self.rgb, bytes) or len(self.rgb) != self.width * self.height * 3:
            raise ContractError("frame must contain packed HWC RGB uint8 bytes")


@dataclass(frozen=True)
class Observation:
    stamp_ns: int
    left_rad: tuple
    left_gripper_rad: float
    right_rad: tuple
    right_gripper_rad: float
    frames: tuple[RGBFrame, ...]
    prompt: str

    def physical_state(self):
        left = vector(self.left_rad, 7, "left joints")
        right = vector(self.right_rad, 7, "right joints")
        return vector((*left, self.left_gripper_rad, *right, self.right_gripper_rad), STATE_DIM, "state")


@dataclass(frozen=True)
class GripperCalibration:
    open_rad: float = 0.0
    closed_rad: float = 0.94

    def __post_init__(self):
        vector((self.open_rad, self.closed_rad), 2, "gripper calibration")
        if self.closed_rad == self.open_rad:
            raise ContractError("gripper calibration has zero range")

    def normalize(self, angle):
        value = (angle - self.open_rad) / (self.closed_rad - self.open_rad)
        self._range(value)
        return value

    def denormalize(self, value):
        self._range(value)
        return self.open_rad + value * (self.closed_rad - self.open_rad)

    @staticmethod
    def _range(value):
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ContractError("gripper value outside calibrated range; refusing to clip")


@dataclass(frozen=True)
class Adapter:
    # Explicit source identities, in head/left-wrist/right-wrist order; not ROS topic guesses.
    camera_sources: tuple[str, str, str]
    joint_unit: str = "rad"
    arm_action_mode: str = "absolute"
    left_gripper: GripperCalibration = GripperCalibration()
    right_gripper: GripperCalibration = GripperCalibration()
    max_age_ns: int = 200_000_000
    max_skew_ns: int = 50_000_000

    def __post_init__(self):
        if len(self.camera_sources) != 3 or len(set(self.camera_sources)) != 3:
            raise ContractError("three distinct camera sources are required")
        if any(not isinstance(x, str) or not x.strip() for x in self.camera_sources):
            raise ContractError("camera source identities must be nonempty strings")
        if self.joint_unit not in ("rad", "deg"):
            raise ContractError("joint_unit must be rad or deg")
        if self.arm_action_mode not in ("absolute", "delta"):
            raise ContractError("arm_action_mode must be absolute or delta")
        if self.max_age_ns <= 0 or self.max_skew_ns < 0:
            raise ContractError("invalid freshness thresholds")

    def inputs(self, observation, *, now_ns):
        state = list(observation.physical_state())
        if not isinstance(observation.prompt, str) or not observation.prompt.strip():
            raise ContractError("a nonempty task prompt is required")
        frames = {frame.source: frame for frame in observation.frames}
        if len(frames) != len(observation.frames):
            raise ContractError("duplicate frame source")
        images = {}
        stamps = [observation.stamp_ns]
        for key, source in zip(IMAGE_KEYS, self.camera_sources):
            if source not in frames:
                raise ContractError(f"missing camera: {source}")
            frame = frames[source]
            frame.validate()
            images[key] = frame
            stamps.append(frame.stamp_ns)
        if any(isinstance(t, bool) or not isinstance(t, int) or t < 0 for t in (*stamps, now_ns)):
            raise ContractError("timestamps must be nonnegative integer session-monotonic nanoseconds")
        if any(t > now_ns for t in stamps):
            raise ContractError("future sensor timestamp or mismatched clocks")
        if any(now_ns - t > self.max_age_ns for t in stamps):
            raise ContractError("stale observation")
        if max(stamps) - min(stamps) > self.max_skew_ns:
            raise ContractError("camera/state timestamp skew exceeds limit")
        if self.joint_unit == "deg":
            for index in JOINT_INDICES:
                state[index] = math.degrees(state[index])
        for index, calibration in zip(GRIPPER_INDICES, (self.left_gripper, self.right_gripper)):
            state[index] = calibration.normalize(state[index])
        return {
            "state": tuple(state) + (0.0,) * (MODEL_DIM - STATE_DIM),
            "image": images,
            "image_mask": dict.fromkeys(IMAGE_KEYS, True),
            "prompt": observation.prompt,
        }

    def actions(self, raw_chunk, observation, *, now_ns):
        # Recheck age AFTER inference too. This method returns unnormalized physical targets.
        anchor = self.inputs(observation, now_ns=now_ns)["state"]
        if not len(raw_chunk):
            raise ContractError("empty action chunk")
        targets = []
        for row in raw_chunk:
            action = list(vector(row, MODEL_DIM, "model action")[:STATE_DIM])
            for index in JOINT_INDICES:
                # Every delta is relative to the observation, never the preceding prediction.
                if self.arm_action_mode == "delta":
                    action[index] += anchor[index]
                if self.joint_unit == "deg":
                    action[index] = math.radians(action[index])
            for index, calibration in zip(GRIPPER_INDICES, (self.left_gripper, self.right_gripper)):
                action[index] = calibration.denormalize(action[index])
            targets.append(tuple(action))
        return tuple(targets)


@dataclass(frozen=True)
class NumericalLimits:
    source: str
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple

    def __post_init__(self):
        for name in ("lower", "upper", "velocity", "acceleration"):
            vector(getattr(self, name), STATE_DIM, name)
        if not self.source or any(lo >= hi for lo, hi in zip(self.lower, self.upper)):
            raise ContractError("invalid limit source or position interval")
        if any(x <= 0 for x in (*self.velocity, *self.acceleration)):
            raise ContractError("velocity and acceleration limits must be positive")

    def check(self, targets, initial_state, initial_velocity, period_seconds):
        """Check the entire chunk before returning any prefix. No collision claim."""
        previous = vector(initial_state, STATE_DIM, "initial state")
        velocity = vector(initial_velocity, STATE_DIM, "initial velocity")
        if not math.isfinite(period_seconds) or period_seconds <= 0:
            raise ContractError("invalid control period")
        self._position(previous)
        if any(abs(v) > limit for v, limit in zip(velocity, self.velocity)):
            raise ContractError("initial velocity exceeds limit")
        for step, values in enumerate(targets):
            target = vector(values, STATE_DIM, f"target {step}")
            self._position(target)
            next_velocity = tuple((q - p) / period_seconds for q, p in zip(target, previous))
            if any(abs(v) > limit + 1e-9 for v, limit in zip(next_velocity, self.velocity)):
                raise ContractError(f"step {step}: velocity limit")
            if any(abs(v - old) / period_seconds > limit + 1e-9
                   for v, old, limit in zip(next_velocity, velocity, self.acceleration)):
                raise ContractError(f"step {step}: acceleration limit")
            previous, velocity = target, next_velocity

    def _position(self, state):
        if any(q < lo or q > hi for q, lo, hi in zip(state, self.lower, self.upper)):
            raise ContractError("position limit")


def plan_prefix(adapter, limits, observation, raw_chunk, *, now_ns, initial_velocity, hz, prefix_steps):
    if not math.isfinite(hz) or hz <= 0:
        raise ContractError("hz must be positive")
    period_ns = round(1_000_000_000 / hz)
    if period_ns <= 0:
        raise ContractError("control period rounds to zero")
    targets = adapter.actions(raw_chunk, observation, now_ns=now_ns)
    if isinstance(prefix_steps, bool) or not isinstance(prefix_steps, int) or not 1 <= prefix_steps <= len(targets):
        raise ContractError("prefix_steps must be within the chunk")
    limits.check(targets, observation.physical_state(), initial_velocity, period_ns / 1e9)
    return {
        "targets_rad": [list(row) for row in targets[:prefix_steps]],
        "due_ns": [now_ns + period_ns * (i + 1) for i in range(prefix_steps)],
        "chunk_steps": len(targets),
        "discarded_tail_steps": len(targets) - prefix_steps,
        "numerical_checks_passed": True,
        "collision_checked": False,
        "execution_supported": False,
    }

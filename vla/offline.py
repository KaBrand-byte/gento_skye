"""Synthetic policy, lossless local trace, and deterministic action replay."""

import base64
from dataclasses import asdict
import json
import math
from pathlib import Path
import zlib

from .core import Adapter, ContractError, GripperCalibration, JOINT_INDICES
from .core import NumericalLimits, Observation, RGBFrame, plan_prefix


def observation_record(observation):
    record = asdict(observation)
    for frame in record["frames"]:
        frame["rgb_zlib_base64"] = base64.b64encode(zlib.compress(frame.pop("rgb"))).decode("ascii")
    return record


def read_observation(record):
    fields = dict(record)
    frames = []
    for saved in fields.pop("frames"):
        frame = dict(saved)
        frame["rgb"] = zlib.decompress(base64.b64decode(frame.pop("rgb_zlib_base64")))
        frames.append(RGBFrame(**frame))
    return Observation(**fields, frames=tuple(frames))


def synthetic_frames(stamp_ns, sources):
    # Distinct pixel values make accidental swaps visible and testable.
    colors = ((220, 30, 10), (10, 210, 30), (30, 10, 200))
    return tuple(RGBFrame(source, stamp_ns - 5_000_000, bytes(color) * (224 * 224))
                 for source, color in zip(sources, colors))


def stub_chunk(adapter, observation, now_ns):
    """A scripted fixture, NOT a VLA or simulator: only two joints change."""
    anchor = adapter.inputs(observation, now_ns=now_ns)["state"]
    chunk = []
    for step in range(6):
        row = list(anchor)
        if adapter.arm_action_mode == "delta":
            for index in JOINT_INDICES:
                row[index] = 0.0
        amount = 0.0005 * (step + 1) ** 2
        if adapter.joint_unit == "deg":
            amount = math.degrees(amount)
        row[0] += amount
        row[8] -= amount
        chunk.append(row)
    return chunk


def make_demo(*, joint_unit="rad", arm_action_mode="absolute"):
    adapter = Adapter(("head_monocular", "wrist_left", "wrist_right"), joint_unit, arm_action_mode)
    lower, upper = [-2.0] * 16, [2.0] * 16
    for index in (7, 15):
        lower[index], upper[index] = 0.0, 0.94
    limits = NumericalLimits("synthetic_demo_NOT_robot_limits", tuple(lower), tuple(upper), (0.5,) * 16, (2.0,) * 16)
    trace = {
        "schema_version": 1,
        "clock": "session_monotonic_ns",
        "policy": "synthetic_stub_NOT_pi05",
        "execution_supported": False,
        "adapter": asdict(adapter),
        "limits": asdict(limits),
        "hz": 20.0,
        "prefix_steps": 3,
        "samples": [],
    }
    left = tuple(0.01 * i for i in range(7))
    right = tuple(-0.02 * i for i in range(7))
    velocity = (0.0,) * 16
    for cycle in range(3):
        stamp_ns = 1_000_000_000 + cycle * 170_000_000
        observation = Observation(stamp_ns, left, 0.235, right, 0.705,
                                  synthetic_frames(stamp_ns, adapter.camera_sources), "offline data-contract check")
        received_ns = stamp_ns + 20_000_000
        raw_chunk = stub_chunk(adapter, observation, stamp_ns)
        plan = plan_prefix(adapter, limits, observation, raw_chunk, now_ns=received_ns,
                           initial_velocity=velocity, hz=trace["hz"], prefix_steps=trace["prefix_steps"])
        trace["samples"].append({
            "observation": observation_record(observation),
            "received_ns": received_ns,
            "initial_velocity": list(velocity),
            "raw_chunk": raw_chunk,
            "expected_plan": plan,
        })
        # Perfect tracking is only a fixture to connect cycles; it is not robot feedback.
        last, previous = plan["targets_rad"][-1], plan["targets_rad"][-2]
        left, right = tuple(last[:7]), tuple(last[8:15])
        velocity = tuple((q - p) * trace["hz"] for q, p in zip(last, previous))
    return trace


def replay(trace):
    if trace.get("schema_version") != 1 or trace.get("execution_supported") is not False:
        raise ContractError("unsupported trace schema or execution flag")
    if trace.get("clock") != "session_monotonic_ns":
        raise ContractError("unsupported trace clock")
    fields = dict(trace["adapter"])
    fields["camera_sources"] = tuple(fields["camera_sources"])
    for name in ("left_gripper", "right_gripper"):
        fields[name] = GripperCalibration(**fields[name])
    adapter = Adapter(**fields)
    limits = NumericalLimits(**trace["limits"])
    if not trace["samples"]:
        raise ContractError("trace contains no samples")
    count = 0
    previous_due_ns = -1
    for sample in trace["samples"]:
        observation = read_observation(sample["observation"])
        if observation.stamp_ns < previous_due_ns:
            raise ContractError("next observation predates previous planned prefix")
        plan = plan_prefix(adapter, limits, observation, sample["raw_chunk"], now_ns=sample["received_ns"],
                           initial_velocity=sample["initial_velocity"], hz=trace["hz"],
                           prefix_steps=trace["prefix_steps"])
        if plan != sample["expected_plan"]:
            raise ContractError("replay differs from the recorded plan")
        previous_due_ns = plan["due_ns"][-1]
        count += len(plan["targets_rad"])
    return {
        "samples": len(trace["samples"]),
        "planned_targets": count,
        "policy": trace["policy"],
        "numerical_replay_passed": True,
        "collision_checked": False,
        "execution_supported": False,
    }


def save_trace(trace, path):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(trace, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def load_trace(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))

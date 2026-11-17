import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from vla.core import Adapter, ContractError, GripperCalibration, IMAGE_KEYS, MODEL_DIM
from vla.core import NumericalLimits, Observation, RGBFrame, plan_prefix
from vla.offline import load_trace, make_demo, observation_record, read_observation, replay, save_trace
from vla.offline import synthetic_frames


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.adapter = Adapter(("head_monocular", "wrist_left", "wrist_right"))
        self.observation = Observation(
            self.now, (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6), 0.235,
            (-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7), 0.705,
            synthetic_frames(self.now, self.adapter.camera_sources), "pick up the object",
        )
        lower, upper = [-2.0] * 16, [2.0] * 16
        lower[7] = lower[15] = 0.0
        upper[7] = upper[15] = 0.94
        self.limits = NumericalLimits("test_fixture", tuple(lower), tuple(upper), (0.5,) * 16, (2.0,) * 16)

    def row(self, adapter=None):
        return list((adapter or self.adapter).inputs(self.observation, now_ns=self.now)["state"])

    def plan(self, rows, **overrides):
        options = dict(now_ns=self.now, initial_velocity=[0.0] * 16, hz=20.0, prefix_steps=1)
        options.update(overrides)
        return plan_prefix(self.adapter, self.limits, self.observation, rows, **options)

    def test_arm_and_gripper_order_and_model_padding(self):
        state = self.row()
        self.assertEqual(state[:7], list(self.observation.left_rad))
        self.assertAlmostEqual(state[7], 0.25)
        self.assertEqual(state[8:15], list(self.observation.right_rad))
        self.assertAlmostEqual(state[15], 0.75)
        self.assertEqual(state[16:], [0.0] * 16)

    def test_explicit_image_sources_ignore_arrival_order_and_extra_depth_view(self):
        extra = RGBFrame("head_depth_rgb", self.now, bytes((1, 1, 1)) * (224 * 224))
        observation = replace(self.observation, frames=(*reversed(self.observation.frames), extra))
        packet = self.adapter.inputs(observation, now_ns=self.now)
        self.assertEqual([packet["image"][key].rgb[:3] for key in IMAGE_KEYS],
                         [bytes((220, 30, 10)), bytes((10, 210, 30)), bytes((30, 10, 200))])
        self.assertTrue(all(packet["image_mask"].values()))

    def test_degree_conversion_keeps_grippers_normalized(self):
        adapter = replace(self.adapter, joint_unit="deg")
        row = self.row(adapter)
        self.assertAlmostEqual(row[6], math.degrees(0.6))
        self.assertAlmostEqual(row[14], math.degrees(-0.7))
        self.assertAlmostEqual(row[7], 0.25)
        physical, = adapter.actions([row], self.observation, now_ns=self.now)
        for actual, expected in zip(physical, self.observation.physical_state()):
            self.assertAlmostEqual(actual, expected)

    def test_deltas_are_anchored_to_observation_not_accumulated(self):
        adapter = replace(self.adapter, arm_action_mode="delta")
        first, second = [0.0] * MODEL_DIM, [0.0] * MODEL_DIM
        first[1], second[1] = 0.01, 0.02
        first[7], second[7] = 0.25, 0.5
        first[15] = second[15] = 0.75
        result = adapter.actions([first, second], self.observation, now_ns=self.now)
        self.assertAlmostEqual(result[0][1], 0.11)
        self.assertAlmostEqual(result[1][1], 0.12)
        self.assertAlmostEqual(result[1][7], 0.47)  # Grippers are absolute, never deltas.

    def test_degree_deltas_and_unused_model_dimensions(self):
        adapter = replace(self.adapter, joint_unit="deg", arm_action_mode="delta")
        row = [0.0] * MODEL_DIM
        row[1], row[7], row[15], row[31] = 5.0, 0.25, 0.75, 99.0
        result, = adapter.actions([row], self.observation, now_ns=self.now)
        self.assertEqual(len(result), 16)
        self.assertAlmostEqual(result[1], 0.1 + math.radians(5.0))

    def test_gripper_endpoints_and_asymmetric_calibrations(self):
        adapter = replace(self.adapter, right_gripper=GripperCalibration(0.1, 1.1))
        row = self.row(adapter)
        row[7], row[15] = 0.0, 1.0
        state, = adapter.actions([row], self.observation, now_ns=self.now)
        self.assertEqual(state[7], 0.0)
        self.assertEqual(state[15], 1.1)

    def test_bad_state_shape_and_nonfinite_values(self):
        for observation in (
            replace(self.observation, left_rad=(0.0,) * 6),
            replace(self.observation, right_rad=(float("nan"),) * 7),
            replace(self.observation, left_gripper_rad=True),
        ):
            with self.subTest(observation=observation), self.assertRaises(ContractError):
                self.adapter.inputs(observation, now_ns=self.now)

    def test_missing_duplicate_wrong_size_or_malformed_image_rejected(self):
        frames = self.observation.frames
        for bad_frames in (frames[:2], (*frames, frames[0]),
                           (replace(frames[0], width=223), *frames[1:]),
                           (replace(frames[0], rgb=b"bad"), *frames[1:])):
            with self.subTest(frames=len(bad_frames)), self.assertRaises(ContractError):
                self.adapter.inputs(replace(self.observation, frames=bad_frames), now_ns=self.now)

    def test_stale_skew_future_and_wrong_clock_rejected(self):
        frame = self.observation.frames[0]
        for stamp in (self.now - 250_000_000, self.now - 60_000_000, self.now + 1, 1.0):
            observation = replace(self.observation, frames=(replace(frame, stamp_ns=stamp), *self.observation.frames[1:]))
            with self.subTest(stamp=stamp), self.assertRaises(ContractError):
                self.adapter.inputs(observation, now_ns=self.now)

    def test_observation_expiring_during_inference_is_rejected(self):
        row = self.row()
        with self.assertRaisesRegex(ContractError, "stale"):
            self.adapter.actions([row], self.observation, now_ns=self.now + 210_000_000)

    def test_gripper_out_of_range_is_rejected_without_clipping(self):
        for grip in (-0.01, 1.01):
            row = self.row()
            row[7] = grip
            with self.assertRaisesRegex(ContractError, "gripper"):
                self.plan([row])

    def test_empty_wrong_dim_and_nonfinite_action_rejected(self):
        row = self.row()
        nonfinite = row.copy()
        nonfinite[3] = float("inf")
        for chunk in ([], [row[:16]], [nonfinite]):
            with self.subTest(length=len(chunk)), self.assertRaises(ContractError):
                self.plan(chunk)

    def test_position_velocity_and_acceleration_rejected(self):
        for value, reason in ((3.0, "position"), (0.03, "velocity"), (0.01, "acceleration")):
            row = self.row()
            row[0] = value
            with self.subTest(reason=reason), self.assertRaisesRegex(ContractError, reason):
                self.plan([row])

    def test_initial_position_and_velocity_are_checked(self):
        bad = replace(self.observation, left_rad=(3.0,) * 7)
        with self.assertRaisesRegex(ContractError, "position"):
            plan_prefix(self.adapter, self.limits, bad, [self.row()], now_ns=self.now,
                        initial_velocity=[0.0] * 16, hz=20.0, prefix_steps=1)
        with self.assertRaisesRegex(ContractError, "initial velocity"):
            self.plan([self.row()], initial_velocity=[1.0] * 16)

    def test_invalid_tail_rejects_entire_chunk_before_prefix(self):
        first, bad_tail = self.row(), self.row()
        bad_tail[8] = 3.0
        with self.assertRaisesRegex(ContractError, "position"):
            self.plan([first, bad_tail], prefix_steps=1)

    def test_prefix_cadence_and_no_execution_claim(self):
        rows = [self.row() for _ in range(6)]
        plan = self.plan(rows, prefix_steps=3)
        self.assertEqual(plan["due_ns"], [self.now + 50_000_000, self.now + 100_000_000, self.now + 150_000_000])
        self.assertEqual(plan["discarded_tail_steps"], 3)
        self.assertFalse(plan["execution_supported"])
        self.assertFalse(plan["collision_checked"])

    def test_invalid_prefix_and_rate(self):
        for options in ({"prefix_steps": 0}, {"prefix_steps": 2}, {"hz": 0}, {"hz": float("nan")}):
            with self.subTest(options=options), self.assertRaises(ContractError):
                self.plan([self.row()], **options)

    def test_lossless_observation_record_preserves_real_pixel_bytes(self):
        restored = read_observation(json.loads(json.dumps(observation_record(self.observation))))
        for original, replayed in zip(self.observation.frames, restored.frames):
            self.assertEqual(original.rgb, replayed.rgb)
            self.assertEqual(original.source, replayed.source)


class ReplayTests(unittest.TestCase):
    def test_all_units_and_action_modes_replay(self):
        for unit in ("rad", "deg"):
            for mode in ("absolute", "delta"):
                with self.subTest(unit=unit, mode=mode):
                    trace = json.loads(json.dumps(make_demo(joint_unit=unit, arm_action_mode=mode)))
                    result = replay(trace)
                    self.assertEqual(result["planned_targets"], 9)
                    self.assertFalse(result["execution_supported"])

    def test_altered_action_is_detected(self):
        trace = make_demo()
        trace["samples"][0]["raw_chunk"][0][0] += 0.0001
        with self.assertRaisesRegex(ContractError, "differs"):
            replay(trace)

    def test_overlapping_replans_are_detected(self):
        trace = make_demo()
        trace["samples"].append(copy.deepcopy(trace["samples"][0]))
        with self.assertRaisesRegex(ContractError, "predates"):
            replay(trace)

    def test_trace_persistence_and_refusal_to_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.json"
            save_trace(make_demo(), path)
            self.assertEqual(replay(load_trace(path))["samples"], 3)
            with self.assertRaises(FileExistsError):
                save_trace(make_demo(), path)

    def test_cli_offline_demo_then_replay(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "trace.json")
            for arguments in (("demo", "--output", path), ("replay", path)):
                result = subprocess.run([sys.executable, "-m", "vla", *arguments], cwd=root,
                                        capture_output=True, text=True, check=True)
                self.assertTrue(json.loads(result.stdout)["numerical_replay_passed"])
            result = subprocess.run([sys.executable, "-m", "vla", "execute"], cwd=root,
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()

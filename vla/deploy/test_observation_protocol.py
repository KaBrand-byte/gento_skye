"""Checks the physical observation boundary using the GPU deployment environment."""

import copy
import unittest

import numpy as np

from observation_protocol import droid_request
from serve_observation import ObservationPolicy


def observation():
    return {"schema": "skye_observation_v1", "observation_id": 7, "joint_unit": "rad",
            "arm_joints_rad": np.arange(14, dtype=np.float32) / 10,
            "grippers_rad": [0.47, None], "prompt": "Pick up the tray.",
            "images": {name: np.full((224, 224, 3), value, dtype=np.uint8)
                       for name, value in (("head", 10), ("wrist_left", 20), ("wrist_right", 30))},
            "sensor": {"image_ros_age_ms": 12}}


class ObservationBoundaryTests(unittest.TestCase):
    def test_left_arm_camera_mapping_and_gripper_normalization(self):
        obs = observation()
        request, assumptions = droid_request(obs)
        np.testing.assert_array_equal(request["observation/joint_position"], obs["arm_joints_rad"][:7])
        np.testing.assert_array_equal(request["observation/gripper_position"], [0.5])
        self.assertIs(request["observation/exterior_image_1_left"], obs["images"]["head"])
        self.assertIs(request["observation/wrist_image_left"], obs["images"]["wrist_left"])
        self.assertEqual(assumptions, [])

    def test_missing_gripper_requires_explicit_diagnostic_substitution(self):
        obs = observation()
        obs["grippers_rad"][0] = None
        with self.assertRaisesRegex(ValueError, "Missing left gripper"):
            droid_request(obs)
        request, assumptions = droid_request(obs, diagnostic_left_gripper_rad=0.0)
        self.assertFalse(assumptions[0]["measured"])
        self.assertEqual(assumptions[0]["diagnostic_value"], 0.0)
        np.testing.assert_array_equal(request["observation/gripper_position"], [0.0])

    def test_reject_wrong_units_missing_views_stale_or_nonfinite_data(self):
        mutations = [lambda x: x.update(joint_unit="deg"),
                     lambda x: x["images"].pop("wrist_right"),
                     lambda x: x["sensor"].update(image_ros_age_ms=900),
                     lambda x: x["arm_joints_rad"].__setitem__(0, np.nan),
                     lambda x: x["grippers_rad"].__setitem__(0, 1.5)]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                obs = copy.deepcopy(observation())
                mutate(obs)
                with self.assertRaises(ValueError):
                    droid_request(obs)

    def test_response_stays_in_droid_domain_and_cannot_be_executed(self):
        class FakePolicy:
            def infer(self, request):
                return {"actions": np.zeros((15, 8), dtype=np.float32)}
        policy = ObservationPolicy(FakePolicy(), 15, None)
        response = policy.infer(observation())
        self.assertFalse(response["execution_supported"])
        self.assertEqual(response["robot_actions_sent"], 0)
        self.assertEqual(response["observation_id"], 7)
        self.assertFalse(response["right_wrist_used_by_model"])
        self.assertEqual(response["action_domain"], "DROID_SINGLE_ARM_DIAGNOSTIC")


if __name__ == "__main__":
    unittest.main()

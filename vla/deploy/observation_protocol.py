"""Skye observations mapped to an explicitly diagnostic DROID request."""

import numpy as np


CAMERAS = ("head", "wrist_left", "wrist_right")


def droid_request(observation, *, diagnostic_left_gripper_rad=None):
    if observation.get("schema") != "skye_observation_v1":
        raise ValueError("Expected skye_observation_v1")
    if observation.get("joint_unit") != "rad":
        raise ValueError("Expected physical joint radians")
    joints = np.asarray(observation["arm_joints_rad"], dtype=np.float32)
    if joints.shape != (14,) or not np.isfinite(joints).all():
        raise ValueError("Expected 14 finite arm joints, left seven then right seven")
    images = observation["images"]
    if set(images) != set(CAMERAS):
        raise ValueError("Head and both wrist images are required")
    for name in CAMERAS:
        image = np.asarray(images[name])
        if image.shape != (224, 224, 3) or image.dtype != np.uint8:
            raise ValueError(f"{name}: expected 224x224 HWC RGB uint8")
    grippers = observation["grippers_rad"]
    if len(grippers) != 2:
        raise ValueError("Expected left and right gripper positions, or null when unavailable")
    for value in grippers:
        if value is not None and not np.isfinite(value):
            raise ValueError("Non-finite gripper feedback")
    assumptions = []
    left = grippers[0]
    if left is None:
        if diagnostic_left_gripper_rad is None:
            raise ValueError("Missing left gripper feedback; diagnostic substitution is disabled")
        left = diagnostic_left_gripper_rad
        assumptions.append({"field": "left_gripper_rad", "diagnostic_value": float(left),
                            "measured": False})
    if not np.isfinite(left) or not 0 <= left <= 0.94:
        raise ValueError("Left gripper is outside the currently configured 0..0.94 rad calibration")
    age = observation["sensor"]["image_ros_age_ms"]
    if not np.isfinite(age) or not -50 <= age <= 500:
        raise ValueError("Image timestamp is stale or incompatible with the Orin ROS clock")
    request = {
        "observation/exterior_image_1_left": images["head"],
        "observation/wrist_image_left": images["wrist_left"],
        "observation/joint_position": joints[:7],
        "observation/gripper_position": np.asarray([left / 0.94], dtype=np.float32),
        "prompt": observation["prompt"],
    }
    return request, assumptions

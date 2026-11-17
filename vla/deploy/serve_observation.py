"""Receive real Skye observations; return DROID diagnostics without robot execution."""

import argparse
import json
import logging
from pathlib import Path

import numpy as np

from observation_protocol import droid_request


class ObservationPolicy:
    def __init__(self, policy, horizon, diagnostic_left_gripper_rad):
        self.policy = policy
        self.horizon = horizon
        self.diagnostic_left_gripper_rad = diagnostic_left_gripper_rad
        self.metadata = {
            "schema": "skye_observation_v1", "checkpoint_config": "pi05_droid",
            "action_domain": "DROID_SINGLE_ARM_DIAGNOSTIC",
            "expected_action_shape": [horizon, 8],
            "execution_supported": False, "robot_actions_sent": 0,
            "cameras_received": ["head", "wrist_left", "wrist_right"],
            "cameras_used_by_droid": ["head", "wrist_left"],
            "diagnostic_left_gripper_rad_when_missing": diagnostic_left_gripper_rad,
            "skye_dual_arm_policy_ready": False,
        }

    def infer(self, observation):
        request, assumptions = droid_request(
            observation, diagnostic_left_gripper_rad=self.diagnostic_left_gripper_rad
        )
        result = self.policy.infer(request)
        actions = np.asarray(result["actions"])
        if actions.shape != (self.horizon, 8) or not np.isfinite(actions).all():
            raise RuntimeError(f"Invalid DROID response: {actions.shape}")
        result.update({
            "action_domain": "DROID_SINGLE_ARM_DIAGNOSTIC", "execution_supported": False,
            "robot_actions_sent": 0, "observation_id": observation["observation_id"],
            "diagnostic_assumptions": assumptions,
            "right_wrist_used_by_model": False,
            "grippers_measured": [value is not None for value in observation["grippers_rad"]],
        })
        print(json.dumps({"observation_id": observation["observation_id"],
                          "shape": list(actions.shape), "diagnostic_assumptions": assumptions,
                          "robot_actions_sent": 0}), flush=True)
        return result


def main():
    from openpi.policies import policy_config
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer
    from openpi.training import config

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--host", default="6.6.7.99")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--diagnostic-left-gripper-rad", type=float)
    args = parser.parse_args()
    if args.diagnostic_left_gripper_rad is not None and not 0 <= args.diagnostic_left_gripper_rad <= 0.94:
        parser.error("Diagnostic gripper value must be 0..0.94 rad")
    logging.basicConfig(level=logging.INFO)
    train_config = config.get_config("pi05_droid")
    policy = policy_config.create_trained_policy(train_config, args.checkpoint)
    wrapper = ObservationPolicy(policy, train_config.model.action_horizon, args.diagnostic_left_gripper_rad)
    print(json.dumps(wrapper.metadata), flush=True)
    WebsocketPolicyServer(wrapper, host=args.host, port=args.port, metadata=wrapper.metadata).serve_forever()


if __name__ == "__main__":
    main()

"""One passive SDK snapshot. Never enables, stops, resets, or commands a joint."""

import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    wrapper = args.sdk_root / "PYTHON_SDK/GentoRobot.py"
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location("vla_observe_gento_sdk", wrapper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    lock_path = Path("/tmp") / f"skye-dm-sdk-{os.getuid()}-6.6.7.190.lock"
    with lock_path.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        robot = module.GentoRobot(precision=6)
        connected = False
        try:
            sdk_version = robot.get_sdk_version()
            if not sdk_version.upper().startswith("000407"):
                raise RuntimeError(f"Expected the inspected 0407 SDK, got {sdk_version}")
            status = robot.link(6, 6, 7, 190, log_level=0)
            if status < 0:
                raise RuntimeError(f"SDK connection rejected: {status}; no retry or takeover")
            connected = True
            controller_version = robot.get_controller_version()
            if not controller_version.upper().startswith("000407"):
                raise RuntimeError(f"Expected 0407 controller, got {controller_version}")
            samples = []
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                feedback = robot.get_rt_dict()
                if "error" in feedback:
                    raise RuntimeError(str(feedback["error"]))
                samples.append((time.monotonic_ns(), feedback))
                time.sleep(0.01)
            configuration = robot.get_sg_dict()
            result = {
                "sdk_version": sdk_version, "controller_version": controller_version,
                "robot_type": robot.get_robot_type(), "poll_samples": len(samples),
                "sample_window_seconds": (samples[-1][0] - samples[0][0]) / 1e9,
                "first_rt_frame_serial": samples[0][1]["frame_serial"],
                "last_rt_frame_serial": samples[-1][1]["frame_serial"],
                "last_receive_monotonic_ns": samples[-1][0],
                "last_rt": samples[-1][1], "sg_configuration": configuration,
                "gripper_position_available": False,
                "images_synchronized": False,
                "robot_actions_sent": 0,
            }
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps({key: value for key, value in result.items()
                              if key not in ("last_rt", "sg_configuration")}, indent=2))
        finally:
            if connected:
                robot.unlink()
                time.sleep(2)  # Respect the controller's disconnect cooldown.


if __name__ == "__main__":
    main()

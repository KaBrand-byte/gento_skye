"""Orin camera + passive SDK client. Records policy responses; has no motion API."""

import argparse
from ctypes import byref, c_int32, c_ubyte, c_uint32
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import cv2
import numpy as np


class PassiveRobot:
    def __init__(self, sdk_root, task_repo):
        self.sdk_root = sdk_root
        self.task_repo = task_repo
        self.robot = None
        self.lock = None
        self.grippers = {}
        self.last_frame = None
        self.last_frame_changed = time.monotonic()

    def __enter__(self):
        for service in ("apex-robot.service", "apex-teleop.service"):
            state = subprocess.run(["systemctl", "is-active", service], capture_output=True, text=True)
            if state.stdout.strip() in ("active", "activating", "reloading"):
                raise RuntimeError(f"{service} owns robot control; refusing SDK connection")
        self.lock = open(f"/tmp/skye-dm-sdk-{os.getuid()}-6.6.7.190.lock", "a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            sys.path.insert(0, str(self.task_repo))
            from skye_task.dm_protocol import decode_feedback
            self.decode_feedback = decode_feedback
            wrapper = self.sdk_root / "PYTHON_SDK/GentoRobot.py"
            spec = importlib.util.spec_from_file_location("skye_vla_passive_sdk", wrapper)
            module = importlib.util.module_from_spec(spec)
            exec(compile(wrapper.read_bytes(), str(wrapper), "exec"), module.__dict__)
            robot = module.GentoRobot(precision=6)
            if not robot.get_sdk_version().upper().startswith("000407"):
                raise RuntimeError("Expected the inspected 0407 SDK")
            if robot.link(6, 6, 7, 190, log_level=0) < 0:
                raise RuntimeError("SDK connection rejected; no takeover or retry")
            self.robot = robot
            if not robot.get_controller_version().upper().startswith("000407"):
                raise RuntimeError("Expected the inspected 0407 controller")
            return self
        except BaseException:
            self.close()
            raise

    def read(self):
        if self.robot.check_link_state() != 1:
            raise RuntimeError("Robot feedback connection lost")
        feedback = self.robot.get_rt_dict()
        if "error" in feedback:
            raise RuntimeError(str(feedback["error"]))
        now = time.monotonic()
        if feedback["frame_serial"] != self.last_frame:
            self.last_frame = feedback["frame_serial"]
            self.last_frame_changed = now
        elif now - self.last_frame_changed > 0.5:
            raise RuntimeError("RT feedback stopped changing")
        joints = [math.radians(value) for arm in feedback["arms"] for value in arm["fb"]["fb_pos"]]
        for arm in (0, 1):
            for _ in range(16):
                channel, elapsed, buffer = c_int32(), c_uint32(), (c_ubyte * 64)()
                count = self.robot.dll.FX_L1_Terminal_GetData(arm, 0, byref(channel), buffer, byref(elapsed))
                if count in (0, -10):
                    break
                if count != 12 or channel.value != 1:
                    raise RuntimeError(f"Unexpected passive terminal feedback: count={count}, channel={channel.value}")
                raw = bytes(buffer[:count])
                can_id, = struct.unpack("<I", raw[:4])
                frame = struct.pack("<IB3x8s", can_id, 8, raw[4:])
                decoded = self.decode_feedback(frame, slave_id=arm + 1, master_id=17 + arm,
                                               received_at=time.monotonic())
                if decoded is not None:
                    if decoded.status not in (0, 1):
                        raise RuntimeError(f"Gripper {arm} reports fault {decoded.status}")
                    self.grippers[arm] = decoded
        grippers = [self.grippers[arm].position_rad
                    if arm in self.grippers and time.monotonic() - self.grippers[arm].received_at <= 0.5
                    else None for arm in (0, 1)]
        return np.asarray(joints, dtype=np.float32), grippers, feedback["frame_serial"], time.monotonic_ns()

    def close(self):
        try:
            if self.robot is not None:
                self.robot.unlink()
                self.robot = None
                time.sleep(1.1)
        finally:
            if self.lock is not None:
                self.lock.close()
                self.lock = None

    def __exit__(self, *exc):
        self.close()


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from openpi_client.image_tools import resize_with_pad
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="6.6.7.99")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--requests", type=int, default=5)
    parser.add_argument("--max-seconds", type=int, default=150)
    parser.add_argument("--prompt", default="Pick up the tray and place it on the table.")
    parser.add_argument("--topic", default="/quad_tile/jpeg/compressed")
    parser.add_argument("--sdk-root", type=Path, default=Path.home() / "robot_sdk/tianji-robot-SDK")
    parser.add_argument("--task-repo", type=Path, default=Path.home() / "gento_skye")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.requests < 1 or args.max_seconds < 1:
        parser.error("Requests and timeout must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    summary = {"passed": False, "requests": [], "robot_actions_sent": 0,
               "execution_supported": False, "source": "live Orin cameras and passive SDK"}

    def timeout(signum, frame):
        raise TimeoutError("Observation/inference session exceeded its time limit")

    signal.signal(signal.SIGALRM, timeout)
    signal.alarm(args.max_seconds)
    rclpy.init()
    node = rclpy.create_node("skye_vla_observe_infer")
    latest = [None]

    def receive(message):
        if "jpeg" in message.format.lower() or message.format.lower() == "jpg":
            latest[0] = (message, time.monotonic_ns())

    node.create_subscription(CompressedImage, args.topic, receive,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    client = None
    try:
        client = WebsocketClientPolicy(host=args.host, port=args.port)
        summary["server_metadata"] = client.get_server_metadata()
        metadata = summary["server_metadata"]
        if metadata.get("schema") != "skye_observation_v1" or metadata.get("execution_supported") is not False:
            raise RuntimeError("Expected the Skye observation-only service")
        previous_stamp = None
        with PassiveRobot(args.sdk_root, args.task_repo) as robot, (args.output / "requests.jsonl").open("w") as log:
            for index in range(args.requests):
                wait_until = time.monotonic() + 40
                while True:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if latest[0] is not None:
                        message, received_ns = latest[0]
                        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                        age_ms = (node.get_clock().now().nanoseconds - stamp) / 1e6
                        if stamp != previous_stamp and -50 <= age_ms <= 500:
                            break
                    if time.monotonic() > wait_until:
                        raise TimeoutError("No fresh mosaic JPEG; initialize/start the camera-only node")
                previous_stamp = stamp
                mosaic = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if mosaic is None or mosaic.shape != (720, 1280, 3):
                    raise RuntimeError("Expected the configured 1280x720 2x2 mosaic")
                # Factory layout: CSI0 head TL; CSI1 disabled TR; CSI4 left BL; CSI5 right BR.
                tiles = {"head": mosaic[:360, :640], "wrist_left": mosaic[360:, :640],
                         "wrist_right": mosaic[360:, 640:]}
                images = {name: resize_with_pad(cv2.cvtColor(tile, cv2.COLOR_BGR2RGB), 224, 224)
                          for name, tile in tiles.items()}
                joints, grippers, serial, state_ns = robot.read()
                observation = {
                    "schema": "skye_observation_v1", "observation_id": index, "joint_unit": "rad",
                    "arm_joints_rad": joints, "grippers_rad": grippers, "images": images,
                    "prompt": args.prompt,
                    "sensor": {"image_ros_stamp_ns": stamp, "state_frame_serial": serial,
                               "image_ros_age_ms": (node.get_clock().now().nanoseconds - stamp) / 1e6,
                               "image_to_state_receive_ms": (state_ns - received_ns) / 1e6,
                               "per_camera_capture_stamps_available": False},
                }
                started = time.perf_counter()
                response = client.infer(observation)
                elapsed_ms = (time.perf_counter() - started) * 1000
                actions = np.asarray(response["actions"])
                if (response.get("observation_id") != index or response.get("execution_supported") is not False
                        or response.get("action_domain") != "DROID_SINGLE_ARM_DIAGNOSTIC"
                        or list(actions.shape) != metadata["expected_action_shape"] or not np.isfinite(actions).all()):
                    raise RuntimeError("Unexpected observation policy response")
                record = {"observation_id": index, "sensor": observation["sensor"],
                          "arm_joints_rad": joints, "grippers_rad": grippers, "prompt": args.prompt,
                          "response": response, "client_ms": elapsed_ms}
                log.write(json.dumps(record, default=json_value) + "\n")
                log.flush()
                sample = {"index": index, "client_ms": elapsed_ms, "shape": list(actions.shape),
                          "image_ros_age_ms": observation["sensor"]["image_ros_age_ms"],
                          "grippers_measured": response["grippers_measured"],
                          "diagnostic_assumptions": response["diagnostic_assumptions"],
                          "robot_actions_sent": 0}
                summary["requests"].append(sample)
                print(json.dumps(sample), flush=True)
                if index == 0:
                    cv2.imwrite(str(args.output / "mosaic.jpg"), mosaic)
                    for name, image in images.items():
                        cv2.imwrite(str(args.output / f"{name}.jpg"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        summary["passed"] = True
    except BaseException as error:
        summary["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        signal.alarm(0)
        if client is not None:
            client._ws.close()
        node.destroy_node()
        rclpy.shutdown()
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2, default=json_value) + "\n")


if __name__ == "__main__":
    main()

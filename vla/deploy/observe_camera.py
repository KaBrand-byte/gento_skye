"""Read-only ROS JPEG observation probe; no robot control publishers or services."""

import argparse
import json
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/quad_tile/jpeg/compressed")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("skye_vla_camera_observer")
    received = []
    ros_stamps = []
    formats = set()
    last_image = None

    def receive(message):
        nonlocal last_image
        formats.add(message.format)
        if "jpeg" not in message.format.lower() and message.format.lower() != "jpg":
            return
        received.append(time.monotonic_ns())
        ros_stamps.append(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)
        last_image = bytes(message.data)

    node.create_subscription(CompressedImage, args.topic, receive,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    started = time.monotonic()
    try:
        while time.monotonic() - started < 45:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(received) > 1 and (received[-1] - received[0]) >= 8_000_000_000:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    duration = (received[-1] - received[0]) / 1e9 if len(received) > 1 else 0
    result = {
        "topic": args.topic, "jpeg_messages": len(received),
        "received_formats": sorted(formats),
        "observed_seconds": duration,
        "received_hz": (len(received) - 1) / duration if duration else None,
        "distinct_ros_timestamps": len(set(ros_stamps)),
        "first_receive_monotonic_ns": received[0] if received else None,
        "last_receive_monotonic_ns": received[-1] if received else None,
        "last_ros_stamp_ns": ros_stamps[-1] if ros_stamps else None,
        "robot_actions_sent": 0,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    if last_image:
        (args.output / "mosaic.jpg").write_bytes(last_image)
    (args.output / "camera-observation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if len(received) < 2:
        raise SystemExit("No usable JPEG stream received")


if __name__ == "__main__":
    main()

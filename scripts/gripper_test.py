#!/usr/bin/env python3
"""Standalone absolute-angle calibration; all motion uses skye_task's checked driver."""

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from skye_task.gripper_cli import main as gripper_main


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="独立夹爪绝对角度测试：正方向闭合，负方向张开；结束后失能，仅用于空载。")
    parser.add_argument("action", choices=("open", "close"), help="张开或闭合到指定绝对角度")
    parser.add_argument("--side", choices=("left", "right", "both"), default="left")
    parser.add_argument("--position-rad", type=float, required=True,
                        help="本次绝对目标电机角度；本机闭合目标应高于当前角度，张开目标应低于当前角度")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config/gripper-can.json")
    parser.add_argument("--robot-config", type=Path, default=PROJECT_ROOT / "config/default.json")
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--result", type=Path, help="完整结果路径，默认 runs/gripper-test-侧别-动作.json")
    args = parser.parse_args(argv)
    result = args.result or PROJECT_ROOT / f"runs/gripper-test-{args.side}-{args.action}.json"
    return gripper_main([
        args.action, "--transport", "sdk", "--side", args.side,
        "--position-rad", str(args.position_rad), "--config", str(args.config),
        "--robot-config", str(args.robot_config), "--hold-seconds", str(args.hold_seconds),
        "--disable-on-exit", "--brief", "--result", str(result),
    ])


if __name__ == "__main__":
    raise SystemExit(main())

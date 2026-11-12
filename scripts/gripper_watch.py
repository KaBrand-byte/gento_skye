#!/usr/bin/env python3
"""Read fresh gripper angles while keeping the selected motors disabled."""

import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import signal
import sys
import threading
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from skye_task.config import load_config
from skye_task.dm_gripper import DMGripper, load_gripper_config, selected_sides
from skye_task.dm_terminal import GentoTerminalConnection


def format_angles(snapshot, sides):
    values = []
    for side in sides:
        angle = snapshot["feedback"][side]["position_rad"]
        label = "左" if side == "left" else "右"
        values.append(f"{label} {angle:+.6f} rad ({math.degrees(angle):+.2f}°) Off")
    return " | ".join(values)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="独立夹爪角度监视：周期发送失能帧获取新反馈，确认 Off 后可空载手动开合；Ctrl-C 退出。")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config/gripper-can.json")
    parser.add_argument("--robot-config", type=Path, default=PROJECT_ROOT / "config/default.json")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="读取／刷新频率，默认 10 Hz，范围 1–50")
    parser.add_argument("--samples", type=int, help="读取指定次数后退出；默认一直读取")
    parser.add_argument("--csv", type=Path, help="可选：将每次新反馈记录到 CSV（覆盖同名文件）")
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.rate_hz) or not 1 <= args.rate_hz <= 50:
            raise ValueError("rate-hz 必须是 1–50 的有限数值")
        if args.samples is not None and args.samples <= 0:
            raise ValueError("samples 必须为正整数")
        config = load_gripper_config(args.config)
        robot_config = load_config(args.robot_config)
    except (ValueError, OSError) as error:
        parser.error(str(error))

    sides = selected_sides(args.side)
    stop = threading.Event()
    old_handlers = {}
    connection = device = csv_file = None
    width = count = 0
    exit_code = 0
    try:
        if args.csv is not None:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            csv_file = args.csv.open("w", encoding="utf-8", newline="")
            writer = csv.writer(csv_file)
            writer.writerow(("time_utc", "side", "position_rad", "position_deg",
                             "velocity_rad_s", "torque_nm", "status"))
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: stop.set())

        print("将周期发送失能帧并读取反馈；等待所选夹爪确认 Off。", flush=True)
        connection = GentoTerminalConnection(robot_config, config)
        connection.connect()
        device = DMGripper(config, args.side, transport_factory=connection.transport)
        device.connect()
        while not stop.is_set() and (args.samples is None or count < args.samples):
            started = time.monotonic()
            # The motor need not publish unsolicited feedback while Off. FD has
            # already been verified to produce an Off response on this hardware.
            # disable() drains old data and requires a fresh Off from EVERY side;
            # failure therefore never reprints a cached angle as a new sample.
            device.disable()
            if stop.is_set():
                break
            sample = device.snapshot()
            if count == 0:
                print("所选夹爪均已确认 Off，可空载手动开合。角度增大为闭合，减小为张开。", flush=True)
                print("显示电机实际绝对角度；Ctrl-C 退出。", flush=True)
            count += 1
            timestamp = datetime.now(timezone.utc)
            row = timestamp.astimezone().strftime("%H:%M:%S.%f")[:-3] + " | " + format_angles(sample, sides)
            if sys.stdout.isatty():
                width = max(width, len(row))
                print("\r" + row.ljust(width), end="", flush=True)
            else:
                print(row, flush=True)
            if csv_file is not None:
                for side in sides:
                    value = sample["feedback"][side]
                    writer.writerow((timestamp.isoformat(), side, value["position_rad"],
                                     math.degrees(value["position_rad"]), value["velocity_rad_s"],
                                     value["torque_nm"], value["status"]))
                csv_file.flush()
            if args.samples is None or count < args.samples:
                stop.wait(max(0.0, 1 / args.rate_hz - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if width:
            print()
            width = 0
        print(f"角度读取停止: {error}。未收到新鲜 Off 反馈时，请勿据旧值判断角度或失能状态。",
              file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        if width:
            print()
        # Never enable or send a position/velocity/torque target, including on exit.
        for resource in (device, connection, csv_file):
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    print(f"关闭角度监视失败: {error}", file=sys.stderr)
                    exit_code = 1
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        print(f"角度监视已退出，共读取 {count} 组新反馈。", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

"""Standalone DM4310 checks over SocketCAN or a dedicated SDK Terminal link."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import socket
import threading
import time

from .dm_gripper import DMGripper, load_gripper_config, selected_sides
from .config import load_config
from .dm_terminal import GentoTerminalConnection
from .sdk import MotionError
from .sequence import Stopped


def main(argv=None):
    parser = argparse.ArgumentParser(description="独立 DM4310 夹爪检测，不移动双臂")
    parser.add_argument("action", choices=("inspect", "status", "step", "move", "open", "close", "disable"),
                        help="inspect 检查配置／接口；status 被动反馈；step 有符号小步；"
                             "move 绝对角度；open/close 去指定或配置的绝对角度；disable 失能")
    parser.add_argument("--config", type=Path, help="独立夹爪配置，例如 config/gripper-can.json")
    parser.add_argument("--transport", choices=("socketcan", "sdk"), default="socketcan",
                        help="socketcan 使用已桥接的 CAN 接口；sdk 通过以太网直连末端 CAN")
    parser.add_argument("--robot-config", type=Path, help="sdk 连接配置，例如 config/default.json")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--delta-rad", type=float, help="step 有符号相对位移；本机正方向闭合、负方向张开")
    parser.add_argument("--position-rad", type=float, help="move/open/close 的绝对电机角度，单位 rad")
    parser.add_argument("--timeout-seconds", type=float, default=3.0, help="被动反馈接收超时")
    parser.add_argument("--hold-seconds", type=float, default=2.0, help="到位后继续周期保持的秒数")
    parser.add_argument("--disable-on-exit", action="store_true",
                        help="运动结束或失败时发送失能；仅在允许卸力时使用")
    parser.add_argument("--brief", action="store_true", help="只显示目标／实际角度及结果，完整记录仍保存为 JSON")
    parser.add_argument("--result", type=Path, default=Path("runs/gripper-latest.json"))
    args = parser.parse_args(argv)
    motion_actions = ("step", "move", "open", "close")
    absolute_targets = None
    try:
        config = load_gripper_config(args.config)
        robot_config = load_config(args.robot_config) if args.transport == "sdk" else None
        if args.robot_config is not None and args.transport != "sdk":
            raise ValueError("robot-config 只用于 --transport sdk")
        if not math.isfinite(args.timeout_seconds) or args.timeout_seconds <= 0:
            raise ValueError("timeout-seconds 必须为有限正数")
        if not math.isfinite(args.hold_seconds) or args.hold_seconds < 0:
            raise ValueError("hold-seconds 必须为有限非负数")
        if args.action == "step":
            if args.position_rad is not None:
                raise ValueError("step 使用 delta-rad；position-rad 用于绝对角度动作")
            if (args.delta_rad is None or not math.isfinite(args.delta_rad)
                    or not 0 < abs(args.delta_rad) <= config.max_step_rad):
                raise ValueError(f"step 需要 --delta-rad，且 0 < |delta| <= {config.max_step_rad}")
        elif args.action in ("move", "open", "close"):
            if args.delta_rad is not None:
                raise ValueError("delta-rad 只用于 step；绝对角度动作使用 position-rad")
            if args.position_rad is not None:
                absolute_targets = {name: args.position_rad for name in selected_sides(args.side)}
            elif args.action == "move":
                raise ValueError("move 需要 --position-rad 指定绝对目标角度")
            else:
                absolute_targets = config.action_positions(args.action, args.side)
            absolute_targets = config.validate_positions(absolute_targets, selected_sides(args.side))
        elif args.delta_rad is not None or args.position_rad is not None or args.disable_on_exit:
            raise ValueError("delta-rad、position-rad 和 disable-on-exit 只用于运动动作")
    except (ValueError, OSError) as error:
        parser.error(str(error))

    # Verify result writability before enabling a physical motor.
    args.result.parent.mkdir(parents=True, exist_ok=True)
    with args.result.open("a", encoding="utf-8"):
        pass
    report = {"mode": "standalone_dm4310", "action": args.action, "side": args.side,
              "transport": args.transport,
              "started_at": datetime.now(timezone.utc).isoformat(), "status": "checking",
              "config": asdict(config), "arm_motion": "not_called", "grasped": None,
              "placed": None, "error": "", "cleanup_error": ""}
    stop_requested = threading.Event()
    old_handlers = {}
    device = None
    terminal_connection = None
    exit_code = 1

    def checkpoint():
        if stop_requested.is_set():
            raise Stopped("收到停止请求，结束夹爪检测")

    def cleanup_failed(error):
        nonlocal exit_code
        report["cleanup_error"] = "; ".join(filter(None, [report["cleanup_error"], str(error)]))
        report["status"] = "failed"
        exit_code = 1

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: stop_requested.set())
        if args.action == "inspect" and args.transport == "sdk":
            report["controller_ip"] = robot_config.controller_ip
            report["sdk_root"] = robot_config.sdk_root
            report["requires_vcan"] = False
            report["physical_route"] = "not_verified"
        elif args.action == "inspect":
            try:
                interfaces = {name for _, name in socket.if_nameindex()}
            except OSError:
                # Some restricted shells prohibit the netlink call used by if_nameindex.
                interfaces = {entry.name for entry in Path("/sys/class/net").iterdir()}
            report["interfaces"] = {
                name: {"name": getattr(config, name).interface,
                       "exists": getattr(config, name).interface in interfaces}
                for name in selected_sides(args.side)}
            report["physical_route"] = "not_verified"
            if not all(info["exists"] for info in report["interfaces"].values()):
                raise MotionError("本机缺少配置的 CAN 接口；未连接实机透传链路，不能检测物理夹爪")
        else:
            if args.transport == "sdk":
                terminal_connection = GentoTerminalConnection(robot_config, config)
                terminal_connection.connect()
                device = DMGripper(config, args.side, transport_factory=terminal_connection.transport)
            else:
                device = DMGripper(config, args.side)
            device.connect()
            if args.action == "status":
                report.update(device.read_status(args.timeout_seconds, checkpoint))
                # Off is valid passive feedback; faults remain visible and return failure.
                if any(value["status"] not in (0, 1) for value in report["feedback"].values()):
                    raise MotionError("夹爪反馈含故障或未知状态，详见原始帧")
            elif args.action == "disable":
                device.disable()
                report["disable"] = "confirmed_off"
            else:
                if args.action == "step":
                    report["motion_scope"] = "relative_step"
                    deltas = {name: args.delta_rad for name in selected_sides(args.side)}
                    report["delta_rad"] = args.delta_rad
                    report["requested_deltas_rad"] = deltas
                    report["gripper_actions"] = {
                        name: "close" if delta * getattr(config, name).close_direction > 0 else "open"
                        for name, delta in deltas.items()}
                else:
                    report["motion_scope"] = "absolute_position"
                    report["requested_targets_rad"] = absolute_targets
                    if args.action in ("open", "close"):
                        report["gripper_actions"] = {name: args.action for name in selected_sides(args.side)}
                device.enable(checkpoint)
                report["before_step"] = device.snapshot()
                if args.action == "step":
                    report["at_target"] = device.step(args.delta_rad, checkpoint)
                else:
                    report["at_target"] = device.move_to(
                        absolute_targets, checkpoint, action=args.action if args.action != "move" else None)
                until = time.monotonic() + args.hold_seconds
                while time.monotonic() < until:
                    checkpoint()
                    device.check_health()
                    stop_requested.wait(min(0.02, max(0, until - time.monotonic())))
                device.check_health()
                report.update(device.snapshot())
        report["status"] = "completed"
        exit_code = 0
    except (Exception, KeyboardInterrupt) as error:
        stopped = isinstance(error, (Stopped, KeyboardInterrupt))
        report.update(status="stopped" if stopped else "failed", error=str(error) or type(error).__name__)
        exit_code = 130 if stopped else 1
    finally:
        if device is not None:
            # Preserve the last active feedback before an explicit disable changes
            # status/torque and allows the mechanism to relax.
            try:
                report["before_cleanup"] = device.snapshot()
            except Exception as error:
                cleanup_failed(error)
            if args.action in motion_actions and args.disable_on_exit and device.enabled_sides:
                try:
                    device.disable()
                    report["disable_on_exit"] = "confirmed_off"
                except Exception as error:
                    report["disable_on_exit"] = "not_confirmed"
                    cleanup_failed(error)
            try:
                report["final"] = device.snapshot()
            except Exception as error:
                cleanup_failed(error)
            try:
                device.close()
            except Exception as error:
                cleanup_failed(error)
        if terminal_connection is not None:
            try:
                report["sdk_terminal"] = terminal_connection.snapshot()
            except Exception as error:
                cleanup_failed(error)
            try:
                terminal_connection.close()
            except Exception as error:
                cleanup_failed(error)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        temporary = args.result.with_suffix(args.result.suffix + ".tmp")
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                             encoding="utf-8")
        temporary.replace(args.result)
        if args.brief:
            _print_brief(report)
        else:
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        print(f"夹爪检测结果: {args.result} ({report['status']})")
    return exit_code


def _print_brief(report):
    active = report.get("before_cleanup", {})
    initial = report.get("before_step", {}).get("feedback", {})
    final = report.get("final", {}).get("feedback", {})
    targets = report.get("requested_targets_rad", active.get("targets_rad", {}))
    def angle(value):
        return "未知" if value is None else f"{value:.6f}"
    for name in selected_sides(report["side"]):
        feedback = active.get("feedback", {}).get(name, {})
        target, actual = targets.get(name), feedback.get("position_rad")
        error = target - actual if target is not None and actual is not None else None
        print(f"{name}: 起始 {angle(initial.get(name, {}).get('position_rad'))} rad，"
              f"目标 {angle(target)} rad，实际 {angle(actual)} rad，误差 {angle(error)} rad")
        print(f"  运动结束状态 {feedback.get('status_name', '未知')}，"
              f"退出状态 {final.get(name, {}).get('status_name', '未知')}")
    if report.get("disable_on_exit"):
        print(f"退出失能: {report['disable_on_exit']}")
    if report.get("error"):
        print(f"错误: {report['error']}")
    if report.get("cleanup_error"):
        print(f"清理错误: {report['cleanup_error']}")

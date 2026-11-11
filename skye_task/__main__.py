"""CLI: config inspection is offline; run explicitly connects and moves hardware."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import select
import signal
import sys

from .config import load_config
from .preflight import check_current_plan
from .sdk import GentoSDK
from .sequence import Result, Sequence, Stopped


def wait_for_start(sequence):
    message = ("已到接盘位且双爪张开。递盘就绪后按 Enter 闭爪并执行任务，Ctrl-C 停止。"
               if sequence.config.gripper_enabled else "已到接盘位。按 Enter 触发空载任务，Ctrl-C 停止。")
    print(message, flush=True)
    while True:
        sequence.checked_feedback()
        readable, _, _ = select.select([sys.stdin], [], [], sequence.config.poll_seconds)
        if readable:
            line = sys.stdin.readline()
            if not line:
                raise Stopped("标准输入关闭，未触发任务")
            if not line.strip():
                return


def write_report(path, report):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
                         + "\n", encoding="utf-8")
    temporary.replace(path)


def run_plan_check(config, path):
    sdk = None
    report = {"mode": "read_only_plan_check", "status": "failed", "error": ""}
    try:
        sdk = GentoSDK(config)
        check_current_plan(sdk, config, report,
                           on_event=lambda name, detail: logging.info("%s %s", name, detail))
    except (Exception, KeyboardInterrupt) as error:
        report.update(status="failed", error=str(error) or type(error).__name__)
    finally:
        if sdk is not None:
            try:
                sdk.close()
            except Exception as error:
                report.update(status="failed", cleanup_error=str(error))
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        write_report(path, report)
    print(f"只读规划结果: {path} ({report['status']})")
    if report["status"] != "planned":
        logging.error("只读规划失败: %s", report["error"])
    return 0 if report["status"] == "planned" else 1


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "gripper":
        # Keep standalone diagnostics independent of the robot controller connection.
        from .gripper_cli import main as gripper_main
        return gripper_main(argv[1:])
    parser = argparse.ArgumentParser(description="Gento Skye 固定底盘双臂与夹爪编排")
    parser.epilog = "独立夹爪检测: python3 -m skye_task gripper --help"
    parser.add_argument("command", choices=("check", "plan-check", "run"),
                        help="check 离线配置检查；plan-check 只读规划；run 连接并执行")
    parser.add_argument("--config", type=Path, help="JSON 配置覆盖文件")
    parser.add_argument("--result", type=Path, help="结果 JSON 路径")
    parser.add_argument("--gripper-config", type=Path,
                        default=Path(__file__).resolve().parents[1] / "config/gripper-can.json",
                        help="整体任务夹爪配置；默认使用项目中的 config/gripper-can.json")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        gripper_config = None
        if config.gripper_enabled and args.command != "plan-check":
            from .dm_gripper import load_gripper_config
            gripper_config = load_gripper_config(args.gripper_config)
            for action in ("open", "close"):
                gripper_config.action_positions(action, "both")
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if args.command == "check":
        values = asdict(config)
        if gripper_config is not None:
            values["gripper"] = asdict(gripper_config)
        print(json.dumps(values, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    if args.result is None:
        args.result = Path("runs/plan-check.json" if args.command == "plan-check" else "runs/latest.json")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    # Check output writability before issuing any robot command.
    args.result.parent.mkdir(parents=True, exist_ok=True)
    with args.result.open("a", encoding="utf-8"):
        pass
    if args.command == "plan-check":
        return run_plan_check(config, args.result)
    sdk = None
    sequence = None
    gripper = None
    result = Result()
    old_handlers = {}
    exit_code = 1
    try:
        sdk = GentoSDK(config)
        if gripper_config is not None:
            from .task_gripper import TaskGripper
            gripper = TaskGripper(sdk, gripper_config)
        sequence = Sequence(sdk, config, gripper=gripper,
                            on_event=lambda name, detail: logging.info("%s %s", name, detail))
        result = sequence.result
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: sequence.request_stop())
        sequence.prepare()
        wait_for_start(sequence)
        sequence.start()
        exit_code = 0
    except (Exception, KeyboardInterrupt) as error:
        logging.error("流程停止: %s", error)
        if sequence is not None:
            sequence.fail(error)
            exit_code = 130 if sequence.result.status == "stopped" else 1
        else:
            result.status = "failed"
            result.error = str(error)
            result.finished_at = datetime.now(timezone.utc).isoformat()
    finally:
        sdk_can_close = True
        if gripper is not None:
            try:
                result.metadata["gripper_final"] = gripper.snapshot()
            except Exception as error:
                result.metadata["gripper_snapshot_error"] = str(error)
            try:
                gripper.close()
                result.metadata["gripper_cleanup"] = "periodic_commands_stopped_no_disable_sent"
            except Exception as error:
                logging.error("夹爪资源释放失败: %s", error)
                result.metadata["gripper_cleanup_error"] = str(error)
                result.error = result.error or str(error)
                result.status = "failed"
                exit_code = 1
                if gripper.worker_running:
                    sdk_can_close = False
                    result.metadata["sdk_cleanup_skipped"] = "gripper_worker_still_running"
        if sdk is not None and sdk_can_close:
            try:
                sdk.close()
            except Exception as error:
                logging.error("SDK 资源释放失败: %s", error)
                exit_code = 1
                if sequence is not None:
                    sequence.result.metadata["cleanup_error"] = str(error)
                result.status = "failed"
                result.error = result.error or str(error)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        write_report(args.result, result.to_dict())
        print(f"执行结果: {args.result} ({result.status})")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

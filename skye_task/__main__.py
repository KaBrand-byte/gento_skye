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
    print("已到接盘位。按 Enter 触发空载任务，Ctrl-C 停止。", flush=True)
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
    parser = argparse.ArgumentParser(description="Gento Skye 固定底盘空载双臂编排")
    parser.add_argument("command", choices=("check", "plan-check", "run"),
                        help="check 离线配置检查；plan-check 只读规划；run 连接并执行")
    parser.add_argument("--config", type=Path, help="JSON 配置覆盖文件")
    parser.add_argument("--result", type=Path, help="结果 JSON 路径")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if args.command == "check":
        print(json.dumps(asdict(config), ensure_ascii=False, indent=2, allow_nan=False))
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
    result = Result()
    old_handlers = {}
    exit_code = 1
    try:
        sdk = GentoSDK(config)
        sequence = Sequence(sdk, config, on_event=lambda name, detail: logging.info("%s %s", name, detail))
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
        if sdk is not None:
            try:
                sdk.close()
            except Exception as error:
                logging.error("SDK 资源释放失败: %s", error)
                exit_code = 1
                if sequence is not None:
                    sequence.result.metadata["cleanup_error"] = str(error)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        write_report(args.result, result.to_dict())
        print(f"执行结果: {args.result} ({result.status})")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

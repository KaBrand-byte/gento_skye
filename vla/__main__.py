"""Run with python3 -m vla; all commands are offline."""

import argparse
import json

from .core import ContractError
from .offline import load_trace, make_demo, replay, save_trace


def main():
    parser = argparse.ArgumentParser(description="Skye VLA offline logic validation; no robot connections")
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="create and validate a synthetic trace")
    demo.add_argument("--output", required=True, help="new trace file; existing files are never overwritten")
    demo.add_argument("--joint-unit", choices=("rad", "deg"), default="rad")
    demo.add_argument("--arm-action-mode", choices=("absolute", "delta"), default="absolute")
    replay_parser = commands.add_parser("replay", help="validate a recorded observation/action trace")
    replay_parser.add_argument("trace")
    args = parser.parse_args()
    try:
        if args.command == "demo":
            trace = make_demo(joint_unit=args.joint_unit, arm_action_mode=args.arm_action_mode)
            result = replay(trace)
            save_trace(trace, args.output)
            result["trace"] = args.output
        else:
            result = replay(load_trace(args.trace))
    except (ContractError, OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"REJECTED: {error}\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

"""Official pi05-DROID smoke test with random observations, without robot I/O."""

import argparse
import json
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import sys
import time
import urllib.request


def serve(checkpoint, port):
    from openpi.policies import policy_config
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer
    from openpi.training import config

    policy = policy_config.create_trained_policy(config.get_config("pi05_droid"), checkpoint)
    # Only local synthetic clients are involved in this stage.
    WebsocketPolicyServer(policy=policy, host="127.0.0.1", port=port, metadata=policy.metadata).serve_forever()


def benchmark(root, checkpoint, steps):
    import numpy as np
    from openpi.policies.droid_policy import make_droid_example
    from openpi_client.websocket_client_policy import WebsocketClientPolicy
    from openpi.training import config

    expected_shape = (config.get_config("pi05_droid").model.action_horizon, 8)

    def timeout_handler(signum, frame):
        raise TimeoutError("Smoke test exceeded 15 minutes; see server log")

    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(900)
    output = root / "logs/smoke-result.json"
    result = {
        "checkpoint": str(checkpoint), "config": "pi05_droid",
        "observation_source": "official make_droid_example, random synthetic inputs",
        "robot_connected": False, "robot_actions_sent": 0,
        "openpi_commit": subprocess.check_output(
            ["git", "-C", str(root / "openpi"), "rev-parse", "HEAD"], text=True
        ).strip(),
        "passed": False,
        "expected_action_shape": list(expected_shape),
    }
    server = None
    client = None
    server_log = root / "logs/policy-server.log"
    try:
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        with server_log.open("w") as log:
            server = subprocess.Popen(
                [sys.executable, "-u", str(Path(__file__).resolve()), "serve", "--root", str(root),
                 "--checkpoint", str(checkpoint), "--port", str(port)], stdout=log, stderr=subprocess.STDOUT
            )
        print(f"Loading policy; server PID={server.pid}; log={server_log}", flush=True)
        ready_deadline = time.monotonic() + 600
        while True:
            if server.poll() is not None:
                raise RuntimeError(f"Policy server exited {server.returncode}; see {server_log}")
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/healthz", timeout=1):
                    break
            except OSError:
                if time.monotonic() >= ready_deadline:
                    raise TimeoutError("Policy server did not start in 10 minutes")
                time.sleep(2)
        client = WebsocketClientPolicy(host="127.0.0.1", port=port)
        result["server_metadata"] = client.get_server_metadata()
        np.random.seed(0)
        samples = []
        for index in range(steps + 2):
            started = time.perf_counter()
            response = client.infer(make_droid_example())
            elapsed_ms = (time.perf_counter() - started) * 1000
            actions = np.asarray(response["actions"])
            if actions.shape != expected_shape or not np.isfinite(actions).all():
                raise RuntimeError(f"Unexpected DROID output: {actions.shape}, finite={np.isfinite(actions).all()}")
            sample = {
                "iteration": index, "warmup": index < 2, "client_ms": elapsed_ms,
                "shape": list(actions.shape), "finite": True,
                "server_timing": response.get("server_timing", {}),
                "policy_timing": response.get("policy_timing", {}),
            }
            samples.append(sample)
            print(json.dumps(sample), flush=True)
        measured = [sample["client_ms"] for sample in samples if not sample["warmup"]]
        result.update({
            "passed": True, "samples": samples, "measured_requests": len(measured),
            "client_ms_mean": statistics.mean(measured),
            "client_ms_p50": float(np.quantile(measured, 0.5)),
            "client_ms_p95": float(np.quantile(measured, 0.95)),
            "gpu_at_end": subprocess.check_output(
                ["nvidia-smi", "--query-gpu=name,memory.used,memory.total,utilization.gpu", "--format=csv"],
                text=True,
            ).strip(),
        })
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        signal.alarm(0)
        if client is not None:
            # The pinned official wrapper has no public close method.
            client._ws.close()
        if server is not None and server.poll() is None:
            server.terminate()
            try:
                server.wait(timeout=20)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2, default=str) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("serve", "benchmark"))
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--port", type=int, default=18000)
    parser.add_argument("--steps", type=int, default=5)
    args = parser.parse_args()
    if args.checkpoint is None:
        parser.error("--checkpoint is required for serve/benchmark")
    if args.mode == "serve":
        serve(args.checkpoint, args.port)
    else:
        if args.steps < 1:
            parser.error("--steps must be positive")
        benchmark(args.root, args.checkpoint, args.steps)


if __name__ == "__main__":
    main()

"""Offline test of the full-trial runner with skye_task's own test double (no robot, ROS or SSH).

Run from the repository root:  python3 -m unittest discover -s chassis/tests -v
"""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "chassis" / "client"))

import run_task_with_navigation as runner  # noqa: E402
from gento_navigation import NavigationError  # noqa: E402
from test_sequence import Device  # noqa: E402  (skye_task's offline robot double)


class FakeNav:
    log = []

    def __init__(self, fail_on=None, **kwargs):
        self.fail_on, self.active, self.count = fail_on, None, 0
        FakeNav.log.append(("init", kwargs.get("gaps")))

    def start(self, destination):
        FakeNav.log.append(("start", destination)); self.active, self.count = destination, 0

    def is_arrived_and_stopped(self):
        self.count += 1
        return self.count > 2 and self.active != self.fail_on

    def raise_if_failed(self):
        if self.active == self.fail_on and self.count > 1:
            raise NavigationError("ABORT: blocked")

    def start_undock(self):
        FakeNav.log.append(("start", "undock")); self.active, self.count = "undock", 0

    def job_status(self):
        return {"state": "done", "job": self.active, "final": "FINAL {}"}

    def pose(self):
        return [0.0, 0.0, 0.0]

    def stop(self):
        FakeNav.log.append(("stop", self.active))

    def selfscan(self):
        FakeNav.log.append(("selfscan",)); return "ok"

    def close(self):
        pass


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg = Path(self.tmp.name) / "fast.json"
        self.cfg.write_text(json.dumps({"stable_seconds": 0.04, "start_timeout_seconds": 0.15,
                                        "execution_timeout_seconds": 0.5, "poll_seconds": 0.02}))
        self.result = Path(self.tmp.name) / "result.json"
        FakeNav.log = []

    def tearDown(self):
        self.tmp.cleanup()

    def run_trial(self, fail_on=None, extra=()):
        with mock.patch.object(runner, "GentoSDK", side_effect=lambda config: Device(config)), \
             mock.patch.object(runner, "GentoNavigation", side_effect=lambda **kw: FakeNav(fail_on=fail_on, **kw)), \
             mock.patch.object(runner, "wait_for_start", lambda seq: None):
            code = runner.main(["--config", str(self.cfg), "--result", str(self.result), "--poll", "0.01",
                                "--place-gap", "0.3", *extra])
        return code, json.loads(self.result.read_text(encoding="utf-8"))

    def test_full_trial_order(self):
        code, report = self.run_trial(extra=["--selfscan"])
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["navigation"], "arrived")
        self.assertEqual([l["destination"] for l in report["metadata"]["navigation_legs"]], ["T", "D", "undock", "T"])
        self.assertEqual(FakeNav.log, [("init", {"D": 0.3}), ("selfscan",), ("start", "T"), ("start", "D"),
                                       ("start", "undock"), ("start", "T")])
        names = [s["name"] for s in report["steps"]]
        self.assertLess(names.index("receive_lift"), names.index("forward"))   # carry pose held while driving
        back = [s["name"] for s in report["metadata"]["return_steps"]]
        self.assertEqual(back[-2:], ["handover_and_close", "awaiting_task_start"])   # carry pose again before driving back

    def test_no_return(self):
        code, report = self.run_trial(extra=["--no-return"])
        self.assertEqual(code, 0)
        self.assertEqual([l["destination"] for l in report["metadata"]["navigation_legs"]], ["T", "D"])

    def test_failure_on_the_way_back_keeps_the_placement_result(self):
        code, report = self.run_trial(fail_on="undock")
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["metadata"]["return_status"], "failed")
        self.assertIn(("stop", "undock"), FakeNav.log)

    def test_failure_at_the_place_station_stops_chassis_and_arms_without_placing(self):
        code, report = self.run_trial(fail_on="D")
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "failed")
        self.assertIn(("stop", "D"), FakeNav.log)
        self.assertNotIn("forward", [s["name"] for s in report["steps"]])
        self.assertEqual(report["metadata"]["navigation_legs"][-1]["status"], "failed")


if __name__ == "__main__":
    unittest.main()

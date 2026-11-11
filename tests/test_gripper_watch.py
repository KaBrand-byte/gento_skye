"""Live angle monitor tests; SDK and motor communication are replaced offline."""

from contextlib import redirect_stderr, redirect_stdout
import csv
import importlib.util
from io import StringIO
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from skye_task.sdk import MotionError


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("gripper_watch", ROOT / "scripts/gripper_watch.py")
watch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(watch)


def sample(left=.5, right=.7):
    return {"feedback": {
        side: {"position_rad": angle, "velocity_rad_s": 0, "torque_nm": 0, "status": 0}
        for side, angle in (("left", left), ("right", right))}}


class WatchTests(unittest.TestCase):
    def setUp(self):
        connection_patch = patch.object(watch, "GentoTerminalConnection")
        driver_patch = patch.object(watch, "DMGripper")
        self.connection_factory = connection_patch.start()
        self.driver_factory = driver_patch.start()
        self.addCleanup(connection_patch.stop)
        self.addCleanup(driver_patch.stop)
        self.connection = self.connection_factory.return_value
        self.driver = self.driver_factory.return_value
        self.driver.snapshot.return_value = sample()
        self.output, self.errors = StringIO(), StringIO()

    def run_watch(self, args):
        with redirect_stdout(self.output), redirect_stderr(self.errors):
            return watch.main(args)

    def test_continuous_samples_follow_manual_changes_and_only_disable(self):
        self.driver.snapshot.side_effect = [sample(.5, .7), sample(.8, .4)]
        self.assertEqual(self.run_watch(["--samples", "2", "--rate-hz", "50"]), 0)
        output = self.output.getvalue()
        self.assertIn("左 +0.500000 rad", output)
        self.assertIn("左 +0.800000 rad", output)
        self.assertIn("右 +0.400000 rad", output)
        self.assertEqual(output.count("可空载手动开合"), 1)
        self.assertEqual(self.driver.disable.call_count, 2)
        self.driver.enable.assert_not_called()
        self.driver.step.assert_not_called()
        self.driver.move_to.assert_not_called()
        self.driver.close.assert_called_once()
        self.connection.close.assert_called_once()

    def test_single_side_only_displays_the_selected_motor(self):
        self.assertEqual(self.run_watch(["--side", "right", "--samples", "1"]), 0)
        self.assertEqual(self.driver_factory.call_args.args[1], "right")
        self.assertIn("右 +0.700000 rad", self.output.getvalue())
        self.assertNotIn("左 +", self.output.getvalue())

    def test_lost_feedback_is_failure_and_cached_angle_is_not_reprinted(self):
        self.driver.disable.side_effect = [None, MotionError("无法确认物理失能")]
        self.assertEqual(self.run_watch(["--samples", "2", "--rate-hz", "50"]), 1)
        self.assertEqual(self.output.getvalue().count("左 +0.500000 rad"), 1)
        self.assertIn("无法确认物理失能", self.errors.getvalue())
        self.driver.snapshot.assert_called_once()
        self.driver.close.assert_called_once()
        self.connection.close.assert_called_once()

    def test_no_off_confirmation_does_not_invite_manual_movement(self):
        self.driver.disable.side_effect = MotionError("Off timeout")
        self.assertEqual(self.run_watch(["--samples", "1"]), 1)
        self.assertNotIn("可空载手动开合", self.output.getvalue())
        self.driver.snapshot.assert_not_called()
        self.driver.enable.assert_not_called()

    def test_connection_failure_is_reported_and_closed(self):
        self.connection.connect.side_effect = MotionError("no link")
        self.assertEqual(self.run_watch(["--samples", "1"]), 1)
        self.assertIn("no link", self.errors.getvalue())
        self.driver_factory.assert_not_called()
        self.connection.close.assert_called_once()

    def test_connection_is_closed_even_if_motor_close_fails(self):
        self.driver.close.side_effect = MotionError("close failed")
        self.assertEqual(self.run_watch(["--samples", "1"]), 1)
        self.connection.close.assert_called_once()

    def test_csv_records_actual_angles_with_units_and_side(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "angles.csv"
            self.assertEqual(self.run_watch(["--samples", "1", "--csv", str(path)]), 0)
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual([row["side"] for row in rows], ["left", "right"])
        self.assertEqual(float(rows[0]["position_rad"]), .5)
        self.assertAlmostEqual(float(rows[0]["position_deg"]), 28.64788976)
        self.assertEqual(rows[0]["status"], "0")

    def test_arguments_are_validated_before_any_connection(self):
        for args in (["--rate-hz", "nan"], ["--rate-hz", "0"], ["--rate-hz", "51"],
                     ["--samples", "0"], ["--samples", "-1"]):
            with self.subTest(args=args), self.assertRaises(SystemExit):
                self.run_watch(args)
        self.connection_factory.assert_not_called()
        self.driver_factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()

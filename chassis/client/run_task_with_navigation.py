"""Full trial: skye_task arm sequence + chassis navigation, WITHOUT modifying skye_task.

Order (run from the repository root on the machine that runs skye_task, ROS env loaded):
  1. skye_task Sequence.prepare()      arms -> ready -> grippers open -> transition -> receive/carry pose
                                       (lift heights from the config)
  2. [--selfscan]                      lidar measures arms (+tray) on the chassis (needs ~1 m free in front)
  3. navigate to the start station     (default T, the handover point)            <- chassis moves
  4. handover                          press Enter: the grippers close (and stay checked while driving)
  5. navigate to the place station     (default D, the conveyor)                  <- chassis moves
  6. place                            skye_task's own motion steps: forward, down, open, up, and back by the
                                       forward distance - i.e. straight back into the carry pose (no detour
                                       over transition / ready, no second prepare)
  7. return (unless --no-return)       reverse straight out of the place station, turn, dock at the start
                                       station again: arms in the carry pose, grippers open - ready for the
                                       next tray.  A new run started like this does not move the arms at all.

While the chassis drives, the arm/lift/waist feedback is checked continuously through the sequence's own
`checked_feedback()`; Ctrl-C / SIGTERM, an arm deviation, a chassis failure or a timeout stops BOTH the
chassis and the arms.  The chassis only reports "arrived" after its docking job (approach + re-measure)
finished; for a place station it has also checked that the place motion (forward/down/sideways) is clear.

    python3 chassis/client/run_task_with_navigation.py --config config/default.json --selfscan
"""

import argparse
from datetime import datetime, timezone
import logging
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
LINK_RELEASE_S = 2.0     # the controller refuses a new link (-22) for ~1 s after the previous one was closed
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from gento_navigation import GentoNavigation, NavigationError  # noqa: E402
from skye_task.__main__ import wait_for_start, write_report  # noqa: E402
from skye_task.config import load_config  # noqa: E402
from skye_task.interfaces import GripperAction, GripperTarget  # noqa: E402
from skye_task.sdk import GentoSDK, MotionError  # noqa: E402
from skye_task.sequence import FIXED_AXES, Result, Sequence  # noqa: E402


class Trial:
    def __init__(self, args):
        self.args = args
        self.legs = []
        self.navigating = False
        self.nav = None
        self.sequence = None
        self.gripper = None
        self._sdk = None

    @staticmethod
    def arm_start(config):
        """Where the arms are before anything moves: "carry" (receive pose: start without arm motion), "prepare"
        (ready / transition pose: skye_task's prepare()).  Anything else stops: his prepare() first moves the
        arms STRAIGHT to the ready pose, and from the receive side that made the arms collide (2026-11-12)."""
        sdk = GentoSDK(config)
        try:
            sdk.connect()
            joints = sdk.read().joints
        finally:
            sdk.close()
            time.sleep(LINK_RELEASE_S)

        def near(pose, tol=5.0):
            return all(abs(a - b) <= tol for arm, target in zip(joints, pose) for a, b in zip(arm, target))
        if near(config.receive, 1.0):
            return "carry"
        if near(config.ready) or near(config.transition):
            return "prepare"
        raise MotionError("双臂既不在运输（接盘）位，也不在准备位或过渡位；prepare() 会直接回准备位（曾导致双臂相撞）。"
                          "请先经过渡位把双臂移回准备位。"
                          f"当前关节: {[[round(v, 1) for v in arm] for arm in joints]}")

    def attach_at_carry(self):
        """The arms already stand in the carry (receive) pose (e.g. left there by the previous run): do what
        skye_task's prepare() does to connect and enable - without any arm or lift motion - and open the grippers
        for the handover."""
        seq, sdk, gripper = self.sequence, self._sdk, self.gripper
        seq.begin = time.monotonic()
        seq._checkpoint()
        if gripper is not None:
            gripper.reserve()
        seq.result.metadata = sdk.connect()
        feedback = sdk.read()
        seq._guard(feedback, require_position=False)
        if any(state >= 3 for state in feedback.trajectory_states):
            raise MotionError("已有双臂轨迹正在执行，需人工处理")
        if any(not seq._near(a, t) for a, t in zip(feedback.joints, seq.config.receive)):
            raise MotionError("双臂已不在运输（接盘）位")
        seq.fixed = {name: list(feedback.rt[name]["fb_pos"][:size]) for name, size in FIXED_AXES.items()}
        sdk.enter_position(feedback, seq._checkpoint)
        seq.result.metadata["tool_xyzabc_mm_deg"] = sdk.tools
        seq._record("initialized", status="completed", note="arms already in the carry pose: no arm motion")
        if gripper is not None:
            gripper.connect()
            seq._gripper_action("open_before_receive", GripperAction.OPEN)
        seq.prepared = True
        seq._record("awaiting_task_start", status="waiting")

    def place_and_back_to_carry(self):
        """Place with skye_task's own motion steps - forward, down, open, up - then back by the forward distance,
        i.e. straight back into the carry pose along the path just taken (his start() ends at the ready pose via
        the transition pose instead, and a second prepare() would follow).  Grippers stay open."""
        seq, c = self.sequence, self.sequence.config
        feedback = seq.checked_feedback()
        if any(not seq._near(a, t) for a, t in zip(feedback.joints, c.receive)):
            raise MotionError("放置前双臂已偏离运输（接盘）位")
        if any(feedback.trajectory_states):
            raise MotionError("放置前双臂轨迹尚未空闲")
        seq.started = True
        seq._move_linear("forward", [[c.forward_mm, 0, 0]] * 2, c.forward)
        seq._move_linear("down", [[0, 0, -c.down_mm]] * 2, c.down)
        if self.gripper is not None:
            seq._gripper_action("release", GripperAction.OPEN)
        else:
            seq._record("release", status="skipped", reason="配置关闭夹爪")
        seq._move_linear("up", [[0, 0, c.up_mm]] * 2, c.up)
        seq._move_linear("back_to_carry", [[-c.forward_mm, 0, 0]] * 2, c.forward)
        feedback = seq.checked_feedback()
        if any(not seq._near(a, t) for a, t in zip(feedback.joints, c.receive)):
            seq._move_joint("carry", c.receive)          # tiny correction if the planner ended a little off
        seq.result.motion_completed = True
        if self.gripper is not None:
            seq.result.gripper_actions = "completed"
        seq.result.status = "completed"
        seq.result.finished_at = datetime.now(timezone.utc).isoformat()
        seq._record("finished", status="completed", grasped=None, placed=None)

    def make_sequence(self, sdk, config, label=""):
        """His Sequence with the grippers driven the same way skye_task's own `run` does."""
        self.gripper = None
        if config.gripper_enabled:
            from skye_task.dm_gripper import load_gripper_config
            from skye_task.task_gripper import TaskGripper
            self.gripper = TaskGripper(sdk, load_gripper_config(self.args.gripper_config))
        self._sdk = sdk
        return Sequence(sdk, config, gripper=self.gripper,
                        on_event=lambda name, detail: logging.info("%s%s %s", label, name, detail))

    def close_grippers(self):
        """Handover at the start station: close both grippers now, so the tray is held while driving (his
        start() closes them again at the place station, which is then a no-op)."""
        seq, gripper = self.sequence, self.gripper
        if gripper is None:
            return
        logging.info("交接：双爪闭合")
        gripper.control(GripperTarget.BOTH, GripperAction.CLOSE, seq.checked_feedback)
        until = time.monotonic() + seq.config.handover_wait_seconds
        while time.monotonic() < until:
            seq.checked_feedback()
            time.sleep(seq.config.poll_seconds)
        logging.info("双爪已闭合，托盘已交接")

    def release_hardware(self):
        """Gripper worker first, then the arm SDK link (same order as skye_task's run)."""
        gripper, sdk, self.gripper, self._sdk = self.gripper, self._sdk, None, None
        sdk_can_close = True
        if gripper is not None:
            try:
                self.sequence.result.metadata.setdefault("gripper_final", gripper.snapshot())
            except Exception as error:
                logging.error("夹爪状态读取失败: %s", error)
            try:
                gripper.close()
            except Exception as error:
                logging.error("夹爪资源释放失败: %s", error)
            sdk_can_close = not gripper.worker_running
        if sdk is not None and sdk_can_close:
            try:
                sdk.close()
            except Exception as error:
                logging.error("SDK 资源释放失败: %s", error)
            time.sleep(LINK_RELEASE_S)

    def navigate(self, destination, undock=False):
        """Drive to `destination` (or reverse out with undock=True); keep checking the arms; return only after
        docked and standing still."""
        nav, seq, begin = self.nav, self.sequence, time.monotonic()
        leg = {"destination": destination, "status": "running"}
        self.legs.append(leg)
        logging.info("导航开始: %s", destination)
        self.navigating = True
        nav.start_undock() if undock else nav.start(destination)
        while True:
            seq.checked_feedback()                      # stop request + arm/lift/waist feedback unchanged
            if nav.is_arrived_and_stopped():
                break
            nav.raise_if_failed()
            if time.monotonic() - begin > self.args.nav_timeout:
                raise MotionError(f"导航超时（{self.args.nav_timeout} s）: {destination}")
            time.sleep(self.args.poll)
        self.navigating = False
        feedback = seq.checked_feedback()
        if not undock and any(not seq._near(actual, target) for actual, target in zip(feedback.joints, seq.config.receive)):
            raise MotionError(f"导航到 {destination} 后双臂已偏离运输（接盘）姿态")
        status = nav.job_status()
        leg.update(status="arrived", seconds=round(time.monotonic() - begin, 1), final=status.get("final"),
                   pose=nav.pose())
        logging.info("已到站并停稳: %s %s", destination, status.get("final"))

    def stop_chassis(self):
        if self.navigating and self.nav is not None:
            try:
                self.nav.stop()
            except Exception as error:                  # keep the original failure
                logging.error("底盘停止失败: %s", error)
                return f"底盘停止失败: {error}"
            finally:
                self.navigating = False
        return ""

    def return_to_start(self):
        """After placing (arms already back in the carry pose): reverse out along the approach, then dock at the
        start station again - the chassis job turns where turning is clear."""
        self.navigate("undock", undock=True)
        self.navigate(self.args.start)

    def run(self):
        a = self.args
        config = load_config(a.config)
        gaps = {k: v for k, v in ((a.start, a.start_gap), (a.place, a.place_gap)) if v is not None}
        self.nav = GentoNavigation(transport=a.transport, ssh_target=a.ssh_target, gaps=gaps)
        sdk = GentoSDK(config)
        self.sequence = self.make_sequence(sdk, config)
        result = self.sequence.result
        old = {s: signal.signal(s, lambda *_: self.sequence.request_stop()) for s in (signal.SIGINT, signal.SIGTERM)}
        exit_code = 1
        try:
            if self.arm_start(config) == "carry":           # before any motion
                logging.info("双臂已在运输（接盘）位：不动手臂，直接接入")
                self.attach_at_carry()
            else:
                self.sequence.prepare()                     # arms into the carry (receive) pose
            if a.selfscan:
                logging.info("selfscan: %s", self.nav.selfscan())
            if a.start:
                self.navigate(a.start)
            print("已在起始点。放好托盘后按 Enter：双爪闭合，然后出发去放置点；Ctrl-C 停止。", flush=True)
            wait_for_start(self.sequence)
            self.close_grippers()
            self.navigate(a.place)
            self.place_and_back_to_carry()                  # forward, down, open, up, back -> carry pose
            if a.start and not a.no_return:
                self.return_to_start()
            exit_code = 0
        except (Exception, KeyboardInterrupt) as error:
            logging.error("流程停止: %s", error)
            chassis_error = self.stop_chassis()
            if result.status == "completed":                # placed; the failure was on the way back
                result.metadata["return_status"] = "stopped" if isinstance(error, KeyboardInterrupt) else "failed"
                result.metadata.setdefault("return_error", str(error))
            else:
                self.sequence.fail(error)
            if chassis_error:
                result.stop_error = "; ".join(x for x in (result.stop_error, chassis_error) if x)
            for leg in self.legs:
                if leg["status"] == "running":
                    leg["status"] = "stopped" if isinstance(error, KeyboardInterrupt) or "停止" in str(error) else "failed"
            exit_code = 130 if result.status == "stopped" or isinstance(error, KeyboardInterrupt) else 1
        finally:
            self.release_hardware()
            try:
                self.nav.close()
            except Exception:
                pass
            for s, handler in old.items():
                signal.signal(s, handler)
            result.navigation = "arrived" if self.legs and all(l["status"] == "arrived" for l in self.legs) else \
                ("failed" if self.legs else "not_called")
            result.metadata["navigation_legs"] = self.legs
            result.metadata["navigation_note"] = "导航由 chassis/client/run_task_with_navigation.py 在 Sequence 外部执行"
            if not result.finished_at:
                result.finished_at = datetime.now(timezone.utc).isoformat()
            a.result.parent.mkdir(parents=True, exist_ok=True)
            write_report(a.result, result.to_dict())
            print(f"执行结果: {a.result} ({result.status})")
        return exit_code


def main(argv=None):
    p = argparse.ArgumentParser(description="skye_task 双臂流程 + 底盘导航 完整试运行（不修改 skye_task）")
    p.add_argument("--config", type=Path, help="skye_task JSON 配置")
    p.add_argument("--result", type=Path, default=Path("runs/with-navigation.json"))
    p.add_argument("--start", default="T", help="起始（交接）站点；空字符串表示不导航到起始点")
    p.add_argument("--place", default="D", help="放置站点")
    p.add_argument("--start-gap", type=float, help="起始站点间隙 m（默认用站点配置）")
    p.add_argument("--place-gap", type=float, help="放置站点间隙 m（默认用站点配置）")
    p.add_argument("--transport", choices=("ros", "ssh"), default="ros")
    p.add_argument("--ssh-target", default="chassis")
    p.add_argument("--gripper-config", type=Path, default=ROOT / "config" / "gripper-can.json",
                   help="夹爪配置（与 skye_task run 相同）")
    p.add_argument("--no-return", action="store_true", help="放置后不返回起始点")
    p.add_argument("--selfscan", action="store_true", help="准备完成后先测手臂／托盘占位（车前约 1 m 需无物体）")
    p.add_argument("--nav-timeout", type=float, default=600.0)
    p.add_argument("--poll", type=float, default=0.5)
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    try:
        return Trial(args).run()
    except (NavigationError, MotionError, OSError, ValueError) as error:
        logging.error("%s", error)
        return 1


if __name__ == "__main__":
    sys.exit(main())

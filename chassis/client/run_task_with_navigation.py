"""Full trial: skye_task arm sequence + chassis navigation, WITHOUT modifying skye_task.

Order (run from the repository root on the machine that runs skye_task, ROS env loaded):
  1. skye_task Sequence.prepare()      arms -> ready (lift 500) -> receive/carry pose (lift 450)
  2. [--selfscan]                      lidar measures arms (+tray) on the chassis (needs ~1 m free in front)
  3. navigate to the start station     (default T, the table)                     <- chassis moves
  4. handover                          press Enter (grippers not integrated yet: unloaded run)
  5. navigate to the place station     (default D, the conveyor)                  <- chassis moves
  6. skye_task Sequence.start()        forward 100 mm, down 100 mm, hands out, back to ready

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
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from gento_navigation import GentoNavigation, NavigationError  # noqa: E402
from skye_task.__main__ import wait_for_start, write_report  # noqa: E402
from skye_task.config import load_config  # noqa: E402
from skye_task.sdk import GentoSDK, MotionError  # noqa: E402
from skye_task.sequence import Result, Sequence  # noqa: E402


class Trial:
    def __init__(self, args):
        self.args = args
        self.legs = []
        self.navigating = False
        self.nav = None
        self.sequence = None

    def navigate(self, destination):
        """Drive to `destination`; keep checking the arms; return only after docked and standing still."""
        nav, seq, begin = self.nav, self.sequence, time.monotonic()
        leg = {"destination": destination, "status": "running"}
        self.legs.append(leg)
        logging.info("导航开始: %s", destination)
        self.navigating = True
        nav.start(destination)
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
        if any(not seq._near(actual, target) for actual, target in zip(feedback.joints, seq.config.receive)):
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

    def run(self):
        a = self.args
        config = load_config(a.config)
        gaps = {k: v for k, v in ((a.start, a.start_gap), (a.place, a.place_gap)) if v is not None}
        self.nav = GentoNavigation(transport=a.transport, ssh_target=a.ssh_target, gaps=gaps)
        sdk = GentoSDK(config)
        self.sequence = Sequence(sdk, config, on_event=lambda name, detail: logging.info("%s %s", name, detail))
        result = self.sequence.result
        old = {s: signal.signal(s, lambda *_: self.sequence.request_stop()) for s in (signal.SIGINT, signal.SIGTERM)}
        exit_code = 1
        try:
            self.sequence.prepare()                         # arms into the carry (receive) pose, lift 450
            if a.selfscan:
                logging.info("selfscan: %s", self.nav.selfscan())
            if a.start:
                self.navigate(a.start)
            print("已在起始点。交接托盘后按 Enter 出发去放置点（夹爪未接入：空载），Ctrl-C 停止。", flush=True)
            wait_for_start(self.sequence)
            self.navigate(a.place)
            self.sequence.start()                           # forward, down, hands out, back to ready
            exit_code = 0
        except (Exception, KeyboardInterrupt) as error:
            logging.error("流程停止: %s", error)
            chassis_error = self.stop_chassis()
            self.sequence.fail(error)
            if chassis_error:
                result.stop_error = "; ".join(x for x in (result.stop_error, chassis_error) if x)
            for leg in self.legs:
                if leg["status"] == "running":
                    leg["status"] = "stopped" if isinstance(error, KeyboardInterrupt) or "停止" in str(error) else "failed"
            exit_code = 130 if result.status == "stopped" else 1
        finally:
            try:
                sdk.close()
            except Exception as error:
                logging.error("SDK 资源释放失败: %s", error)
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

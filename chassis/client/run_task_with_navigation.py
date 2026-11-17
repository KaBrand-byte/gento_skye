"""Full trial: skye_task arm sequence + chassis navigation, WITHOUT modifying skye_task.

Order (run from the repository root on the machine that runs skye_task, ROS env loaded):
  1. skye_task Sequence.prepare()      arms -> ready -> grippers open -> transition -> receive/carry pose
                                       (lift heights from the config)
  2. [--selfscan]                      lidar measures arms (+tray) on the chassis (needs ~1 m free in front)
  3. handover, decoupled from driving  where the robot stands (normally at T), arms not moving: the tray is
                                       put between the grippers, Enter closes them, then the operator answers
                                       1 = held right, go;  0 = not right: the grippers open, put it right,
                                       Enter closes again.  o / c open / close by hand at any time.
                                       (--holding: no prompt; --handover: drive to T and open first)
  4. navigate to the place station     (default D, the conveyor)                  <- chassis moves
  5. place                            skye_task's own motion steps: forward, down, open, up, and back by the
                                       forward distance - i.e. straight back into the carry pose (no detour
                                       over transition / ready, no second prepare)
  6. return (unless --no-return)       reverse straight out of the place station, turn, dock at the start
                                       station again: arms in the carry pose, grippers open - ready for the
                                       next tray.  A new run started like this does not move the arms at all.

While the chassis drives, the arm/lift/waist feedback is checked continuously through the sequence's own
`checked_feedback()`; Ctrl-C / SIGTERM, an arm deviation, a chassis failure or a timeout stops BOTH the
chassis and the arms.  The chassis only reports "arrived" after its docking job (approach + re-measure)
finished; for a place station it has also checked that the place motion (forward/down/sideways) is clear.

    python3 chassis/client/run_task_with_navigation.py --config config/default.json --selfscan
"""

import argparse
from copy import deepcopy
import dataclasses
from datetime import datetime, timezone
import logging
import math
import os
from pathlib import Path
import select
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
LINK_RELEASE_S = 2.0
RANDOM_ROUTE_JITTER = 0.08  # m: --random-route, random spot within this of the taught safe points
BLOCK_RETRIES = 5          # an object blocking the docking: ask to clear it and retry this often
MAX_LIFT_ASSIST_MM = 60.0  # at most this much of the place descent is done by the lift instead of the arms
PLACE_DOWN_MM = 135.0      # total place descent (arms + lift).  Basket since 2026-11-17: the tray's 185 mm
                           # (arms 135 + lift 50) pressed the basket into the belt and pushed the arms up -> 5 cm less
PLACE_LIFT_MM = 0.0        # the lift lowers at least this much after the arms (tray: 50; basket: none needed)
WRIST_ADJUST_DEG = 10.0    # arms in an older carry pose where only the two wrist joints differ by up to this: turn
                           # just the wrists to the configured carry pose (receive changed 2026-11-17)
FINISH_TOL_MM = 40.0       # --finish-place: up + back must end within this of the carry pose (tool position)
ARM_MARGIN_MM = 10.0     # stay this far inside the arms' planning limit (elbow not near straight)     # the controller refuses a new link (-22) for ~1 s after the previous one was closed
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from gento_navigation import GentoNavigation, NavigationError  # noqa: E402
from skye_task.__main__ import wait_for_start, write_report  # noqa: E402
from skye_task.config import load_config  # noqa: E402
from skye_task.geometry import local_displacement, translated  # noqa: E402
from skye_task.interfaces import GripperAction, GripperTarget  # noqa: E402
from skye_task.sdk import GentoSDK, MotionError  # noqa: E402
from skye_task.sequence import FIXED_AXES, Result, Sequence, Stopped  # noqa: E402


def read_answer(sequence):
    """One typed line (stripped, lower case); the arm / lift / gripper feedback stays checked while waiting."""
    while True:
        sequence.checked_feedback()
        readable, _, _ = select.select([sys.stdin], [], [], sequence.config.poll_seconds)
        if readable:
            line = sys.stdin.readline()
            if not line:
                raise Stopped("标准输入关闭")
            return line.strip().lower()


def banner(*lines):
    bar = "=" * 60
    print(chr(7)); print(bar)                               # terminal bell
    for line in lines:
        print(f"  {line}")
    print(bar, flush=True)


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
        if all(abs(a - b) <= (WRIST_ADJUST_DEG if j >= 5 else 1.0)
               for arm, target in zip(joints, config.receive) for j, (a, b) in enumerate(zip(arm, target))):
            return "carry_wrist"                            # an older carry pose: only the wrists differ
        if near(config.ready) or near(config.transition):
            return "prepare"
        raise MotionError("双臂既不在运输（接盘）位，也不在准备位或过渡位；prepare() 会直接回准备位（曾导致双臂相撞）。"
                          "请先经过渡位把双臂移回准备位。"
                          f"当前关节: {[[round(v, 1) for v in arm] for arm in joints]}")

    def _finish_plan(self, arm_up):
        """Read-only: from the arms' current pose, where do up(arm_up) + back(forward) end?  None if within
        FINISH_TOL_MM of the carry pose (tool position, both arms), else the reason."""
        sdk, c = self._sdk, self.sequence.config
        predicted = deepcopy(sdk.read())
        shoulders = sdk.shoulders(predicted)
        ends = [sdk.forward(i, predicted.joints[i]) for i in range(2)]      # tool poses, moved along below
        for name, deltas, motion in (("up", [[0, 0, arm_up]] * 2, c.up),
                                     ("back_to_carry", [[-c.forward_mm, 0, 0]] * 2, c.forward)):
            starts = [sdk.forward(i, predicted.joints[i]) for i in range(2)]
            targets = [translated(starts[i], local_displacement(shoulders[i], deltas[i])) for i in range(2)]
            ends = [translated(ends[i], local_displacement(shoulders[i], deltas[i])) for i in range(2)]
            try:
                trajectories = sdk.plan_linear(predicted, starts, targets, motion)
            except Exception as error:
                return f"{name}: {error}"
            for i, trajectory in enumerate(trajectories):
                predicted.rt["arms"][i]["fb"]["fb_pos"] = trajectory.endpoint
        off = []
        for i in range(2):                                   # where the straight moves end vs the carry pose
            carry = sdk.forward(i, c.receive[i])
            off.append(math.dist([ends[i][k][3] for k in range(3)], [carry[k][3] for k in range(3)]))
        self.finish_offsets_mm = [round(d, 1) for d in off]
        if max(off) > FINISH_TOL_MM:
            return f"上移 {arm_up:.0f} mm、后移 {c.forward_mm} mm 后离运输位 {self.finish_offsets_mm} mm（允许 {FINISH_TOL_MM:.0f}）"
        return None

    def finish_interrupted_place(self, arm_up):
        """A place that stopped with the arms down at the place station (e.g. the basket pressed against the
        belt, the arms pushed up: "升降调整期间双臂未保持静止"): release, lift back to the carry height, arms up by
        `arm_up`, back by the forward distance -> carry pose.  Planned read-only first and refused unless that
        ends near the carry pose.  The release does not require the arms to stand still: freed from the
        basket they spring back to where they were commanded."""
        seq, c = self.sequence, self.sequence.config
        self.attach_at_carry(anywhere=True)
        why = self._finish_plan(arm_up)
        if why:
            raise MotionError(f"收尾动作只读规划不通过（松爪前）：{why}")
        logging.info("收尾规划通过（松爪前）：上移 %.0f、后移 %s 后离运输位 %s mm", arm_up, c.forward_mm, self.finish_offsets_mm)
        seq.started = True
        if self.gripper is not None:
            logging.info("松开夹爪（双臂可能回弹到指令位置）")
            seq._record("release", status="started", note="finish an interrupted place")
            self.gripper.control(GripperTarget.BOTH, GripperAction.OPEN, seq._checkpoint)
            seq._record("release", status="completed")
        until, still_since = time.monotonic() + 10.0, None
        while True:                                          # wait for the arms to stand still
            feedback = seq.checked_feedback()
            still = not any(abs(v) > c.joint_velocity_tolerance_deg_s for arm in feedback.rt["arms"]
                            for v in arm["fb"]["fb_vel"])
            still_since = (still_since or time.monotonic()) if still else None
            if still_since and time.monotonic() - still_since >= 1.0:
                break
            if time.monotonic() > until:
                raise MotionError("松爪后双臂 10 s 内未静止")
            time.sleep(c.poll_seconds)
        why = self._finish_plan(arm_up)
        if why:
            raise MotionError(f"收尾动作只读规划不通过（松爪后）：{why}")
        logging.info("收尾规划通过（松爪后）：离运输位 %s mm", self.finish_offsets_mm)
        lift = seq.checked_feedback().rt["lift"]["fb_pos"][0]
        if abs(lift - c.receive_lift[0]) > c.fixed_lift_tolerance_mm:
            seq._move_lift("place_lift_up", c.receive_lift[0])
        seq._move_linear("up", [[0, 0, arm_up]] * 2, c.up)
        seq._move_linear("back_to_carry", [[-c.forward_mm, 0, 0]] * 2, c.forward)
        feedback = seq.checked_feedback()
        if any(not seq._near(a, t) for a, t in zip(feedback.joints, c.receive)):
            seq._move_joint("carry", c.receive)
        seq.result.motion_completed = True
        seq.result.status = "completed"
        seq.result.finished_at = datetime.now(timezone.utc).isoformat()
        seq._record("finished", status="completed", note="interrupted place finished")

    def attach_at_carry(self, wrists=False, anywhere=False):
        """The arms already stand in the carry (receive) pose (e.g. left there by the previous run): do what
        skye_task's prepare() does to connect and enable - without any arm or lift motion.  The grippers are NOT
        opened here (a tray may still be held after a stop); a normal run opens them on arrival at the start
        station, --holding never opens them before the place station.
        wrists=True: an older carry pose where only the two wrist joints differ (the config's carry pose was
        changed, 2026-11-17): only the wrists turn to the configured pose."""
        seq, sdk, gripper = self.sequence, self._sdk, self.gripper
        seq.begin = time.monotonic()
        seq._checkpoint()
        if gripper is not None:
            gripper.reserve()
        seq.result.metadata = sdk.connect()
        feedback = sdk.read()
        if anywhere:                                        # the lift may stand below the carry height (stopped
            seq.fixed = {name: list(feedback.rt[name]["fb_pos"][:size])   # on its way down): its height now
                         for name, size in FIXED_AXES.items()}             # is what has to stay put
        seq._guard(feedback, require_position=False)
        if any(state >= 3 for state in feedback.trajectory_states):
            raise MotionError("已有双臂轨迹正在执行，需人工处理")
        if not (wrists or anywhere) and any(not seq._near(a, t) for a, t in zip(feedback.joints, seq.config.receive)):
            raise MotionError("双臂已不在运输（接盘）位")
        seq.fixed = {name: list(feedback.rt[name]["fb_pos"][:size]) for name, size in FIXED_AXES.items()}
        sdk.enter_position(feedback, seq._checkpoint)
        seq.result.metadata["tool_xyzabc_mm_deg"] = sdk.tools
        if wrists:
            target = seq.config.receive
            if any(abs(a - b) > (WRIST_ADJUST_DEG if j >= 5 else 1.0)
                   for arm, t in zip(feedback.joints, target) for j, (a, b) in enumerate(zip(arm, t))):
                raise MotionError("双臂已不在运输（接盘）位")
            logging.info("运输位姿态已更新：只转动腕部关节到新姿态 %s", [arm[5:] for arm in target])
            seq._move_joint("carry_wrists", target)
        seq._record("initialized", status="completed",
                    note="wrists turned to the new carry pose" if wrists else
                    "attached where the arms stand (finish place)" if anywhere else
                    "arms already in the carry pose: no arm motion")
        if gripper is not None:
            gripper.connect()                               # enables at the measured position: holds what it holds
        seq.prepared = True
        seq._record("awaiting_task_start", status="waiting")

    def place_segments(self):
        """Arm segments of the place motion; the lift part (if any) runs between "down" and "up"."""
        c = self.sequence.config
        plan = getattr(self, "place_plan", None) or {"arm_down": c.down_mm, "arm_up": c.up_mm, "lift_down": 0.0}
        return (("forward", [[c.forward_mm, 0, 0]] * 2, c.forward),
                ("down", [[0, 0, -plan["arm_down"]]] * 2, c.down),
                ("up", [[0, 0, plan["arm_up"]]] * 2, c.up),
                ("back_to_carry", [[-c.forward_mm, 0, 0]] * 2, c.forward))

    def _plans(self, arm_down, arm_up):
        """Read-only: can forward, down(arm_down), up(arm_up), back be planned from the current arm pose?"""
        sdk, c = self._sdk, self.sequence.config
        predicted = deepcopy(sdk.read())
        for name, deltas, motion in (("forward", [[c.forward_mm, 0, 0]] * 2, c.forward),
                                     ("down", [[0, 0, -arm_down]] * 2, c.down), ("up", [[0, 0, arm_up]] * 2, c.up),
                                     ("back_to_carry", [[-c.forward_mm, 0, 0]] * 2, c.forward)):
            starts = [sdk.forward(i, predicted.joints[i]) for i in range(2)]
            shoulders = sdk.shoulders(predicted)
            targets = [translated(starts[i], local_displacement(shoulders[i], deltas[i])) for i in range(2)]
            try:
                trajectories = sdk.plan_linear(predicted, starts, targets, motion)
            except Exception as error:
                return f"{name}: {error}"
            for i, trajectory in enumerate(trajectories):
                predicted.rt["arms"][i]["fb"]["fb_pos"] = trajectory.endpoint
        return None

    def preflight_place(self):
        """Read-only, from the carry pose the arms are in now and BEFORE the tray is handed over: plan the whole
        place motion.  If the configured descent is beyond the arms' reach (measured: forward 100 + down 150
        straightens the elbow - out of workspace), the arms go down as far as they comfortably can and the LIFT
        does the rest: the tray ends at the same place, set down instead of dropped."""
        seq, c = self.sequence, self.sequence.config
        down, up = float(c.down_mm), float(c.up_mm)
        why = self._plans(down, up)
        if why is None:
            self.place_plan = {"arm_down": down, "arm_up": up, "lift_down": 0.0}
        else:
            reach = next((d for d in (down - k for k in range(5, int(MAX_LIFT_ASSIST_MM + ARM_MARGIN_MM) + 1, 5))
                          if d > 0 and self._plans(d, d) is None), None)
            if reach is None or down - (reach - ARM_MARGIN_MM) > MAX_LIFT_ASSIST_MM:
                raise MotionError(f"放置动作只读规划失败（前伸 {c.forward_mm}、下降 {down} mm）：{why}；"
                                  f"升降最多分担 {MAX_LIFT_ASSIST_MM:.0f} mm 也不够")
            arm_down = reach - ARM_MARGIN_MM
            lift_down = down - arm_down
            if c.receive_lift[0] - lift_down < min(c.ready_lift[0], c.receive_lift[0]) - MAX_LIFT_ASSIST_MM:
                raise MotionError("升降分担后低于允许范围")
            self.place_plan = {"arm_down": arm_down, "arm_up": max(0.0, up - lift_down), "lift_down": lift_down}
            if self._plans(arm_down, self.place_plan["arm_up"]) is not None:
                raise MotionError(f"放置动作只读规划失败（升降分担 {lift_down:.0f} mm 后仍不可行）")
            logging.info("下降 %.0f mm 超出双臂可达（%s）：双臂下降 %.0f mm，其余 %.0f mm 由升降完成",
                         down, why, arm_down, lift_down)
        lift_min = float(self.args.place_lift_mm or 0.0)
        if self.place_plan["lift_down"] < lift_min:
            # the arms stay as planned (they come back up by what they went down); the lift goes lower
            if lift_min > MAX_LIFT_ASSIST_MM:
                raise MotionError(f"放置时升降下降 {lift_min:.0f} mm 超过允许的 {MAX_LIFT_ASSIST_MM:.0f} mm")
            self.place_plan["lift_down"] = lift_min
        self.place_plan["total_down"] = self.place_plan["arm_down"] + self.place_plan["lift_down"]
        seq.result.metadata["place_motion_mm"] = {"forward": c.forward_mm, "down": down, "up": up, **self.place_plan}
        logging.info("放置动作只读规划通过：前伸 %s，双臂下降 %.0f + 升降 %.0f = %.0f mm，松爪后升降回升、双臂上移 %.0f，再后移回运输位",
                     c.forward_mm, self.place_plan["arm_down"], self.place_plan["lift_down"], self.place_plan["total_down"],
                     self.place_plan["arm_up"])

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
        lift_down = (getattr(self, "place_plan", None) or {}).get("lift_down", 0.0)
        for name, deltas, motion in self.place_segments():
            seq._move_linear(name, deltas, motion)
            if name == "down":
                if lift_down:                               # the rest of the descent: the tray is set down
                    seq._move_lift("place_lift_down", c.receive_lift[0] - lift_down)
                if self.gripper is not None:
                    seq._gripper_action("release", GripperAction.OPEN)
                else:
                    seq._record("release", status="skipped", reason="配置关闭夹爪")
                if lift_down:
                    seq._move_lift("place_lift_up", c.receive_lift[0])
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
        """Close both grippers at handover and keep the command active during transport."""
        seq, gripper = self.sequence, self.gripper
        if gripper is None:
            return
        logging.info("交接：双爪闭合")
        gripper.control(GripperTarget.BOTH, GripperAction.CLOSE, seq.checked_feedback)
        until = time.monotonic() + seq.config.handover_wait_seconds
        while time.monotonic() < until:
            seq.checked_feedback()
            time.sleep(seq.config.poll_seconds)
        seq.result.metadata["gripper_handover"] = gripper.snapshot()
        logging.info("双爪已闭合，托盘已交接")

    def grip(self, action):
        """Manual gripper action ("o" / "c") at the current arm pose; the arms do not move."""
        if action == "c":
            self.close_grippers()
        else:
            self.sequence._gripper_action("manual_open", GripperAction.OPEN)
            logging.info("双爪已张开")

    def handover(self, title):
        """The tray is put between the grippers, Enter (or c) closes them; then the operator answers 1 = held
        right, go - or 0 = not right: the grippers open again, the tray is put right, Enter closes again.
        o / c open / close by hand at any time (arms do not move); 1 also takes a tray closed before the run
        (gento grip close)."""
        place = self.args.place
        log = self.sequence.result.metadata.setdefault("handover_feedback", [])
        if self.gripper is None:
            banner(title, f"按 Enter 出发去 {place}；Ctrl-C 停止。")
            while read_answer(self.sequence) not in ("", "1"):
                pass
            return
        closed = False
        while True:
            if closed:
                banner("夹好了吗？", f"1 = 夹好了，出发去 {place}   0 = 没夹好（张开夹爪，重新放）   o / c = 张开 / 闭合",
                       "Ctrl-C 停止。")
            else:
                banner(title, "把托盘放到双爪之间，按 Enter 闭合夹爪。",
                       f"Enter / c = 闭合   o = 张开   1 = 已经夹好（比如事先 gento grip close），出发去 {place}",
                       "Ctrl-C 停止。")
            while True:
                answer = read_answer(self.sequence)
                if answer == "c" or (answer == "" and not closed):
                    self.grip("c"); closed = True; log.append("close")
                elif answer in ("o", "0"):
                    self.grip("o"); closed = False; log.append(answer)
                elif answer == "1":
                    if not closed:                          # closed before the run: close here too, so the
                        self.grip("c"); log.append("close")  # grippers keep holding (hold torque) while driving
                    log.append("1"); logging.info("交接反馈 1：托盘已夹好")
                    return
                else:
                    print("  请输入：" + ("1 / 0 / o / c" if closed else "Enter / c / o / 1"), flush=True)
                    continue
                break

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
        retries, note = 0, None
        while True:
            seq.checked_feedback()                      # stop request + arm/lift/waist feedback unchanged
            if nav.is_arrived_and_stopped():
                break
            try:
                nav.raise_if_failed()
            except NavigationError as error:
                # an object in the way (the chassis already waited for it to go): ask to clear it, then retry
                if undock or "BLOCKED" not in str(error) or retries >= BLOCK_RETRIES or not sys.stdin.isatty():
                    raise
                retries += 1
                self.navigating = False
                bar = "=" * 60
                print(chr(7)); print(bar)
                print(f"  去 {destination} 的路上被挡住（底盘已等待）：{error}")
                print(f"  清除障碍后按 Enter 重试（第 {retries}/{BLOCK_RETRIES} 次）；Ctrl-C 停止。")
                print(bar, flush=True)
                wait_for_start(seq)
                logging.info("重试导航: %s", destination)
                self.navigating = True; begin = time.monotonic()
                nav.start(destination)
                continue
            text = getattr(nav, "note", lambda: None)()
            if text and text != note:
                logging.info("底盘: %s", text)          # e.g. WAITING: ... blocked ... waiting for it to clear
            note = text
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
        if a.down_mm is None:
            a.down_mm = PLACE_DOWN_MM                       # the basket's descent; skye_task's config stays as it is
        if a.down_mm is not None or a.up_mm is not None:      # this run only; the config file stays as it is
            down = config.down_mm if a.down_mm is None else a.down_mm
            config = dataclasses.replace(config, down_mm=down, up_mm=a.up_mm if a.up_mm is not None else down)
            logging.info("本次运行：下降 %s mm，上移 %s mm（配置文件不变）", config.down_mm, config.up_mm)
        gaps = {k: v for k, v in ((a.start, a.start_gap), (a.place, a.place_gap)) if v is not None}
        route = {"via": a.via, "jitter": a.jitter}
        if a.random_route:
            route = {"via": a.via or "random", "jitter": RANDOM_ROUTE_JITTER if a.jitter is None else a.jitter}
        if any(route.values()):
            logging.info("路线变化：途经点 %s，安全点随机偏移 %s m", route["via"] or "无", route["jitter"] or 0)
        self.nav = GentoNavigation(transport=a.transport, ssh_target=a.ssh_target, gaps=gaps, route=route)
        sdk = GentoSDK(config)
        self.sequence = self.make_sequence(sdk, config)
        result = self.sequence.result
        old = {s: signal.signal(s, lambda *_: self.sequence.request_stop()) for s in (signal.SIGINT, signal.SIGTERM)}
        exit_code = 1
        try:
            if a.finish_place is not None:                  # a place stopped with the arms down: finish it
                self.finish_interrupted_place(a.finish_place)
            else:
                start_pose = self.arm_start(config)             # before any motion
                if a.holding and start_pose == "prepare":
                    raise MotionError("--holding：双臂必须已在运输（接盘）位且夹着托盘")
                if start_pose in ("carry", "carry_wrist"):
                    logging.info("双臂已在运输（接盘）位：%s，直接接入%s",
                                 "只转腕部到新姿态" if start_pose == "carry_wrist" else "不动手臂",
                                 "（夹着托盘，不张爪）" if a.holding else "")
                    self.attach_at_carry(wrists=start_pose == "carry_wrist")
                else:
                    self.sequence.prepare()                     # arms into the carry (receive) pose
                self.preflight_place()                          # read-only, before the tray is handed over
                if a.selfscan:
                    logging.info("selfscan: %s", self.nav.selfscan())
                cycle = os.environ.get("GENTO_CYCLE", "")
                prefix = f"第 {cycle} 次：" if cycle and cycle != "1/1" else ""
                if a.holding:
                    logging.info("--holding：托盘已在双爪中，不交接；再闭合一次（保持力矩），直接去放置点")
                    self.close_grippers()
                elif not sys.stdin.isatty():
                    logging.info("无终端：不提示交接，按托盘已夹好出发")
                else:
                    if a.handover:                              # drive to T first and open there
                        if a.start:
                            self.navigate(a.start)
                        if start_pose != "prepare" and self.gripper is not None:
                            self.sequence._gripper_action("open_before_receive", GripperAction.OPEN)
                    self.handover(f"{prefix}交接托盘（就在当前位置，双臂在运输位不动）。")
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
    p.add_argument("--down-mm", type=float,
                   help=f"本次放置下降距离 mm（默认 {PLACE_DOWN_MM:.0f}，篮子；配置文件里的 150 是托盘的）")
    p.add_argument("--up-mm", type=float, help="本次放置后上移距离 mm（默认等于下降距离）")
    p.add_argument("--place-lift-mm", type=float, default=PLACE_LIFT_MM,
                   help=f"放置时双臂下降后升降再下降的距离 mm（默认 {PLACE_LIFT_MM:.0f}；0 = 只在双臂够不到时补足）")
    p.add_argument("--holding", action="store_true",
                   help="托盘已在双爪中：不张爪、不提示交接，直接去放置点")
    p.add_argument("--handover", action="store_true",
                   help="先导航到起始点 T、张爪，再交接（默认：在当前位置交接）")
    p.add_argument("--no-return", action="store_true", help="放置后不返回起始点")
    p.add_argument("--finish-place", type=float, metavar="UP_MM",
                   help="放置中途停在放置点（双臂已下降）后收尾：松爪 → 升降回原高度 → 双臂上移 UP_MM → 后移回运输位 → 回起始点")
    p.add_argument("--selfscan", action="store_true", help="准备完成后先测手臂／托盘占位（车前约 1 m 需无物体）")
    p.add_argument("--random-route", action="store_true",
                   help="另外在安全点附近随机偏移 0.08 m（途经点的随机选择本来就默认开启）")
    p.add_argument("--via", help="途经点：默认 random（各途经点和直达等概率）| none 不经过途经点 | 途经点名（如 W1）")
    p.add_argument("--jitter", type=float, help="安全点随机偏移半径 m（0–0.20；--random-route 默认 0.08）")
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

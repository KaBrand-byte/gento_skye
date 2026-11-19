"""Run the arms when the lift / waist / head drivers did not come onto the arm controller's bus.

After some power-ups the controller never initialises these drivers (2026-11-12, 11-13, 11-20): they read exactly 0
with no error, stay out of position mode, and switching them fails with -28 "PDO data not init complete".  The SDK
has no call to bring them back; a power-up with all e-stops released does.  The task never moves them (the lift is
fixed at its taught height; the place descent is done by the arms), so instead of refusing to start, OfflineAxes
wraps the SDK object for skye_task's Sequence:
  - read(): an offline axis is reported at its taught value (lift receive_lift, waist / head body / head) and in
    position mode, so the fixed-axis checks and the shoulder kinematics (a vertical shift only) work
  - enter_position(): offline axes are left alone (no mode switch, no hold command)
  - move_lift(): refused while the lift is offline; hold_lift(): nothing to hold
A lift that IS online but at another height still stops the task: then the place height really is different.
"""
import copy
import logging

from skye_task.sdk import Feedback, MotionError

AXES = ("lift", "body", "head")
NAMES = {"lift": "升降", "body": "腰部", "head": "头部"}
log = logging.getLogger(__name__)


def is_offline(data):
    """The signature of a driver the controller never initialised: no error, not in position mode, all zeros."""
    state, pos = data.get("state") or {}, data.get("fb_pos") or []
    return state.get("err") == 0 and state.get("cur") == 0 and len(pos) > 0 and all(v == 0 for v in pos)


def nominal(config, name):
    return list(config.receive_lift) if name == "lift" else list(getattr(config, name))


class OfflineAxes:
    def __init__(self, sdk, config):
        object.__setattr__(self, "_sdk", sdk)
        object.__setattr__(self, "_config", config)
        object.__setattr__(self, "offline", set())

    def __getattr__(self, name):
        return getattr(self._sdk, name)

    def __setattr__(self, name, value):
        setattr(self._sdk, name, value)

    def read(self):
        fb = self._sdk.read()
        off = {name for name in AXES if is_offline(fb.rt[name])}
        if off != self.offline:
            if off:
                log.warning("%s没有上线（读数为 0，控制器未初始化这些驱动器）：按示教值（升降 %.0f mm）处理，"
                            "任务中不使用、不控制它们。下次开机前先松开所有急停", "、".join(NAMES[n] for n in AXES if n in off),
                            self._config.receive_lift[0])
            else:
                log.info("升降 / 腰部 / 头部已上线")
            object.__setattr__(self, "offline", off)
        if not off:
            return fb
        rt = copy.deepcopy(fb.rt)
        for name in off:
            values = nominal(self._config, name)
            rt[name]["fb_pos"] = values + list(rt[name]["fb_pos"][len(values):])
            rt[name]["state"] = dict(rt[name]["state"], cur=1)
        return Feedback(rt, fb.sg)

    def enter_position(self, feedback, checkpoint):
        objects = getattr(type(self._sdk), "OBJECTS", None)
        if not self.offline or objects is None:
            return self._sdk.enter_position(feedback, checkpoint)
        self._sdk.OBJECTS = {k: v for k, v in objects.items() if k not in self.offline}   # instance attribute
        try:
            return self._sdk.enter_position(feedback, checkpoint)
        finally:
            del self._sdk.OBJECTS                                                    # back to the class one

    def move_lift(self, target_mm):
        """skye_task's prepare() always sends the lift to the carry height, even when it is already there: with
        the lift offline (fixed at that height) that is already done; any other height is refused."""
        if "lift" in self.offline:
            fixed = self._config.receive_lift[0]
            if abs(target_mm - fixed) <= self._config.fixed_lift_tolerance_mm:
                log.info("升降没有上线：目标 %.1f mm 就是固定高度，不发指令", target_mm)
                return None
            raise MotionError(f"升降没有上线（读数为 0），不能把升降移到 {target_mm:.0f} mm（固定高度 {fixed:.0f} mm）")
        return self._sdk.move_lift(target_mm)

    def hold_lift(self):
        if "lift" in self.offline:
            return None
        return self._sdk.hold_lift()

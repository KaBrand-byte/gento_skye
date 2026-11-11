"""Dual grippers sharing the arm task's existing SDK connection."""

from .dm_gripper import DMGripper
from .dm_terminal import GentoTerminalConnection
from .interfaces import Gripper, GripperAction, GripperTarget
from .sdk import MotionError


class TaskGripper(Gripper):
    def __init__(self, sdk, config):
        self.config = config.validate()
        # Refuse incomplete endpoints before either arms or motors can move.
        for action in ("open", "close"):
            config.validate_positions(config.action_positions(action, "both"), ("left", "right"))
        self.connection = GentoTerminalConnection(sdk.config, config, sdk=sdk)
        self.device = None
        self.settled = False
        self._hold_error_since = {}

    def reserve(self):
        """Exclude standalone gripper tools before opening the arm SDK link."""
        self.connection.reserve()

    def connect(self):
        self.connection.connect()
        self.device = DMGripper(self.config, "both", transport_factory=self.connection.transport)
        self.device.connect()

    def control(self, target, action, checkpoint=lambda: None):
        if target != GripperTarget.BOTH:
            raise ValueError("整体任务的夹爪动作必须包含双侧")
        action = GripperAction(action).value
        targets = self.config.action_positions(action, "both")
        if self.device is None:
            raise MotionError("任务夹爪尚未连接")
        self.settled = False
        self._hold_error_since.clear()
        if self.device.thread is None:
            self.device.enable(checkpoint)
        result = self.device.move_to(targets, checkpoint, action=action)
        self.settled = True
        return result

    def check_health(self):
        if self.device is not None and self.device.thread is not None:
            self.device.check_health()
            if self.settled:
                sample = self.device.snapshot()
                for name, target in sample["targets_rad"].items():
                    feedback = sample["feedback"][name]
                    actual = feedback["position_rad"]
                    if abs(actual - target) <= self.config.hold_position_tolerance_rad:
                        self._hold_error_since.pop(name, None)
                        continue
                    # Count feedback time, so rereading one sample cannot extend
                    # an excursion. check_health above still rejects stale/faulty data.
                    stamp = feedback["received_at"]
                    since = self._hold_error_since.setdefault(name, stamp)
                    if stamp - since >= self.config.hold_error_seconds:
                        raise MotionError(f"{name} 夹爪持续偏离保持角度: 目标 {target:.6f} rad，实际 {actual:.6f} rad；"
                                          f"偏差超过 {self.config.hold_position_tolerance_rad:.3f} rad "
                                          f"达 {self.config.hold_error_seconds:.2f} s")

    def freeze(self):
        self.settled = False
        self._hold_error_since.clear()
        if self.device is not None:
            self.device.freeze()

    def snapshot(self):
        return {"device": self.device.snapshot() if self.device is not None else {},
                "terminal": self.connection.snapshot()}

    def close(self):
        # A task may still hold a tray after a failure. Never send FD or reopen here.
        # Stop the worker BEFORE the owner closes/unlinks the shared SDK.
        try:
            if self.device is not None:
                self.device.close()
        finally:
            if not self.worker_running:
                self.connection.close()

    @property
    def worker_running(self):
        return (self.device is not None and self.device.thread is not None
                and self.device.thread.is_alive())

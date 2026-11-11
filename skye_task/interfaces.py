"""Task interfaces. Motion completion and grasp/placement detection are separate."""

from enum import Enum
from typing import Optional, Protocol


class GripperTarget(str, Enum):
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


class GripperAction(str, Enum):
    OPEN = "open"
    CLOSE = "close"


class Gripper:
    def control(self, target: GripperTarget, action: GripperAction, checkpoint=lambda: None):
        raise NotImplementedError("请选择实际夹爪驱动，例如 TaskGripper")

    def is_grasped(self) -> Optional[bool]:
        return None

    def is_placed(self) -> Optional[bool]:
        return None


class Navigation(Protocol):
    def start(self, destination: str) -> None:
        """Submit a navigation task; return is not an arrival indication."""
        ...

    def is_arrived_and_stopped(self) -> bool:
        """Return true only after arrival and stationary feedback."""
        ...

    def stop(self) -> None:
        ...

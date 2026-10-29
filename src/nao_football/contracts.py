"""Shared skill vocabulary; these declarations do not implement robot control."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class Skill(str, Enum):
    STAND = "STAND"
    STOP = "STOP"
    WALK = "WALK"
    HEAD_LOOK = "HEAD_LOOK"
    KICK_FORWARD = "KICK_FORWARD"
    GET_UP = "GET_UP"


class SkillStatus(str, Enum):
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class SkillRequest:
    robot_id: str
    sequence_id: int
    skill: Skill
    frame_id: str
    timeout_s: float
    target_x_m: Optional[float] = None
    target_y_m: Optional[float] = None
    target_yaw_rad: Optional[float] = None


@dataclass(frozen=True)
class SkillResult:
    robot_id: str
    sequence_id: int
    status: SkillStatus
    stamp_s: float
    reason: str


# FALL_DETECTED is an observation/preemption event. There is no RECEIVE skill:
# wait for a stopped ball, approach using WALK/HEAD_LOOK, STOP and KICK_FORWARD.

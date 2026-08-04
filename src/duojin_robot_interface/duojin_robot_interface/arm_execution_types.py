"""Data contracts shared by the ROS arm execution modules."""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class ArmExecutionConfig:
    execute: bool
    ik_solver_frame: str
    public_pose_frame: str
    pose_publish_period_sec: float
    feedback_freshness_sec: float
    feedback_wait_timeout_sec: float
    tf_timeout_sec: float
    ik_response_timeout_sec: float
    poll_period_sec: float
    default_pose_timeout_sec: float
    maximum_timeout_sec: float
    hold_timeout_sec: float
    shutdown_drain_timeout_sec: float
    settle_duration_sec: float
    pose_position_tolerance_m: float
    pose_orientation_tolerance_rad: float
    joint_position_tolerance_rad: float
    joint_limits_rad: Tuple[Tuple[float, float], ...]
    joint_limit_margin_rad: float
    maximum_joint_delta_rad: float
    safe_joint_velocity_rad_s: Tuple[float, ...]
    hold_joint_velocity_rad_s: float
    maximum_cartesian_delta_m: float


class MotionFailure(Exception):
    """Expected failure that maps directly to an Action result."""

    def __init__(self, outcome: int, reason: int, message: str) -> None:
        super().__init__(message)
        self.outcome = outcome
        self.reason = reason
        self.message = message


@dataclass(frozen=True)
class HoldResult:
    confirmed: bool
    message: str
    final_positions_rad: Tuple[float, ...]

"""Stable Python result mapping for the public arm Actions."""

from dataclasses import dataclass
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from duojin_interfaces.msg import ArmMotionStatus


@dataclass(frozen=True)
class ArmResult:
    """Terminal API result with explicit unknown/invalid-state markers."""

    succeeded: bool
    outcome: int
    reason: int
    message: str
    executed: Optional[bool]
    execution_state_known: bool = True
    final_state_valid: bool = False
    final_position_xyz: Optional[Tuple[float, float, float]] = None
    final_orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
    final_frame_id: Optional[str] = None
    final_positions_rad: Optional[Tuple[float, ...]] = None
    position_error_m: Optional[float] = None
    orientation_error_rad: Optional[float] = None
    max_position_error_rad: Optional[float] = None


@dataclass(frozen=True)
class ArmPoseReading:
    """One fresh public end-effector pose sample."""

    frame_id: str
    position_xyz: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]
    stamp_sec: float


def map_pose_reading(message) -> ArmPoseReading:
    position = message.pose.position
    orientation = message.pose.orientation
    stamp = message.header.stamp
    return ArmPoseReading(
        frame_id=message.header.frame_id,
        position_xyz=(position.x, position.y, position.z),
        orientation_xyzw=(orientation.x, orientation.y, orientation.z, orientation.w),
        stamp_sec=float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0,
    )


def known_failure(reason: int, message: str, *, fatal: bool = False) -> ArmResult:
    """Return a locally proven no-publication failure."""

    outcome = (
        ArmMotionStatus.FATAL_FAILURE if fatal else ArmMotionStatus.RETRYABLE_FAILURE
    )
    return ArmResult(False, outcome, reason, message, False)


def unknown_execution(message: str) -> ArmResult:
    """Never encode an unconfirmed remote goal as executed=False."""

    warning = (
        f"{message} Execution/stop state is UNKNOWN; do not send another goal. "
        "Check the robot and use the hardware emergency stop if motion may continue."
    )
    return ArmResult(
        False,
        ArmMotionStatus.FATAL_FAILURE,
        ArmMotionStatus.EXECUTION_STATE_UNKNOWN,
        warning,
        None,
        execution_state_known=False,
    )


def rejected_result() -> ArmResult:
    """A rejected Action goal is known not to have entered execution."""

    return known_failure(
        ArmMotionStatus.SERVER_UNAVAILABLE,
        "arm Action server rejected the goal, possibly because it is shutting down",
    )


def locally_cancelled(message: str) -> ArmResult:
    """Cancellation completed before any goal request was transmitted."""

    return ArmResult(
        False,
        ArmMotionStatus.CANCELLED,
        ArmMotionStatus.NONE,
        message,
        False,
    )


def _expected_transport_status(outcome: int) -> Optional[int]:
    if outcome == ArmMotionStatus.SUCCESS:
        return GoalStatus.STATUS_SUCCEEDED
    if outcome == ArmMotionStatus.CANCELLED:
        return GoalStatus.STATUS_CANCELED
    if outcome in (
        ArmMotionStatus.RETRYABLE_FAILURE,
        ArmMotionStatus.FATAL_FAILURE,
        ArmMotionStatus.TIMEOUT,
    ):
        return GoalStatus.STATUS_ABORTED
    return None


def _transport_is_consistent(wrapped_result) -> bool:
    payload_outcome = int(wrapped_result.result.status.outcome)
    expected = _expected_transport_status(payload_outcome)
    return expected is not None and int(wrapped_result.status) == expected


def _transport_failure(wrapped_result) -> ArmResult:
    payload_outcome = int(wrapped_result.result.status.outcome)
    return unknown_execution(
        "ROS Action terminal status disagrees with its payload "
        f"(transport={int(wrapped_result.status)}, payload_outcome={payload_outcome})."
    )


def map_pose_result(wrapped_result) -> ArmResult:
    """Map a pose result only after validating the Action transport terminal state."""

    if not _transport_is_consistent(wrapped_result):
        return _transport_failure(wrapped_result)
    result = wrapped_result.result
    status = result.status
    valid = bool(result.final_pose_valid)
    if int(status.outcome) == ArmMotionStatus.SUCCESS and not valid:
        return unknown_execution("Pose Action reported SUCCESS without a valid final pose.")
    position = result.final_pose.pose.position
    orientation = result.final_pose.pose.orientation
    return ArmResult(
        succeeded=int(status.outcome) == ArmMotionStatus.SUCCESS,
        outcome=int(status.outcome),
        reason=int(status.reason),
        message=status.message,
        executed=bool(status.executed),
        final_state_valid=valid,
        final_position_xyz=(position.x, position.y, position.z) if valid else None,
        final_orientation_xyzw=(
            (orientation.x, orientation.y, orientation.z, orientation.w)
            if valid
            else None
        ),
        final_frame_id=result.final_pose.header.frame_id if valid else None,
        position_error_m=float(result.position_error_m) if valid else None,
        orientation_error_rad=(
            float(result.orientation_error_rad) if valid else None
        ),
    )


def map_joint_result(wrapped_result) -> ArmResult:
    """Map a joint result only after validating the Action transport terminal state."""

    if not _transport_is_consistent(wrapped_result):
        return _transport_failure(wrapped_result)
    result = wrapped_result.result
    status = result.status
    valid = bool(result.final_positions_valid)
    if int(status.outcome) == ArmMotionStatus.SUCCESS and not valid:
        return unknown_execution(
            "Joint Action reported SUCCESS without valid final joint feedback."
        )
    return ArmResult(
        succeeded=int(status.outcome) == ArmMotionStatus.SUCCESS,
        outcome=int(status.outcome),
        reason=int(status.reason),
        message=status.message,
        executed=bool(status.executed),
        final_state_valid=valid,
        final_positions_rad=tuple(result.final_positions_rad) if valid else None,
        max_position_error_rad=(
            float(result.max_position_error_rad) if valid else None
        ),
    )

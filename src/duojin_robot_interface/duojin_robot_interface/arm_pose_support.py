"""Result, feedback, TF and hold helpers for the pose Action."""

from copy import deepcopy
from dataclasses import dataclass
from functools import partial
import time
from typing import Any, Optional, Tuple

from duojin_interfaces.msg import ArmMotionStatus
from geometry_msgs.msg import PoseStamped

from .arm_domain import (
    ArmValidationError,
    is_fresh,
    position_error_m,
    quaternion_shortest_angle_rad,
    validate_cartesian_target,
)
from .arm_execution import MotionFailure


def pose_values(pose: PoseStamped) -> tuple[Tuple[float, ...], Tuple[float, ...]]:
    position = pose.pose.position
    orientation = pose.pose.orientation
    return (
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    )


def pose_result(
    executor,
    action_type,
    outcome: int,
    reason: int,
    message: str,
    executed: bool,
    final_pose: Optional[PoseStamped] = None,
    position_error: float = 0.0,
    orientation_error: float = 0.0,
):
    result = action_type.Result()
    result.status = executor.status(outcome, reason, message, executed)
    result.final_pose = deepcopy(final_pose) if final_pose is not None else PoseStamped()
    result.final_pose_valid = final_pose is not None
    result.position_error_m = float(position_error)
    result.orientation_error_rad = float(orientation_error)
    return result


def pose_feedback(
    goal_handle,
    action_type,
    phase: int,
    current_pose: Optional[PoseStamped],
    position_error: float,
    orientation_error: float,
    started_at_s: float,
) -> None:
    message = action_type.Feedback()
    message.phase = int(phase)
    message.current_pose = (
        deepcopy(current_pose) if current_pose is not None else PoseStamped()
    )
    message.position_error_m = float(position_error)
    message.orientation_error_rad = float(orientation_error)
    message.elapsed_sec = max(0.0, time.monotonic() - started_at_s)
    goal_handle.publish_feedback(message)


def transform_pose(executor, adapter, pose: PoseStamped, target_frame: str) -> PoseStamped:
    try:
        transformed = adapter.transform_pose(
            pose, target_frame, executor.config.tf_timeout_sec
        )
        xyz, quaternion = pose_values(transformed)
        validated = validate_cartesian_target(xyz, quaternion)
        transformed.pose.orientation.x = validated.orientation_xyzw[0]
        transformed.pose.orientation.y = validated.orientation_xyzw[1]
        transformed.pose.orientation.z = validated.orientation_xyzw[2]
        transformed.pose.orientation.w = validated.orientation_xyzw[3]
        return transformed
    except (ArmValidationError, ValueError, RuntimeError) as exc:
        raise MotionFailure(
            ArmMotionStatus.RETRYABLE_FAILURE,
            ArmMotionStatus.TF_UNAVAILABLE,
            f"cannot transform/validate end-effector pose into {target_frame}: {exc}",
        ) from exc
    except Exception as exc:
        # tf2 exception subclasses differ between Humble Python builds. This
        # call has no side effect, so all failures here map to the TF gate.
        raise MotionFailure(
            ArmMotionStatus.RETRYABLE_FAILURE,
            ArmMotionStatus.TF_UNAVAILABLE,
            f"TF lookup into {target_frame} failed: {exc}",
        ) from exc


def pose_errors(
    current_pose: PoseStamped, target_pose: PoseStamped
) -> tuple[float, float]:
    current_xyz, current_quaternion = pose_values(current_pose)
    target_xyz, target_quaternion = pose_values(target_pose)
    return (
        position_error_m(current_xyz, target_xyz),
        quaternion_shortest_angle_rad(current_quaternion, target_quaternion),
    )


@dataclass
class _PoseHoldContext:
    executor: Any
    goal_handle: Any
    action_type: Any
    adapter: Any
    owner: int
    target_goal_frame: PoseStamped
    started_at_s: float
    outcome: int
    reason: int
    message: str
    canceled: bool
    last_pose: Optional[PoseStamped] = None
    last_position_error: float = 0.0
    last_orientation_error: float = 0.0


def _refresh_hold_pose(context: _PoseHoldContext) -> None:
    snapshot = context.adapter.feedback()
    if snapshot.pose is None or not is_fresh(
        snapshot.pose_received_at_s,
        context.executor.config.feedback_freshness_sec,
    ):
        return
    try:
        context.last_pose = transform_pose(
            context.executor,
            context.adapter,
            snapshot.pose,
            context.target_goal_frame.header.frame_id,
        )
        context.last_position_error, context.last_orientation_error = pose_errors(
            context.last_pose, context.target_goal_frame
        )
    except MotionFailure:
        pass


def _publish_hold_feedback(
    context: _PoseHoldContext, _positions, _joint_error
) -> None:
    _refresh_hold_pose(context)
    pose_feedback(
        context.goal_handle,
        context.action_type,
        context.action_type.Feedback.HOLDING,
        context.last_pose,
        context.last_position_error,
        context.last_orientation_error,
        context.started_at_s,
    )


def _hold_failed_result(context: _PoseHoldContext, hold):
    context.goal_handle.abort()
    return pose_result(
        context.executor,
        context.action_type,
        ArmMotionStatus.FATAL_FAILURE,
        ArmMotionStatus.STOP_FAILED,
        f"{context.message}; hold failed: {hold.message}. Use the hardware emergency "
        "stop if motion continues.",
        True,
        context.last_pose,
        context.last_position_error,
        context.last_orientation_error,
    )


def _hold_confirmed_result(context: _PoseHoldContext, hold):
    context.executor.release_physical_command(context.adapter.arm, context.owner)
    if context.canceled:
        context.goal_handle.canceled()
    else:
        context.goal_handle.abort()
    return pose_result(
        context.executor,
        context.action_type,
        context.outcome,
        context.reason,
        f"{context.message}; {hold.message}",
        True,
        context.last_pose,
        context.last_position_error,
        context.last_orientation_error,
    )


def hold_and_finish_pose(
    executor,
    goal_handle,
    action_type,
    adapter,
    owner: int,
    target_goal_frame: PoseStamped,
    started_at_s: float,
    outcome: int,
    reason: int,
    message: str,
    canceled: bool = False,
):
    context = _PoseHoldContext(
        executor,
        goal_handle,
        action_type,
        adapter,
        owner,
        target_goal_frame,
        started_at_s,
        outcome,
        reason,
        message,
        canceled,
    )
    hold = executor.hold_current_joints(
        adapter, partial(_publish_hold_feedback, context), owner=owner
    )
    if not hold.confirmed:
        return _hold_failed_result(context, hold)
    return _hold_confirmed_result(context, hold)

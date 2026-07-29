"""IK response and closed-loop arrival monitoring for the pose Action."""

import time
from typing import Optional

from duojin_interfaces.msg import ArmMotionStatus
from geometry_msgs.msg import PoseStamped

from .arm_domain import (
    ArmValidationError,
    ArrivalGate,
    ArrivalGateConfig,
    is_fresh,
    max_joint_error_rad,
    validate_joint_positions,
)
from .arm_execution import MotionFailure
from .arm_pose_support import (
    hold_and_finish_pose,
    pose_errors,
    pose_feedback,
    pose_result,
    transform_pose,
)


def _cancel_result(context):
    return hold_and_finish_pose(
        context.executor,
        context.goal_handle,
        context.action_type,
        context.adapter,
        context.owner,
        context.target_goal_frame,
        context.started_at_s,
        ArmMotionStatus.CANCELLED,
        ArmMotionStatus.NONE,
        f"{context.arm} end-effector goal canceled",
        canceled=True,
    )


def _fresh_pose(context, snapshot) -> PoseStamped:
    context.executor.ensure_fresh_joint_feedback(snapshot, context.adapter)
    if snapshot.pose is None or not is_fresh(
        snapshot.pose_received_at_s,
        context.executor.config.feedback_freshness_sec,
    ):
        raise MotionFailure(
            ArmMotionStatus.RETRYABLE_FAILURE,
            ArmMotionStatus.FEEDBACK_STALE,
            f"end-effector feedback on {context.adapter.pose_feedback_topic} became stale",
        )
    return snapshot.pose


def wait_for_ik(context) -> Optional[object]:
    ik_deadline = min(
        context.motion_deadline,
        context.published_after_s + context.executor.config.ik_response_timeout_sec,
    )
    while time.monotonic() < ik_deadline:
        context.executor.require_running()
        if context.goal_handle.is_cancel_requested:
            return _cancel_result(context)
        snapshot = context.adapter.feedback()
        _fresh_pose(context, snapshot)
        ik_positions = context.adapter.new_ik_command_after(context.published_after_s)
        if ik_positions is not None:
            _validate_ik_output(context, snapshot, ik_positions)
            return None
        time.sleep(context.executor.config.poll_period_sec)
    raise MotionFailure(
        ArmMotionStatus.RETRYABLE_FAILURE,
        ArmMotionStatus.IK_NO_RESPONSE,
        "Relaxed IK produced no new joint target within "
        f"{context.executor.config.ik_response_timeout_sec:.2f}s",
    )


def _validate_ik_output(context, snapshot, ik_positions) -> None:
    try:
        validated = validate_joint_positions(
            ik_positions,
            joint_limits_rad=context.executor.config.joint_limits_rad,
            joint_limit_margin_rad=context.executor.config.joint_limit_margin_rad,
        )
    except ArmValidationError as exc:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.OUT_OF_RANGE,
            f"Relaxed IK output violates effective joint limits: {exc}",
        ) from exc
    current = context.executor.ensure_fresh_joint_feedback(snapshot, context.adapter)
    delta_rad = max_joint_error_rad(current, validated)
    if delta_rad > context.executor.config.maximum_joint_delta_rad:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.OUT_OF_RANGE,
            f"Relaxed IK output joint delta {delta_rad:.4f}rad exceeds per-goal limit "
            f"{context.executor.config.maximum_joint_delta_rad:.4f}rad",
        )


def _arrival_success(context):
    context.executor.release_physical_command(context.arm, context.owner)
    context.goal_handle.succeed()
    return pose_result(
        context.executor,
        context.action_type,
        ArmMotionStatus.SUCCESS,
        ArmMotionStatus.NONE,
        f"{context.arm} end-effector goal reached and remained within tolerance",
        True,
        context.current_goal_frame,
        context.position_error,
        context.orientation_error,
    )


def _pose_is_arrived(context, gate: ArrivalGate) -> bool:
    return gate.update(
        context.position_error
        <= context.executor.config.pose_position_tolerance_m
        and context.orientation_error
        <= context.executor.config.pose_orientation_tolerance_rad,
        time.monotonic(),
    )


def wait_for_arrival(context):
    gate = ArrivalGate(
        ArrivalGateConfig(
            min_samples=3,
            min_continuous_s=context.executor.config.settle_duration_sec,
        )
    )
    while time.monotonic() < context.motion_deadline:
        context.executor.require_running()
        if context.goal_handle.is_cancel_requested:
            return _cancel_result(context)
        snapshot = context.adapter.feedback()
        pose = _fresh_pose(context, snapshot)
        context.current_goal_frame = transform_pose(
            context.executor, context.adapter, pose, context.target_frame
        )
        context.position_error, context.orientation_error = pose_errors(
            context.current_goal_frame, context.target_goal_frame
        )
        pose_feedback(
            context.goal_handle,
            context.action_type,
            context.action_type.Feedback.WAITING_FOR_GOAL,
            context.current_goal_frame,
            context.position_error,
            context.orientation_error,
            context.started_at_s,
        )
        if _pose_is_arrived(context, gate):
            return _arrival_success(context)
        time.sleep(context.executor.config.poll_period_sec)
    return hold_and_finish_pose(
        context.executor,
        context.goal_handle,
        context.action_type,
        context.adapter,
        context.owner,
        context.target_goal_frame,
        context.started_at_s,
        ArmMotionStatus.TIMEOUT,
        ArmMotionStatus.NONE,
        f"{context.arm} end-effector goal timed out after {context.timeout_sec:.2f}s",
    )

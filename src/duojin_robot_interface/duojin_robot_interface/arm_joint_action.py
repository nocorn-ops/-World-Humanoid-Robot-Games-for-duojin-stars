"""Joint-space arm Action closed loop."""

from dataclasses import dataclass
import time
from typing import Any, Optional, Tuple

from duojin_interfaces.action import MoveArmJoints
from duojin_interfaces.msg import ArmMotionStatus

from .arm_domain import (
    ArmValidationError,
    ArrivalGate,
    ArrivalGateConfig,
    JointTarget,
    compute_joint_timeout_s,
    max_joint_error_rad,
    validate_joint_positions,
    validate_joint_target,
    validate_speed_scale,
)
from .arm_execution import MotionFailure
from .arm_joint_support import (
    hold_and_finish_joints as _hold_and_finish,
    joint_feedback as _feedback,
    joint_result as _result,
)


@dataclass
class _JointExecutionContext:
    executor: Any
    goal_handle: Any
    arm: str
    adapter: Any
    owner: int
    started_at_s: float
    command_published: bool = False
    last_positions: Tuple[float, ...] = tuple()
    last_error_rad: float = 0.0
    last_joint_received_at_s: Optional[float] = None


@dataclass(frozen=True)
class _PreparedJointGoal:
    target: JointTarget
    velocities_rad_s: Tuple[float, ...]
    timeout_sec: float


def _busy_result(context: _JointExecutionContext) -> Optional[MoveArmJoints.Result]:
    if context.executor.arbiter.try_acquire(context.arm, context.owner):
        return None
    context.goal_handle.abort()
    return _result(
        context.executor,
        ArmMotionStatus.RETRYABLE_FAILURE,
        ArmMotionStatus.BUSY,
        f"{context.arm} arm already has an active goal; cancel it before sending another",
        False,
    )


def _prepare_joint_goal(context: _JointExecutionContext) -> _PreparedJointGoal:
    executor, goal_handle = context.executor, context.goal_handle
    _feedback(
        goal_handle,
        MoveArmJoints.Feedback.VALIDATING,
        (),
        0.0,
        context.started_at_s,
    )
    snapshot = executor.require_ready(context.adapter, "joints", require_pose=False)
    context.last_positions = tuple(snapshot.joint_positions_rad or ())
    context.last_joint_received_at_s = snapshot.joint_received_at_s
    requested_positions, speed_scale = _validate_joint_request(context)
    try:
        target = validate_joint_target(
            requested_positions,
            context.last_positions,
            speed_scale,
            joint_limits_rad=executor.config.joint_limits_rad,
            joint_limit_margin_rad=executor.config.joint_limit_margin_rad,
            maximum_delta_rad=executor.config.maximum_joint_delta_rad,
        )
    except ArmValidationError as exc:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.OUT_OF_RANGE,
            str(exc),
        ) from exc
    velocities_rad_s = tuple(
        velocity * target.speed_scale
        for velocity in executor.config.safe_joint_velocity_rad_s
    )
    automatic_timeout_sec = compute_joint_timeout_s(
        context.last_positions,
        target.positions_rad,
        executor.config.safe_joint_velocity_rad_s,
        target.speed_scale,
    )
    timeout_sec = executor.resolve_timeout(
        float(goal_handle.request.timeout_sec), automatic_timeout_sec
    )
    context.last_error_rad = max_joint_error_rad(
        context.last_positions, target.positions_rad
    )
    return _PreparedJointGoal(target, velocities_rad_s, timeout_sec)


def _validate_joint_request(
    context: _JointExecutionContext,
) -> tuple[Tuple[float, ...], float]:
    try:
        positions = validate_joint_positions(
            context.goal_handle.request.positions_rad,
            enforce_operating_limits=False,
        )
        speed_scale = validate_speed_scale(context.goal_handle.request.speed_scale)
        return positions, speed_scale
    except ArmValidationError as exc:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.INVALID_GOAL,
            str(exc),
        ) from exc


def _preview_or_enable(
    context: _JointExecutionContext,
) -> Optional[MoveArmJoints.Result]:
    if context.goal_handle.is_cancel_requested:
        context.goal_handle.canceled()
        return _result(
            context.executor,
            ArmMotionStatus.CANCELLED,
            ArmMotionStatus.NONE,
            f"{context.arm} joint goal canceled before command publication",
            False,
            context.last_positions,
            context.last_error_rad,
        )
    if not context.goal_handle.request.execute:
        _feedback(
            context.goal_handle,
            MoveArmJoints.Feedback.PREVIEW,
            context.last_positions,
            context.last_error_rad,
            context.started_at_s,
        )
        context.goal_handle.succeed()
        return _result(
            context.executor,
            ArmMotionStatus.SUCCESS,
            ArmMotionStatus.PREVIEW_COMPLETE,
            f"preview passed for {context.arm} joint goal; no motion target was published",
            False,
            context.last_positions,
            context.last_error_rad,
        )
    if not context.executor.config.execute:
        raise MotionFailure(
            ArmMotionStatus.RETRYABLE_FAILURE,
            ArmMotionStatus.EXECUTION_DISABLED,
            "server is in preview-only mode; relaunch with execute:=true only after "
            "the operator completes the safety checklist",
        )
    return None


def _publish_joint_goal(
    context: _JointExecutionContext, prepared: _PreparedJointGoal
) -> None:
    _feedback(
        context.goal_handle,
        MoveArmJoints.Feedback.PUBLISHING,
        context.last_positions,
        context.last_error_rad,
        context.started_at_s,
    )
    context.executor.publish_motion_target(
        context.arm,
        context.owner,
        lambda: context.adapter.publish_joint_target(
            prepared.target.positions_rad, prepared.velocities_rad_s
        ),
    )
    context.command_published = True


def _update_wait_feedback(
    context: _JointExecutionContext, prepared: _PreparedJointGoal
) -> bool:
    snapshot = context.adapter.feedback()
    context.last_positions = context.executor.ensure_fresh_joint_feedback(
        snapshot, context.adapter
    )
    is_new_sample = snapshot.joint_received_at_s != context.last_joint_received_at_s
    context.last_joint_received_at_s = snapshot.joint_received_at_s
    context.last_error_rad = max_joint_error_rad(
        context.last_positions, prepared.target.positions_rad
    )
    _feedback(
        context.goal_handle,
        MoveArmJoints.Feedback.WAITING_FOR_GOAL,
        context.last_positions,
        context.last_error_rad,
        context.started_at_s,
    )
    return is_new_sample


def _finish_canceled(context: _JointExecutionContext) -> MoveArmJoints.Result:
    return _hold_and_finish(
        context.executor,
        context.goal_handle,
        context.adapter,
        context.owner,
        context.started_at_s,
        ArmMotionStatus.CANCELLED,
        ArmMotionStatus.NONE,
        f"{context.arm} joint goal canceled",
        canceled=True,
    )


def _finish_success(context: _JointExecutionContext) -> MoveArmJoints.Result:
    context.executor.release_physical_command(context.arm, context.owner)
    context.goal_handle.succeed()
    return _result(
        context.executor,
        ArmMotionStatus.SUCCESS,
        ArmMotionStatus.NONE,
        f"{context.arm} joint goal reached and remained within tolerance",
        True,
        context.last_positions,
        context.last_error_rad,
    )


def _finish_timeout(
    context: _JointExecutionContext, timeout_sec: float
) -> MoveArmJoints.Result:
    return _hold_and_finish(
        context.executor,
        context.goal_handle,
        context.adapter,
        context.owner,
        context.started_at_s,
        ArmMotionStatus.TIMEOUT,
        ArmMotionStatus.NONE,
        f"{context.arm} joint goal timed out after {timeout_sec:.2f}s",
    )


def _wait_for_joint_arrival(
    context: _JointExecutionContext, prepared: _PreparedJointGoal
) -> MoveArmJoints.Result:
    deadline = time.monotonic() + prepared.timeout_sec
    gate = ArrivalGate(
        ArrivalGateConfig(
            min_samples=3,
            min_continuous_s=context.executor.config.settle_duration_sec,
        )
    )
    while time.monotonic() < deadline:
        context.executor.require_running()
        if context.goal_handle.is_cancel_requested:
            return _finish_canceled(context)
        is_new_sample = _update_wait_feedback(context, prepared)
        if is_new_sample and gate.update(
            context.last_error_rad
            <= context.executor.config.joint_position_tolerance_rad,
            time.monotonic(),
        ):
            return _finish_success(context)
        time.sleep(context.executor.config.poll_period_sec)
    return _finish_timeout(context, prepared.timeout_sec)


def _finish_failure(
    context: _JointExecutionContext, failure: MotionFailure
) -> MoveArmJoints.Result:
    if context.command_published or context.executor.has_physical_command(
        context.arm, context.owner
    ):
        return _hold_and_finish(
            context.executor,
            context.goal_handle,
            context.adapter,
            context.owner,
            context.started_at_s,
            failure.outcome,
            failure.reason,
            failure.message,
        )
    if context.goal_handle.is_cancel_requested:
        context.goal_handle.canceled()
        return _result(
            context.executor,
            ArmMotionStatus.CANCELLED,
            ArmMotionStatus.NONE,
            f"{context.arm} joint goal canceled before command publication",
            False,
            context.last_positions,
            context.last_error_rad,
        )
    context.goal_handle.abort()
    return _result(
        context.executor,
        failure.outcome,
        failure.reason,
        failure.message,
        False,
        context.last_positions,
        context.last_error_rad,
    )


def _finish_unexpected(
    context: _JointExecutionContext, error: Exception
) -> MoveArmJoints.Result:
    context.executor.node.get_logger().error(
        f"unexpected {context.arm} joint Action error: {error}"
    )
    if context.command_published or context.executor.has_physical_command(
        context.arm, context.owner
    ):
        return _hold_and_finish(
            context.executor,
            context.goal_handle,
            context.adapter,
            context.owner,
            context.started_at_s,
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.INTERNAL_ERROR,
            f"internal error: {error}",
        )
    context.goal_handle.abort()
    return _result(
        context.executor,
        ArmMotionStatus.FATAL_FAILURE,
        ArmMotionStatus.INTERNAL_ERROR,
        f"internal error: {error}",
        False,
        context.last_positions,
        context.last_error_rad,
    )


def execute_joints(executor, goal_handle, arm: str) -> MoveArmJoints.Result:
    """Validate, publish once, then require continuous joint arrival."""

    context = _JointExecutionContext(
        executor,
        goal_handle,
        arm,
        executor.adapters[arm],
        id(goal_handle),
        time.monotonic(),
    )
    busy = _busy_result(context)
    if busy is not None:
        return busy
    try:
        prepared = _prepare_joint_goal(context)
        preview = _preview_or_enable(context)
        if preview is not None:
            return preview
        _publish_joint_goal(context, prepared)
        return _wait_for_joint_arrival(context, prepared)
    except MotionFailure as exc:
        return _finish_failure(context, exc)
    except Exception as exc:
        return _finish_unexpected(context, exc)
    finally:
        executor.arbiter.release(arm, context.owner)

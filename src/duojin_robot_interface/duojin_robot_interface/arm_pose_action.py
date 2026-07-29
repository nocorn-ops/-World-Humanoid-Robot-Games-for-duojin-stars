"""End-effector position Action with explicit frame normalization."""

from dataclasses import dataclass
import time
from typing import Any, Optional

from duojin_interfaces.action import MoveArmPose, MoveArmRelative
from duojin_interfaces.msg import ArmMotionStatus
from geometry_msgs.msg import PoseStamped

from .arm_domain import ArmValidationError, position_error_m
from .arm_domain import validate_cartesian_target, validate_xyz
from .arm_execution import MotionFailure
from .arm_pose_domain import relative_cartesian_target
from .arm_pose_monitor import (
    wait_for_arrival as _wait_for_arrival,
    wait_for_ik as _wait_for_ik,
)
from .arm_pose_support import (
    hold_and_finish_pose as _hold_and_finish,
    pose_feedback as _feedback,
    pose_result as _result,
    pose_values as _pose_values,
    transform_pose as _transform,
)


@dataclass
class _PoseContext:
    executor: Any
    goal_handle: Any
    arm: str
    adapter: Any
    owner: int
    started_at_s: float
    action_type: Any
    relative: bool
    target_frame: str = ""
    command_published: bool = False
    target_goal_frame: Optional[PoseStamped] = None
    current_goal_frame: Optional[PoseStamped] = None
    target_solver_frame: Optional[PoseStamped] = None
    position_error: float = 0.0
    orientation_error: float = 0.0
    timeout_sec: float = 0.0
    published_after_s: float = 0.0
    motion_deadline: float = 0.0


def _new_context(executor, goal_handle, arm: str, action_type, relative) -> _PoseContext:
    return _PoseContext(
        executor=executor,
        goal_handle=goal_handle,
        arm=arm,
        adapter=executor.adapters[arm],
        owner=id(goal_handle),
        started_at_s=time.monotonic(),
        action_type=action_type,
        relative=relative,
    )


def _busy_result(context: _PoseContext):
    context.goal_handle.abort()
    return _result(
        context.executor,
        context.action_type,
        ArmMotionStatus.RETRYABLE_FAILURE,
        ArmMotionStatus.BUSY,
        f"{context.arm} arm already has an active goal; cancel it before sending another",
        False,
    )


def _validate_request(context: _PoseContext) -> tuple[float, ...]:
    request = context.goal_handle.request
    if not request.keep_current_orientation:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.INVALID_GOAL,
            "v1 supports only keep_current_orientation=true",
        )
    source = request.delta if context.relative else request.target_pose
    context.target_frame = source.header.frame_id
    if not context.target_frame:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.INVALID_GOAL,
            "command frame_id must not be empty",
        )
    try:
        values = (
            (source.vector.x, source.vector.y, source.vector.z)
            if context.relative
            else _pose_values(source)[0]
        )
        return validate_xyz(values)
    except ArmValidationError as exc:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.INVALID_GOAL,
            str(exc),
        ) from exc


def _build_target_pose(context: _PoseContext, validated_target) -> PoseStamped:
    target = PoseStamped()
    target.header.frame_id = context.target_frame
    target.pose.position.x = validated_target.position_xyz_m[0]
    target.pose.position.y = validated_target.position_xyz_m[1]
    target.pose.position.z = validated_target.position_xyz_m[2]
    target.pose.orientation.x = validated_target.orientation_xyzw[0]
    target.pose.orientation.y = validated_target.orientation_xyzw[1]
    target.pose.orientation.z = validated_target.orientation_xyzw[2]
    target.pose.orientation.w = validated_target.orientation_xyzw[3]
    return target


def _validate_displacement(context: _PoseContext) -> None:
    limit_m = context.executor.config.maximum_cartesian_delta_m
    if context.position_error > limit_m:
        raise MotionFailure(
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.OUT_OF_RANGE,
            f"Cartesian displacement {context.position_error:.4f}m exceeds per-goal "
            f"limit {limit_m:.4f}m",
        )


def _prepare_goal(context: _PoseContext) -> None:
    _feedback(
        context.goal_handle,
        context.action_type,
        context.action_type.Feedback.VALIDATING,
        None,
        0.0,
        0.0,
        context.started_at_s,
    )
    command_xyz = _validate_request(context)
    snapshot = context.executor.require_ready(
        context.adapter, "pose", require_pose=True
    )
    context.current_goal_frame = _transform(
        context.executor, context.adapter, snapshot.pose, context.target_frame
    )
    current_xyz, current_quaternion = _pose_values(context.current_goal_frame)
    validated_target = (
        relative_cartesian_target(current_xyz, current_quaternion, command_xyz)
        if context.relative
        else validate_cartesian_target(command_xyz, current_quaternion)
    )
    context.position_error = position_error_m(
        current_xyz, validated_target.position_xyz_m
    )
    _validate_displacement(context)
    context.target_goal_frame = _build_target_pose(context, validated_target)
    context.target_solver_frame = _transform(
        context.executor,
        context.adapter,
        context.target_goal_frame,
        context.executor.config.ik_solver_frame,
    )
    context.timeout_sec = context.executor.resolve_timeout(
        float(context.goal_handle.request.timeout_sec),
        context.executor.config.default_pose_timeout_sec,
    )
    context.orientation_error = 0.0


def _preview_or_require_execution(
    context: _PoseContext,
) -> Optional[object]:
    if context.goal_handle.is_cancel_requested:
        context.goal_handle.canceled()
        return _result(
            context.executor,
            context.action_type,
            ArmMotionStatus.CANCELLED,
            ArmMotionStatus.NONE,
            f"{context.arm} end-effector goal canceled before command publication",
            False,
            context.current_goal_frame,
            context.position_error,
            context.orientation_error,
        )
    if not context.goal_handle.request.execute:
        _feedback(
            context.goal_handle,
            context.action_type,
            context.action_type.Feedback.PREVIEW,
            context.current_goal_frame,
            context.position_error,
            context.orientation_error,
            context.started_at_s,
        )
        context.goal_handle.succeed()
        return _result(
            context.executor,
            context.action_type,
            ArmMotionStatus.SUCCESS,
            ArmMotionStatus.PREVIEW_COMPLETE,
            f"preview passed for {context.arm} target in {context.target_frame}; "
            "no motion target was published",
            False,
            context.current_goal_frame,
            context.position_error,
            context.orientation_error,
        )
    if not context.executor.config.execute:
        raise MotionFailure(
            ArmMotionStatus.RETRYABLE_FAILURE,
            ArmMotionStatus.EXECUTION_DISABLED,
            "server is in preview-only mode; relaunch with execute:=true only after "
            "the operator completes the safety checklist",
        )
    context.executor.node.get_logger().warning(
        f"Executing experimental {context.arm} Cartesian goal through vendor "
        "Relaxed IK; joint-output validation occurs after vendor publication"
    )
    return None


def _publish_target(context: _PoseContext) -> None:
    _feedback(
        context.goal_handle,
        context.action_type,
        context.action_type.Feedback.PUBLISHING,
        context.current_goal_frame,
        context.position_error,
        context.orientation_error,
        context.started_at_s,
    )
    context.published_after_s = time.monotonic()
    context.executor.publish_motion_target(
        context.arm,
        context.owner,
        lambda: context.adapter.publish_pose_target(context.target_solver_frame),
    )
    context.command_published = True
    context.motion_deadline = time.monotonic() + context.timeout_sec


def _finish_failure(
    context: _PoseContext, outcome: int, reason: int, message: str
):
    if (
        context.target_goal_frame is not None
        and (
            context.command_published
            or context.executor.has_physical_command(context.arm, context.owner)
        )
    ):
        return _hold_and_finish(
            context.executor,
            context.goal_handle,
            context.action_type,
            context.adapter,
            context.owner,
            context.target_goal_frame,
            context.started_at_s,
            outcome,
            reason,
            message,
        )
    if context.goal_handle.is_cancel_requested:
        context.goal_handle.canceled()
        return _result(
            context.executor,
            context.action_type,
            ArmMotionStatus.CANCELLED,
            ArmMotionStatus.NONE,
            f"{context.arm} end-effector goal canceled before command publication",
            False,
            context.current_goal_frame,
            context.position_error,
            context.orientation_error,
        )
    context.goal_handle.abort()
    return _result(
        context.executor,
        context.action_type,
        outcome,
        reason,
        message,
        False,
        context.current_goal_frame,
        context.position_error,
        context.orientation_error,
    )


def _finish_unexpected(context: _PoseContext, exc: Exception):
    context.executor.node.get_logger().error(
        f"unexpected {context.arm} pose Action error: {exc}"
    )
    return _finish_failure(
        context,
        ArmMotionStatus.FATAL_FAILURE,
        ArmMotionStatus.INTERNAL_ERROR,
        f"internal error: {exc}",
    )


def _execute_pose(executor, goal_handle, arm: str, action_type, relative):
    context = _new_context(executor, goal_handle, arm, action_type, relative)
    if not executor.arbiter.try_acquire(arm, context.owner):
        return _busy_result(context)
    try:
        _prepare_goal(context)
        preview_result = _preview_or_require_execution(context)
        if preview_result is not None:
            return preview_result
        _publish_target(context)
        canceled_result = _wait_for_ik(context)
        if canceled_result is not None:
            return canceled_result
        return _wait_for_arrival(context)
    except MotionFailure as exc:
        return _finish_failure(context, exc.outcome, exc.reason, exc.message)
    except Exception as exc:
        return _finish_unexpected(context, exc)
    finally:
        executor.arbiter.release(arm, context.owner)


def execute_pose(executor, goal_handle, arm: str) -> MoveArmPose.Result:
    """Validate an absolute target while retaining current orientation."""

    return _execute_pose(executor, goal_handle, arm, MoveArmPose, False)


def execute_relative(executor, goal_handle, arm: str) -> MoveArmRelative.Result:
    """Offset xyz in the requested frame while retaining current orientation."""

    return _execute_pose(executor, goal_handle, arm, MoveArmRelative, True)

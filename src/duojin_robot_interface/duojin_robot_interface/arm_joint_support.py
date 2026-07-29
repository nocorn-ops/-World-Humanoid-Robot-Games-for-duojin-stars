"""Result, feedback and stop helpers for the joint-space Action."""

import time
from typing import Sequence

from duojin_interfaces.action import MoveArmJoints
from duojin_interfaces.msg import ArmMotionStatus


def joint_result(
    executor,
    outcome: int,
    reason: int,
    message: str,
    executed: bool,
    positions_rad: Sequence[float] = (),
    error_rad: float = 0.0,
) -> MoveArmJoints.Result:
    result = MoveArmJoints.Result()
    result.status = executor.status(outcome, reason, message, executed)
    values = tuple(float(value) for value in positions_rad)
    valid = len(values) == 6
    result.final_positions_rad = list(values if valid else (0.0,) * 6)
    result.final_positions_valid = valid
    result.max_position_error_rad = float(error_rad)
    return result


def joint_feedback(
    goal_handle,
    phase: int,
    positions_rad: Sequence[float],
    error_rad: float,
    started_at_s: float,
) -> None:
    message = MoveArmJoints.Feedback()
    values = tuple(float(value) for value in positions_rad)
    message.phase = int(phase)
    message.current_positions_rad = list(
        values if len(values) == 6 else (0.0,) * 6
    )
    message.max_position_error_rad = float(error_rad)
    message.elapsed_sec = max(0.0, time.monotonic() - started_at_s)
    goal_handle.publish_feedback(message)


def hold_and_finish_joints(
    executor,
    goal_handle,
    adapter,
    owner: int,
    started_at_s: float,
    outcome: int,
    reason: int,
    message: str,
    canceled: bool = False,
) -> MoveArmJoints.Result:
    hold = executor.hold_current_joints(
        adapter,
        lambda positions, error: joint_feedback(
            goal_handle,
            MoveArmJoints.Feedback.HOLDING,
            positions,
            error,
            started_at_s,
        ),
        owner=owner,
    )
    if not hold.confirmed:
        goal_handle.abort()
        return joint_result(
            executor,
            ArmMotionStatus.FATAL_FAILURE,
            ArmMotionStatus.STOP_FAILED,
            f"{message}; hold failed: {hold.message}. Use the hardware emergency stop "
            "if motion continues.",
            True,
            hold.final_positions_rad,
        )
    executor.release_physical_command(adapter.arm, owner)
    if canceled:
        goal_handle.canceled()
    else:
        goal_handle.abort()
    return joint_result(
        executor,
        outcome,
        reason,
        f"{message}; {hold.message}",
        True,
        hold.final_positions_rad,
    )

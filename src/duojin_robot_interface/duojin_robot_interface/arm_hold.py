"""Current-position hold publication and confirmation for arm recovery."""

from __future__ import annotations

import time
from typing import Callable, Optional, TYPE_CHECKING, Tuple

from .arm_domain import ArrivalGate, ArrivalGateConfig, max_joint_error_rad
from .arm_execution_types import HoldResult, MotionFailure
from .arm_lifecycle import PhysicalCommandOwnedError

if TYPE_CHECKING:
    from .arm_sdk_adapter import ArmSdkAdapter


def _hold_preflight(
    executor, adapter: ArmSdkAdapter
) -> tuple[Tuple[float, ...], Optional[HoldResult]]:
    snapshot = adapter.feedback()
    try:
        hold_positions = executor.ensure_fresh_joint_feedback(snapshot, adapter)
    except MotionFailure as exc:
        return tuple(), HoldResult(False, exc.message, tuple())
    if not adapter.target_has_subscriber("joints"):
        return hold_positions, HoldResult(
            False, "joint hold target has no SDK subscriber", hold_positions
        )
    conflicts = adapter.conflicting_publishers()
    if conflicts:
        return hold_positions, HoldResult(
            False,
            "cannot own hold target while conflicting publishers exist: "
            + ", ".join(conflicts),
            hold_positions,
        )
    return hold_positions, None


def _publish_hold_target(
    executor,
    adapter: ArmSdkAdapter,
    hold_positions: Tuple[float, ...],
    owner: Optional[int],
) -> Optional[HoldResult]:
    hold_velocity = (executor.config.hold_joint_velocity_rad_s,) * 6
    try:
        executor.lifecycle.publish_hold(
            adapter.arm,
            lambda: adapter.publish_joint_target(hold_positions, hold_velocity),
            goal_token=owner,
        )
    except PhysicalCommandOwnedError as exc:
        return HoldResult(False, str(exc), hold_positions)
    except Exception as exc:
        return HoldResult(
            False,
            f"joint hold target publication failed: {exc}",
            hold_positions,
        )
    return None


def _confirm_hold(
    executor,
    adapter: ArmSdkAdapter,
    hold_positions: Tuple[float, ...],
    feedback_callback: Optional[Callable[[Tuple[float, ...], float], None]],
) -> HoldResult:
    gate = ArrivalGate(
        ArrivalGateConfig(
            min_samples=3,
            min_continuous_s=executor.config.settle_duration_sec,
        )
    )
    deadline = time.monotonic() + executor.config.hold_timeout_sec
    last_positions = hold_positions
    last_received_at_s = adapter.feedback().joint_received_at_s
    while time.monotonic() < deadline:
        snapshot = adapter.feedback()
        try:
            last_positions = executor.ensure_fresh_joint_feedback(snapshot, adapter)
        except MotionFailure as exc:
            return HoldResult(False, exc.message, last_positions)
        if snapshot.joint_received_at_s == last_received_at_s:
            time.sleep(executor.config.poll_period_sec)
            continue
        last_received_at_s = snapshot.joint_received_at_s
        error_rad = max_joint_error_rad(last_positions, hold_positions)
        if feedback_callback is not None:
            try:
                feedback_callback(last_positions, error_rad)
            except Exception as exc:
                return HoldResult(
                    False,
                    f"hold feedback publication failed: {exc}",
                    last_positions,
                )
        if gate.update(
            error_rad <= executor.config.joint_position_tolerance_rad,
            time.monotonic(),
        ):
            return HoldResult(True, "current-position hold confirmed", last_positions)
        time.sleep(executor.config.poll_period_sec)
    return HoldResult(
        False,
        f"hold was not confirmed within {executor.config.hold_timeout_sec:.2f}s",
        last_positions,
    )


def hold_current_joints(
    executor,
    adapter: ArmSdkAdapter,
    feedback_callback: Optional[Callable[[Tuple[float, ...], float], None]] = None,
    owner: Optional[int] = None,
) -> HoldResult:
    """Send one current-position target and confirm position settles around it."""

    hold_positions, failure = _hold_preflight(executor, adapter)
    if failure is not None:
        return failure
    failure = _publish_hold_target(executor, adapter, hold_positions, owner)
    if failure is not None:
        return failure
    return _confirm_hold(executor, adapter, hold_positions, feedback_callback)


def shutdown_hold_active_arms(executor) -> None:
    """Best-effort non-blocking hold before process exit; never claim hard stop."""

    for arm, _owner in executor.lifecycle.physical_commands():
        adapter = executor.adapters[arm]
        snapshot = adapter.feedback()
        try:
            positions = executor.ensure_fresh_joint_feedback(snapshot, adapter)
            executor.lifecycle.publish_hold(
                arm,
                lambda: adapter.publish_joint_target(
                    positions, (executor.config.hold_joint_velocity_rad_s,) * 6
                ),
            )
            executor.node.get_logger().warning(
                f"Published best-effort {arm}-arm hold during server shutdown"
            )
        except Exception as exc:
            executor.node.get_logger().error(
                f"Could not publish {arm}-arm shutdown hold: {exc}. Use the "
                "hardware emergency stop if motion continues."
            )

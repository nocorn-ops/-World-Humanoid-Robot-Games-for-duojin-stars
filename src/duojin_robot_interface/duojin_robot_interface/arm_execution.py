"""Shared Action execution gates and recovery for both arm command modes."""

import math
import time
from typing import Callable, Dict, Optional, Tuple

from duojin_interfaces.msg import ArmMotionStatus

from .arm_domain import (
    ArmArbiter,
    ArmValidationError,
    is_fresh,
    validate_joint_positions,
)
from .arm_execution_types import ArmExecutionConfig, HoldResult, MotionFailure
from .arm_hold import hold_current_joints, shutdown_hold_active_arms
from .arm_sdk_adapter import ArmFeedbackSnapshot, ArmSdkAdapter
from .arm_lifecycle import (
    ArmExecutionLifecycle,
    PhysicalCommandOwnedError,
    ShutdownRequestedError,
)

class ArmMotionExecutor:
    """Coordinate per-arm ownership, readiness and best-effort hold recovery."""

    def __init__(
        self,
        node,
        adapters: Dict[str, ArmSdkAdapter],
        config: ArmExecutionConfig,
    ) -> None:
        self.node = node
        self.adapters = adapters
        self.config = config
        self.arbiter = ArmArbiter()
        self.lifecycle = ArmExecutionLifecycle()

    def execute_pose(self, goal_handle, arm: str):
        from .arm_pose_action import execute_pose

        owner = id(goal_handle)
        self.lifecycle.mark_running(owner)
        try:
            return execute_pose(self, goal_handle, arm)
        finally:
            self.lifecycle.finish(owner)

    def execute_relative(self, goal_handle, arm: str):
        from .arm_pose_action import execute_relative

        owner = id(goal_handle)
        self.lifecycle.mark_running(owner)
        try:
            return execute_relative(self, goal_handle, arm)
        finally:
            self.lifecycle.finish(owner)

    def execute_joints(self, goal_handle, arm: str):
        from .arm_joint_action import execute_joints

        owner = id(goal_handle)
        self.lifecycle.mark_running(owner)
        try:
            return execute_joints(self, goal_handle, arm)
        finally:
            self.lifecycle.finish(owner)

    def try_admit_goal(self, goal_request) -> bool:
        return self.lifecycle.try_admit(id(goal_request))

    def register_accepted_goal(self, goal_handle) -> None:
        self.lifecycle.register_accepted(id(goal_handle.request), id(goal_handle))

    @property
    def shutdown_requested(self) -> bool:
        return self.lifecycle.shutdown_requested

    def request_shutdown(self) -> bool:
        return self.lifecycle.request_shutdown()

    def wait_for_goal_drain(self, timeout_sec: float) -> bool:
        return self.lifecycle.wait_for_idle(timeout_sec)

    def discard_unhandled_admissions(self) -> int:
        return self.lifecycle.discard_unhandled_admissions()

    def require_running(self) -> None:
        try:
            self.lifecycle.require_running()
        except ShutdownRequestedError as exc:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                str(exc),
            ) from exc

    def publish_motion_target(
        self,
        arm: str,
        owner: int,
        publisher: Callable[[], None],
    ) -> None:
        try:
            self.lifecycle.publish_motion(arm, owner, publisher)
        except ShutdownRequestedError as exc:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                str(exc),
            ) from exc
        except PhysicalCommandOwnedError as exc:
            raise MotionFailure(
                ArmMotionStatus.FATAL_FAILURE,
                ArmMotionStatus.CONTROL_CONFLICT,
                str(exc),
            ) from exc

    def has_physical_command(self, arm: str, owner: int) -> bool:
        return self.lifecycle.has_physical_owner(arm, owner)

    def release_physical_command(self, arm: str, owner: int) -> bool:
        return self.lifecycle.release_physical(arm, owner)

    @staticmethod
    def status(
        outcome: int,
        reason: int,
        message: str,
        executed: bool,
    ) -> ArmMotionStatus:
        status = ArmMotionStatus()
        status.outcome = int(outcome)
        status.reason = int(reason)
        status.message = str(message)
        status.executed = bool(executed)
        return status

    def resolve_timeout(self, requested_sec: float, default_sec: float) -> float:
        if not math.isfinite(requested_sec):
            raise MotionFailure(
                ArmMotionStatus.FATAL_FAILURE,
                ArmMotionStatus.INVALID_GOAL,
                "timeout_sec must be finite",
            )
        if requested_sec < 0.0:
            raise MotionFailure(
                ArmMotionStatus.FATAL_FAILURE,
                ArmMotionStatus.INVALID_GOAL,
                "timeout_sec must be zero (automatic) or positive",
            )
        if requested_sec == 0.0:
            return min(default_sec, self.config.maximum_timeout_sec)
        timeout_sec = requested_sec
        if timeout_sec > self.config.maximum_timeout_sec:
            raise MotionFailure(
                ArmMotionStatus.FATAL_FAILURE,
                ArmMotionStatus.INVALID_GOAL,
                f"timeout_sec={timeout_sec:.3f} exceeds maximum "
                f"{self.config.maximum_timeout_sec:.3f}s",
            )
        return timeout_sec

    def require_ready(
        self,
        adapter: ArmSdkAdapter,
        mode: str,
        require_pose: bool,
    ) -> ArmFeedbackSnapshot:
        self.require_running()
        snapshot = self._wait_for_ready_feedback(adapter, mode, require_pose)
        self.require_running()
        self._require_ready_joint_feedback(snapshot, adapter)
        if require_pose:
            self._require_ready_pose_feedback(snapshot, adapter)
        self._require_target_subscriber(adapter, mode)
        conflicts = adapter.conflicting_publishers()
        if conflicts:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.CONTROL_CONFLICT,
                "unexpected arm target publisher(s): " + ", ".join(conflicts),
            )
        return snapshot

    def _wait_for_ready_feedback(
        self,
        adapter: ArmSdkAdapter,
        mode: str,
        require_pose: bool,
    ) -> ArmFeedbackSnapshot:
        deadline = time.monotonic() + self.config.feedback_wait_timeout_sec
        snapshot = adapter.feedback()
        while time.monotonic() < deadline:
            self.require_running()
            snapshot = adapter.feedback()
            joint_ready = self._joint_feedback_is_ready(snapshot)
            pose_ready = not require_pose or self._pose_feedback_is_ready(snapshot)
            if joint_ready and pose_ready and adapter.target_has_subscriber(mode):
                break
            time.sleep(self.config.poll_period_sec)
        return snapshot

    def _joint_feedback_is_ready(self, snapshot: ArmFeedbackSnapshot) -> bool:
        return snapshot.joint_positions_rad is not None and is_fresh(
            snapshot.joint_received_at_s,
            self.config.feedback_freshness_sec,
        )

    def _pose_feedback_is_ready(self, snapshot: ArmFeedbackSnapshot) -> bool:
        return snapshot.pose is not None and is_fresh(
            snapshot.pose_received_at_s,
            self.config.feedback_freshness_sec,
        )

    def _require_ready_joint_feedback(
        self,
        snapshot: ArmFeedbackSnapshot,
        adapter: ArmSdkAdapter,
    ) -> None:
        if snapshot.joint_positions_rad is None:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                f"no joint feedback received from {adapter.joint_feedback_topic}",
            )
        if not is_fresh(
            snapshot.joint_received_at_s, self.config.feedback_freshness_sec
        ):
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.FEEDBACK_STALE,
                f"joint feedback on {adapter.joint_feedback_topic} is stale",
            )
        try:
            validate_joint_positions(
                snapshot.joint_positions_rad, enforce_operating_limits=False
            )
        except ArmValidationError as exc:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                f"invalid joint feedback: {exc}",
            ) from exc

    def _require_ready_pose_feedback(
        self,
        snapshot: ArmFeedbackSnapshot,
        adapter: ArmSdkAdapter,
    ) -> None:
        if snapshot.pose is None:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                f"no end-effector feedback received from {adapter.pose_feedback_topic}",
            )
        if not is_fresh(
            snapshot.pose_received_at_s, self.config.feedback_freshness_sec
        ):
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.FEEDBACK_STALE,
                f"end-effector feedback on {adapter.pose_feedback_topic} is stale",
            )
        if not snapshot.pose.header.frame_id:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                f"{adapter.pose_feedback_topic} has an empty frame_id; record and "
                "resolve the SDK frame before executing",
            )

    def _require_target_subscriber(
        self,
        adapter: ArmSdkAdapter,
        mode: str,
    ) -> None:
        if not adapter.target_has_subscriber(mode):
            target_topic = (
                adapter.pose_target_topic if mode == "pose" else adapter.joint_target_topic
            )
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                f"the expected unique SDK subscriber is not present on {target_topic}",
            )

    def ensure_fresh_joint_feedback(
        self, snapshot: ArmFeedbackSnapshot, adapter: ArmSdkAdapter
    ) -> Tuple[float, ...]:
        if snapshot.joint_positions_rad is None or not is_fresh(
            snapshot.joint_received_at_s, self.config.feedback_freshness_sec
        ):
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.FEEDBACK_STALE,
                f"joint feedback on {adapter.joint_feedback_topic} became stale",
            )
        try:
            return validate_joint_positions(
                snapshot.joint_positions_rad, enforce_operating_limits=False
            )
        except ArmValidationError as exc:
            raise MotionFailure(
                ArmMotionStatus.RETRYABLE_FAILURE,
                ArmMotionStatus.SDK_NOT_READY,
                f"joint feedback became invalid: {exc}",
            ) from exc

    def hold_current_joints(
        self,
        adapter: ArmSdkAdapter,
        feedback_callback: Optional[Callable[[Tuple[float, ...], float], None]] = None,
        owner: Optional[int] = None,
    ) -> HoldResult:
        return hold_current_joints(
            self, adapter, feedback_callback=feedback_callback, owner=owner
        )

    def shutdown_hold_active_arms(self) -> None:
        shutdown_hold_active_arms(self)

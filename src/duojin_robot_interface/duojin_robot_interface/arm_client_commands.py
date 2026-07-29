"""Public command/query methods mixed into :class:`ArmClient`."""

import math
from typing import Callable, Optional, Sequence

from duojin_interfaces.action import MoveArmJoints, MoveArmPose, MoveArmRelative
from geometry_msgs.msg import PoseStamped, Vector3Stamped

from .arm_client_result import (
    ArmPoseReading,
    ArmResult,
    map_joint_result,
    map_pose_reading,
    map_pose_result,
)


def _timeout(value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("timeout_sec must be finite and non-negative")
    return value


def _wait_seconds(timeout_sec: float) -> float:
    return (timeout_sec if timeout_sec > 0.0 else 60.0) + 10.0


class ArmClientCommands:
    """Goal construction and current-pose query for the blocking client."""

    def move_to(
        self, x: float, y: float, z: float, frame_id: str = "base_link",
        timeout_sec: float = 0.0, execute: bool = False,
        feedback_callback: Optional[Callable] = None,
    ) -> ArmResult:
        timeout_sec = _timeout(timeout_sec)
        goal = MoveArmPose.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = str(frame_id)
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = float(z)
        goal.target_pose.pose.orientation.w = 1.0
        goal.keep_current_orientation = True
        goal.timeout_sec = timeout_sec
        goal.execute = bool(execute)
        return self._run_goal(
            self._runtime.pose_client, self._runtime.pose_cancel_client, goal,
            _wait_seconds(timeout_sec), map_pose_result, feedback_callback,
        )

    def move_by(
        self, dx: float, dy: float, dz: float, frame_id: str = "base_link",
        timeout_sec: float = 0.0, execute: bool = False,
        feedback_callback: Optional[Callable] = None,
    ) -> ArmResult:
        timeout_sec = _timeout(timeout_sec)
        goal = MoveArmRelative.Goal()
        goal.delta = Vector3Stamped()
        goal.delta.header.frame_id = str(frame_id)
        goal.delta.vector.x = float(dx)
        goal.delta.vector.y = float(dy)
        goal.delta.vector.z = float(dz)
        goal.keep_current_orientation = True
        goal.timeout_sec = timeout_sec
        goal.execute = bool(execute)
        return self._run_goal(
            self._runtime.relative_client, self._runtime.relative_cancel_client, goal,
            _wait_seconds(timeout_sec), map_pose_result, feedback_callback,
        )

    def move_joints(
        self, positions_rad: Sequence[float], speed_scale: float = 0.2,
        timeout_sec: float = 0.0, execute: bool = False,
        feedback_callback: Optional[Callable] = None,
    ) -> ArmResult:
        if len(positions_rad) != 6:
            raise ValueError("positions_rad must contain joint1..joint6 exactly")
        timeout_sec = _timeout(timeout_sec)
        goal = MoveArmJoints.Goal()
        goal.positions_rad = [float(value) for value in positions_rad]
        goal.speed_scale = float(speed_scale)
        goal.timeout_sec = timeout_sec
        goal.execute = bool(execute)
        return self._run_goal(
            self._runtime.joint_client, self._runtime.joint_cancel_client, goal,
            _wait_seconds(timeout_sec), map_joint_result, feedback_callback,
        )

    def get_pose(
        self, timeout_sec: float = 2.0, max_age_sec: float = 0.25
    ) -> ArmPoseReading:
        timeout_sec = float(timeout_sec)
        max_age_sec = float(max_age_sec)
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (timeout_sec, max_age_sec)
        ):
            raise ValueError("timeout_sec and max_age_sec must be positive")
        with self._state_lock:
            if self._closed:
                raise RuntimeError("ArmClient is closed")
        return map_pose_reading(
            self._runtime.wait_current_pose(timeout_sec, max_age_sec)
        )

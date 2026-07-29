"""Publish fresh, frame-normalized end-effector poses for public consumers."""

import time

from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .arm_domain import is_fresh, validate_cartesian_target


def _pose_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class ArmPoseStream:
    """Republish only new, fresh SDK poses in one documented public frame."""

    def __init__(self, node, adapters, config) -> None:
        self.node = node
        self.adapters = adapters
        self.config = config
        self._last_source_time = {"left": None, "right": None}
        self._last_warning_time = {"left": 0.0, "right": 0.0}
        self.publishers = {
            arm: node.create_publisher(
                PoseStamped, f"/duojin/arm/{arm}/current_pose", _pose_qos()
            )
            for arm in ("left", "right")
        }
        self.timer = node.create_timer(
            config.pose_publish_period_sec, self._publish_fresh_poses
        )

    def _warn(self, arm: str, message: str) -> None:
        now_s = time.monotonic()
        if now_s - self._last_warning_time[arm] < 2.0:
            return
        self._last_warning_time[arm] = now_s
        self.node.get_logger().warning(f"{arm} current_pose unavailable: {message}")

    def _feedback_is_fresh(self, snapshot) -> bool:
        max_age_s = self.config.feedback_freshness_sec
        return (
            snapshot.pose is not None
            and is_fresh(snapshot.pose_received_at_s, max_age_s)
            and is_fresh(snapshot.joint_received_at_s, max_age_s)
        )

    @staticmethod
    def _normalize_pose(pose: PoseStamped) -> None:
        position = pose.pose.position
        orientation = pose.pose.orientation
        target = validate_cartesian_target(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
        )
        orientation.x = target.orientation_xyzw[0]
        orientation.y = target.orientation_xyzw[1]
        orientation.z = target.orientation_xyzw[2]
        orientation.w = target.orientation_xyzw[3]

    def _publish_arm(self, arm: str) -> None:
        adapter = self.adapters[arm]
        snapshot = adapter.feedback()
        if not self._feedback_is_fresh(snapshot):
            self._warn(arm, "joint or end-effector feedback is missing/stale")
            return
        if snapshot.pose_received_at_s == self._last_source_time[arm]:
            return
        try:
            pose = adapter.transform_pose(
                snapshot.pose,
                self.config.public_pose_frame,
                self.config.tf_timeout_sec,
            )
            self._normalize_pose(pose)
        except Exception as exc:
            self._warn(arm, f"TF/pose validation failed: {exc}")
            return
        pose.header.stamp = self.node.get_clock().now().to_msg()
        self.publishers[arm].publish(pose)
        self._last_source_time[arm] = snapshot.pose_received_at_s

    def _publish_fresh_poses(self) -> None:
        for arm in ("left", "right"):
            self._publish_arm(arm)

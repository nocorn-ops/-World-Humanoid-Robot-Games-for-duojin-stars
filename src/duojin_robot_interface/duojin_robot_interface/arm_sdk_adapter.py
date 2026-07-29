"""Thin, auditable boundary around the Galaxea arm topics and TF tree."""

from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time
from typing import Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
import tf2_geometry_msgs  # noqa: F401 - registers geometry transforms with tf2.
from tf2_ros import Buffer


def _topic_qos(reliability: ReliabilityPolicy, depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


@dataclass(frozen=True)
class ArmFeedbackSnapshot:
    """Latest feedback copied under a lock, with local monotonic receipt times."""

    joint_positions_rad: Optional[Tuple[float, ...]]
    joint_received_at_s: Optional[float]
    pose: Optional[PoseStamped]
    pose_received_at_s: Optional[float]
    joint_target_positions_rad: Optional[Tuple[float, ...]]
    joint_target_received_at_s: Optional[float]


class ArmSdkAdapter:
    """Own the vendor topic names, QoS choices, TF and graph checks for one arm."""

    def __init__(
        self,
        node: Node,
        arm: str,
        tf_buffer: Buffer,
        allowed_joint_publishers: Sequence[str],
    ) -> None:
        if arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")

        self.node = node
        self.arm = arm
        self.tf_buffer = tf_buffer
        self.allowed_joint_publishers = frozenset(allowed_joint_publishers)

        self.joint_feedback_topic = f"/hdas/feedback_arm_{arm}"
        self.pose_feedback_topic = f"/motion_control/pose_ee_arm_{arm}"
        self.joint_target_topic = f"/motion_target/target_joint_state_arm_{arm}"
        self.pose_target_topic = f"/motion_target/target_pose_arm_{arm}"

        self._feedback_lock = threading.Lock()
        self._joint_positions_rad: Optional[Tuple[float, ...]] = None
        self._joint_received_at_s: Optional[float] = None
        self._pose: Optional[PoseStamped] = None
        self._pose_received_at_s: Optional[float] = None
        self._joint_target_positions_rad: Optional[Tuple[float, ...]] = None
        self._joint_target_received_at_s: Optional[float] = None
        self._create_endpoints()

    def _create_endpoints(self) -> None:
        feedback_qos = _topic_qos(ReliabilityPolicy.BEST_EFFORT, depth=1)
        pose_target_qos = _topic_qos(ReliabilityPolicy.BEST_EFFORT, depth=1)
        # RELIABLE can serve the tracker's BEST_EFFORT reader. VOLATILE avoids
        # replaying a stale arm goal when a controller restarts.
        joint_target_qos = _topic_qos(ReliabilityPolicy.RELIABLE, depth=1)
        observed_joint_target_qos = _topic_qos(
            ReliabilityPolicy.BEST_EFFORT, depth=10
        )

        self.pose_target_publisher = self.node.create_publisher(
            PoseStamped, self.pose_target_topic, pose_target_qos
        )
        self.joint_target_publisher = self.node.create_publisher(
            JointState, self.joint_target_topic, joint_target_qos
        )
        self._joint_feedback_subscription = self.node.create_subscription(
            JointState,
            self.joint_feedback_topic,
            self._on_joint_feedback,
            feedback_qos,
        )
        self._pose_feedback_subscription = self.node.create_subscription(
            PoseStamped,
            self.pose_feedback_topic,
            self._on_pose_feedback,
            feedback_qos,
        )
        self._joint_target_subscription = self.node.create_subscription(
            JointState,
            self.joint_target_topic,
            self._on_joint_target,
            observed_joint_target_qos,
        )

    def _on_joint_feedback(self, message: JointState) -> None:
        positions = tuple(float(value) for value in message.position[:6])
        with self._feedback_lock:
            self._joint_positions_rad = positions
            self._joint_received_at_s = time.monotonic()

    def _on_pose_feedback(self, message: PoseStamped) -> None:
        with self._feedback_lock:
            self._pose = deepcopy(message)
            self._pose_received_at_s = time.monotonic()

    def _on_joint_target(self, message: JointState) -> None:
        with self._feedback_lock:
            self._joint_target_positions_rad = tuple(
                float(value) for value in message.position[:6]
            )
            self._joint_target_received_at_s = time.monotonic()

    def feedback(self) -> ArmFeedbackSnapshot:
        with self._feedback_lock:
            return ArmFeedbackSnapshot(
                joint_positions_rad=self._joint_positions_rad,
                joint_received_at_s=self._joint_received_at_s,
                pose=deepcopy(self._pose),
                pose_received_at_s=self._pose_received_at_s,
                joint_target_positions_rad=self._joint_target_positions_rad,
                joint_target_received_at_s=self._joint_target_received_at_s,
            )

    def target_has_subscriber(self, mode: str) -> bool:
        topic = self._target_topic(mode)
        expected = self._expected_target_subscriber(mode)
        matches = [
            info
            for info in self.node.get_subscriptions_info_by_topic(topic)
            if self._endpoint_node_name(info) == expected
        ]
        return len(matches) == 1

    def _expected_target_subscriber(self, mode: str) -> str:
        if mode == "pose":
            return f"/relaxed_ik_{self.arm}"
        if mode == "joints":
            return "/r1_lite_jointTracker_demo_node"
        raise ValueError(f"unsupported arm mode: {mode}")

    def _target_topic(self, mode: str) -> str:
        if mode == "pose":
            return self.pose_target_topic
        if mode == "joints":
            return self.joint_target_topic
        raise ValueError(f"unsupported arm mode: {mode}")

    @staticmethod
    def _endpoint_node_name(endpoint_info) -> str:
        namespace = endpoint_info.node_namespace or "/"
        if not namespace.startswith("/"):
            namespace = "/" + namespace
        namespace = namespace.rstrip("/")
        return f"{namespace}/{endpoint_info.node_name}"

    def conflicting_publishers(self) -> Tuple[str, ...]:
        """Return unexpected target publishers that can contend with this server."""

        own_name = self.node.get_fully_qualified_name()
        conflicts = self._publishers_beyond_allowance(
            self.pose_target_topic, {own_name: 1}
        )
        allowed_joint = {name: 1 for name in self.allowed_joint_publishers}
        allowed_joint[own_name] = 1
        conflicts.extend(
            self._publishers_beyond_allowance(
                self.joint_target_topic, allowed_joint
            )
        )
        return tuple(sorted(conflicts))

    def _publishers_beyond_allowance(self, topic: str, allowance) -> list[str]:
        seen = {}
        conflicts = []
        for info in self.node.get_publishers_info_by_topic(topic):
            name = self._endpoint_node_name(info)
            seen[name] = seen.get(name, 0) + 1
            if seen[name] > allowance.get(name, 0):
                conflicts.append(f"{name} endpoint#{seen[name]} -> {topic}")
        return conflicts

    def transform_pose(
        self,
        source_pose: PoseStamped,
        target_frame: str,
        timeout_sec: float,
    ) -> PoseStamped:
        """Transform with latest TF; vendor header timestamps are not trusted."""

        if not source_pose.header.frame_id:
            raise ValueError("pose feedback/goal has an empty frame_id")
        if not target_frame:
            raise ValueError("target frame_id is empty")

        pose = deepcopy(source_pose)
        pose.header.stamp = Time().to_msg()
        if pose.header.frame_id == target_frame:
            pose.header.frame_id = target_frame
            return pose
        transformed = self.tf_buffer.transform(
            pose,
            target_frame,
            timeout=Duration(seconds=timeout_sec),
        )
        transformed.header.frame_id = target_frame
        return transformed

    def publish_pose_target(self, target_solver: PoseStamped) -> None:
        command = deepcopy(target_solver)
        command.header.stamp = self.node.get_clock().now().to_msg()
        self.pose_target_publisher.publish(command)

    def publish_joint_target(
        self,
        positions_rad: Sequence[float],
        velocity_limits_rad_s: Sequence[float],
    ) -> None:
        if len(positions_rad) != 6 or len(velocity_limits_rad_s) != 6:
            raise ValueError("joint command requires exactly 6 positions and velocities")
        if not all(math.isfinite(value) for value in positions_rad):
            raise ValueError("joint command positions must be finite")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in velocity_limits_rad_s
        ):
            raise ValueError("joint command velocities must be finite and positive")

        command = JointState()
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.position = [float(value) for value in positions_rad]
        command.velocity = [float(value) for value in velocity_limits_rad_s]
        self.joint_target_publisher.publish(command)

    def new_ik_command_after(
        self, published_after_s: float
    ) -> Optional[Tuple[float, ...]]:
        snapshot = self.feedback()
        received = snapshot.joint_target_received_at_s
        if received is None or received <= published_after_s:
            return None
        return snapshot.joint_target_positions_rad


def initialize_tf(node: Node) -> tuple[Buffer, object]:
    """Create Buffer/TransformListener together so the listener stays alive."""

    from tf2_ros import TransformListener

    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    return buffer, listener

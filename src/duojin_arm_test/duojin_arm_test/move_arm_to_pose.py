#!/usr/bin/env python3
"""Preview a small Cartesian target for the Galaxea Relaxed IK chain.

The node reads the current end-effector pose produced by ``relaxed_ik``, keeps
the current orientation, and previews a small offset. It does not publish a
motion target; physical Cartesian tests use the unified arm API instead.
"""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ArmCartesianNudge(Node):
    """Validate the Pose -> Relaxed IK -> Joint Tracker control path."""

    def __init__(self) -> None:
        super().__init__("arm_cartesian_nudge")

        self.declare_parameter("arm", "left")
        self.declare_parameter("delta_x", 0.0)
        self.declare_parameter("delta_y", 0.0)
        self.declare_parameter("delta_z", 0.03)
        self.declare_parameter("max_delta_m", 0.08)
        self.declare_parameter("execute", False)
        self.declare_parameter("timeout_sec", 15.0)

        self.arm = str(self.get_parameter("arm").value).lower()
        if self.arm not in ("left", "right"):
            raise ValueError("parameter 'arm' must be 'left' or 'right'")

        self.delta_x = float(self.get_parameter("delta_x").value)
        self.delta_y = float(self.get_parameter("delta_y").value)
        self.delta_z = float(self.get_parameter("delta_z").value)
        self.max_delta_m = float(self.get_parameter("max_delta_m").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.solver_frame = "torso_link3"

        self._validate_parameters()

        self.target_topic = f"/motion_target/target_pose_arm_{self.arm}"
        self.current_pose_topic = (
            f"/relaxed_ik/motion_control/pose_ee_arm_{self.arm}"
        )

        current_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            self.current_pose_topic,
            self._on_current_pose,
            current_pose_qos,
        )
        self.current_pose: Optional[PoseStamped] = None

    def _validate_parameters(self) -> None:
        delta_norm = math.sqrt(
            self.delta_x**2 + self.delta_y**2 + self.delta_z**2
        )
        if delta_norm <= 0.0:
            raise ValueError("Cartesian offset must be non-zero")
        if self.max_delta_m <= 0.0 or delta_norm > self.max_delta_m:
            raise ValueError(
                f"requested offset {delta_norm:.3f} m exceeds "
                f"max_delta_m={self.max_delta_m:.3f} m"
            )
        if self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if self.execute:
            raise RuntimeError(
                "Cartesian diagnostic execution is disabled: Relaxed IK publishes "
                "joint targets before project-side validation"
            )

    def _on_current_pose(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def wait_until_ready(self) -> bool:
        """Wait for current FK and for a Relaxed IK target subscriber."""
        deadline = time.monotonic() + self.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if self.current_pose is not None and self.count_subscribers(self.target_topic) > 0:
                return True

        if self.current_pose is None:
            self.get_logger().error(
                f"No current end-effector pose received from {self.current_pose_topic}. "
                f"Start the {self.arm}-arm Relaxed IK node and verify arm feedback."
            )
        if self.count_subscribers(self.target_topic) == 0:
            self.get_logger().error(
                f"No subscriber on {self.target_topic}. The Relaxed IK node is not ready."
            )
        return False

    def build_target(self) -> PoseStamped:
        if self.current_pose is None:
            raise RuntimeError("current end-effector pose is unavailable")

        current = self.current_pose.pose
        quaternion_norm = math.sqrt(
            current.orientation.x**2
            + current.orientation.y**2
            + current.orientation.z**2
            + current.orientation.w**2
        )
        if quaternion_norm < 1.0e-6:
            raise RuntimeError("current end-effector orientation is not a valid quaternion")

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.solver_frame
        target.pose.position.x = current.position.x + self.delta_x
        target.pose.position.y = current.position.y + self.delta_y
        target.pose.position.z = current.position.z + self.delta_z
        target.pose.orientation.x = current.orientation.x / quaternion_norm
        target.pose.orientation.y = current.orientation.y / quaternion_norm
        target.pose.orientation.z = current.orientation.z / quaternion_norm
        target.pose.orientation.w = current.orientation.w / quaternion_norm
        return target

    def run(self) -> bool:
        self.get_logger().info(
            "Waiting for the current end-effector pose and Relaxed IK subscriber..."
        )
        if not self.wait_until_ready():
            return False

        target = self.build_target()
        current = self.current_pose.pose.position
        goal = target.pose.position
        self.get_logger().info(
            f"Current {self.arm} EE solver coordinates in {self.solver_frame}: "
            f"({current.x:.4f}, {current.y:.4f}, {current.z:.4f}) m"
        )
        self.get_logger().info(
            f"Selected target: ({goal.x:.4f}, {goal.y:.4f}, {goal.z:.4f}) m; "
            f"offset=({self.delta_x:.3f}, {self.delta_y:.3f}, "
            f"{self.delta_z:.3f}) m; orientation unchanged"
        )

        self.get_logger().warning(
            "PREVIEW ONLY: this legacy diagnostic published no command. Use the "
            "unified arm API for explicitly authorized Cartesian execution."
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[ArmCartesianNudge] = None
    exit_code = 0
    try:
        node = ArmCartesianNudge()
        if not node.run():
            exit_code = 2
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning("Interrupted by operator")
        exit_code = 130
    except Exception as exc:  # Keep command-line failures explicit for field tests.
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"arm_cartesian_nudge: {exc}")
        exit_code = 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

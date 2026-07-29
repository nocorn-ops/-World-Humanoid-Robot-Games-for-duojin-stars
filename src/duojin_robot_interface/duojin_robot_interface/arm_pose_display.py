"""Terminal display for the public end-effector pose stream."""

import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class ArmPoseDisplay(Node):
    def __init__(self) -> None:
        super().__init__("arm_pose_display")
        self.declare_parameter("arm", "left")
        self.declare_parameter("display_rate_hz", 10.0)
        arm = str(self.get_parameter("arm").value).lower()
        rate_hz = float(self.get_parameter("display_rate_hz").value)
        if arm not in ("left", "right"):
            raise ValueError("arm must be left or right")
        if rate_hz <= 0.0:
            raise ValueError("display_rate_hz must be positive")
        self._minimum_period_s = 1.0 / rate_hz
        self._last_print_s = 0.0
        self.received = False
        self.subscription = self.create_subscription(
            PoseStamped,
            f"/duojin/arm/{arm}/current_pose",
            self._display,
            _qos(),
        )

    def _display(self, message: PoseStamped) -> None:
        now_s = time.monotonic()
        self.received = True
        if now_s - self._last_print_s < self._minimum_period_s:
            return
        self._last_print_s = now_s
        p = message.pose.position
        q = message.pose.orientation
        stamp = message.header.stamp
        print(
            f"frame={message.header.frame_id} "
            f"xyz[m]=({p.x:+.6f}, {p.y:+.6f}, {p.z:+.6f}) "
            f"q=({q.x:+.6f}, {q.y:+.6f}, {q.z:+.6f}, {q.w:+.6f}) "
            f"stamp={stamp.sec}.{stamp.nanosec:09d}",
            flush=True,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ArmPoseDisplay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

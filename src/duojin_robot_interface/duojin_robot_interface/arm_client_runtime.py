"""Private ROS context, executor and callback isolation for :mod:`arm_client`."""

from copy import deepcopy
from dataclasses import dataclass, field
import queue
import threading
import time
from typing import Any, Callable, Optional
import uuid

from action_msgs.srv import CancelGoal
from duojin_interfaces.action import MoveArmJoints, MoveArmPose, MoveArmRelative
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from unique_identifier_msgs.msg import UUID


@dataclass
class ClientOperation:
    action_client: Any
    cancel_client: Any
    mapper: Callable
    goal_uuid: UUID = field(
        default_factory=lambda: UUID(uuid=list(uuid.uuid4().bytes))
    )
    cancel_lock: threading.Lock = field(default_factory=threading.Lock)
    caller_done: threading.Event = field(default_factory=threading.Event)
    cancel_requested: bool = False
    sent: bool = False
    send_future: Any = None
    goal_handle: Any = None
    result_future: Any = None
    cancel_result: Any = None


class ArmClientRuntime:
    """Own ROS resources without touching the process default rclpy context."""

    def __init__(self, arm: str) -> None:
        self.arm = arm
        self._spin_lock = threading.Lock()
        self._spin_error: Optional[BaseException] = None
        self._executor_shutdown_failed = False
        self._feedback_queue: queue.Queue = queue.Queue(maxsize=16)
        self._feedback_stopping = threading.Event()
        self._pose_condition = threading.Condition()
        self._current_pose = None
        self._current_pose_received_s = None
        self.context = Context()
        rclpy.init(
            context=self.context,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        self._create_ros_entities()
        self._start_workers()

    def _create_ros_entities(self) -> None:
        suffix = uuid.uuid4().hex[:8]
        self.node = Node(
            f"duojin_{self.arm}_arm_client_{suffix}", context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        namespace = f"/duojin/arm/{self.arm}"
        pose_name = f"{namespace}/move_to"
        relative_name = f"{namespace}/move_by"
        joint_name = f"{namespace}/move_joints"
        self.pose_client = ActionClient(self.node, MoveArmPose, pose_name)
        self.relative_client = ActionClient(
            self.node, MoveArmRelative, relative_name
        )
        self.joint_client = ActionClient(self.node, MoveArmJoints, joint_name)
        self.pose_cancel_client = self.node.create_client(
            CancelGoal, f"{pose_name}/_action/cancel_goal"
        )
        self.joint_cancel_client = self.node.create_client(
            CancelGoal, f"{joint_name}/_action/cancel_goal"
        )
        self.relative_cancel_client = self.node.create_client(
            CancelGoal, f"{relative_name}/_action/cancel_goal"
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_subscription = self.node.create_subscription(
            PoseStamped,
            f"{namespace}/current_pose",
            self._on_current_pose,
            qos,
        )

    def _on_current_pose(self, message: PoseStamped) -> None:
        with self._pose_condition:
            self._current_pose = deepcopy(message)
            self._current_pose_received_s = time.monotonic()
            self._pose_condition.notify_all()

    def wait_current_pose(self, timeout_sec: float, max_age_sec: float) -> PoseStamped:
        deadline = time.monotonic() + timeout_sec
        with self._pose_condition:
            while True:
                now_s = time.monotonic()
                if (
                    self._current_pose is not None
                    and self._current_pose_received_s is not None
                    and now_s - self._current_pose_received_s <= max_age_sec
                ):
                    return deepcopy(self._current_pose)
                remaining = deadline - now_s
                if remaining <= 0.0:
                    raise TimeoutError("no fresh current_pose was received")
                self._pose_condition.wait(min(0.1, remaining))
                self._raise_if_runtime_stopped()

    def _start_workers(self) -> None:
        self._feedback_thread = threading.Thread(
            target=self._feedback_loop,
            name=f"duojin-{self.arm}-arm-feedback",
            daemon=True,
        )
        self._spin_thread = threading.Thread(
            target=self._spin,
            name=f"duojin-{self.arm}-arm-client",
            daemon=True,
        )
        self._feedback_thread.start()
        self._spin_thread.start()

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except BaseException as exc:  # executor failures must wake blocking callers
            with self._spin_lock:
                self._spin_error = exc

    def _feedback_loop(self) -> None:
        while not self._feedback_stopping.is_set():
            try:
                item = self._feedback_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            callback, message = item
            if self._feedback_stopping.is_set():
                continue
            try:
                callback(message)
            except Exception as exc:
                self.node.get_logger().warning(f"arm feedback callback failed: {exc}")

    def _queue_feedback(self, callback: Callable, message) -> None:
        if self._feedback_stopping.is_set():
            return
        try:
            self._feedback_queue.put_nowait((callback, message))
        except queue.Full:
            self._replace_oldest_feedback(callback, message)

    def _replace_oldest_feedback(self, callback: Callable, message) -> None:
        try:
            self._feedback_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._feedback_queue.put_nowait((callback, message))
        except queue.Full:
            pass

    def feedback_proxy(self, callback: Optional[Callable]) -> Optional[Callable]:
        if callback is None:
            return None
        return lambda message: self._queue_feedback(callback, message)

    def wait_future(self, future, timeout_sec: float):
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while not future.done() and not future.cancelled():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                if future.done() or future.cancelled():
                    break
                raise TimeoutError(
                    f"ROS Action future did not complete within {timeout_sec:.1f}s"
                )
            completed.wait(min(0.1, remaining))
            if future.done() or future.cancelled():
                break
            self._raise_if_runtime_stopped()
        if future.cancelled():
            raise RuntimeError("ROS Action future was cancelled before completion")
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    def wait_for_server(
        self, action_client: ActionClient, timeout_sec: float, should_stop: Callable
    ) -> str:
        """Poll server readiness so a concurrent cancel/close is prompt."""

        deadline = time.monotonic() + timeout_sec
        while self.context.ok() and time.monotonic() < deadline:
            if should_stop():
                return "stopped"
            if action_client.server_is_ready():
                return "ready"
            time.sleep(0.05)
        return "unavailable"

    def _raise_if_runtime_stopped(self) -> None:
        with self._spin_lock:
            spin_error = self._spin_error
        if spin_error is not None:
            raise RuntimeError(f"ROS executor stopped unexpectedly: {spin_error}")
        if not self.context.ok():
            raise RuntimeError("ROS context stopped before Action completion")

    def _stop_feedback_worker(self) -> None:
        self._feedback_stopping.set()
        if threading.current_thread() is self._feedback_thread:
            return
        self._feedback_thread.join(timeout=1.0)
        if self._feedback_thread.is_alive():
            raise RuntimeError(
                "feedback callback did not stop; ROS resources were kept alive"
            )

    def close(self) -> None:
        self._stop_feedback_worker()
        if self._executor_shutdown_failed:
            raise RuntimeError(
                "ROS executor previously failed to stop; entities remain alive until "
                "process exit"
            )
        stopped = self.executor.shutdown(timeout_sec=2.0)
        if not stopped:
            self._executor_shutdown_failed = True
            self.context.try_shutdown()
            self._spin_thread.join(timeout=2.0)
            raise RuntimeError(
                "ROS executor shutdown timed out; ROS resources were kept alive"
            )
        self._spin_thread.join(timeout=2.0)
        if self._spin_thread.is_alive():
            self._executor_shutdown_failed = True
            raise RuntimeError(
                "ROS executor did not stop; ROS resources were kept alive"
            )
        self.pose_client.destroy()
        self.relative_client.destroy()
        self.joint_client.destroy()
        self.node.destroy_client(self.pose_cancel_client)
        self.node.destroy_client(self.relative_cancel_client)
        self.node.destroy_client(self.joint_cancel_client)
        self.node.destroy_subscription(self.pose_subscription)
        self.node.destroy_node()
        self.context.try_shutdown()

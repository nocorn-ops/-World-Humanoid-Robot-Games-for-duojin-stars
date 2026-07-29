"""ROS 2 Action server exposing safe left/right R1 Lite arm capabilities."""

from functools import partial
import fcntl
import os
import re
import socket
import sys
import time

from duojin_interfaces.action import MoveArmJoints, MoveArmPose, MoveArmRelative
import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .arm_execution import ArmMotionExecutor
from .arm_execution_types import ArmExecutionConfig
from .arm_pose_stream import ArmPoseStream
from .arm_server_config import declare_arm_parameters, read_arm_config
from .arm_sdk_adapter import ArmSdkAdapter, initialize_tf


_SINGLETON_LOCK_PATH = "/tmp/duojin_arm_motion_server.lock"


def _server_node_name() -> str:
    host = re.sub(r"[^A-Za-z0-9_]", "_", socket.gethostname()) or "host"
    return f"arm_motion_server_{host}_{os.getpid()}"


class ArmMotionServer(Node):
    """Own six public Actions while sharing one safety arbiter per arm."""

    def __init__(self) -> None:
        # Host/PID suffixes let graph checks distinguish another API process,
        # including one accidentally started on a second computer.
        super().__init__(_server_node_name())
        declare_arm_parameters(self)
        config = read_arm_config(self)
        tf_buffer, self._tf_listener = initialize_tf(self)
        adapters = self._create_adapters(tf_buffer, config.ik_solver_frame)
        self.motion = ArmMotionExecutor(self, adapters, config)
        self.pose_stream = ArmPoseStream(self, adapters, config)
        self._action_group = ReentrantCallbackGroup()
        self._action_servers = self._create_action_servers()
        self._log_ready(config)

    def _create_adapters(
        self, tf_buffer, ik_solver_frame: str
    ) -> dict[str, ArmSdkAdapter]:
        return {
            "left": ArmSdkAdapter(
                self,
                "left",
                tf_buffer,
                self.get_parameter("allowed_left_joint_publishers").value,
                ik_solver_frame,
            ),
            "right": ArmSdkAdapter(
                self,
                "right",
                tf_buffer,
                self.get_parameter("allowed_right_joint_publishers").value,
                ik_solver_frame,
            ),
        }

    def _create_action_servers(self) -> list[ActionServer]:
        servers = []
        for arm in ("left", "right"):
            namespace = f"/duojin/arm/{arm}"
            servers.append(
                ActionServer(
                    self,
                    MoveArmPose,
                    f"{namespace}/move_to",
                    execute_callback=partial(self.motion.execute_pose, arm=arm),
                    goal_callback=self._accept_goal,
                    handle_accepted_callback=self._handle_accepted,
                    cancel_callback=self._accept_cancel,
                    callback_group=self._action_group,
                )
            )
            servers.append(
                ActionServer(
                    self,
                    MoveArmRelative,
                    f"{namespace}/move_by",
                    execute_callback=partial(self.motion.execute_relative, arm=arm),
                    goal_callback=self._accept_goal,
                    handle_accepted_callback=self._handle_accepted,
                    cancel_callback=self._accept_cancel,
                    callback_group=self._action_group,
                )
            )
            servers.append(
                ActionServer(
                    self,
                    MoveArmJoints,
                    f"{namespace}/move_joints",
                    execute_callback=partial(self.motion.execute_joints, arm=arm),
                    goal_callback=self._accept_goal,
                    handle_accepted_callback=self._handle_accepted,
                    cancel_callback=self._accept_cancel,
                    callback_group=self._action_group,
                )
            )
        return servers

    def _log_ready(self, config: ArmExecutionConfig) -> None:
        mode = "EXECUTION ENABLED" if config.execute else "PREVIEW ONLY"
        log = self.get_logger().warning if config.execute else self.get_logger().info
        log(
            f"Arm API ready: {mode}. Endpoints are /duojin/arm/<left|right>/"
            "<move_to|move_by|move_joints>; current_pose topics are active."
        )
        if config.execute:
            self.get_logger().warning(
                "Real arm goals can now reach the SDK. Keep the emergency stop ready; "
                "this API does not perform collision planning."
            )
            self.get_logger().warning(
                "Cartesian execution uses vendor Relaxed IK; its joint output is "
                "validated only after the vendor has published it to Joint Tracker."
            )

    def _accept_goal(self, goal_request) -> GoalResponse:
        # Execution callbacks perform atomic BUSY admission and return a
        # machine-readable result. Direct ROS rejection cannot carry BUSY.
        if self.motion.try_admit_goal(goal_request):
            return GoalResponse.ACCEPT
        self.get_logger().warning("Rejecting arm goal because server shutdown has begun")
        return GoalResponse.REJECT

    def _handle_accepted(self, goal_handle) -> None:
        """Account a queued callback before asking rclpy to execute it."""

        self.motion.register_accepted_goal(goal_handle)
        try:
            goal_handle.execute()
        except Exception:
            self.motion.lifecycle.finish(id(goal_handle))
            raise

    @staticmethod
    def _accept_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def begin_shutdown(self) -> None:
        """Close publication admission before sending any shutdown hold."""

        if self.motion.request_shutdown():
            self.get_logger().warning("Arm API shutdown gate is now closed")
        self.motion.shutdown_hold_active_arms()

    def destroy_action_servers(self) -> None:
        for server in self._action_servers:
            server.destroy()


def _acquire_singleton_lock():
    lock_file = open(_SINGLETON_LOCK_PATH, "a+", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        raise RuntimeError(
            "another duojin arm motion server holds " + _SINGLETON_LOCK_PATH
        )
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(f"{os.getpid()}\n")
    lock_file.flush()
    return lock_file


def _drain_goal_callbacks(
    node: ArmMotionServer,
    ros_executor: MultiThreadedExecutor,
    timeout_sec: float,
) -> bool:
    """Keep ROS feedback flowing while shutdown callbacks hold and finish."""

    deadline = time.monotonic() + timeout_sec
    while True:
        snapshot = node.motion.lifecycle.snapshot()
        if not (
            snapshot.admitted_count
            or snapshot.queued_count
            or snapshot.running_count
        ):
            return True
        if not ros_executor.context.ok():
            return False
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        # MultiThreadedExecutor schedules one ready callback per spin_once. Keep
        # taking joint/pose feedback while Action workers perform their hold.
        ros_executor.spin_once(timeout_sec=min(0.05, remaining))


def _join_executor_workers(ros_executor: MultiThreadedExecutor) -> None:
    """Join Humble's private worker pool before destroying callback entities.

    Humble's MultiThreadedExecutor.shutdown() drains its work tracker but does
    not join the underlying ThreadPoolExecutor. At this point Action lifecycle
    accounting is idle and the ROS executor has been shut down, so no new work
    can be submitted. Joining prevents a queued feedback handler from racing
    node destruction.
    """

    worker_pool = getattr(ros_executor, "_executor", None)
    if worker_pool is not None:
        worker_pool.shutdown(wait=True)


def _shutdown_server(
    node: ArmMotionServer, ros_executor: MultiThreadedExecutor
) -> None:
    node.begin_shutdown()
    drain_timeout = node.motion.config.shutdown_drain_timeout_sec
    try:
        goals_drained = _drain_goal_callbacks(node, ros_executor, drain_timeout)
    except ExternalShutdownException:
        goals_drained = node.motion.wait_for_goal_drain(0.0)
    executor_drained = ros_executor.shutdown(timeout_sec=1.0)
    if executor_drained:
        dropped = node.motion.discard_unhandled_admissions()
        if dropped:
            node.get_logger().warning(
                f"Dropped {dropped} admitted goal request(s) that never received "
                "an accepted goal handle"
            )
            goals_drained = node.motion.wait_for_goal_drain(0.0)
    if not goals_drained:
        snapshot = node.motion.lifecycle.snapshot()
        node.get_logger().error(
            "Arm Action callbacks did not drain before shutdown timeout: "
            f"admitted={snapshot.admitted_count}, queued={snapshot.queued_count}, "
            f"running={snapshot.running_count}. Hardware emergency stop remains "
            "the authoritative stop."
        )
    if goals_drained and executor_drained:
        _join_executor_workers(ros_executor)
        node.destroy_action_servers()
        ros_executor.remove_node(node)
        node.destroy_node()
    else:
        node.get_logger().error(
            "Skipping ROS entity destruction because callback workers may still "
            "be using them; process exit will reclaim the entities."
        )


def main(args=None) -> None:
    try:
        singleton_lock = _acquire_singleton_lock()
    except RuntimeError as exc:
        print(f"arm_motion_server: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc

    # Keep the ROS context valid when Python's default SIGINT handler raises
    # KeyboardInterrupt. The shutdown path can then publish a hold and continue
    # spinning feedback until in-flight Actions finish their recovery.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    ros_executor = MultiThreadedExecutor(num_threads=8)
    try:
        node = ArmMotionServer()
        ros_executor.add_node(node)
        ros_executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().warning("Arm API interrupted by operator")
    finally:
        try:
            if node is not None:
                _shutdown_server(node, ros_executor)
            else:
                ros_executor.shutdown(timeout_sec=1.0)
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                singleton_lock.close()


if __name__ == "__main__":
    main()

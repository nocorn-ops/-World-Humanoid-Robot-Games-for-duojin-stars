"""Blocking Python API over the arm Actions; never publishes SDK topics."""

import math
import threading
import time
from typing import Callable, Optional

from action_msgs.srv import CancelGoal
from duojin_interfaces.msg import ArmMotionStatus
from rclpy.action import ActionClient

from .arm_client_commands import ArmClientCommands
from .arm_client_runtime import ArmClientRuntime, ClientOperation
from .arm_client_result import (
    ArmResult, known_failure, locally_cancelled, rejected_result, unknown_execution,
)


class ArmClient(ArmClientCommands):
    """Thread-safe blocking convenience client for one explicitly selected arm."""

    def __init__(
        self,
        arm: str,
        server_timeout_sec: float = 5.0,
        cancel_timeout_sec: float = 5.0,
    ) -> None:
        arm = str(arm).lower()
        if arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")
        timeout_values = (float(server_timeout_sec), float(cancel_timeout_sec))
        if any(not math.isfinite(value) or value <= 0.0 for value in timeout_values):
            raise ValueError("server and cancel timeouts must be positive")

        self.arm = arm
        self.server_timeout_sec = float(server_timeout_sec)
        self.cancel_timeout_sec = float(cancel_timeout_sec)
        self._state_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._operation: Optional[ClientOperation] = None
        self._closed = False
        self._runtime_closed = False
        self._runtime = ArmClientRuntime(arm)

    def __enter__(self) -> "ArmClient":
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def _reserve_operation(
        self, action_client, cancel_client, mapper: Callable
    ) -> Optional[ClientOperation]:
        operation = ClientOperation(action_client, cancel_client, mapper)
        with self._state_lock:
            if self._closed:
                raise RuntimeError("ArmClient is closed")
            if self._operation is not None:
                return None
            self._operation = operation
        return operation

    def _clear_operation(self, operation: ClientOperation) -> None:
        with self._state_lock:
            if self._operation is operation:
                self._operation = None

    def _mark_and_send(
        self, operation: ClientOperation, goal, feedback_callback: Optional[Callable]
    ) -> Optional[ArmResult]:
        with self._state_lock:
            if operation.cancel_requested or self._closed:
                return locally_cancelled("arm goal canceled before transmission")
            operation.sent = True
            operation.send_future = operation.action_client.send_goal_async(
                goal,
                feedback_callback=self._runtime.feedback_proxy(feedback_callback),
                goal_uuid=operation.goal_uuid,
            )
        return None

    def _adopt_goal_handle(
        self, operation: ClientOperation, goal_handle
    ) -> Optional[ArmResult]:
        if not goal_handle.accepted:
            return rejected_result()
        try:
            with self._state_lock:
                if operation.goal_handle is None:
                    result_future = goal_handle.get_result_async()
                    operation.result_future = result_future
                    operation.goal_handle = goal_handle
        except Exception as exc:
            return unknown_execution(f"Accepted Goal result channel failed ({exc}).")
        return None

    def _execute_operation(
        self,
        operation: ClientOperation,
        goal,
        result_timeout_sec: float,
        feedback_callback: Optional[Callable],
    ) -> ArmResult:
        server_state = self._runtime.wait_for_server(
            operation.action_client,
            self.server_timeout_sec,
            lambda: operation.cancel_requested or self._closed,
        )
        if server_state == "stopped":
            return locally_cancelled("arm goal canceled before transmission")
        if server_state != "ready":
            return known_failure(
                ArmMotionStatus.SERVER_UNAVAILABLE,
                f"arm Action server for {self.arm} was unavailable within "
                f"{self.server_timeout_sec:.1f}s",
            )
        not_sent = self._mark_and_send(operation, goal, feedback_callback)
        if not_sent is not None:
            return not_sent
        try:
            handle = self._runtime.wait_future(
                operation.send_future, self.server_timeout_sec
            )
        except Exception as exc:
            return self._cancel_after_uncertain_send(operation, exc)
        early_result = self._adopt_goal_handle(operation, handle)
        if early_result is not None:
            if not early_result.execution_state_known:
                return self._cancel_after_result_wait(
                    operation, RuntimeError(early_result.message)
                )
            return early_result
        with self._state_lock:
            cancel_requested = operation.cancel_requested
        if cancel_requested:
            return self._cancel_operation(operation, self.cancel_timeout_sec)
        try:
            wrapped = self._runtime.wait_future(
                operation.result_future, result_timeout_sec
            )
            return operation.mapper(wrapped)
        except Exception as exc:
            return self._cancel_after_result_wait(operation, exc)

    def _cancel_after_uncertain_send(
        self, operation: ClientOperation, error: Exception
    ) -> ArmResult:
        canceled = self._cancel_operation(operation, self.cancel_timeout_sec)
        if canceled.execution_state_known:
            return canceled
        return unknown_execution(f"Goal acknowledgement failed after send ({error}).")

    def _cancel_after_result_wait(
        self, operation: ClientOperation, error: Exception
    ) -> ArmResult:
        canceled = self._cancel_operation(operation, self.cancel_timeout_sec)
        if canceled.execution_state_known:
            return canceled
        return unknown_execution(f"Goal result wait failed after acceptance ({error}).")

    def _request_uuid_cancel(
        self, operation: ClientOperation, deadline: float
    ) -> Optional[CancelGoal.Response]:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            if remaining <= 0.0 or not operation.cancel_client.wait_for_service(
                timeout_sec=remaining
            ):
                return None
            request = CancelGoal.Request()
            request.goal_info.goal_id = operation.goal_uuid
            return self._runtime.wait_future(
                operation.cancel_client.call_async(request),
                max(0.0, deadline - time.monotonic()),
            )
        except Exception:
            return None

    def _wait_cancel_ack(
        self, operation: ClientOperation, deadline: float
    ) -> Optional[ArmResult]:
        with self._state_lock:
            goal_handle = operation.goal_handle
        if goal_handle is not None:
            return None
        try:
            handle = self._runtime.wait_future(
                operation.send_future, max(0.0, deadline - time.monotonic())
            )
        except Exception:
            return unknown_execution("Goal acknowledgement remained unavailable after cancel.")
        return self._adopt_goal_handle(operation, handle)

    def _cancel_operation(
        self, operation: ClientOperation, timeout_sec: float
    ) -> ArmResult:
        with operation.cancel_lock:
            if operation.cancel_result is not None:
                return operation.cancel_result
            with self._state_lock:
                operation.cancel_requested = True
                sent = operation.sent
            if not sent:
                result = locally_cancelled("arm goal canceled before transmission")
                operation.cancel_result = result
                return result
            deadline = time.monotonic() + max(0.0, float(timeout_sec))
            response = self._request_uuid_cancel(operation, deadline)
            ack_result = self._wait_cancel_ack(operation, deadline)
            if ack_result is not None:
                operation.cancel_result = ack_result
                return ack_result
            if response is None or response.return_code in (
                CancelGoal.Response.ERROR_REJECTED,
                CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
            ):
                response = self._request_uuid_cancel(operation, deadline)
            try:
                wrapped = self._runtime.wait_future(
                    operation.result_future, max(0.0, deadline - time.monotonic())
                )
                result = operation.mapper(wrapped)
            except Exception as exc:
                result = unknown_execution(
                    f"Cancel/hold terminal result was not confirmed ({exc})."
                )
            operation.cancel_result = result
            return result

    @staticmethod
    def _allow_cancel_retry(operation: ClientOperation) -> None:
        if not operation.caller_done.is_set():
            return
        with operation.cancel_lock:
            previous = operation.cancel_result
            if previous is not None and not previous.execution_state_known:
                operation.cancel_result = None

    def _run_goal(
        self,
        action_client: ActionClient,
        cancel_client,
        goal,
        result_timeout_sec: float,
        result_mapper: Callable,
        feedback_callback: Optional[Callable],
    ) -> ArmResult:
        operation = self._reserve_operation(
            action_client, cancel_client, result_mapper
        )
        if operation is None:
            return known_failure(
                ArmMotionStatus.BUSY,
                "this Python client already has an active or unconfirmed goal",
            )
        result: Optional[ArmResult] = None
        try:
            result = self._execute_operation(
                operation, goal, result_timeout_sec, feedback_callback
            )
            return result
        except KeyboardInterrupt:
            result = self._cancel_operation(operation, self.cancel_timeout_sec)
            if not result.execution_state_known:
                self._runtime.node.get_logger().error(result.message)
            raise
        except Exception as exc:
            result = (
                self._cancel_operation(operation, self.cancel_timeout_sec)
                if operation.sent
                else known_failure(ArmMotionStatus.SERVER_UNAVAILABLE, str(exc))
            )
            return result
        finally:
            operation.caller_done.set()
            if result is not None and result.execution_state_known:
                self._clear_operation(operation)

    def cancel(self, timeout_sec: Optional[float] = None) -> Optional[ArmResult]:
        """Cancel an active/unconfirmed goal and wait for its terminal hold result."""

        with self._control_lock:
            return self._cancel_locked(timeout_sec)

    def _cancel_locked(self, timeout_sec: Optional[float]) -> Optional[ArmResult]:
        with self._state_lock:
            if self._runtime_closed:
                raise RuntimeError("ArmClient is closed")
            operation = self._operation
        if operation is None:
            return None
        wait_sec = self.cancel_timeout_sec if timeout_sec is None else float(timeout_sec)
        if not math.isfinite(wait_sec) or wait_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        self._allow_cancel_retry(operation)
        result = self._cancel_operation(operation, wait_sec)
        if result.execution_state_known and operation.caller_done.is_set():
            self._clear_operation(operation)
        return result

    def close(self) -> None:
        with self._control_lock:
            self._close_locked()

    def _close_locked(self) -> None:
        with self._state_lock:
            if self._runtime_closed:
                return
            self._closed = True
            operation = self._operation
        if operation is not None:
            self._allow_cancel_retry(operation)
            result = self._cancel_operation(operation, self.cancel_timeout_sec)
            caller_stopped = operation.caller_done.wait(
                timeout=self.server_timeout_sec + self.cancel_timeout_sec + 1.0
            )
            if not caller_stopped:
                raise RuntimeError(
                    "arm call did not stop; ROS resources were kept alive. "
                    "Confirm robot state and call close() again."
                )
            if not result.execution_state_known:
                self._allow_cancel_retry(operation)
                result = self._cancel_operation(operation, self.cancel_timeout_sec)
            if not result.execution_state_known:
                self._runtime.node.get_logger().error(result.message)
        self._runtime.close()
        with self._state_lock:
            self._runtime_closed = True


from .arm_client_functions import get_pose, move_by, move_joints, move_to  # noqa: E402,F401

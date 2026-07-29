"""ROS-free concurrency and cleanup tests for the private arm client runtime."""

from concurrent.futures import Future
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_MODULE = PACKAGE_ROOT / "duojin_robot_interface" / "arm_client_runtime.py"


def _module(name, **attributes):
    module = ModuleType(name)
    for attribute, value in attributes.items():
        setattr(module, attribute, value)
    return module


def _runtime_import_stubs():
    placeholder = type("RosPlaceholder", (), {})
    signal_options = SimpleNamespace(NO=object())
    return {
        "action_msgs": _module("action_msgs"),
        "action_msgs.srv": _module("action_msgs.srv", CancelGoal=placeholder),
        "duojin_interfaces": _module("duojin_interfaces"),
        "duojin_interfaces.action": _module(
            "duojin_interfaces.action",
            MoveArmJoints=placeholder,
            MoveArmPose=placeholder,
            MoveArmRelative=placeholder,
        ),
        "geometry_msgs": _module("geometry_msgs"),
        "geometry_msgs.msg": _module("geometry_msgs.msg", PoseStamped=placeholder),
        "rclpy": _module("rclpy", init=lambda **_kwargs: None),
        "rclpy.action": _module("rclpy.action", ActionClient=placeholder),
        "rclpy.context": _module("rclpy.context", Context=placeholder),
        "rclpy.executors": _module(
            "rclpy.executors", SingleThreadedExecutor=placeholder
        ),
        "rclpy.node": _module("rclpy.node", Node=placeholder),
        "rclpy.qos": _module(
            "rclpy.qos",
            DurabilityPolicy=SimpleNamespace(VOLATILE=0),
            HistoryPolicy=SimpleNamespace(KEEP_LAST=0),
            QoSProfile=placeholder,
            ReliabilityPolicy=SimpleNamespace(RELIABLE=0),
        ),
        "rclpy.signals": _module(
            "rclpy.signals", SignalHandlerOptions=signal_options
        ),
        "unique_identifier_msgs": _module("unique_identifier_msgs"),
        "unique_identifier_msgs.msg": _module(
            "unique_identifier_msgs.msg", UUID=placeholder
        ),
    }


def _load_runtime_module():
    stubs = _runtime_import_stubs()
    module_name = "_duojin_arm_client_runtime_under_test"
    spec = importlib.util.spec_from_file_location(module_name, RUNTIME_MODULE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    previous = {name: sys.modules.get(name) for name in (*stubs, module_name)}
    try:
        sys.modules.update(stubs)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    finally:
        for name, old_module in previous.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module
    return module


RUNTIME = _load_runtime_module()


class _ContextState:
    def __init__(self) -> None:
        self.running = True
        self.shutdown_calls = 0

    def ok(self) -> bool:
        return self.running

    def try_shutdown(self) -> None:
        self.running = False
        self.shutdown_calls += 1


class _Destroyable:
    def __init__(self) -> None:
        self.destroy_calls = 0

    def destroy(self) -> None:
        self.destroy_calls += 1


class _NodeState:
    def __init__(self) -> None:
        self.destroyed_clients = []
        self.destroyed_subscriptions = []
        self.destroy_calls = 0

    def destroy_client(self, client) -> None:
        self.destroyed_clients.append(client)

    def destroy_node(self) -> None:
        self.destroy_calls += 1

    def destroy_subscription(self, subscription) -> None:
        self.destroyed_subscriptions.append(subscription)


class _ExecutorState:
    def __init__(self, stopped: bool) -> None:
        self.stopped = stopped
        self.shutdown_calls = 0

    def shutdown(self, *, timeout_sec: float) -> bool:
        assert timeout_sec == pytest.approx(2.0)
        self.shutdown_calls += 1
        return self.stopped


class _SpinThreadState:
    def __init__(self, alive: bool) -> None:
        self.alive = alive
        self.join_calls = 0

    def join(self, *, timeout: float) -> None:
        assert timeout == pytest.approx(2.0)
        self.join_calls += 1

    def is_alive(self) -> bool:
        return self.alive


def _runtime_for_close(*, executor_stopped: bool, spin_alive: bool):
    runtime = RUNTIME.ArmClientRuntime.__new__(RUNTIME.ArmClientRuntime)
    runtime._executor_shutdown_failed = False
    runtime._stop_feedback_worker = lambda: None
    runtime.executor = _ExecutorState(executor_stopped)
    runtime._spin_thread = _SpinThreadState(spin_alive)
    runtime.pose_client = _Destroyable()
    runtime.relative_client = _Destroyable()
    runtime.joint_client = _Destroyable()
    runtime.pose_cancel_client = object()
    runtime.relative_cancel_client = object()
    runtime.joint_cancel_client = object()
    runtime.pose_subscription = object()
    runtime.node = _NodeState()
    runtime.context = _ContextState()
    return runtime


def _assert_entities_kept_alive(runtime) -> None:
    assert runtime.pose_client.destroy_calls == 0
    assert runtime.relative_client.destroy_calls == 0
    assert runtime.joint_client.destroy_calls == 0
    assert runtime.node.destroyed_clients == []
    assert runtime.node.destroy_calls == 0


def test_cancelled_future_fails_immediately_without_waiting(monkeypatch) -> None:
    class NoWaitEvent:
        def set(self) -> None:
            pass

        def wait(self, _timeout: float) -> None:
            pytest.fail("a cancelled Future must not enter the timed wait loop")

    monkeypatch.setattr(RUNTIME, "threading", SimpleNamespace(Event=NoWaitEvent))
    future = Future()
    assert future.cancel()
    runtime = RUNTIME.ArmClientRuntime.__new__(RUNTIME.ArmClientRuntime)

    with pytest.raises(RuntimeError, match="cancelled before completion"):
        runtime.wait_future(future, timeout_sec=3600.0)


def test_completed_future_wins_over_context_stopping_during_wait(monkeypatch) -> None:
    context = _ContextState()
    future = Future()
    events = []

    class CompletingEvent:
        def __init__(self) -> None:
            self.wait_calls = 0
            events.append(self)

        def set(self) -> None:
            pass

        def wait(self, _timeout: float) -> None:
            self.wait_calls += 1
            context.try_shutdown()
            future.set_result("terminal-result")

    monkeypatch.setattr(RUNTIME, "threading", SimpleNamespace(Event=CompletingEvent))
    runtime = RUNTIME.ArmClientRuntime.__new__(RUNTIME.ArmClientRuntime)
    runtime.context = context
    runtime._spin_lock = threading.Lock()
    runtime._spin_error = None

    assert runtime.wait_future(future, timeout_sec=1.0) == "terminal-result"
    assert context.ok() is False
    assert events[0].wait_calls == 1


def test_current_pose_query_returns_a_copy_of_a_fresh_sample() -> None:
    runtime = RUNTIME.ArmClientRuntime.__new__(RUNTIME.ArmClientRuntime)
    runtime._pose_condition = threading.Condition()
    runtime._current_pose = SimpleNamespace(value=[1, 2, 3])
    runtime._current_pose_received_s = RUNTIME.time.monotonic()

    reading = runtime.wait_current_pose(timeout_sec=0.1, max_age_sec=0.25)

    assert reading.value == [1, 2, 3]
    assert reading is not runtime._current_pose


def test_successful_close_destroys_relative_action_and_pose_subscription() -> None:
    runtime = _runtime_for_close(executor_stopped=True, spin_alive=False)

    runtime.close()

    assert runtime.pose_client.destroy_calls == 1
    assert runtime.relative_client.destroy_calls == 1
    assert runtime.joint_client.destroy_calls == 1
    assert runtime.node.destroyed_clients == [
        runtime.pose_cancel_client,
        runtime.relative_cancel_client,
        runtime.joint_cancel_client,
    ]
    assert runtime.node.destroyed_subscriptions == [runtime.pose_subscription]
    assert runtime.node.destroy_calls == 1


def test_failed_executor_shutdown_keeps_entities_and_failure_is_sticky() -> None:
    runtime = _runtime_for_close(executor_stopped=False, spin_alive=False)

    with pytest.raises(RuntimeError, match="executor shutdown timed out"):
        runtime.close()

    _assert_entities_kept_alive(runtime)
    assert runtime.context.shutdown_calls == 1
    assert runtime.executor.shutdown_calls == 1
    assert runtime._executor_shutdown_failed is True

    with pytest.raises(RuntimeError, match="previously failed to stop"):
        runtime.close()

    _assert_entities_kept_alive(runtime)
    assert runtime.executor.shutdown_calls == 1


def test_live_spin_thread_keeps_entities_even_after_executor_reports_stop() -> None:
    runtime = _runtime_for_close(executor_stopped=True, spin_alive=True)

    with pytest.raises(RuntimeError, match="executor did not stop"):
        runtime.close()

    _assert_entities_kept_alive(runtime)
    assert runtime.executor.shutdown_calls == 1
    assert runtime._spin_thread.join_calls == 1
    assert runtime._executor_shutdown_failed is True

    with pytest.raises(RuntimeError, match="previously failed to stop"):
        runtime.close()
    _assert_entities_kept_alive(runtime)

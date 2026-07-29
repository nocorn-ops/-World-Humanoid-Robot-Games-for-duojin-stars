from pathlib import Path
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duojin_robot_interface.arm_lifecycle import (
    ArmExecutionLifecycle,
    PhysicalCommandOwnedError,
    ShutdownRequestedError,
)


def test_preview_lifecycle_never_becomes_a_physical_owner() -> None:
    lifecycle = ArmExecutionLifecycle()

    assert lifecycle.try_admit("request")
    lifecycle.register_accepted("request", "goal")
    lifecycle.mark_running("goal")
    lifecycle.finish("goal")
    lifecycle.request_shutdown()

    snapshot = lifecycle.snapshot()
    assert snapshot.shutdown_requested
    assert snapshot.physical_arms == ()
    assert lifecycle.wait_for_idle(0.0)


def test_shutdown_rejects_new_admission_and_motion_publication() -> None:
    lifecycle = ArmExecutionLifecycle()
    assert lifecycle.request_shutdown()
    assert not lifecycle.request_shutdown()

    assert not lifecycle.try_admit("late-request")
    with pytest.raises(ShutdownRequestedError):
        lifecycle.require_running()
    with pytest.raises(ShutdownRequestedError):
        lifecycle.publish_motion("left", "late-goal", lambda: None)


def test_shutdown_waits_until_in_progress_publication_leaves_atomic_gate() -> None:
    lifecycle = ArmExecutionLifecycle()
    publisher_entered = threading.Event()
    allow_publisher_to_finish = threading.Event()
    shutdown_finished = threading.Event()

    def publish() -> None:
        publisher_entered.set()
        assert allow_publisher_to_finish.wait(1.0)

    publish_thread = threading.Thread(
        target=lambda: lifecycle.publish_motion("left", "goal", publish)
    )
    publish_thread.start()
    assert publisher_entered.wait(1.0)

    shutdown_thread = threading.Thread(
        target=lambda: (lifecycle.request_shutdown(), shutdown_finished.set())
    )
    shutdown_thread.start()
    time.sleep(0.02)
    assert not shutdown_finished.is_set()

    allow_publisher_to_finish.set()
    publish_thread.join(timeout=1.0)
    shutdown_thread.join(timeout=1.0)

    assert not publish_thread.is_alive()
    assert not shutdown_thread.is_alive()
    assert shutdown_finished.is_set()
    assert lifecycle.physical_commands() == (("left", "goal"),)


def test_physical_ownership_is_latched_until_arrival_or_confirmed_hold() -> None:
    lifecycle = ArmExecutionLifecycle()
    lifecycle.publish_motion("left", "first", lambda: None)

    with pytest.raises(PhysicalCommandOwnedError):
        lifecycle.publish_motion("left", "second", lambda: None)
    assert not lifecycle.release_physical("left", "second")
    assert lifecycle.release_physical("left", "first")

    lifecycle.publish_motion("left", "second", lambda: None)
    lifecycle.publish_hold("left", lambda: None, goal_token="second")


def test_publication_exception_conservatively_keeps_physical_owner() -> None:
    lifecycle = ArmExecutionLifecycle()

    with pytest.raises(RuntimeError, match="middleware failed"):
        lifecycle.publish_motion(
            "right",
            "goal",
            lambda: (_ for _ in ()).throw(RuntimeError("middleware failed")),
        )

    assert lifecycle.has_physical_owner("right", "goal")


def test_goal_drain_tracks_admitted_queued_and_running_callbacks() -> None:
    lifecycle = ArmExecutionLifecycle()
    assert lifecycle.try_admit("request")
    assert not lifecycle.wait_for_idle(0.0)

    lifecycle.register_accepted("request", "goal")
    assert lifecycle.snapshot().queued_count == 1
    lifecycle.mark_running("goal")
    assert lifecycle.snapshot().running_count == 1
    lifecycle.finish("goal")

    assert lifecycle.wait_for_idle(0.01)


def test_failed_goal_response_reservation_can_be_discarded_after_executor_drain() -> None:
    lifecycle = ArmExecutionLifecycle()
    assert lifecycle.try_admit("orphaned-request")

    assert lifecycle.discard_unhandled_admissions() == 1
    assert lifecycle.discard_unhandled_admissions() == 0
    assert lifecycle.wait_for_idle(0.0)


def test_wait_for_idle_rejects_negative_timeout() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        ArmExecutionLifecycle().wait_for_idle(-0.1)

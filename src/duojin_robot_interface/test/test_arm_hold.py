from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duojin_robot_interface.arm_hold import shutdown_hold_active_arms
from duojin_robot_interface.arm_lifecycle import (
    ArmExecutionLifecycle,
    ShutdownRequestedError,
)


class _Logger:
    def warning(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class _Adapter:
    arm = "left"

    def __init__(self, publications: list[str]) -> None:
        self.publications = publications

    @staticmethod
    def feedback():
        return object()

    def publish_joint_target(self, _positions, _velocities) -> None:
        self.publications.append("hold")


class _Executor:
    def __init__(self, publications: list[str]) -> None:
        self.lifecycle = ArmExecutionLifecycle()
        self.adapters = {"left": _Adapter(publications)}
        self.config = SimpleNamespace(hold_joint_velocity_rad_s=0.1)
        self.node = SimpleNamespace(get_logger=lambda: _Logger())

    @staticmethod
    def ensure_fresh_joint_feedback(_snapshot, _adapter):
        return (0.0,) * 6


def test_shutdown_of_preview_goal_does_not_publish_a_joint_hold() -> None:
    publications: list[str] = []
    executor = _Executor(publications)
    executor.lifecycle.try_admit("preview-request")
    executor.lifecycle.register_accepted("preview-request", "preview-goal")
    executor.lifecycle.mark_running("preview-goal")

    executor.lifecycle.request_shutdown()
    shutdown_hold_active_arms(executor)

    assert publications == []


def test_shutdown_hold_is_last_lifecycle_controlled_target() -> None:
    publications: list[str] = []
    executor = _Executor(publications)
    executor.lifecycle.publish_motion(
        "left", "physical-goal", lambda: publications.append("target")
    )

    executor.lifecycle.request_shutdown()
    shutdown_hold_active_arms(executor)

    with pytest.raises(ShutdownRequestedError):
        executor.lifecycle.publish_motion(
            "left", "physical-goal", lambda: publications.append("late-target")
        )
    assert publications == ["target", "hold"]

"""One-shot Python wrappers for callers that do not retain an ArmClient."""

from typing import Sequence

from .arm_client_result import ArmPoseReading, ArmResult


def move_to(
    x: float,
    y: float,
    z: float,
    *,
    arm: str,
    frame_id: str = "base_link",
    timeout_sec: float = 0.0,
    execute: bool = False,
) -> ArmResult:
    """Create one client, issue a pose goal, then close it safely."""

    from .arm_client import ArmClient

    with ArmClient(arm) as client:
        return client.move_to(
            x, y, z, frame_id=frame_id, timeout_sec=timeout_sec, execute=execute
        )


def move_joints(
    positions_rad: Sequence[float],
    *,
    arm: str,
    speed_scale: float = 0.2,
    timeout_sec: float = 0.0,
    execute: bool = False,
) -> ArmResult:
    """Create one client, issue a joint goal, then close it safely."""

    from .arm_client import ArmClient

    with ArmClient(arm) as client:
        return client.move_joints(
            positions_rad,
            speed_scale=speed_scale,
            timeout_sec=timeout_sec,
            execute=execute,
        )


def move_by(
    dx: float,
    dy: float,
    dz: float,
    *,
    arm: str,
    frame_id: str = "base_link",
    timeout_sec: float = 0.0,
    execute: bool = False,
) -> ArmResult:
    """Create one client and issue a relative Cartesian goal."""

    from .arm_client import ArmClient

    with ArmClient(arm) as client:
        return client.move_by(
            dx, dy, dz, frame_id=frame_id, timeout_sec=timeout_sec, execute=execute
        )


def get_pose(
    *, arm: str, timeout_sec: float = 2.0, max_age_sec: float = 0.25
) -> ArmPoseReading:
    """Create one client and return one fresh public pose sample."""

    from .arm_client import ArmClient

    with ArmClient(arm) as client:
        return client.get_pose(timeout_sec=timeout_sec, max_age_sec=max_age_sec)

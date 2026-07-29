"""Parameter declaration and validation for the arm API server."""

import math
from typing import Sequence, Tuple

from .arm_execution_types import ArmExecutionConfig


PARAMETER_DEFAULTS = {
    "execute": False,
    "ik_solver_frame": "torso_link3",
    "public_pose_frame": "base_link",
    "pose_publish_period_sec": 0.05,
    "feedback_freshness_sec": 0.10,
    "feedback_wait_timeout_sec": 3.0,
    "tf_timeout_sec": 0.25,
    "ik_response_timeout_sec": 1.0,
    "poll_period_sec": 0.02,
    "default_pose_timeout_sec": 15.0,
    "maximum_timeout_sec": 60.0,
    "hold_timeout_sec": 2.0,
    "shutdown_drain_timeout_sec": 3.0,
    "settle_duration_sec": 0.25,
    "pose_position_tolerance_m": 0.015,
    "pose_orientation_tolerance_rad": 0.05,
    "joint_position_tolerance_rad": 0.02,
    "controller_joint_lower_rad": [-2.8, 0.0, -3.14, -1.57, -1.57, -2.8],
    "controller_joint_upper_rad": [2.8, 3.14, 0.0, 1.57, 1.57, 2.8],
    "joint_limit_margin_rad": 0.03,
    "maximum_joint_delta_rad": 0.35,
    "safe_joint_velocity_rad_s": [0.25] * 6,
    "hold_joint_velocity_rad_s": 0.10,
    "maximum_cartesian_delta_m": 0.08,
    "allowed_left_joint_publishers": ["/relaxed_ik_left"],
    "allowed_right_joint_publishers": ["/relaxed_ik_right"],
}

_POSITIVE_PARAMETERS = (
    "pose_publish_period_sec",
    "feedback_freshness_sec",
    "feedback_wait_timeout_sec",
    "tf_timeout_sec",
    "ik_response_timeout_sec",
    "poll_period_sec",
    "default_pose_timeout_sec",
    "maximum_timeout_sec",
    "hold_timeout_sec",
    "shutdown_drain_timeout_sec",
    "settle_duration_sec",
    "pose_position_tolerance_m",
    "pose_orientation_tolerance_rad",
    "joint_position_tolerance_rad",
    "joint_limit_margin_rad",
    "maximum_joint_delta_rad",
    "hold_joint_velocity_rad_s",
    "maximum_cartesian_delta_m",
)


def declare_arm_parameters(node) -> None:
    for name, default in PARAMETER_DEFAULTS.items():
        node.declare_parameter(name, default)


def _six_floats(values: Sequence[float], name: str) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != 6 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"parameter {name} must contain exactly 6 finite values")
    return result


def _positive_parameter(node, name: str) -> float:
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"parameter {name} must be finite and positive")
    return value


def _joint_limits(node) -> Tuple[Tuple[float, float], ...]:
    lower = _six_floats(
        node.get_parameter("controller_joint_lower_rad").value,
        "controller_joint_lower_rad",
    )
    upper = _six_floats(
        node.get_parameter("controller_joint_upper_rad").value,
        "controller_joint_upper_rad",
    )
    if any(low >= high for low, high in zip(lower, upper)):
        raise ValueError("each controller joint lower limit must be below its upper")
    return tuple(zip(lower, upper))


def _safe_velocities(node) -> Tuple[float, ...]:
    velocities = _six_floats(
        node.get_parameter("safe_joint_velocity_rad_s").value,
        "safe_joint_velocity_rad_s",
    )
    if any(value <= 0.0 for value in velocities):
        raise ValueError("all safe_joint_velocity_rad_s values must be positive")
    return velocities


def _required_frame(node, name: str) -> str:
    frame = str(node.get_parameter(name).value)
    if not frame:
        raise ValueError(f"parameter {name} must not be empty")
    return frame


def _validate_config(config: ArmExecutionConfig) -> None:
    if config.default_pose_timeout_sec > config.maximum_timeout_sec:
        raise ValueError("default_pose_timeout_sec must not exceed maximum_timeout_sec")
    if any(
        low + config.joint_limit_margin_rad
        >= high - config.joint_limit_margin_rad
        for low, high in config.joint_limits_rad
    ):
        raise ValueError("joint_limit_margin_rad leaves an empty joint range")


def read_arm_config(node) -> ArmExecutionConfig:
    positive = {
        name: _positive_parameter(node, name) for name in _POSITIVE_PARAMETERS
    }
    config = ArmExecutionConfig(
        execute=bool(node.get_parameter("execute").value),
        ik_solver_frame=_required_frame(node, "ik_solver_frame"),
        public_pose_frame=_required_frame(node, "public_pose_frame"),
        joint_limits_rad=_joint_limits(node),
        safe_joint_velocity_rad_s=_safe_velocities(node),
        **positive,
    )
    _validate_config(config)
    return config

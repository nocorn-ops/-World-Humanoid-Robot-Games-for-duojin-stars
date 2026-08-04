import math
from pathlib import Path
import sys
import threading

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from duojin_robot_interface.arm_domain import (
    ArmArbiter,
    ArmBusyError,
    ArmValidationError,
    ArrivalGate,
    ArrivalGateConfig,
    JOINT_TIMEOUT_CAP_S,
    JointValidationError,
    MAX_JOINT_TARGET_DELTA_RAD,
    MonotonicTimeError,
    PoseValidationError,
    compute_joint_timeout_s,
    is_fresh,
    max_joint_error_rad,
    monotonic_age_s,
    normalize_quaternion,
    position_error_m,
    quaternion_shortest_angle_rad,
    validate_cartesian_target,
    validate_joint_positions,
    validate_joint_target,
    validate_speed_scale,
    validate_xyz,
)
from duojin_robot_interface.arm_pose_domain import relative_cartesian_target


SAFE_JOINTS = (0.0, 0.5, -0.5, 0.0, 0.0, 0.0)


def test_joint_limits_accept_both_effective_boundaries() -> None:
    lower = (-2.77, 0.03, -3.11, -1.54, -1.54, -2.77)
    upper = (2.77, 3.11, -0.03, 1.54, 1.54, 2.77)

    assert validate_joint_positions(lower) == pytest.approx(lower)
    assert validate_joint_positions(upper) == pytest.approx(upper)


@pytest.mark.parametrize(
    "positions",
    [
        (-2.7701, 0.5, -0.5, 0.0, 0.0, 0.0),
        (0.0, 0.0299, -0.5, 0.0, 0.0, 0.0),
        (0.0, 0.5, -0.0299, 0.0, 0.0, 0.0),
        (0.0, 0.5, -0.5, 1.5401, 0.0, 0.0),
        (0.0, 0.5, -0.5, 0.0, -1.5401, 0.0),
        (0.0, 0.5, -0.5, 0.0, 0.0, 2.7701),
    ],
)
def test_joint_limits_reject_values_outside_effective_range(positions) -> None:
    with pytest.raises(JointValidationError, match="outside effective limits"):
        validate_joint_positions(positions)


@pytest.mark.parametrize(
    "positions",
    [
        SAFE_JOINTS[:-1],
        SAFE_JOINTS + (0.0,),
        (0.0, 0.5, -0.5, 0.0, 0.0, math.nan),
        (0.0, 0.5, -0.5, 0.0, 0.0, math.inf),
        (0.0, 0.5, -0.5, 0.0, 0.0, True),
        "not-a-joint-vector",
    ],
)
def test_joint_vector_rejects_bad_shape_or_non_finite_values(positions) -> None:
    with pytest.raises(JointValidationError):
        validate_joint_positions(positions)


def test_joint_target_accepts_exact_maximum_delta() -> None:
    target = list(SAFE_JOINTS)
    target[0] = MAX_JOINT_TARGET_DELTA_RAD

    validated = validate_joint_target(target, SAFE_JOINTS, speed_scale=0.25)

    assert validated.positions_rad == pytest.approx(target)
    assert validated.speed_scale == 0.25


def test_joint_target_rejects_delta_above_maximum() -> None:
    target = list(SAFE_JOINTS)
    target[0] = MAX_JOINT_TARGET_DELTA_RAD + 0.001

    with pytest.raises(JointValidationError, match="maximum target delta"):
        validate_joint_target(target, SAFE_JOINTS)


def test_joint_validation_uses_configured_limits_margin_and_delta() -> None:
    limits = list((lower, upper) for lower, upper in (
        (-1.0, 1.0), (0.0, 3.14), (-3.14, 0.0),
        (-1.57, 1.57), (-1.57, 1.57), (-2.8, 2.8),
    ))
    assert validate_joint_positions(
        (0.9, 0.5, -0.5, 0.0, 0.0, 0.0),
        joint_limits_rad=limits,
        joint_limit_margin_rad=0.1,
    )[0] == pytest.approx(0.9)
    with pytest.raises(JointValidationError, match="outside effective limits"):
        validate_joint_positions(
            (0.91, 0.5, -0.5, 0.0, 0.0, 0.0),
            joint_limits_rad=limits,
            joint_limit_margin_rad=0.1,
        )
    with pytest.raises(JointValidationError, match="maximum target delta"):
        validate_joint_target(
            (0.11, 0.5, -0.5, 0.0, 0.0, 0.0),
            SAFE_JOINTS,
            maximum_delta_rad=0.1,
        )


@pytest.mark.parametrize(
    ("limits", "margin"),
    [
        (((-1.0, 1.0),) * 5, 0.0),
        (((1.0, -1.0),) * 6, 0.0),
        (((-1.0, 1.0),) * 6, -0.1),
        (((-1.0, 1.0),) * 6, 1.1),
    ],
)
def test_joint_validation_rejects_invalid_configured_limits(limits, margin) -> None:
    with pytest.raises(JointValidationError):
        validate_joint_positions(
            SAFE_JOINTS,
            joint_limits_rad=limits,
            joint_limit_margin_rad=margin,
        )


def test_joint_target_rejects_negative_configured_maximum_delta() -> None:
    with pytest.raises(JointValidationError, match="maximum_delta_rad"):
        validate_joint_target(SAFE_JOINTS, SAFE_JOINTS, maximum_delta_rad=-0.1)


@pytest.mark.parametrize("speed_scale", [0.0001, 0.5, 1.0])
def test_speed_scale_accepts_open_closed_domain(speed_scale: float) -> None:
    assert validate_speed_scale(speed_scale) == speed_scale


@pytest.mark.parametrize("speed_scale", [0.0, -0.1, 1.0001, math.nan, math.inf, True])
def test_speed_scale_rejects_invalid_values(speed_scale) -> None:
    with pytest.raises(JointValidationError):
        validate_speed_scale(speed_scale)


def test_cartesian_target_is_finite_and_normalizes_quaternion() -> None:
    target = validate_cartesian_target((0.2, -0.1, 0.3), (0.0, 0.0, 0.0, 2.0))

    assert target.position_xyz_m == (0.2, -0.1, 0.3)
    assert target.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_relative_target_adds_delta_and_preserves_normalized_orientation() -> None:
    target = relative_cartesian_target(
        (0.2, -0.1, 0.3), (0.0, 0.0, 0.0, 2.0), (0.01, 0.02, -0.03)
    )

    assert target.position_xyz_m == pytest.approx((0.21, -0.08, 0.27))
    assert target.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_relative_target_rejects_non_finite_delta() -> None:
    with pytest.raises(PoseValidationError):
        relative_cartesian_target(
            (0.2, -0.1, 0.3), (0.0, 0.0, 0.0, 1.0), (0.0, math.nan, 0.0)
        )


def test_quaternion_normalization_is_stable_for_large_finite_components() -> None:
    assert normalize_quaternion((1e308, 0.0, 0.0, 0.0)) == (1.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "position",
    [(1.0, 2.0), (1.0, 2.0, 3.0, 4.0), (1.0, math.nan, 3.0)],
)
def test_xyz_rejects_bad_shape_or_non_finite_values(position) -> None:
    with pytest.raises(PoseValidationError):
        validate_xyz(position)


@pytest.mark.parametrize(
    "quaternion",
    [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, math.nan, 1.0),
    ],
)
def test_quaternion_rejects_bad_shape_zero_norm_or_nan(quaternion) -> None:
    with pytest.raises(PoseValidationError):
        normalize_quaternion(quaternion)


def test_position_error_is_euclidean() -> None:
    assert position_error_m((1.0, 2.0, 3.0), (4.0, 6.0, 3.0)) == pytest.approx(5.0)


def test_quaternion_error_treats_sign_flipped_quaternion_as_same_orientation() -> None:
    assert quaternion_shortest_angle_rad(
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, -1.0),
    ) == pytest.approx(0.0)


def test_quaternion_error_returns_shortest_angle() -> None:
    half_angle = math.pi / 4.0
    quarter_turn_z = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))

    assert quaternion_shortest_angle_rad(
        (0.0, 0.0, 0.0, 1.0),
        quarter_turn_z,
    ) == pytest.approx(math.pi / 2.0)


def test_max_joint_error_uses_largest_absolute_component() -> None:
    actual = (0.0, 0.5, -0.5, 0.0, 0.0, 0.0)
    target = (0.01, 0.48, -0.45, 0.0, -0.03, 0.0)

    assert max_joint_error_rad(actual, target) == pytest.approx(0.05)


def test_freshness_uses_arrival_monotonic_age_and_includes_boundary() -> None:
    assert monotonic_age_s(10.0, 10.4) == pytest.approx(0.4)
    assert is_fresh(10.0, max_age_s=0.4, now_monotonic_s=10.4)
    assert not is_fresh(10.0, max_age_s=0.39, now_monotonic_s=10.4)


def test_feedback_that_never_arrived_is_not_fresh() -> None:
    assert math.isinf(monotonic_age_s(None, 10.0))
    assert not is_fresh(None, max_age_s=1.0, now_monotonic_s=10.0)


def test_freshness_rejects_clock_regression_and_negative_age_limit() -> None:
    with pytest.raises(MonotonicTimeError):
        monotonic_age_s(10.0, 9.99)
    with pytest.raises(ArmValidationError):
        is_fresh(10.0, max_age_s=-0.1, now_monotonic_s=10.0)


def test_arrival_gate_requires_both_continuous_time_and_samples() -> None:
    gate = ArrivalGate(ArrivalGateConfig(min_samples=3, min_continuous_s=0.2))

    assert not gate.update(True, 1.0)
    assert not gate.update(True, 1.1)
    assert gate.consecutive_samples == 2
    assert gate.update(True, 1.2)
    assert gate.arrived


def test_arrival_gate_resets_entire_window_on_out_of_tolerance_sample() -> None:
    gate = ArrivalGate(ArrivalGateConfig(min_samples=2, min_continuous_s=0.1))

    assert not gate.update(True, 1.0)
    assert not gate.update(False, 1.2)
    assert gate.consecutive_samples == 0
    assert not gate.update(True, 1.3)
    assert gate.update(True, 1.4)


def test_arrival_gate_rejects_invalid_config_and_regressing_samples() -> None:
    with pytest.raises(ArmValidationError):
        ArrivalGateConfig(min_samples=0)
    with pytest.raises(ArmValidationError):
        ArrivalGateConfig(min_continuous_s=-0.01)

    gate = ArrivalGate()
    gate.update(True, 2.0)
    with pytest.raises(MonotonicTimeError):
        gate.update(True, 1.9)


def test_arbiter_allows_independent_arms_and_protects_owner_release() -> None:
    arbiter = ArmArbiter()

    assert arbiter.try_acquire("left", "left-goal")
    assert arbiter.try_acquire("right", "right-goal")
    assert not arbiter.try_acquire("left", "second-left-goal")
    assert not arbiter.release("left", "stale-goal")
    assert arbiter.owner("left") == "left-goal"
    assert arbiter.release("left", "left-goal")
    assert not arbiter.is_busy("left")
    assert arbiter.is_busy("right")


def test_arbiter_acquire_reports_busy_and_rejects_unknown_arm() -> None:
    arbiter = ArmArbiter()
    arbiter.acquire("left", "first")

    with pytest.raises(ArmBusyError):
        arbiter.acquire("left", "second")
    with pytest.raises(ArmValidationError):
        arbiter.try_acquire("centre", "goal")


def test_arbiter_claim_is_atomic_under_concurrency() -> None:
    arbiter = ArmArbiter()
    contenders = 8
    barrier = threading.Barrier(contenders)
    results = [False] * contenders

    def contend(index: int) -> None:
        barrier.wait()
        results[index] = arbiter.try_acquire("left", f"goal-{index}")

    threads = [threading.Thread(target=contend, args=(index,)) for index in range(contenders)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2.0)

    assert all(not thread.is_alive() for thread in threads)
    assert sum(results) == 1


def test_joint_timeout_uses_floor_dynamic_formula_and_speed_scale() -> None:
    zeros = (0.0,) * 6
    velocities = (1.0,) * 6

    assert compute_joint_timeout_s(zeros, (0.1,) * 6, velocities) == 5.0
    assert compute_joint_timeout_s(zeros, (3.0,) * 6, velocities) == pytest.approx(8.0)
    assert compute_joint_timeout_s(
        zeros,
        (1.0,) * 6,
        velocities,
        speed_scale=0.5,
    ) == pytest.approx(6.0)


def test_joint_timeout_is_capped_at_sixty_seconds() -> None:
    timeout_s = compute_joint_timeout_s(
        (0.0,) * 6,
        (100.0,) * 6,
        (0.1,) * 6,
        speed_scale=0.1,
    )

    assert timeout_s == JOINT_TIMEOUT_CAP_S


@pytest.mark.parametrize(
    "velocities",
    [(1.0,) * 5, (1.0,) * 5 + (0.0,), (1.0,) * 5 + (math.nan,)],
)
def test_joint_timeout_rejects_invalid_velocity_vector(velocities) -> None:
    with pytest.raises(JointValidationError):
        compute_joint_timeout_s((0.0,) * 6, (1.0,) * 6, velocities)

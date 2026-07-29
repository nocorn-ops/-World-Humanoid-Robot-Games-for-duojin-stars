"""Pure, ROS-free domain rules for safe arm targets and completion checks.

Values use SI units: metres, radians and seconds.
"""

from dataclasses import dataclass
import math
from numbers import Real
import threading
import time
from typing import Iterable, Optional, Sequence, Tuple

ARM_NAMES: Tuple[str, str] = ("left", "right")
ARM_JOINT_COUNT = 6
# Controller-facing limits; a fixed margin is still applied to executable goals.
JOINT_LIMITS_RAD: Tuple[Tuple[float, float], ...] = (
    (-2.8, 2.8), (0.0, 3.14), (-3.14, 0.0),
    (-1.57, 1.57), (-1.57, 1.57), (-2.8, 2.8),
)
JOINT_LIMIT_MARGIN_RAD = 0.03
MAX_JOINT_TARGET_DELTA_RAD = 0.35
JOINT_TIMEOUT_FLOOR_S = 5.0
JOINT_TIMEOUT_OVERHEAD_S = 2.0
JOINT_TIMEOUT_TRAVEL_FACTOR = 2.0
JOINT_TIMEOUT_CAP_S = 60.0
_QUATERNION_NORM_EPSILON = 1e-12
_BOUNDARY_EPSILON = 1e-12

class ArmDomainError(Exception):
    """Base exception for arm-domain failures."""

class ArmValidationError(ArmDomainError):
    """A target or configuration violates a domain contract."""

class JointValidationError(ArmValidationError):
    """A joint goal, feedback vector or velocity vector is invalid."""

class PoseValidationError(ArmValidationError):
    """A Cartesian position or orientation is invalid."""

class MonotonicTimeError(ArmDomainError):
    """A monotonic timestamp moved backwards."""

class ArmBusyError(ArmDomainError):
    """An arm already has a goal owner."""

@dataclass(frozen=True)
class JointTarget:
    """Validated executable joint goal."""
    positions_rad: Tuple[float, ...]
    speed_scale: float

@dataclass(frozen=True)
class CartesianTarget:
    """Validated Cartesian goal with a unit quaternion."""
    position_xyz_m: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]

@dataclass(frozen=True)
class ArrivalGateConfig:
    """Requirements for declaring a target continuously reached."""
    min_samples: int = 3
    min_continuous_s: float = 0.2

    def __post_init__(self) -> None:
        if isinstance(self.min_samples, bool) or not isinstance(self.min_samples, int):
            raise ArmValidationError("min_samples must be an integer")
        if self.min_samples < 1:
            raise ArmValidationError("min_samples must be at least 1")
        continuous_s = _finite_scalar(
            self.min_continuous_s, "min_continuous_s", ArmValidationError
        )
        if continuous_s < 0.0:
            raise ArmValidationError("min_continuous_s must be non-negative")

def _finite_scalar(value: object, name: str, error: type[ArmValidationError]) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise error(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result):
        raise error(f"{name} must be finite")
    return result

def _finite_tuple(values: Iterable[object], length: int, name: str,
                  error: type[ArmValidationError]) -> Tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise error(f"{name} must be a numeric sequence")
    try:
        result = tuple(
            _finite_scalar(value, f"{name}[{index}]", error)
            for index, value in enumerate(values)
        )
    except TypeError as exc:
        raise error(f"{name} must be an iterable numeric sequence") from exc
    if len(result) != length:
        raise error(f"{name} must contain exactly {length} values; got {len(result)}")
    return result

def validate_speed_scale(speed_scale: object) -> float:
    """Return a finite speed scale in ``(0, 1]``."""
    result = _finite_scalar(speed_scale, "speed_scale", JointValidationError)
    if result <= 0.0 or result > 1.0:
        raise JointValidationError("speed_scale must be greater than 0 and at most 1")
    return result


def _validated_joint_limits(
    joint_limits_rad: Iterable[Sequence[object]], joint_limit_margin_rad: object
) -> Tuple[Tuple[Tuple[float, float], ...], float]:
    if isinstance(joint_limits_rad, (str, bytes)):
        raise JointValidationError("joint_limits_rad must be a 6x2 numeric sequence")
    try:
        raw_limits = tuple(joint_limits_rad)
    except TypeError as exc:
        raise JointValidationError("joint_limits_rad must be a 6x2 numeric sequence") from exc
    if len(raw_limits) != ARM_JOINT_COUNT:
        raise JointValidationError("joint_limits_rad must contain exactly 6 pairs")
    limits = tuple(
        _finite_tuple(pair, 2, f"joint_limits_rad[{index}]", JointValidationError)
        for index, pair in enumerate(raw_limits)
    )
    margin = _finite_scalar(
        joint_limit_margin_rad, "joint_limit_margin_rad", JointValidationError
    )
    if margin < 0.0:
        raise JointValidationError("joint_limit_margin_rad must be non-negative")
    for index, (lower, upper) in enumerate(limits):
        if lower >= upper or lower + margin > upper - margin:
            raise JointValidationError(
                f"joint_limits_rad[{index}] is incompatible with margin {margin}"
            )
    return limits, margin


def validate_joint_positions(
    positions_rad: Sequence[object],
    *,
    enforce_operating_limits: bool = True,
    joint_limits_rad: Iterable[Sequence[object]] = JOINT_LIMITS_RAD,
    joint_limit_margin_rad: object = JOINT_LIMIT_MARGIN_RAD,
) -> Tuple[float, ...]:
    """Validate an exact six-joint vector and optionally its effective limits."""
    result = _finite_tuple(
        positions_rad, ARM_JOINT_COUNT, "positions_rad", JointValidationError
    )
    if not enforce_operating_limits:
        return result
    limits, margin_rad = _validated_joint_limits(joint_limits_rad, joint_limit_margin_rad)
    for index, (position_rad, limits_rad) in enumerate(zip(result, limits)):
        lower_rad = limits_rad[0] + margin_rad
        upper_rad = limits_rad[1] - margin_rad
        if (
            position_rad < lower_rad - _BOUNDARY_EPSILON
            or position_rad > upper_rad + _BOUNDARY_EPSILON
        ):
            raise JointValidationError(
                f"positions_rad[{index}]={position_rad} is outside effective limits "
                f"[{lower_rad}, {upper_rad}]"
            )
    return result


def validate_joint_target(
    positions_rad: Sequence[object],
    current_positions_rad: Sequence[object],
    speed_scale: object = 1.0,
    *,
    joint_limits_rad: Iterable[Sequence[object]] = JOINT_LIMITS_RAD,
    joint_limit_margin_rad: object = JOINT_LIMIT_MARGIN_RAD,
    maximum_delta_rad: object = MAX_JOINT_TARGET_DELTA_RAD,
) -> JointTarget:
    """Validate a goal, including the current-to-target 0.35 rad step limit."""
    target = validate_joint_positions(
        positions_rad,
        joint_limits_rad=joint_limits_rad,
        joint_limit_margin_rad=joint_limit_margin_rad,
    )
    current = validate_joint_positions(current_positions_rad, enforce_operating_limits=False)
    delta_limit_rad = _finite_scalar(maximum_delta_rad, "maximum_delta_rad",
                                     JointValidationError)
    if delta_limit_rad < 0.0:
        raise JointValidationError("maximum_delta_rad must be non-negative")
    largest_delta_rad = max(abs(goal - actual) for goal, actual in zip(target, current))
    if largest_delta_rad > delta_limit_rad + _BOUNDARY_EPSILON:
        raise JointValidationError(
            f"maximum target delta {largest_delta_rad} rad exceeds "
            f"{delta_limit_rad} rad"
        )
    return JointTarget(target, validate_speed_scale(speed_scale))


def validate_xyz(position_xyz_m: Sequence[object]) -> Tuple[float, float, float]:
    """Return a finite XYZ position in metres."""
    xyz = _finite_tuple(position_xyz_m, 3, "position_xyz_m", PoseValidationError)
    return xyz[0], xyz[1], xyz[2]


def normalize_quaternion(orientation_xyzw: Sequence[object]
                         ) -> Tuple[float, float, float, float]:
    """Validate and normalize an XYZW quaternion; reject a zero norm."""
    quaternion = _finite_tuple(
        orientation_xyzw, 4, "orientation_xyzw", PoseValidationError
    )
    norm = math.hypot(*quaternion)
    if norm <= _QUATERNION_NORM_EPSILON:
        raise PoseValidationError("orientation_xyzw must have a non-zero norm")
    normalized = tuple(component / norm for component in quaternion)
    return normalized[0], normalized[1], normalized[2], normalized[3]


def validate_cartesian_target(
    position_xyz_m: Sequence[object], orientation_xyzw: Sequence[object]
) -> CartesianTarget:
    """Validate finite XYZ and return a normalized Cartesian target."""
    return CartesianTarget(validate_xyz(position_xyz_m), normalize_quaternion(orientation_xyzw))


def position_error_m(first_xyz_m: Sequence[object], second_xyz_m: Sequence[object]) -> float:
    """Return Euclidean position error in metres."""
    first, second = validate_xyz(first_xyz_m), validate_xyz(second_xyz_m)
    return math.dist(first, second)


def quaternion_shortest_angle_rad(
    first_xyzw: Sequence[object], second_xyzw: Sequence[object]
) -> float:
    """Return shortest orientation error; ``q`` and ``-q`` are equivalent."""
    first, second = normalize_quaternion(first_xyzw), normalize_quaternion(second_xyzw)
    dot = abs(sum(left * right for left, right in zip(first, second)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def max_joint_error_rad(
    current_positions_rad: Sequence[object], target_positions_rad: Sequence[object]
) -> float:
    """Return the maximum absolute error across exactly six finite joints."""
    current = validate_joint_positions(current_positions_rad, enforce_operating_limits=False)
    target = validate_joint_positions(target_positions_rad, enforce_operating_limits=False)
    return max(abs(actual - goal) for actual, goal in zip(current, target))


def monotonic_age_s(
    last_arrival_monotonic_s: Optional[object],
    now_monotonic_s: Optional[object] = None,
) -> float:
    """Return monotonic sample age; no sample returns infinity."""
    if last_arrival_monotonic_s is None:
        return math.inf
    last_s = _finite_scalar(
        last_arrival_monotonic_s, "last_arrival_monotonic_s", ArmValidationError
    )
    now_s = time.monotonic() if now_monotonic_s is None else _finite_scalar(
        now_monotonic_s, "now_monotonic_s", ArmValidationError
    )
    if now_s < last_s:
        raise MonotonicTimeError(f"monotonic clock regressed from {last_s} s to {now_s} s")
    return now_s - last_s


def is_fresh(
    last_arrival_monotonic_s: Optional[object],
    max_age_s: object,
    now_monotonic_s: Optional[object] = None,
) -> bool:
    """Return whether feedback is within the inclusive monotonic age limit."""
    limit_s = _finite_scalar(max_age_s, "max_age_s", ArmValidationError)
    if limit_s < 0.0:
        raise ArmValidationError("max_age_s must be non-negative")
    return monotonic_age_s(last_arrival_monotonic_s, now_monotonic_s) <= (
        limit_s + _BOUNDARY_EPSILON
    )


class ArrivalGate:
    """Require consecutive samples and continuous time inside tolerance."""
    def __init__(self, config: ArrivalGateConfig = ArrivalGateConfig()) -> None:
        if not isinstance(config, ArrivalGateConfig):
            raise ArmValidationError("config must be an ArrivalGateConfig")
        self._config = config
        self.reset()

    @property
    def consecutive_samples(self) -> int:
        return self._consecutive_samples

    @property
    def arrived(self) -> bool:
        return self._arrived

    def reset(self) -> None:
        self._window_start_s: Optional[float] = None
        self._last_update_s: Optional[float] = None
        self._consecutive_samples = 0
        self._arrived = False

    def update(self, within_tolerance: bool, now_monotonic_s: object) -> bool:
        if not isinstance(within_tolerance, bool):
            raise ArmValidationError("within_tolerance must be a bool")
        now_s = _finite_scalar(now_monotonic_s, "now_monotonic_s", ArmValidationError)
        if self._last_update_s is not None and now_s < self._last_update_s:
            raise MonotonicTimeError(
                f"arrival samples regressed from {self._last_update_s} s to {now_s} s"
            )
        self._last_update_s = now_s
        if not within_tolerance:
            self._window_start_s = None
            self._consecutive_samples = 0
            self._arrived = False
            return False
        if self._window_start_s is None:
            self._window_start_s = now_s
            self._consecutive_samples = 1
        else:
            self._consecutive_samples += 1
        self._arrived = (
            self._consecutive_samples >= self._config.min_samples
            and now_s - self._window_start_s + _BOUNDARY_EPSILON
            >= self._config.min_continuous_s
        )
        return self._arrived


class ArmArbiter:
    """Thread-safe per-arm goal ownership with atomic acquire/release."""
    def __init__(self) -> None:
        self._owners: dict[str, Optional[object]] = {arm: None for arm in ARM_NAMES}
        self._lock = threading.Lock()

    @staticmethod
    def _validate_arm(arm: object) -> str:
        if arm not in ARM_NAMES:
            raise ArmValidationError(f"arm must be one of {ARM_NAMES}; got {arm!r}")
        return str(arm)

    @staticmethod
    def _validate_owner(owner: object) -> None:
        if owner is None:
            raise ArmValidationError("owner must not be None")

    def try_acquire(self, arm: object, owner: object) -> bool:
        arm_name = self._validate_arm(arm)
        self._validate_owner(owner)
        with self._lock:
            if self._owners[arm_name] is not None:
                return False
            self._owners[arm_name] = owner
            return True

    def acquire(self, arm: object, owner: object) -> None:
        if not self.try_acquire(arm, owner):
            raise ArmBusyError(f"{arm} arm is already owned by {self.owner(arm)!r}")

    def release(self, arm: object, owner: object) -> bool:
        arm_name = self._validate_arm(arm)
        self._validate_owner(owner)
        with self._lock:
            if self._owners[arm_name] != owner:
                return False
            self._owners[arm_name] = None
            return True

    def owner(self, arm: object) -> Optional[object]:
        arm_name = self._validate_arm(arm)
        with self._lock:
            return self._owners[arm_name]

    def is_busy(self, arm: object) -> bool:
        return self.owner(arm) is not None


def compute_joint_timeout_s(
    current_positions_rad: Sequence[object],
    target_positions_rad: Sequence[object],
    velocity_limits_rad_s: Sequence[object],
    speed_scale: object = 1.0,
) -> float:
    """Return ``max(5, 2 + 2*max(delta/velocity))``, capped at 60 s."""
    current = validate_joint_positions(current_positions_rad, enforce_operating_limits=False)
    target = validate_joint_positions(target_positions_rad, enforce_operating_limits=False)
    velocities = _finite_tuple(
        velocity_limits_rad_s,
        ARM_JOINT_COUNT,
        "velocity_limits_rad_s",
        JointValidationError,
    )
    for index, velocity_rad_s in enumerate(velocities):
        if velocity_rad_s <= 0.0:
            raise JointValidationError(
                f"velocity_limits_rad_s[{index}] must be greater than 0"
            )
    scale = validate_speed_scale(speed_scale)
    travel_s = max(
        abs(goal - actual) / (velocity_rad_s * scale)
        for actual, goal, velocity_rad_s in zip(current, target, velocities)
    )
    estimated_s = max(
        JOINT_TIMEOUT_FLOOR_S,
        JOINT_TIMEOUT_OVERHEAD_S + JOINT_TIMEOUT_TRAVEL_FACTOR * travel_s,
    )
    return min(JOINT_TIMEOUT_CAP_S, estimated_s)

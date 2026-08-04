"""Pure Cartesian target construction shared by pose command modes."""

from .arm_domain import validate_cartesian_target, validate_xyz


def relative_cartesian_target(current_xyz_m, orientation_xyzw, delta_xyz_m):
    """Add a finite xyz delta in one already-normalized coordinate frame."""

    current = validate_xyz(current_xyz_m)
    delta = validate_xyz(delta_xyz_m)
    target = tuple(current[index] + delta[index] for index in range(3))
    return validate_cartesian_target(target, orientation_xyzw)

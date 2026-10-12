# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""
Map ideal planned NED acceleration and flat yaw to a right-handed FRD attitude.

The model assumes a = gravity * e_down - specific_thrust * body_down,
so thrust points along negative body z. It ignores drag, wind, and tracking error.
The defaults are explicit research assumptions, not measured vehicle limits.
No mass, inertia, angular-rate feasibility, or six-DOF execution model is inferred.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from numbers import Real
from typing import Optional, Tuple


_DEGENERACY_EPSILON = 1e-9
_Vector = Tuple[float, float, float]
_Rotation = Tuple[_Vector, _Vector, _Vector]


@dataclass(frozen=True)
class AttitudeConfig:
    """
    Define explicit ideal-model validity limits, in m/s^2 and radians.

    Gravity is positive; 0 < minimum_specific_thrust <= maximum_specific_thrust.
    Maximum tilt and the future angular error margin lie in [0, pi/2).
    The margin is stored and validated but does not alter attitude or feasibility.
    These limits do not establish calibrated error bounds or actuator feasibility.
    """

    gravity: float = 9.80665
    minimum_specific_thrust: float = 2.0
    maximum_specific_thrust: float = 18.0
    maximum_tilt_rad: float = 0.55
    angular_error_margin_rad: float = 0.03


@dataclass(frozen=True)
class PlannedAttitudeResult:
    """
    Return model validity and computed diagnostics without a fallback rotation.

    Rotation is an active FRD-to-NED row-major tuple whose columns are body axes.
    Invalid limits/inputs have no physical diagnostics. For infeasible candidates,
    finite specific thrust and tilt remain available when mathematically defined.
    """

    valid: bool = False
    reason: str = 'INVALID_INPUT'
    rotation_frd_to_ned: Optional[_Rotation] = None
    tilt_rad: Optional[float] = None
    specific_thrust: Optional[float] = None


def _finite_real(value):
    try:
        return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _valid_config(config):
    if not isinstance(config, AttitudeConfig):
        return False
    values = (config.gravity, config.minimum_specific_thrust, config.maximum_specific_thrust,
              config.maximum_tilt_rad, config.angular_error_margin_rad)
    return (all(_finite_real(value) for value in values)
            and config.gravity > 0.0
            and 0.0 < config.minimum_specific_thrust <= config.maximum_specific_thrust
            and 0.0 <= config.maximum_tilt_rad < math.pi / 2
            and 0.0 <= config.angular_error_margin_rad < math.pi / 2)


def _acceleration_vector(value):
    if isinstance(value, (str, bytes, Mapping)):
        return None
    try:
        components = tuple(value)
    except TypeError:
        return None
    if len(components) != 3 or not all(_finite_real(component) for component in components):
        return None
    return tuple(float(component) for component in components)


def planned_attitude(acceleration_ned, yaw_ref, config=AttitudeConfig()):
    """
    Evaluate an ideal planning attitude with explicit rejection of invalid candidates.

    yaw_ref is the flat reference h=(cos(yaw_ref), sin(yaw_ref), 0); inclined
    body-forward horizontal heading need not equal it. The axes are constructed as
    b3=normalize(gravity*e_down-a), b2=normalize(b3 cross h), b1=b2 cross b3.
    Angles are radians, acceleration and specific thrust are m/s^2, and tilt is
    measured between body-down and NED down. Finite yaw is periodic without a
    branch-cut clamp. Near-zero thrust or heading cross products are rejected
    at a numerical 1e-9 threshold; this is not a calibrated physical tolerance.
    angular_error_margin_rad is reserved for future validation by the caller.
    """
    if not _valid_config(config):
        return PlannedAttitudeResult(reason='INVALID_CONFIG')
    acceleration = _acceleration_vector(acceleration_ned)
    if acceleration is None:
        return PlannedAttitudeResult(reason='INVALID_ACCELERATION')
    if not _finite_real(yaw_ref):
        return PlannedAttitudeResult(reason='INVALID_YAW_REF')

    force = (-acceleration[0], -acceleration[1], float(config.gravity) - acceleration[2])
    specific_thrust = math.hypot(*force)
    if not math.isfinite(specific_thrust):
        return PlannedAttitudeResult(reason='NUMERICAL_ERROR')
    if specific_thrust < _DEGENERACY_EPSILON:
        return PlannedAttitudeResult(reason='FREE_FALL', specific_thrust=specific_thrust)

    b3 = tuple(value / specific_thrust for value in force)
    tilt = math.atan2(math.hypot(b3[0], b3[1]), b3[2])
    diagnostics = {'tilt_rad': tilt, 'specific_thrust': specific_thrust}
    if specific_thrust < config.minimum_specific_thrust:
        return PlannedAttitudeResult(reason='SPECIFIC_THRUST_BELOW_MINIMUM', **diagnostics)
    if specific_thrust > config.maximum_specific_thrust:
        return PlannedAttitudeResult(reason='SPECIFIC_THRUST_ABOVE_MAXIMUM', **diagnostics)
    if tilt > config.maximum_tilt_rad:
        return PlannedAttitudeResult(reason='TILT_LIMIT_EXCEEDED', **diagnostics)

    cosine, sine = math.cos(float(yaw_ref)), math.sin(float(yaw_ref))
    b2_unnormalized = (-b3[2] * sine, b3[2] * cosine, b3[0] * sine - b3[1] * cosine)
    heading_norm = math.hypot(*b2_unnormalized)
    if heading_norm < _DEGENERACY_EPSILON:
        return PlannedAttitudeResult(reason='HEADING_DEGENERATE', **diagnostics)
    b2 = tuple(value / heading_norm for value in b2_unnormalized)
    b1 = (b2[1] * b3[2] - b2[2] * b3[1],
          b2[2] * b3[0] - b2[0] * b3[2],
          b2[0] * b3[1] - b2[1] * b3[0])
    rotation = tuple((b1[index], b2[index], b3[index]) for index in range(3))
    return PlannedAttitudeResult(valid=True, reason='OK', rotation_frd_to_ned=rotation,
                                 **diagnostics)

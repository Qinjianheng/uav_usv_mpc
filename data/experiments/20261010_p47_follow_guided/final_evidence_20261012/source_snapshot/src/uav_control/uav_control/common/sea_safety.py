"""Shared final sea-surface safety barrier for NED controllers."""

import math
from dataclasses import dataclass
from enum import Enum


class SeaSafetyState(str, Enum):
    """Runtime state of the final sea-surface control barrier."""

    SAFE = 'SAFE'
    WARNING = 'WARNING'
    BRAKE = 'BRAKE'
    UNRECOVERABLE = 'UNRECOVERABLE'


@dataclass(frozen=True)
class SeaSafetyResult:
    """One evaluation of the final vertical-command safety barrier."""

    state: SeaSafetyState
    command_vz: float
    clearance: float
    response_margin: float
    immediate_margin: float
    stopping_distance: float
    unrecoverable: bool


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def apply_sea_safety_guard(
    current_z,
    current_vz,
    proposed_vz,
    sea_surface_z,
    reserve_clearance,
    response_delay,
    effective_braking_acceleration,
    control_dt,
    warning_margin=0.15,
    maximum_vertical_speed=math.inf,
):
    """Apply the last vertical-command guard after all other limiters."""
    current_z = _finite(current_z, 'current z')
    current_vz = _finite(current_vz, 'current vz')
    proposed_vz = _finite(proposed_vz, 'proposed vz')
    sea_surface_z = _finite(sea_surface_z, 'sea surface z')
    reserve_clearance = max(
        _finite(reserve_clearance, 'reserve clearance'),
        0.0,
    )
    response_delay = max(_finite(response_delay, 'response delay'), 0.0)
    braking = _finite(
        effective_braking_acceleration,
        'effective braking acceleration',
    )
    control_dt = max(_finite(control_dt, 'control dt'), 1e-3)
    warning_margin = max(_finite(warning_margin, 'warning margin'), 0.0)
    if braking <= 0.0:
        raise ValueError('effective braking acceleration must be positive')

    clearance = sea_surface_z - current_z
    descent_speed = max(current_vz, 0.0)
    braking_distance = descent_speed * descent_speed / (2.0 * braking)
    immediate_margin = clearance - braking_distance - reserve_clearance
    response_stopping_distance = (
        descent_speed * response_delay + braking_distance
    )
    response_margin = (
        clearance - response_stopping_distance - reserve_clearance
    )
    unrecoverable = immediate_margin < 0.0

    if unrecoverable:
        state = SeaSafetyState.UNRECOVERABLE
    elif response_margin <= 0.0:
        state = SeaSafetyState.BRAKE
    elif response_margin <= warning_margin:
        state = SeaSafetyState.WARNING
    else:
        state = SeaSafetyState.SAFE

    command_vz = proposed_vz
    if state == SeaSafetyState.WARNING:
        command_vz = min(command_vz, current_vz)
    elif state in (SeaSafetyState.BRAKE, SeaSafetyState.UNRECOVERABLE):
        command_vz = min(command_vz, current_vz - braking * control_dt)
    command_vz = max(
        min(command_vz, maximum_vertical_speed),
        -maximum_vertical_speed,
    )
    return SeaSafetyResult(
        state,
        command_vz,
        clearance,
        response_margin,
        immediate_margin,
        response_stopping_distance + reserve_clearance,
        unrecoverable,
    )

# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Check ideal FRD-to-NED planning attitudes with independent physical fixtures."""

from importlib import import_module
import math

import numpy as np
import pytest


def _plan(acceleration, yaw=0.0, **config_values):
    module = import_module('uav_control.guidance.planned_attitude')
    return module.planned_attitude(acceleration, yaw, module.AttitudeConfig(**config_values))


def test_hover_has_level_frd_axes_and_upward_specific_thrust():
    """Catch gravity signs or body-axis conventions which tilt a hovering vehicle."""
    from uav_control.guidance.planned_attitude import planned_attitude

    result = planned_attitude((0.0, 0.0, 0.0), 0.0)
    assert result.valid and result.reason == 'OK'
    assert result.tilt_rad == pytest.approx(0.0)
    assert result.specific_thrust == pytest.approx(9.80665)
    np.testing.assert_allclose(result.rotation_frd_to_ned, np.eye(3), atol=1e-12)


def test_north_acceleration_pitches_nose_down_with_thrust_along_negative_body_z():
    """Catch a reversed thrust sign or treating FRD z as an upward body axis."""
    result = _plan((5.0, 0.0, 0.0), gravity=10.0)
    assert result.valid
    np.testing.assert_allclose(result.rotation_frd_to_ned, (
        (0.894427190999916, 0.0, -0.447213595499958),
        (0.0, 1.0, 0.0),
        (0.447213595499958, 0.0, 0.894427190999916),
    ), atol=1e-12)
    assert result.tilt_rad == pytest.approx(0.463647609000806)
    assert result.specific_thrust == pytest.approx(11.18033988749895)


def test_east_acceleration_rolls_right_while_north_yaw_stays_level():
    """Catch mixing the NED east axis with leftward FLU or a pitch acceleration."""
    result = _plan((0.0, 5.0, 0.0), gravity=10.0)
    assert result.valid
    np.testing.assert_allclose(result.rotation_frd_to_ned, (
        (1.0, 0.0, 0.0),
        (0.0, 0.894427190999916, -0.447213595499958),
        (0.0, 0.447213595499958, 0.894427190999916),
    ), atol=1e-12)


def test_forward_acceleration_at_east_yaw_tilts_body_forward_to_east():
    """Catch applying yaw to world acceleration twice or with the wrong sign."""
    result = _plan((0.0, 5.0, 0.0), math.pi / 2, gravity=10.0)
    assert result.valid
    np.testing.assert_allclose(result.rotation_frd_to_ned, (
        (0.0, -1.0, 0.0),
        (0.894427190999916, 0.0, -0.447213595499958),
        (0.447213595499958, 0.0, 0.894427190999916),
    ), atol=1e-12)


def test_left_turn_from_east_facing_banks_toward_north():
    """Catch aligning roll with heading rather than world centripetal acceleration."""
    result = _plan((5.0, 0.0, 0.0), math.pi / 2, gravity=10.0)
    assert result.valid
    np.testing.assert_allclose(result.rotation_frd_to_ned, (
        (0.0, -0.894427190999916, -0.447213595499958),
        (1.0, 0.0, 0.0),
        (0.0, -0.447213595499958, 0.894427190999916),
    ), atol=1e-12)


@pytest.mark.parametrize('down_acceleration,expected_thrust', [(2.0, 7.80665), (-2.0, 11.80665)])
def test_vertical_acceleration_changes_specific_thrust_without_tilt(
        down_acceleration, expected_thrust):
    """Catch NED vertical acceleration adding to thrust with the wrong sign."""
    result = _plan((0.0, 0.0, down_acceleration))
    assert result.valid
    assert result.specific_thrust == pytest.approx(expected_thrust)
    assert result.tilt_rad == pytest.approx(0.0)
    np.testing.assert_allclose(result.rotation_frd_to_ned, np.eye(3), atol=1e-12)


def test_combined_acceleration_uses_flat_yaw_rather_than_strict_euler_heading():
    """Catch substituting Euler yaw which changes the chosen flat-output convention."""
    result = _plan((4.0, 3.0, -1.0), gravity=10.0)
    assert result.valid
    matrix = np.asarray(result.rotation_frd_to_ned)
    # Hand-derived: h=north, b2 has zero north component; b1 points slightly west.
    assert matrix[0, 1] == pytest.approx(0.0)
    assert matrix[1, 0] == pytest.approx(-0.08710300577093119)
    assert math.atan2(matrix[1, 0], matrix[0, 0]) == pytest.approx(-0.09204684886340674)


@pytest.mark.parametrize('acceleration,yaw', [
    ((0.0, 0.0, 0.0), -2.8),
    ((2.0, -1.0, -1.5), 1.2),
    ((-3.0, 4.0, 1.0), -1.4),
])
def test_rotation_is_proper_and_reconstructs_ned_acceleration(acceleration, yaw):
    """Catch nonorthogonal axes, reflections, or rotations transposed at the API boundary."""
    result = _plan(acceleration, yaw, maximum_tilt_rad=0.8)
    assert result.valid
    assert isinstance(result.rotation_frd_to_ned, tuple)
    assert all(isinstance(row, tuple) and len(row) == 3 for row in result.rotation_frd_to_ned)
    matrix = np.asarray(result.rotation_frd_to_ned)
    np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)
    assert np.linalg.det(matrix) == pytest.approx(1.0)
    recovered = (0.0, 0.0, 9.80665) - result.specific_thrust * matrix[:, 2]
    np.testing.assert_allclose(recovered, acceleration, atol=1e-12)


def test_large_tilt_is_rejected_with_usable_dynamic_diagnostics():
    """Catch silently saturating a requested attitude or losing infeasibility diagnostics."""
    result = _plan((10.0, 0.0, 0.0), gravity=10.0)
    assert not result.valid and result.reason == 'TILT_LIMIT_EXCEEDED'
    assert result.rotation_frd_to_ned is None
    assert result.tilt_rad == pytest.approx(math.pi / 4)
    assert result.specific_thrust == pytest.approx(14.14213562373095)


def test_tilt_boundary_is_inclusive_and_a_larger_tilt_is_rejected():
    """Catch weakening the configured tilt limit or rejecting the exact feasible boundary."""
    result = _plan((10.0 * math.tan(0.55), 0.0, 0.0), gravity=10.0)
    assert result.valid
    outside = _plan((10.0 * math.tan(0.55001), 0.0, 0.0), gravity=10.0)
    assert not outside.valid and outside.reason == 'TILT_LIMIT_EXCEEDED'


@pytest.mark.parametrize('specific_thrust', [2.0, 18.0])
def test_specific_thrust_bounds_are_inclusive(specific_thrust):
    """Catch interpreting valid minimum or maximum thrust as an open interval."""
    result = _plan((0.0, 0.0, 9.80665 - specific_thrust))
    assert result.valid
    assert result.specific_thrust == pytest.approx(specific_thrust)


@pytest.mark.parametrize('specific_thrust,reason', [
    (1.99999, 'SPECIFIC_THRUST_BELOW_MINIMUM'),
    (18.00001, 'SPECIFIC_THRUST_ABOVE_MAXIMUM'),
])
def test_infeasible_specific_thrust_retains_the_computed_value(specific_thrust, reason):
    """Catch clipping thrust to its feasible interval or reporting a usable rotation."""
    result = _plan((0.0, 0.0, 9.80665 - specific_thrust))
    assert not result.valid and result.reason == reason
    assert result.rotation_frd_to_ned is None
    assert result.specific_thrust == pytest.approx(specific_thrust)
    assert result.tilt_rad == pytest.approx(0.0)


def test_free_fall_has_no_defined_attitude_and_does_not_fall_back_to_hover():
    """Catch normalizing zero thrust or substituting a level vehicle for free fall."""
    result = _plan((0.0, 0.0, 9.80665))
    assert not result.valid and result.reason == 'FREE_FALL'
    assert result.rotation_frd_to_ned is None and result.tilt_rad is None
    assert result.specific_thrust == pytest.approx(0.0)


def test_near_free_fall_is_explicitly_degenerate_even_with_tiny_minimum_thrust():
    """Catch amplifying a numerically zero force direction into a nominal attitude."""
    result = _plan((0.0, 0.0, 10.0 - 1e-12), gravity=10.0,
                   minimum_specific_thrust=1e-14)
    assert not result.valid and result.reason == 'FREE_FALL'
    assert result.rotation_frd_to_ned is None and result.tilt_rad is None
    assert 0.0 < result.specific_thrust < 1e-9


def test_near_heading_singularity_is_rejected_without_arbitrary_axis_fallback():
    """Catch normalizing a nearly parallel body-down axis and flat heading."""
    result = _plan((-10.0, 0.0, 10.0 - 1e-10), gravity=10.0,
                   maximum_tilt_rad=math.nextafter(math.pi / 2, 0.0))
    assert not result.valid and result.reason == 'HEADING_DEGENERATE'
    assert result.rotation_frd_to_ned is None
    assert result.tilt_rad == pytest.approx(math.pi / 2, abs=1e-9)
    assert result.specific_thrust == pytest.approx(10.0)


def test_inverted_required_thrust_is_rejected_by_tilt_limit():
    """Catch using absolute body-down z which hides an inverted thrust requirement."""
    result = _plan((0.0, 0.0, 12.80665))
    assert not result.valid and result.reason == 'TILT_LIMIT_EXCEEDED'
    assert result.tilt_rad == pytest.approx(math.pi)
    assert result.specific_thrust == pytest.approx(3.0)


@pytest.mark.parametrize('values', [
    {'gravity': 0.0}, {'gravity': -1.0}, {'gravity': math.inf}, {'gravity': True},
    {'gravity': '9.8'}, {'minimum_specific_thrust': 0.0},
    {'minimum_specific_thrust': -1.0}, {'maximum_specific_thrust': 1.0},
    {'maximum_specific_thrust': math.nan}, {'maximum_specific_thrust': False},
    {'maximum_tilt_rad': -0.1}, {'maximum_tilt_rad': math.pi / 2},
    {'maximum_tilt_rad': math.nan}, {'maximum_tilt_rad': True},
    {'angular_error_margin_rad': -0.01}, {'angular_error_margin_rad': math.pi / 2},
    {'angular_error_margin_rad': math.inf}, {'angular_error_margin_rad': True},
])
def test_invalid_model_parameters_are_rejected_before_geometry(values):
    """Catch accepting unsupported model limits or nonfinite configuration values."""
    result = _plan((0.0, 0.0, 0.0), **values)
    assert not result.valid and result.reason == 'INVALID_CONFIG'
    assert result.rotation_frd_to_ned is None
    assert result.tilt_rad is None and result.specific_thrust is None


@pytest.mark.parametrize('config', [None, {}, 10.0])
def test_wrong_configuration_type_is_rejected(config):
    """Catch interpreting an absent or unrelated object as an attitude configuration."""
    module = import_module('uav_control.guidance.planned_attitude')
    result = module.planned_attitude((0.0, 0.0, 0.0), 0.0, config)
    assert not result.valid and result.reason == 'INVALID_CONFIG'


@pytest.mark.parametrize('acceleration', [
    None, (), (1.0, 2.0), (1.0, 2.0, 3.0, 4.0), ((0.0, 0.0, 0.0),),
    (math.nan, 0.0, 0.0), (0.0, math.inf, 0.0), (0.0, 0.0, -math.inf),
    (True, 0.0, 0.0), ('0.0', 0.0, 0.0), (0.0, 0.0, 1j), '000',
])
def test_invalid_acceleration_has_no_physical_diagnostics(acceleration):
    """Catch coercing malformed vectors, booleans, or nonfinite acceleration inputs."""
    result = _plan(acceleration)
    assert not result.valid and result.reason == 'INVALID_ACCELERATION'
    assert result.rotation_frd_to_ned is None
    assert result.tilt_rad is None and result.specific_thrust is None


@pytest.mark.parametrize('yaw', [math.nan, math.inf, -math.inf, True, None, '0', (0.0,)])
def test_invalid_yaw_ref_is_rejected(yaw):
    """Catch treating nonfinite yaw or coerced text as an orientation reference."""
    result = _plan((0.0, 0.0, 0.0), yaw)
    assert not result.valid and result.reason == 'INVALID_YAW_REF'
    assert result.rotation_frd_to_ned is None
    assert result.tilt_rad is None and result.specific_thrust is None


def test_nonfinite_derived_thrust_is_rejected_as_a_numerical_error():
    """Catch overflowing a finite acceleration into a valid or silently clipped force."""
    result = _plan((1.6e308, 1.6e308, 0.0))
    assert not result.valid and result.reason == 'NUMERICAL_ERROR'
    assert result.rotation_frd_to_ned is None
    assert result.tilt_rad is None and result.specific_thrust is None


def test_yaw_crossing_pi_preserves_the_same_physical_rotation():
    """Catch yaw branch cuts changing the planned attitude at equivalent references."""
    left = _plan((2.0, 1.0, 0.0), math.pi - 1e-6)
    right = _plan((2.0, 1.0, 0.0), -math.pi - 1e-6)
    assert left.valid and right.valid
    np.testing.assert_allclose(left.rotation_frd_to_ned, right.rotation_frd_to_ned, atol=1e-12)
    after = _plan((2.0, 1.0, 0.0), -math.pi + 1e-6)
    assert after.valid
    assert np.max(np.abs(np.asarray(left.rotation_frd_to_ned)
                         - np.asarray(after.rotation_frd_to_ned))) < 3e-6


def test_angular_error_margin_is_an_unused_interface_without_attitude_inflation():
    """Catch applying an uncalibrated future error margin to nominal pose or feasibility."""
    zero = _plan((5.0, 0.0, 0.0), gravity=10.0, angular_error_margin_rad=0.0)
    large = _plan((5.0, 0.0, 0.0), gravity=10.0, angular_error_margin_rad=0.4)
    assert zero.valid and large.valid
    assert zero == large


def test_zero_tilt_limit_and_single_thrust_value_allow_exact_hover():
    """Catch rejecting a valid closed zero-tilt or singleton specific-thrust interval."""
    result = _plan((0.0, 0.0, 0.0), gravity=10.0, minimum_specific_thrust=10.0,
                   maximum_specific_thrust=10.0, maximum_tilt_rad=0.0)
    assert result.valid

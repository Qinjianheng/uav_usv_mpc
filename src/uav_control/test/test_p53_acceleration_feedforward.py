"""Nominal velocity tracking keeps the analytic horizontal acceleration feedforward."""
from types import SimpleNamespace
import math

import pytest

from uav_control.control.trajectory_tracker_node import flight_command_to_setpoint
from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState
from uav_control.guidance import follow_minco_execution as execution


def curve(acceleration=(0., 2., 7.)):
    return SimpleNamespace(start=100., end=101., sample=lambda t:
                           (0., 0., -5., 4., 0., 0., *acceleration, 0., 0.))


def test_feedforward_is_derivative_of_reference_velocity_plus_position_feedback():
    value = execution.reference_acceleration(curve(), 100.1, 100.08, (4., -1., 0.), .8, 3.)
    assert value[:2] == pytest.approx((0., 2.8))
    assert math.isnan(value[2])


def test_horizontal_feedforward_is_bounded_without_vertical_override():
    value = execution.reference_acceleration(curve((3., 4., -9.)), 100.1, 100.08,
                                             (4., 0., 100.), .8, 3.)
    assert value[:2] == pytest.approx((1.8, 2.4))
    assert math.isnan(value[2])


@pytest.mark.parametrize('now,navigation,velocity', [
    (100.1, 100.2, (4., 0., 0.)), (101.1, 101., (4., 0., 0.)),
    (100.1, 100.08, (math.nan, 0., 0.)),
])
def test_invalid_feedforward_input_never_produces_a_setpoint(now, navigation, velocity):
    with pytest.raises(ValueError):
        execution.reference_acceleration(curve(), now, navigation, velocity, .8, 3.)


def command():
    return SimpleNamespace(mode='VELOCITY', position=(0., 0., -5.), velocity=(4., 0., 0.),
                           acceleration=(1., 2., math.nan))


def test_original_follow_adapter_keeps_acceleration_unset_by_default():
    message = flight_command_to_setpoint(command(), 100000)
    assert all(math.isnan(v) for v in message.acceleration)
    assert all(math.isnan(v) for v in message.position)
    assert tuple(message.velocity) == (4., 0., 0.)


def test_explicit_nominal_feedforward_reaches_velocity_setpoint_with_vertical_unset():
    message = flight_command_to_setpoint(command(), 100000, acceleration_feedforward=True)
    assert tuple(message.acceleration[:2]) == (1., 2.) and math.isnan(message.acceleration[2])
    assert tuple(message.velocity) == (4., 0., 0.)
    assert all(math.isnan(v) for v in message.position)


@pytest.mark.parametrize('previous,desired,raw,expected', [
    ((6.5, 0., 0.), (6.8, 0., 0.), (1.6, 0., math.nan), (0., 0.)),
    ((0., 0., 0.), (4., 0., 0.), (0., 2., math.nan), (3., 0.)),
    ((6.5, 0., 0.), (6.5, 0., 0.), (1., 2., math.nan), (0., 2.)),
])
def test_feedforward_follows_actual_speed_and_acceleration_shaping(
        previous, desired, raw, expected):
    core = FlightGuidanceCore(maximum_horizontal_speed=6.5)
    core.previous_velocity = previous
    emitted = core._velocity_command(
        FlightKinematicState((0., 0., -5.), previous), desired, .05, False, True)
    acceleration = execution.shaped_acceleration(
        raw, desired, emitted.velocity, previous, .05, 3., 6.5)
    assert acceleration[:2] == pytest.approx(expected)
    assert math.isnan(acceleration[2])


def test_unlimited_velocity_keeps_analytic_turning_feedforward():
    value = execution.shaped_acceleration(
        (0., 2.8, math.nan), (4., 0., 0.), (4., 0., 0.), (4., -.1, 0.), .05, 3., 6.5)
    assert value[:2] == (0., 2.8)

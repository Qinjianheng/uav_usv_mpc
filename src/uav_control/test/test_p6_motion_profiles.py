"""P6 motion requirements, independent of ROS and Gazebo."""

import importlib
import hashlib
import math
import struct

import numpy as np
import pytest


def motion_api():
    # An assertion gives a useful RED before the new module exists.
    assert importlib.util.find_spec('uav_control.usv_motion_profiles'), (
        'P6 pure motion module is missing')
    return importlib.import_module('uav_control.usv_motion_profiles')


@pytest.mark.parametrize('mode', [
    'linear', 'figure_eight', 'variable_speed_eight',
    'variable_speed_s', 'variable_speed_z',
])
def test_five_profiles_start_at_rest_and_preserve_initial_heave(mode):
    api = motion_api()
    profile = api.UsvMotionProfile(mode)
    state = profile.state()
    assert state.position == pytest.approx((8.0, 0.0, 0.0), abs=1e-12)
    assert state.velocity[:2] == (0.0, 0.0)
    assert state.yaw == pytest.approx(math.pi / 2)
    assert state.velocity[2] == pytest.approx(.15 * 2 * math.pi * .25)


def test_original_figure_eight_matches_frozen_pre_p6_values_exactly():
    api = motion_api()
    profile = api.UsvMotionProfile('figure_eight')
    fixtures = {
        1: (8.000000078124998, 0.0024999999983638397,
            3.1250000223688625e-06, 0.04999999990234376),
        20: (8.003445856655787, 0.5249849206535487,
             0.013129146352483578, 0.9999138090435873),
        80: (8.852424886726624, 8.038809588663717,
             0.8752957620683154, 3.9030574334622847),
        300: (41.687050204749056, 6.233831479706006,
              2.882317735232317, -2.773489584109745),
        1200: (9.889918048066328, -11.575904207945573,
               -1.3962160568422293, 3.7484104261161075),
        2400: (11.585243694333634, -15.068338686145582,
               -2.1304750073770307, 3.3854211322879526),
    }
    for index in range(1, 2401):
        state = profile.advance(.05)
        if index in fixtures:
            assert state.position[:2] + state.velocity[:2] == fixtures[index]


def test_original_figure_eight_all_2400_samples_and_heave_are_byte_exact():
    api = motion_api()
    profile = api.UsvMotionProfile('figure_eight')
    fingerprint = hashlib.sha256()
    for _ in range(2400):
        state = profile.advance(.05)
        fingerprint.update(struct.pack(
            '<8d', *state.position, *state.velocity, state.phase, profile.elapsed_time))
    # Frozen from the original FigureEightTrajectory + moving_target equations.
    assert fingerprint.hexdigest() == (
        'dff0070a3491e036be9e607f8920ff37f1ef067a409fd4bbc6498f3c7858fb69')


def test_legacy_linear_preserves_direction_and_semi_implicit_startup():
    api = motion_api()
    parameters = api.MotionParameters(linear_vx=3., linear_vy=4.)
    profile = api.UsvMotionProfile('linear', parameters)
    state = profile.advance(.05)
    assert state.position[:2] == (8.0015, .002)
    assert state.velocity[:2] == (.03, .04)
    for _ in range(79):
        state = profile.advance(.05)
    assert math.hypot(*state.velocity[:2]) == pytest.approx(4.)
    assert state.position[:2] == pytest.approx((12.86, 6.48))


@pytest.mark.parametrize('mode', [
    'variable_speed_eight', 'variable_speed_s', 'variable_speed_z',
])
def test_variable_modes_obey_all_dynamics_for_120_seconds(mode):
    api = motion_api()
    parameters = api.MotionParameters()
    profile = api.UsvMotionProfile(mode, parameters)
    previous = profile.state()
    samples = []
    for index in range(2400):
        state = profile.advance(.05)
        speed = math.hypot(*state.velocity[:2])
        acceleration = math.hypot(*state.acceleration[:2])
        assert np.all(np.isfinite(
            state.position + state.velocity + state.acceleration + (
                state.yaw, state.turn_rate, state.curvature,
                state.angular_acceleration)))
        assert 0. <= speed <= 4. + 1e-10
        assert abs(state.longitudinal_acceleration) <= 1. + 1e-10
        assert abs(state.lateral_acceleration) <= 3.2 + 1e-10
        assert acceleration <= parameters.maximum_horizontal_acceleration
        assert abs(state.turn_rate) <= .65 + 1e-10
        assert abs(state.angular_acceleration) <= .35 + 1e-10
        numerical_acceleration = np.linalg.norm(
            (np.array(state.velocity[:2])
             - np.array(previous.velocity[:2])) / .05)
        assert numerical_acceleration <= 3.2 + 1e-6
        assert abs(state.turn_rate - previous.turn_rate) / .05 <= .35 + 1e-6
        if index >= 160:
            assert 2. - 1e-10 <= speed <= 4. + 1e-10
        previous = state
        samples.append(state)
    speeds = [math.hypot(*state.velocity[:2]) for state in samples[160:]]
    assert min(speeds) < 2.05 and max(speeds) > 3.95
    if mode == 'variable_speed_s':
        rates = [state.turn_rate for state in samples[160:1200]]
        assert min(rates) < -.1 and max(rates) > .1
    if mode == 'variable_speed_z':
        stages = {state.stage for state in samples[160:1200]}
        assert {'straight', 'turn_left', 'diagonal', 'turn_right'} <= stages
        assert sum(abs(state.turn_rate) < 1e-12 for state in samples) > 800


@pytest.mark.parametrize('mode', [
    'variable_speed_eight', 'variable_speed_s', 'variable_speed_z',
])
def test_analytic_velocity_and_acceleration_match_independent_differences(mode):
    api = motion_api()
    profile = api.UsvMotionProfile(mode)
    previous = profile.state()
    for _ in range(6000):
        state = profile.advance(.005)
        midpoint_velocity = .5 * (
            np.array(previous.velocity) + np.array(state.velocity))
        assert (np.array(state.position) - previous.position) / .005 == (
            pytest.approx(midpoint_velocity, abs=3e-5))
        midpoint_acceleration = .5 * (
            np.array(previous.acceleration) + np.array(state.acceleration))
        assert (np.array(state.velocity) - previous.velocity) / .005 == (
            pytest.approx(midpoint_acceleration, abs=3e-5))
        previous = state


def test_motion_selection_ack_is_independent_idempotent_and_start_locked():
    api = motion_api()
    selector = api.MotionModeSelector()
    assert selector.status().mode == 'figure_eight'
    assert selector.status().mode_id == 2
    assert selector.select('4').successful
    assert selector.status().mode == 'variable_speed_s'
    assert selector.select('variable_speed_s').successful
    assert not selector.select('6').successful
    assert selector.status().mode_id == 4
    selector.start()
    assert not selector.select('linear').successful
    assert selector.status().mode_id == 4
    assert selector.select('4').successful
    assert selector.status().started


def test_strict_motion_clock_freezes_pause_rollback_and_discontinuity():
    api = motion_api()
    assert api.elapsed_motion_step(None, 1_000_000_000) == 0.
    assert api.elapsed_motion_step(1_000_000_000, 1_000_000_000) == 0.
    assert api.elapsed_motion_step(2_000_000_000, 1_000_000_000) == 0.
    assert api.elapsed_motion_step(1_000_000_000, 3_000_000_000) == 0.
    assert api.elapsed_motion_step(1_000_000_000, 1_087_000_000) == (
        pytest.approx(.087))


@pytest.mark.parametrize('mode', [
    'linear', 'figure_eight', 'variable_speed_eight',
    'variable_speed_s', 'variable_speed_z',
])
def test_actual_dt_determinism_and_zero_dt_do_not_advance(mode):
    api = motion_api()
    first, second = api.UsvMotionProfile(mode), api.UsvMotionProfile(mode)
    for dt in [.037, .09, .003, .12, .05] * 100:
        assert first.advance(dt) == second.advance(dt)
    before = first.state()
    assert first.advance(0.) == before
    assert first.elapsed_time == pytest.approx(30.)
    with pytest.raises(ValueError):
        first.advance(-.1)
    with pytest.raises(ValueError):
        first.advance(float('nan'))


@pytest.mark.parametrize('parameters', [
    {'variable_speed_min': 0.},
    {'variable_speed_min': 4.1},
    {'variable_speed_period': 0.},
    {'variable_startup_duration': .1},
    {'maximum_angular_acceleration': .001},
    {'maximum_horizontal_acceleration': .1},
    {'s_heading_amplitude': float('nan')},
])
def test_new_modes_reject_infeasible_or_invalid_configuration(parameters):
    api = motion_api()
    with pytest.raises(ValueError):
        api.UsvMotionProfile('variable_speed_s', api.MotionParameters(**parameters))


def test_new_parameters_change_motion_without_changing_legacy_default():
    api = motion_api()
    parameters = api.MotionParameters(
        variable_speed_min=2.5, variable_speed_max=3.5,
        variable_speed_period=30., s_heading_amplitude=.3)
    profile = api.UsvMotionProfile('variable_speed_s', parameters)
    states = [profile.advance(.05) for _ in range(1000)]
    speeds = [math.hypot(*s.velocity[:2]) for s in states[160:]]
    assert min(speeds) >= 2.5 - 1e-10
    assert max(speeds) <= 3.5 + 1e-10
    assert max(abs(s.yaw - math.pi / 2) for s in states) < .301


@pytest.mark.parametrize('mode,time', [
    ('variable_speed_s', 4.), ('variable_speed_z', 9.5),
])
def test_first_maneuver_turns_left_in_ned_from_original_positive_y(mode, time):
    api = motion_api()
    sample = api.UsvMotionProfile(mode).advance(time)
    # Looking along +Y (east), a left turn points toward +X (north).
    assert sample.velocity[0] > 0.
    assert sample.turn_rate < 0.


def test_legacy_zero_direction_does_not_report_fictitious_acceleration():
    api = motion_api()
    profile = api.UsvMotionProfile('linear', api.MotionParameters(
        linear_vx=0., linear_vy=0.))
    state = profile.advance(.5)
    assert state.position[:2] == (8., 0.)
    assert state.velocity[:2] == (0., 0.)
    assert state.acceleration[:2] == (0., 0.)


@pytest.mark.parametrize('mode,joins', [
    ('variable_speed_s', [8., 24., 32.]),
    ('variable_speed_z', [7., 8., 12., 20., 25., 32., 39.]),
])
def test_heading_speed_acceleration_are_continuous_at_all_segment_joins(mode, joins):
    api = motion_api()
    for join in joins:
        profile = api.UsvMotionProfile(mode)
        before = profile.advance(join - 1e-5)
        after = profile.advance(2e-5)
        assert np.linalg.norm(np.array(after.velocity) - before.velocity) < 1e-4
        assert np.linalg.norm(np.array(after.acceleration) - before.acceleration) < 1e-4
        assert abs(after.turn_rate - before.turn_rate) < 1e-5
        assert abs(after.angular_acceleration - before.angular_acceleration) < 1e-4

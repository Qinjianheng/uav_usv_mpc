"""Bounded selection, anti-chatter and differentiated observation references."""

from dataclasses import replace
import math

import numpy as np
import pytest

from uav_control.guidance.adaptive_viewpoint import (
    AdaptiveViewpointConfig, AdaptiveViewpointSelector, ViewpointState, profile_state,
)
from uav_control.guidance.follow_reference import viewpoint_state


def test_default_candidates_keep_follow_distance_and_side_rear_bounds():
    selector = AdaptiveViewpointSelector()
    grid = selector.candidates
    assert (5., 0.) in grid
    assert min(d for d, _ in grid) >= 4.5
    assert max(d for d, _ in grid) <= 6.
    assert max(abs(b) for _, b in grid) <= math.radians(20)
    result = selector.select(.1, np.zeros(len(grid)))
    assert result.state == ViewpointState()
    assert result.target == (5., 0.)


def test_higher_risk_has_cost_and_does_not_allow_unbounded_retreat():
    selector = AdaptiveViewpointSelector(AdaptiveViewpointConfig(hysteresis_cost=0.))
    risks = np.array([100. if d <= 5. else 0. for d, b in selector.candidates])
    for _ in range(150):
        result = selector.select(.1, risks)
        assert 4.5 <= result.state.d <= 6.
        assert abs(result.state.d_dot) <= selector.config.maximum_distance_rate + 1e-12
        assert abs(result.state.d_ddot) <= selector.config.maximum_distance_acceleration + 1e-12
    assert result.state.d > 5.
    assert result.state.d <= 6.


def test_small_alternating_preferences_do_not_switch_sides():
    config = AdaptiveViewpointConfig(hysteresis_cost=.2, minimum_dwell_time=.5,
                                     switch_weight=0., follow_error_weight=0.)
    selector = AdaptiveViewpointSelector(config)
    switches = 0
    for i in range(200):
        risks = np.array([1. + (.01 if b * (-1)**i > 0 else 0.)
                          for _, b in selector.candidates])
        result = selector.select(.05, risks)
        switches += result.switched
    assert switches == 0
    assert selector.state.beta == 0.


def test_no_feasible_candidate_preserves_target_and_explicit_reason():
    selector = AdaptiveViewpointSelector()
    result = selector.select(.1, np.full(len(selector.candidates), np.inf))
    assert not result.valid and result.reason == 'NO_FEASIBLE_VIEWPOINT'
    assert result.state == ViewpointState()
    assert result.target == (5., 0.)


def test_reference_derivatives_match_independent_central_difference():
    state = ViewpointState(d=5.1, beta=.12, d_dot=.2, d_ddot=-.1,
                           beta_dot=.04, beta_ddot=-.03)
    t0, h0, w0, alpha = 0., .7, .2, -.06

    def position(t):
        s = state.at_time(t)
        p = np.array((1. + 2.*t + .15*t*t, 2. - t + .1*t*t, 0.))
        theta = h0 + w0*t + .5*alpha*t*t + s.beta
        return np.array((p[0] - s.d*math.cos(theta), p[1] - s.d*math.sin(theta), -5.))

    p, v, a = viewpoint_state(
        (1., 2., 0.), (2., -1., 0.), (.3, .2, 0.), h0, w0, alpha,
        **state.reference_kwargs())
    eps = 1e-4
    np.testing.assert_allclose(p, position(t0), atol=1e-12)
    np.testing.assert_allclose(v, (position(eps) - position(-eps))/(2*eps), atol=1e-7)
    np.testing.assert_allclose(a, (position(eps) - 2*position(0.) + position(-eps))/eps**2,
                               atol=2e-7)


def test_quintic_profile_retains_analytic_derivatives_and_bounded_full_horizon():
    config = AdaptiveViewpointConfig(hysteresis_cost=0., minimum_dwell_time=0.,
                                     follow_error_weight=0., switch_weight=0.)
    selector = AdaptiveViewpointSelector(config)
    dt = .05
    for i in range(200):
        target_beta = math.radians(20) if i < 100 else -math.radians(20)
        risks = np.array([(b-target_beta)**2 + (d-5.)**2
                          for d, b in selector.candidates])
        expected = selector.forecast(dt)
        previously_moving = abs(selector.state.beta_dot) > 1e-10
        result = selector.select(dt, risks)
        s = result.state
        assert abs(s.beta_dot) <= config.maximum_beta_rate + 1e-12
        assert abs(s.beta_ddot) <= config.maximum_beta_acceleration + 1e-12
        if previously_moving:
            assert s == expected
        assert config.minimum_distance <= s.d <= config.maximum_distance
        assert abs(s.beta) <= config.maximum_beta + 1e-12
        for time in np.linspace(0., 1.6, 9):
            future = selector.forecast(time)
            assert config.minimum_distance <= future.d <= config.maximum_distance
            assert abs(future.beta) <= config.maximum_beta + 1e-12
            assert abs(future.beta_dot) <= config.maximum_beta_rate + 1e-12
            assert abs(future.beta_ddot) <= config.maximum_beta_acceleration + 1e-12


def test_profile_prediction_derivatives_and_serialization_are_independently_reproducible():
    selector = AdaptiveViewpointSelector(AdaptiveViewpointConfig(
        hysteresis_cost=0., follow_error_weight=0., smooth_weight=0., switch_weight=0.))
    risks = np.array([(d-6.)**2 + (b-math.radians(20))**2
                      for d, b in selector.candidates])
    selector.select(.1, risks)
    t, eps = 1., 1e-4
    before, now, after = (selector.forecast(x) for x in (t-eps, t, t+eps))
    assert now.d_dot == pytest.approx((after.d-before.d)/(2*eps), abs=1e-8)
    assert now.d_ddot == pytest.approx((after.d-2*now.d+before.d)/eps**2, abs=2e-7)
    assert now.beta_dot == pytest.approx((after.beta-before.beta)/(2*eps), abs=1e-8)
    assert now.beta_ddot == pytest.approx((after.beta-2*now.beta+before.beta)/eps**2, abs=2e-7)
    restored = type(selector.profile).from_tuple(selector.profile.as_tuple())
    assert restored.forecast(1.6) == selector.forecast(1.6)
    terminal = selector.profile.forecast(selector.profile.duration)
    assert terminal.d == pytest.approx(6.)
    assert terminal.beta == pytest.approx(math.radians(20))
    assert terminal.d_dot == terminal.d_ddot == terminal.beta_dot == terminal.beta_ddot == 0.
    assert profile_state(selector.reference_profile(), 1.6) == selector.forecast(1.6)


def test_duration_zero_profile_is_only_a_constant_reference():
    profile = dict(duration=0., elapsed=0., distance_coefficients=(5., 0., 0., 0., 0., 0.),
                   beta_coefficients=(.1, 0., 0., 0., 0., 0.))
    assert profile_state(profile, 1.6) == ViewpointState(d=5., beta=.1)
    profile['distance_coefficients'] = (5., .1, 0., 0., 0., 0.)
    with pytest.raises(ValueError, match='INVALID_VIEWPOINT_PROFILE'):
        profile_state(profile, 1.6)


def test_nested_profile_cannot_hide_wrong_per_axis_coefficient_lengths():
    profile = (1., 0., (5., 0., 0., 0., 0., 0., 0.), (0., 0., 0., 0., 0.))
    with pytest.raises(ValueError, match='INVALID_VIEWPOINT_PROFILE'):
        profile_state(profile, .1)


def test_constant_acceleration_probe_rejects_interior_excursion_and_large_horizon():
    state = ViewpointState(d=5.95, d_dot=.15, d_ddot=-.15)
    with pytest.raises(ValueError, match='VIEWPOINT_FORECAST_OUT_OF_BOUNDS'):
        state.forecast(2.)
    with pytest.raises(ValueError, match='INVALID_VIEWPOINT_FORECAST_TIME'):
        state.forecast(50.)


@pytest.mark.parametrize('dt', [0., -.1, float('nan'), 1.1, None, '0.1'])
def test_bad_dt_fails_without_mutating_selection(dt):
    selector = AdaptiveViewpointSelector()
    result = selector.select(dt, np.zeros(len(selector.candidates)))
    assert not result.valid and result.reason == 'INVALID_SELECTION_DT'
    assert selector.state == ViewpointState()


def test_config_and_initial_state_outside_domain_rejected():
    with pytest.raises(ValueError, match='INVALID_ADAPTIVE_VIEWPOINT_CONFIG'):
        AdaptiveViewpointSelector(replace(AdaptiveViewpointConfig(), minimum_distance=7.))
    with pytest.raises(ValueError, match='INVALID_INITIAL_VIEWPOINT'):
        AdaptiveViewpointSelector(initial_state=ViewpointState(d=20.))


def test_active_infeasible_target_is_failure_without_abrupt_profile_switch():
    selector = AdaptiveViewpointSelector(AdaptiveViewpointConfig(
        hysteresis_cost=0., follow_error_weight=0., smooth_weight=0., switch_weight=0.))
    risks = np.array([(d-6.)**2 + b*b for d, b in selector.candidates])
    first = selector.select(.1, risks)
    previous = selector.profile
    expected = previous.forecast(.1)
    risks[first.candidate_index] = math.inf
    result = selector.select(.1, risks)
    assert not result.valid and result.reason == 'ACTIVE_VIEWPOINT_BECAME_INFEASIBLE'
    assert selector.profile.coefficients == previous.coefficients
    assert selector.profile.elapsed == previous.elapsed+.1
    assert result.state == expected


def test_new_preference_starts_at_current_epoch_without_backdating_motion():
    selector = AdaptiveViewpointSelector(AdaptiveViewpointConfig(
        hysteresis_cost=0., follow_error_weight=0., smooth_weight=0., switch_weight=0.))
    risks = np.array([(d-6.)**2 + b*b for d, b in selector.candidates])
    result = selector.select(.1, risks)
    assert result.switched
    assert selector.state == ViewpointState()
    assert selector.profile.elapsed == 0.
    expected = selector.forecast(.1)
    selector.select(.1, risks)
    assert selector.state == expected
    assert selector.profile.elapsed == .1

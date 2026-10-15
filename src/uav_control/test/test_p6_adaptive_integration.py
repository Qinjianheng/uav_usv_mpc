"""Explicit adaptive references change the real research rollout, retaining guards."""

from dataclasses import replace
import math

import numpy as np
import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.guidance.adaptive_viewpoint import (
    AdaptiveViewpointConfig, AdaptiveViewpointSelector, profile_state,
)
from uav_control.guidance.follow_problem import FollowProblem, future_request
from uav_control.guidance.follow_reference import forecast_viewpoint
from uav_control.guidance.follow_rollout import rollout_follow, rollout_follow_fast


def profiled_request(raw=None, constant=False):
    raw = raw or request(2.)
    if constant:
        profile = (0., 0., (5.5, 0., 0., 0., 0., 0.), (.15, 0., 0., 0., 0., 0.))
    else:
        selector = AdaptiveViewpointSelector(AdaptiveViewpointConfig(
            hysteresis_cost=0., smooth_weight=0., follow_error_weight=0., switch_weight=0.))
        risks = [(d-6.)**2 + (b-math.radians(20))**2 for d, b in selector.candidates]
        selector.select(.1, risks)
        metadata = selector.reference_profile()
        profile = (metadata['duration'], metadata['elapsed'], metadata['distance_coefficients'],
                   metadata['beta_coefficients'])
    return replace(raw, viewpoint_profile=profile)


def test_empty_profile_preserves_default_reference_and_rollout_exactly():
    raw = request(2.)
    plain = FollowProblem(raw, model(), 1.6)
    disabled = FollowProblem(replace(raw, viewpoint_profile=()), model(), 1.6)
    ts = np.linspace(0., 1.6, 9)
    for old, new in zip(plain.reference(ts), disabled.reference(ts)):
        assert np.array_equal(old, new)
    for rollout in (rollout_follow, rollout_follow_fast):
        assert rollout(plain) == rollout(disabled)


def test_profile_reference_pva_matches_finite_difference_and_future_execution_offset():
    req = profiled_request()
    problem = FollowProblem(future_request(req, .15), model(), 1.6)
    time, eps = .6, 1e-3
    before, now, after = (forecast_viewpoint(problem, t) for t in (time-eps, time, time+eps))
    np.testing.assert_allclose(now[1], (after[0]-before[0])/(2*eps), atol=1e-7)
    np.testing.assert_allclose(now[2], (after[0]-2*now[0]+before[0])/eps**2, atol=3e-7)
    duration, elapsed, dc, bc = req.viewpoint_profile
    expected = profile_state(dict(duration=duration, elapsed=elapsed,
                                  distance_coefficients=dc, beta_coefficients=bc), time+.15)
    target, _ = problem.target_state((time,))
    np.testing.assert_allclose(now[0][:2], target[0, :2]-expected.d*np.array(
        (math.cos(expected.beta), math.sin(expected.beta))), atol=1e-12)
    ref_p, ref_v, heading = problem.reference((time,))
    np.testing.assert_allclose(ref_p[0], now[0], atol=1e-12)
    np.testing.assert_allclose(ref_v[0], now[1], atol=1e-12)
    assert heading[0] == pytest.approx(now[3])


def test_adaptive_main_and_fast_rollout_change_commands_without_changing_target_geometry():
    raw = request(2.)
    plain = FollowProblem(raw, model(), 1.6)
    adaptive = FollowProblem(profiled_request(raw, constant=True), model(), 1.6)
    before_target = adaptive.target_state((0., .8, 1.6))
    nominal, changed = rollout_follow(plain), rollout_follow(adaptive)
    fast = rollout_follow_fast(adaptive)
    assert nominal.commands != changed.commands
    assert changed.metrics['adaptive_viewpoint']
    np.testing.assert_allclose(changed.positions, fast.positions, atol=1e-12)
    np.testing.assert_allclose(changed.commands, fast.commands, atol=1e-12)
    assert np.max(np.linalg.norm(np.asarray(changed.commands)[:, :2], axis=1)) <= 6.2+1e-12
    commands = np.vstack((raw.state[3:6], changed.commands))
    assert np.max(np.linalg.norm(np.diff(commands[:, :2], axis=0), axis=1)) <= 3.*.05+1e-12
    after_target = adaptive.target_state((0., .8, 1.6))
    for before, after in zip(before_target, after_target):
        assert np.array_equal(before, after)


def test_adaptive_rollout_retains_the_original_sea_guard():
    raw = replace(request(2.), state=(-5., 0., -.55, 2., 0., 1., 0., 0., 0., 0.))
    problem = FollowProblem(profiled_request(raw, constant=True), model(), 1.6)
    result = rollout_follow(problem)
    original = rollout_follow(FollowProblem(raw, model(), 1.6))
    assert result.commands[0][2] == original.commands[0][2]
    assert 'UNRECOVERABLE' in result.metrics['safety_states']


def test_profile_completion_is_c2_and_target_heave_never_enters_altitude():
    req = profiled_request(constant=True)
    heave = replace(req, target_positions=tuple((p[0], p[1], .15*math.sin(t))
                                                for p, t in zip(req.target_positions,
                                                                req.prediction_times)),
                    target_velocities=tuple((2., 0., .15*math.cos(t))
                                            for t in req.prediction_times))
    problem = FollowProblem(heave, model(), 1.6)
    for time in np.linspace(0., 1.6, 9):
        p, v, a, _, _ = forecast_viewpoint(problem, time)
        assert p[2] == -5. and v[2] == 0. and a[2] == 0.

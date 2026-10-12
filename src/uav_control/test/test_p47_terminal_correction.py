"""Bounded terminal MINCO derivatives, immutable accepted starts and full admission."""
from dataclasses import replace

import numpy as np
import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.controllers.direct_reference_minco import follow_guided_seed
from uav_control.guidance.follow_minco_optimizer import _trajectory
from uav_control.guidance.follow_problem import FollowProblem, future_request
from uav_control.guidance.follow_rollout import rollout_follow


def setup_objective(mode='va'):
    from uav_control.guidance.follow_terminal_correction import TerminalCorrectionObjective
    raw = replace(request(2.), state=(-7., .2, -4.8, 1.8, 0., 0., .1, .05, -.1, 0.))
    req = future_request(raw, .15)
    problem = FollowProblem(req, model(), 1.2)
    reference = rollout_follow(problem)
    seed = follow_guided_seed(req, problem, reference)
    return req, seed, TerminalCorrectionObjective(problem, seed, reference, mode=mode)


@pytest.mark.parametrize('mode,size', [('a', 3), ('va', 6), ('qva', 12)])
def test_terminal_linear_map_and_full_objective_gradient(mode, size):
    req, seed, objective = setup_objective(mode)
    x = np.linspace(-.02, .02, size)
    candidate = objective.seed_from(x)
    assert candidate.start == seed.start == req.state
    assert candidate.end[:3] == seed.end[:3]
    assert candidate.durations == seed.durations
    assert candidate.yaw == seed.yaw
    curve = _trajectory(candidate, candidate.q, candidate.durations)
    assert objective.coefficients(x) == pytest.approx(curve.coefficients, abs=2e-10)
    value, gradient = objective(x)
    numerical = []
    for index in range(size):
        direction = np.zeros(size)
        direction[index] = 1e-5
        numerical.append((objective(x+direction)[0]-objective(x-direction)[0])/2e-5)
    assert np.isfinite(value)
    assert gradient == pytest.approx(numerical, rel=3e-4, abs=3e-5)


@pytest.mark.parametrize('mode', ['a', 'va', 'qva'])
def test_box_quadratic_correction_is_bounded_and_retains_height_reference(mode):
    _, seed, objective = setup_objective(mode)
    x = objective.fixed()
    bounds = np.asarray(objective.bounds)
    assert np.all(x >= bounds[:, 0]-1e-12)
    assert np.all(x <= bounds[:, 1]+1e-12)
    candidate = objective.seed_from(x)
    assert candidate.start == seed.start
    assert candidate.end[2] == seed.end[2]
    assert objective.quadratic(x)[0] <= objective.quadratic(np.zeros(len(x)))[0]+1e-10
    bad = x.copy()
    bad[0] = bounds[0, 1]+.01
    with pytest.raises(ValueError, match='TERMINAL_BOUNDS'):
        objective.seed_from(bad)


def test_exact_jerk_envelope_gradient_matches_independent_extrema():
    from uav_control.guidance.polynomial_extrema import derivative_peak
    _, seed, objective = setup_objective('qva')
    x = np.linspace(-.01, .01, 12)
    value, gradient, peak = objective.jerk_peak(x)
    curve = _trajectory(objective.seed_from(x), objective.seed_from(x).q, seed.durations)
    exact = derivative_peak(curve.coefficients, seed.durations, 3, (0, 1))
    assert value == pytest.approx(exact['value'])
    assert peak['piece'] == exact['piece']
    numerical = []
    for index in range(len(x)):
        direction = np.zeros(len(x))
        direction[index] = 1e-6
        numerical.append((objective.jerk_peak(x+direction)[0]
                          - objective.jerk_peak(x-direction)[0])/2e-6)
    assert gradient == pytest.approx(numerical, abs=2e-5, rel=3e-4)


@pytest.mark.parametrize('mode', ['bad', '', None])
def test_unknown_terminal_mode_rejected(mode):
    with pytest.raises(ValueError, match='TERMINAL_MODE'):
        setup_objective(mode)

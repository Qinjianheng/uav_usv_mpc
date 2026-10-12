"""Opt-in scalar integration preserves the 20 Hz original FOLLOW control law."""
from dataclasses import replace
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.guidance.follow_problem import FollowProblem, future_request
from uav_control.guidance.follow_rollout import FollowRolloutConfig, rollout_follow


def fast():
    import uav_control.guidance.follow_rollout as module
    assert hasattr(module, 'rollout_follow_fast'), 'P4.7 opt-in scalar rollout is missing'
    return module.rollout_follow_fast


def problem(height=-5., horizon=1.2, turn=.0):
    raw = replace(request(2.), state=(-8., 1., height, 1., .2, .1, .2, -.1, .05, 0.))
    if turn:
        ts = np.asarray(raw.prediction_times)
        raw = replace(raw, target_positions=tuple(map(tuple, np.c_[
            2*np.sin(turn*ts)/turn, 2*(1-np.cos(turn*ts))/turn, .15*np.sin(ts)])),
                      target_velocities=tuple(map(tuple, np.c_[
                          2*np.cos(turn*ts), 2*np.sin(turn*ts), .15*np.cos(ts)])))
    req = future_request(raw, .15)
    return FollowProblem(req, model(), horizon)


@pytest.mark.parametrize('height', [-5., -3., -7., -.3])
@pytest.mark.parametrize('horizon', [1.2, 1.6])
def test_scalar_10ms_matches_every_state_command_and_metric(height, horizon):
    p = problem(height, horizon, .3)
    initial = (1.2, -.1, .1)
    slow = rollout_follow(p, initial_command=initial)
    quick = fast()(p, initial_command=initial)
    assert quick.times == slow.times
    for field in ('positions', 'velocities', 'accelerations', 'commands'):
        np.testing.assert_allclose(getattr(quick, field), getattr(slow, field),
                                   atol=3e-13, rtol=0.)
    assert quick.metrics == slow.metrics
    for epoch in (.01, .123, horizon/3, horizon):
        for actual, expected in zip(quick.sample(epoch), slow.sample(epoch)):
            np.testing.assert_allclose(actual, expected, atol=3e-13, rtol=0.)


def test_deadline_interrupts_inside_first_control_interval():
    ticks = iter(np.arange(0., 1., .001))
    with pytest.raises(ValueError, match='DEADLINE_EXCEEDED'):
        fast()(problem(), deadline=.005, clock=lambda: next(ticks))


@pytest.mark.parametrize('initial', [
    (float('nan'), 0., 0.), (7., 0., 0.), (0., 0.), (0., 0., 5.), [[1.], [0.], [0.]], 2.])
def test_invalid_initial_command_fails_closed(initial):
    with pytest.raises(ValueError, match='INVALID_FOLLOW_INITIAL_COMMAND'):
        fast()(problem(), initial_command=initial)


def test_submillisecond_step_limit_is_checked_before_target_allocation():
    with pytest.raises(ValueError, match='FOLLOW_ROLLOUT_STEP_LIMIT'):
        fast()(problem(), FollowRolloutConfig(integration_dt=1e-12))


@pytest.mark.parametrize('step', [.005, .01, .02])
def test_each_explicit_step_still_uses_original_control_period_and_altitude(step):
    # Zero initial vertical state is preserved under target heave.
    raw = replace(request(2.), state=(-8., 1., -5., 1., .2, 0., .2, -.1, 0., 0.))
    req = future_request(raw, .15)
    p = FollowProblem(req, model(), 1.2)
    config = FollowRolloutConfig(integration_dt=step)
    slow, quick = rollout_follow(p, config), fast()(p, config)
    assert quick.metrics['control_dt'] == .05
    assert len(quick.commands) == 24
    assert np.asarray(quick.positions)[:, 2] == pytest.approx(-5.)
    np.testing.assert_allclose(quick.positions, slow.positions, atol=3e-13, rtol=0.)


def test_overspeed_measurement_is_preserved_while_initial_command_is_clipped():
    raw = replace(request(2.), state=(-8., 1., -5., 6.3, 0., 0., 0., 0., 0., 0.))
    req = future_request(raw, .15)
    p = FollowProblem(req, model(), 1.2)
    actual = fast()(p)
    assert actual.velocities[0][0] == 6.3
    assert actual.metrics['command_initialization_clipped']
    assert max(np.linalg.norm(np.asarray(actual.commands)[:, :2], axis=1)) <= 6.2


def test_common_grid_sampling_matches_reference_interface_and_rejects_extrapolation():
    script = Path(__file__).resolve().parents[3] / 'scripts/p47_rollout_performance.py'
    assert script.exists(), 'P4.7 rollout comparison script is missing'
    spec = importlib.util.spec_from_file_location('p47_rollout_performance', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    reference = fast()(problem())
    epochs = np.array([0., .023, .05, .4, 1.2])
    batched = module.reference_samples(reference, epochs)
    for i, epoch in enumerate(epochs):
        for actual, expected in zip(batched, reference.sample(epoch)):
            np.testing.assert_allclose(actual[i], expected, atol=1e-14, rtol=0.)
    with pytest.raises(ValueError, match='OUTSIDE'):
        module.reference_samples(reference, np.array([-.001]))

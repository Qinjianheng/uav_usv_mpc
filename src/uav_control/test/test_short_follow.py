"""Short local progress uses the original MINCO map and full camera acceptance."""
from dataclasses import replace
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from follow_minco_experiment import scenario  # noqa: E402
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig  # noqa: E402
from uav_control.controllers.short_follow_solver import (  # noqa: E402
    ShortHorizonFollowSolver, local_seed,
)


def model():
    return FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))


@pytest.mark.parametrize('duration', [.8, 1.2, 1.6])
def test_local_seed_is_known_cubic_with_exact_boundary_and_minco_jerk(duration):
    from uav_control.guidance.follow_minco_optimizer import _trajectory
    request = scenario('constant_velocity')
    jerk = np.array((.2, -.1, .05))
    seed = local_seed(request, duration, jerk, .1)
    curve = _trajectory(seed, seed.q, seed.durations)
    assert np.array_equal(seed.start, request.state)
    for t in (0., duration/4, duration*.8, duration):
        sample = curve.sample(t)
        p, v, a = np.array(request.state[:9]).reshape(3, 3)
        assert np.allclose(sample.position, p+v*t+a*t*t/2+jerk*t**3/6, atol=1e-9)
        assert np.allclose(sample.jerk, jerk, atol=1e-9)
    assert not seed.final_validated


def test_short_result_requires_full_geometry_and_preserves_epochs():
    request = scenario('constant_velocity')
    solver = ShortHorizonFollowSolver(model(), wall_clock=lambda: request.now_stamp)
    result = solver.solve(request)
    assert result.valid and result.metrics['full_projection_validated']
    assert result.relative_times[-1] == pytest.approx(1.2)
    assert result.context == request.context
    assert not result.metrics['short_follow']['accepted_by_tracker']
    assert result.metrics['short_follow']['terminal_policy'] == 'local_progress'


def test_short_budget_failure_clears_rolling_hint():
    request = scenario('constant_velocity')
    solver = ShortHorizonFollowSolver(model(), wall_clock=lambda: request.now_stamp)
    assert solver.solve(request).valid
    assert solver.hint is not None
    solver.wall_clock = lambda: request.now_stamp+.2
    assert not solver.solve(request).valid
    assert solver.hint is None


def test_rolling_hint_requires_new_source_same_mission_generation_and_unexpired_input():
    request = scenario('constant_velocity')
    solver = ShortHorizonFollowSolver(model(), wall_clock=lambda: request.now_stamp)
    assert solver.solve(request).valid
    newer = replace(request, now_stamp=request.now_stamp+.01, context=replace(
        request.context, prediction_source_stamp=request.context.prediction_source_stamp+.01))
    assert solver.usable_hint(newer)
    for change in ({'mission_id': 2}, {'clock_generation': 10}):
        assert not solver.usable_hint(replace(newer, context=replace(newer.context, **change)))
    assert not solver.usable_hint(request)
    assert not solver.usable_hint(replace(newer, now_stamp=request.now_stamp+.2))


@pytest.mark.parametrize('duration', [.1, 2.4, math.nan])
def test_short_horizon_configuration_is_explicit_and_bounded(duration):
    with pytest.raises(ValueError):
        ShortHorizonFollowSolver(model(), duration=duration)


def test_short_wrong_yaw_is_not_admitted_by_position_or_small_jerk():
    request = scenario('wrong_yaw')
    result = ShortHorizonFollowSolver(model(), wall_clock=lambda: request.now_stamp).solve(request)
    assert not result.valid

"""Independent cubic/progress cases; hypothetical state is never physical navigation."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from follow_minco_experiment import scenario  # noqa: E402
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig  # noqa: E402
from uav_control.controllers.progress_follow_solver import (  # noqa: E402
    ProgressFollowSolver, ProgressWeights, optimal_local_jerk,
)


def model():
    return FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))


def test_velocity_and_acceleration_damping_at_zero_position_error():
    j = optimal_local_jerk((0, 0, -5, 3, 0, 0, 1, 0, 0), (0, 0, -5),
                           (0, 0, 0), 1.2, ProgressWeights(), np.zeros(3))
    assert j[0] < 0 and np.array_equal(j[1:], (0., 0.))


def test_zero_error_constant_velocity_is_equilibrium():
    j = optimal_local_jerk((0, 0, -5, 2, 0, 0, 0, 0, 0), (2.4, 0, -5),
                           (2, 0, 0), 1.2, ProgressWeights(), np.zeros(3))
    assert np.allclose(j, 0)


def test_progress_moves_toward_distant_reference_and_reports_independent_dimensions():
    r = scenario('constant_velocity')
    r = replace(r, state=(-9., 0., -5., *r.state[3:6], 0., 0., 0., 0.))
    s = ProgressFollowSolver(model(), wall_clock=lambda: r.now_stamp)
    out = s.solve(r)
    assert out.valid
    assert out.positions[-1][0] > -9.
    assert out.metrics['progress']['position_improvement'] > 0
    assert out.metrics['progress']['stage_C_progress']
    assert not out.metrics['progress']['stage_D_closed_loop_proven']
    assert out.metrics['full_projection_validated']


def test_math_hint_survives_200ms_but_not_mission_generation_source_reset():
    r = scenario('constant_velocity')
    s = ProgressFollowSolver(model(), wall_clock=lambda: r.now_stamp)
    assert s.solve(r).valid
    newer = replace(r, now_stamp=r.now_stamp+.2, context=replace(
        r.context, navigation_stamp=r.now_stamp+.2, attitude_stamp=r.now_stamp+.2,
        observation_stamp=r.now_stamp+.2, prediction_source_stamp=r.now_stamp+.2,
        execution_start_stamp=r.now_stamp+.2, prediction_sequence_id=2,
        prediction_valid_until=r.now_stamp+.325))
    assert s.usable_hint(newer)
    for change in ({'mission_id': 2}, {'clock_generation': 9},
                   {'prediction_sequence_id': 0}, {'frame_id': 'enu'}):
        assert not s.usable_hint(replace(newer, context=replace(newer.context, **change)))
    s.wall_clock = lambda: newer.now_stamp
    out = s.solve(newer)
    assert out.metrics['progress']['warm_used']
    assert not out.metrics['progress']['hint_grants_execution']


def test_invalid_weights_and_horizon_are_rejected():
    with pytest.raises(ValueError):
        ProgressFollowSolver(model(), weights=ProgressWeights(position=-1.))
    with pytest.raises(ValueError):
        ProgressFollowSolver(model(), duration=2.4)

"""Dynamic-reference equilibria and explicit policy ablations, independent of simulation."""
from dataclasses import replace

import numpy as np
import pytest

from test_progress_follow import model
from test_follow_research_algorithms import request
from uav_control.controllers.progress_follow_solver import ProgressWeights, optimal_local_jerk


def test_constant_acceleration_reference_is_exact_equilibrium_of_the_objective():
    # Independent kinematics: a=0.5 over 1.2s => p=2.76, v=2.6, a=0.5.
    x = (0., 0., -5., 2., 0., 0., .5, 0., 0.)
    j = optimal_local_jerk(x, (2.76, 0., -5.), (2.6, 0., 0.), 1.2,
                           ProgressWeights(), np.zeros(3), reference_a=(.5, 0., 0.))
    assert np.linalg.norm(j) < 1e-12
    old = optimal_local_jerk(x, (2.76, 0., -5.), (2.6, 0., 0.), 1.2,
                             ProgressWeights(), np.zeros(3))
    assert old[0] < -.2


def test_dynamic_solver_supports_same_local_objective_at_24s_without_changing_p33():
    from uav_control.controllers.progress_follow_solver import TrackingFollowSolver
    from uav_control.controllers.progress_follow_solver import ProgressFollowSolver
    req = request(2.)
    solver = TrackingFollowSolver(model(), duration=2.4, wall_clock=lambda: req.now_stamp)
    result = solver.solve(req)
    assert result.valid and result.metrics['progress']['duration'] == 2.4
    assert result.metrics['progress']['acceleration_target'] == 'bounded'
    with pytest.raises(ValueError):
        ProgressFollowSolver(model(), duration=2.4)


def test_low_speed_and_prediction_edge_derivatives_explicitly_report_support():
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.follow_reference import forecast_viewpoint
    _, _, a, _, details = forecast_viewpoint(FollowProblem(request(), model()), 0.)
    assert np.array_equal(a, np.zeros(3)) and not details['derivative_available']
    _, _, _, _, details = forecast_viewpoint(FollowProblem(request(2.), model()), 0.)
    assert details['fit_samples'] >= 3 and details['derivative_available']


def test_reference_acceleration_validation_and_none_ablation():
    from uav_control.controllers.progress_follow_solver import TrackingFollowSolver
    with pytest.raises(ValueError):
        TrackingFollowSolver(model(), acceleration_target='mystery')
    weights = replace(ProgressWeights(), acceleration=0.)
    x = (0., 0., -5., 2., 0., 0., .5, 0., 0.)
    j = optimal_local_jerk(x, (2.76, 0., -5.), (2.6, 0., 0.), 1.2,
                           weights, np.zeros(3), reference_a=(9., 0., 0.))
    assert np.linalg.norm(j) < 1e-12

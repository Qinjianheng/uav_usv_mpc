"""Full nominal revalidation against a new causal prediction, never a TTL renewal."""
from types import SimpleNamespace
from dataclasses import replace

import numpy as np

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.controllers.progress_follow_solver import TrackingFollowSolver
from uav_control.guidance.follow_revalidation import revalidate_prediction
from uav_control.guidance.follow_problem import future_request


def inputs():
    req = future_request(request(2.), .15)
    result = TrackingFollowSolver(model(), wall_clock=lambda: req.now_stamp).solve(req)
    assert result.valid
    c = req.context
    pred = SimpleNamespace(mission_id=c.mission_id, sequence_id=c.prediction_sequence_id+1,
                           source='tracking', frame_id='local_ned',
                           source_stamp=c.prediction_source_stamp,
                           observation_stamp=c.observation_stamp, generated_stamp=req.now_stamp,
                           valid_until=c.prediction_valid_until,
                           prediction_times=req.prediction_times,
                           target_positions=req.target_positions,
                           target_velocities=req.target_velocities)
    return req, result, pred


def test_same_geometry_new_version_checks_full_curve_preserves_boundary_and_navigation():
    req, output, pred = inputs()
    updated, report = revalidate_prediction(req, output, pred, req.now_stamp, model())
    assert report['valid'] and report['samples'] >= 25
    assert updated.context.prediction_sequence_id == pred.sequence_id
    assert updated.context.navigation_stamp == req.context.navigation_stamp
    assert updated.context.execution_start_stamp == req.context.execution_start_stamp
    assert updated.state == req.state and not report['holding_qualified']


def test_changed_prediction_behind_camera_rejects_even_small_solver_cost():
    req, output, pred = inputs()
    pred.target_positions = tuple((x-100, y, z) for x, y, z in pred.target_positions)
    _, report = revalidate_prediction(req, output, pred, req.now_stamp, model())
    assert not report['valid'] and report['reason'] == 'PREDICTION_REVALIDATION_FAILED'


def test_new_prediction_cannot_refresh_old_navigation_or_missed_start():
    req, output, pred = inputs()
    for now in (req.context.navigation_stamp+.126, req.context.execution_start_stamp+.001):
        _, report = revalidate_prediction(req, output, pred, now, model())
        assert not report['valid']
    pred.mission_id += 1
    _, report = revalidate_prediction(req, output, pred, req.now_stamp, model())
    assert report['reason'] == 'MISSION_CHANGED'


def test_dynamics_and_prediction_coverage_are_revalidated():
    req, output, pred = inputs()
    bad = np.asarray(output.metrics['xyz_coefficients']).copy()
    bad[0, 3, 0] = 50.
    output = replace(output, metrics=dict(output.metrics, xyz_coefficients=bad.tolist()))
    _, report = revalidate_prediction(req, output, pred, req.now_stamp, model())
    assert not report['valid']
    pred.prediction_times = (0., .1)
    _, report = revalidate_prediction(req, output, pred, req.now_stamp, model())
    assert not report['valid']


def test_nonfinite_generated_epoch_is_not_a_causal_new_prediction():
    req, output, pred = inputs()
    pred.generated_stamp = float('nan')
    _, report = revalidate_prediction(req, output, pred, req.now_stamp, model())
    assert not report['valid'] and report['reason'] == 'INVALID_PREDICTION'

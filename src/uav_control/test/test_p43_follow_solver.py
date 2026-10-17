"""Fast ranking never admits a proxy-only curve; PVA/yaw handover stays exact."""
from dataclasses import replace
import numpy as np
import pytest

from test_follow_revalidation import inputs
from test_progress_follow import model
from uav_control.controllers import p43_follow_solver as solver
from uav_control.guidance.follow_profile import CycleProfile
from uav_control.guidance.follow_problem import hypothetical_request
from uav_control.guidance.follow_contract import FollowCurve


def to_curve(req, result):
    """Only a synthetic coefficient carrier for exact handover comparison."""
    m, c = result.metrics, req.context
    return FollowCurve(1, c.mission_id, c.clock_generation, c.prediction_sequence_id, 0,
                       c.navigation_stamp, c.attitude_stamp, c.observation_stamp,
                       c.prediction_source_stamp, req.now_stamp+.125, req.now_stamp+4.,
                       c.execution_start_stamp, c.execution_start_stamp+sum(m['durations']), 0.,
                       tuple(m['durations']), tuple(map(tuple, np.asarray(
                           m['xyz_coefficients']).reshape(-1, 3))),
                       tuple(map(tuple, np.asarray(m['yaw_coefficients'])[::-1].T)))


def test_single_construction_full_final_validation_and_pva_boundary():
    req, _, _ = inputs()
    with CycleProfile(10) as profile:
        instance = solver.P43FollowSolver(model(), refinement='none',
                                          wall_clock=lambda: req.now_stamp)
        result = instance.solve(req)
    assert result.valid and result.metrics['full_projection_validated']
    assert profile.report()['counts']['minco_builds'] == 1
    assert result.metrics['validation_samples'] >= 25
    assert result.metrics['boundary_residual'] < 1e-7
    assert np.allclose(result.positions[0], req.state[:3])
    assert not result.metrics['p43']['holding_qualified']


@pytest.mark.parametrize('refinement', ('none', 'q', 'qt'))
def test_refinement_never_replaces_validated_seed_with_unsafe_curve(refinement):
    req, _, _ = inputs()
    instance = solver.P43FollowSolver(model(), refinement=refinement,
                                      wall_clock=lambda: req.now_stamp)
    result = instance.solve(req)
    assert result.valid and result.metrics['full_projection_validated']
    previous = to_curve(req, result)
    epoch = req.context.execution_start_stamp+.2
    next_context = replace(
        req.context, navigation_stamp=req.context.navigation_stamp+.2,
        attitude_stamp=req.context.attitude_stamp+.2,
        observation_stamp=req.context.observation_stamp+.2,
        prediction_source_stamp=req.context.prediction_source_stamp+.2)
    payload = replace(req, now_stamp=req.now_stamp+.2, context=next_context)
    handover = hypothetical_request(payload, previous, epoch)
    instance.wall_clock = lambda: handover.now_stamp
    next_result = instance.solve(handover)
    if next_result.valid:
        assert np.allclose(to_curve(handover, next_result).sample(epoch),
                           previous.sample(epoch), atol=1e-6)


def test_bad_forecast_and_expired_raw_input_are_not_proxy_qualified():
    req, _, _ = inputs()
    bad = replace(req, target_positions=tuple((x-100, y, z) for x, y, z in req.target_positions))
    assert not solver.P43FollowSolver(model(), wall_clock=lambda: req.now_stamp).solve(bad).valid
    expired = solver.P43FollowSolver(model(), wall_clock=lambda: req.now_stamp+.13).solve(req)
    assert not expired.valid


@pytest.mark.parametrize('method', ('__call__', '__init__'))
def test_refinement_numerical_failure_retains_full_validated_baseline(monkeypatch, method):
    def failure(*args, **kwargs):
        raise ValueError('bad gradient')
    req, _, _ = inputs()
    monkeypatch.setattr(solver.TrackingMincoObjective, method, failure)
    result = solver.P43FollowSolver(model(), wall_clock=lambda: req.now_stamp).solve(req)
    assert result.valid and result.metrics['full_projection_validated']

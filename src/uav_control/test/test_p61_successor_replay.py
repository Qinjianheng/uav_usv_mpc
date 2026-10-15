"""Real failed-flight requests, actual MINCO math, and unchanged deadline rejection."""
from dataclasses import asdict
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from uav_control.controllers import follow_guided_feasible, p43_follow_solver
from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
from uav_control.guidance import camera_visibility_batch
from uav_control.guidance.camera_visibility import (
    CameraExtrinsics, CameraIntrinsics, TargetBoundingSphere, VisibilityConstraints,
    evaluate_visibility,
)
from uav_control.guidance.follow_fast_validation import assess_curve
from uav_control.guidance.follow_minco_execution import request_from_payload
from uav_control.guidance.follow_revalidation import revalidate_prediction
from uav_control.guidance.planned_attitude import AttitudeConfig


def saved_case(cycle):
    data = json.loads((Path(__file__).parent/'fixtures/p61_supply_cases.json').read_text())
    row = next(r for r in data['cases'] if r['cycle'] == cycle)
    request = request_from_payload(row['request'])
    assert json.loads(json.dumps(asdict(request))) == row['request']
    m = row['model_config']
    mount = dict(m['extrinsics'])
    mount['translation_flu'] = tuple(mount['translation_flu'])
    mount['rotation_camera_to_body_flu'] = tuple(map(tuple, mount['rotation_camera_to_body_flu']))
    sphere = dict(m['target'])
    sphere['center_offset_ned'] = tuple(sphere['center_offset_ned'])
    model = FollowMpcSeed(MpcConfig(**m['mpc']), intrinsics=CameraIntrinsics(**m['intrinsics']),
                          extrinsics=CameraExtrinsics(**mount),
                          target=TargetBoundingSphere(**sphere),
                          visibility=VisibilityConstraints(**m['visibility']),
                          attitude_config=AttitudeConfig(**m['attitude']))
    return row, request, model


def deterministic_solver(row, request, model, clock=lambda: 0.):
    delay = row['worker_timeline']['solve_start_ros']-request.now_stamp
    solver = P47FollowGuidedMinco(
        model, duration=1.6, terminal_mode='qva', refinement=False, rolling=False,
        scheduling='adaptive', online_budget_guard=True, clock=clock,
        wall_clock=lambda: request.now_stamp+delay+clock())
    solver.follow_estimate = row['budget']['follow_estimate']
    solver.fallback_estimate = row['budget']['fallback_estimate']
    return solver


@pytest.mark.parametrize('cycle', (26, 27, 78, 79, 80, 139, 140, 141))
def test_real_failed_requests_solve_and_full_constraints_match_scalar_geometry(cycle, monkeypatch):
    row, request, model = saved_case(cycle)
    result = deterministic_solver(row, request, model).solve(request)
    assert result.valid
    assert result.context == request.context
    vector, _ = assess_curve(request, result.metrics, model, clock=lambda: 0.)

    def scalar(p, r, t, camera, mount, sphere, limits):
        return [evaluate_visibility(pi, ri, ti, camera, mount, sphere, limits)
                for pi, ri, ti in zip(p, r, t)]

    monkeypatch.setattr(camera_visibility_batch, 'visibility_batch', scalar)
    original, _ = assess_curve(request, result.metrics, model, clock=lambda: 0.)
    assert original['valid'] == vector['valid'] is True
    assert original['analytic_peaks'] == vector['analytic_peaks']
    for key in ('minimum_horizontal_margin', 'minimum_vertical_margin'):
        assert original[key] == pytest.approx(vector[key], abs=1e-10)
    assert result.metrics['validation_samples'] >= 33


def test_actual_feasible_candidate_is_rejected_after_original_input_expiry():
    row, request, model = saved_case(27)
    result = deterministic_solver(row, request, model).solve(request)
    assert result.valid
    node = object.__new__(P4FollowPlannerNode)
    node.runner = SimpleNamespace(config=model.config,
                                  mission=dict(mission_id=request.context.mission_id),
                                  _follow=lambda: True)
    node.adapter = SimpleNamespace(clock_generation=request.context.clock_generation)
    node._ros_seconds = lambda: request.context.observation_stamp+.126
    event = dict(request=asdict(request), output=asdict(result),
                 cycle_started_monotonic=time.monotonic())
    assert node._publish_proposal(event) == dict(published=False, reason='STALE_INPUT')


def test_prediction_change_revalidates_real_curve_without_refreshing_navigation_or_acquisition():
    row, request, model = saved_case(26)
    result = deterministic_solver(row, request, model).solve(request)
    c = request.context
    prediction = SimpleNamespace(
        mission_id=c.mission_id, sequence_id=c.prediction_sequence_id+1,
        source=c.prediction_source, frame_id=c.frame_id,
        source_stamp=c.prediction_source_stamp, observation_stamp=c.observation_stamp,
        generated_stamp=request.now_stamp, valid_until=c.prediction_valid_until,
        prediction_times=request.prediction_times, target_positions=request.target_positions,
        target_velocities=request.target_velocities)
    updated, report = revalidate_prediction(
        request, result, prediction, request.now_stamp, model, clock=lambda: 0.)
    assert report['valid'] and report['samples'] >= 33
    assert updated.context.navigation_stamp == c.navigation_stamp
    assert updated.context.observation_stamp == c.observation_stamp
    assert updated.context.prediction_valid_until == c.prediction_valid_until
    prediction.target_positions = tuple((p[0]-100., *p[1:]) for p in request.target_positions)
    _, invalid = revalidate_prediction(
        request, result, prediction, request.now_stamp, model, clock=lambda: 0.)
    assert not invalid['valid']


def test_two_consecutive_real_validations_finishing_late_never_become_valid(monkeypatch):
    original = follow_guided_feasible.validate_seed
    elapsed = [0.]
    checked = []

    def delayed_real_validation(*args, **kwargs):
        value = original(*args, **kwargs)
        checked.append(value)
        elapsed[0] += .13
        return value

    monkeypatch.setattr(follow_guided_feasible, 'validate_seed', delayed_real_validation)
    monkeypatch.setattr(p43_follow_solver, 'validate_seed', delayed_real_validation)
    for cycle in (140, 141):
        row, request, model = saved_case(cycle)
        elapsed[0] = 0.
        result = deterministic_solver(row, request, model, lambda: elapsed[0]).solve(request)
        assert not result.valid
        assert result.solver_status == 'DEADLINE_EXCEEDED'
    assert len(checked) == 2 and all(value.valid for value in checked)

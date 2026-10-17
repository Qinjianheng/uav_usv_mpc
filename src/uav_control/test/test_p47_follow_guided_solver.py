"""Remaining-time scheduling never substitutes deadlines or a forged parent boundary."""
from dataclasses import replace

import numpy as np
import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.guidance.follow_problem import future_request


def solver(req, **kwargs):
    from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
    return P47FollowGuidedMinco(model(), wall_clock=lambda: req.now_stamp, **kwargs)


def test_new_engine_preserves_full_validation_and_fixed_endpoint_height():
    req = future_request(request(2.), .15)
    result = solver(req).solve(req)
    assert result.valid
    assert result.metrics['full_projection_validated']
    detail = result.metrics['follow_guided']
    assert detail['independent_valid']
    assert not detail['fallback_used']
    assert result.positions[0] == pytest.approx(req.state[:3])
    assert result.velocities[0] == pytest.approx(req.state[3:6])
    assert result.accelerations[0] == pytest.approx(req.state[6:9])
    assert result.metrics['end'][2] == pytest.approx(-5.)
    assert np.asarray(result.positions)[:, 2] == pytest.approx(-5.)
    correction = detail['terminal_correction']
    assert correction['boundary_candidate']['delta'] is not correction['delta']
    assert correction['boundary_candidate']['start'] == correction['start']
    assert correction['boundary_candidate']['validation']['status'] == detail['initial_status']


def test_each_attempted_repair_is_recorded_even_when_final_candidate_is_retained(monkeypatch):
    from uav_control.guidance.follow_terminal_correction import TerminalCorrectionObjective
    req = future_request(request(2.), .15)

    def worse(self, x, *args, **kwargs):
        modified = np.asarray(x).copy()
        modified[0] += .01
        return modified, dict(status='test failed improvement', objective_initial=0.,
                              objective_final=1., success=False)

    monkeypatch.setattr(TerminalCorrectionObjective, 'repair', worse)
    result = solver(req, refinement=True).solve(req)
    correction = result.metrics['follow_guided']['terminal_correction']
    assert result.valid
    assert correction['repair_candidate']['delta'] != correction['delta']
    assert correction['repair_candidate']['retained'] is False
    assert correction['repair_candidate']['start'] == correction['start']


@pytest.mark.parametrize('remaining', [.008, .012, .020, .024])
def test_short_budget_uses_existing_p44_without_uniform_25ms_gate(remaining, monkeypatch):
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    req = future_request(request(2.), .15)
    mono = [0.]
    instance = solver(req, clock=lambda: mono[0])
    from uav_control.guidance.fast_follow_minco import freshness
    until, _ = freshness(req, req.now_stamp)
    instance.wall_clock = lambda: until-.02-remaining
    # If initialization is attempted, a numerical failure must still leave fallback usable.

    def fail_initialization(*args, **kwargs):
        raise ValueError('INITIALIZATION_FAILURE')
    monkeypatch.setattr('uav_control.controllers.follow_guided_feasible.rollout_follow_fast',
                        fail_initialization)
    calls = []

    def fallback(candidate):
        calls.append(candidate)
        mono[0] += .001
        return MpcSeedResult(req.context, valid=True, solver_status='SAMPLED_FEASIBLE')

    instance.fallback._solve = fallback
    result = instance.solve(req)
    assert calls == [req]
    assert result.valid
    assert result.metrics['follow_guided']['fallback_used']
    assert result.metrics['follow_guided']['budget']['initial_gate'] != .025
    mono[0] = 0.
    instance.scheduling = 'legacy'
    calls.clear()
    assert not instance.solve(req).valid
    assert not calls


def test_optional_repair_timeout_keeps_independently_valid_boundary(monkeypatch):
    from uav_control.guidance.follow_terminal_correction import TerminalCorrectionObjective
    req = future_request(request(2.), .15)

    def timeout(*args, **kwargs):
        raise TimeoutError('optional bounded work')

    monkeypatch.setattr(TerminalCorrectionObjective, 'repair', timeout)
    result = solver(req, refinement=True).solve(req)
    assert result.valid
    assert result.metrics['follow_guided']['independent_valid']
    assert result.metrics['follow_guided']['refinement_status'] == 'BOUNDED_STOP_RETAIN_VALID'


def test_deadline_crossed_during_validation_invalidates_an_otherwise_valid_curve(monkeypatch):
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    req = future_request(request(2.), .15)
    elapsed = [0.]
    instance = solver(req, clock=lambda: elapsed[0], refinement=False, fallback_enabled=False)
    instance.wall_clock = lambda: req.now_stamp+elapsed[0]

    def late_valid(*args, **kwargs):
        elapsed[0] += .2
        return MpcSeedResult(req.context, valid=True, solver_status='SAMPLED_FEASIBLE')

    monkeypatch.setattr('uav_control.controllers.follow_guided_feasible.validate_seed', late_valid)
    result = instance.solve(req)
    assert not result.valid
    assert result.solver_status == 'DEADLINE_EXCEEDED'
    assert not result.metrics['follow_guided']['independent_valid']
    assert not result.metrics['follow_guided']['fallback_used']


@pytest.mark.parametrize('change', ['truth', 'expired', 'sea', 'behind', 'acceleration'])
def test_full_constraints_and_original_expiry_still_reject(change):
    req = future_request(request(2.), .15)
    if change == 'truth':
        req = replace(req, context=replace(req.context, prediction_source='truth'))
    elif change == 'expired':
        req = replace(req, now_stamp=req.now_stamp+.126)
    elif change == 'sea':
        req = future_request(replace(request(2.), state=(
            -5., 0., -.1, 2., 0., 0., 0., 0., 0., 0.)), .15)
    elif change == 'behind':
        req = replace(req, target_positions=tuple((x-100., y, z)
                                                  for x, y, z in req.target_positions))
    else:
        raw = replace(request(2.), state=(-5., 0., -5., 2., 0., 0., 3.5, 0., 0., 0.))
        req = future_request(raw, .15)
    assert not solver(req).solve(req).valid


def test_prediction_change_and_forged_accepted_parent_cannot_reuse_admission():
    from test_follow_sitl_execution import candidate
    from uav_control.guidance.follow_minco_execution import accepted_request
    req = future_request(request(2.), .15)
    instance = solver(req)
    assert instance.solve(req).valid
    changed = replace(req, context=replace(req.context, prediction_sequence_id=2),
                      target_positions=tuple((x-100., y, z) for x, y, z in req.target_positions))
    assert not instance.solve(changed).valid
    parent = candidate()
    req = replace(req, context=replace(req.context, execution_start_stamp=parent.start+.1))
    accepted = accepted_request(req, parent)
    original = accepted.prior_curve
    result = solver(req).solve(accepted)
    assert accepted.prior_curve == original
    if result.valid:
        assert result.positions[0] == pytest.approx(accepted.state[:3])
    assert not solver(req).solve(replace(accepted, state=(99.,)*10)).valid


def test_explicit_engine_registration_keeps_old_engines_selectable():
    from types import SimpleNamespace
    from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    from uav_control.controllers.follow_research_shadow_node import FollowResearchShadowNode
    for engine, cls in (('p47_follow_guided', P47FollowGuidedMinco),
                        ('follow_guided_minco', FollowGuidedMinco),
                        ('p44_adaptive', P44FollowSolver)):
        parameters = dict(research_mode='greedy_minco', minco_engine=engine, shadow_rate_hz=5.)
        fake = SimpleNamespace(
            declare_parameter=lambda name, value: parameters.setdefault(name, value),
            get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=100_000_000_000)),
            adapter=None, pool=None, _publish_event=lambda event: None)
        runner = FollowResearchShadowNode.make_runner(
            fake, replace(model().config, allow_synthetic_predictions=False))
        assert isinstance(runner.solver, cls)


def test_fallback_invariants_reused_but_new_target_geometry_is_revalidated():
    from types import SimpleNamespace
    from uav_control.guidance.follow_revalidation import revalidate_prediction
    req = future_request(request(2.), .15)
    instance = solver(req)
    result = instance.fallback.solve(req)
    assert result.valid
    assert instance.fallback.invariant_cache is instance.invariant_cache
    c = req.context
    pred = SimpleNamespace(mission_id=c.mission_id, sequence_id=c.prediction_sequence_id+1,
                           source='tracking', frame_id='local_ned',
                           source_stamp=c.prediction_source_stamp,
                           observation_stamp=c.observation_stamp, generated_stamp=req.now_stamp,
                           valid_until=c.prediction_valid_until,
                           prediction_times=req.prediction_times,
                           target_positions=req.target_positions,
                           target_velocities=req.target_velocities)
    _, report = revalidate_prediction(req, result, pred, req.now_stamp, model(),
                                      clock=lambda: 0., cache=instance.invariant_cache)
    assert report['valid']
    pred.target_positions = tuple((x-100., y, z) for x, y, z in pred.target_positions)
    _, report = revalidate_prediction(req, result, pred, req.now_stamp, model(),
                                      clock=lambda: 0., cache=instance.invariant_cache)
    assert not report['valid']

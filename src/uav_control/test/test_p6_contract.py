"""P6 uncertainty is explicit and opt-in; the nominal contract remains identical."""
import json
import ast
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def test_p6_default_does_not_change_nominal_constraint_fingerprint():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_limits import constraint_snapshot
    from uav_control.guidance.p6_follow import P6Policy
    model = FollowMpcSeed()
    original = constraint_snapshot(model)
    model.p6_policy = P6Policy()
    assert constraint_snapshot(model) == original
    model.p6_policy = P6Policy(adaptive_viewpoint=True)
    enhanced = constraint_snapshot(model)
    assert enhanced.fingerprint != original.fingerprint
    assert json.loads(enhanced.payload)['p6']['adaptive_viewpoint'] is True


def test_conditional_predictions_must_match_every_source_identity():
    from uav_control.guidance.p6_follow import attach_model_diagnostics
    from uav_control.controllers.follow_mpc_shadow_node import ShadowPrediction
    prediction = ShadowPrediction(1, 3, 100., 99.99, 100., 100.1,
                                  'local_ned', 'tracking', 'IMM', 1.,
                                  (0., 1.), ((0., 0., 0.), (1., 0., 0.)),
                                  ((1., 0., 0.), (1., 0., 0.)))
    diag = dict(mission_id=1, sequence_id=4, source_stamp=100.,
                observation_stamp=99.99, valid=True)
    with pytest.raises(ValueError, match='P6_MODEL_IDENTITY_MISMATCH'):
        attach_model_diagnostics(prediction, diag)


def test_robust_policy_requires_calibration_and_keeps_soft_fallback_explicit():
    from uav_control.guidance.p6_follow import P6Policy
    with pytest.raises(ValueError, match='P6_CALIBRATION_REQUIRED'):
        P6Policy(visibility_mode='robust').validate()
    assert P6Policy().visibility_mode == 'nominal'
    assert np.isfinite(P6Policy().yaw_error_bound_rad)


def calibrated_policy(**changes):
    from uav_control.guidance.p6_follow import P6Policy
    return P6Policy(visibility_mode='robust', calibration_id='unit-test-envelope',
                    calibration_sha256='test-only', calibration_family='imm', **changes)


def test_uncertainty_checks_reject_family_mismatch_and_missing_conditionals():
    from uav_control.guidance.p6_follow import interpolate_uncertainty
    prediction = SimpleNamespace(model='IMM_CV_CTRV_CTRA', prediction_times=(0., 1.),
                                 target_covariances=(np.eye(3), np.eye(3)), model_positions=())
    with pytest.raises(ValueError, match='FAMILY_MISMATCH'):
        interpolate_uncertainty(prediction, (0., .5), replace(
            calibrated_policy(), calibration_family='bctra'))
    with pytest.raises(ValueError, match='CONDITIONAL_PREDICTIONS_MISSING'):
        interpolate_uncertainty(prediction, (0., .5), calibrated_policy())


def test_receiver_records_fallback_and_strict_mode_rejects_missing_covariance():
    from test_follow_sitl_execution import candidate
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_minco_execution import nominal_curve
    model, curve = FollowMpcSeed(), candidate()
    prediction = SimpleNamespace(source_stamp=100., prediction_times=(0., 4.), model='IMM',
                                 target_positions=((8., 0., 0.), (12., 0., 0.)),
                                 target_covariances=())
    # Exercise missing covariance itself, with full-curve risk checking so a
    # separately tested missing 0.45s prefix endpoint cannot preempt this gate.
    model.p6_policy = calibrated_policy(risk_horizon_seconds=0.)
    detail = {}
    assert nominal_curve(curve, prediction, model, curve.start, curve.end, detail)
    assert detail['p6']['used_nominal_fallback']
    assert not detail['p6']['robust_ok']
    assert detail['p6']['reason'] == 'P6_UNCERTAINTY_MISSING_OR_INVALID'
    model.p6_policy = replace(model.p6_policy, allow_nominal_fallback=False)
    assert not nominal_curve(curve, prediction, model, curve.start, curve.end, detail)
    assert detail['reason'] == 'P6_ROBUST_VISIBILITY_FAILED'


def test_receiver_never_accepts_nominal_failure_through_robust_fallback():
    from test_follow_sitl_execution import candidate
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_minco_execution import nominal_curve
    model, curve = FollowMpcSeed(), candidate()
    model.p6_policy = calibrated_policy()
    prediction = SimpleNamespace(source_stamp=100., prediction_times=(0., 4.), model='IMM',
                                 target_positions=((-8., 0., 0.), (-4., 0., 0.)),
                                 target_covariances=())
    assert not nominal_curve(curve, prediction, model, curve.start, curve.end)


def test_offline_config_reconstruction_matches_nominal_and_adaptive_contract():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_limits import constraint_snapshot
    from uav_control.guidance.p6_follow import P6Policy, model_from_parameters
    assert constraint_snapshot(model_from_parameters({})) == constraint_snapshot(FollowMpcSeed())
    expected = FollowMpcSeed()
    expected.p6_policy = P6Policy(adaptive_viewpoint=True)
    assert constraint_snapshot(model_from_parameters(dict(p6_adaptive_viewpoint=True))) == (
        constraint_snapshot(expected))


@pytest.mark.parametrize('shadow', ('true', 'false'))
@pytest.mark.parametrize('evaluator', ('true', 'false'))
def test_imm_primary_graph_is_unique_with_every_optional_diagnostic_switch(shadow, evaluator):
    path = Path(__file__).resolve().parents[2]/'uav_usv_bringup/launch/modular_intercept.launch.py'
    choices = dict(enable_shadow_perception=shadow, enable_evaluator=evaluator)
    primary = []
    for call in ast.walk(ast.parse(path.read_text())):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            continue
        if call.func.id != 'Node':
            continue
        fields = {item.arg: item.value for item in call.keywords}
        name = fields['name'].value
        if name not in ('target_predictor_node', 'imm_target_predictor_node'):
            continue
        active = 'condition' not in fields or eval(compile(
            ast.Expression(fields['condition']), str(path), 'eval'), dict(
            IfCondition=lambda value: value, PythonExpression=lambda parts: eval(''.join(parts)),
            LaunchConfiguration=lambda key: choices[key]))
        if active and (name == 'target_predictor_node'
                       or '/planning/target_prediction' in ast.unparse(fields['parameters'])):
            primary.append(name)
    assert primary == ['imm_target_predictor_node']


def test_revalidation_preserves_adaptive_reference_at_absolute_execution_time():
    from test_follow_revalidation import inputs
    from test_progress_follow import model
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.follow_reference import forecast_viewpoint
    from uav_control.guidance.follow_revalidation import revalidate_prediction
    req, output, pred = inputs()
    req = replace(req, viewpoint_profile=(
        3., .2, (5., .1, 0., 0., 0., 0.), (0., 0., 0., 0., 0., 0.)))
    before = forecast_viewpoint(FollowProblem(req, model(), 1.2), .3)
    revised, report = revalidate_prediction(
        req, output, pred, req.now_stamp+.02, model(), clock=lambda: 0.)
    assert report['valid']
    after = forecast_viewpoint(FollowProblem(revised, model(), 1.2), .3)
    assert np.allclose(before[0], after[0], rtol=0., atol=1e-9)
    assert np.allclose(before[1], after[1], rtol=0., atol=1e-9)


def test_robust_execution_scope_is_explicit_and_does_not_claim_the_full_curve():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.p6_follow import assess_risk
    model = FollowMpcSeed()
    model.p6_policy = calibrated_policy(risk_horizon_seconds=.45)
    prediction = SimpleNamespace(model='IMM_CV_CTRV_CTRA', prediction_times=(0., 1.6),
                                 target_covariances=(np.eye(3)*.01, np.eye(3)*.01),
                                 model_positions=np.tile((8., 0., 0.), (3, 2, 1)).tolist(),
                                 model_covariances=np.tile(np.eye(3)*.01, (3, 2, 1, 1)).tolist(),
                                 model_probabilities=(.5, .3, .2))
    query = np.array((0., .2, .45, .8, 1.6))
    p, target = np.tile((0., 0., -5.), (5, 1)), np.tile((8., 0., 0.), (5, 1))
    report, details = assess_risk(
        model, prediction, query, p, np.tile(np.eye(3), (5, 1, 1)), target)
    assert report.valid_input
    assert details['checked_indices'] == (0, 1, 2)
    assert details['risk_scope'] == 'execution_lease_prefix_discrete_samples'
    assert not details['entire_minco_curve_robust']
    model.p6_policy = replace(model.p6_policy, risk_horizon_seconds=0.)
    _, details = assess_risk(model, prediction, query, p, np.tile(np.eye(3), (5, 1, 1)), target)
    assert details['checked_indices'] == tuple(range(5))


@pytest.mark.parametrize('horizon', (.451, .2))
def test_execution_risk_horizon_must_cover_the_original_nonrenewable_lease(horizon):
    from uav_control.guidance.p6_follow import P6Policy
    with pytest.raises(ValueError, match='INVALID_P6_POLICY'):
        P6Policy(risk_horizon_seconds=horizon).validate()


def test_missing_lease_endpoint_cannot_be_reported_as_passed():
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.p6_follow import assess_risk
    model = FollowMpcSeed()
    model.p6_policy = calibrated_policy(risk_horizon_seconds=.45)
    prediction = SimpleNamespace(model='IMM_CV_CTRV_CTRA', prediction_times=(0., 1.6),
                                 target_covariances=(np.eye(3)*.01, np.eye(3)*.01),
                                 model_positions=np.tile((8., 0., 0.), (3, 2, 1)).tolist(),
                                 model_covariances=np.tile(np.eye(3)*.01, (3, 2, 1, 1)).tolist(),
                                 model_probabilities=(.5, .3, .2))
    p, target = np.tile((0., 0., -5.), (3, 1)), np.tile((8., 0., 0.), (3, 1))
    report, details = assess_risk(
        model, prediction, (0., .4, 1.6), p, np.tile(np.eye(3), (3, 1, 1)), target)
    assert report is None and not details['robust_ok']
    assert details['reason'] == 'P6_RISK_ENDPOINT_NOT_SAMPLED'

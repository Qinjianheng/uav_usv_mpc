"""Preserve the fixed 500ms stress case and distinguish actuator tau from transport delay."""
import numpy as np
import pytest

from test_p761_calibration import ROOT, script
from uav_control.common.p76_contract import load_contract


def contract():
    return load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')


def test_full_attitude_rate_characteristic_exposes_missing_horizontal_lag_boundary():
    analysis = script('p765_e2_analysis')
    result = analysis.linear_stability(contract())
    assert result['strict_rate_time_constant_bound_seconds'] == pytest.approx(.4375)
    assert result['characteristic_coefficients_at_500ms'] == pytest.approx([.5, 1., 4., 8., 4.])
    assert max(np.real(np.roots([.5, 1., 4., 8., 4.]))) > 0
    assert not result['physical_qualification']


def test_fixed_nonlinear_response_matches_immutable_p764_and_preserves_failure():
    import json
    analysis = script('p765_e2_analysis')
    saved = json.loads((ROOT/'data/baselines/20261021_p764_validation_01/'
                       'e2_control_replay.json').read_text())
    expected = next(r for r in saved['response_cases'] if r['lag_seconds'] == .5)
    actual = analysis.response_case(contract(), .5)
    assert actual['final_error'] == pytest.approx(expected['final_error'], abs=1e-10)
    assert actual['maximum_speed'] == pytest.approx(expected['maximum_speed'], abs=1e-10)
    assert not actual['passed']
    assert actual['output_bounds_passed']
    assert actual['lag_model'] == 'FIRST_ORDER_TIME_CONSTANT_NOT_TRANSPORT_DELAY'
    assert actual['diagnostics']['jerk_limited_samples'] > 0
    json.dumps(actual, allow_nan=False)


def test_offline_candidates_are_finite_and_do_not_change_gains_or_claim_minco_admission():
    analysis = script('p765_e2_analysis')
    assert analysis.CANDIDATES == ('baseline', 'reference_governor_10s', 'cv_prediction_120ms')
    result = analysis.response_case(contract(), .5, candidate='reference_governor_10s')
    assert result['settings'] == contract()['bodyrate_follow']
    assert not result['minco_feasibility_proved']
    assert not result['flight_qualified']
    with pytest.raises(ValueError, match='PREDECLARED'):
        analysis.response_case(contract(), .5, candidate='gain_search')

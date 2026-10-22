"""Offline identification must not turn transient samples into physical qualification."""
import numpy as np
import pytest

from test_p761_calibration import ROOT, script


def test_short_transient_fit_recovers_synthetic_motor_tau_without_qualifying_it():
    analysis = script('p765_dynamics_analysis')
    t = np.arange(0., .22, .02)
    force = (np.sqrt(40.)+(np.sqrt(20.)-np.sqrt(40.))*np.exp(-t/.035))**2
    fit = analysis.fit_motor_response(t, force, 20., 65.)
    assert fit['speed_time_constant_seconds'] == pytest.approx(.035, abs=1e-5)
    assert fit['steady_force_newtons'] == pytest.approx(40., abs=1e-4)
    assert fit['physical_error_bound'] is None
    assert not fit['qualified']


def test_sparse_or_constant_response_cannot_claim_identifiable_motor_dynamics():
    analysis = script('p765_dynamics_analysis')
    fit = analysis.fit_motor_response([0., .02], [20., 20.], 20., 65.)
    assert fit['status'] == 'INSUFFICIENT_EXCITATION_OR_SAMPLES'
    assert not fit['qualified']


def test_five_protocols_keep_full_force_bound_and_do_not_invent_executed_samples():
    analysis = script('p765_dynamics_analysis')
    report = analysis.analyze_session(ROOT/'data/experiments/20261021_p761_01/session_A1_03')
    protocols = report['protocol_comparison']
    assert len(protocols) == 5
    assert report['decision']['A1'] == 'NO-GO'
    assert report['decision']['E0'] == 'BLOCKED'
    assert report['per_level_upper_bounds']['proved'] is False
    assert all(p['force_upper_bound_newtons'] == report['static_model']['maximum_thrust']
               for p in protocols)
    assert protocols[0]['actual_new_protocol_force_samples'] > 0
    assert all(p['actual_new_protocol_force_samples'] == 0 for p in protocols[1:])
    assert not any(p['original_A1_equivalent'] for p in protocols)
    assert report['mode_exit']['scope'] == 'BODY_RATE_TO_VELOCITY_WITHDRAWAL_ONLY'
    assert report['mode_exit']['executed_collective_withdrawal_seconds'] == pytest.approx(
        .240118, abs=1e-6)
    assert report['motor_ulog']['measured_rotor_rpm_present'] is False
    observed = report['local_data']['phase_samples']
    assert any(p['executed_collective_matches_command_seconds'] < .05 for p in observed
               if p['executed_collective_matches_command_seconds'] is not None)
    assert all(p['same_mode_physical_response_qualified'] is False for p in observed)

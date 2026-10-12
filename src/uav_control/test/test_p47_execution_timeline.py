"""Guard causal attribution and clock joins in the offline P4.7 ownership audit."""
import importlib.util
from pathlib import Path

import pytest


def audit():
    path = Path(__file__).resolve().parents[3]/'scripts/p47_execution_timeline.py'
    assert path.exists(), 'P4.7 causal timeline auditor is not implemented'
    spec = importlib.util.spec_from_file_location('p47_execution_timeline', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def evidence():
    """One independently specified 450 ms lease, crossing inside one callback."""
    return [
        dict(kind='proposal', stamp=9.60, plan=7, parent=6, prediction=4,
             start=9.65, end=10.85, source=9.5, input_until=9.625),
        dict(kind='follow_ack', stamp=9.61, plan=7, state='ACCEPTED', reasons=[]),
        dict(kind='follow_ack', stamp=9.66, plan=7, state='ACTIVE', reasons=[]),
        dict(kind='uav', stamp=10.02, p=[1, 0, -4.9], v=[2, 0, 0], a=[3, 0, 0]),
        dict(kind='uav', stamp=10.08, p=[1.1, 0, -4.8], v=[2.1, 0, 0], a=[3.1, 0, 0]),
        dict(kind='uav', stamp=10.095, p=[99, 0, -3], v=[9, 0, 0], a=[8, 0, 0]),
        dict(kind='diagnostic', stamp=10.05, plan=7, status='MINCO_FOLLOW'),
        dict(kind='execution_debug', stamp=10.053, plan=7, prediction=5,
             state_stamp=10.02, lease_deadline=10.1, active_start=9.65,
             prefix_checks={'7': {'reason': 'VALID'}},
             reference=[1.2, 0, -5, 2.2, 0, 0, 3.2, 0, 0, 0, 0]),
        dict(kind='reference', stamp=10.0501, p=[float('nan')]*3,
             v=[2.3, 0, 0], a=[float('nan')]*3),
        dict(kind='diagnostic', stamp=10.1, plan=0, status='FOLLOW'),
        dict(kind='execution_debug', stamp=10.102, plan=0, prediction=6,
             state_stamp=10.08, prefix_checks={}),
        dict(kind='reference', stamp=10.1001, p=[float('nan')]*3,
             v=[2.4, 0, 0], a=[float('nan')]*3),
        dict(kind='follow_ack', stamp=10.102, plan=7, state='EXPIRED', reasons=[]),
        dict(kind='diagnostic', stamp=10.15, plan=0, status='FOLLOW'),
    ]


def test_expired_ack_is_direct_cause_even_when_prediction_changed():
    result = audit().reconstruct(evidence())
    switch = result['switches'][0]
    assert switch['direct_cause'] == 'NOMINAL_LEASE_EXPIRED'
    assert switch['cause_evidence'] == 'EXPIRED_ACK'
    assert switch['prediction_changed_since_previous_callback'] is True
    assert switch['latest_prefix_before']['reason'] == 'VALID'
    assert switch['elapsed_since_last_active_seconds'] == pytest.approx(.442)


def test_pending_start_missed_does_not_hide_reconstructed_old_lease_expiry():
    rows = [r for r in evidence() if r.get('state') != 'EXPIRED']
    rows += [dict(kind='proposal', stamp=9.9, plan=8, parent=7, prediction=5,
                  start=10.04, end=11.24, input_until=9.98),
             dict(kind='follow_ack', stamp=9.91, plan=8, state='ACCEPTED', reasons=[]),
             dict(kind='follow_ack', stamp=10.102, plan=8, state='REJECTED',
                  reasons=['START_MISSED'])]
    switch = audit().reconstruct(rows)['switches'][0]
    assert switch['direct_cause'] == 'NOMINAL_LEASE_EXPIRED'
    assert switch['cause_evidence'] == 'LEASE_CROSSING_AND_PENDING_START_MISSED'
    assert switch['upstream_update_state'] == 'ACCEPTED_REPLACEMENT_START_MISSED'
    assert switch['replacement_candidates'][0]['timely_start_before_old_deadline'] is True


def test_parent_rejection_is_upstream_not_direct_active_revocation():
    rows = evidence()+[
        dict(kind='proposal', stamp=9.9, plan=8, parent=6, prediction=5,
             start=10.04, end=11.24, input_until=9.98),
        dict(kind='follow_ack', stamp=9.91, plan=8, state='REJECTED',
             reasons=['PARENT_REFERENCE_INVALID']),
    ]
    switch = audit().reconstruct(rows)['switches'][0]
    assert switch['direct_cause'] == 'NOMINAL_LEASE_EXPIRED'
    assert switch['upstream_update_state'] == 'REPLACEMENT_REJECTED'
    assert switch['replacement_rejection_reasons'] == ['PARENT_REFERENCE_INVALID']


def test_actual_pva_joins_exact_navigation_sample_and_missing_reference_stays_unknown():
    switch = audit().reconstruct(evidence())['switches'][0]
    assert switch['after']['actual_pva']['p'] == [1.1, 0, -4.8]
    assert switch['after']['actual_pva']['a'] == [3.1, 0, 0]
    assert switch['after']['reference_pva'] is None
    assert (switch['after']['reference_pva_status'] ==
            'ORIGINAL_FOLLOW_GEOMETRIC_REFERENCE_NOT_LOGGED')
    assert switch['after']['emitted_setpoint']['p'] == [None]*3
    assert switch['after']['emitted_setpoint']['v'] == [2.4, 0, 0]
    assert switch['after']['configured_height_error_m'] == pytest.approx(.2)
    assert switch['command_velocity_step_mps'] == pytest.approx(.1)


def test_missing_ack_and_missing_prefix_do_not_prove_divergence_or_lease_expiry():
    rows = [r for r in evidence() if r['kind'] != 'follow_ack']
    for row in rows:
        if row['kind'] == 'execution_debug':
            row['lease_deadline'] = 11.
    switch = audit().reconstruct(rows)['switches'][0]
    assert switch['direct_cause'] == 'UNKNOWN'
    assert switch['measured_divergence_trigger'] == 'NOT_OBSERVED'


def test_prefix_revocation_is_distinct_from_prediction_change():
    rows = [r for r in evidence() if r.get('state') != 'EXPIRED']
    rows += [dict(kind='follow_ack', stamp=10.102, plan=7, state='REVOKED',
                  reasons=['LATEST_PREFIX_INVALID'])]
    switch = audit().reconstruct(rows)['switches'][0]
    assert switch['direct_cause'] == 'LATEST_PREFIX_INVALID'


def test_cadence_uses_producer_epochs_and_retains_long_intervals():
    rows = [dict(kind='proposal', stamp=t, receipt=t+10, plan=i,
                 start=t+.10, end=t+1, input_until=t+.08)
            for i, t in enumerate((1., 1.2, 2.))]
    cadence = audit().reconstruct(rows)['cadence']['proposal']
    assert cadence['interval_seconds']['max'] == pytest.approx(.8)
    assert cadence['intervals_ge_lease'] == 1
    assert cadence['published_to_start_seconds']['median'] == pytest.approx(.1)


def test_unaccepted_start_missed_is_not_evidence_of_overwritten_old_expiry():
    rows = [r for r in evidence() if r.get('state') != 'EXPIRED']
    rows += [dict(kind='follow_ack', stamp=10.102, plan=8, state='REJECTED',
                  reasons=['START_MISSED'])]
    switch = audit().reconstruct(rows)['switches'][0]
    assert switch['direct_cause'] == 'UNKNOWN'


def test_plan_history_keeps_parent_lease_ack_and_tracker_used_prediction_versions():
    plan = audit().reconstruct(evidence())['plans'][0]
    assert plan['plan'] == 7
    assert plan['parent'] == 6
    assert plan['nominal_lease_deadline'] == pytest.approx(10.1)
    assert [r['state'] for r in plan['ack_history']] == ['ACCEPTED', 'ACTIVE', 'EXPIRED']
    assert plan['final_owner_diagnostic_epochs'] == [10.05]
    assert plan['prefix_validation_history'][0]['prediction'] == 5

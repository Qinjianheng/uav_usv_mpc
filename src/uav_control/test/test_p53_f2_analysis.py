"""Causal analysis must distinguish failed production, wire delivery and activation."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
from p53_f2_analysis import attempt_blocker, interval_distribution  # noqa: E402


def test_successful_solver_can_first_fail_at_publication_or_late_activation():
    event = dict(output=dict(valid=True, solver_status='SAMPLED_FEASIBLE'))
    assert attempt_blocker(event, dict(published=False, reason='STALE_INPUT'), []) == 'STALE_INPUT'
    acks = [dict(state='ACCEPTED', reasons=[]), dict(state='REJECTED', reasons=['START_MISSED'])]
    assert attempt_blocker(event, dict(published=True), acks) == 'START_MISSED'


def test_missing_delivery_is_unknown_and_duplicate_ack_does_not_imply_activation():
    event = dict(output=dict(valid=True))
    assert attempt_blocker(event, dict(published=True), []) == 'NO_RECORDED_RECEIVER_ACK'
    acks = [dict(state='ACCEPTED', reasons=[])]*2
    assert attempt_blocker(event, dict(published=True), acks) == 'NO_RECORDED_ACTIVE'
    assert attempt_blocker(None, None, []) == 'NO_RECORDED_REQUEST'


def test_solver_failure_precedes_downstream_absence():
    event = dict(output=dict(valid=False, solver_status='DEADLINE_EXCEEDED'))
    assert attempt_blocker(event, None, []) == 'DEADLINE_EXCEEDED'
    failed = dict(candidate_output={'exception_type': 'ValueError'},
                  output=dict(valid=False, solver_status='SOLVER_EXCEPTION'))
    assert attempt_blocker(failed, None, []) == 'SOLVER_EXCEPTION'


def test_intervals_are_unique_physical_epochs_with_explicit_sample_count():
    d = interval_distribution([100., 100.2, 100.2, 100.4])
    assert d['count'] == 2
    assert abs(d['p95']-.2) < 1e-10
    assert interval_distribution([]) is None


def test_late_research_marker_never_overwrites_real_wire_and_active_outcome():
    event = dict(output=dict(valid=False, solver_status='STALE_INPUT'),
                 candidate_output=dict(valid=True, solver_status='SAMPLED_FEASIBLE'))
    final = dict(published=True)
    assert attempt_blocker(event, final, [dict(state='ACTIVE', reasons=[])]) == 'ACTIVE'
    assert attempt_blocker(event, final, [dict(state='ACCEPTED', reasons=[])]) == (
        'NO_RECORDED_ACTIVE')
    assert attempt_blocker(event, dict(published=False, reason='INPUT_EXPIRED'), []) == (
        'INPUT_EXPIRED')

"""Fail-closed F2 evidence audit; synthetic rows are never flight qualification."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))


def rows():
    from test_follow_sitl_execution import candidate
    c = candidate()
    proposal = dict(kind='proposal', receipt=100.01, stamp=100.01, plan=1, parent=0,
                    start=c.start, end=c.end, durations=c.durations,
                    xyz=[v for row in c.xyz for v in row],
                    yaw=[v for row in c.yaw for v in row],
                    source=100., observation=100., navigation=100., attitude=100.,
                    input_until=100.125)
    qualified = dict(
        plan_id=1, stamp=100.01, stable_since=95., recovery_until=90.,
        valid_probe_count=3, last_probe=100., entry_attempt=1, loss=None)
    result = [proposal, dict(
        kind='follow_ack', receipt=100.04, stamp=100.04, plan=1,
        state='ACTIVE', owner='MINCO_FOLLOW', active=1, reasons=[])]
    for i in range(5):
        t = 100.04+i*.05
        result += [dict(kind='diagnostic', receipt=t, stamp=t, plan=1, status='MINCO_FOLLOW'),
                   dict(kind='execution_debug', receipt=t, stamp=t, plan=1,
                        reference=list(c.sample(t)), active_start=c.start,
                        lease_deadline=c.start+.45, entry_evidence=qualified,
                        prefix_checks={1: dict(reason='VALID', visible=True)},
                        execution_context=dict(following=True, visible=True,
                                               offboard=True, armed=True))]
    return result


def test_complete_reference_and_qualification_evidence_can_pass():
    from p51_execution_audit import audit_rows
    r = audit_rows(rows(), 100.04, 'f2')
    assert r['passed'] and all(r['checks'].values())
    assert r['reference_height']['maximum_error_m'] == pytest.approx(0.)


@pytest.mark.parametrize('fault,check', [
    ('height', 'reference_height'), ('expired', 'execution_gates'),
    ('entry', 'entry_recovery'), ('prefix', 'execution_gates'),
    ('missing', 'evidence_coverage'), ('ack', 'execution_gates'),
    ('incomplete_entry', 'entry_recovery'),
])
def test_bad_or_missing_actual_evidence_cannot_qualify_f3(fault, check):
    from p51_execution_audit import audit_rows
    r = deepcopy(rows())
    debug = [row for row in r if row['kind'] == 'execution_debug']
    if fault == 'height':
        debug[-1]['reference'][2] = -3.9
    elif fault == 'expired':
        debug[-1]['lease_deadline'] = debug[-1]['stamp']
    elif fault == 'entry':
        debug[0]['entry_evidence']['stable_since'] = 96.
    elif fault == 'incomplete_entry':
        debug[0]['entry_evidence']['stable_since'] = None
    elif fault == 'prefix':
        debug[-1]['prefix_checks'] = {1: dict(reason='FOV_OR_SEA', visible=True)}
    elif fault == 'missing':
        r = [row for row in r if row['kind'] != 'execution_debug']
    else:
        r = [row for row in r if row['kind'] != 'follow_ack']
    report = audit_rows(r, 100.04, 'f2')
    assert not report['passed'] and not report['checks'][check]


def test_safe_fallback_is_classified_and_does_not_by_itself_fail_the_audit():
    from p51_execution_audit import audit_rows
    r = rows()
    r += [dict(kind='follow_ack', receipt=100.29, stamp=100.29, plan=1, state='REVOKED',
               owner='ORIGINAL_FOLLOW', active=0, reasons=['LATEST_PREFIX_INVALID']),
          dict(kind='diagnostic', receipt=100.29, stamp=100.29, plan=0, status='FOLLOW')]
    report = audit_rows(r, 100.04, 'f2')
    assert report['passed']
    assert report['fallbacks'][0]['reason'] == 'LATEST_PREFIX_INVALID'
    assert report['lease_expiry_count'] == 0


def test_rapid_reentry_after_a_recorded_loss_is_rejected_even_with_high_control_fraction():
    from p51_execution_audit import audit_rows
    r = rows()
    r += [dict(kind='follow_ack', receipt=100.29, stamp=100.29, plan=1, state='REVOKED',
               owner='ORIGINAL_FOLLOW', active=0, reasons=['LATEST_PREFIX_INVALID']),
          dict(kind='diagnostic', receipt=100.29, stamp=100.29, plan=0, status='FOLLOW')]
    proposal = deepcopy(r[0])
    proposal.update(plan=2, stamp=100.3, start=100.34, end=101.94, source=100.3,
                    observation=100.3, navigation=100.3, attitude=100.3, input_until=100.425)
    r += [proposal, dict(kind='follow_ack', receipt=100.34, stamp=100.34, plan=2,
                         state='ACTIVE', owner='MINCO_FOLLOW', active=2, reasons=[])]
    debug = deepcopy(next(row for row in r if row['kind'] == 'execution_debug'))
    debug.update(receipt=100.34, stamp=100.34, plan=2, active_start=100.34, lease_deadline=100.79,
                 prefix_checks={2: dict(reason='VALID', visible=True)})
    debug['entry_evidence'].update(plan_id=2, stamp=100.3, stable_since=95.,
                                   last_probe=100.3, loss=dict(plan_id=1, stamp=100.29))
    r += [debug, dict(kind='diagnostic', receipt=100.34, stamp=100.34, plan=2,
                      status='MINCO_FOLLOW')]
    report = audit_rows(r, 100.04, 'f2')
    assert not report['passed']
    assert not report['checks']['switching'] and not report['checks']['entry_recovery']

#!/usr/bin/env python3
"""Read-only P5.1 execution evidence checks; absent evidence never permits F3."""
from collections import Counter
import math

import numpy as np

from p45_execution_analysis import sample
from uav_control.guidance.follow_admission import AdmissionConfig

CHECKS = ('reference_height', 'entry_recovery', 'execution_gates',
          'evidence_coverage', 'switching', 'handovers')


def audit_rows(rows, first_minco, scene, config=None):
    """Correlate actual diagnostics, receiver debug, wire curves and ACKs by plan/time."""
    checks = dict.fromkeys(CHECKS, True)
    errors, fallbacks, heights, vertical_speeds, steps = [], [], [], [], []
    config = AdmissionConfig() if config is None else config

    def finite(value):
        return isinstance(value, (int, float)) and math.isfinite(value)

    def fail(check, reason, row=None):
        checks[check] = False
        row = row or {}
        errors.append(dict(check=check, reason=reason, stamp=row.get('stamp'),
                           plan=row.get('plan')))

    end = first_minco+(25. if scene == 'f2' else 60.) if first_minco is not None else -math.inf
    selected = [r for r in rows if first_minco is not None
                and first_minco-.15 <= r['receipt'] <= end]
    diag = sorted((r for r in selected if r['kind'] == 'diagnostic'), key=lambda r: r['stamp'])
    debug = [r for r in selected if r['kind'] == 'execution_debug' and r.get('plan')]
    proposals = {r['plan']: r for r in rows if r['kind'] == 'proposal'}
    acks = [r for r in rows if r['kind'] == 'follow_ack']
    active = {r['plan']: r for r in acks if r['state'] == 'ACTIVE'}
    minco = [r for r in diag if r['status'] == 'MINCO_FOLLOW']
    matched = sum(any(r['plan'] == d['plan'] and abs(r['stamp']-d['stamp']) <= .0251
                      for r in debug) for d in minco)
    coverage = matched/len(minco) if minco else 0.
    if not debug or coverage < .95:
        fail('evidence_coverage', 'MISSING_CONTROL_CYCLE_REFERENCE_OR_OWNER')
    seen = set()
    for r in debug:
        plan, t = r['plan'], r['stamp']
        c, ack = proposals.get(plan), active.get(plan)
        reference = r.get('reference', [])
        if len(reference) != 11 or not all(math.isfinite(v) for v in reference):
            fail('reference_height', 'INVALID_REFERENCE', r)
            continue
        heights.append(abs(reference[2]+5.))
        vertical_speeds.append(reference[5])
        if heights[-1] >= 1.:
            fail('reference_height', 'METER_LEVEL_REFERENCE_HEIGHT_ERROR', r)
        context = r.get('execution_context', {})
        prefix = r.get('prefix_checks', {})
        prefix = prefix.get(str(plan), prefix.get(plan, {}))
        legacy_context = context.get('schema_version', 1) == 1
        if not (c and ack and ack.get('owner') == 'MINCO_FOLLOW' and ack.get('active') == plan
                and c['start'] <= ack['stamp'] <= c['start']+.05+1e-7
                and ack['stamp'] <= t < min(c['end'], c['start']+.45)
                and r.get('lease_deadline') == min(c['end'], c['start']+.45)
                and all(context.get(k) for k in ('following', 'visible', 'offboard', 'armed'))
                and all(context.get(k, legacy_context) is True for k in (
                    'navigation_fresh', 'target_fresh', 'prediction_fresh', 'observation_fresh'))
                and context.get('recovering', False if legacy_context else None) is False
                and prefix.get('reason') == 'VALID' and prefix.get('visible')):
            fail('execution_gates', 'MISSING_OR_ILLEGAL_ACTIVE_CONTEXT_PREFIX_LEASE', r)
        if plan in seen:
            continue
        seen.add(plan)
        if not c:
            fail('execution_gates', 'MISSING_WIRE_CURVE', r)
            continue
        raw = [c.get(k, -math.inf) for k in ('source', 'observation', 'navigation', 'attitude')]
        if not (all(finite(epoch) and 0 <= c['stamp']-epoch <= .125 for epoch in raw)
                and c['stamp'] < c['input_until'] and c['stamp'] < c['start']):
            fail('execution_gates', 'STALE_OR_FUTURE_PROPOSAL_INPUT', r)
        if c['parent']:
            parent = proposals.get(c['parent'])
            if not parent or not parent['start'] <= c['start'] < min(
                    parent['end'], parent['start']+.45):
                fail('handovers', 'PARENT_EXPIRED_OR_MISSING', r)
            else:
                delta = sample(c, c['start'])-sample(parent, c['start'])
                delta[9] = math.atan2(math.sin(delta[9]), math.cos(delta[9]))
                steps.append(float(np.max(np.abs(delta))))
                if steps[-1] > 1e-6:
                    fail('handovers', 'PVA_YAW_RATE_DISCONTINUITY', r)
        else:
            q = r.get('entry_evidence', {})
            qt = q.get('stamp', -math.inf)
            loss = q.get('loss')
            numbers = ('stamp', 'stable_since', 'recovery_until', 'last_probe',
                       'valid_probe_count', 'entry_attempt')
            if not (all(finite(q.get(k)) for k in numbers)
                    and q.get('plan_id') == plan and math.isfinite(qt)
                    and qt <= c['start']
                    and qt-q.get('stable_since', math.inf) >= config.stable_seconds-1e-7
                    and qt >= q.get('recovery_until', math.inf)
                    and q.get('valid_probe_count', 0) >= config.required_probes
                    and 0 <= qt-q.get('last_probe', -math.inf) <= config.probe_maximum_gap
                    and 0 < q.get('entry_attempt', 0) <= config.maximum_entries):
                fail('entry_recovery', 'MISSING_OR_INVALID_ENTRY_QUALIFICATION', r)
            if loss and (not finite(loss.get('stamp')) or not finite(q.get('stable_since'))
                         or qt < loss['stamp']+config.cooldown_seconds+config.stable_seconds-1e-7
                         or q['stable_since'] < loss['stamp']+config.cooldown_seconds-1e-7):
                fail('entry_recovery', 'RECOVERY_COOLDOWN_OR_STABILITY_BYPASSED', r)
    for a, b in zip(diag, diag[1:]):
        if a['status'] != 'MINCO_FOLLOW' or b['status'] == 'MINCO_FOLLOW':
            continue
        terminal = [r for r in acks if r['plan'] == a['plan']
                    and r['state'] in ('REVOKED', 'EXPIRED')
                    and a['stamp']-.05 <= r['stamp'] <= b['stamp']+.05]
        reason = (','.join(terminal[-1].get('reasons', [])) or 'LEASE_EXPIRED_NO_SUCCESSOR'
                  if terminal else 'MISSING_LOSS_ACK')
        successors = [p for p in proposals.values() if p['parent'] == a['plan']]
        fallbacks.append(dict(plan=a['plan'], stamp=b['stamp'], reason=reason,
                              terminal_ack=terminal[-1] if terminal else None,
                              successor_plans=[p['plan'] for p in successors]))
        if not terminal:
            fail('switching', 'UNEXPLAINED_OWNER_LOSS', b)
        # Every renewed initial entry must carry this loss, even if its own numbers look valid.
        renewed = next((r for r in debug if r['stamp'] > b['stamp']
                        and not proposals.get(r['plan'], {}).get('parent', 0)), None)
        if renewed:
            loss = renewed.get('entry_evidence', {}).get('loss') or {}
            if loss.get('plan_id') != a['plan'] or renewed['stamp']-b['stamp'] < 7.-.1:
                fail('switching', 'RAPID_OR_UNQUALIFIED_REENTRY', renewed)
    if not heights:
        fail('reference_height', 'NO_ACTUAL_MINCO_REFERENCE_SAMPLES')
    return dict(passed=all(checks.values()), checks=checks, violations=errors,
                actual_minco_diagnostics=len(minco), reference_samples=len(debug),
                reference_coverage=coverage, active_plans=len(seen), fallbacks=fallbacks,
                fallback_reasons=dict(Counter(r['reason'] for r in fallbacks)),
                lease_expiry_count=sum(r['reason'] == 'LEASE_EXPIRED_NO_SUCCESSOR'
                                       for r in fallbacks),
                reference_height=dict(maximum_error_m=max(heights, default=None),
                                      rmse_m=float(np.sqrt(np.mean(np.square(heights))))
                                      if heights else None),
                reference_vertical_velocity_mps=vertical_speeds,
                handover_max_abs_pva_yaw_rate=max(steps, default=None),
                thresholds=dict(reference_height_max_error_m=1., diagnostic_coverage=.95,
                                lease_seconds=.45, activation_window_seconds=.05,
                                raw_input_age_seconds=.125, admission=vars(config)),
                note='Actual logged evidence only; safe fallback is classified, not concealed. '
                'Zero avoidable expiry remains a research target, not a new absolute threshold.')


def audit(output, rows, first_minco, scene):
    """Write only into the new actual run; preserve every recorded event."""
    import json
    from p5_live_health import geometry_config
    _, _, config = geometry_config(output)
    report = audit_rows(rows, first_minco, scene, config)
    (output/'p51_execution_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    return report

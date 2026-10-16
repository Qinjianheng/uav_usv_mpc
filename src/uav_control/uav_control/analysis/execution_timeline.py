"""
Pure original-epoch ownership attribution; no qualification or output writing.

Only causal reconstruction helpers migrated from 55c1003; no historical CLI.
"""
from collections import Counter, defaultdict
import math

import numpy as np


def clean(value):
    """Encode absent/NaN signals as JSON null without fabricating finite samples."""
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def stats(values):
    """Keep all finite intervals, including lease-breaking gaps."""
    values = [float(v) for v in values if v is not None and math.isfinite(v)]
    return dict(count=len(values), min=min(values) if values else None,
                median=float(np.median(values)) if values else None,
                p95=float(np.quantile(values, .95)) if values else None,
                max=max(values) if values else None)


def vector_distance(a, b):
    """Unknown vectors do not turn into zero errors."""
    if a is None or b is None or any(v is None for v in [*a, *b]):
        return None
    return float(np.linalg.norm(np.subtract(a, b)))


def interval_member(rows, lo, hi):
    """Join the uniquely logged event inside one original control callback interval."""
    matches = [r for r in rows if lo <= r['stamp'] < hi]
    return matches[0] if len(matches) == 1 else None


def cadence(rows, lease):
    """Measure producer stamp intervals, not monitor receipt time or count-based ownership."""
    stamps = sorted(r['stamp'] for r in rows)
    intervals = np.diff(stamps).tolist()
    return dict(count=len(rows), interval_seconds=stats(intervals),
                intervals_ge_lease=sum(dt >= lease for dt in intervals),
                published_to_start_seconds=stats(
                    [r['start']-r['stamp'] for r in rows if 'start' in r]),
                input_ttl_at_publication_seconds=stats(
                    [r['input_until']-r['stamp'] for r in rows if 'input_until' in r]))


def callback_state(diag, debug, emitted, navigation, height_z):
    """Actual PVA uses debug.state_stamp; final setpoint is a separate signal."""
    state_stamp = debug.get('state_stamp') if debug else None
    actual = navigation.get(state_stamp)
    pva = ({k: actual.get(k) for k in ('p', 'v', 'a')} if actual else None)
    if not actual and debug and 'actual_position' in debug:
        pva = dict(p=debug['actual_position'], v=debug['actual_velocity'], a=None)
    reference = debug.get('reference') if debug else None
    rpva = (dict(p=reference[:3], v=reference[3:6], a=reference[6:9])
            if reference is not None else None)
    reference_status = 'LOGGED_MINCO_REFERENCE' if rpva else 'UNKNOWN'
    if diag['status'] == 'FOLLOW':
        reference_status = 'ORIGINAL_FOLLOW_GEOMETRIC_REFERENCE_NOT_LOGGED'
        rpva = None
    return clean(dict(
        diagnostic_epoch=diag['stamp'], diagnostic_owner=diag['status'],
        diagnostic_plan=diag.get('plan'),
        debug_epoch=debug['stamp'] if debug else None,
        navigation_epoch=state_stamp, navigation_native=actual.get('native') if actual else None,
        actual_pva=pva, actual_pva_status=(
            'EXACT_RECORDED_NAVIGATION_SAMPLE' if actual else
            'DEBUG_PV_ONLY' if pva else 'UNKNOWN'),
        reference_pva=rpva, reference_pva_status=reference_status,
        reference_epoch=max(state_stamp, debug['active_start'])
        if rpva and state_stamp is not None and debug.get('active_start') else None,
        configured_height_error_m=pva['p'][2]-height_z if pva and pva.get('p') else None,
        reference_vertical_error_m=pva['p'][2]-rpva['p'][2] if pva and rpva else None,
        emitted_epoch=emitted['stamp'] if emitted else None,
        emitted_setpoint={k: emitted.get(k) for k in ('p', 'v', 'a')} if emitted else None,
        latest_prediction=debug.get('prediction') if debug else None,
        prefix_checks=debug.get('prefix_checks') if debug else None,
        lease_deadline=debug.get('lease_deadline') if debug else None))


def direct_cause(acks, old_plan, tick, deadline, before_tick, all_acks):
    """ACK reasons are triggers; reconstruct only the documented overwritten-ACK case."""
    terminating = [r for r in acks if r.get('plan') == old_plan
                   and r['state'] in ('EXPIRED', 'REVOKED')]
    if terminating:
        ack = terminating[-1]
        if ack['state'] == 'EXPIRED':
            return 'NOMINAL_LEASE_EXPIRED', 'EXPIRED_ACK', ack
        reasons = ack.get('reasons', [])
        for reason in ('LATEST_PREFIX_INVALID', 'PREDICTION_CHANGED',
                       'PARENT_REFERENCE_INVALID'):
            if reason in reasons:
                return reason, 'REVOKED_ACK', ack
        return 'CONTEXT_REVOKED', 'REVOKED_ACK', ack
    missed = [r for r in acks if r['state'] == 'REJECTED' and r['stamp'] == tick
              and 'START_MISSED' in r.get('reasons', [])
              and any(a['plan'] == r['plan'] and a['state'] == 'ACCEPTED'
                      and a['stamp'] < r['stamp'] for a in all_acks)]
    if (tick is not None and deadline is not None and before_tick is not None
            and before_tick < deadline <= tick and missed):
        return ('NOMINAL_LEASE_EXPIRED', 'LEASE_CROSSING_AND_PENDING_START_MISSED',
                missed[-1])
    return 'UNKNOWN', 'INSUFFICIENT_CAUSAL_EVIDENCE', None


def replacement_evidence(proposals, acks, old, active_epoch, tick, deadline):
    """Separate loss of renewal from the direct active-lease expiry trigger."""
    lo = old.get('start', active_epoch)
    candidates = []
    if lo is not None and tick is not None:
        for proposal in proposals:
            if proposal['plan'] == old.get('plan') or not lo <= proposal['stamp'] <= tick:
                continue
            history = [a for a in acks if a['plan'] == proposal['plan'] and a['stamp'] <= tick]
            candidates.append(dict(
                plan=proposal['plan'], parent=proposal.get('parent'),
                publication_epoch=proposal['stamp'], start=proposal.get('start'),
                deadline=deadline, input_until=proposal.get('input_until'),
                timely_start_before_old_deadline=(proposal['start'] < deadline)
                if deadline is not None and 'start' in proposal else None,
                input_ttl_at_publication_seconds=proposal['input_until']-proposal['stamp']
                if 'input_until' in proposal else None,
                ack_history=history))
    reasons = sorted({r for c in candidates for a in c['ack_history']
                      if a['state'] == 'REJECTED' for r in a.get('reasons', [])})
    states = {a['state'] for c in candidates for a in c['ack_history']}
    accepted_missed = any(
        any(a['state'] == 'ACCEPTED' for a in c['ack_history'])
        and any(a['state'] == 'REJECTED' and 'START_MISSED' in a.get('reasons', [])
                for a in c['ack_history']) for c in candidates)
    if accepted_missed:
        upstream = 'ACCEPTED_REPLACEMENT_START_MISSED'
    elif 'REJECTED' in states:
        upstream = 'REPLACEMENT_REJECTED'
    elif candidates:
        upstream = 'REPLACEMENT_PROPOSED_WITHOUT_TIMELY_ACTIVE'
    else:
        upstream = 'NO_REPLACEMENT_PROPOSAL_DURING_ACTIVE_LEASE'
    return candidates, reasons, upstream


def reconstruct(rows, lease_seconds=.45, height_z=-5.):
    """Reconstruct final ownership; all output times stay on their recorded epochs."""
    groups = defaultdict(list)
    for row in rows:
        groups[row['kind']].append(row)
    for group in groups.values():
        group.sort(key=lambda r: r.get('stamp', r.get('receipt', 0.)))
    diag = groups['diagnostic']
    proposals = {r['plan']: r for r in groups['proposal']}
    acks = groups['follow_ack']
    navigation = {r['stamp']: r for r in groups['uav']}
    callbacks, debug_rows = [], []
    seconds = Counter()
    for i, row in enumerate(diag):
        hi = diag[i+1]['stamp'] if i+1 < len(diag) else row['stamp']+.05
        debug = interval_member(groups['execution_debug'], row['stamp'], hi)
        emitted = interval_member(groups['reference'], row['stamp'], hi)
        callbacks.append(callback_state(row, debug, emitted, navigation, height_z))
        debug_rows.append(debug)
        if i+1 < len(diag) and row['status'] in ('FOLLOW', 'MINCO_FOLLOW'):
            dt = hi-row['stamp']
            seconds[row['status'] if dt <= .15 else 'UNKNOWN_GAP'] += dt
    switches = []
    for i, (a, b) in enumerate(zip(diag, diag[1:])):
        if (a['status'], b['status']) != ('MINCO_FOLLOW', 'FOLLOW'):
            continue
        before, after = callbacks[i:i+2]
        d_before, d_after = debug_rows[i:i+2]
        tick = after['debug_epoch']
        hi = diag[i+2]['stamp'] if i+2 < len(diag) else b['stamp']+.05
        relevant_acks = [r for r in acks if a['stamp'] < r['stamp'] < hi]
        old = proposals.get(a.get('plan'), dict(plan=a.get('plan')))
        deadline = before.get('lease_deadline')
        cause, basis, terminal_ack = direct_cause(
            relevant_acks, a.get('plan'), tick, deadline, before['debug_epoch'], acks)
        active = [r for r in acks if r['plan'] == a.get('plan')
                  and r['state'] == 'ACTIVE' and r['stamp'] <= a['stamp']]
        active_epoch = active[-1]['stamp'] if active else None
        candidates, reasons, upstream = replacement_evidence(
            groups['proposal'], acks, old, active_epoch, tick, deadline)
        latest_prefix = (d_before or {}).get('prefix_checks', {}).get(str(a.get('plan')))
        after_prefix = (d_after or {}).get('prefix_checks', {}).get(str(a.get('plan')))
        switches.append(clean(dict(
            index=len(switches)+1, control_callback_epoch=b['stamp'], receiver_tick_epoch=tick,
            old_plan=a.get('plan'), old_parent=old.get('parent'),
            old_prediction_id=old.get('prediction'), old_start=old.get('start'),
            old_lease_deadline=deadline, direct_cause=cause, cause_evidence=basis,
            direct_ack=terminal_ack, callback_acks=relevant_acks,
            latest_prefix_before=latest_prefix, latest_prefix_at_switch=after_prefix,
            prediction_changed_since_previous_callback=(
                before['latest_prediction'] != after['latest_prediction'])
            if None not in (before['latest_prediction'], after['latest_prediction']) else None,
            old_prediction_differs_from_latest=(
                old.get('prediction') != after['latest_prediction'])
            if old.get('prediction') is not None
            and after['latest_prediction'] is not None else None,
            measured_divergence_trigger='NOT_OBSERVED',
            last_active_epoch=active_epoch,
            elapsed_since_last_active_seconds=tick-active_epoch
            if tick is not None and active_epoch is not None else None,
            old_reference_age_seconds=tick-old['start']
            if tick is not None and 'start' in old else None,
            expiry_overrun_seconds=tick-deadline
            if tick is not None and deadline is not None else None,
            replacement_candidates=candidates, replacement_rejection_reasons=reasons,
            upstream_update_state=upstream, before=before, after=after,
            actual_reference_position_error_before_m=vector_distance(
                before['actual_pva']['p'], before['reference_pva']['p'])
            if before['actual_pva'] and before['reference_pva'] else None,
            command_velocity_step_mps=vector_distance(
                before['emitted_setpoint']['v'], after['emitted_setpoint']['v'])
            if before['emitted_setpoint'] and after['emitted_setpoint'] else None)))
    lifecycle = dict(ack_states=dict(Counter(r['state'] for r in acks)),
                     rejection_reasons=dict(Counter(reason for r in acks
                                                    if r['state'] == 'REJECTED'
                                                    for reason in r.get('reasons', []))))
    plans = []
    for plan, proposal in proposals.items():
        checks = [dict(stamp=r['stamp'], prediction=r.get('prediction'),
                       prefix=r['prefix_checks'][str(plan)])
                  for r in groups['execution_debug'] if str(plan) in r.get('prefix_checks', {})]
        plans.append(dict(
            plan=plan, parent=proposal.get('parent'), prediction_id=proposal.get('prediction'),
            proposal=proposal, ack_history=[r for r in acks if r['plan'] == plan],
            nominal_lease_deadline=min(proposal['end'], proposal['start']+lease_seconds),
            prefix_validation_history=checks,
            final_owner_diagnostic_epochs=[r['stamp'] for r in diag
                                           if r['status'] == 'MINCO_FOLLOW'
                                           and r.get('plan') == plan]))
    return dict(switches=switches, callbacks=callbacks, plans=plans, lifecycle=lifecycle,
                direct_causes=dict(Counter(s['direct_cause'] for s in switches)),
                cause_evidence=dict(Counter(s['cause_evidence'] for s in switches)),
                upstream_update_states=dict(Counter(s['upstream_update_state'] for s in switches)),
                ownership_seconds=dict(seconds),
                cadence=dict(proposal=cadence(groups['proposal'], lease_seconds),
                             accepted=cadence([r for r in acks if r['state'] == 'ACCEPTED'],
                                              lease_seconds),
                             active=cadence([r for r in acks if r['state'] == 'ACTIVE'],
                                            lease_seconds)))

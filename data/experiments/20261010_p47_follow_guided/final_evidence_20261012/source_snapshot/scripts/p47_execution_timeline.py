#!/usr/bin/env python3
"""Read-only P4.5 original-clock ownership and causal fallback reconstruction.

Writes exclusively to a separate output tree. Missing geometric references, ACKs,
and navigation samples remain unknown. A prediction update is not a revocation.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


RUNS = ('b_f2_1', 'b_f2_2', 'c_f2_1', 'c_f2_2')
CODE = ('src/uav_control/uav_control/control/trajectory_tracker_node.py',
        'src/uav_control/uav_control/guidance/follow_minco_execution.py',
        'src/uav_control/uav_control/guidance/follow_contract.py')


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
    """Producer stamp intervals, not monitor receipt time or count-based ownership."""
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


def load_jsonl(path):
    """Fail explicitly on malformed evidence and preserve original line numbers."""
    rows = []
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            row = json.loads(line)
            row['_source_line'] = number
            rows.append(row)
    return rows


def sha256(path):
    """Hash original inputs without rewriting them."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def ulog_check(root, rows, callbacks):
    """Cross-check final emitted velocity in PX4 on original native navigation pairs."""
    from pyulog import ULog
    nav = sorted([r for r in rows if r['kind'] == 'uav' and r.get('valid')],
                 key=lambda r: r['stamp'])
    native = np.asarray([r['native'] for r in nav])
    epochs = np.asarray([r['stamp'] for r in nav])
    if not len(native) or not np.all(np.diff(native) > 0):
        return dict(status='UNKNOWN', reason='NONMONOTONIC_OR_ABSENT_NAVIGATION_PAIRS')
    samples = []
    for path in sorted((root/'ulogs').glob('*.ulg')):
        ulog = ULog(str(path), message_name_filter_list=['trajectory_setpoint'])
        for dataset in ulog.data_list:
            data = dataset.data
            if dataset.name != 'trajectory_setpoint':
                continue
            for i, stamp in enumerate(data['timestamp']):
                if native[0] <= stamp <= native[-1]:
                    samples.append(dict(native_timestamp=int(stamp),
                                        mapped_epoch=float(np.interp(stamp, native, epochs)),
                                        v=[float(data[f'velocity[{k}]'][i]) for k in range(3)]))
    matched = 0
    for state in callbacks:
        command, epoch = state['emitted_setpoint'], state['emitted_epoch']
        matches = [r for r in samples if command and epoch is not None
                   and abs(r['mapped_epoch']-epoch) <= .125
                   and np.allclose(r['v'], command['v'], rtol=0., atol=1e-6, equal_nan=False)]
        match = min(matches, key=lambda r: abs(r['mapped_epoch']-epoch)) if matches else None
        state['px4_ulog_command'] = match
        matched += match is not None
    return dict(status='CHECKED', setpoints_inside_native_mapping=len(samples),
                callbacks_checked=len(callbacks), velocity_matches=matched,
                mapping='ULog native timestamp -> original navigation sample epoch pairs',
                join=('same finite velocity <=1e-6 m/s within 125 ms mapped epoch; '
                      'no extrapolation'),
                note=('Mapping is a clock cross-check, '
                      'not an exact PX4 receive-to-control-tick latency'))


def write_json(path, value):
    path.write_text(json.dumps(clean(value), indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def write_switch_csv(path, switches):
    """Nested cells retain the complete evidence and explicit nulls for missing signals."""
    fields = list(switches[0]) if switches else ['index']
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k: json.dumps(v, allow_nan=False) if isinstance(v, (dict, list)) else v
                         for k, v in row.items()} for row in switches)


def plot(path, report, first_follow):
    """Show final owner, per-callback reference error, velocity and height on source clock."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import ScalarFormatter
    states = [r for r in report['callbacks']
              if r['diagnostic_owner'] in ('FOLLOW', 'MINCO_FOLLOW')]
    fig, axs = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    stamps = [r['diagnostic_epoch'] for r in states]
    axs[0].step(stamps, [r['diagnostic_owner'] == 'MINCO_FOLLOW' for r in states], where='post')
    axs[0].set_yticks([0, 1], ['FOLLOW', 'MINCO'])
    active = [r for r in states if r['actual_pva'] and r['reference_pva']]
    axs[1].plot([r['diagnostic_epoch'] for r in active],
                [vector_distance(r['actual_pva']['p'], r['reference_pva']['p']) for r in active],
                '.', ms=2, label='MINCO reference-position error at native navigation epoch')
    axs[1].set_ylabel('Error (m)')
    for owner, color in (('MINCO_FOLLOW', 'tab:blue'), ('FOLLOW', 'tab:orange')):
        subset = [r for r in states if r['diagnostic_owner'] == owner and r['emitted_setpoint']]
        axs[2].plot([r['diagnostic_epoch'] for r in subset],
                    [np.linalg.norm(r['emitted_setpoint']['v'][:2]) for r in subset],
                    '.', ms=2, color=color, label=owner+' final emitted Vxy')
    axs[2].set_ylabel('Command (m/s)')
    axs[3].plot(stamps, [r['configured_height_error_m'] for r in states], label='z - (-5 m)')
    axs[3].set_ylabel('NED z error (m)')
    axs[3].set_xlabel('Original ROS epoch (s), first FOLLOW = '+format(first_follow, '.9f'))
    formatter = ScalarFormatter(useOffset=False)
    formatter.set_scientific(False)
    axs[3].xaxis.set_major_formatter(formatter)
    for ax in axs:
        for switch in report['switches']:
            ax.axvline(switch['control_callback_epoch'], color='red', alpha=.18, lw=.7)
        ax.grid(alpha=.2)
    for ax in axs[1:]:
        ax.legend(loc='upper right', fontsize=8)
    fig.suptitle(path.parent.name+': final ownership and 450 ms lease fallbacks')
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def analyze(root, output):
    """All mutations are new analysis products; input hashes must survive unchanged."""
    import yaml
    if root.resolve() == output.resolve() or root.resolve() in output.resolve().parents:
        raise ValueError('OUTPUT_MUST_BE_OUTSIDE_ORIGINAL_RUN')
    paths = [root/'offline_evidence.jsonl', root/'condition.json']
    paths += sorted((root/'shadow').glob('*.jsonl'))
    paths += sorted((root/'config').glob('*.yaml'))
    paths += sorted((root/'ulogs').glob('*.ulg'))
    paths += [root/'source_snapshot'/rel for rel in CODE]
    before_hashes = {str(p.resolve()): sha256(p) for p in paths}
    rows = load_jsonl(root/'offline_evidence.jsonl')
    condition = json.loads((root/'condition.json').read_text())
    config = yaml.safe_load((root/'config/flight.yaml').read_text())
    tracker = config['trajectory_tracker_node']['ros__parameters']
    research = yaml.safe_load((root/'config/research.yaml').read_text())
    research_parameters = next(iter(research.values()))['ros__parameters']
    lease = condition.get('execution_lease_seconds', .45)
    report = reconstruct(rows, lease, tracker['flight_altitude'])
    report['run'] = root.name
    report['condition'] = condition
    report['config_caveat'] = (
        'flight.yaml records follow_minco_enabled=false; runtime launch overrides '
        'must be read with final diagnostics and real ACKs')
    report['execution_lead_seconds'] = research_parameters['execution_lead']
    report['execution_lease_seconds'] = lease
    shadow_rows, shadow_acks = [], []
    for path in sorted((root/'shadow').glob('*.jsonl')):
        target = shadow_acks if path.name.startswith('follow_ack_') else shadow_rows
        target.extend(load_jsonl(path))

    def key(row):
        return row['stamp'], row.get('plan', row.get('identity', [None])[-1]), row['state']
    monitor_ack_keys = {key(r) for r in rows if r['kind'] == 'follow_ack'}
    shadow_ack_keys = {key(r) for r in shadow_acks}
    report['independent_ack_recorder_crosscheck'] = dict(
        monitor_count=len(monitor_ack_keys), shadow_count=len(shadow_ack_keys),
        monitor_only=len(monitor_ack_keys-shadow_ack_keys),
        shadow_only=len(shadow_ack_keys-monitor_ack_keys))
    publications = [r for r in shadow_rows if r['event'] == 'follow_publication_final']
    for switch in report['switches']:
        lo, hi = switch['last_active_epoch'], switch['receiver_tick_epoch']
        attempts = [dict(stamp=r['stamp'], cycle_id=r['cycle_id'], published=r['published'],
                         reason=r['reason'], remaining_raw_ttl=r.get('remaining_raw_ttl'))
                    for r in publications if lo is not None and hi is not None
                    and lo <= r['stamp'] <= hi]
        switch['planner_publication_attempts_during_last_active'] = attempts
        switch['planner_nonpublication_reasons'] = dict(Counter(
            r['reason'] for r in attempts if not r['published']))
    all_switch_callbacks = [state for switch in report['switches']
                            for state in (switch['before'], switch['after'])]
    report['px4_ulog_crosscheck'] = ulog_check(root, rows, all_switch_callbacks)
    report['limitations'] = [
        'Original FOLLOW desired geometric P/V/A is not logged. PX4 velocity mode P/A are NaN.',
        'Actual acceleration uses the exact recorded navigation sample, not a finite difference.',
        'Before/after are consecutive actual control callbacks, not one common instant.',
        'Prediction IDs in execution_debug are tracker-used IDs; receipt is not causal proof.',
        'LATEST_PREFIX_INVALID and parent candidate rejection differ from active lease expiry.',
        'No measured-PVA divergence gate exists in this nominal execution receiver.',
        'Two c_f2_2 EXPIRED ACKs are overwritten by pending START_MISSED in the same tick.',
        'Planner nonpublication is an upstream event, not proof of a unique physical root cause.',
        'No robust holding qualification or counterfactual closed-loop validation is claimed.']
    output.mkdir(parents=True, exist_ok=True)
    write_json(output/'switches.json', report['switches'])
    write_switch_csv(output/'switches.csv', report['switches'])
    write_json(output/'plans.json', report['plans'])
    write_json(output/'callbacks.json', report['callbacks'])
    events = [dict(r, event_epoch=r.get('generated', r.get('stamp')))
              for r in rows if r['kind'] in (
                  'proposal', 'follow_ack', 'prediction',
                  'execution_debug', 'diagnostic', 'receiver_epoch')]
    events.sort(key=lambda r: r['event_epoch'])
    with (output/'original_clock_timeline.jsonl').open('w') as stream:
        for event in events:
            stream.write(json.dumps(clean(event), allow_nan=False)+'\n')
    first_follow = min(r['stamp'] for r in rows if r['kind'] == 'diagnostic'
                       and r['status'] in ('FOLLOW', 'MINCO_FOLLOW'))
    plot(output/'timeline.png', report, first_follow)
    after_hashes = {str(p.resolve()): sha256(p) for p in paths}
    report['source_inputs_unchanged'] = before_hashes == after_hashes
    if not report['source_inputs_unchanged']:
        raise RuntimeError('ORIGINAL_INPUT_CHANGED_DURING_AUDIT')
    write_json(output/'source_inputs_sha256.json', before_hashes)
    summary = {k: v for k, v in report.items() if k not in ('switches', 'callbacks', 'plans')}
    summary['fallback_count'] = len(report['switches'])
    summary['switch_statistics'] = {k: stats([r[k] for r in report['switches']]) for k in (
        'elapsed_since_last_active_seconds', 'old_reference_age_seconds',
        'expiry_overrun_seconds', 'actual_reference_position_error_before_m',
        'command_velocity_step_mps')}
    write_json(output/'summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root', type=Path,
                        default=Path('data/experiments/20261010_p45_minco_execution'))
    parser.add_argument('--output', type=Path, default=Path(
        'data/experiments/20261010_p47_follow_guided/execution_timeline'))
    parser.add_argument('--runs', nargs='+', default=RUNS)
    args = parser.parse_args()
    summaries = [analyze(args.input_root/run, args.output/run) for run in args.runs]
    write_json(args.output/'summary.json', summaries)
    repo = Path(__file__).resolve().parents[1]
    code = (*CODE, 'scripts/p47_execution_timeline.py')
    write_json(args.output/'audit_code_sha256.json',
               {str(repo/rel): sha256(repo/rel) for rel in code})
    keys = ('run', 'fallback_count', 'direct_causes', 'cause_evidence',
            'upstream_update_states', 'source_inputs_unchanged')
    print(json.dumps([{k: r[k] for k in keys} for r in summaries]))


if __name__ == '__main__':
    main()

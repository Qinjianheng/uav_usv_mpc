#!/usr/bin/env python3
"""Read-only P4.5 reference/vehicle attribution at original navigation epochs.

All truth use is offline evaluation. Native ULog time is paired only with
recorded navigation sample epochs; neither arrival time nor fitted lag is used.
Intermediate shaping/sea signals absent from the recorder remain explicitly
unobserved. A source-fingerprint-gated replay is reported separately.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = WORKSPACE / 'data/experiments/20261010_p45_minco_execution'
DEFAULT_OUTPUT = WORKSPACE / (
    'data/experiments/20261010_p47_follow_guided/execution_physical')
RUNS = ('b_f2_1', 'b_f2_2', 'c_f2_1', 'c_f2_2')
SOURCE_FILES = (
    'src/uav_control/uav_control/control/trajectory_tracker_node.py',
    'src/uav_control/uav_control/control/flight_guidance.py',
    'src/uav_control/uav_control/guidance/follow_minco_execution.py',
    'src/uav_control/uav_control/common/sea_safety.py',
    'src/uav_control/uav_control/controllers/p44_follow_solver.py',
    'src/uav_control/uav_control/controllers/p43_follow_solver.py',
    'src/uav_control/uav_control/guidance/adaptive_follow_initializer.py',
    'scripts/p4_shadow_monitor.py',
)


def digest(path):
    """Stream original bytes, including large ULogs, without modifying a source."""
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def interpolate(times, values, query, max_gap):
    """Bracketed interpolation only; explicit refusal of gaps and clock resets."""
    times, values = np.asarray(times), np.asarray(values)
    if len(times) < 2 or not np.all(np.diff(times) > 0):
        raise ValueError('NON_MONOTONIC_OR_EMPTY_SUPPORT')
    if not times[0] <= query <= times[-1]:
        raise ValueError('OUTSIDE_SUPPORT')
    hi = int(np.searchsorted(times, query))
    if hi < len(times) and query == times[hi]:
        return np.asarray(values[hi])
    lo = hi - 1
    if times[hi] - times[lo] > max_gap:
        raise ValueError('INTERPOLATION_GAP')
    weight = (query - times[lo]) / (times[hi] - times[lo])
    return values[lo] + weight * (values[hi] - values[lo])


def navigation_mapping(nav):
    """Keep original order so a native or ROS epoch reset cannot be concealed."""
    nav = [r for r in nav if r['valid']]
    native = np.asarray([r['native'] for r in nav], dtype=np.int64)
    epochs = np.asarray([r['stamp'] for r in nav])
    if (len(native) < 2 or not np.all(np.diff(native) > 0)
            or not np.all(np.diff(epochs) > 0)):
        raise ValueError('NAVIGATION_MAPPING_NOT_MONOTONIC')
    return native, epochs


def map_native(mapping, native):
    """Map native microseconds to observed navigation epoch, bounded at 100 ms."""
    return float(interpolate(mapping[0], mapping[1], native, 100000))


def desired_point(position, velocity):
    """Offline USV rear 5 m viewpoint; desired altitude is fixed NED -5 m."""
    speed = np.linalg.norm(velocity[:2])
    if speed <= 1e-6:
        raise ValueError('TRUTH_HEADING_UNDEFINED')
    return np.r_[np.asarray(position[:2]) - 5 * np.asarray(velocity[:2]) / speed, -5.]


def decompose(reference, tracking, actual):
    """Vector RMSE with E||r+t||² identity and centered covariance, no scalar subtraction."""
    if not len(reference):
        return dict(samples=0)
    reference, tracking, actual = map(np.asarray, (reference, tracking, actual))
    result = dict(samples=len(reference))
    for name, columns in (('xy', [0, 1]), ('z', [2]), ('xyz', [0, 1, 2])):
        r, t, e = (x[:, columns] for x in (reference, tracking, actual))
        mean_r, mean_t = r.mean(axis=0), t.mean(axis=0)
        mse_r, mse_t, mse_e = (float(np.mean(np.sum(x*x, axis=1))) for x in (r, t, e))
        covariance = float(np.mean(np.sum((r-mean_r)*(t-mean_t), axis=1)))
        cross = float(2*np.mean(np.sum(r*t, axis=1)))
        result[name] = dict(reference_rmse=mse_r**.5, tracking_rmse=mse_t**.5,
                            actual_rmse=mse_e**.5, reference_mse=mse_r,
                            tracking_mse=mse_t, actual_mse=mse_e, cross_term=cross,
                            centered_cross_covariance=covariance,
                            mean_reference_error=mean_r.tolist(),
                            mean_tracking_error=mean_t.tolist(),
                            mean_actual_error=e.mean(axis=0).tolist(),
                            closure_max_abs=float(np.max(np.abs(e-r-t))),
                            mse_identity_residual=mse_e-mse_r-mse_t-cross)
    return result


def sample_curve(proposal, epoch):
    """Sample recorded polynomial coefficients only within declared support."""
    if not proposal['start'] <= epoch <= proposal['end']:
        raise ValueError('CURVE_OUTSIDE_SUPPORT')
    durations = np.asarray(proposal['durations'])
    relative = epoch-proposal['start']
    if not 0 <= relative <= durations.sum()+2*np.spacing(proposal['end']):
        raise ValueError('CURVE_OUTSIDE_SUPPORT')
    ends = np.cumsum(durations)
    index = min(int(np.searchsorted(ends, relative, side='right')), len(ends)-1)
    local = relative-(ends[index-1] if index else 0.)
    coefficients = np.asarray(proposal['xyz']).reshape(-1, 6, 3)[index]
    p = sum(coefficients[k]*local**k for k in range(6))
    v = sum(k*coefficients[k]*local**(k-1) for k in range(1, 6))
    a = sum(k*(k-1)*coefficients[k]*local**(k-2) for k in range(2, 6))
    return np.r_[p, v, a]


def pair_callback(debug, diagnostics, commands):
    """Pair within a recorded callback span using message header epochs only."""
    epochs = [r['stamp'] for r in diagnostics]
    i = int(np.searchsorted(epochs, debug['stamp'], side='right'))-1
    if i < 0:
        raise ValueError('CALLBACK_NOT_FOUND')
    tick = diagnostics[i]
    if (debug['stamp']-tick['stamp'] > tick['compute']+.002
            or tick['plan'] != debug['plan'] or tick['status'] != 'MINCO_FOLLOW'):
        raise ValueError('CALLBACK_SPAN_OR_OWNER_MISMATCH')
    cs = [r['stamp'] for r in commands]
    j = int(np.searchsorted(cs, tick['stamp']-.000002))
    if j >= len(commands) or not -.000002 <= commands[j]['stamp']-tick['stamp'] <= .005:
        raise ValueError('CALLBACK_COMMAND_NOT_FOUND')
    return tick, commands[j], i


def prepare_output(output, sources):
    """Refuse in-place analysis and repeat-run overwrites."""
    output = output.resolve()
    if any(output == source.resolve() or source.resolve() in output.parents for source in sources):
        raise ValueError('OUTPUT_INSIDE_RAW_SOURCE')
    output.mkdir(parents=True, exist_ok=False)


def stats(values):
    """Finite numeric population summary; no fabricated zero for absent data."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return dict(count=0, min=None, max=None, mean=None, rmse=None, p95_abs=None)
    return dict(count=len(values), min=float(values.min()), max=float(values.max()),
                mean=float(values.mean()), rmse=float(np.sqrt(np.mean(values*values))),
                p95_abs=float(np.percentile(np.abs(values), 95)))


def write_csv(path, records):
    """Flat new CSV; columns follow first occurrence across records."""
    keys = list(dict.fromkeys(k for r in records for k in r))
    with path.open('x') as stream:
        writer = csv.DictWriter(stream, keys)
        writer.writeheader()
        writer.writerows(records)


def source_provenance(root):
    """Gate replay on hashes from that run, not a historical HEAD assumption."""
    snapshot = json.loads((root/'source_snapshot.json').read_text())
    files = {}
    for name in SOURCE_FILES:
        path = WORKSPACE/name
        current = digest(path) if path.is_file() else None
        recorded = snapshot['files'].get(name)
        files[name] = dict(current_sha256=current, recorded_sha256=recorded,
                           matches=bool(current and current == recorded))
    replay_files = SOURCE_FILES[:4]
    return dict(recorded_head=snapshot['head'], files=files,
                replay_source_matches=all(files[n]['matches'] for n in replay_files))


def read_native(root, nav, start):
    """Read PX4 native sampled states and distinct native command publication epochs."""
    from pyulog import ULog
    mapping = navigation_mapping(nav)
    nav_by_native = {r['native']: r for r in nav if r['valid']}
    records, dataset_inventory, rejects, checks = [], [], Counter(), []
    names = ['vehicle_local_position', 'trajectory_setpoint',
             'vehicle_local_position_setpoint', 'vehicle_thrust_setpoint',
             'vehicle_angular_velocity']
    for path in sorted((root/'ulogs').glob('*.ulg')):
        ulog = ULog(str(path), message_name_filter_list=names)
        for dataset in ulog.data_list:
            data = dataset.data
            dataset_inventory.append(dict(path=str(path), name=dataset.name,
                                          multi_id=dataset.multi_id,
                                          count=len(data['timestamp']), fields=list(data)))
            # Command topics do not have timestamp_sample. Their timestamp is
            # native publication time, never represented as a measurement epoch.
            is_sample = 'timestamp_sample' in data
            native_times = data['timestamp_sample'] if is_sample else data['timestamp']
            for i, native in enumerate(native_times):
                try:
                    epoch = map_native(mapping, int(native))
                except ValueError as error:
                    rejects[str(error)] += 1
                    continue
                record = dict(dataset=dataset.name, native=int(native), epoch=epoch,
                              follow_seconds=epoch-start,
                              epoch_kind='sample' if is_sample else 'publication')
                if dataset.name == 'vehicle_local_position':
                    original = nav_by_native.get(int(native))
                    if original:
                        checks.append(max(abs(float(data[k][i])-original['p'][j])
                                          for j, k in enumerate(('x', 'y', 'z'))))
                    record.update(z=float(data['z'][i]), vz=float(data['vz'][i]))
                elif dataset.name == 'trajectory_setpoint':
                    record.update(vx=float(data['velocity[0]'][i]),
                                  vy=float(data['velocity[1]'][i]),
                                  vz=float(data['velocity[2]'][i]))
                elif dataset.name == 'vehicle_local_position_setpoint':
                    record.update(vx=float(data['vx'][i]), vy=float(data['vy'][i]),
                                  vz=float(data['vz'][i]))
                else:
                    record.update(vector_norm=float(np.linalg.norm(
                        [data[f'xyz[{j}]'][i] for j in range(3)])))
                records.append(record)
    summary = dict(dataset_inventory=dataset_inventory, mapping_rejections=dict(rejects),
                   exact_navigation_native_matches=len(checks),
                   max_position_difference_on_exact_native_match=max(checks) if checks else None,
                   mapping_convention='original native sample -> navigation stamp pairs; '
                   'bracketed interpolation <=100 ms; command timestamp is publication epoch')
    for name in names:
        selected = sorted((r for r in records if r['dataset'] == name
                           and 20 <= r['follow_seconds'] <= 60), key=lambda r: r['native'])
        summary[name] = dict(count=len(selected),
                             logged_period_seconds=stats(np.diff([r['native']/1e6
                                                                  for r in selected])),
                             vz=stats([r['vz'] for r in selected if 'vz' in r]))
    return records, summary


def native_correspondence(observed, commands):
    """Value corroboration in a causal 150 ms interval, never a fitted epoch alignment.

    Decimated ULog command publication time can follow its ROS control header.
    Count identical command vectors in the bounded prior interval and retain
    ambiguity; do not shift timestamps to make a preferred command match.
    """
    epochs = np.asarray([r['stamp'] for r in commands])
    records = []
    for row in observed:
        if row['dataset'] != 'trajectory_setpoint':
            continue
        lo = int(np.searchsorted(epochs, row['epoch']-.15))
        hi = int(np.searchsorted(epochs, row['epoch'], side='right'))
        logged = np.asarray([row[k] for k in ('vx', 'vy', 'vz')])
        matches = [r for r in commands[lo:hi]
                   if np.allclose(logged, r['v'], atol=1e-6, rtol=0)]
        records.append(dict(
            epoch=row['epoch'], native=row.get('native'),
            follow_seconds=row.get('follow_seconds'), causal_candidates=hi-lo,
            exact_vector_matches=len(matches),
            matching_header_epochs=';'.join(str(r['stamp']) for r in matches)))
    return records


def replay_command(debug, tick, previous_tick, proposal, params):
    """Derived replay only: inputs logged, source hash checked by caller, output cross-checked."""
    sys.path.insert(0, str(WORKSPACE/'src/uav_control'))
    from uav_control.control.flight_guidance import FlightGuidanceCore
    from uav_control.common.sea_safety import apply_sea_safety_guard
    dt = 1./params['control_rate_hz']
    measured_dt = tick['stamp']-previous_tick['stamp']
    if .001 <= measured_dt <= .25:
        dt = measured_dt
    dt = min(dt, params['maximum_command_dt'])
    current = sample_curve(proposal, debug['stamp'])
    raw = current[3:6] + params['follow_position_gain'] * (
        np.asarray(debug['reference'][:3])-debug['actual_position'])
    core = FlightGuidanceCore(
        maximum_horizontal_speed=params['guidance_maximum_horizontal_speed'],
        maximum_vertical_speed=params['maximum_vertical_speed'],
        control_dt=1./params['control_rate_hz'])
    core.previous_velocity = tuple(debug['previous_command'])
    shaped = core._shape_velocity(raw, dt, params['guidance_maximum_horizontal_acceleration'],
                                  params['maximum_vertical_acceleration'])
    guard = apply_sea_safety_guard(
        debug['actual_position'][2], debug['actual_velocity'][2], shaped[2],
        params['sea_surface_z'], params['reserve_clearance'], params['safety_response_delay'],
        params['vertical_braking_acceleration'], max(dt, core.control_dt),
        maximum_vertical_speed=params['maximum_vertical_speed'])
    final = np.r_[shaped[:2], guard.command_vz]
    return raw, np.asarray(shaped), final, guard


def shadow_metadata(root):
    """Plan id is logged cycle id; capture actual selected descriptor and parent boundary."""
    metadata = {}
    for path in sorted((root/'shadow').glob('mpc_seed_shadow_*.jsonl')):
        with path.open() as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('event') != 'completion' or not row.get('published_valid'):
                    continue
                result = row.get('core_result') or {}
                request = row.get('request') or {}
                context = request.get('context', {})
                metrics = result.get('metrics', {})
                progress = metrics.get('progress', {}).get('reference_derivatives', {})
                p43 = metrics.get('p43', {})
                attempts = p43.get('attempts', [])
                selected = next((a for a in reversed(attempts) if a.get('valid')), {})
                state = request.get('state', [])
                metadata[context.get('cycle_id')] = dict(
                    cycle=context.get('cycle_id'), published=row.get('publish_stamp'),
                    start=context.get('execution_start_stamp'),
                    scale=selected.get('scale', progress.get('scale')),
                    candidate_index=selected.get(
                        'candidate_index', progress.get('candidate_index')),
                    beta=selected.get('beta'), boundary=request.get('boundary_policy'),
                    parent_plan=(request.get('prior_curve') or {}).get('plan_id'),
                    initial_z=state[2] if len(state) >= 9 else None,
                    initial_vz=state[5] if len(state) >= 9 else None,
                    initial_az=state[8] if len(state) >= 9 else None,
                    ranked_candidates=p43.get('ranked_candidates'), attempts=attempts)
    return metadata


def analyze(root, output):
    """Analyze raw records once, with rejection counts and separate observation/replay signals."""
    import yaml
    kinds = defaultdict(list)
    with (root/'offline_evidence.jsonl').open() as stream:
        for line_number, line in enumerate(stream, 1):
            row = json.loads(line)
            row['_source_line'] = line_number
            kinds[row['kind']].append(row)
    diagnostics = sorted(kinds['diagnostic'], key=lambda r: r['stamp'])
    follow = [r for r in diagnostics if r['status'] in ('FOLLOW', 'MINCO_FOLLOW')]
    start = follow[0]['stamp']
    end = follow[-1]['stamp']
    nav = [r for r in kinds['uav'] if r['valid']]
    navigation_mapping(nav)
    nav_by_stamp = {r['stamp']: r for r in nav}
    truth = [r for r in kinds['ideal'] if r['valid']]
    truth_times = [r['stamp'] for r in truth]
    truth_values = [r['p']+r['v'] for r in truth]
    proposals = {r['plan']: r for r in kinds['proposal']}
    commands = sorted(kinds['reference'], key=lambda r: r['stamp'])
    provenance = source_provenance(root)
    params = yaml.safe_load((root/'config/flight.yaml').read_text())[
        'trajectory_tracker_node']['ros__parameters']
    metadata = shadow_metadata(root)
    traces, physical, replay, exclusions = [], [], [], Counter()
    diag_times = [r['stamp'] for r in diagnostics]
    for row in nav:
        epoch = row['stamp']
        if not start <= epoch <= end:
            continue
        try:
            target = interpolate(truth_times, truth_values, epoch, .15)
            desired = desired_point(target[:3], target[3:])
        except ValueError as error:
            exclusions['physical_'+str(error)] += 1
            continue
        i = int(np.searchsorted(diag_times, epoch, side='right'))-1
        owner = diagnostics[i]['status'] if i >= 0 and epoch-diag_times[i] <= .15 else 'UNKNOWN'
        error = np.asarray(row['p'])-desired
        physical.append(dict(epoch=epoch, native=row['native'], follow_seconds=epoch-start,
                             owner_causal_at_sample=owner, z_uav=row['p'][2], z_desired=-5.,
                             vx_uav=row['v'][0], vy_uav=row['v'][1], vz_uav=row['v'][2],
                             error_x=error[0], error_y=error[1], error_z=error[2]))
    for row in kinds['execution_debug']:
        if not row['plan'] or 'reference' not in row or not start <= row['state_stamp'] <= end:
            continue
        epoch = row['state_stamp']
        if epoch < row['active_start']:
            exclusions['reference_sample_clamped_to_future_active_start'] += 1
            continue
        proposal = proposals.get(row['plan'])
        if not proposal:
            exclusions['missing_proposal'] += 1
            continue
        original = nav_by_stamp.get(epoch)
        if not original:
            exclusions['exact_navigation_epoch_missing'] += 1
            continue
        if not np.allclose(original['p'], row['actual_position'], atol=1e-9, rtol=0):
            exclusions['debug_navigation_position_mismatch'] += 1
            continue
        try:
            target = interpolate(truth_times, truth_values, epoch, .15)
            desired = desired_point(target[:3], target[3:])
            recorded = np.asarray(row['reference'][:9])
            recomputed = sample_curve(proposal, epoch)
            tick, command, tick_index = pair_callback(row, diagnostics, commands)
        except ValueError as error:
            exclusions[str(error)] += 1
            continue
        actual = np.asarray(row['actual_position'])
        r_error, t_error, a_error = recorded[:3]-desired, actual-recorded[:3], actual-desired
        meta = metadata.get(row['plan'], {})
        record = dict(epoch=epoch, native=original['native'], follow_seconds=epoch-start,
                      debug_epoch=row['stamp'], command_header_epoch=command['stamp'],
                      tick_epoch=tick['stamp'], plan=row['plan'], parent=row.get('parent'),
                      active_start=row['active_start'], scale=meta.get('scale'),
                      candidate_index=meta.get('candidate_index'), boundary=meta.get('boundary'),
                      z_ref=recorded[2], z_uav=actual[2], z_desired=-5.,
                      vx_ref=recorded[3], vy_ref=recorded[4], vz_ref=recorded[5],
                      vx_uav=row['actual_velocity'][0], vy_uav=row['actual_velocity'][1],
                      vz_uav=row['actual_velocity'][2],
                      vx_final_ros=command['v'][0], vy_final_ros=command['v'][1],
                      vz_final_ros=command['v'][2],
                      coefficient_sample_max_abs=float(np.max(np.abs(recorded-recomputed))),
                      source_debug_line=row['_source_line'],
                      source_nav_line=original['_source_line'])
        for name, error in (('reference', r_error), ('tracking', t_error), ('actual', a_error)):
            record.update({name+'_error_'+axis: float(error[i]) for i, axis in enumerate('xyz')})
        traces.append(record)
        if provenance['replay_source_matches'] and tick_index and row.get('previous_command'):
            try:
                raw, shaped, final, guard = replay_command(
                    row, tick, diagnostics[tick_index-1], proposal, params)
                replay.append(dict(epoch=epoch, follow_seconds=epoch-start, plan=row['plan'],
                                   tick_epoch=tick['stamp'], debug_epoch=row['stamp'],
                                   raw_vz_derived=float(raw[2]),
                                   shaped_vz_derived=float(shaped[2]),
                                   sea_final_vz_derived=float(final[2]),
                                   logged_final_vz=command['v'][2],
                                   vertical_shape_delta=float(shaped[2]-raw[2]),
                                   vertical_sea_delta=float(final[2]-shaped[2]),
                                   sea_state_derived=guard.state.value,
                                   replay_max_abs_residual=float(np.max(
                                       np.abs(final-np.asarray(command['v']))))))
            except (ValueError, KeyError, TypeError) as error:
                exclusions['replay_'+str(error)] += 1
    output.mkdir()
    native, native_summary = read_native(root, nav, start)
    corroboration = native_correspondence(native, commands)
    write_csv(output/'active_same_epoch.csv', traces)
    write_csv(output/'physical_navigation.csv', physical)
    write_csv(output/'native_px4_signals.csv', native)
    write_csv(output/'native_command_corroboration.csv', corroboration)
    write_csv(output/'derived_shaping_replay.csv', replay)
    write_csv(output/'ros_final_commands.csv', [
        dict(epoch=r['stamp'], follow_seconds=r['stamp']-start,
             vx=r['v'][0], vy=r['v'][1], vz=r['v'][2])
        for r in commands if start <= r['stamp'] <= end])
    for plan, meta in metadata.items():
        meta['active_ack_epochs'] = [
            r['stamp'] for r in kinds['follow_ack']
            if r['plan'] == plan and r['state'] == 'ACTIVE']
        meta['same_epoch_execution_samples'] = sum(r['plan'] == plan for r in traces)
        proposal = proposals.get(plan)
        meta['proposal_start_matches'] = bool(
            proposal and abs(proposal['start']-meta['start']) < 1e-6)
    (output/'selected_candidates.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
    report = dict(run=root.name, condition=json.loads((root/'condition.json').read_text()),
                  follow_start=start, follow_end=end, source=str(root),
                  provenance=provenance, input_kind_counts={k: len(v) for k, v in kinds.items()},
                  exclusions=dict(exclusions), native=native_summary,
                  missing_signals=[
                      'Direct pre-shaping and intermediate shaped velocity were not logged.',
                      'Recorder omitted ControllerDiagnostic safety_state and safety_margin.',
                      'ULog trajectory_setpoint is decimated; no full-rate PX4 receive history.',
                      'No physical plant counterfactual or new flight was performed.'],
                  conventions=dict(
                      desired='truth XY -5 m * unit horizontal velocity, z=-5 m; offline only',
                      attribution='actual-desired=(reference-desired)+(actual-reference), vectors',
                      epoch='exact debug state_stamp paired to original navigation record; '
                      'reject state_stamp<active_start; truth bracket <=150 ms, no extrapolation',
                      command='ROS header is control tick time, debug header later in callback; '
                      'velocities shown at their recorded epochs; no receipt-time alignment',
                      replay='derived from logged inputs and byte-matching runtime sources; '
                      'never labelled as directly observed shaping or safety state'))
    windows = {}
    for name, lo, hi in (('all_follow', 0., end-start), ('transition_0_20', 0., 20.),
                         ('stable_20_60', 20., 60.), ('late_60_end', 60., end-start)):
        selected = [r for r in traces if lo <= r['follow_seconds'] <= hi]
        measured = [r for r in physical if lo <= r['follow_seconds'] <= hi]
        derived = [r for r in replay if lo <= r['follow_seconds'] <= hi]
        ref = [[r['reference_error_'+a] for a in 'xyz'] for r in selected]
        track = [[r['tracking_error_'+a] for a in 'xyz'] for r in selected]
        act = [[r['actual_error_'+a] for a in 'xyz'] for r in selected]
        window = dict(active_decomposition=decompose(ref, track, act),
                      all_owner_physical=dict(
                          samples=len(measured),
                          xy_rmse=(float(np.sqrt(np.mean([r['error_x']**2+r['error_y']**2
                                                         for r in measured])))
                                   if measured else None),
                          z_error=stats([r['error_z'] for r in measured])),
                      z_ref=stats([r['z_ref'] for r in selected]),
                      vz_ref=stats([r['vz_ref'] for r in selected]),
                      vz_final_ros=stats([r['vz_final_ros'] for r in selected]),
                      vz_actual=stats([r['vz_uav'] for r in selected]),
                      replay=dict(samples=len(derived),
                                  residual=stats([r['replay_max_abs_residual'] for r in derived]),
                                  shape_changes=sum(abs(r['vertical_shape_delta']) > 1e-6
                                                    for r in derived),
                                  sea_changes=sum(abs(r['vertical_sea_delta']) > 1e-6
                                                  for r in derived),
                                  derived_sea_states=dict(Counter(
                                      r['sea_state_derived'] for r in derived))))
        high = [r for r in selected if abs(r['z_ref']+5) >= 1.]
        window['high_altitude_error'] = dict(
            samples=len(high), unique_plans=len({r['plan'] for r in high}),
            scale_counts=dict(Counter(str(r['scale']) for r in high)),
            parented_samples=sum(bool(r['parent']) for r in high),
            zero_scale_parented_samples=sum(r['scale'] == 0 and bool(r['parent']) for r in high))
        windows[name] = window
    report['windows'] = windows
    stable_native = [r for r in corroboration if 20 <= r['follow_seconds'] <= 60]
    report['native_command_corroboration'] = dict(
        stable_logged_samples=len(stable_native),
        at_least_one_exact_causal_ros_vector=sum(
            r['exact_vector_matches'] > 0 for r in stable_native),
        ambiguous_matches=sum(r['exact_vector_matches'] > 1 for r in stable_native),
        convention='Identical vectors among ROS headers in prior 150 ms after native-pair '
        'mapping. This corroborates reception; it is not fitted lag or exact command attribution.')
    report['coefficient_sample_max_abs'] = max(
        (r['coefficient_sample_max_abs'] for r in traces), default=None)
    transitions = []
    for a, b in zip(follow, follow[1:]):
        if a['status'] != b['status'] and b['stamp']-a['stamp'] <= .15:
            prior = [r for r in commands if abs(r['stamp']-a['stamp']) < .005]
            after = [r for r in commands if abs(r['stamp']-b['stamp']) < .005]
            if prior and after:
                delta = np.subtract(after[0]['v'], prior[0]['v'])
                transitions.append(dict(epoch=b['stamp'], follow_seconds=b['stamp']-start,
                                        previous=a['status'], next=b['status'],
                                        delta_vxy=float(np.linalg.norm(delta[:2])),
                                        delta_vz=float(delta[2])))
    write_csv(output/'owner_switches.csv', transitions)
    stable_switches = [r for r in transitions if 20 <= r['follow_seconds'] <= 60]
    report['stable_owner_switches'] = dict(
        count=len(stable_switches),
        delta_vxy=stats([r['delta_vxy'] for r in stable_switches]),
        delta_vz=stats([r['delta_vz'] for r in stable_switches]))
    report['stable_ownership_ticks'] = dict(Counter(
        r['status'] for r in follow if 20 <= r['stamp']-start <= 60))
    (output/'diagnosis.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    plot(output, traces, physical, commands, native, transitions, start)
    return report


def plot(output, traces, physical, commands, native, transitions, start):
    """Standalone scientific figures, preserving separate sample/publication epochs."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    axes[0].plot([r['follow_seconds'] for r in physical], [r['z_uav'] for r in physical],
                 label='UAV z, navigation samples', lw=1.1)
    axes[0].scatter([r['follow_seconds'] for r in traces], [r['z_ref'] for r in traces],
                    label='MINCO z reference, same nav epoch', s=3, c='tab:orange')
    axes[0].axhline(-5, c='black', ls='--', label='desired z = -5 m')
    axes[0].set_ylabel('NED z (m)')
    axes[1].plot([r['follow_seconds'] for r in physical], [r['vz_uav'] for r in physical],
                 label='actual vz, navigation samples', lw=1.1)
    axes[1].scatter([r['follow_seconds'] for r in traces], [r['vz_ref'] for r in traces],
                    label='MINCO vz at navigation epoch', s=3)
    selected = [r for r in commands if start <= r['stamp'] <= start+75]
    axes[1].plot([r['stamp']-start for r in selected], [r['v'][2] for r in selected],
                 label='final ROS vz, control header epoch', lw=.7, alpha=.8)
    logged = [r for r in native if r['dataset'] == 'trajectory_setpoint']
    axes[1].scatter([r['follow_seconds'] for r in logged], [r['vz'] for r in logged],
                    label='PX4 trajectory vz, native publication mapped', s=9, marker='x')
    axes[1].set_ylabel('NED vz (m/s)')
    for kind, label in (('reference', 'reference - desired'), ('tracking', 'UAV - reference'),
                        ('actual', 'UAV - desired')):
        axes[2].scatter([r['follow_seconds'] for r in traces],
                        [np.hypot(r[kind+'_error_x'], r[kind+'_error_y']) for r in traces],
                        label=label, s=3)
    axes[2].set_ylabel('XY error norm (m)')
    axes[2].set_xlabel('Seconds from first recorded FOLLOW tick')
    for ax in axes:
        ax.axvspan(20, 60, alpha=.07, color='green')
        for switch in transitions:
            ax.axvline(switch['follow_seconds'], c='gray', alpha=.08, lw=.6)
        ax.grid(alpha=.2)
        ax.legend(loc='upper left', fontsize=8)
        ax.set_xlim(0, min(75, max(r['follow_seconds'] for r in physical)))
    fig.suptitle(output.name+' | native epochs; truth only in offline desired viewpoint')
    fig.tight_layout()
    fig.savefig(output/'physical_attribution.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--runs', nargs='+', default=list(RUNS))
    args = parser.parse_args()
    roots = [args.source/name for name in args.runs]
    prepare_output(args.output, roots)
    inputs = {}
    for root in roots:
        paths = [root/'offline_evidence.jsonl', root/'condition.json', root/'source_snapshot.json']
        paths += sorted((root/'shadow').glob('*.jsonl'))
        paths += sorted((root/'ulogs').glob('*.ulg'))
        paths += sorted((root/'evaluator').glob('*'))
        paths += sorted((root/'config').glob('*'))
        for path in paths:
            if path.is_file():
                inputs[str(path.resolve())] = dict(bytes=path.stat().st_size, sha256=digest(path))
    reports = [analyze(root, args.output/root.name) for root in roots]
    changed = [path for path, entry in inputs.items() if digest(Path(path)) != entry['sha256']]
    if changed:
        raise RuntimeError('RAW_INPUT_CHANGED_DURING_ANALYSIS: '+str(changed))
    summary = dict(stage='A: diagnosis only; no algorithm/control changes or SITL',
                   raw_input_unchanged=True, inputs=inputs,
                   analysis_script_sha256=digest(Path(__file__)),
                   focused_test_sha256=digest(
                       WORKSPACE/'src/uav_control/test/test_p47_physical_diagnosis.py'),
                   reports={r['run']: str((args.output/r['run']/'diagnosis.json').resolve())
                            for r in reports})
    (args.output/'provenance.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    table = ['| Run | H | Active n | XY ref/track/actual RMSE m | Z ref/track/actual RMSE m |',
             '|---|---:|---:|---|---|']
    for report in reports:
        d = report['windows']['stable_20_60']['active_decomposition']
        xy = '/'.join(f'{d["xy"][k]:.4f}' for k in (
            'reference_rmse', 'tracking_rmse', 'actual_rmse'))
        z = '/'.join(f'{d["z"][k]:.4f}' for k in (
            'reference_rmse', 'tracking_rmse', 'actual_rmse'))
        table.append(f'| {report["run"]} | {report["condition"]["horizon"]} '
                     f'| {d["samples"]} | {xy} | {z} |')
    findings = '\n'.join(table)+'\n\n'+(
        'Population: navigation state epochs 20–60 s after first FOLLOW diagnostic; '
        'active MINCO only, same-epoch truth brackets <=150 ms, no extrapolation. '
        'Each report also separates mixed-owner physical navigation samples and '
        '0–20/60–end intervals. Reference RMSE and tracking RMSE cannot be subtracted: '
        'the reports include vector cross terms and centered cross covariance.\n\n'
        'The H1.6 vertical error is already present in accepted MINCO references; '
        'the vehicle stays much closer to these references than to z=-5 m. '
        'These four logs establish that observation, not a horizon-only causal effect. '
        'b_f2_1 has substantial horizontal reference error.\n\n'
        'Pre-shaping velocities and safety states were not retained by the recorder. '
        'derived_shaping_replay.csv is a source-matched reconstruction with output residuals; '
        'it is not direct observation. Native PX4 setpoints are decimated and remain '
        'at their own mapped publication epochs. Gray lines show observed controller-owner '
        'switches; association is not a flight counterfactual.\n')
    (args.output/'findings.md').write_text(findings)
    outputs = [p for p in args.output.rglob('*') if p.is_file()]
    (args.output/'SHA256SUMS').write_text(''.join(
        f'{digest(path)}  {path.relative_to(args.output)}\n' for path in sorted(outputs)))
    print(findings)


if __name__ == '__main__':
    main()

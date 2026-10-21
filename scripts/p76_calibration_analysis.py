#!/usr/bin/env python3
"""Derive effective physical lift from native inertial response, never DDS thrust units."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


A1_REQUIRED = ('hover', 'thrust_curve', 'rates_response', 'acceleration_5mps2',
               'exclusive_px4_bodyrate', 'navigation_reference_and_sea_datum')
A2_REQUIRED = ('descent_2mps_braking', 'fault_recovery', 'exclusive_px4_bodyrate',
               'navigation_reference_and_sea_datum')


def stage_verdict(stage, run, analysis, rows, identity_valid, raw_complete,
                  prior_qualified=True):
    """Process completion, fixed measured criteria and task success are distinct facts."""
    statuses = [r for r in rows if r['kind'] == 'wire' and r['topic'] == 'vehicle_status']
    landed = any(r['kind'] == 'wire' and r['topic'] == 'vehicle_land_detected'
                 and r['phase'] == 'LAND' and r['values'].get('landed') for r in rows)
    disarmed = bool(statuses and statuses[-1]['phase'] == 'LAND'
                    and statuses[-1]['values'].get('arming_state') == 1
                    and any(r['values'].get('arming_state') == 2 for r in statuses))
    required = A1_REQUIRED if stage == 'A1' else A2_REQUIRED
    checks = {key: analysis.get('checks', {}).get(key) is True for key in required}
    checks.update(identity_current=bool(identity_valid), raw_complete=bool(raw_complete),
                  landed=bool(landed), disarmed=disarmed,
                  no_fault=(not run.get('failure')
                            and not any(r['kind'] == 'failure' for r in rows)))
    if stage == 'A1':
        checks['A1_passed'] = analysis.get('A1_passed') is True
    else:
        checks['prior_A1_qualified'] = bool(prior_qualified)
        checks['measured_braking'] = (analysis.get('braking_acceleration', 0) > 0
                                      and analysis.get('response_seconds', 0) > 0)
    completed = run.get('completed') is True
    qualified = completed and all(checks.values())
    return dict(stage=stage, completed=completed, qualified=qualified, success=qualified,
                checks=checks, failed_checks=[key for key, value in checks.items() if not value])


def assess_stage(session):
    """Recompute immutable raw evidence; publish only a new assessment, never rewrite a result."""
    from uav_control.common.p76_readiness import sha256, source_fingerprint
    from uav_control.common.p76_contract import load_contract
    from uav_control.common.p76_model import verify_model
    root = Path(__file__).resolve().parents[1]
    run = json.loads((session/'run_result.json').read_text())
    identity = json.loads((session/'identity.json').read_text())
    rows = [json.loads(line) for line in (session/'calibration.jsonl').read_text().splitlines()]
    analysis = analyze_rows(rows, identity['model']['total_mass'], identity['model']['gravity'])
    saved_analysis = json.loads((session/'analysis.json').read_text())
    current = False
    try:
        contract = load_contract(identity['contract_file'])
        model_root = Path(json.loads((session/'p76_session_model.json').read_text())['model_root'])
        verify_model(model_root, contract)
        actual_model = json.loads((model_root/'p76_model_manifest.json').read_text())
        px4 = Path(contract['dynamics']['px4_model_root']).parents[3]
        current = (identity['sources'] == source_fingerprint(root)
                   and identity['contract_sha256'] == sha256(contract['_path'])
                   and identity['detector_sha256'] == sha256(contract['detector']['model_file'])
                   and identity['model'] == actual_model
                   and identity['px4'] == dict(
                       binary_sha256=sha256(px4/'build/px4_sitl_default/bin/px4'),
                       dds_sha256=sha256(px4/'src/modules/uxrce_dds_client/dds_topics.yaml')))
    except (OSError, ValueError, KeyError):
        pass
    prior_ok = run['stage'] == 'A1'
    if run['stage'] == 'A2':
        environment = json.loads((session/'environment.json').read_text())
        prior = Path(environment['UAV_USV_P761_PRIOR_A1'])
        prior_ok = (assess_stage(prior)['qualified'] and
                    json.loads((prior/'identity.json').read_text()) == identity)
    result = stage_verdict(run['stage'], run, analysis, rows, current,
                           saved_analysis == analysis and bool(rows), prior_ok)
    result['raw_evidence'] = {str(session/name): sha256(session/name) for name in (
        'calibration.jsonl', 'identity.json', 'analysis.json', 'run_result.json')}
    target = session/'stage_assessment.json'
    temporary = target.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(result, indent=2)+'\n')
    temporary.replace(target)
    return result


def native_force_samples(rows, mass, gravity):
    """Pair full native poses within 50ms with physical navigation acceleration samples."""
    poses = sorted((r for r in rows if r['kind'] == 'pose'), key=lambda r: r['stamp'])
    by_stamp = {r['stamp']: r for r in poses}
    times = np.array(sorted(by_stamp))
    result = []
    for row in rows:
        if row['kind'] != 'state' or len(times) < 2 or row.get('age', 1.) > .125:
            continue
        stamp = row['stamp']
        index = int(np.searchsorted(times, stamp))
        if index < len(times) and abs(times[index]-stamp) < 1e-9:
            q = by_stamp[times[index]]['q']
            rotation = Rotation.from_quat(np.asarray(q)[[1, 2, 3, 0]])
        elif 0 < index < len(times) and times[index]-times[index-1] <= .05+1e-9:
            interval = times[index-1:index+1]
            q = np.array([by_stamp[t]['q'] for t in interval])[:, [1, 2, 3, 0]]
            rotation = Slerp(interval, Rotation.from_quat(q))([stamp])[0]
        else:
            continue
        lift = -rotation.as_matrix()[:, 2]
        force = mass*float(lift@(np.asarray(row['a'])-[0., 0., gravity]))
        if np.isfinite(force) and force > 0:
            result.append(dict(row, force=force,
                               tilt=float(np.arccos(np.clip(-lift[2], -1., 1.)))))
    return result


def analyze_rows(rows, mass, gravity=9.8):
    """Use fixed current-candidate criteria; missing evidence is a failed check."""
    samples = native_force_samples(rows, mass, gravity)
    reasons, curve = [], []
    if not samples:
        reasons.append('NO_NATIVE_FORCE_SAMPLES')
    hover = [r for r in samples if r['phase'] == 'HOVER' and np.linalg.norm(r['v']) < .25]
    for phase in sorted({r['phase'] for r in samples if r['phase'].startswith('THRUST_')}):
        subset = [r for r in samples if r['phase'] == phase and r['phase_elapsed'] >= .08
                  and r['tilt'] < .12]
        if len(subset) >= 3:
            curve.append((float(phase.split('_')[1]), float(
                np.median([r['force'] for r in subset]))))
    curve.sort()
    monotone = len(curve) >= 5 and np.all(np.diff([f for _, f in curve]) > 0)
    maximum = max([f for _, f in curve], default=0.)
    hover_error = (abs(np.median([r['force'] for r in hover])/(mass*gravity)-1.)
                   if hover else float('inf'))
    mode_rows = [r for r in rows if r['kind'] == 'mode']
    exclusive = any(r.get('flags', {}).get('offboard') and r['flags'].get('rates')
                    and not any(r['flags'].get(k) for k in (
                        'position', 'velocity', 'attitude', 'altitude', 'climb_rate'))
                    for r in mode_rows)
    rates = []
    for axis in range(3):
        segment = [r for r in rows if r['kind'] == 'rate_sample'
                   and r['phase'] == f'RATE_{axis}' and r['phase_elapsed'] >= .2]
        rates.append(dict(axis=axis, count=len(segment),
                          rmse=float(np.sqrt(np.mean([(r['desired'][axis]-r['actual'][axis])**2
                                                      for r in segment]))) if segment else None))
    maneuver = [np.linalg.norm(r['a'][:2]) for r in samples if r['phase'] == 'MANEUVER']
    checks = dict(hover=len(hover) >= 25 and hover_error <= .05,
                  thrust_curve=bool(monotone) and maximum/(mass*gravity) >= 2.85,
                  rates_response=all(r['count'] >= 10 and r['rmse'] <= .15 for r in rates),
                  acceleration_5mps2=sum(a >= 5. for a in maneuver) >= 10,
                  exclusive_px4_bodyrate=exclusive,
                  navigation_reference_and_sea_datum=any(r['kind'] == 'reference' for r in rows))
    brakes = [r for r in rows if r['kind'] == 'brake_trial']
    checks['descent_2mps_braking'] = (
        len(brakes) == 3 and any(
            r['initial_speed'] >= 1.9 for r in brakes) and all(
            r['stopped'] for r in brakes))
    fault = [r for r in rows if r['kind'] == 'fault_recovered']
    checks['fault_recovery'] = any(
        r['initial_velocity'][2] >= .5 and r['reliable_deceleration'] > 0 for r in fault)
    checks = {key: bool(value) for key, value in checks.items()}
    if not monotone:
        reasons.append('INSUFFICIENT_OR_NONMONOTONE_MEASURED_FORCE_CURVE')
    hover_u = [-r['xyz'][2] for r in rows
               if r['kind'] == 'executed_collective' and r['phase'] == 'HOVER']
    return dict(hover_normalized=float(np.median(hover_u)) if hover_u else None,
                hover_samples=len(hover),
                scope='Motion-derived effective lift; includes aerodynamic-model uncertainty',
                A1_passed=all(checks[k] for k in checks if k not in (
                    'descent_2mps_braking', 'fault_recovery')), checks=checks, reasons=reasons,
                native_force_samples=len(samples), measured_curve=[dict(normalized=u, force=f)
                                                                   for u, f in curve],
                maximum_effective_thrust=maximum, measured_twr=maximum/(mass*gravity),
                hover_relative_error=hover_error if hover else None, rate_tracking=rates,
                response_seconds=max(
                    [r.get('response_seconds', 0.) for r in brakes+fault], default=0.),
                braking_acceleration=min(
                    [r.get('reliable_deceleration', 0.) for r in brakes+fault], default=0.))


def main():
    """Analyze immutable recorded rows using the session model's measured mass identity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    metadata = json.loads((args.session/'identity.json').read_text())
    rows = [json.loads(line)
            for line in (args.session/'calibration.jsonl').read_text().splitlines()]
    result = analyze_rows(rows, metadata['model']['total_mass'], metadata['model']['gravity'])
    (args.session/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

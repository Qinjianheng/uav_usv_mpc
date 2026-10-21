#!/usr/bin/env python3
"""Derive effective physical lift from native inertial response, never DDS thrust units."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


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
    return dict(scope='Motion-derived effective lift; includes aerodynamic-model uncertainty',
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

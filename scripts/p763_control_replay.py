#!/usr/bin/env python3
"""Offline engineering E2: fixed gain/lag analysis and causal recorded MINCO PVA exposure."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.linalg import eigvals

from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_native import NativePoseHistory
from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker, sample_reference
from uav_control.control.p763_calibration_control import load_bootstrap
from uav_control.guidance.follow_contract import FollowCurve

ROOT = Path(__file__).resolve().parents[1]


def gain_analysis():
    """A finite small-signal lag grid: this is not PX4 or nonlinear flight qualification."""
    rows = []
    for wn in (.8, 1., 1.2):
        for zeta in (.9, 1.):
            kp, kv = wn**2, 2*zeta*wn
            poles = []
            for lag in (.02, .12, .24, .5):
                matrix = np.array(((0., 1., 0.), (0., 0., 1.),
                                   (-kp/lag, -kv/lag, -1/lag)))
                poles.append(float(max(eigvals(matrix).real)))
            rows.append(dict(wn=wn, zeta=zeta, kp=kp, kv=kv,
                             worst_pole_real=max(poles), stable=max(poles) < 0))
    # The explicit conservative candidate is chosen once, not fitted to historical flight error.
    p = v = a = 0.
    errors = []
    for _ in range(4000):
        desired = float(np.clip(1.-p-2.*v, -5., 5.))
        a += .005*(desired-a)/.24
        v += .005*a
        p += .005*v
        errors.append(abs(1.-p))
    return dict(grid=rows, candidate=dict(kp=1., kv=2., lag=.24),
                step_final_error=errors[-1], scope='1D linear lag and bounded acceleration model',
                passed=all(r['stable'] for r in rows) and errors[-1] < .001)


def curve_from_row(row):
    """Restore the saved polynomial coefficients, not the old 20Hz velocity commands."""
    return FollowCurve(row['plan'], 1, row['generation'], row['prediction'], row['parent'],
                       row['navigation'], row['attitude'], row['observation'], row['source'],
                       row['input_until'], row['end'], row['start'], row['end'], row['holding'],
                       tuple(row['durations']), tuple(row['xyz']),
                       tuple(tuple(x) for x in np.asarray(row['yaw']).reshape(-1, 4)))


def recorded_replay(path, contract, output):
    """Retain original receipt/acquisition epochs; grant no actual control ownership."""
    rows = [json.loads(line) for line in path.open()]
    allowed = {'proposal', 'uav', 'attitude', 'execution_debug'}
    events = sorted((r for r in rows if r['kind'] in allowed), key=lambda r: r['receipt'])
    traces = [r for r in events if r['kind'] == 'execution_debug' and r.get('plan', 0)]
    if not traces:
        raise ValueError('P763_NO_RECORDED_MINCO_EXECUTION')
    model = json.loads((ROOT/'data/experiments/20261021_p761_01/frozen/models/'
                       'p76_model_manifest.json').read_text())
    old_identity = json.loads((ROOT/'data/experiments/20261021_p761_01/'
                              'session_A1_03/identity.json').read_text())
    bootstrap = load_bootstrap(ROOT/'data/experiments/20261021_p761_01/session_A1_03',
                               dict(model=model, px4=old_identity['px4']))
    tracker = BodyrateTracker(contract, model['total_mass'], model['maximum_thrust'], bootstrap)
    tracker.initialize([0., 0., 0.], -bootstrap.hover)
    poses, curves, state, debug = NativePoseHistory(), {}, None, None
    cursor, records, costs, rejected = 0, [], [], {}
    first, last = traces[0]['receipt'], traces[-1]['receipt']
    for index in range(int((last-first)/.005)+1):
        now = first+index*.005
        while cursor < len(events) and events[cursor]['receipt'] <= now:
            row = events[cursor]
            cursor += 1
            kind = row['kind']
            if kind == 'proposal':
                curves[row['plan']] = curve_from_row(row)
            elif kind == 'uav':
                state = row
            elif kind == 'attitude':
                poses.add(row['stamp'], row['q'], [0., 0., 0.], [0., 0., 0.])
            elif kind == 'execution_debug':
                debug = row
        try:
            if (state is None or not state['valid'] or not 0 <= now-state['stamp'] <= .125
                    or debug is None or not 0 <= now-debug['stamp'] <= .125):
                raise ValueError('RECORDED_INPUT_NOT_CURRENT')
            pose = poses.at(state['stamp'])
            curve = curves[debug['plan']]
            ref = sample_reference(curve, now, debug['lease_deadline'])
            feedback = sample_reference(curve, state['stamp'], debug['lease_deadline'])
            started = time.perf_counter()
            command = tracker.step(ref, state['p'], state['v'], pose.rotation, .005, pose.omega,
                                   feedback_reference=feedback)
            costs.append(time.perf_counter()-started)
            records.append(dict(control_epoch=now, reference_epoch=now,
                                navigation_epoch=state['stamp'], plan=curve.plan_id,
                                feedback_reference_epoch=state['stamp'],
                                reference=ref.tolist(), rates=command['rates'].tolist(),
                                thrust=command['thrust'],
                                acceleration=command['acceleration'].tolist()))
        except (ValueError, KeyError) as error:
            reason = str(error)
            rejected[reason] = rejected.get(reason, 0)+1
    (output/'recorded_commands.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    valid = True
    for left, right in zip(records, records[1:]):
        valid &= np.linalg.norm(np.subtract(right['rates'], left['rates'])) <= .015+1e-9
        valid &= abs(right['thrust']-left['thrust']) <= .01+1e-9
        valid &= np.linalg.norm(np.subtract(
            right['acceleration'], left['acceleration'])) <= .015+1e-9
    bounds = [contract['candidate']['maximum_body_roll_pitch_rate']]*2 + [
        contract['candidate']['maximum_body_yaw_rate']]
    valid &= all(np.all(np.abs(r['rates']) <= np.asarray(bounds)+1e-9)
                 and np.linalg.norm(r['acceleration']) <= 5.+1e-9
                 and -1 <= r['thrust'] < 0 for r in records)
    return dict(source=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                source_scene='Historical P7 28deg/5m Velocity flight; not current P763 flight',
                consumed_kinds=sorted(allowed), outputs=len(records), rejected=rejected,
                controller_seconds_p95=float(np.percentile(costs, 95)) if costs else None,
                controller_seconds_p99=float(np.percentile(costs, 99)) if costs else None,
                counterfactual_only=True, actual_control_qualified=False,
                attitude_uncertainty_scope=('Historical q only; zero omega/variance '
                                            'placeholders for mapping'),
                output_bounds_and_continuity=bool(valid), passed=bool(records) and bool(valid))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    c = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    gains = gain_analysis()
    recorded = recorded_replay(ROOT/'data/experiments/20261020_p7_radius05_capture_01/'
                               'offline_evidence.jsonl', c, args.output)
    summary = dict(schema_version=2, gain_analysis=gains, recorded_replay=recorded,
                   logic_passed=gains['passed'] and recorded['passed'], flight_qualified=False,
                   scope=('Offline engineering math/replay; '
                          'no closed-loop PX4 stability qualification'))
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['logic_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Derive effective physical lift from native inertial response, never DDS thrust units."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


A1_REQUIRED = ('hover', 'thrust_curve', 'rates_response', 'acceleration_5mps2',
               'exclusive_px4_bodyrate', 'navigation_reference_and_sea_datum')
SCHEMA_VERSION = 2
ARCHITECTURE = 'parallel_dkf_unified_bodyrate'
FORMAL_CORE_PATHS = tuple('src/uav_control/uav_control/control/'+name+'.py' for name in (
    'p76_minco_bodyrate_tracker', 'p76_recovery', 'p76_bodyrate_safety', 'p76_paper_thrust'))
A2_REQUIRED = ('descent_2mps_braking', 'bodyrate_recovery', 'native_land_takeover',
               'exclusive_px4_bodyrate', 'navigation_reference_and_sea_datum')


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
    checks.update(schema_current=(run.get('schema_version') == SCHEMA_VERSION
                                  and run.get('architecture') == ARCHITECTURE
                                  and analysis.get('schema_version') == SCHEMA_VERSION
                                  and analysis.get('architecture') == ARCHITECTURE),
                  identity_current=bool(identity_valid), raw_complete=bool(raw_complete),
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
    return dict(schema_version=SCHEMA_VERSION, architecture=ARCHITECTURE,
                stage=stage, completed=completed, qualified=qualified, success=qualified,
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
    target = session/'stage_assessment_v2.json'
    content = json.dumps(result, indent=2)+'\n'
    if target.exists() and target.read_text() != content:
        target = session/('stage_assessment_v2_'+hashlib.sha256(content.encode()).hexdigest()[:12]
                          + '.json')
    if not target.exists():
        with target.open('x') as stream:
            stream.write(content)
    result['assessment_file'] = str(target)
    return result


def native_force_samples(rows, mass, gravity):
    """Pair full native poses within 50ms with physical navigation acceleration samples."""
    poses = sorted((r for r in rows if r['kind'] == 'pose'), key=lambda r: r['stamp'])
    by_stamp = {r['stamp']: r for r in poses}
    times = np.array(sorted(by_stamp))
    result = []
    for row in rows:
        if row['kind'] != 'state' or len(times) < 2 or not 0 <= row.get('age', 1.) <= .125:
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


def formal_controller_identity(contract, mass, maximum, curve, points, braking,
                               delay, a1_evidence):
    """Pin the exact formal pure core and A1 physical inputs actually exercised by A2."""
    from uav_control.common.p76_readiness import sha256
    root = Path(__file__).resolve().parents[1]
    return dict(controller='P763_BODYRATE_RECOVERY', braking_request=float(braking),
                response_bound=float(delay), settings=dict(contract['bodyrate_follow']),
                core_sources={p: sha256(root/p) for p in FORMAL_CORE_PATHS},
                a1_evidence=dict(a1_evidence),
                contract=dict(candidate=dict(contract['candidate']),
                              bodyrate_follow=dict(contract['bodyrate_follow']),
                              dynamics=dict(gravity=contract['dynamics']['gravity'])),
                mass=mass, maximum_thrust=maximum, measured_curve=curve,
                collision_points_frd=np.asarray(points).tolist(),
                minimum_body_clearance=contract['candidate']['minimum_body_clearance'])


def formal_identity_digest(identity):
    """Reference one immutable raw identity record instead of repeating geometry at 200Hz."""
    return hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()


def make_formal_controller(identity):
    """Calibration and evidence replay instantiate the same target-independent flight core."""
    from uav_control.control.p76_minco_bodyrate_tracker import BodyrateTracker
    from uav_control.control.p76_recovery import BodyrateRecovery
    from uav_control.control.p76_bodyrate_safety import BodyrateSeaBarrier
    from uav_control.control.p76_paper_thrust import CalibratedThrustMap
    curve = identity['measured_curve']
    # The map is the separately validated A1 physical result, not a complete A qualification.
    mapping = CalibratedThrustMap([r['force'] for r in curve],
                                  [r['normalized'] for r in curve], qualified=True)
    outer = BodyrateTracker(identity['contract'], identity['mass'],
                            identity['maximum_thrust'], mapping)
    recovery = BodyrateRecovery(identity['braking_request'], identity['response_bound'])
    barrier = BodyrateSeaBarrier(identity['collision_points_frd'],
                                 identity['minimum_body_clearance'], identity['response_bound'],
                                 identity['braking_request'])
    return outer, recovery, barrier


def formal_recovery_evidence(rows):
    """Replay raw control inputs and compare published outputs; a label cannot qualify A2."""
    from uav_control.common.p76_readiness import sha256
    root = Path(__file__).resolve().parents[1]
    traces = [r for r in rows if r['kind'] == 'formal_control']
    result = dict(passed=False, samples=0, phases=[], identity=None)
    identities = [r for r in rows if r['kind'] == 'formal_controller_identity']
    if not traces or len(identities) != 1:
        return result
    try:
        identity = identities[0]['identity']
        identity_digest = formal_identity_digest(identity)
        if identities[0]['identity_sha256'] != identity_digest:
            return result
        if (identity['controller'] != 'P763_BODYRATE_RECOVERY' or not identity['a1_evidence']
                or identity['settings'] != identity['contract']['bodyrate_follow']
                or set(identity['core_sources']) != set(FORMAL_CORE_PATHS)
                or any(not Path(p).is_file() or sha256(p) != digest
                       for p, digest in identity['a1_evidence'].items())
                or any(sha256(root/p) != digest
                       for p, digest in identity['core_sources'].items())):
            return result
        outer, recovery, barrier = make_formal_controller(identity)
        native = {r['stamp']: r for r in rows if r['kind'] == 'state'}
        poses = {r['stamp']: r for r in rows if r['kind'] == 'pose'}
        pose_times = np.array(sorted(poses))
        commands = {r['stamp']: r for r in rows if r['kind'] == 'command'
                    and 'stamp' in r}
        recovery_phase, phases, seeded, previous_now = None, set(), False, None
        for row in traces:
            if row['controller_identity_sha256'] != identity_digest:
                return result
            data = row['inputs']
            state, command = native[data['native_stamp']], commands[row['command_stamp']]
            stamp = data['native_stamp']
            index = int(np.searchsorted(pose_times, stamp))
            if index < len(pose_times) and pose_times[index] == stamp:
                q, omega = poses[stamp]['q'], poses[stamp]['omega']
                rotation = Rotation.from_quat(np.asarray(q)[[1, 2, 3, 0]]).as_matrix()
            elif 0 < index < len(pose_times) and pose_times[index]-pose_times[index-1] <= .05:
                times = pose_times[index-1:index+1]
                q = np.asarray([poses[t]['q'] for t in times])[:, [1, 2, 3, 0]]
                rotation = Slerp(times, Rotation.from_quat(q))([stamp])[0].as_matrix()
                fraction = (stamp-times[0])/(times[1]-times[0])
                omega = ((1-fraction)*np.asarray(poses[times[0]]['omega'])
                         + fraction*np.asarray(poses[times[1]]['omega']))
            else:
                return result
            if (not np.allclose(rotation, data['rotation'], rtol=0, atol=1e-8)
                    or not np.allclose(omega, data['omega'], rtol=0, atol=1e-8)):
                return result
            if previous_now is not None and abs(data['dt']-(data['now']-previous_now)) > 1e-6:
                return result
            previous_now = data['now']
            if (not np.allclose(state['p'], data['position'], rtol=0, atol=1e-9)
                    or not np.allclose(state['v'], data['velocity'], rtol=0, atol=1e-9)
                    or not 0 <= data['age'] <= .125
                    or abs(data['now']-data['native_stamp']-data['age']) > 1e-6):
                return result
            if row.get('seed') is not None:
                if seeded:
                    return result
                outer.initialize(row['seed']['rates'], row['seed']['thrust'])
                seeded = True
            if not seeded:
                return result
            barrier.sea = data['sea']
            if row['recovery']:
                if recovery_phase != row['phase_begin']:
                    recovery.reset()
                    recovery_phase = row['phase_begin']
                output = recovery.step(outer, np.asarray(data['position']),
                                       np.asarray(data['velocity']), np.asarray(data['rotation']),
                                       np.asarray(data['omega']), data['now'], data['dt'],
                                       data['age'], barrier)
                phases.add(row['phase'])
            else:
                output = outer.step(row['reference'], data['position'], data['velocity'],
                                    data['rotation'], data['dt'], data['omega'])
            if (command.get('interface') != 'body_rate'
                    or not np.allclose(command['rates'], output['rates'], rtol=0, atol=1e-8)
                    or abs(command['thrust']+output['thrust']) > 1e-8
                    or not np.allclose(row['acceleration'], output['acceleration'],
                                       rtol=0, atol=1e-8)):
                return result
            result['samples'] += 1
        result.update(passed=True, phases=sorted(phases), identity=identity)
    except (KeyError, ValueError, TypeError, OSError, IndexError, np.linalg.LinAlgError):
        return result
    return result


def bounded_response(rows, phase, formal_evidence=None):
    """Measure braking from native epochs; event names alone provide no evidence."""
    formal = (formal_recovery_evidence(rows) if formal_evidence is None else formal_evidence)
    formal_valid = formal['passed'] and phase in formal['phases']
    states = sorted((r for r in rows if r['kind'] == 'state' and r.get('phase') == phase
                     and 0 <= r.get('age', 1.) <= .125), key=lambda r: r['stamp'])
    states = list({r['stamp']: r for r in states}.values())
    commands = [r for r in rows if r['kind'] == 'command' and r.get('phase') == phase]
    modes = [r['flags'] for r in rows if r['kind'] == 'mode' and r.get('phase') == phase]
    initial = float(states[0]['v'][2]) if states else 0.
    response, positive = 0., []
    for before, after in zip(states, states[1:]):
        delta = after['stamp']-before['stamp']
        if 0 < delta <= .05 and before['v'][2] > .1:
            deceleration = (before['v'][2]-after['v'][2])/delta
            if deceleration > 0:
                positive.append(deceleration)
        if not response and after['v'][2] < initial-.05:
            response = after['stamp']-states[0]['stamp']
    braking = float(np.percentile(positive, 10)) if positive else 0.
    body_only = bool(commands and modes) and all(
        c.get('interface') == 'body_rate' for c in commands) and all(
        m.get('offboard') and m.get('rates') and not any(m.get(k) for k in (
            'position', 'velocity', 'attitude', 'altitude', 'climb_rate')) for m in modes)
    stopped = bool(states and np.linalg.norm(states[-1]['v']) <= .25)
    bounded = bool(states) and all(np.linalg.norm(r['v']) <= 7. for r in states)
    displacement = max([r['p'][2]-states[0]['p'][2] for r in states], default=0.)
    return dict(passed=bool(len(states) >= 5 and body_only and formal_valid and stopped and bounded
                            and response > 0
                            and braking > 0 and states[-1]['stamp']-states[0]['stamp'] >= .1),
                native_samples=len(states), body_rate_only=body_only, stopped=stopped,
                motion_bounded=bounded, formal_controller_replayed=bool(formal_valid),
                initial_speed=initial, response_seconds=response,
                reliable_deceleration=braking, maximum_downward_displacement=displacement)


def native_land_takeover(rows):
    """One LAND request must produce native takeover, motor response, landed/disarmed."""
    requests = [r for r in rows if r['kind'] == 'vehicle_command' and r.get('command') == 21]
    status = [r for r in rows if r['kind'] == 'wire' and r.get('phase') == 'LAND'
              and r['topic'] == 'vehicle_status']
    landed = any(r['kind'] == 'wire' and r.get('phase') == 'LAND'
                 and r['topic'] == 'vehicle_land_detected' and r['values'].get('landed')
                 for r in rows)
    actual = [r for r in status if r['values'].get('nav_state') == 18]
    thrust = [r for r in rows if r['kind'] == 'executed_collective' and r.get('phase') == 'LAND']
    motors = [r for r in rows if r['kind'] == 'wire' and r.get('phase') == 'LAND'
              and r['topic'] == 'actuator_motors']
    motor_trace = []
    for row in motors:
        values = np.asarray(row['values'].get('control', []), dtype=float)
        finite = values[np.isfinite(values)]
        if len(values):
            motor_trace.append(float(np.max(np.abs(finite))) if len(finite) else 0.)
    motor_response = bool(len(motor_trace) >= 2 and max(motor_trace) > .1
                          and motor_trace[-1] <= .1 and np.ptp(motor_trace) > .01)
    states = [r for r in rows if r['kind'] == 'state' and r.get('phase') == 'LAND'
              and 0 <= r.get('age', 1.) <= .125]
    landing_motion = bool(len(states) >= 5 and any(r['v'][2] > .1 for r in states)
                          and np.linalg.norm(states[-1]['v']) <= .25)
    response = (actual[0]['stamp']-requests[0]['stamp'] if requests and actual else None)
    changing = bool(len(thrust) >= 3 and np.ptp([r['xyz'][2] for r in thrust]) > .01)
    disarmed = bool(status and status[-1]['values'].get('arming_state') == 1)
    no_external = not any(r['kind'] == 'command' and r.get('phase') == 'LAND' for r in rows)
    passed = (len(requests) == 1 and response is not None and 0 <= response <= 1.
              and changing and motor_response and landing_motion
              and landed and disarmed and no_external)
    return dict(passed=bool(passed), requests=len(requests), response_seconds=response,
                thrust_response=changing, native_motor_samples=len(motors),
                actual_motor_response=motor_response, landing_motion=landing_motion,
                landed=landed, disarmed=disarmed, no_external_commands=no_external)


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
    body_commands = [i for i, r in enumerate(rows) if r['kind'] == 'command'
                     and r.get('interface') == 'body_rate']
    commands_after = ([r for r in rows[body_commands[0]:] if r['kind'] == 'command']
                      if body_commands else [])
    measured_modes = [r for r in mode_rows if r.get('phase', '').startswith(
        ('THRUST_', 'RATE_', 'BODY_BRAKE', 'BODY_HOLD', 'DESCENT_', 'BRAKE_', 'FAULT_'))
        or r.get('phase') == 'MANEUVER']
    exclusive = bool(measured_modes and commands_after) and all(
        r.get('interface') == 'body_rate' for r in commands_after) and all(
        r.get('flags', {}).get('offboard') and r['flags'].get('rates')
        and not any(r['flags'].get(k) for k in (
            'position', 'velocity', 'attitude', 'altitude', 'climb_rate'))
        for r in measured_modes)
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
    formal = formal_recovery_evidence(rows)
    braking = [bounded_response(rows, f'BRAKE_{speed}', formal) for speed in (.5, 1., 2.)]
    checks['descent_2mps_braking'] = (
        len(brakes) == 3 and all(b['passed'] for b in braking)
        and all(b['initial_speed'] >= .95*speed
                for b, speed in zip(braking, (.5, 1., 2.))))
    recovery = bounded_response(rows, 'FAULT_RECOVERY', formal)
    checks['bodyrate_recovery'] = recovery['passed'] and recovery['initial_speed'] >= .5
    land = native_land_takeover(rows)
    checks['native_land_takeover'] = land['passed']
    checks = {key: bool(value) for key, value in checks.items()}
    if not monotone:
        reasons.append('INSUFFICIENT_OR_NONMONOTONE_MEASURED_FORCE_CURVE')
    hover_u = [-r['xyz'][2] for r in rows
               if r['kind'] == 'executed_collective' and r['phase'] == 'HOVER']
    return dict(schema_version=SCHEMA_VERSION, architecture=ARCHITECTURE,
                hover_normalized=float(np.median(hover_u)) if hover_u else None,
                hover_samples=len(hover),
                scope='Motion-derived effective lift; includes aerodynamic-model uncertainty',
                A1_passed=all(checks[k] for k in A1_REQUIRED), checks=checks, reasons=reasons,
                bodyrate_recovery=recovery, native_land_takeover=land, formal_recovery=formal,
                native_force_samples=len(samples), measured_curve=[dict(normalized=u, force=f)
                                                                   for u, f in curve],
                maximum_effective_thrust=maximum, measured_twr=maximum/(mass*gravity),
                hover_relative_error=hover_error if hover else None, rate_tracking=rates,
                response_seconds=max(
                    [r['response_seconds'] for r in braking+[recovery]], default=0.),
                braking_acceleration=min(
                    [r['reliable_deceleration'] for r in braking+[recovery]], default=0.))


def main():
    """Analyze immutable recorded rows using the session model's measured mass identity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    metadata = json.loads((args.session/'identity.json').read_text())
    rows = [json.loads(line)
            for line in (args.session/'calibration.jsonl').read_text().splitlines()]
    result = analyze_rows(rows, metadata['model']['total_mass'], metadata['model']['gravity'])
    target = args.session/'analysis_v2.json'
    if target.exists():
        raise ValueError('P763_NEVER_OVERWRITE_ANALYSIS')
    target.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

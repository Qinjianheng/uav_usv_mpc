#!/usr/bin/env python3
"""E0 replay of immutable A1_03 native samples; no PX4/Gazebo and no qualification."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_body_geometry import (  # noqa: E402
    base_reference_flu, navigation_vertices, lowest_world_z,
)
from uav_control.common.p76_contract import load_contract  # noqa: E402
from uav_control.control.p763_calibration_control import (  # noqa: E402
    CalibrationBodyHold, load_bootstrap, pulse_admission,
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(session):
    """The known late exit is unsafe even with immediate explicit command replacement."""
    rows = [json.loads(line) for line in (session/'calibration.jsonl').read_text().splitlines()]
    identity = json.loads((session/'identity.json').read_text())
    model = identity['model']
    begin = next(r['stamp'] for r in rows if r['kind'] == 'phase'
                 and r['phase'] == 'THRUST_1.000')
    end = next(r['stamp'] for r in rows if r['kind'] == 'phase'
               and r['phase'] == 'RESET' and r['stamp'] > begin)
    native = [r for r in rows if r['kind'] == 'state']
    start = max((r for r in native if r['stamp'] <= begin), key=lambda r: r['stamp'])
    exit_state = next(r for r in native if r['phase'] == 'RESET' and r['stamp'] >= end)
    response = next(r for r in rows if r['kind'] == 'executed_collective'
                    and r['stamp'] >= end and -r['xyz'][2] < .95)
    exceed = next(r for r in native if r['stamp'] >= end and np.linalg.norm(r['v']) > 7.)
    bootstrap = load_bootstrap(session, identity)
    contract = load_contract(identity['contract_file'])
    model_root = Path(contract['dynamics']['px4_model_root'])
    base_file = model_root/'x500_base/model.sdf'
    if digest(base_file) != model['files']['x500_base/model.sdf']:
        raise ValueError('P763_E0_NATIVE_GEOMETRY_IDENTITY_MISMATCH')
    points = navigation_vertices(model['collision_envelope'], model_root)
    base_height = base_reference_flu(model_root)[2, 3]
    poses = {r['stamp']: r for r in rows if r['kind'] == 'pose'}

    def geometry(state):
        pose = poses[state['stamp']]
        rotation = Rotation.from_quat(np.asarray(pose['q'])[[1, 2, 3, 0]]).as_matrix()
        reference = max((r for r in rows if r['kind'] == 'reference'
                         and r['stamp'] <= state['stamp']), key=lambda r: r['stamp'])
        sea = reference['ref_alt']-contract['candidate']['sea_altitude_msl']
        home_z = sea-8.-base_height
        return (rotation, state['p'][2]-(home_z-4.),
                sea-lowest_world_z(points, state['p'], rotation))

    initial_rotation, initial_room, initial_clearance = geometry(start)
    exit_rotation, exit_room, exit_clearance = geometry(exit_state)
    parameters = dict(mass=model['total_mass'], maximum_thrust=model['maximum_thrust'],
                      gravity=model['gravity'],
                      upward_braking=model['gravity']-bootstrap.force[0]/model['total_mass'],
                      downward_braking=bootstrap.force[-1]/model['total_mass']-model['gravity'])
    preflight = pulse_admission(velocity=start['v'], acceleration=start['a'],
                                sample_age=start['age'], duration=.22,
                                clearance=initial_clearance, upward_room=initial_room,
                                **parameters)
    late = pulse_admission(velocity=exit_state['v'], acceleration=exit_state['a'],
                           sample_age=exit_state['age'], duration=0.,
                           clearance=exit_clearance, upward_room=exit_room, **parameters)
    hold = CalibrationBodyHold(model['total_mass'], model['gravity'], bootstrap)
    hold.initialize(poses[exit_state['stamp']]['omega'], 1.)
    initial_yaw = float(Rotation.from_matrix(initial_rotation).as_euler('xyz')[2])
    command = hold.step(start['p'], exit_state['p'], exit_state['v'], exit_rotation, .005,
                        initial_yaw, brake=True)
    return dict(schema_version=2, stage='E0',
                outcome='SAFE_REFUSAL' if not preflight['admitted'] else 'UNSAFE_ADMISSION',
                passed=(not preflight['admitted'] and not late['admitted']
                        and command.thrust < .503),
                qualified=False,
                scope=('Offline conservative admission/command logic only; '
                       'no actual motor validation'),
                source_session=str(session), raw_sha256=digest(session/'calibration.jsonl'),
                geometry=dict(source=str(base_file), sha256=digest(base_file),
                              native_full_attitude=True, native_reference_datum=True),
                old=dict(pulse_requested=1., pulse_duration_nominal=.22,
                         pulse_duration_observed=end-begin,
                         exit_native_upward_speed=-exit_state['v'][2],
                         exit_native_timestamp_sample=exit_state['native_timestamp_sample'],
                         exit_native_sample_after_command=exit_state['stamp']-end,
                         measured_collective_response_delay=response['stamp']-end,
                         first_exceed_speed=float(np.linalg.norm(exceed['v'])),
                         first_exceed_native_timestamp_sample=exceed['native_timestamp_sample'],
                         limit=7.),
                guarded=dict(admission=preflight, late_exit=late,
                             explicit_body_brake_thrust=command.thrust,
                             explicit_body_brake_rates=command.rates.tolist(),
                             maximum_pulse_executed=False,
                             explanation=('Reject before pulse; .24s worst delay '
                                          'makes late exit unsafe')),
                dynamics=dict(static_model_twr=model['static_model_twr'],
                              short_window_effective_twr=bootstrap.force[-1]/(
                                  model['total_mass']*model['gravity']),
                              qualification_threshold=2.85,
                              steady_state_maximum='UNKNOWN'),
                blockers=['Fixed .22s maximum pulse cannot be guaranteed inside original envelope',
                          'Actual Body-rate brake/hold and motor response remain unqualified'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, default=ROOT/'data/experiments/'
                        '20261021_p761_01/session_A1_03')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = replay(args.session.resolve())
    target = args.output/'exit_replay.json'
    target.write_text(json.dumps(result, indent=2)+'\n')
    (args.output/'SHA256SUMS').write_text(f'{digest(target)}  exit_replay.json\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

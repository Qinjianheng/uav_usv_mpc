#!/usr/bin/env python3
"""Audit immutable A1 dynamics and five identification families without flying."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.control.p763_calibration_control import pulse_admission  # noqa: E402
from p76_calibration_analysis import native_force_samples  # noqa: E402


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fit_motor_response(times, forces, initial_force, maximum_force):
    """Conditional speed first-order/squared-force fit; uncertainty is not a safety bound.

    F(t)=[sqrt(Finf)+(sqrt(F0)-sqrt(Finf))*exp(-max(0,t-d)/tau)]^2.
    Unknown delay and steady force compete with tau in a short, sparse transient.
    """
    t, force = np.asarray(times, float), np.asarray(forces, float)
    empty = dict(status='INSUFFICIENT_EXCITATION_OR_SAMPLES', qualified=False,
                 physical_error_bound=None, actual_samples=len(t))
    if (len(t) < 5 or len(t) != len(force) or not np.isfinite(t).all()
            or not np.isfinite(force).all() or np.ptp(force) < .05):
        return empty

    def residual(parameters):
        steady, tau, delay = parameters
        speed = np.sqrt(steady)+(np.sqrt(max(initial_force, 0.))-np.sqrt(steady))*np.exp(
            -np.maximum(0., t-delay)/tau)
        return speed**2-force
    fit = least_squares(residual, [np.clip(force[-1], .01, maximum_force-.01), .03, .001],
                        bounds=([.001, .0001, 0.], [maximum_force, 2., .22]),
                        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=2000,
                        x_scale=[maximum_force, .03, .03])
    singular = np.linalg.svd(fit.jac, compute_uv=False)
    rank = int(np.linalg.matrix_rank(fit.jac))
    condition = float(singular[0]/singular[-1]) if singular[-1] > 1e-15 else None
    residual_std = float(np.sqrt(np.sum(fit.fun**2)/max(1, len(t)-3)))
    stderr = (np.sqrt(np.maximum(0., np.diag(np.linalg.pinv(fit.jac.T@fit.jac))))
              * residual_std).tolist()
    return dict(status='CONDITIONAL_FIT_ONLY', actual_samples=len(t),
                steady_force_newtons=float(fit.x[0]), speed_time_constant_seconds=float(fit.x[1]),
                input_to_force_fit_delay_seconds=float(fit.x[2]), jacobian_rank=rank,
                jacobian_condition=condition, residual_rms_newtons=float(np.sqrt(np.mean(
                    fit.fun**2))), conditional_parameter_standard_errors=stderr,
                conditional_uncertainty_scope=(
                    'Local iid residual assumption only; not confidence bounds'),
                fitted_at_parameter_boundary=bool(np.any(fit.active_mask)),
                physical_error_bound=None, qualified=False)


def motor_ulog(session, phases):
    """Native actuator setpoints/outputs are not measured rotor RPM or measured force."""
    from pyulog import ULog
    path = session/'native_px4.ulg'
    ulog = ULog(str(path))
    datasets = {d.name: d for d in ulog.data_list}
    esc = datasets['esc_status'].data
    bridge = Path('/home/qin/Projects/PX4-Autopilot/src/modules/simulation/gz_bridge/'
                  'GZMixingInterfaceESC.cpp')
    result = dict(source=str(path), sha256=digest(path), datasets=sorted(datasets),
                  esc_rpm_fields_present=True, measured_rotor_rpm_present=False,
                  esc_status_rows=len(esc['timestamp']),
                  esc_nonzero_rows_by_motor=[
                      int(np.count_nonzero(esc[f'esc[{i}].esc_rpm'])) for i in range(4)],
                  esc_feedback_scope=(
                      'GZ bridge subscribes to its own command/motor_speed and copies velocity '
                      'into esc_rpm; not independent rotor feedback'),
                  inspected_bridge_source=dict(path=str(bridge), sha256=digest(bridge),
                                               historical_source_binary_binding=False),
                  actuator_channels_scope=(
                      'actuator_motors: normalized allocation; actuator_outputs: motor-speed '
                      'commands; neither is an independent rotor-speed measurement'),
                  per_phase=[])
    for phase in phases:
        row = dict(phase=phase['phase'])
        for name, prefix in (('actuator_motors', 'control'), ('actuator_outputs', 'output')):
            data = datasets[name].data
            stamp = data.get('timestamp_sample', data['timestamp'])
            mask = (stamp >= phase['native_start_us']) & (stamp < phase['native_end_us'])
            values = np.column_stack([data[f'{prefix}[{i}]'][mask] for i in range(4)])
            row[name] = dict(count=len(values),
                             channel_median=(np.median(values, axis=0).tolist()
                                             if len(values) else []),
                             channel_min=np.min(values, axis=0).tolist() if len(values) else [],
                             channel_max=np.max(values, axis=0).tolist() if len(values) else [])
        result['per_phase'].append(row)
    return result


def analyze_session(session):
    """Keep original sample epochs, source identity, unqualified force curve and refusal."""
    session = Path(session).resolve()
    rows = [json.loads(line) for line in (session/'calibration.jsonl').read_text().splitlines()]
    identity = json.loads((session/'identity.json').read_text())
    model = identity['model']
    model_root = session.parent/'frozen/models'
    for name, expected in model['files'].items():
        if digest(model_root/name) != expected:
            raise ValueError('P765_HISTORICAL_MODEL_IDENTITY_MISMATCH')
    motors = [p for p in ET.parse(model_root/'x500/model.sdf').findall('.//plugin')
              if p.find('motorConstant') is not None]
    maximum = sum(float(p.findtext('motorConstant'))*float(p.findtext('maxRotVelocity'))**2
                  for p in motors)
    if abs(maximum-model['maximum_thrust']) > 1e-8:
        raise ValueError('P765_STATIC_FORCE_IDENTITY_MISMATCH')
    samples = native_force_samples(rows, model['total_mass'], model['gravity'])
    samples = list({r['native_timestamp_sample']: r for r in samples}.values())
    phase_rows = [r for r in rows if r['kind'] == 'phase']
    state_rows = sorted((r for r in rows if r['kind'] == 'state'), key=lambda r: r['stamp'])
    offset = np.median([r['stamp']-r['native_timestamp_sample']*1e-6 for r in state_rows])
    phase_reports = []
    for i, phase in enumerate(phase_rows):
        if not phase['phase'].startswith('THRUST_'):
            continue
        begin, end = phase['stamp'], phase_rows[i+1]['stamp']
        command = next(r for r in rows if r['kind'] == 'command'
                       and begin <= r['stamp'] < end and r.get('interface') == 'body_rate')
        matched = [r for r in rows if r['kind'] == 'executed_collective'
                   and command['stamp'] <= r['stamp'] < end
                   and abs(-r['xyz'][2]-command['thrust']) <= .005]
        segment = sorted((r for r in samples if begin <= r['stamp'] < end),
                         key=lambda r: r['stamp'])
        subset = [r for r in segment if r['phase_elapsed'] >= .08 and r['tilt'] < .12]
        before = [r['force'] for r in samples if begin-.08 <= r['stamp'] < begin]
        forces = [r['force'] for r in subset]
        median = float(np.median(forces)) if forces else None
        response_segment = [r for r in segment if r['stamp'] >= command['stamp']]
        fit = fit_motor_response([r['stamp']-command['stamp'] for r in response_segment],
                                 [r['force'] for r in response_segment],
                                 float(np.median(before)), maximum)
        phase_reports.append(dict(phase=phase['phase'], normalized=float(phase['phase'][7:]),
                                  duration_seconds=end-begin,
                                  native_start_us=round((begin-offset)*1e6),
                                  native_end_us=round((end-offset)*1e6),
                                  all_native_force_samples=len(segment),
                                  original_warmup_force_samples=len(subset),
                                  force_median_newtons=median,
                                  force_min_newtons=min(forces) if forces else None,
                                  force_max_newtons=max(forces) if forces else None,
                                  force_MAD_newtons=float(np.median(np.abs(
                                      np.asarray(forces)-median))) if forces else None,
                                  command_stamp=command['stamp'],
                                  executed_collective_matches_command_seconds=(
                                      matched[0]['stamp']-command['stamp'] if matched else None),
                                  same_mode_physical_response_qualified=False,
                                  motor_fit=fit, steady_state_verified=False))
    highest = phase_reports[-1]
    begin = next(r['stamp'] for r in phase_rows if r['phase'] == 'THRUST_1.000')
    end = next(r['stamp'] for r in phase_rows if r['phase'] == 'RESET' and r['stamp'] > begin)
    response = next(r for r in rows if r['kind'] == 'executed_collective'
                    and r['stamp'] >= end and -r['xyz'][2] < .95)
    reset_commands = [r for r in rows if r['kind'] == 'command'
                      and end <= r['stamp'] < end+.25]
    epochs = sorted({r['native_timestamp_sample'] for r in state_rows})
    period = float(np.max(np.diff(epochs)))*1e-6
    inputs = dict(velocity=[0., 0., 0.], acceleration=[0., 0., 0.], sample_age=.125,
                  mass=model['total_mass'], maximum_thrust=maximum, gravity=model['gravity'],
                  upward_room=4., clearance=8., upward_braking=4., downward_braking=4.)
    definitions = [
        ('fixed_seven_022s', .22, ['hover', 'transient local force curve'],
         'Seven full-envelope pulses; historical execution failed before rate/maneuver phases'),
        ('progressive_steps', .10, ['local force increments'],
         'Seven predeclared incremental levels, full settled reset between trials'),
        ('independent_short_pulses', .04, ['transient impulse integral'],
         'Independent 40ms pulses followed by complete native-state settling'),
        ('small_signal', .20, ['hover-local gain and conditional local time constant'],
         'Hover +/-0.02 normalized input; no inference of maximum force'),
        ('motor_model_fit', .22, ['conditional speed tau/delay/force parameters'],
         'Fit only existing transients; do not execute a new pulse'),
    ]
    protocols = []
    for index, (name, duration, observables, definition) in enumerate(definitions):
        bound = pulse_admission(duration=duration, **inputs)
        protocols.append(dict(name=name, definition=definition,
                              required_input_window_seconds=duration,
                              maximum_delivered_native_interval_seconds=period,
                              nominal_post_80ms_sample_capacity=max(
                                  0, int(np.floor((duration-.08)/period))+1),
                              actual_new_protocol_force_samples=sum(
                                  r['original_warmup_force_samples'] for r in phase_reports)
                              if index == 0 else 0,
                              historical_samples_available_for_offline_fit=sum(
                                  r['all_native_force_samples'] for r in phase_reports),
                              force_upper_bound_newtons=maximum,
                              safety='NOT_PROVED' if index == 4 else 'REJECTED_BY_RETAINED_BOUND',
                              retained_guard=bound, conditional_identifiable=observables,
                              uncertainty='Physical input/force/model error bound UNKNOWN',
                              maximum_steady_thrust_identifiable=False,
                              rates_response_identifiable=False, braking_identifiable=False,
                              original_A1_equivalent=False, authorized_to_execute=False))
    hover = [r for r in samples if r['phase'] == 'HOVER' and np.linalg.norm(r['v']) < .25]
    static = dict(mass=model['total_mass'], gravity=model['gravity'], maximum_thrust=maximum,
                  twr=maximum/(model['total_mass']*model['gravity']),
                  scope='Verified SDF static available motor-force ceiling; not measured TWR',
                  motor_speed_time_constants_seconds=[
                      dict(up=float(p.findtext('timeConstantUp')),
                           down=float(p.findtext('timeConstantDown'))) for p in motors])
    names = ('identity.json', 'calibration.jsonl', 'analysis.json', 'native_px4.ulg',
             'p76_session_model.json')
    paths = [session/name for name in names]
    paths += [model_root/name for name in model['files']]
    paths += [Path(__file__), ROOT/'scripts/p76_calibration_analysis.py',
              ROOT/'src/uav_control/uav_control/control/p763_calibration_control.py']
    return dict(schema_version=1, stage='E0_DYNAMICS_METHOD_AUDIT',
                source_inputs={str(p): digest(p) for p in paths}, static_model=static,
                mode_exit=dict(scope='BODY_RATE_TO_VELOCITY_WITHDRAWAL_ONLY',
                               executed_collective_withdrawal_seconds=response['stamp']-end,
                               reset_command_interfaces=sorted(
                                   {r['interface'] for r in reset_commands}),
                               within_bodyrate_response_bound='UNKNOWN',
                               used_240ms_as_measured_within_mode_delay=False),
                local_data=dict(force_samples=len(samples), hover_force_samples=len(hover),
                                force_scope=(
                                    'Motion-derived lift including aero/navigation uncertainty'),
                                phase_samples=phase_reports,
                                maximum_short_window_twr=highest['force_median_newtons']/(
                                    model['total_mass']*model['gravity']),
                                maximum_steady_twr='UNKNOWN', physical_force_error_bound=None),
                per_level_upper_bounds=dict(
                    proved=False, retained_upper_bound_newtons=maximum,
                    reason='No independent measured RPM or qualified allocation/lag/error bound'),
                motor_ulog=motor_ulog(session, phase_reports), protocol_comparison=protocols,
                decision=dict(E0='BLOCKED', A1='NO-GO', maximum_force_unknown=True,
                              qualification_definition_change_required=True,
                              reason=('Local/transient identification cannot replace original '
                                      'maximum/rate/maneuver A1'),
                              next_minimum_experiment=(
                                  'Defer dynamics experiments; the single next-session proposal '
                                  'is a separately approved 30s unarmed ROI/timing collection '
                                  'to resolve zero valid 3D observations and E1'),
                              possible_future_identification_method=(
                                  'Restrained thrust-stand identification with native input, '
                                  'allocation, independent RPM/force and uncertainty; requires '
                                  'separate design and authorization; '
                                  'not a current recommendation')),
                assumption_audit=dict(retained_guard_delay_seconds=.24,
                                      measured_within_mode_delay_bound=False,
                                      retained_guard_braking_mps2=4., measured_braking_bound=False,
                                      scope=('Conservative engineering reserves do not prove '
                                             'actual A1 safety')))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, default=ROOT/'data/experiments/'
                        '20261021_p761_01/session_A1_03')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = analyze_session(args.session)
    target = args.output/'dynamics_analysis.json'
    target.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'SHA256SUMS').write_text(f'{digest(target)}  {target.name}\n')
    print(json.dumps(report['decision'], indent=2))
    return 2


if __name__ == '__main__':
    raise SystemExit(main())

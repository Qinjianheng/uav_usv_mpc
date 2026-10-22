#!/usr/bin/env python3
"""Fixed offline E2 diagnosis; no gains, safety limits, runtime or paper-law changes."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.common.p76_contract import load_contract  # noqa: E402
from uav_control.common.p76_readiness import sha256  # noqa: E402
from uav_control.control.p76_minco_bodyrate_tracker import WaypointReference  # noqa: E402
from p764_handover_replay import engineering_tracker, control_checks, handover_replay  # noqa: E402

CANDIDATES = ('baseline', 'reference_governor_10s', 'cv_prediction_120ms')
LAGS = (.02, .12, .24, .4, .44, .5)


def linear_stability(contract):
    """Routh-Hurwitz for the complete unsaturated horizontal attitude/rate chain.

    p'=v, v'=g*tilt, tilt'=omega,
    tau*omega'=-omega+ka*((kp*(pr-p)+kv*(vr-v))/g-tilt).
    Its characteristic is tau*s^4+s^3+ka*s^2+ka*kv*s+ka*kp.
    This local continuous-time model excludes jerk/slew, force/tilt coupling and PX4.
    """
    s = contract['bodyrate_follow']
    kp, kv, ka = (s[k] for k in ('kp', 'kv', 'attitude_gain'))
    bound = min(1./kv, (ka*kv-kp)/(ka*kv*kv))
    rows = []
    for lag in LAGS:
        roots = np.roots([lag, 1., ka, ka*kv, ka*kp])

        def discrete_step(state):
            # Exact order of the P764 semi-implicit plant update at small signal.
            p, v, tilt, rate = state
            rate += .005*(-ka*(kp*p+kv*v)/9.8-ka*tilt-rate)/lag
            tilt += .005*rate
            v += .005*9.8*tilt
            p += .005*v
            return [p, v, tilt, rate]
        matrix = np.array([discrete_step(x) for x in np.eye(4)]).T
        rows.append(dict(rate_time_constant_seconds=lag,
                         maximum_pole_real=float(max(roots.real)),
                         stable=bool(max(roots.real) < 0),
                         numerical_5ms_spectral_radius=float(max(abs(np.linalg.eigvals(matrix))))))
    return dict(strict_rate_time_constant_bound_seconds=bound,
                characteristic_coefficients_at_500ms=[.5, 1., ka, ka*kv, ka*kp],
                poles=rows, strict_bound_excludes_equality=True,
                scope='Local continuous-time horizontal small signal; no constrained flight proof',
                physical_qualification=False)


def response_case(contract, lag, candidate='baseline', *, rate_lag=True, force_lag=True):
    """Reproduce P764 integration, recording causal limit and plant-error diagnostics.

    Two predeclared offline candidates retain every gain and output limit. The governor
    replaces the discontinuous hold reference by a ten-second quintic. CV prediction is
    only a synthetic sensitivity test, never a runtime freshness/extrapolation change.
    Neither candidate has MINCO lease/obstacle/FOV requalification.
    """
    if candidate not in CANDIDATES or lag not in LAGS:
        raise ValueError('P765_PREDECLARED_CASE_REQUIRED')
    tracker = engineering_tracker(contract)
    p, v, omega = np.array([-1., .5, .5]), np.zeros(3), np.zeros(3)
    initial = p.copy()
    rotation, force, maximum_error, maximum_speed = np.eye(3), 19.6, 0., 0.
    dt, counts, trace = .005, Counter(), []
    maximum_rate_error, maximum_force_error, bounds = 0., 0., True
    settings = contract['bodyrate_follow']
    governor = WaypointReference(0., initial, np.zeros(3), np.zeros(3), 10., 0.)
    previous_rate, previous_thrust = np.zeros(3), -.5
    for index in range(4000):
        ref = governor.sample(index*dt) if candidate == 'reference_governor_10s' else np.zeros(11)
        feedback_p = p+.12*v if candidate == 'cv_prediction_120ms' else p
        raw = ref[6:9]+settings['kp']*(ref[:3]-feedback_p)+settings['kv']*(ref[3:6]-v)
        limited = raw*min(1., settings['maximum_acceleration']/max(np.linalg.norm(raw), 1e-12))
        tilt_bound = max(0., 9.8-limited[2])*np.tan(settings['maximum_tilt'])
        limited[:2] *= min(1., tilt_bound/max(np.linalg.norm(limited[:2]), 1e-12))
        counts['acceleration_limited_samples'] += np.linalg.norm(raw) > 5.+1e-9
        counts['jerk_limited_samples'] += (
            np.linalg.norm(limited-tracker.previous_acceleration) > 3.*dt+1e-9)
        out = tracker.step(ref, feedback_p, v, rotation, dt, omega)
        dr = np.linalg.norm(out['rates']-previous_rate)
        du = abs(out['thrust']-previous_thrust)
        counts['rate_slew_boundary_samples'] += dr >= 3.*dt-1e-9
        counts['thrust_slew_boundary_samples'] += du >= 2.*dt-1e-9
        bounds &= bool(dr <= 3.*dt+1e-9 and du <= 2.*dt+1e-9
                       and np.linalg.norm(out['acceleration']) <= 5.+1e-9
                       and np.all(np.abs(out['rates']) <= tracker.limiter.bound+1e-9)
                       and -1 <= out['thrust'] < 0)
        previous_rate, previous_thrust = out['rates'].copy(), out['thrust']
        if rate_lag:
            omega += dt*(out['rates']-omega)/lag
        else:
            omega = out['rates'].copy()
        requested = np.interp(-out['thrust'], [0., .5, 1.], [0., 19.6, 60.])
        force = force+dt*(requested-force)/lag if force_lag else requested
        maximum_rate_error = max(maximum_rate_error, float(np.linalg.norm(out['rates']-omega)))
        maximum_force_error = max(maximum_force_error, float(abs(requested-force)))
        rotation = rotation@Rotation.from_rotvec(omega*dt).as_matrix()
        acceleration = np.array([0., 0., 9.8])-rotation[:, 2]*force/2.
        v += dt*acceleration
        p += dt*v
        maximum_error = max(maximum_error, float(np.linalg.norm(p)))
        maximum_speed = max(maximum_speed, float(np.linalg.norm(v)))
        if index % 200 == 199:
            trace.append(dict(seconds=(index+1)*dt, error=float(np.linalg.norm(p)),
                              horizontal_error=float(np.linalg.norm(p[:2])),
                              vertical_error=abs(float(p[2])), speed=float(np.linalg.norm(v))))
    prediction_kv = .12*settings['kp'] if candidate == 'cv_prediction_120ms' else 0.
    effective_kv = settings['kv']+prediction_kv
    local_poles = np.roots([lag, 1., settings['attitude_gain'],
                           settings['attitude_gain']*effective_kv,
                           settings['attitude_gain']*settings['kp']])
    return dict(candidate=candidate, lag_seconds=lag, rate_lag=rate_lag, force_lag=force_lag,
                lag_model='FIRST_ORDER_TIME_CONSTANT_NOT_TRANSPORT_DELAY',
                duration_seconds=20., dt_seconds=dt, initial_error=float(np.linalg.norm(initial)),
                final_error=float(np.linalg.norm(p)), final_speed=float(np.linalg.norm(v)),
                final_horizontal_error=float(np.linalg.norm(p[:2])),
                final_vertical_error=abs(float(p[2])), maximum_error=maximum_error,
                maximum_speed=maximum_speed, passed=bool(np.linalg.norm(p) < .15),
                acceptance=dict(final_error_strictly_below_m=.15, unchanged=True),
                output_bounds_passed=bool(bounds), settings=dict(settings),
                diagnostics=dict({k: int(value) for k, value in counts.items()},
                                 total_samples=4000,
                                 maximum_rate_follow_error=maximum_rate_error,
                                 maximum_force_follow_error_newtons=maximum_force_error,
                                 reference_feedback_epoch_mismatch_seconds=0.),
                trace_1hz=trace, minco_feasibility_proved=False, flight_qualified=False,
                continuous_local_rate_chain_maximum_pole_real=(
                    float(max(local_poles.real)) if rate_lag else None),
                transient_acceptance_is_stability_proof=False)


def analyze(contract):
    """Finite declared cases and ablations; never select gains or delete failed stress."""
    original = control_checks(contract)
    cases = [response_case(contract, lag) for lag in LAGS]
    candidates = [response_case(contract, .5, candidate=c) for c in CANDIDATES[1:]]
    ablations = [response_case(contract, .5, rate_lag=rate, force_lag=force)
                 for rate, force in ((True, False), (False, True), (False, False))]
    return dict(schema_version=1, stage='E2_OFFLINE_DIAGNOSIS',
                declaration=dict(lag_time_constants_seconds=LAGS, candidates=CANDIDATES,
                                 no_gain_search=True, limits_unchanged=True,
                                 candidate_selection_from_results=False),
                original_fixed_checks=original, linear_stability=linear_stability(contract),
                baseline_cases=cases, fixed_candidate_cases=candidates,
                actuator_ablations=ablations, handover=handover_replay(contract),
                decision=dict(E2='FAIL', stress_500ms_retained=True, flight_qualified=False,
                              largest_tested_passing_time_constant_seconds=max(
                                  (r['lag_seconds'] for r in cases if r['passed']), default=None),
                              largest_tested_case_inside_continuous_local_stability_bound=max(
                                  (r['lag_seconds'] for r in cases if r['passed']
                                   and r['continuous_local_rate_chain_maximum_pole_real'] < 0),
                                  default=None),
                              physical_delay_bound='UNKNOWN',
                              continuous_nonlinear_stability_interval='NOT_PROVED'),
                limitations=['Point-mass plant and ideal synthetic qualified map only',
                             'No transport/feedback delay or measured PX4 actuator fit',
                             'No sea/obstacle/FOV/MINCO proof for candidate references',
                             'Recovery timing is not exercised by this hold-response model'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    contract_file = ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'
    report = analyze(load_contract(contract_file))
    paths = [Path(__file__), contract_file, ROOT/'scripts/p764_handover_replay.py']
    paths += list((ROOT/'src/uav_control/uav_control/control').glob('p76*.py'))
    report['source_inputs'] = {str(p): sha256(p) for p in paths}
    target = args.output/'e2_analysis.json'
    target.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'SHA256SUMS').write_text(f'{sha256(target)}  {target.name}\n')
    print(json.dumps(report['decision'], indent=2))
    return 2  # Deliberately preserve the fixed stress failure.


if __name__ == '__main__':
    raise SystemExit(main())

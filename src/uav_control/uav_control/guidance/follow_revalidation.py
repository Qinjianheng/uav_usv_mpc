"""Prediction policy B: full nominal curve revalidation, without execution permission."""
from uav_control.guidance.follow_profile import profiled

from dataclasses import replace
import math
import time

import numpy as np

from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.polynomial_bounds import derivative_upper_bound
from uav_control.guidance.polynomial_extrema import derivative_peak


@profiled('prediction_revalidation')
def revalidate_prediction(request, output, prediction, now, model, budget=.03,
                          clock=time.perf_counter, cache=None):
    """
    Retain raw navigation and the exact curve; replace only explicit prediction inputs.

    Adaptive whole-sphere FOV checks are nominal sampling. Analytic/Bernstein
    derivative checks cover continuous dynamics. Neither grants holding authority.
    """
    started = clock()
    report = dict(valid=False, holding_qualified=False, samples=0,
                  old_sequence=request.context.prediction_sequence_id,
                  new_sequence=prediction.sequence_id)
    try:
        c = request.context
        if prediction.mission_id != c.mission_id:
            raise ValueError('MISSION_CHANGED')
        if (not math.isfinite(now) or now < request.now_stamp
                or not math.isfinite(budget) or budget <= 0
                or now >= c.execution_start_stamp):
            raise ValueError('EXECUTION_START_MISSED')
        if (not math.isfinite(prediction.generated_stamp) or prediction.generated_stamp <= 0
                or prediction.generated_stamp < prediction.source_stamp):
            raise ValueError('INVALID_PREDICTION')
        if prediction.generated_stamp > now:
            raise ValueError('PREDICTION_FUTURE')
        if (prediction.sequence_id < c.prediction_sequence_id
                or prediction.source_stamp < c.prediction_source_stamp
                or prediction.observation_stamp < c.observation_stamp):
            raise ValueError('PREDICTION_OUT_OF_ORDER')
        context = replace(c, prediction_sequence_id=prediction.sequence_id,
                          prediction_source_stamp=prediction.source_stamp,
                          observation_stamp=prediction.observation_stamp,
                          prediction_valid_until=prediction.valid_until,
                          prediction_source=prediction.source, frame_id=prediction.frame_id)
        updated = replace(request, context=context, now_stamp=now,
                          prediction_times=prediction.prediction_times,
                          target_positions=prediction.target_positions,
                          target_velocities=prediction.target_velocities)
        metrics = output.metrics
        if cache is not None:
            from uav_control.guidance.follow_fast_validation import assess_curve
            if now >= min(c.navigation_stamp+.125, c.attitude_stamp+.125,
                          prediction.source_stamp+.125, prediction.observation_stamp+.125,
                          prediction.valid_until):
                raise ValueError('INPUT_EXPIRED')
            checked, _ = assess_curve(updated, metrics, model, cache,
                                      budget-(clock()-started), clock)
            report.update(checked)
            return updated, dict(report, elapsed=clock()-started)
        ts = np.asarray(metrics['durations'])
        xyz = np.asarray(metrics['xyz_coefficients'])
        yaw = np.asarray(metrics['yaw_coefficients'])[::-1].T
        if (ts.ndim != 1 or len(ts) < 1 or np.any(ts <= 0)
                or xyz.shape != (len(ts), 6, 3) or yaw.shape != (len(ts), 4)
                or not np.all(np.isfinite(np.r_[ts, xyz.ravel(), yaw.ravel()]))):
            raise ValueError('INVALID_COEFFICIENTS')
        for coefficients, duration in zip(yaw, ts):
            times = [0., duration]
            if abs(coefficients[3]) > 1e-12:
                vertex = -coefficients[2]/(3*coefficients[3])
                if 0 < vertex < duration:
                    times.append(vertex)
            if np.max(np.abs(np.polynomial.polynomial.polyval(
                    times, np.polynomial.polynomial.polyder(coefficients)))) > (
                        model.config.maximum_yaw_rate+1e-6):
                raise ValueError('YAW_RATE_LIMIT')
        problem = FollowProblem(updated, model, sum(ts))
        # No future prediction can refresh the unchanged raw navigation epoch.
        if now >= min(c.navigation_stamp+.125, c.attitude_stamp+.125,
                      prediction.source_stamp+.125, prediction.observation_stamp+.125,
                      prediction.valid_until):
            raise ValueError('INPUT_EXPIRED')
        peaks = {}
        for name, derivative, axes in (
                ('horizontal_speed', 1, (0, 1)), ('vertical_speed', 1, (2,)),
                ('horizontal_acceleration', 2, (0, 1)), ('vertical_acceleration', 2, (2,)),
                ('horizontal_jerk', 3, (0, 1)), ('vertical_jerk', 3, (2,))):
            upper = derivative_upper_bound(xyz, ts, derivative, axes)
            limit = getattr(model.config, 'maximum_'+name)
            peaks[name] = (upper if upper <= limit else
                           derivative_peak(xyz, ts, derivative, axes)['value'])
            if peaks[name] > limit+1e-6:
                raise ValueError('DYNAMIC_INFEASIBLE')
        knots = np.r_[0., np.cumsum(ts)]
        query = np.unique(np.r_[knots, np.linspace(0., sum(ts), int(sum(ts)/.05)+2)])

        def assess(times):
            if clock()-started >= budget:
                raise ValueError('REVALIDATION_DEADLINE')
            index = np.minimum(np.searchsorted(knots[1:], times, side='right'), len(ts)-1)
            local = times-knots[index]
            values = []
            for derivative in range(4):
                coeff = np.polynomial.polynomial.polyder(xyz, derivative, axis=1)
                values.append(np.array([np.polynomial.polynomial.polyval(t, coeff[i])
                                        for t, i in zip(local, index)]))
            angles = np.array([np.polynomial.polynomial.polyval(t, yaw[i])
                               for t, i in zip(local, index)])
            dy = np.polynomial.polynomial.polyder(yaw, axis=1)
            rates = np.array([np.polynomial.polynomial.polyval(t, dy[i])
                              for t, i in zip(local, index)])
            return problem.assess(times, *values, angles, rates, batch=True)

        for iteration in range(3):
            margins, views, geometry, _ = assess(query)
            if iteration == 2:
                break
            close = ((geometry.min(axis=1) < .08)
                     | (np.vstack(tuple(margins.values())).min(axis=0) < .1))
            intervals = close[:-1] | close[1:]
            if not intervals.any():
                break
            query = np.unique(np.r_[query, ((query[:-1]+query[1:])/2)[intervals]])
        valid = (all(v.whole_target_safe for v in views)
                 and all(np.min(value) >= -1e-6 for value in margins.values()))
        if clock()-started >= budget:
            raise ValueError('REVALIDATION_DEADLINE')
        report.update(valid=valid, reason='' if valid else 'PREDICTION_REVALIDATION_FAILED',
                      samples=len(query), minimum_horizontal_margin=float(geometry[:, 0].min()),
                      minimum_vertical_margin=float(geometry[:, 1].min()), analytic_peaks=peaks)
        return updated, dict(report, elapsed=clock()-started)
    except (ValueError, TypeError, KeyError, OverflowError, FloatingPointError) as error:
        return request, dict(report, reason=str(error), elapsed=clock()-started)

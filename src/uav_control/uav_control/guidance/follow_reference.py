"""Explicit observation-point kinematics; bounded forecast fitting is a research model."""
import numpy as np


def viewpoint_state(position, velocity, acceleration, heading, heading_rate,
                    heading_acceleration, distance=5., beta=0., beta_rate=0.,
                    beta_acceleration=0., distance_rate=0., distance_acceleration=0.,
                    altitude=-5.):
    """Differentiate target - d e(heading+beta); z is an independent altitude hold."""
    values = np.r_[position, velocity, acceleration, heading, heading_rate,
                   heading_acceleration, distance, beta, beta_rate, beta_acceleration,
                   distance_rate, distance_acceleration, altitude]
    if len(values) != 19 or not np.all(np.isfinite(values)) or distance <= 0:
        raise ValueError('INVALID_VIEWPOINT_STATE')
    theta, omega, alpha = heading + beta, heading_rate + beta_rate, (
        heading_acceleration + beta_acceleration)
    e = np.array((np.cos(theta), np.sin(theta), 0.))
    n = np.array((-e[1], e[0], 0.))
    p = np.asarray(position) - distance * e
    v = np.asarray(velocity) - distance_rate * e - distance * omega * n
    a = (np.asarray(acceleration) + (distance * omega**2 - distance_acceleration) * e
         - (2 * distance_rate * omega + distance * alpha) * n)
    p[2], v[2], a[2] = altitude, 0., 0.
    return p, v, a


def viewpoint_state_batch(position, velocity, acceleration, heading, omega, alpha,
                          distances, betas, altitude):
    """Batch the constant-beta/d subset of the same observation reference model."""
    d, beta = np.asarray(distances), np.asarray(betas)
    if (d.ndim != 1 or beta.shape != d.shape or np.any(d <= 0)
            or not np.all(np.isfinite(np.r_[position, velocity, acceleration, heading,
                                            omega, alpha, d, beta, altitude]))):
        raise ValueError('INVALID_VIEWPOINT_STATE')
    theta = heading+beta
    e = np.c_[np.cos(theta), np.sin(theta), np.zeros(len(d))]
    n = np.c_[-e[:, 1], e[:, 0], np.zeros(len(d))]
    p = np.asarray(position)-d[:, None]*e
    v = np.asarray(velocity)-d[:, None]*omega*n
    a = np.asarray(acceleration)+d[:, None]*omega**2*e-d[:, None]*alpha*n
    p[:, 2], v[:, 2], a[:, 2] = altitude, 0., 0.
    return p, v, a


def forecast_viewpoint(problem, time, beta=0., distance=None):
    """
    Fit local forecast heading/speed, with explicit acceleration/jerk-derived bounds.

    Prediction messages have P/V, not trusted angular acceleration. Fit at most
    nine forecast samples over +/-0.4s, no past extrapolation. Clip research
    derivatives using UAV acceleration/jerk limits; report every clip/residual.
    This is not a measured USV dynamics bound or a Tracker boundary.
    """
    relative = problem.request.context.execution_start_stamp + time - (
        problem.request.context.prediction_source_stamp)
    ts = np.asarray(problem.request.prediction_times)
    velocities = np.asarray(problem.request.target_velocities)
    speed = np.linalg.norm(velocities[:, :2], axis=1)
    mask = (abs(ts - relative) <= .400001) & (speed >= .2)
    p, v = problem.target_state((time,))
    if np.count_nonzero(mask) < 3 or np.linalg.norm(v[0, :2]) < .2:
        heading, omega, alpha, acceleration, residual, clipped = (
            problem.request.state[9], 0., 0., np.zeros(3), 0., False)
    else:
        x = ts[mask] - relative
        angles = np.unwrap(np.arctan2(velocities[mask, 1], velocities[mask, 0]))
        h = np.polynomial.polynomial.polyfit(x, angles, 2)
        s = np.polynomial.polynomial.polyfit(x, speed[mask], 2)
        heading = float(h[0])
        vmax = max(float(np.linalg.norm(v[0, :2])), .2)
        bound_w = problem.limits.maximum_horizontal_acceleration / vmax
        bound_alpha = problem.limits.maximum_horizontal_jerk / vmax
        omega, alpha = float(np.clip(h[1], -bound_w, bound_w)), float(
            np.clip(2*h[2], -bound_alpha, bound_alpha))
        tangential = float(np.clip(s[1], -problem.limits.maximum_horizontal_acceleration,
                                   problem.limits.maximum_horizontal_acceleration))
        e = np.array((np.cos(heading), np.sin(heading), 0.))
        acceleration = tangential * e + vmax * omega * np.array((-e[1], e[0], 0.))
        residual = float(np.sqrt(np.mean((np.polynomial.polynomial.polyval(x, h)-angles)**2)))
        clipped = abs(omega-h[1]) > 1e-9 or abs(alpha-2*h[2]) > 1e-9 or abs(
            tangential-s[1]) > 1e-9
    distance = problem.limits.follow_distance if distance is None else distance
    state = viewpoint_state(p[0], v[0], acceleration, heading, omega, alpha,
                            distance=distance, beta=beta, altitude=problem.limits.flight_altitude)
    return (*state, heading + beta, dict(heading_rate=omega, heading_acceleration=alpha,
                                         heading_fit_rmse=residual,
                                         derivative_clipped=bool(clipped),
                                         beta_rate_assumption=0., distance_rate_assumption=0.,
                                         fit_samples=int(np.count_nonzero(mask)),
                                         derivative_available=bool(np.count_nonzero(mask) >= 3
                                                                   and np.linalg.norm(
                                                                       v[0, :2]) >= .2)))

"""
Read-only physical-epoch and vector attribution; never retime controller inputs.

Migrated pure functions from historical tools at 55c1003; no CLI or online wiring.
"""

import numpy as np


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


def native_correspondence(observed, commands):
    """
    Value corroboration in a causal 150 ms interval, never a fitted epoch alignment.

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

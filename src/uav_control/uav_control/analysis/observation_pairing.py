"""
Offline causal observation pairing and independently prescribed geometry.

Migrated pure functions from historical tools at 55c1003; no CLI or online wiring.
"""
from bisect import bisect_right
import math

import numpy as np


def vector_at(stamp, times, vectors, maximum_gap=.05):
    """Interpolate only a finite native-time bracket, never extrapolate."""
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) < 1e-8:
        return vectors[index]
    if index == 0 or index == len(times):
        raise ValueError('position_not_bracketed')
    left, right = times[index - 1:index + 1]
    if right <= left or max(stamp-left, right-stamp) > maximum_gap:
        raise ValueError('position_bracket_too_wide')
    return vectors[index-1] + (vectors[index]-vectors[index-1]) * (
        (stamp-left)/(right-left))


def attitude_surrogate(position, vision, reference, estimated, truth):
    """Hold position fixed and change only attitude using a ULog surrogate.

    Valid production frames lack saved camera/body vectors and quaternions.
    Thus this is NOT exact reconstruction of the production pose or a new
    physical measurement. No truth is sent back to a running system.
    """
    body = estimated.inv().apply(vision-position)
    return position + truth.apply(body) - reference


def bounded_interpolate(stamp, times, values, gap=0.075):
    """Use two nearby samples, with no extrapolation or nearest-value fill."""
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) < 1e-7:
        return values[index]
    if index == 0 or index == len(times):
        return None
    left, right = times[index - 1:index + 1]
    if max(stamp - left, right - stamp) > gap or right <= left:
        return None
    return values[index - 1] + (values[index] - values[index - 1]) * (
        (stamp - left) / (right - left))


def causal_kf_at_image(states, stamp, maximum_age=0.075):
    """Project last *published* state to image time, with no future update."""
    stamps = [state['stamp'] for state in states]
    index = bisect_right(stamps, stamp) - 1
    if index < 0:
        return None, 'NO_PRIOR_KF'
    state = states[index]
    age = stamp - state['stamp']
    if age < -1e-8 or age > maximum_age:
        return None, 'KF_PUBLISH_GAP'
    if state['source_stamp'] > stamp + 1e-6:
        return None, 'KF_FUTURE_SOURCE'
    if not state['valid'] or state['frame_id'] != 'local_ned':
        return None, 'KF_INVALID_OR_FRAME'
    position = np.asarray(state['position_ned'], dtype=float)
    velocity = np.asarray(state['velocity_ned'], dtype=float)
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
        return None, 'KF_NONFINITE'
    return {
        'position': (position + age * velocity).tolist(),
        'velocity': velocity.tolist(),
        'publish_stamp': state['stamp'],
        'source_stamp': state['source_stamp'],
        'publish_age_s': age,
        'source_age_s': stamp - state['source_stamp'],
        'status': 'UPDATED_SINCE_PREVIOUS_PUBLISH' if (
            index > 0 and state['source_stamp']
            > states[index - 1]['source_stamp'] + 1e-8)
        else 'PREDICT_ONLY_SINCE_PREVIOUS_PUBLISH',
    }, ''


def finite_difference_velocity(rows, key, maximum_dt=0.15):
    """Raw B has no velocity field; use adjacent image-time difference."""
    result = {}
    previous = None
    for row in rows:
        if not row.get('frame_valid') or key not in row:
            previous = None
            continue
        stamp = row['timestamp']
        if previous is not None:
            dt = stamp - previous['timestamp']
            if 0 < dt <= maximum_dt and row['reset_epoch'] == previous[
                            'reset_epoch']:
                result[stamp] = ((np.asarray(row[key]) - np.asarray(
                    previous[key])) / dt).tolist()
        previous = row
    return result


def lag_sweep(pairs, estimate_key, truth_key):
    """Compare every lag on the same bounded-interpolation image subset."""
    times = np.asarray([pair['timestamp'] for pair in pairs])
    truth = np.asarray([pair[truth_key] for pair in pairs])
    lags = np.arange(-0.2, 0.20001, 0.01)
    common = []
    for pair in pairs:
        shifted = [bounded_interpolate(pair['timestamp'] + lag,
                                       times, truth, gap=0.15)
                   for lag in lags]
        if all(value is not None for value in shifted):
            common.append((pair, shifted))
    if len(common) < 20:
        return {'curve': [], 'best': None, 'common_count': len(common)}
    curve = []
    for index, lag in enumerate(lags):
        errors = [np.asarray(pair[estimate_key]) - shifted[index]
                  for pair, shifted in common]
        curve.append({'lag_s': round(float(lag), 3), **metrics(errors)})
    return {'curve': curve,
            'best': min(curve, key=lambda item: item['rmse_m']),
            'common_count': len(common)}


def enu_to_ned(position):
    x, y, z = (float(value) for value in position)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError('nonfinite ENU position')
    return [y, x, -z]


def xy_displacement(start, end):
    return (np.asarray(end, dtype=float)[:2]
            - np.asarray(start, dtype=float)[:2]).tolist()


def rotation(theta):
    cosine, sine = math.cos(theta), math.sin(theta)
    return np.array(((cosine, -sine), (sine, cosine)))


def require_noncollinear(displacements):
    values = np.asarray(displacements, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 2:
        raise ValueError('noncollinear displacement set requires two axes')
    singular = np.linalg.svd(values, compute_uv=False)
    if singular[0] < 1.0 or singular[1] / singular[0] < 0.15:
        raise ValueError('noncollinear displacement set required')


def fit_fixed_rotation(gazebo_deltas, px4_deltas):
    gazebo = np.asarray(gazebo_deltas, dtype=float)
    px4 = np.asarray(px4_deltas, dtype=float)
    if gazebo.shape != px4.shape:
        raise ValueError('displacement shapes differ')
    require_noncollinear(gazebo)
    dot = float(np.sum(gazebo * px4))
    cross = float(np.sum(gazebo[:, 0] * px4[:, 1]
                         - gazebo[:, 1] * px4[:, 0]))
    return math.atan2(cross, dot)


def displacement_rmse(gazebo_deltas, px4_deltas, theta):
    gazebo = np.asarray(gazebo_deltas, dtype=float)
    px4 = np.asarray(px4_deltas, dtype=float)
    errors = px4 - gazebo @ rotation(theta).T
    return float(np.sqrt(np.mean(np.sum(errors * errors, axis=1))))


def bracket_gap(stamp, left, right, limit):
    values = (stamp, left, right, limit)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('nonfinite timestamp bracket')
    if left > stamp + 1e-6 or right < stamp - 1e-6:
        raise ValueError('timestamp not bracketed')
    gap = max(stamp - left, right - stamp)
    if gap > limit:
        raise ValueError('timestamp gap exceeds limit')
    return gap


def segment_epochs(rows):
    epochs = []
    previous_stamp = None
    previous_key = None
    epoch = 0
    for row in rows:
        stamp = float(row['ros_stamp'])
        key = tuple(row['reset_key'])
        if previous_stamp is not None and (
                stamp <= previous_stamp or key != previous_key):
            epoch += 1
        epochs.append(epoch)
        previous_stamp, previous_key = stamp, key
    return epochs


def metrics(vectors):
    values = np.asarray(vectors, dtype=float)
    if not values.size:
        return {'n': 0}
    norms = np.linalg.norm(values, axis=1)
    return {
        'n': len(values), 'rmse_m': float(np.sqrt(np.mean(norms ** 2))),
        'p50_m': float(np.percentile(norms, 50)),
        'p95_m': float(np.percentile(norms, 95)),
        'max_m': float(max(norms)),
        'signed_mean_xyz_m': values.mean(axis=0).tolist(),
    }

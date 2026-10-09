"""Shared-calibration batch adapters; the original P1 geometry kernel is unchanged."""
import numpy as np

from uav_control.guidance.camera_visibility import (
    _validated_inputs, _evaluate_geometry, evaluate_visibility,
)
from uav_control.guidance.planned_attitude import (
    PlannedAttitudeResult, _valid_config, planned_attitude,
)


def attitude_batch(acceleration, yaw, config):
    """Vectorize ideal FRD axes; scalar rejection retains exact P2 reason precedence."""
    a, y = np.asarray(acceleration), np.asarray(yaw)
    if a.ndim != 2 or a.shape[1] != 3 or y.shape != (len(a),):
        raise ValueError('INVALID_BATCH_SHAPE')
    if (a.dtype.kind not in 'fi' or y.dtype.kind not in 'fi' or not _valid_config(config)
            or not np.all(np.isfinite(a)) or not np.all(np.isfinite(y))):
        return [planned_attitude(ai, yi, config) for ai, yi in zip(a, y)]
    force = np.array((0., 0., config.gravity))-a
    thrust = np.linalg.norm(force, axis=1)
    safe = np.maximum(thrust, 1e-9)
    b3 = force/safe[:, None]
    tilt = np.arctan2(np.linalg.norm(b3[:, :2], axis=1), b3[:, 2])
    h = np.c_[np.cos(y), np.sin(y), np.zeros(len(y))]
    b2 = np.cross(b3, h)
    norms = np.linalg.norm(b2, axis=1)
    b2 /= np.maximum(norms, 1e-9)[:, None]
    rotations = np.stack((np.cross(b2, b3), b2, b3), axis=2)
    valid = ((thrust >= config.minimum_specific_thrust)
             & (thrust <= config.maximum_specific_thrust) & (thrust >= 1e-9)
             & (tilt <= config.maximum_tilt_rad) & (norms >= 1e-9))
    return [PlannedAttitudeResult(True, 'OK', tuple(map(tuple, rotations[i])),
                                  float(tilt[i]), float(thrust[i])) if valid[i]
            else planned_attitude(a[i], y[i], config) for i in range(len(a))]


def visibility_batch(positions, rotations, targets, intrinsics, extrinsics, target, constraints):
    """
    Validate mount once and SO(3) in batch, then use exactly the P1 sphere projection.

    Invalid rows use the scalar adapter to preserve its input/error priority. No
    smooth optimization proxy, horizontal-only FOV, or center-only admission is used.
    """
    p, r, t = np.asarray(positions), np.asarray(rotations), np.asarray(targets)
    if p.ndim != 2 or p.shape[1] != 3 or t.shape != p.shape or r.shape != (len(p), 3, 3):
        raise ValueError('INVALID_BATCH_SHAPE')
    try:
        values = _validated_inputs(np.zeros(3), np.eye(3), np.zeros(3), intrinsics,
                                   extrinsics, target, constraints)
    except ValueError:
        return [evaluate_visibility(pi, ri, ti, intrinsics, extrinsics, target, constraints)
                for pi, ri, ti in zip(p, r, t)]
    finite_r = np.all(np.isfinite(r), axis=(1, 2))
    valid = (np.all(np.isfinite(p), axis=1) & np.all(np.isfinite(t), axis=1) & finite_r
             & np.all(np.abs(r) <= 1.+1e-7, axis=(1, 2)))
    indices = np.flatnonzero(finite_r)
    finite_rotations = r[indices]
    products = np.transpose(finite_rotations, (0, 2, 1))@finite_rotations
    orthogonal = np.all(np.abs(products-np.eye(3)) <= 1e-7, axis=(1, 2))
    valid[indices] &= orthogonal & (np.abs(np.linalg.det(finite_rotations)-1.) <= 1e-7)
    translation, mount, offset = values[3:]
    results = []
    for i in range(len(p)):
        if valid[i]:
            try:
                with np.errstate(over='raise', invalid='raise', divide='raise'):
                    result = _evaluate_geometry(p[i], r[i], t[i], translation, mount, offset,
                                                intrinsics, target, constraints)
            except (FloatingPointError, OverflowError):
                result = evaluate_visibility(p[i], r[i], t[i], intrinsics, extrinsics,
                                             target, constraints)
        else:
            result = evaluate_visibility(p[i], r[i], t[i], intrinsics, extrinsics,
                                         target, constraints)
        results.append(result)
    return results

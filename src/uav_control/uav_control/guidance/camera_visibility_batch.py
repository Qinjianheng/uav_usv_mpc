"""Shared-calibration batch adapters; the original P1 geometry kernel is unchanged."""
from uav_control.guidance.follow_profile import profiled, count

import numpy as np

from uav_control.guidance.camera_visibility import (
    _validated_inputs, _evaluate_geometry, evaluate_visibility,
    CameraVisibilityResult, _FLU_TO_FRD, _OPTICAL_TO_CAMERA_FLU, _BOUNDARY_TOLERANCE,
)
from uav_control.guidance.planned_attitude import (
    PlannedAttitudeResult, _valid_config, planned_attitude,
)


def _geometry_batch(p, r, t, translation, mount, offset, intrinsics, target, constraints):
    """Batch the original sphere projection; scalar authority resolves numeric boundaries."""
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        origin = p + (r @ _FLU_TO_FRD @ translation[..., None])[..., 0]
        optical_rotation = r @ _FLU_TO_FRD @ mount @ _OPTICAL_TO_CAMERA_FLU
        optical = (np.swapaxes(optical_rotation, 1, 2)
                   @ (t+offset-origin)[..., None])[..., 0]
        x, y, z = optical.T
        radius, tol = float(target.radius), _BOUNDARY_TOLERANCE
        front, whole_front = z > constraints.depth_epsilon, z-radius > constraints.depth_epsilon
        distance = z if constraints.distance_mode == 'axial' else np.hypot(np.hypot(x, y), z)
        near, far = distance-radius, distance+radius
        if constraints.distance_mode == 'radial':
            near = np.maximum(0., near)
        center_range = ((distance >= constraints.minimum_distance-tol)
                        & (distance <= constraints.maximum_distance+tol))
        whole_range = ((near >= constraints.minimum_distance-tol)
                       & (far <= constraints.maximum_distance+tol))
        uv = np.c_[intrinsics.fx*(x/z)+intrinsics.cx, intrinsics.fy*(y/z)+intrinsics.cy]
        center_image = ((uv[:, 0] >= -tol) & (uv[:, 0] <= intrinsics.width+tol)
                        & (uv[:, 1] >= -tol) & (uv[:, 1] <= intrinsics.height+tol))
        h, v = np.arctan2(x, z), np.arctan2(y, z)
        he = np.arcsin(np.minimum(1., radius/np.hypot(x, z)))
        ve = np.arcsin(np.minimum(1., radius/np.hypot(y, z)))
        h0, h1, v0, v1 = h-he, h+he, v-ve, v+ve
        left, right, up, down = intrinsics.angle_bounds()
        hm = np.minimum(h0-left, right-h1)-constraints.horizontal_margin_rad
        vm = np.minimum(v0-up, down-v1)-constraints.vertical_margin_rad
        bbox = np.c_[intrinsics.fx*np.tan(h0)+intrinsics.cx,
                     intrinsics.fy*np.tan(v0)+intrinsics.cy,
                     intrinsics.fx*np.tan(h1)+intrinsics.cx,
                     intrinsics.fy*np.tan(v1)+intrinsics.cy]
        flu = (np.broadcast_to(_OPTICAL_TO_CAMERA_FLU, r.shape) @ optical[..., None])[..., 0]
        numeric = (np.all(np.isfinite(optical), axis=1) & np.isfinite(distance)
                   & np.isfinite(near) & np.isfinite(far)
                   & (~front | np.all(np.isfinite(uv), axis=1))
                   & (~whole_front | np.all(np.isfinite(bbox), axis=1)))
        # This is a computation fallback band, not a safety tolerance. The
        # unchanged scalar kernel decides all near-boundary results exactly.
        boundary = (np.minimum(abs(hm), abs(vm)) < 1e-10) & whole_front
        for value in (z-constraints.depth_epsilon, z-radius-constraints.depth_epsilon,
                      near-constraints.minimum_distance, far-constraints.maximum_distance,
                      distance-constraints.minimum_distance, distance-constraints.maximum_distance,
                      uv[:, 0], uv[:, 0]-intrinsics.width, uv[:, 1], uv[:, 1]-intrinsics.height):
            boundary |= abs(value) < 1e-10
    results = []
    for i in range(len(p)):
        if not numeric[i] or boundary[i]:
            try:
                with np.errstate(over='raise', invalid='raise', divide='raise'):
                    result = _evaluate_geometry(p[i], r[i], t[i], translation, mount, offset,
                                                intrinsics, target, constraints)
            except (FloatingPointError, OverflowError):
                result = CameraVisibilityResult(reasons=('NUMERICAL_ERROR',))
            results.append(result)
            continue
        reasons = []
        if not front[i]:
            reasons.append('TARGET_BEHIND_CAMERA' if z[i] < -constraints.depth_epsilon
                           else 'ZERO_DEPTH')
        elif not whole_front[i]:
            reasons.append('TARGET_CROSSES_CAMERA_PLANE')
        else:
            if hm[i] < -tol:
                reasons.append('HORIZONTAL_FOV')
            if vm[i] < -tol:
                reasons.append('VERTICAL_FOV')
        if near[i] < constraints.minimum_distance-tol:
            reasons.append('BELOW_MINIMUM_DISTANCE')
        if far[i] > constraints.maximum_distance+tol:
            reasons.append('ABOVE_MAXIMUM_DISTANCE')
        results.append(CameraVisibilityResult(
            valid_input=True, target_camera_optical=tuple(map(float, optical[i])),
            target_camera_flu=tuple(map(float, flu[i])),
            image_center_uv=tuple(map(float, uv[i])) if front[i] else None,
            image_bounds_uv=tuple(map(float, bbox[i])) if front[i] and whole_front[i] else None,
            center_in_front=bool(front[i]), whole_in_front=bool(whole_front[i]),
            center_in_image=bool(front[i] and center_image[i]),
            center_in_range=bool(center_range[i]),
            center_visible=bool(front[i] and center_image[i] and center_range[i]),
            whole_in_range=bool(whole_range[i]),
            distance_interval_m=(float(near[i]), float(far[i])),
            horizontal_margin_rad=float(hm[i]) if front[i] and whole_front[i] else None,
            vertical_margin_rad=float(vm[i]) if front[i] and whole_front[i] else None,
            whole_target_safe=not reasons, reasons=tuple(reasons)))
    return results


@profiled('planned_attitude')
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


@profiled('full_fov')
def visibility_batch(positions, rotations, targets, intrinsics, extrinsics, target, constraints):
    """
    Validate mount once and SO(3) in batch, then use exactly the P1 sphere projection.

    Invalid rows use the scalar adapter to preserve its input/error priority. No
    smooth optimization proxy, horizontal-only FOV, or center-only admission is used.
    """
    count('projection_samples', len(positions))
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
    results = [None]*len(p)
    indices = np.flatnonzero(valid)
    if len(indices):
        checked = _geometry_batch(p[indices], r[indices], t[indices], translation, mount, offset,
                                  intrinsics, target, constraints)
        for index, result in zip(indices, checked):
            results[index] = result
    for index in np.flatnonzero(~valid):
        results[index] = evaluate_visibility(p[index], r[index], t[index], intrinsics, extrinsics,
                                             target, constraints)
    return results

"""
Research MINCO adjoint and batched full-pose smooth sphere cost.

Q/T mapping, jerk and integration-time derivatives are analytic. Local smooth
constraint partials use complex-step (no complete-objective finite differences).
Clamped yaw duration sensitivities use six small spline finite differences.
Final admission always uses the independent P1 projection, never this proxy.
"""
import math
import time

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.linalg import lu_factor, lu_solve

from uav_control.guidance.camera_visibility import _FLU_TO_FRD, _OPTICAL_TO_CAMERA_FLU
from uav_control.guidance.follow_minco_optimizer import _trajectory
from uav_control.guidance.minco_trajectory import MincoS3Trajectory


def _norm(x):
    return np.sqrt(np.sum(x*x, axis=-1) + 1e-18)


def _positive(x):
    return np.where(x.real > 0, x, 0.)


def _softplus(x):
    result = np.empty_like(x)
    mask = x.real > 0
    result[mask] = x[mask] + np.log1p(np.exp(-x[mask]))
    result[~mask] = np.log1p(np.exp(x[~mask]))
    return result


class FastMincoObjective:
    """Expose (cost, gradient) to L-BFGS-B; reuse one coefficient/LU solve per call."""

    def __init__(self, problem, seed, config, yaw_optimize=True, visibility_mode='full'):
        """Precompute calibration and normalized Gauss nodes, preserving explicit bounds."""
        self.problem, self.seed, self.config = problem, seed, config
        self.yaw_optimize, self.visibility_mode = yaw_optimize, visibility_mode
        self.start_rate = getattr(problem.request, 'reference_yaw_rate', 0.)
        self.total = sum(seed.durations)
        self.u = np.array((.21132486540518713, .7886751345948129))
        self.pieces = np.repeat(np.arange(3), 2)
        self.local_u = np.tile(self.u, 3)
        self.rotation = (_OPTICAL_TO_CAMERA_FLU.T @ np.asarray(
            problem.model.extrinsics.rotation_camera_to_body_flu).T @ _FLU_TO_FRD)
        self.translation = _FLU_TO_FRD @ np.asarray(problem.model.extrinsics.translation_flu)
        self.offset = np.asarray(problem.model.target.center_offset_ned)
        self.radius = problem.model.target.radius
        left, right, up, down = problem.model.intrinsics.angle_bounds()
        hm, vm = (problem.model.visibility.horizontal_margin_rad,
                  problem.model.visibility.vertical_margin_rad)
        left, right, up, down = left+hm, right-hm, up+vm, down-vm
        self.planes = np.array(((np.cos(left), 0, -np.sin(left)),
                                (-np.cos(right), 0, np.sin(right)),
                                (0, np.cos(up), -np.sin(up)),
                                (0, -np.cos(down), np.sin(down))))
        self.counts = dict(objective=0, gradient=0, minco_builds=0, adjoint_solves=0,
                           yaw_constructions=0, batch_geometry=0, local_complex_steps=0)
        self.timing = dict(objective_gradient=0., matrix=0., local_partials=0., yaw=0.)
        self.cache_key, self.cache_value = None, None

    def initial(self):
        """Use original fixed-total softmax coordinates and continuous yaw knot variables."""
        x = np.r_[np.asarray(self.seed.q).ravel()]
        if self.config.mode == 'qt':
            x = np.r_[x, np.log(np.asarray(self.seed.durations[:2])/self.seed.durations[-1])]
        if self.yaw_optimize:
            x = np.r_[x, np.unwrap(self.seed.yaw)[1:]]
        return x

    def unpack(self, x):
        """Separate Q/T/yaw without confusing yaw with duration variables."""
        durations = (np.asarray(MincoS3Trajectory.durations_from_logits(self.total, x[6:8]))
                     if self.config.mode == 'qt' else np.asarray(self.seed.durations))
        values = (np.unwrap(np.r_[self.seed.yaw[0], x[-3:]]) if self.yaw_optimize
                  else np.unwrap(self.seed.yaw))
        return x[:6].reshape(2, 3), durations, values

    def geometry(self, p, a, yaw, target):
        """Batch exact planned axes and plane/sphere margins, including mount translation."""
        self.counts['batch_geometry'] += 1
        force = np.array((0, 0, self.problem.model.attitude_config.gravity)) - a
        b3 = force / _norm(force)[:, None]
        h = np.stack((np.cos(yaw), np.sin(yaw), yaw*0), axis=1)
        b2 = np.cross(b3, h)
        b2 = b2 / _norm(b2)[:, None]
        b1 = np.cross(b2, b3)
        body = np.stack((np.sum((target+self.offset-p)*b1, axis=1),
                         np.sum((target+self.offset-p)*b2, axis=1),
                         np.sum((target+self.offset-p)*b3, axis=1)), axis=1)
        optical = (body - self.translation) @ self.rotation.T
        normals = optical @ self.planes.T - self.radius
        angular_scale = np.stack((_norm(optical[:, (0, 2)]),)*2 + (
            _norm(optical[:, (1, 2)]),)*2, axis=1)
        fov = normals / angular_scale
        if self.visibility_mode == 'horizontal':
            fov = fov[:, :2]
        depth = optical[:, 2]
        vis = self.problem.model.visibility
        range_center = depth if vis.distance_mode == 'axial' else _norm(optical)
        margins = np.c_[fov, (depth-self.radius-vis.depth_epsilon)/5,
                        (range_center-self.radius-vis.minimum_distance)/5,
                        (vis.maximum_distance-range_center-self.radius)/5]
        return optical, margins, force, b3

    def local_cost(self, p, v, a, j, yaw, rate, target):
        """Smooth visibility and C1 squared physical violations; no hard admission here."""
        _, geometric, force, b3 = self.geometry(p, a, yaw, target)
        l, c = self.problem.limits, self.config
        dynamic = np.c_[l.maximum_horizontal_speed-_norm(v[:, :2]),
                        l.maximum_vertical_speed-np.sqrt(v[:, 2]**2+1e-18),
                        l.maximum_horizontal_acceleration-_norm(a[:, :2]),
                        l.maximum_vertical_acceleration-np.sqrt(a[:, 2]**2+1e-18),
                        l.maximum_horizontal_jerk-_norm(j[:, :2]),
                        l.maximum_vertical_jerk-np.sqrt(j[:, 2]**2+1e-18),
                        l.maximum_yaw_rate-np.sqrt(rate**2+1e-18),
                        b3[:, 2]-np.cos(self.problem.model.attitude_config.maximum_tilt_rad),
                        _norm(force)-self.problem.model.attitude_config.minimum_specific_thrust,
                        self.problem.model.attitude_config.maximum_specific_thrust-_norm(force)]
        descend = _positive(v[:, 2])
        sea = l.sea_surface_z-l.reserve_clearance-p[:, 2]-descend*l.response_delay-(
            descend**2/(2*l.braking_acceleration))
        return (c.visibility_weight*np.mean(_softplus(-12*geometric)**2, axis=1)/144
                + c.dynamic_weight*np.sum(_positive(-dynamic)**2, axis=1)
                + c.safety_weight*_positive(-sea)**2 + c.yaw_weight*rate**2)

    def __call__(self, x):
        """Return shared cost/gradient using MINCO adjoint and explicit physical-time terms."""
        x = np.asarray(x, dtype=float)
        key = x.tobytes()
        if key == self.cache_key:
            return self.cache_value
        start = time.perf_counter()
        self.counts['objective'] += 1
        self.counts['gradient'] += 1
        stamp = time.perf_counter()
        q, durations, values = self.unpack(x)
        trajectory = _trajectory(self.seed, q, durations)
        matrix, _ = trajectory._minimum_control_system(
            *[tuple(self.seed.start[i:i+3]) for i in (0, 3, 6)],
            *[tuple(self.seed.end[i:i+3]) for i in (0, 3, 6)], tuple(map(tuple, q)))
        factor = lu_factor(matrix, check_finite=False)
        self.counts['minco_builds'] += 1
        self.timing['matrix'] += time.perf_counter()-stamp
        knots = np.r_[0., np.cumsum(durations)]
        times = knots[self.pieces] + self.local_u*durations[self.pieces]
        local = self.local_u*durations[self.pieces]
        basis = [np.stack([MincoS3Trajectory._basis(t, d) for t in local])
                 for d in range(5)]
        fields = [np.einsum('nk,nkd->nd', b, trajectory.coefficients[self.pieces])
                  for b in basis]
        stamp = time.perf_counter()
        spline = CubicSpline(knots, values, bc_type=((1, self.start_rate), (1, 0.)))
        self.counts['yaw_constructions'] += 1
        yaw, rate = spline(times), spline(times, 1)
        target, _ = self.problem.target_state(times)
        weights = durations[self.pieces]/(2*self.total)
        args = [*fields[:4], yaw, rate, target]
        local_cost = self.local_cost(*args)
        follow, tracking_partials, tracking_time = self.tracking(fields, times)
        jerk_cost, jerk_gradient, explicit_t = self.jerk(trajectory)
        cost = jerk_cost + np.dot(weights, local_cost+follow)
        self.timing['yaw'] += time.perf_counter()-stamp
        stamp = time.perf_counter()
        partials = self.local_partials(args)
        for k in range(4):
            partials[k] += tracking_partials[k]
        self.timing['local_partials'] += time.perf_counter()-stamp
        gc = jerk_gradient.copy()
        for d in range(4):
            np.add.at(gc, self.pieces, np.einsum('n,nk,nd->nkd', weights, basis[d], partials[d]))
        gt = explicit_t.copy()
        # Forecast interpolation slopes provide physical global-time dependencies.
        ts = np.asarray(self.problem.request.prediction_times)
        rt = self.problem.request.context.execution_start_stamp + times - (
            self.problem.request.context.prediction_source_stamp)
        index = np.clip(np.searchsorted(ts, rt, side='right')-1, 0, len(ts)-2)
        tp = np.asarray(self.problem.request.target_positions)
        slope_p = (tp[index+1]-tp[index])/(ts[index+1]-ts[index])[:, None]
        time_cost = np.sum(partials[6]*slope_p, axis=1)+tracking_time
        for i in range(3):
            selector = self.pieces == i
            gt[i] += np.sum((local_cost+follow)[selector])/(2*self.total)
            global_dt = (self.pieces > i).astype(float) + selector * self.local_u
            gt[i] += np.dot(weights, time_cost*global_dt)
            for d in range(4):
                gt[i] += np.dot(weights, np.sum(partials[d]*fields[d+1], axis=1) *
                                selector * self.local_u)
            # Remaining finite differences are ONLY clamped yaw/T sensitivities.
            step = 1e-5*durations[i]
            samples = []
            for sign in (-1, 1):
                varied = durations.copy()
                varied[i] += sign*step
                nodes = np.r_[0., np.cumsum(varied)]
                query = nodes[self.pieces]+self.local_u*varied[self.pieces]
                ys = CubicSpline(nodes, values, bc_type=((1, self.start_rate), (1, 0.)))
                samples.append((ys(query), ys(query, 1)))
                self.counts['yaw_constructions'] += 1
            gt[i] += np.dot(weights, partials[4]*(samples[1][0]-samples[0][0])/(2*step)
                            + partials[5]*(samples[1][1]-samples[0][1])/(2*step))
        adjoint = lu_solve(factor, gc.reshape(18, 3), trans=1, check_finite=False)
        self.counts['adjoint_solves'] += 1
        gq = np.stack([adjoint[6+6*i]+adjoint[7+6*i] for i in range(2)])
        coefficients = trajectory.coefficients
        for i in range(3):
            if i == 2:
                for d in range(3):
                    gt[i] -= np.dot(adjoint[3+d], MincoS3Trajectory._basis(durations[i], d+1)
                                    @ coefficients[i])
            else:
                for row, d in [(6+6*i, 0), *[(8+6*i+k, 1+k) for k in range(4)]]:
                    gt[i] -= np.dot(adjoint[row], MincoS3Trajectory._basis(durations[i], d+1)
                                    @ coefficients[i])
        gradient = gq.ravel()
        if self.config.mode == 'qt':
            softmax_jac = np.diag(durations)-np.outer(durations, durations)/self.total
            gradient = np.r_[gradient, gt @ softmax_jac[:, :2]]
        if self.yaw_optimize:
            gy = []
            for i in range(1, 4):
                vector = np.zeros(4)
                vector[i] = 1.
                ys = CubicSpline(knots, vector, bc_type=((1, 0.), (1, 0.)))
                gy.append(np.dot(weights, partials[4]*ys(times)+partials[5]*ys(times, 1)))
                self.counts['yaw_constructions'] += 1
            gradient = np.r_[gradient, gy]
        if not math.isfinite(cost) or not np.all(np.isfinite(gradient)):
            raise ValueError('NONFINITE_FAST_OBJECTIVE')
        self.timing['objective_gradient'] += time.perf_counter()-start
        self.cache_key, self.cache_value = key, (float(cost), gradient)
        return self.cache_value

    def local_partials(self, args):
        """Original complex-step implementation remains available for P31 comparisons."""
        partials = []
        for k, field in enumerate(args):
            derivative = np.empty_like(field)
            for axis in range(field.shape[1] if field.ndim == 2 else 1):
                perturbed = list(args)
                perturbed[k] = field.astype(complex)
                if field.ndim == 2:
                    perturbed[k][:, axis] += 1e-24j
                    derivative[:, axis] = self.local_cost(*perturbed).imag/1e-24
                else:
                    perturbed[k] += 1e-24j
                    derivative[:] = self.local_cost(*perturbed).imag/1e-24
                self.counts['local_complex_steps'] += 1
            partials.append(derivative)
        return partials

    def tracking(self, fields, times):
        """Original position-only objective; P43 overrides through the same adjoint chain."""
        reference, vp, headings = self.problem.reference(times)
        rt = self.problem.request.context.execution_start_stamp + times - (
            self.problem.request.context.prediction_source_stamp)
        ts = np.asarray(self.problem.request.prediction_times)
        index = np.clip(np.searchsorted(ts, rt, side='right')-1, 0, len(ts)-2)
        tp = np.asarray(self.problem.request.target_positions)
        tv = np.asarray(self.problem.request.target_velocities)
        dt = (ts[index+1]-ts[index])[:, None]
        slope_p, slope_v = (tp[index+1]-tp[index])/dt, (tv[index+1]-tv[index])/dt
        speed2 = np.sum(vp[:, :2]**2, axis=1)
        heading_rate = np.where(speed2 >= .04, (vp[:, 0]*slope_v[:, 1]-vp[:, 1]*slope_v[:, 0]) /
                                np.maximum(speed2, .04), 0.)
        slope_ref = slope_p.copy()
        slope_ref[:, :2] -= self.problem.limits.follow_distance*heading_rate[:, None]*np.c_[
            -np.sin(headings), np.cos(headings)]
        slope_ref[:, 2] = 0.
        delta = fields[0]-reference
        partials = [2*self.config.follow_weight*delta,
                    *[np.zeros_like(delta) for _ in range(3)]]
        return (self.config.follow_weight*np.sum(delta**2, axis=1), partials,
                -np.sum(partials[0]*slope_ref, axis=1))

    def jerk(self, trajectory):
        """Analytic coefficient gradient and explicit endpoint derivative of jerk integral."""
        gradient = np.zeros_like(trajectory.coefficients)
        gt = np.empty(3)
        total = 0.
        factors = np.array((6., 24., 60.))
        for i, t in enumerate(trajectory.durations):
            powers = np.arange(3)
            gram = factors[:, None]*factors[None, :]*(
                t**(powers[:, None]+powers[None, :]+1))/(powers[:, None]+powers[None, :]+1)
            c = trajectory.coefficients[i, 3:]
            total += float(np.sum(c*(gram@c)))
            gradient[i, 3:] = 2*gram@c
            gt[i] = np.sum((MincoS3Trajectory._basis(t, 3)@trajectory.coefficients[i])**2)
        return self.config.jerk_weight*total, self.config.jerk_weight*gradient, (
            self.config.jerk_weight*gt)

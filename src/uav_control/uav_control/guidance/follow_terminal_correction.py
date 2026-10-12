"""Fixed-time bounded terminal/Q corrections through the existing linear MINCO map.

The quadratic initializer and exact jerk envelope are ranking/repair tools. Only
the independent original validate_seed may admit a curve. No start, yaw or time
coordinate is optimized here.
"""
from dataclasses import replace
from functools import lru_cache
import time

import numpy as np
from scipy.optimize import minimize

from uav_control.guidance.follow_fast_validation import fields
from uav_control.guidance.follow_minco_optimizer import MincoConfig, _trajectory
from uav_control.guidance.minco_trajectory import MincoS3Trajectory
from uav_control.guidance.p43_minco_objective import TrackingMincoObjective
from uav_control.guidance.polynomial_extrema import derivative_peak
from uav_control.guidance.yaw_trajectory import YawTrajectory


@lru_cache(maxsize=24)
def linear_geometry(durations, mode):
    """Cache only target-independent scalar coefficient influences and exact quadrature."""
    names = {'a': ('a',), 'va': ('v', 'a'), 'qva': ('q0', 'q1', 'v', 'a')}[mode]
    zero = (0., 0., 0.)
    columns = []
    for name in names:
        q, velocity, acceleration = [zero, zero], zero, zero
        if name.startswith('q'):
            q[int(name[-1])] = (1., 0., 0.)
        elif name == 'v':
            velocity = (1., 0., 0.)
        else:
            acceleration = (1., 0., 0.)
        curve = MincoS3Trajectory(zero, zero, zero, zero, velocity, acceleration,
                                  tuple(q), durations)
        columns.append(curve.coefficients[:, :, 0])
    scalar = np.stack(columns, axis=-1)
    influence = np.zeros((3, 6, 3, 3*len(names)))
    for i in range(len(names)):
        for axis in range(3):
            influence[:, :, axis, i*3+axis] = scalar[:, :, i]
    u, w = np.polynomial.legendre.leggauss(3)
    starts = np.r_[0., np.cumsum(durations)[:-1]]
    query = np.concatenate([s+(u+1)*t/2 for s, t in zip(starts, durations)])
    weights = np.concatenate([w*t/2 for t in durations])
    jacobian = np.stack([np.stack([
        fields(influence[:, :, :, i], durations, np.zeros((3, 4)), query)[d]
        for i in range(influence.shape[-1])], axis=-1) for d in range(4)])
    for array in (influence, query, weights, jacobian):
        array.setflags(write=False)
    return names, influence, query, weights, jacobian


class TerminalCorrectionObjective:
    """3/6/12 bounded coordinates; endpoint P and Accepted start remain immutable."""

    def __init__(self, problem, seed, reference, mode='qva'):
        if mode not in ('a', 'va', 'qva'):
            raise ValueError('TERMINAL_MODE')
        self.problem, self.seed, self.reference, self.mode = problem, seed, reference, mode
        self.names, self.influence, self.times, self.weights, self.jacobian = linear_geometry(
            tuple(seed.durations), mode)
        radii = dict(q0=.2, q1=.2, v=.5, a=1.5)
        self.radii = np.repeat([radii[name] for name in self.names], 3)
        self.bounds = tuple((-float(r), float(r)) for r in self.radii)
        self.base = _trajectory(seed, seed.q, seed.durations).coefficients
        self.base_fields = np.asarray(fields(self.base, seed.durations, np.zeros((3, 4)),
                                             self.times)[:4])
        samples = [reference.sample(float(t)) for t in self.times]
        self.follow = np.asarray(samples).transpose(1, 0, 2)
        target, velocity = problem.target_state(self.times)
        # Differentiate the rear point under the source's piecewise-linear V model.
        c = problem.request.context
        relative = self.times+c.execution_start_stamp-c.prediction_source_stamp
        ts = problem.times
        index = np.clip(np.searchsorted(ts, relative, side='right')-1, 0, len(ts)-2)
        slope = (problem.velocities[index+1]-problem.velocities[index])/(
            ts[index+1]-ts[index])[:, None]
        speed2 = np.sum(velocity[:, :2]**2, axis=1)
        heading = np.arctan2(velocity[:, 1], velocity[:, 0])
        omega = np.where(speed2 > 1e-12,
                         (velocity[:, 0]*slope[:, 1]-velocity[:, 1]*slope[:, 0]) /
                         np.maximum(speed2, 1e-12), 0.)
        heading[speed2 <= 1e-12] = 0.
        distance = problem.limits.follow_distance
        wanted_p, wanted_v = target.copy(), velocity.copy()
        wanted_p[:, :2] -= distance*np.c_[np.cos(heading), np.sin(heading)]
        wanted_v[:, :2] -= distance*omega[:, None]*np.c_[-np.sin(heading), np.cos(heading)]
        wanted_p[:, 2], wanted_v[:, 2] = problem.limits.flight_altitude, 0.
        self.desired = np.asarray((wanted_p, wanted_v))
        self.components = ((0, self.follow[0], .16), (1, self.follow[1], 2/9),
                           (0, wanted_p, .32), (1, wanted_v, 2/9),
                           (3, np.zeros_like(wanted_p), .03/36))
        size = len(self.radii)
        self.hessian, self.gradient, self.constant = np.zeros((size, size)), np.zeros(size), 0.
        for derivative, wanted, weight in self.components:
            scale = self.weights*weight/(1. if derivative == 3 else sum(seed.durations))
            delta, jac = self.base_fields[derivative]-wanted, self.jacobian[derivative]
            self.constant += float(np.einsum('n,nd,nd->', scale, delta, delta))
            self.gradient += 2*np.einsum('n,nd,ndk->k', scale, delta, jac)
            self.hessian += 2*np.einsum('n,ndk,ndl->kl', scale, jac, jac)
        self.hessian += np.diag(.002/self.radii**2)
        self.proxy = TrackingMincoObjective(problem, seed, MincoConfig(
            mode='q', jerk_weight=0., follow_weight=0., visibility_weight=20.,
            dynamic_weight=100., safety_weight=1000., yaw_weight=0.),
            reference=reference, yaw_optimize=False)
        yaw = YawTrajectory(np.r_[0., np.cumsum(seed.durations)], seed.yaw,
                            start_rate=getattr(problem.request, 'reference_yaw_rate', 0.))
        self.yaw, self.rate = yaw.sample(self.times)
        self.target = target

    def coefficients(self, x):
        return self.base+np.einsum('pdak,k->pda', self.influence, x)

    def seed_from(self, x):
        """Reject out-of-domain values rather than silently clipping a retained candidate."""
        x = np.asarray(x)
        if x.shape != self.radii.shape or not np.all(np.isfinite(x)) or np.any(
                abs(x) > self.radii+1e-9):
            raise ValueError('TERMINAL_BOUNDS')
        q, end = np.asarray(self.seed.q).copy(), np.asarray(self.seed.end).copy()
        for name, delta in zip(self.names, x.reshape(-1, 3)):
            if name.startswith('q'):
                q[int(name[-1])] += delta
            else:
                begin = 3 if name == 'v' else 6
                end[begin:begin+3] += delta
        return replace(self.seed, q=tuple(map(tuple, q)), end=tuple(end))

    def quadratic(self, x):
        x = np.asarray(x)
        return (float(self.constant+self.gradient@x+x@self.hessian@x/2),
                self.gradient+self.hessian@x)

    def fixed(self):
        """Small box quadratic active-set solve; no candidate search or new yaw/time variables."""
        x = np.clip(np.linalg.solve(self.hessian, -self.gradient), -self.radii, self.radii)
        for _ in range(12):
            gradient = self.gradient+self.hessian@x
            active = ((x <= -self.radii+1e-10) & (gradient >= 0)) | (
                (x >= self.radii-1e-10) & (gradient <= 0))
            free = ~active
            if not free.any():
                break
            candidate = x.copy()
            candidate[free] = np.linalg.solve(self.hessian[np.ix_(free, free)],
                                              -self.gradient[free]
                                              - self.hessian[np.ix_(free, active)]@x[active])
            candidate = np.clip(candidate, -self.radii, self.radii)
            if np.max(abs(candidate-x)) < 1e-10:
                break
            x = candidate
        return x if self.quadratic(x)[0] <= self.constant else np.zeros_like(x)

    def __call__(self, x):
        """Explicit FOLLOW/USV P,V, exact jerk, endpoint penalty and whole-sphere proxy."""
        values = self.base_fields+np.einsum('dnck,k->dnc', self.jacobian, x)
        args = [*values, self.yaw, self.rate, self.target]
        scale = self.weights/sum(self.seed.durations)
        value, gradient = self.quadratic(x)
        value += float(scale@self.proxy.local_cost(*args))
        partials = self.proxy.local_partials(args)
        gradient = gradient+sum(np.einsum('n,nc,nck->k', scale, partials[d], self.jacobian[d])
                                for d in range(4))
        if not np.isfinite(value) or not np.all(np.isfinite(gradient)):
            raise ValueError('NONFINITE_TERMINAL_OBJECTIVE')
        return value, gradient

    def jerk_peak(self, x, axes=(0, 1)):
        """Envelope derivative at the exact polynomial norm maximum, including interior roots."""
        xyz = self.coefficients(x)
        peak = derivative_peak(xyz, self.seed.durations, 3, axes)
        basis = MincoS3Trajectory._basis(peak['local_time'], 3)
        jerk = basis@xyz[peak['piece']]
        jac = np.einsum('d,dak->ak', basis, self.influence[peak['piece']])
        gradient = jerk[list(axes)]@jac[list(axes)]/max(peak['value'], 1e-12)
        return peak['value'], gradient, peak

    def repair(self, initial, deadline=float('inf'), clock=time.perf_counter):
        """At most four low-dimensional SQP iterations with exact jerk-envelope constraints."""
        incumbent = [float('inf'), np.asarray(initial).copy()]

        def evaluate(x):
            if clock() >= deadline:
                raise TimeoutError()
            value = self(x)
            if value[0] < incumbent[0]:
                incumbent[:] = [value[0], np.asarray(x).copy()]
            return value

        def constraint(x, axes, limit):
            if clock() >= deadline:
                raise TimeoutError()
            value, gradient, _ = self.jerk_peak(x, axes)
            return limit-value, -gradient

        constraints = []
        for axes, limit in (((0, 1), self.problem.limits.maximum_horizontal_jerk),
                            ((2,), self.problem.limits.maximum_vertical_jerk)):
            constraints.append(dict(type='ineq',
                                    fun=lambda x, a=axes, l=limit: constraint(x, a, l)[0],
                                    jac=lambda x, a=axes, l=limit: constraint(x, a, l)[1]))
        evaluate(initial)
        solved = minimize(evaluate, initial, jac=True, bounds=self.bounds, method='SLSQP',
                          constraints=constraints, options=dict(maxiter=4, ftol=1e-5))
        # Constraint repair may increase the unconstrained objective; validation decides.
        selected = solved.x if np.all(np.isfinite(solved.x)) else incumbent[1]
        return selected, dict(status=str(solved.message), iterations=int(solved.nit),
                              objective_initial=float(self(initial)[0]),
                              objective_final=float(self(selected)[0]),
                              success=bool(solved.success))

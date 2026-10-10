"""Opt-in shared/batched five-candidate search; independent P43 full admission."""
from types import SimpleNamespace
import math

import numpy as np
from scipy.interpolate import CubicSpline

from uav_control.controllers.p43_follow_solver import P43FollowSolver
from uav_control.controllers.progress_follow_solver import optimal_local_jerk
from uav_control.controllers.short_follow_solver import local_seed
from uav_control.guidance.fast_minco_objective import FastMincoObjective
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_profile import stage, count
from uav_control.guidance.follow_reference import (
    forecast_target_kinematics, viewpoint_state, viewpoint_state_batch,
)
from uav_control.guidance.yaw_trajectory import YawTrajectory


class P44FollowSolver(P43FollowSolver):
    """B shared only, C batch, D lazy, E reorder, F conservative invariant math."""

    def __init__(self, model, ablation='D', refinement='none', **kwargs):
        """Retain full five-row fallback population, P43 timing and publication contract."""
        if ablation not in 'BCDEF' or len(ablation) != 1:
            raise ValueError('INVALID_P44_ABLATION')
        super().__init__(model, refinement=refinement, **kwargs)
        self.ablation, self.mode = ablation, 'p44_adaptive'
        if ablation == 'F':
            from uav_control.guidance.p44_invariant_cache import P44InvariantCache
            self.invariant_cache = P44InvariantCache()

    def candidates(self, request, problem, old):
        """Equivalent definitions/clipping/stable indices; proxies never reject a row."""
        h, limits = self.duration, self.config
        if self.ablation == 'B':
            # Shared forecast only: original scalar jerk, PVA, seed, spline and scoring.
            with stage('candidate_reference'):
                tp, tv, ta, heading, omega, alpha, details = forecast_target_kinematics(problem, h)
                references = {
                    b: (*viewpoint_state(tp, tv, ta, heading, omega, alpha,
                                         distance=limits.follow_distance, beta=b,
                                         altitude=limits.flight_altitude), heading+b, details)
                    for b in (0., -.6, .6)}
            return super().candidates(request, problem, old, references)
        p, v, a = np.asarray(request.state[:9]).reshape(3, 3)
        beta = np.array((0., 0., 0., -.6, .6))
        scales = np.array((1., .5, 0., 1., 1.))
        query = np.array((0., h/2, h))
        with stage('candidate_reference'):
            tp, tv, ta, heading, omega, alpha, details = forecast_target_kinematics(problem, h)
            rp, rv, ra = viewpoint_state_batch(tp, tv, ta, heading, omega, alpha,
                                               np.full(5, limits.follow_distance), beta,
                                               limits.flight_altitude)
            ra[:, :2] *= np.minimum(1., limits.maximum_horizontal_acceleration/np.maximum(
                np.linalg.norm(ra[:, :2], axis=1), 1e-12))[:, None]
            targets, _ = problem.target_state(query)
        with stage('candidate_jerk'):
            if self.ablation == 'B':
                j = np.array([optimal_local_jerk(request.state[:9], rp[i], rv[i], h,
                                                 self.weights, old, ra[i])*scales[i]
                              for i in range(5)])
            else:
                w = self.weights
                w.validate()
                kp, kv = h**3/6, h*h/2
                wp, wv, wa, wj, ws = (w.position/25, w.velocity/9, w.acceleration/9,
                                      w.jerk/36, w.switching/36)
                denominator = wp*kp*kp+wv*kv*kv+wa*h*h+wj*h+ws
                j = -(wp*kp*(p+v*h+a*h*h/2-rp)+wv*kv*(v+a*h-rv)+wa*h*(a-ra)-ws*old)
                j = j/denominator*scales[:, None]
            j[:, :2] *= np.minimum(1., limits.maximum_horizontal_jerk/np.maximum(
                np.linalg.norm(j[:, :2], axis=1), 1e-12))[:, None]
            j[:, 2] = np.clip(j[:, 2], -limits.maximum_vertical_jerk, limits.maximum_vertical_jerk)
            end_a = a+j*h
            end_a[:, :2] *= np.minimum(1., limits.maximum_horizontal_acceleration/np.maximum(
                np.linalg.norm(end_a[:, :2], axis=1), 1e-12))[:, None]
            end_a[:, 2] = np.clip(end_a[:, 2], -limits.maximum_vertical_acceleration,
                                  limits.maximum_vertical_acceleration)
            j = (end_a-a)/h
        with stage('candidate_yaw'):
            # Match scalar atan2 boundary convention; unwrap each spline independently.
            delta = heading+beta-request.state[9]
            rates = np.array([np.clip(math.atan2(math.sin(x), math.cos(x))/h,
                                      -limits.maximum_yaw_rate, limits.maximum_yaw_rate)
                              for x in delta])
        lazy = self.ablation in 'DEF'
        with stage('candidate_seed_construction'):
            seeds = ([None]*5 if lazy else [local_seed(request, h, j[i], rates[i],
                                                       self.maximum_horizon) for i in range(5)])
            # Geometry uses only calibration and duration from this lightweight carrier.
            carrier = (SimpleNamespace(durations=(h/3,)*3) if lazy else
                       local_seed(request, h, np.zeros(3), 0., self.maximum_horizon))
            proxy = FastMincoObjective(problem, carrier, MincoConfig(mode='q'))
        with stage('candidate_proxy_geometry'):
            t = query[None, :, None]
            cp = p+v*t+a*t**2/2+j[:, None, :]*t**3/6
            cv = v+a*t+j[:, None, :]*t**2/2
            ca = a+j[:, None, :]*t
            knots = np.r_[0., np.cumsum((h/3,)*3)]
            if self.ablation == 'B':
                margins = []
                for i in range(5):
                    ys = YawTrajectory(knots, seeds[i].yaw,
                                       start_rate=getattr(request, 'reference_yaw_rate', 0.))
                    angles, _ = ys.sample(query)
                    margins.append(proxy.geometry(cp[i], ca[i], angles, targets)[1])
                margin = np.asarray(margins)
            else:
                # local_seed uses arange(1,4)*h/3 (not cumsum); keep this rounding order.
                yaw_knots = request.state[9]+rates[None, :]*np.r_[0., np.arange(1, 4)*h/3][:, None]
                start_rates = np.full(5, getattr(request, 'reference_yaw_rate', 0.))
                ys = CubicSpline(knots, np.unwrap(yaw_knots, axis=0), axis=0,
                                 bc_type=((1, start_rates),
                                          (1, np.zeros(5))))
                angles = ys(query).T
                margin = proxy.geometry(cp.reshape(-1, 3), ca.reshape(-1, 3), angles.ravel(),
                                        np.tile(targets, (5, 1)))[1].reshape(5, 3, -1)
        with stage('candidate_proxy_score'):
            costs = (4*np.sum((cp[:, -1, :2]-rp[:, :2])**2, axis=1)/25
                     + 2*np.sum((cv[:, -1, :2]-rv[:, :2])**2, axis=1)/9
                     + np.sum((ca[:, -1]-ra)**2, axis=1)/9
                     + .03*h*np.sum(j*j, axis=1)/36
                     + .01*(beta**2+np.sum((j-old)**2, axis=1)/36)
                     + .05*np.mean(np.maximum(.1-margin, 0.)**2/.01, axis=(1, 2)))
            if not np.all(np.isfinite(np.r_[j.ravel(), costs, rates])):
                raise ValueError('INVALID_PROGRESS_STATE')
        count('coarse_candidates', 5)
        count('coarse_geometry_calls', 5 if self.ablation == 'B' else 1)
        return [(int(beta[i] != 0.), costs[i], seeds[i], beta[i], j[i],
                 dict(details, scale=scales[i], candidate_index=i, yaw_rate=rates[i],
                      end_acceleration=tuple(end_a[i])))
                for i in range(5)]

    def reorder(self, remaining, tested):
        """Only reprioritize original untried descriptors; unknown failures keep order."""
        if self.ablation not in 'EF':
            return remaining
        status = tested.solver_status
        if status == 'DYNAMIC_INFEASIBLE':
            return sorted(remaining, key=lambda r: np.linalg.norm(r[4]))
        if status == 'VISIBILITY_INFEASIBLE':
            return sorted(remaining, key=lambda r: int(r[3] == 0.))
        if status == 'YAW_RATE_LIMIT':
            return sorted(remaining, key=lambda r: abs(r[5]['yaw_rate']))
        if status == 'ATTITUDE_LIMIT':
            return sorted(remaining, key=lambda r: np.linalg.norm(r[5]['end_acceleration']))
        return remaining

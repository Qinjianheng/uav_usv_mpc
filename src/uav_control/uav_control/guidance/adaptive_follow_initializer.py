"""Bounded research viewpoint candidates with a separately labelled free terminal strategy."""
from dataclasses import dataclass
import math
import time

import numpy as np

from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_reference import (
    forecast_viewpoint, viewpoint_state, viewpoint_state_batch,
)
from uav_control.guidance.greedy_follow_initializer import FollowSeed, GreedyFollowInitializer


@dataclass(frozen=True)
class GreedyConfig:
    """A/B/C/D: rear, three fixed angles, adaptive angle, angle plus distance."""

    strategy: str = 'D'
    dynamic_endpoint: bool = True
    duration: float = 2.4
    minimum_distance: float = 3.
    maximum_distance: float = 10.

    batch: bool = False
    two_level: bool = False
    feasible_first: bool = False
    maximum_angles: int = 5
    maximum_distances: int = 3
    start_score: bool = False
    detailed: bool = True

    def validate(self):
        """Only forecast-covered bounded horizons and implemented strategies are accepted."""
        if (not 1 <= self.maximum_angles <= 5 or not 1 <= self.maximum_distances <= 3
                or self.strategy not in ('A', 'B', 'C', 'D') or not 2.4 <= self.duration <= 2.8
                or not np.all(np.isfinite((self.minimum_distance, self.maximum_distance)))
                or not 0 < self.minimum_distance <= 5. <= self.maximum_distance <= 10.):
            raise ValueError('INVALID_GREEDY_CONFIG')


class AdaptiveFollowInitializer:
    """At most 15 candidates per knot, one-step endpoint lookahead, bounded fallback."""

    def __init__(self, model, config=GreedyConfig()):
        """Keep explicit research parameters; do not alter camera/range/dynamics limits."""
        config.validate()
        self.model, self.config = model, config

    def build(self, request, previous_direction=0., common_endpoint=None,
              deadline=None, clock=time.perf_counter):
        """Record all candidates, penalties and rejections; only final validation means safe."""
        c = self.config
        if deadline is not None and clock() >= deadline:
            return FollowSeed(request.context, reason='INITIALIZATION_DEADLINE')
        try:
            problem = FollowProblem(request, self.model, c.duration)
            times = np.arange(1, 4)*c.duration/3
            dt = c.duration/3
            prev_p, prev_v, prev_yaw = np.asarray(request.state[:3]), np.asarray(
                request.state[3:6]), request.state[9]
            direction = previous_direction
            points, yaws, directions, candidates = [], [prev_yaw], [], []
            selected_kinematics = None
            count = precise_count = 0
            for index, t in enumerate(times):
                if deadline is not None and clock() >= deadline:
                    return FollowSeed(request.context, reason='INITIALIZATION_DEADLINE')
                tp, tv = problem.target_state((t,))
                heading = math.atan2(tv[0, 1], tv[0, 0]) if np.linalg.norm(tv[0, :2]) >= .2 else (
                    request.state[9])
                relative = math.atan2(tp[0, 1]-prev_p[1], tp[0, 0]-prev_p[0])-heading
                relative = math.atan2(math.sin(relative), math.cos(relative))
                angle = float(np.clip(relative, -.9, .9))
                _, _, base_a, _, fit = forecast_viewpoint(problem, t)
                e = np.array((np.cos(heading), np.sin(heading), 0.))
                n = np.array((-e[1], e[0], 0.))
                target_a = (base_a - problem.limits.follow_distance*fit['heading_rate']**2*e
                            + problem.limits.follow_distance*fit['heading_acceleration']*n)
                turn = float(np.clip(fit['heading_rate']*dt, -.6, .6))
                angles = ((0.,) if c.strategy == 'A' else (0., -.6, .6) if c.strategy == 'B'
                          else tuple(sorted(set((0., direction, angle, -turn, turn)))))
                nominal = problem.limits.follow_distance
                horizontal_gap = float(np.linalg.norm((tp[0]-prev_p)[:2]))
                # Distance is a strategy variable, never a relaxed camera constraint.
                distances = ((nominal,) if c.strategy != 'D' else tuple(sorted(set((nominal,
                             float(np.clip(horizontal_gap, c.minimum_distance,
                                           c.maximum_distance)),
                             float(np.clip(nominal+np.linalg.norm(tv[0, :2])*abs(turn)*dt,
                                           c.minimum_distance, c.maximum_distance)))))))
                # Limits change candidate enumeration only in explicit P3.2 modes.
                angles = tuple(sorted(angles, key=lambda x: (abs(x), x)))[:c.maximum_angles] if (
                    c.maximum_angles < 5) else angles
                distances = tuple(sorted(distances, key=lambda x: abs(x-nominal)))[
                    :c.maximum_distances] if c.maximum_distances < 3 else distances
                parameters = [(beta, d) for beta in angles for d in distances]
                if c.batch:
                    ps, vs, acs = viewpoint_state_batch(
                        tp[0], tv[0], target_a, heading, fit['heading_rate'],
                        fit['heading_acceleration'], np.array([d for _, d in parameters]),
                        np.array([beta for beta, _ in parameters]), problem.limits.flight_altitude)
                else:
                    states = [viewpoint_state(tp[0], tv[0], target_a, heading,
                                              fit['heading_rate'], fit['heading_acceleration'],
                                              distance=d, beta=beta,
                                              altitude=problem.limits.flight_altitude)
                              for beta, d in parameters]
                    ps, vs, acs = (np.array([state[i] for state in states]) for i in range(3))
                ys = np.array([heading+beta for beta, _ in parameters])
                ys += 2*math.pi*np.round((prev_yaw-ys)/(2*math.pi))
                cvs = (ps-prev_p)/dt
                cas = (cvs-prev_v)/dt
                # Integral of a bounded jerk: ||p-p0-v0*t-a0*t^2/2|| <= J*t^3/6.
                # It is necessary for this fixed waypoint/time, not a proof about all seeds.
                residual = ps-np.asarray(request.state[:3])-np.asarray(request.state[3:6])*t-(
                    np.asarray(request.state[6:9])*t*t/2)
                reach_ratio = np.linalg.norm(residual[:, :2], axis=1)/(
                    problem.limits.maximum_horizontal_jerk*t**3/6)
                reachable = (reach_ratio <= 1.+1e-8) & (np.abs(residual[:, 2]) <= (
                    problem.limits.maximum_vertical_jerk*t**3/6+1e-8))
                indices = np.flatnonzero(reachable) if c.two_level and np.any(reachable) else (
                    np.arange(len(parameters)))
                assessments = {}
                if c.batch:
                    m, views, geometry, _ = problem.assess(
                        np.full(len(indices), t), ps[indices], cvs[indices], cas[indices],
                        np.zeros((len(indices), 3)), ys[indices], (ys[indices]-prev_yaw)/dt,
                        batch=True)
                    for row, k in enumerate(indices):
                        assessments[k] = ({key: float(value[row]) for key, value in m.items()},
                                          views[row].whole_target_safe, geometry[row])
                else:
                    for k in indices:
                        m, views, geometry, _ = problem.assess(
                            (t,), ps[k:k+1], cvs[k:k+1], cas[k:k+1], np.zeros((1, 3)),
                            ys[k:k+1], (ys[k:k+1]-prev_yaw)/dt)
                        assessments[k] = ({key: float(value[0]) for key, value in m.items()},
                                          views[0].whole_target_safe, geometry[0])
                count += len(parameters)
                precise_count += len(indices)
                options = []
                for k, (beta, distance) in enumerate(parameters):
                    p, v, a = ps[k], vs[k], acs[k]
                    yaw = ys[k]
                    if k in assessments:
                        margins, safe, geometry = assessments[k]
                        violations = {key: max(0., -value) for key, value in margins.items()}
                        if not safe:
                            violations['visibility'] = 1.+max(0., -float(np.min(geometry)))
                    else:
                        violations = {'necessary_reachability': float(reach_ratio[k])}
                        geometry = np.full(4, -math.pi)
                    reach = np.linalg.norm(p-prev_p-prev_v*dt)**2
                    mismatch = np.linalg.norm(cvs[k]-v)**2
                    cost = (reach+.3*mismatch+.1*np.linalg.norm(a)**2
                            + 2*(beta-direction)**2+.1*(yaw-prev_yaw)**2
                            + 100*sum(value**2 for value in violations.values()))
                    if c.start_score:
                        cost += 10.*max(0., float(reach_ratio[k])-1.)**2
                    item = dict(knot=index+1, time=float(t), beta=float(beta),
                                distance=float(distance), p=p.tolist(), v=v.tolist(),
                                a=a.tolist(), yaw=float(yaw), cost=float(cost),
                                violations=violations, fov=geometry.tolist(),
                                feasible_coarse=max(violations.values()) <= 1e-6,
                                reference_model=fit, selected=False)
                    if c.two_level or c.start_score:
                        item.update(necessary_reachable=bool(reachable[k]),
                                    reachability_ratio=float(reach_ratio[k]),
                                    precise_evaluated=k in assessments)
                    if c.detailed:
                        candidates.append(item)
                    options.append((cost, item, (p, v, a)))
                _, chosen, selected_kinematics = min(options, key=lambda item: (
                    not item[1]['feasible_coarse'], item[0]) if c.feasible_first else item[0])
                if not c.detailed:
                    candidates.append(chosen)
                chosen['selected'] = True
                # A coarse unsafe candidate is an explicit fallback, never a valid result.
                direction, prev_p, prev_v, prev_yaw = (chosen['beta'], np.array(chosen['p']),
                                                       np.array(chosen['v']), chosen['yaw'])
                points.append(tuple(prev_p))
                yaws.append(prev_yaw)
                directions.append(direction)
            end = np.concatenate(selected_kinematics)
            if not c.dynamic_endpoint:
                ref, tv, _ = problem.reference((c.duration,), direction)
                end = np.r_[points[-1], tv[0, :2], 0., 0., 0., 0.]
            if common_endpoint is not None:
                end = np.asarray(common_endpoint)
            if deadline is not None and clock() >= deadline:
                return FollowSeed(request.context, reason='INITIALIZATION_DEADLINE')
            metrics_extra = dict(candidate_count=count, precise_count=precise_count) if (
                c.batch or c.two_level or c.feasible_first or c.start_score
                or c.maximum_angles < 5 or c.maximum_distances < 3 or not c.detailed) else {}
            return FollowSeed(request.context, True, 'COARSE_SEED', tuple(request.state),
                              tuple(end), tuple(points[:2]), (dt, dt, dt), tuple(yaws),
                              tuple(directions), dict(**metrics_extra, candidates=candidates,
                                                      strategy=c.strategy,
                                                      fallback=any(i['selected'] and not
                                                                   i['feasible_coarse']
                                                                   for i in candidates),
                                                      terminal_policy='common' if common_endpoint
                                                      is not None else 'dynamic' if
                                                      c.dynamic_endpoint else 'legacy'))
        except (ValueError, TypeError, OverflowError) as error:
            # Use the established simple seed only at the established horizon.
            fallback = GreedyFollowInitializer(self.model).build(request, method='simple')
            if c.duration == 2.4 and fallback.valid_input:
                from dataclasses import replace
                return replace(fallback, metrics=dict(fallback=True, reason=str(error)))
            return FollowSeed(request.context, reason=str(error))

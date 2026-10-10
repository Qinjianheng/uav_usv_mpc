"""Bounded research viewpoint candidates with a separately labelled free terminal strategy."""
from dataclasses import dataclass
import math

import numpy as np

from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_reference import forecast_viewpoint, viewpoint_state
from uav_control.guidance.greedy_follow_initializer import FollowSeed, GreedyFollowInitializer


@dataclass(frozen=True)
class GreedyConfig:
    """A/B/C/D: rear, three fixed angles, adaptive angle, angle plus distance."""

    strategy: str = 'D'
    dynamic_endpoint: bool = True
    duration: float = 2.4

    def validate(self):
        """Only forecast-covered bounded horizons and implemented strategies are accepted."""
        if self.strategy not in ('A', 'B', 'C', 'D') or not 2.4 <= self.duration <= 2.8:
            raise ValueError('INVALID_GREEDY_CONFIG')


class AdaptiveFollowInitializer:
    """At most 15 candidates per knot, one-step endpoint lookahead, bounded fallback."""

    def __init__(self, model, config=GreedyConfig()):
        """Keep explicit research parameters; do not alter camera/range/dynamics limits."""
        config.validate()
        self.model, self.config = model, config

    def build(self, request, previous_direction=0., common_endpoint=None):
        """Record all candidates, penalties and rejections; only final validation means safe."""
        c = self.config
        try:
            problem = FollowProblem(request, self.model, c.duration)
            times = np.arange(1, 4)*c.duration/3
            dt = c.duration/3
            prev_p, prev_v, prev_yaw = np.asarray(request.state[:3]), np.asarray(
                request.state[3:6]), request.state[9]
            direction = previous_direction
            points, yaws, directions, candidates = [], [prev_yaw], [], []
            selected_kinematics = None
            for index, t in enumerate(times):
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
                             float(np.clip(horizontal_gap, 3., 10.)),
                             float(np.clip(nominal+np.linalg.norm(tv[0, :2])*abs(turn)*dt,
                                           3., 10.)))))))
                options = []
                for beta in angles:
                    for distance in distances:
                        p, v, a = viewpoint_state(
                            tp[0], tv[0], target_a, heading, fit['heading_rate'],
                            fit['heading_acceleration'], distance=distance, beta=beta,
                            altitude=problem.limits.flight_altitude)
                        yaw, diagnostic = heading+beta, fit
                        yaw += 2*math.pi*round((prev_yaw-yaw)/(2*math.pi))
                        coarse_v = (p-prev_p)/dt
                        coarse_a = (coarse_v-prev_v)/dt
                        m, views, geometry, _ = problem.assess(
                            (t,), p[None], coarse_v[None], coarse_a[None], np.zeros((1, 3)),
                            np.array([yaw]), np.array([(yaw-prev_yaw)/dt]))
                        violations = {k: max(0., -float(x[0])) for k, x in m.items()}
                        if not views[0].whole_target_safe:
                            violations['visibility'] = 1.+max(0., -float(np.min(geometry)))
                        # Endpoint lookahead includes the actual chosen reference velocity.
                        reach = np.linalg.norm(p-prev_p-prev_v*dt)**2
                        mismatch = np.linalg.norm(coarse_v-v)**2
                        cost = (reach+.3*mismatch+.1*np.linalg.norm(a)**2
                                + 2*(beta-direction)**2+.1*(yaw-prev_yaw)**2
                                + 100*sum(value**2 for value in violations.values()))
                        item = dict(knot=index+1, time=float(t), beta=float(beta),
                                    distance=float(distance), p=p.tolist(), v=v.tolist(),
                                    a=a.tolist(), yaw=float(yaw), cost=float(cost),
                                    violations=violations, fov=geometry[0].tolist(),
                                    feasible_coarse=max(violations.values()) <= 1e-6,
                                    reference_model=diagnostic, selected=False)
                        candidates.append(item)
                        options.append((cost, item, (p, v, a)))
                _, chosen, selected_kinematics = min(options, key=lambda item: item[0])
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
            return FollowSeed(request.context, True, 'COARSE_SEED', tuple(request.state),
                              tuple(end), tuple(points[:2]), (dt, dt, dt), tuple(yaws),
                              tuple(directions), dict(candidates=candidates, strategy=c.strategy,
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

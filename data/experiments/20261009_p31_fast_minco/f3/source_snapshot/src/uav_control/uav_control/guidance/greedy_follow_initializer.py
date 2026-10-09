"""Sequential bounded viewpoint selection; coarse seeds never mean final safety."""

from dataclasses import dataclass, field
import math

import numpy as np

from uav_control.controllers.follow_mpc_seed import PlanningContext
from uav_control.guidance.follow_problem import FollowProblem


@dataclass(frozen=True)
class FollowSeed:
    """Complete fixed boundary and Q/T/yaw initialization for a three-piece MINCO."""

    context: PlanningContext
    valid_input: bool = False
    reason: str = ''
    start: tuple = ()
    end: tuple = ()
    q: tuple = ()
    durations: tuple = (.8, .8, .8)
    yaw: tuple = ()
    directions: tuple = ()
    metrics: dict = field(default_factory=dict)
    final_validated: bool = False


class GreedyFollowInitializer:
    """Select at most four candidates per knot, including a retained direction."""

    def __init__(self, model=None, switch_penalty=2.):
        """Keep explicit geometry and switching cost, without mutable ROS history."""
        self.model, self.switch_penalty = model, float(switch_penalty)

    def build(self, request, previous_direction=0., method='greedy'):
        """Produce Q/T and common PVA endpoints; a coarse failure has a specific reason."""
        try:
            problem = FollowProblem(request, self.model)
            times = np.array((.8, 1.6, 2.4))
            ref, target_v, ref_yaw = problem.reference(times)
            # Hold the same endpoint PVA across all initializer comparisons.
            # Constant flight altitude also requires zero terminal vertical velocity.
            end = (*ref[-1], *target_v[-1, :2], 0., 0., 0., 0.)
            if method == 'simple':
                yaw = np.unwrap(np.r_[request.state[9], ref_yaw])
                return FollowSeed(request.context, True, 'COARSE_SEED', tuple(request.state),
                                  tuple(end), tuple(map(tuple, ref[:2])), yaw=tuple(yaw),
                                  directions=(0., 0., 0.))
            if method != 'greedy' or not math.isfinite(previous_direction):
                raise ValueError('INVALID_INITIALIZER')
            previous_p = np.asarray(request.state[:3])
            previous_v = np.asarray(request.state[3:6])
            previous_yaw = request.state[9]
            direction = previous_direction
            chosen, yaws, directions, margins = [], [previous_yaw], [], []
            for index, time in enumerate(times):
                options = []
                for side in sorted(set((0., .6, -.6, direction))):
                    # The final endpoint remains common; directions affect only Q.
                    side = side if index < 2 else 0.
                    point, _, yaw = problem.reference((time,), side)
                    point = point[0]
                    yaw = float(yaw[0] + 2 * math.pi * round(
                        (previous_yaw - yaw[0]) / (2 * math.pi)))
                    velocity = (point - previous_p) / .8
                    acceleration = (velocity - previous_v) / .8
                    margin, views, geometry, _ = problem.assess(
                        (time,), point[None, :], velocity[None, :], acceleration[None, :],
                        np.zeros((1, 3)), np.array([yaw]),
                        np.array([(yaw - previous_yaw) / .8]))
                    if (min(float(np.min(v)) for v in margin.values()) < -1e-6
                            or not views[0].whole_target_safe):
                        continue
                    cost = (np.linalg.norm(point - previous_p - previous_v * .8)**2
                            + self.switch_penalty * (side - direction)**2
                            + .1 * (yaw - previous_yaw)**2
                            - .2 * min(geometry[0, :2]))
                    options.append((cost, side, point, yaw, velocity, geometry[0, :2]))
                if not options:
                    raise ValueError('NO_REACHABLE_VISIBLE_CANDIDATE')
                _, direction, previous_p, previous_yaw, previous_v, view_margin = min(
                    options, key=lambda item: item[0])
                chosen.append(tuple(previous_p))
                yaws.append(previous_yaw)
                directions.append(direction)
                margins.append(tuple(view_margin))
            return FollowSeed(request.context, True, 'COARSE_SEED', tuple(request.state),
                              tuple(end), tuple(chosen[:2]), yaw=tuple(yaws),
                              directions=tuple(directions),
                              metrics={'visibility_margins': margins})
        except (ValueError, TypeError, OverflowError) as error:
            return FollowSeed(request.context, reason=str(error))

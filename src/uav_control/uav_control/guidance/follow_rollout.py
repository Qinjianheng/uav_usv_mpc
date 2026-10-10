"""Forecast-only original FOLLOW commands and an explicit virtual velocity response."""
from dataclasses import dataclass
import math
import time

import numpy as np

from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState


@dataclass(frozen=True)
class FollowRolloutConfig:
    """Research response assumptions, not an identified PX4 dynamics model."""

    position_gain: float = .8
    altitude_gain: float = 1.
    control_dt: float = .05
    integration_dt: float = .01
    response_time: float = .5

    def validate(self):
        if (not all(math.isfinite(v) and v > 0 for v in (
                self.position_gain, self.altitude_gain, self.control_dt,
                self.integration_dt, self.response_time))
                or self.integration_dt > self.control_dt):
            raise ValueError('INVALID_FOLLOW_ROLLOUT_CONFIG')


def _bound(vector, horizontal, vertical):
    value = np.asarray(vector, dtype=float).copy()
    value[:2] *= min(1., horizontal/max(math.hypot(*value[:2]), 1e-12))
    value[2] = np.clip(value[2], -vertical, vertical)
    return value


@dataclass(frozen=True)
class FollowReference:
    """Piecewise constant-jerk virtual P/V/A; commands are separate from velocity."""

    times: tuple
    positions: tuple
    velocities: tuple
    accelerations: tuple
    commands: tuple
    metrics: dict

    def sample(self, time):
        if not math.isfinite(time) or not 0 <= time <= self.times[-1]+1e-9:
            raise ValueError('FOLLOW_REFERENCE_OUTSIDE')
        index = min(np.searchsorted(self.times, time, side='right')-1, len(self.times)-2)
        dt = time-self.times[index]
        p, v, a = (np.asarray(field[index]) for field in (
            self.positions, self.velocities, self.accelerations))
        jerk = (np.asarray(self.accelerations[index+1])-a)/(
            self.times[index+1]-self.times[index])
        return p+v*dt+a*dt**2/2+jerk*dt**3/6, v+a*dt+jerk*dt**2/2, a+jerk*dt


def rollout_follow(problem, config=FollowRolloutConfig(), initial_command=None,
                   deadline=None, clock=time.perf_counter):
    """
    Run a private unchanged FlightGuidanceCore, then integrate a lagged virtual UAV.

    a_des=(v_cmd-v)/tau, acceleration clipped; acceleration slews at bounded jerk.
    Each integration interval uses exact constant-jerk P/V/A integration. Commands
    are held at the original 20 Hz period. No ROS, measured future, or truth source.
    """
    config.validate()
    limits, req, duration = problem.limits, problem.request, problem.duration
    if (duration/config.control_dt > 10000 or duration/config.integration_dt > 10000):
        raise ValueError('FOLLOW_ROLLOUT_STEP_LIMIT')

    def check_deadline():
        if deadline is not None and (not math.isfinite(deadline) or clock() >= deadline):
            raise ValueError('DEADLINE_EXCEEDED')

    check_deadline()
    p, v, a = np.asarray(req.state[:9], dtype=float).reshape(3, 3).copy()
    source = 'provided' if initial_command is not None else 'execution_velocity_initialization'
    command = (_bound(v, limits.maximum_horizontal_speed, limits.maximum_vertical_speed)
               if initial_command is None else np.asarray(initial_command, dtype=float))
    clipped = initial_command is None and not np.array_equal(command, v)
    if (command.shape != (3,) or not np.all(np.isfinite(command))
            or np.linalg.norm(command[:2]) > limits.maximum_horizontal_speed+1e-6
            or abs(command[2]) > limits.maximum_vertical_speed+1e-6):
        raise ValueError('INVALID_FOLLOW_INITIAL_COMMAND')
    core = FlightGuidanceCore(
        flight_altitude=limits.flight_altitude, follow_distance=limits.follow_distance,
        follow_position_gain=config.position_gain, altitude_velocity_gain=config.altitude_gain,
        maximum_horizontal_speed=limits.maximum_horizontal_speed,
        maximum_vertical_speed=limits.maximum_vertical_speed,
        maximum_horizontal_acceleration=limits.maximum_horizontal_acceleration,
        maximum_vertical_acceleration=limits.maximum_vertical_acceleration,
        sea_surface_z=limits.sea_surface_z, reserve_clearance=limits.reserve_clearance,
        safety_response_delay=limits.response_delay,
        vertical_braking_acceleration=limits.braking_acceleration, control_dt=config.control_dt)
    core.previous_velocity = tuple(command)
    control_times = np.arange(0., duration-1e-10, config.control_dt)
    target_p, target_v = problem.target_state(control_times)
    times, positions, velocities, accelerations, commands = [0.], [tuple(p)], [tuple(v)], [
        tuple(a)], []
    safety_states = []
    for i, start in enumerate(control_times):
        check_deadline()
        target = FlightKinematicState(tuple(target_p[i]), tuple(target_v[i]))
        output = core.command('FOLLOW', FlightKinematicState(tuple(p), tuple(v)), target,
                              config.control_dt)
        command = np.asarray(output.velocity)
        commands.append(tuple(command))
        safety_states.append(output.safety_state)
        stop = min(start+config.control_dt, duration)
        while times[-1] < stop-1e-10:
            check_deadline()
            dt = min(config.integration_dt, stop-times[-1])
            desired_a = _bound((command-v)/config.response_time,
                               limits.maximum_horizontal_acceleration,
                               limits.maximum_vertical_acceleration)
            delta_a = _bound(desired_a-a, limits.maximum_horizontal_jerk*dt,
                             limits.maximum_vertical_jerk*dt)
            jerk = delta_a/dt
            p = p+v*dt+a*dt**2/2+jerk*dt**3/6
            v = v+a*dt+jerk*dt**2/2
            a = a+delta_a
            times.append(float(stop if abs(times[-1]+dt-stop) < 1e-10 else times[-1]+dt))
            positions.append(tuple(p))
            velocities.append(tuple(v))
            accelerations.append(tuple(a))
    return FollowReference(tuple(times), tuple(positions), tuple(velocities),
                           tuple(accelerations), tuple(commands), dict(
                               command_source=source, response_model='jerk_limited_velocity_lag',
                               command_initialization_clipped=bool(clipped),
                               response_time=config.response_time, control_dt=config.control_dt,
                               integration_dt=config.integration_dt,
                               future_target_source=req.context.prediction_source,
                               prediction_sequence_id=req.context.prediction_sequence_id,
                               flight_altitude=limits.flight_altitude,
                               safety_states=tuple(sorted(set(safety_states)))))

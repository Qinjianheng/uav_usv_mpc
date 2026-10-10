"""
Shared research prediction, boundary and constraint evaluation without ROS.

Original acquisition epochs are immutable. A future boundary is explicitly
predicted with constant acceleration, never called an accepted tracker state.
"""
from uav_control.guidance.follow_profile import profiled


from dataclasses import dataclass, replace
import math

import numpy as np

from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcRequest
from uav_control.guidance.camera_visibility import evaluate_visibility
from uav_control.guidance.planned_attitude import planned_attitude


@dataclass(frozen=True)
class FutureRequest(MpcRequest):
    """Store physical measurement separately from a projected planning boundary."""

    measurement_state: tuple = ()
    boundary_policy: str = 'constant_acceleration_projection'


def future_request(request, lead=.15):
    """Project to now+lead while retaining all original observation/source/sample stamps."""
    if not math.isfinite(lead) or not 0 <= lead <= .3:
        raise ValueError('INVALID_EXECUTION_LEAD')
    if isinstance(request, FutureRequest):
        raise ValueError('BOUNDARY_ALREADY_PROJECTED')
    x = np.asarray(request.state, dtype=float)
    if x.shape != (10,) or not np.all(np.isfinite(x)):
        raise ValueError('INVALID_INPUT')
    epoch = request.now_stamp + lead
    dt = epoch - request.context.navigation_stamp
    if not 0 <= dt <= .425:
        raise ValueError('INVALID_BOUNDARY_PROJECTION')
    state = x.copy()
    state[:3] += x[3:6] * dt + x[6:9] * dt**2 / 2
    state[3:6] += x[6:9] * dt
    payload = dict(request.__dict__)
    payload.update(context=replace(request.context, execution_start_stamp=epoch),
                   state=tuple(state), measurement_state=tuple(x))
    return FutureRequest(**payload)


@dataclass(frozen=True)
class HypotheticalRequest(FutureRequest):
    """Explicit offline ideal-model boundary; never Tracker acceptance or measured navigation."""

    prior_curve: object = None
    reference_yaw_rate: float = 0.
    boundary_policy: str = 'hypothetical_previous_curve'


def hypothetical_request(request, previous_curve, epoch):
    """Sample only this rollout's own previous coefficients, retaining logged raw epochs."""
    sample = previous_curve.sample(epoch)
    payload = dict(request.__dict__)
    payload.update(context=replace(request.context, execution_start_stamp=epoch),
                   state=tuple(sample[:10]), measurement_state=tuple(getattr(
                       request, 'measurement_state', ()) or request.state),
                   boundary_policy='hypothetical_previous_curve', prior_curve=previous_curve,
                   reference_yaw_rate=sample[10])
    return HypotheticalRequest(**payload)


class FollowProblem:
    """Use the P2 camera configuration and the P1 evaluator for all new research modes."""

    def __init__(self, request, model=None, duration=2.4):
        """Validate physical input, future-boundary provenance, and prediction coverage."""
        self.model = model or FollowMpcSeed()
        self.limits = self.model.config
        self.request, self.duration = request, float(duration)
        c = request.context
        stamps = (request.now_stamp, c.execution_start_stamp, c.navigation_stamp,
                  c.attitude_stamp, c.prediction_source_stamp, c.observation_stamp,
                  c.prediction_valid_until)
        self.state = np.asarray(request.state, dtype=float)
        self.times = np.asarray(request.prediction_times, dtype=float)
        self.positions = np.asarray(request.target_positions, dtype=float)
        self.velocities = np.asarray(request.target_velocities, dtype=float)
        rotation = np.asarray(request.actual_rotation, dtype=float)
        if (not all(math.isfinite(s) and s > 0 for s in stamps) or c.mission_id < 1
                or c.frame_id != 'local_ned'
                or c.prediction_source not in ('tracking', 'synthetic')
                or (c.prediction_source == 'synthetic'
                    and not self.limits.allow_synthetic_predictions)
                or self.state.shape != (10,) or not np.all(np.isfinite(self.state))
                or self.times.ndim != 1 or len(self.times) < 2
                or not np.all(np.isfinite(self.times)) or self.times[0] < 0
                or np.any(np.diff(self.times) <= 0)
                or self.positions.shape != (len(self.times), 3)
                or self.velocities.shape != self.positions.shape
                or not np.all(np.isfinite(self.positions))
                or not np.all(np.isfinite(self.velocities)) or rotation.shape != (3, 3)
                or not np.all(np.isfinite(rotation))
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
                or not math.isclose(np.linalg.det(rotation), 1., abs_tol=1e-7)
                or abs(c.navigation_stamp - c.attitude_stamp) > 1e-7
                or c.navigation_stamp > request.now_stamp + 1e-7
                or c.observation_stamp > c.prediction_source_stamp + 1e-7
                or c.prediction_source_stamp > request.now_stamp + 1e-7):
            raise ValueError('INVALID_INPUT')
        if (max(request.now_stamp - c.navigation_stamp,
                request.now_stamp - c.observation_stamp) > self.limits.maximum_input_age
                or request.now_stamp >= c.prediction_valid_until):
            raise ValueError('STALE_INPUT')
        if isinstance(request, HypotheticalRequest):
            if (request.boundary_policy != 'hypothetical_previous_curve'
                    or request.prior_curve is None or len(request.measurement_state) != 10
                    or not np.allclose(request.prior_curve.sample(c.execution_start_stamp)[:10],
                                       self.state, rtol=0., atol=1e-9)
                    or abs(request.prior_curve.sample(c.execution_start_stamp)[10]
                           - request.reference_yaw_rate) > 1e-9):
                raise ValueError('INVALID_HYPOTHETICAL_BOUNDARY')
        elif abs(c.execution_start_stamp - c.navigation_stamp) > 1e-7:
            if (not isinstance(request, FutureRequest) or len(request.measurement_state) != 10
                    or request.boundary_policy != 'constant_acceleration_projection'):
                raise ValueError('UNPROVEN_EXECUTION_BOUNDARY')
            measured = np.asarray(request.measurement_state, dtype=float)
            dt = c.execution_start_stamp - c.navigation_stamp
            expected = measured.copy()
            expected[:3] += measured[3:6] * dt + measured[6:9] * dt**2 / 2
            expected[3:6] += measured[6:9] * dt
            if (not 0 <= c.execution_start_stamp - request.now_stamp <= .300001
                    or not np.allclose(expected, self.state, rtol=0., atol=1e-9)):
                raise ValueError('INVALID_BOUNDARY_PROJECTION')
        self.target_state((0., self.duration))

    @profiled('target_interpolation')
    def target_state(self, relative_times):
        """Interpolate only inside the original source-stamped prediction window."""
        query = (np.asarray(relative_times) + self.request.context.execution_start_stamp
                 - self.request.context.prediction_source_stamp)
        if (not np.all(np.isfinite(query)) or np.min(query) < self.times[0] - 1e-8
                or np.max(query) > self.times[-1] + 1e-8):
            raise ValueError('PREDICTION_HORIZON')
        p = np.column_stack([
            np.interp(query, self.times, self.positions[:, i]) for i in range(3)])
        v = np.column_stack([
            np.interp(query, self.times, self.velocities[:, i]) for i in range(3)])
        return p, v

    def reference(self, relative_times, direction=0.):
        """Return rear/side-rear viewpoints without copying target heave into UAV altitude."""
        p, v = self.target_state(relative_times)
        heading = np.arctan2(v[:, 1], v[:, 0])
        heading[np.linalg.norm(v[:, :2], axis=1) < .2] = self.state[9]
        heading = np.unwrap(heading) + direction
        ref = p.copy()
        ref[:, :2] -= self.limits.follow_distance * np.c_[np.cos(heading), np.sin(heading)]
        ref[:, 2] = self.limits.flight_altitude
        return ref, v, heading

    @profiled('other_constraints')
    def assess(self, times, p, v, a, j, yaw, rate, batch=False):
        """Evaluate physical margins and original whole-target geometry at common times."""
        targets, _ = self.target_state(times)
        views, tilts, thrusts = [], [], []
        if batch:
            from uav_control.guidance.camera_visibility_batch import (
                attitude_batch, visibility_batch,
            )
            attitudes = attitude_batch(a, yaw, self.model.attitude_config)
            tilts = [w.tilt_rad if w.tilt_rad is not None else math.pi for w in attitudes]
            thrusts = [w.specific_thrust or 0. for w in attitudes]
            rotations = np.array([w.rotation_frd_to_ned if w.valid else
                                  np.full((3, 3), np.nan) for w in attitudes])
            if not isinstance(self.request, FutureRequest):
                rotations[np.asarray(times) == 0.] = self.request.actual_rotation
            views = visibility_batch(p, rotations, targets, self.model.intrinsics,
                                     self.model.extrinsics, self.model.target,
                                     self.model.visibility)
        else:
            for i in range(len(times)):
                attitude = planned_attitude(a[i], yaw[i], self.model.attitude_config)
                tilts.append(attitude.tilt_rad if attitude.tilt_rad is not None
                             else math.pi)
                thrusts.append(attitude.specific_thrust or 0.)
                rotation = attitude.rotation_frd_to_ned
                if times[i] == 0. and not isinstance(self.request, FutureRequest):
                    rotation = self.request.actual_rotation
                views.append(evaluate_visibility(
                    p[i], rotation, targets[i], self.model.intrinsics,
                    self.model.extrinsics, self.model.target, self.model.visibility))
        c = self.limits
        descent = np.maximum(v[:, 2], 0.)
        margin = {
            'horizontal_speed': c.maximum_horizontal_speed - np.linalg.norm(v[:, :2], axis=1),
            'vertical_speed': c.maximum_vertical_speed - np.abs(v[:, 2]),
            'horizontal_acceleration': (c.maximum_horizontal_acceleration
                                        - np.linalg.norm(a[:, :2], axis=1)),
            'vertical_acceleration': c.maximum_vertical_acceleration - np.abs(a[:, 2]),
            'horizontal_jerk': c.maximum_horizontal_jerk - np.linalg.norm(j[:, :2], axis=1),
            'vertical_jerk': c.maximum_vertical_jerk - np.abs(j[:, 2]),
            'yaw_rate': c.maximum_yaw_rate - np.abs(rate),
            'sea_clearance': (c.sea_surface_z - c.reserve_clearance - p[:, 2]
                              - descent * c.response_delay
                              - descent**2 / (2 * c.braking_acceleration)),
            'tilt': self.model.attitude_config.maximum_tilt_rad - np.asarray(tilts),
            'minimum_thrust': (np.asarray(thrusts)
                               - self.model.attitude_config.minimum_specific_thrust),
            'maximum_thrust': (self.model.attitude_config.maximum_specific_thrust
                               - np.asarray(thrusts)),
        }
        h = np.array([w.horizontal_margin_rad if w.horizontal_margin_rad is not None else -math.pi
                      for w in views])
        vertical = np.array([w.vertical_margin_rad if w.vertical_margin_rad is not None
                             else -math.pi for w in views])
        near = np.array([(w.distance_interval_m or (-1e3, 1e3))[0]
                         - self.model.visibility.minimum_distance for w in views])
        far = np.array([self.model.visibility.maximum_distance
                       - (w.distance_interval_m or (-1e3, 1e3))[1] for w in views])
        return margin, views, np.c_[h, vertical, near, far], np.asarray(tilts)

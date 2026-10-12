"""
Bounded research MPC seed generation; no ROS, PX4 commands or flight authority.

NED z is down. Jerk/yaw-rate are optimization inputs, not actuator commands.
Every optimizer output is checked against physical limits and original P1 geometry.
"""

from dataclasses import dataclass, field, replace
import math
import time

import numpy as np
from scipy.optimize import minimize

from uav_control.guidance.camera_visibility import (
    CameraExtrinsics, CameraIntrinsics, CameraVisibilityResult,
    TargetBoundingSphere, VisibilityConstraints,
    evaluate_visibility,
)
from uav_control.guidance.planned_attitude import AttitudeConfig, planned_attitude


@dataclass(frozen=True)
class MpcConfig:
    """Research defaults restricted to the current FOLLOW/tracker envelope."""

    dt: float = .2
    horizon_steps: int = 12
    control_blocks: int = 2
    follow_distance: float = 5.0
    flight_altitude: float = -5.0
    maximum_horizontal_speed: float = 6.2
    maximum_vertical_speed: float = 4.0
    maximum_horizontal_acceleration: float = 3.0
    maximum_vertical_acceleration: float = 3.0
    maximum_horizontal_jerk: float = 6.0
    maximum_vertical_jerk: float = 4.0
    maximum_yaw_rate: float = 1.0
    sea_surface_z: float = 0.0
    reserve_clearance: float = .5
    response_delay: float = .15
    braking_acceleration: float = 2.5
    maximum_input_age: float = .125
    solve_budget: float = .5
    maximum_iterations: int = 4
    validation_substeps: int = 2
    allow_synthetic_predictions: bool = False

    def validate(self):
        """Reject unsafe configuration rather than widening current flight limits."""
        integers = (self.horizon_steps, self.control_blocks, self.maximum_iterations,
                    self.validation_substeps)
        if (any(isinstance(v, bool) or not isinstance(v, int) or v < 1 for v in integers)
                or self.control_blocks > self.horizon_steps):
            raise ValueError('invalid integer MPC dimensions')
        positives = (self.dt, self.follow_distance, self.maximum_horizontal_speed,
                     self.maximum_vertical_speed, self.maximum_horizontal_acceleration,
                     self.maximum_vertical_acceleration, self.maximum_horizontal_jerk,
                     self.maximum_vertical_jerk, self.maximum_yaw_rate, self.reserve_clearance,
                     self.braking_acceleration, self.maximum_input_age, self.solve_budget)
        if (not all(math.isfinite(v) and v > 0 for v in positives)
                or not all(math.isfinite(v) for v in (
                    self.flight_altitude, self.sea_surface_z, self.response_delay))
                or self.response_delay < 0
                or self.maximum_horizontal_speed > 6.2 or self.maximum_vertical_speed > 4
                or self.maximum_horizontal_acceleration > 3
                or self.maximum_vertical_acceleration > 3 or self.maximum_yaw_rate > 1
                or self.flight_altitude > self.sea_surface_z - self.reserve_clearance):
            raise ValueError('invalid or widened MPC limits')


@dataclass(frozen=True)
class PlanningContext:
    """Input provenance and common MPC/MINCO execution epoch, independent of receipt."""

    mission_id: int
    cycle_id: int
    execution_start_stamp: float
    navigation_stamp: float
    attitude_stamp: float
    prediction_source_stamp: float
    observation_stamp: float
    prediction_sequence_id: int
    prediction_valid_until: float
    clock_generation: int = 0
    frame_id: str = 'local_ned'
    prediction_source: str = 'tracking'


@dataclass(frozen=True)
class MpcRequest:
    """Same-epoch navigation pose with one immutable target prediction snapshot."""

    context: PlanningContext
    state: tuple
    prediction_times: tuple
    target_positions: tuple
    target_velocities: tuple
    actual_rotation: tuple
    now_stamp: float


@dataclass(frozen=True)
class MpcSeedResult:
    """Research seed admission; valid never means accepted by a flight tracker."""

    context: PlanningContext
    valid: bool = False
    solver_status: str = 'INVALID_INPUT'
    reason: str = ''
    relative_times: tuple = ()
    positions: tuple = ()
    velocities: tuple = ()
    accelerations: tuple = ()
    jerks: tuple = ()
    yaw_refs: tuple = ()
    yaw_rates: tuple = ()
    visibility_margins: tuple = ()
    solve_time: float = 0.0
    timing: dict = field(default_factory=dict)
    constraint_violations: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    warm_started: bool = False
    initial_visibility: bool = False
    candidate_kind: str = ''
    objective_value: float = 0.0


def integrate_state(state, control, dt):
    """Exactly integrate a constant NED jerk and unwrapped yaw rate over dt."""
    x, u = np.asarray(state, dtype=float), np.asarray(control, dtype=float)
    if (x.shape != (10,) or u.shape != (4,) or not np.all(np.isfinite(x))
            or not np.all(np.isfinite(u)) or not math.isfinite(dt) or dt <= 0):
        raise ValueError('invalid integration input')
    result = x.copy()
    result[:3] += x[3:6] * dt + x[6:9] * dt**2 / 2 + u[:3] * dt**3 / 6
    result[3:6] += x[6:9] * dt + u[:3] * dt**2 / 2
    result[6:9] += u[:3] * dt
    result[9] += u[3] * dt
    return result


def rollout(initial_state, controls, dt):
    """Predict states including the exact initial P/V/A/yaw boundary."""
    states = [np.asarray(initial_state, dtype=float)]
    for control in controls:
        states.append(integrate_state(states[-1], control, dt))
    return np.asarray(states)


def _limit_horizontal(vector, limit):
    vector = np.asarray(vector, dtype=float).copy()
    norm = np.linalg.norm(vector[:2])
    if norm > limit:
        vector[:2] *= limit / norm
    return vector


class _AdmissionError(Exception):
    def __init__(self, status, reason):
        self.status, self.reason = status, reason


class _Deadline(Exception):
    pass


class FollowMpcSeed:
    """Move-blocked nonlinear finite-horizon MPC with strict post-solve admission."""

    def __init__(self, config=MpcConfig(), intrinsics=None, extrinsics=None, target=None,
                 visibility=None, attitude_config=AttitudeConfig(), clock=time.perf_counter):
        """Use explicit current simulation calibration defaults; never read live data."""
        config.validate()
        self.config, self.clock, self.attitude_config = config, clock, attitude_config
        self.intrinsics = intrinsics or CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
        self.extrinsics = extrinsics or CameraExtrinsics.from_sdf_pose(
            (.18, 0, .39), 0, math.radians(28), 0)
        self.target = target or TargetBoundingSphere(.25, (0, 0, -.42))
        margin = .04 + attitude_config.angular_error_margin_rad
        self.visibility = visibility or VisibilityConstraints(.05, 25, margin, margin)
        check = evaluate_visibility((0, 0, -5), np.eye(3), (5, 0, 0), self.intrinsics,
                                    self.extrinsics, self.target, self.visibility)
        if not check.valid_input or not planned_attitude((0, 0, 0), 0, attitude_config).valid:
            raise ValueError('invalid camera or attitude configuration')
        self.last_valid = None
        self._block_indices = np.arange(config.horizon_steps) * config.control_blocks // (
            config.horizon_steps)

    def clear_warm_start(self):
        """Invalidate research warm state on mission/clock changes or explicit reset."""
        self.last_valid = None

    def _prepare(self, request):
        c, ctx = self.config, request.context
        stamps = (request.now_stamp, ctx.execution_start_stamp, ctx.navigation_stamp,
                  ctx.attitude_stamp, ctx.prediction_source_stamp, ctx.observation_stamp,
                  ctx.prediction_valid_until)
        x = np.asarray(request.state, dtype=float)
        times = np.asarray(request.prediction_times, dtype=float)
        positions = np.asarray(request.target_positions, dtype=float)
        velocities = np.asarray(request.target_velocities, dtype=float)
        actual_rotation = np.asarray(request.actual_rotation, dtype=float)
        if (not all(math.isfinite(v) and v > 0 for v in stamps) or ctx.mission_id < 1
                or ctx.frame_id != 'local_ned'
                or ctx.prediction_source not in ('tracking', 'synthetic')
                or (ctx.prediction_source == 'synthetic' and not c.allow_synthetic_predictions)
                or x.shape != (10,) or not np.all(np.isfinite(x))
                or times.ndim != 1 or times.size < 2 or not np.all(np.isfinite(times))
                or np.any(np.diff(times) <= 0) or times[0] < 0
                or positions.shape != (times.size, 3) or velocities.shape != positions.shape
                or not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities))
                or actual_rotation.shape != (3, 3)
                or not np.all(np.isfinite(actual_rotation))
                or abs(ctx.navigation_stamp - ctx.execution_start_stamp) > 1e-7
                or abs(ctx.attitude_stamp - ctx.execution_start_stamp) > 1e-7
                or ctx.observation_stamp > ctx.prediction_source_stamp + 1e-7
                or ctx.prediction_source_stamp > request.now_stamp
                or ctx.navigation_stamp > request.now_stamp):
            raise _AdmissionError('INVALID_INPUT', 'input shape, source, frame or epoch mismatch')
        if (max(request.now_stamp - ctx.navigation_stamp,
                request.now_stamp - ctx.observation_stamp)
                > c.maximum_input_age or request.now_stamp >= ctx.prediction_valid_until):
            raise _AdmissionError('STALE_INPUT', 'navigation or image acquisition expired')
        query = ctx.execution_start_stamp - ctx.prediction_source_stamp + np.arange(
            c.horizon_steps + 1) * c.dt
        if query[0] < times[0] - 1e-8 or query[-1] > times[-1] + 1e-8:
            raise _AdmissionError('PREDICTION_HORIZON',
                                  'prediction does not cover execution horizon')
        targets = np.column_stack([np.interp(query, times, positions[:, i]) for i in range(3)])
        target_v = np.column_stack([np.interp(query, times, velocities[:, i]) for i in range(3)])
        check = self._view(x, targets[0], request.actual_rotation)
        if not check.valid_input:
            raise _AdmissionError('INVALID_INPUT', 'invalid measured attitude/calibration')
        return x, targets, target_v, check.whole_target_safe

    def _view(self, state, target, actual_rotation=None):
        attitude = planned_attitude(state[6:9], state[9], self.attitude_config)
        rotation = actual_rotation if actual_rotation is not None else attitude.rotation_frd_to_ned
        if rotation is None:
            return CameraVisibilityResult(reasons=('ATTITUDE_MODEL_INVALID', attitude.reason))
        return evaluate_visibility(state[:3], rotation, target, self.intrinsics, self.extrinsics,
                                   self.target, self.visibility)

    def _references(self, x, targets, target_v, side):
        heading = np.arctan2(target_v[:, 1], target_v[:, 0])
        slow = np.linalg.norm(target_v[:, :2], axis=1) < .2
        fallback = math.atan2(targets[0, 1] - x[1], targets[0, 0] - x[0])
        heading[slow] = fallback
        directions = np.column_stack((np.cos(heading + side), np.sin(heading + side)))
        reference = targets.copy()
        reference[:, :2] -= self.config.follow_distance * directions
        reference[:, 2] = self.config.flight_altitude
        yaw = np.unwrap(np.arctan2(
            targets[:, 1] - reference[:, 1], targets[:, 0] - reference[:, 0]))
        yaw += 2 * math.pi * round((x[9] - yaw[0]) / (2 * math.pi))
        return reference, yaw

    def _initial_controls(self, x, reference, target_v, yaw):
        c = self.config
        blocks = np.zeros((c.control_blocks, 4))
        state = x.copy()
        for block in range(c.control_blocks):
            indices = np.flatnonzero(self._block_indices == block)
            duration = len(indices) * c.dt
            index = int(indices[-1]) + 1
            desired = .7 * (reference[index] - state[:3]) + 1.2 * (target_v[index] - state[3:6])
            desired = _limit_horizontal(desired, c.maximum_horizontal_acceleration)
            desired[2] = np.clip(desired[2], -c.maximum_vertical_acceleration,
                                 c.maximum_vertical_acceleration)
            jerk = _limit_horizontal((desired - state[6:9]) / duration, c.maximum_horizontal_jerk)
            jerk[2] = np.clip(jerk[2], -c.maximum_vertical_jerk, c.maximum_vertical_jerk)
            rate = np.clip((yaw[index] - state[9]) / duration,
                           -c.maximum_yaw_rate, c.maximum_yaw_rate)
            blocks[block] = (*jerk, rate)
            state = integrate_state(state, blocks[block], duration)
        return blocks

    def _warm_controls(self, request):
        old, ctx, c = self.last_valid, request.context, self.config
        if (old is None or not old.valid or old.context.mission_id != ctx.mission_id
                or old.context.clock_generation != ctx.clock_generation
                or ctx.execution_start_stamp < old.context.execution_start_stamp
                or ctx.execution_start_stamp >= (
                    old.context.execution_start_stamp + old.relative_times[-1])
                or ctx.prediction_source_stamp < old.context.prediction_source_stamp):
            return None
        old_controls = np.column_stack((old.jerks, old.yaw_rates))
        shifted = []
        for k in range(c.horizon_steps):
            absolute = ctx.execution_start_stamp + k * c.dt
            index = int(np.searchsorted(
                old.relative_times, absolute - old.context.execution_start_stamp + 1e-9,
                side='right') - 1)
            shifted.append(old_controls[min(index, len(old_controls) - 1)])
        return np.array([np.mean(np.asarray(shifted)[self._block_indices == b], axis=0)
                         for b in range(c.control_blocks)])

    def _assessment(self, request, states, controls, targets, reference, dense=False):
        c = self.config
        if dense:
            dt = c.dt / c.validation_substeps
            controls = np.repeat(controls, c.validation_substeps, axis=0)
            states = rollout(request.state, controls, dt)
            absolute = request.context.execution_start_stamp + np.arange(len(states)) * dt
            relative = absolute - request.context.prediction_source_stamp
            original = np.asarray(request.target_positions)
            targets = np.column_stack([
                np.interp(relative, request.prediction_times, original[:, i]) for i in range(3)])
            reference = np.column_stack([
                np.interp(np.arange(len(states)) * dt,
                          np.arange(c.horizon_steps + 1) * c.dt, reference[:, i])
                for i in range(3)])
        views, attitudes = [], []
        for k, (state, target) in enumerate(zip(states, targets)):
            attitudes.append(planned_attitude(state[6:9], state[9], self.attitude_config))
            views.append(self._view(state, target, request.actual_rotation if k == 0 else None))
        speed = np.linalg.norm(states[:, 3:5], axis=1)
        acceleration = np.linalg.norm(states[:, 6:8], axis=1)
        descent = np.maximum(states[:, 5], 0)
        stopping = descent * c.response_delay + descent**2 / (2 * c.braking_acceleration)
        margins = {
            'horizontal_speed': c.maximum_horizontal_speed - speed,
            'vertical_speed': c.maximum_vertical_speed - np.abs(states[:, 5]),
            'horizontal_acceleration': c.maximum_horizontal_acceleration - acceleration,
            'vertical_acceleration': c.maximum_vertical_acceleration - np.abs(states[:, 8]),
            'horizontal_jerk': c.maximum_horizontal_jerk - np.linalg.norm(controls[:, :2], axis=1),
            'vertical_jerk': c.maximum_vertical_jerk - np.abs(controls[:, 2]),
            'yaw_rate': c.maximum_yaw_rate - np.abs(controls[:, 3]),
            'sea_clearance': c.sea_surface_z - c.reserve_clearance - states[:, 2] - stopping,
            'tilt': np.array([self.attitude_config.maximum_tilt_rad - (a.tilt_rad or 0)
                              for a in attitudes]),
            'minimum_thrust': np.array([
                (a.specific_thrust or 0) - self.attitude_config.minimum_specific_thrust
                for a in attitudes]),
            'maximum_thrust': np.array([self.attitude_config.maximum_specific_thrust
                                       - (a.specific_thrust or 0) for a in attitudes]),
            'attitude_model': np.array([1.0 if a.valid else -1.0 for a in attitudes]),
        }
        h = np.array([v.horizontal_margin_rad if v.horizontal_margin_rad is not None else -math.pi
                      for v in views])
        v = np.array([v.vertical_margin_rad if v.vertical_margin_rad is not None else -math.pi
                      for v in views])
        distance_near = np.array([(w.distance_interval_m or (-1e3, 1e3))[0]
                                  - self.visibility.minimum_distance for w in views])
        distance_far = np.array([self.visibility.maximum_distance
                                 - (w.distance_interval_m or (-1e3, 1e3))[1] for w in views])
        geometric = np.concatenate((h, v, distance_near, distance_far))
        dynamic = np.concatenate(tuple(margins.values()))
        return states, controls, views, attitudes, margins, dynamic, geometric, reference

    def _result(self, request, controls, targets, target_v, initial_visible, kind,
                reference=None, warm=False, objective=0.0):
        c = self.config
        states = rollout(request.state, controls, c.dt)
        if reference is None:
            reference, _ = self._references(states[0], targets, target_v, 0)
        coarse = self._assessment(request, states, controls, targets, reference)
        dense = self._assessment(request, states, controls, targets, reference, dense=True)
        ss, uu, views, attitudes, margins, dynamic, geometry, ref = dense
        violations = {key: max(0.0, float(-np.min(values))) for key, values in margins.items()}
        violations['visibility'] = max(0.0, float(-np.min(geometry)))
        dynamic_valid = bool(np.min(dynamic) >= -1e-6)
        visible = [w.whole_target_safe for w in views]
        safe = dynamic_valid and all(visible)
        if not dynamic_valid:
            status, reason = 'DYNAMIC_INFEASIBLE', 'strict dynamic/attitude/sea validation failed'
        elif safe:
            status, reason = 'SAFE', 'all sampled constraints passed'
        elif not initial_visible and all(visible[-min(3, len(visible)):]):
            status, reason = ('RECOVERY_CANDIDATE',
                              'initially invisible; only future reacquisition predicted')
        else:
            status, reason = 'VISIBILITY_INFEASIBLE', 'strict P1 whole-target view/range failed'
        residual = max(float(np.max(np.abs(integrate_state(states[k], controls[k], c.dt)
                                           - states[k + 1]))) for k in range(c.horizon_steps))
        metrics = {
            'dynamics_residual': residual, 'visible_fraction': float(np.mean(visible)),
            'minimum_horizontal_margin': float(min(
                w.horizontal_margin_rad if w.horizontal_margin_rad is not None else -math.pi
                for w in views)),
            'minimum_vertical_margin': float(min(
                w.vertical_margin_rad if w.vertical_margin_rad is not None else -math.pi
                for w in views)),
            'follow_rmse': float(np.sqrt(np.mean(np.sum((ss[:, :3] - ref)**2, axis=1)))),
            'maximum_horizontal_speed': float(np.max(np.linalg.norm(ss[:, 3:5], axis=1))),
            'maximum_vertical_speed': float(np.max(np.abs(ss[:, 5]))),
            'maximum_horizontal_acceleration': float(np.max(np.linalg.norm(ss[:, 6:8], axis=1))),
            'maximum_vertical_acceleration': float(np.max(np.abs(ss[:, 8]))),
            'maximum_horizontal_jerk': float(np.max(np.linalg.norm(uu[:, :2], axis=1))),
            'maximum_vertical_jerk': float(np.max(np.abs(uu[:, 2]))),
            'maximum_yaw_rate': float(np.max(np.abs(uu[:, 3]))),
            'maximum_tilt_rad': float(max(a.tilt_rad or 0 for a in attitudes)),
        }
        return MpcSeedResult(request.context, safe, status, reason,
                             tuple(float(k * c.dt) for k in range(c.horizon_steps + 1)),
                             tuple(map(tuple, states[:, :3])), tuple(map(tuple, states[:, 3:6])),
                             tuple(map(tuple, states[:, 6:9])), tuple(map(tuple, controls[:, :3])),
                             tuple(states[:, 9]), tuple(controls[:, 3]),
                             tuple((w.horizontal_margin_rad, w.vertical_margin_rad)
                                   for w in coarse[2]),
                             constraint_violations=violations, metrics=metrics, warm_started=warm,
                             initial_visibility=initial_visible, candidate_kind=kind,
                             objective_value=float(objective))

    def evaluate_controls(self, request, controls, candidate_kind='baseline'):
        """Admit a baseline through identical exact dynamics and dense geometry checks."""
        start = self.clock()
        try:
            _, targets, target_v, initial_visible = self._prepare(request)
            controls = np.asarray(controls, dtype=float)
            if (controls.shape != (self.config.horizon_steps, 4)
                    or not np.all(np.isfinite(controls))):
                raise _AdmissionError('INVALID_INPUT', 'controls must be finite horizon_steps x 4')
            result = self._result(request, controls, targets, target_v,
                                  initial_visible, candidate_kind)
        except _AdmissionError as error:
            result = MpcSeedResult(request.context, solver_status=error.status,
                                   reason=error.reason)
        except (ValueError, TypeError, OverflowError):
            result = MpcSeedResult(request.context, reason='invalid numeric input')
        return replace(result, solve_time=self.clock() - start)

    def solve(self, request):
        """Generate candidates, optimize controls and strictly validate before warm reuse."""
        start = self.clock()
        timing = {}
        warm_used = False

        def deadline():
            if self.clock() - start >= self.config.solve_budget:
                raise _Deadline()

        try:
            x, targets, target_v, initial_visible = self._prepare(request)
            deadline()
            timing['preparation'] = self.clock() - start
            # Only current boundary violations are irreparable; future zero-control limits are not.
            limits = self.config
            if (math.hypot(*x[3:5]) > limits.maximum_horizontal_speed + 1e-6
                    or abs(x[5]) > limits.maximum_vertical_speed + 1e-6
                    or math.hypot(*x[6:8]) > limits.maximum_horizontal_acceleration + 1e-6
                    or abs(x[8]) > limits.maximum_vertical_acceleration + 1e-6
                    or not planned_attitude(x[6:9], x[9], self.attitude_config).valid
                    or x[2] > limits.sea_surface_z - limits.reserve_clearance):
                current = self.evaluate_controls(
                    request, np.zeros((limits.horizon_steps, 4)))
                return replace(current, valid=False, solver_status='DYNAMIC_INFEASIBLE',
                               reason='initial boundary violates dynamic/model/sea constraints',
                               solve_time=self.clock() - start)
            init_start = self.clock()
            candidates = []
            for kind, side in (('rear', 0), ('left_rear', .6), ('right_rear', -.6)):
                ref, yaw = self._references(x, targets, target_v, side)
                candidates.append((kind, ref, yaw, self._initial_controls(x, ref, target_v, yaw)))
            warm = self._warm_controls(request)
            if warm is not None:
                ref, yaw = self._references(x, targets, target_v, 0)
                candidates.append(('warm', ref, yaw, warm))
            c = self.config

            def evaluate(blocks, ref, yaw):
                deadline()
                blocks = blocks.reshape(c.control_blocks, 4)
                controls = blocks[self._block_indices]
                states = rollout(x, controls, c.dt)
                assessed = self._assessment(request, states, controls, targets, ref)
                view_penalty = np.sum(np.logaddexp(0, -assessed[6] * 12)**2) / 144
                state_cost = np.mean(np.sum((states[:, :3] - ref)**2, axis=1))
                velocity_cost = np.mean(np.sum((states[:, 3:6] - target_v)**2, axis=1))
                yaw_cost = np.mean((states[:, 9] - yaw)**2)
                cost = (state_cost + .4 * velocity_cost + 2 * yaw_cost + 200 * view_penalty
                        + .03 * np.mean(states[:, 6:9]**2) + .02 * np.mean(blocks[:, :3]**2)
                        + .1 * np.mean(blocks[:, 3]**2) + .04 * np.sum(np.diff(blocks, axis=0)**2))
                constraint = assessed[5]
                if initial_visible:
                    constraint = np.concatenate((constraint, assessed[6]))
                return float(cost), constraint

            scored = [(evaluate(blocks.ravel(), ref, yaw)[0], kind, ref, yaw, blocks)
                      for kind, ref, yaw, blocks in candidates]
            _, kind, reference, yaw, initial = min(scored, key=lambda item: item[0])
            warm_used = kind == 'warm'
            timing['initialization'] = self.clock() - init_start
            # SLSQP asks objective and constraints for the same finite-difference
            # points. Retain their joint evaluations within this solve only.
            cache = {}

            def value(flat):
                deadline()
                key = flat.tobytes()
                if key not in cache:
                    if len(cache) >= 512:
                        cache.clear()
                    cache[key] = evaluate(flat, reference, yaw)
                return cache[key]

            bounds = [(-c.maximum_horizontal_jerk, c.maximum_horizontal_jerk)] * 2
            bounds += [(-c.maximum_vertical_jerk, c.maximum_vertical_jerk),
                       (-c.maximum_yaw_rate, c.maximum_yaw_rate)]
            solver_start = self.clock()
            solved = minimize(lambda flat: value(flat)[0], initial.ravel(), method='SLSQP',
                              bounds=bounds * c.control_blocks,
                              constraints=[{'type': 'ineq', 'fun': lambda flat: value(flat)[1]}],
                              options={'maxiter': c.maximum_iterations, 'ftol': 1e-4})
            deadline()
            timing['optimization'] = self.clock() - solver_start
            validation_start = self.clock()
            optimized_controls = solved.x.reshape(c.control_blocks, 4)[self._block_indices]
            optimized = self._result(request, optimized_controls,
                                     targets, target_v, initial_visible, kind, reference,
                                     warm=warm_used, objective=float(solved.fun))
            fallback = self._result(request, initial[self._block_indices], targets, target_v,
                                    initial_visible, kind, reference, warm=warm_used,
                                    objective=value(initial.ravel())[0])
            result = optimized
            if fallback.valid and (not optimized.valid
                                   or fallback.objective_value < optimized.objective_value):
                result = fallback
            metrics = dict(result.metrics, optimizer_success=bool(solved.success),
                           optimizer_status=int(solved.status),
                           optimizer_iterations=int(solved.nit),
                           optimizer_message=str(solved.message))
            result = replace(result, metrics=metrics)
            timing['validation'] = self.clock() - validation_start
            deadline()
            elapsed = self.clock() - start
            if elapsed >= c.solve_budget:
                raise _Deadline()
            timing['total'] = elapsed
            result = replace(result, solve_time=elapsed, timing=timing)
            if result.valid:
                self.last_valid = result
            return result
        except _AdmissionError as error:
            status, reason = error.status, error.reason
        except _Deadline:
            status, reason = ('DEADLINE_EXCEEDED',
                              'whole preparation/solve/validation budget expired')
        except (ValueError, TypeError, FloatingPointError, OverflowError) as error:
            status, reason = 'INVALID_INPUT', str(error)
        elapsed = self.clock() - start
        timing['total'] = elapsed
        return MpcSeedResult(request.context, solver_status=status, reason=reason,
                             solve_time=elapsed, timing=timing, warm_started=warm_used)


def to_minco_seed(result, waypoint_stride=4):
    """Export exact boundary P/V/A and common epoch, without optimizing MINCO."""
    if not result.valid or isinstance(waypoint_stride, bool) or waypoint_stride < 1:
        raise ValueError('only strictly safe seeds and positive waypoint stride can be exported')
    indices = [0, *range(waypoint_stride, len(result.relative_times) - 1, waypoint_stride),
               len(result.relative_times) - 1]
    return dict(context=result.context, start_position=result.positions[0],
                start_velocity=result.velocities[0], start_acceleration=result.accelerations[0],
                end_position=result.positions[-1], end_velocity=result.velocities[-1],
                end_acceleration=result.accelerations[-1],
                intermediate_positions=tuple(result.positions[k] for k in indices[1:-1]),
                durations=tuple(result.relative_times[b] - result.relative_times[a]
                                for a, b in zip(indices, indices[1:])),
                yaw_refs=result.yaw_refs, relative_times=result.relative_times)

"""Opt-in P6 uncertainty contract, local verification and reference selection."""
from collections import OrderedDict, deque
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from uav_control.guidance.robust_visibility import (
    RobustVisibilityConfig, evaluate_robust_visibility_batch,
)


@dataclass(frozen=True)
class P6Policy:
    """Receiver-owned policy; disabled defaults preserve the P5 contract exactly."""

    visibility_mode: str = 'nominal'
    adaptive_viewpoint: bool = False
    allow_nominal_fallback: bool = True
    risk_budget: float = .05
    risk_horizon_seconds: float = .45
    calibration_file: str = ''
    calibration_id: str = ''
    calibration_sha256: str = ''
    calibration_family: str = ''
    distribution: str = 'empirical_envelope'
    covariance_scale: float = 1.
    covariance_floor_m2: float = 0.
    empirical_scale: float = 1.
    yaw_error_bound_rad: float = .06
    attitude_error_bound_rad: float = .03
    extrinsic_rotation_bound_rad: float = 0.
    extrinsic_translation_bound_m: float = 0.
    minimum_distance: float = 4.5
    maximum_distance: float = 6.
    maximum_beta: float = math.radians(20.)
    maximum_distance_rate: float = .15
    maximum_beta_rate: float = .05
    version: str = 'p6.follow.v1'

    @property
    def enabled(self):
        return self.visibility_mode == 'robust' or self.adaptive_viewpoint

    def validate(self):
        values = [v for v in asdict(self).values() if isinstance(v, float)]
        if (not np.all(np.isfinite(values)) or self.visibility_mode not in ('nominal', 'robust')
                or not 0. < self.risk_budget < 1.
                or self.risk_horizon_seconds not in (0., .45)
                or self.distribution not in ('gaussian', 'moment_bound', 'empirical_envelope')
                or min(self.covariance_scale, self.empirical_scale) < 1.
                or self.covariance_floor_m2 < 0.
                or min(self.yaw_error_bound_rad, self.attitude_error_bound_rad,
                       self.extrinsic_rotation_bound_rad,
                       self.extrinsic_translation_bound_m) < 0.
                or not 0. < self.minimum_distance <= 5. <= self.maximum_distance
                or not 0. < self.maximum_beta < math.pi/2
                or min(self.maximum_distance_rate, self.maximum_beta_rate) <= 0.):
            raise ValueError('INVALID_P6_POLICY')
        if self.visibility_mode == 'robust' and not (
                self.calibration_id and self.calibration_sha256 and self.calibration_family):
            raise ValueError('P6_CALIBRATION_REQUIRED')
        return self

    def risk_config(self, count):
        return RobustVisibilityConfig(
            risk_budget=self.risk_budget, sample_count=count, distribution=self.distribution,
            calibrated=bool(self.calibration_sha256), calibration_id=self.calibration_id,
            covariance_scale=self.covariance_scale, empirical_scale=self.empirical_scale,
            yaw_error_bound_rad=self.yaw_error_bound_rad,
            attitude_error_bound_rad=self.attitude_error_bound_rad,
            extrinsic_rotation_bound_rad=self.extrinsic_rotation_bound_rad,
            extrinsic_translation_bound_m=self.extrinsic_translation_bound_m,
            error_bound_source='engineering_bounds_not_measured', attitude_source='planned')


def load_calibration(policy):
    """Bind exact offline evidence bytes; a filename alone is never calibration."""
    raw = Path(policy.calibration_file).read_bytes()
    data = json.loads(raw)
    if (data.get('schema') != 'p6-calibration-v1' or not data.get('id')
            or data.get('family') != 'imm'
            or not data.get('source_provenance') or not data.get('heldout_provenance')):
        raise ValueError('INVALID_P6_CALIBRATION')
    return replace(policy, calibration_id=data['id'],
                   calibration_sha256=hashlib.sha256(raw).hexdigest(),
                   calibration_family=data['family'], distribution=data['distribution'],
                   covariance_scale=float(data['covariance_scale']),
                   empirical_scale=float(data['empirical_scale']),
                   covariance_floor_m2=float(data.get('covariance_floor_m2', 0.))).validate()


def configure_p6(node, model):
    """Declare immutable local settings before constructing constraint fingerprints."""
    from rcl_interfaces.msg import ParameterDescriptor
    defaults = P6Policy()
    fields = asdict(defaults)
    derived = ('calibration_id', 'calibration_sha256', 'calibration_family',
               'covariance_scale', 'covariance_floor_m2', 'empirical_scale', 'distribution',
               'version')
    for key, value in fields.items():
        if key not in derived:
            node.declare_parameter('p6_'+key, value, ParameterDescriptor(read_only=True))
            fields[key] = node.get_parameter('p6_'+key).value
    policy = P6Policy(**fields)
    if policy.calibration_file:
        policy = load_calibration(policy)
    model.p6_policy = policy.validate()
    return P6Runtime(model)


def model_from_parameters(parameters):
    """Reconstruct the runtime contract for audit, without ROS or flight authority."""
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.follow_limits import ResearchConfig
    physical = asdict(ResearchConfig())
    physical.update({key: parameters[key] for key in physical if key in parameters})
    model = FollowMpcSeed(ResearchConfig(**physical))
    values = asdict(P6Policy())
    derived = ('calibration_id', 'calibration_sha256', 'calibration_family',
               'covariance_scale', 'covariance_floor_m2', 'empirical_scale', 'distribution',
               'version')
    for key in values:
        if key not in derived and 'p6_'+key in parameters:
            values[key] = parameters['p6_'+key]
    policy = P6Policy(**values)
    model.p6_policy = (load_calibration(policy) if policy.calibration_file
                       else policy.validate())
    return model


def _tuples(value):
    if isinstance(value, (list, tuple)):
        return tuple(_tuples(item) for item in value)
    return float(value)


def attach_model_diagnostics(prediction, diagnostic):
    """Attach conditionals only to their exact acquired/source prediction identity."""
    def wire_epoch(value):
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('P6_MODEL_IDENTITY_MISMATCH')
        second = math.floor(value)
        return second*1_000_000_000+round((value-second)*1e9)

    identity = ('mission_id', 'sequence_id', 'source_stamp', 'observation_stamp')
    time_fields = ('source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until')
    exact_fields = ('mission_id', 'sequence_id', 'frame_id', 'source', 'model')
    if (not diagnostic.get('valid')
            or any(diagnostic.get(key) != getattr(prediction, key) for key in exact_fields)
            or any(wire_epoch(diagnostic.get(key)) != wire_epoch(getattr(prediction, key))
                   for key in time_fields)
            or tuple(map(wire_epoch, diagnostic.get('relative_times', ())))
            != tuple(map(wire_epoch, prediction.prediction_times))):
        raise ValueError('P6_MODEL_IDENTITY_MISMATCH')
    models = diagnostic.get('model_predictions', [])
    positions = np.asarray([m['positions'] for m in models], dtype=float)
    covariances = np.asarray([m['position_covariances'] for m in models], dtype=float)
    weights = np.asarray(diagnostic.get('model_probabilities'), dtype=float)
    n = len(prediction.prediction_times)
    if (positions.shape != (3, n, 3) or covariances.shape != (3, n, 3, 3)
            or weights.shape != (3,) or not np.all(np.isfinite(positions))
            or not np.all(np.isfinite(covariances)) or np.any(weights < 0)
            or not math.isclose(float(weights.sum()), 1., abs_tol=1e-8)):
        raise ValueError('INVALID_P6_MODEL_DIAGNOSTICS')
    return replace(prediction, model_positions=_tuples(positions.tolist()),
                   model_covariances=_tuples(covariances.tolist()),
                   model_probabilities=tuple(weights), uncertainty_identity=tuple(
                       diagnostic[key] for key in identity))


def interpolate_uncertainty(prediction, query, policy):
    """Linear PSD interpolation inside coverage; preserve all model conditionals."""
    times = np.asarray(prediction.prediction_times, dtype=float)
    query = np.asarray(query, dtype=float)
    if (not np.all(np.isfinite(query)) or query.min() < times[0]-1e-8
            or query.max() > times[-1]+1e-8):
        raise ValueError('P6_UNCERTAINTY_HORIZON')
    if 'IMM' not in str(getattr(prediction, 'model', '')).upper():
        raise ValueError('P6_IMM_ONLY')
    family = 'imm'
    if family != policy.calibration_family:
        raise ValueError('P6_CALIBRATION_FAMILY_MISMATCH')

    def interpolate(values, tail):
        array = np.asarray(values, dtype=float)
        if tail == (3, 3) and array.shape == (len(times), 9):
            array = array.reshape(len(times), 3, 3)
        if array.shape != (len(times), *tail) or not np.all(np.isfinite(array)):
            raise ValueError('P6_UNCERTAINTY_MISSING_OR_INVALID')
        flat = array.reshape(len(times), -1)
        return np.column_stack([np.interp(query, times, flat[:, i])
                                for i in range(flat.shape[1])]).reshape(len(query), *tail)

    covariance = interpolate(prediction.target_covariances, (3, 3))
    covariance += np.eye(3)*policy.covariance_floor_m2
    models = getattr(prediction, 'model_positions', ())
    if family == 'imm' and not models:
        raise ValueError('P6_CONDITIONAL_PREDICTIONS_MISSING')
    kwargs = {}
    if models:
        kwargs = dict(model_positions=np.asarray([interpolate(m, (3,)) for m in models]),
                      model_covariances=np.asarray([interpolate(m, (3, 3))
                                                   + np.eye(3)*policy.covariance_floor_m2
                                                   for m in prediction.model_covariances]),
                      model_probabilities=np.asarray(prediction.model_probabilities))
    return covariance, kwargs


def assess_risk(model, prediction, query, positions, rotations, targets):
    """Evaluate local uncertainty with explicit fail-closed or nominal fallback semantics."""
    policy = getattr(model, 'p6_policy', P6Policy())
    if policy.visibility_mode != 'robust':
        return None, {}
    try:
        query = np.asarray(query, dtype=float)
        checked = np.arange(len(query))
        scope = 'configured_discrete_window_union_bound'
        if policy.risk_horizon_seconds:
            scope = 'execution_lease_prefix_discrete_samples'
            context = getattr(prediction, 'context', None)
            epoch = getattr(prediction, 'source_stamp', getattr(
                context, 'prediction_source_stamp', 0.))
            tolerance = max(1e-8, 2*math.ulp(epoch))
            boundary = min(float(query.max()), float(query.min())+policy.risk_horizon_seconds)
            if not np.any(abs(query-boundary) <= tolerance):
                raise ValueError('P6_RISK_ENDPOINT_NOT_SAMPLED')
            checked = np.flatnonzero(query <= boundary+tolerance)
        cov, kwargs = interpolate_uncertainty(prediction, query, policy)
        if kwargs:
            kwargs['model_positions'] = kwargs['model_positions'][:, checked]
            kwargs['model_covariances'] = kwargs['model_covariances'][:, checked]
        report = evaluate_robust_visibility_batch(
            np.asarray(positions)[checked], np.asarray(rotations)[checked],
            np.asarray(targets)[checked], cov[checked], model.intrinsics, model.extrinsics,
            model.target, model.visibility, policy.risk_config(len(query)), **kwargs)
        all_samples = (*report.samples,
                       *(s for branch in report.model_reports for s in branch.samples))
        fingerprint = hashlib.sha256((report.risk_config_fingerprint
                                      + ':'+str(policy.risk_horizon_seconds)).encode()).hexdigest()
        detail = dict(robust_ok=report.robust_ok, nominal_ok=report.nominal_ok,
                      valid_input=report.valid_input, reason=report.reason,
                      risk_config_fingerprint=fingerprint,
                      risk_kernel_fingerprint=report.risk_config_fingerprint,
                      risk_scope=scope, guarantee_type=report.guarantee_type,
                      checked_indices=tuple(map(int, checked)),
                      checked_relative_times=tuple(map(float, query[checked])),
                      entire_minco_curve_robust=not bool(policy.risk_horizon_seconds)
                      and report.robust_ok,
                      probability_bound_available=report.probability_bound_available,
                      continuous_time_guarantee=False,
                      calibration_sha256=policy.calibration_sha256,
                      used_nominal_fallback=not report.robust_ok and policy.allow_nominal_fallback,
                      minimum_horizontal_margin=min((s.horizontal_margin_rad
                                                     for s in all_samples), default=-math.pi),
                      minimum_vertical_margin=min((s.vertical_margin_rad
                                                   for s in all_samples), default=-math.pi))
        return report, detail
    except (ValueError, TypeError, KeyError, IndexError, np.linalg.LinAlgError) as error:
        return None, dict(robust_ok=False, valid_input=False, reason=str(error),
                          continuous_time_guarantee=False, probability_bound_available=False,
                          used_nominal_fallback=policy.allow_nominal_fallback)


class P6Runtime:
    """Latest immutable diagnostics and bounded persistent observation-point reference."""

    def __init__(self, model):
        from uav_control.guidance.adaptive_viewpoint import (
            AdaptiveViewpointConfig, AdaptiveViewpointSelector,
        )
        self.model, self.policy = model, model.p6_policy
        self.cache = OrderedDict()
        self.pending_diagnostics = deque(maxlen=16)
        self.last_stamp, self.last_mission = None, None
        self.selector = AdaptiveViewpointSelector(AdaptiveViewpointConfig(
            minimum_distance=self.policy.minimum_distance,
            maximum_distance=self.policy.maximum_distance,
            candidate_distances=(self.policy.minimum_distance, 5., self.policy.maximum_distance),
            candidate_betas=tuple(np.linspace(
                -self.policy.maximum_beta, self.policy.maximum_beta, 3)),
            maximum_beta=self.policy.maximum_beta,
            maximum_distance_rate=self.policy.maximum_distance_rate,
            maximum_beta_rate=self.policy.maximum_beta_rate))

    def diagnostic(self, message):
        """Retain bounded immutable payloads; BCTRA never consumes IMM model evidence."""
        payload = getattr(message, 'data', None)
        if isinstance(payload, str):
            self.pending_diagnostics.append(payload)

    def prediction(self, prediction):
        """Decode on IMM demand, then apply the unchanged strict model identity checks."""
        if 'IMM' not in str(getattr(prediction, 'model', '')).upper():
            return prediction
        while self.pending_diagnostics:
            try:
                data = json.loads(self.pending_diagnostics.popleft())
                key = (data['mission_id'], data['sequence_id'])
                self.cache[key] = data
                self.cache.move_to_end(key)
                while len(self.cache) > 16:
                    self.cache.popitem(last=False)
            except (ValueError, KeyError, TypeError):
                continue
        data = self.cache.get((prediction.mission_id, prediction.sequence_id))
        if data is None:
            return prediction
        try:
            return attach_model_diagnostics(prediction, data)
        except (ValueError, KeyError, TypeError):
            return prediction

    def prepare(self, request):
        """Choose from forecast-only candidates; never consult evaluator truth."""
        if not self.policy.adaptive_viewpoint:
            return request
        from uav_control.guidance.follow_problem import FollowProblem
        from uav_control.guidance.follow_reference import (
            forecast_target_kinematics, viewpoint_state_batch,
        )
        from uav_control.guidance.camera_visibility_batch import visibility_batch, attitude_batch
        problem = FollowProblem(request, self.model, 1.2)
        sample_times = (np.array((0., .225, .45, .8, 1.2)) if self.policy.risk_horizon_seconds
                        else np.array((0., .4, .8, 1.2)))
        sample_count = len(sample_times)
        targets, _ = problem.target_state(sample_times)
        kinetics = [forecast_target_kinematics(problem, t) for t in sample_times]
        grid = np.asarray(self.selector.candidates)
        states = [viewpoint_state_batch(
            *k[:6], grid[:, 0], grid[:, 1], problem.limits.flight_altitude) for k in kinetics]
        p = np.transpose([s[0] for s in states], (1, 0, 2)).reshape(-1, 3)
        a = np.transpose([s[2] for s in states], (1, 0, 2)).reshape(-1, 3)
        target_batch = np.tile(targets, (len(grid), 1))
        yaw = np.arctan2(target_batch[:, 1]-p[:, 1], target_batch[:, 0]-p[:, 0])
        attitudes = attitude_batch(a, yaw, self.model.attitude_config)
        rotations = np.asarray([s.rotation_frd_to_ned for s in attitudes])
        views = visibility_batch(p, rotations, target_batch, self.model.intrinsics,
                                 self.model.extrinsics, self.model.target, self.model.visibility)
        report, risk_details = assess_risk(self.model, request, np.tile(
            sample_times+request.context.execution_start_stamp
            - request.context.prediction_source_stamp, len(grid)), p, rotations, target_batch)
        margin = np.asarray([min(s.horizontal_margin_rad if s.horizontal_margin_rad is not None
                                 else -math.pi, s.vertical_margin_rad
                                 if s.vertical_margin_rad is not None else -math.pi)
                             for s in views])
        nominal_ok = np.asarray([s.whole_target_safe for s in views])
        robust_ok = np.zeros(len(p), dtype=bool)
        checked = np.zeros(len(p), dtype=bool)
        if report is not None and report.valid_input:
            indices = np.asarray(risk_details['checked_indices'], dtype=int)
            checked[indices] = True
            sets = (report.samples, *(m.samples for m in report.model_reports))
            margin[indices] = np.min([[min(s.horizontal_margin_rad, s.vertical_margin_rad)
                                      for s in sample_set] for sample_set in sets], axis=0)
            robust_ok[indices] = np.all([[s.robust_ok for s in sample_set]
                                        for sample_set in sets], axis=0)
        feasible = nominal_ok.reshape(-1, sample_count).all(axis=1)
        if self.policy.visibility_mode == 'robust' and not self.policy.allow_nominal_fallback:
            feasible &= (checked.reshape(-1, sample_count).any(axis=1)
                         & (~checked | robust_ok).reshape(-1, sample_count).all(axis=1))
        risks = np.maximum(0., .12-margin.reshape(-1, sample_count).min(axis=1))**2*100.
        risks[~feasible] = math.inf
        errors = (grid[:, 0]-5.)**2+grid[:, 1]**2
        shaped = a.reshape(-1, sample_count, 3)
        acceleration_cost = np.mean(np.sum(shaped**2, axis=2), axis=1)
        jerk_cost = np.mean(np.sum((np.diff(shaped, axis=1)
                                   / np.diff(sample_times)[None, :, None])**2, axis=2), axis=1)
        mission = (request.context.mission_id, request.context.clock_generation)
        if mission != self.last_mission or self.last_stamp is None or (
                request.now_stamp <= self.last_stamp):
            self.selector = type(self.selector)(self.selector.config)
            dt = .01
        else:
            dt = request.now_stamp-self.last_stamp
            if dt > self.selector.config.maximum_dt:
                self.last_stamp = None
                raise ValueError('P6_VIEWPOINT_CLOCK_GAP')
        self.last_stamp, self.last_mission = request.now_stamp, mission
        report = self.selector.select(
            dt, risks, candidate_errors=errors, candidate_accelerations=acceleration_cost,
            candidate_jerks=jerk_cost)
        if not report.valid:
            raise ValueError('P6_VIEWPOINT_'+report.reason)
        profile = self.selector.reference_profile()
        payload = (profile['duration'], profile['elapsed'],
                   tuple(profile['distance_coefficients']), tuple(profile['beta_coefficients']))
        return replace(request, viewpoint_profile=payload)

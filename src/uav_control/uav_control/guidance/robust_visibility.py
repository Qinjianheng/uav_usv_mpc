"""
Pure whole-sphere visibility under explicit conditional uncertainty models.

Admission uses support functions of a covariance ellipsoid plus the target sphere,
not a local angular linearization. Gaussian ellipsoids describe the declared model;
they are not a distribution-free guarantee for real target manoeuvres. The union
bound covers configured discrete samples, never unsampled portions of a trajectory.
"""

from dataclasses import asdict, dataclass
import hashlib
import json
import math

import numpy as np
from scipy.stats import chi2

from uav_control.guidance.camera_visibility import (
    _FLU_TO_FRD, _OPTICAL_TO_CAMERA_FLU, _rotation, _vector,
)
from uav_control.guidance.camera_visibility_batch import visibility_batch


@dataclass(frozen=True)
class RobustVisibilityConfig:
    """
    Risk for a whole sampled window, with explicit calibration and error source.

    ``calibrated`` is a caller assertion backed by ``calibration_id``; this pure
    kernel cannot establish calibration. Engineering SO(3)/yaw/mount bounds are
    deterministic assumptions, distinguished from measured attitude in reports.
    ``model_mass_floor`` labels major models; it never discards a model.
    """

    version: str = 'p6.robust_visibility.v1'
    risk_budget: float = .05
    sample_count: int = 1
    distribution: str = 'gaussian'
    calibrated: bool = False
    calibration_id: str = ''
    covariance_scale: float = 1.
    empirical_scale: float = 1.
    yaw_error_bound_rad: float = 0.
    attitude_error_bound_rad: float = 0.
    extrinsic_rotation_bound_rad: float = 0.
    extrinsic_translation_bound_m: float = 0.
    attitude_source: str = 'planned'
    error_bound_source: str = 'none'
    model_mass_floor: float = .05

    def fingerprint(self):
        """Stable identity of every risk/calibration/engineering assumption."""
        encoded = json.dumps(asdict(self), sort_keys=True, separators=(',', ':'),
                             allow_nan=False).encode('utf-8')
        return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class RobustVisibilitySample:
    nominal_ok: bool
    robust_ok: bool
    horizontal_dynamic_margin_rad: float
    vertical_dynamic_margin_rad: float
    horizontal_margin_rad: float
    vertical_margin_rad: float
    center_angle_covariance: tuple
    reasons: tuple = ()


@dataclass(frozen=True)
class ModelVisibilityReport:
    model_index: int
    probabilities: tuple
    major: bool
    checked: bool
    robust_ok: bool
    samples: tuple


@dataclass(frozen=True)
class RobustVisibilityReport:
    """Serializable evidence; nominal authority remains the original geometry."""

    nominal_ok: bool = False
    robust_ok: bool = False
    valid_input: bool = False
    reason: str = 'INVALID_INPUT'
    reasons: tuple = ()
    samples: tuple = ()
    nominal_results: tuple = ()
    horizontal_dynamic_margins_rad: tuple = ()
    vertical_dynamic_margins_rad: tuple = ()
    model_reports: tuple = ()
    unchecked_model_mass: float = 0.
    risk_config_fingerprint: str = ''
    risk_config_version: str = ''
    per_sample_risk: float = 0.
    confidence_scale: float = 0.
    risk_scope: str = 'configured_discrete_window_union_bound'
    continuous_time_guarantee: bool = False
    probability_bound_available: bool = False
    guarantee_type: str = 'none'
    attitude_source: str = ''
    error_bound_source: str = ''


def _validate_config(config):
    if not isinstance(config, RobustVisibilityConfig):
        raise ValueError('INVALID_RISK_CONFIG')
    numeric = (config.risk_budget, config.covariance_scale, config.empirical_scale,
               config.yaw_error_bound_rad, config.attitude_error_bound_rad,
               config.extrinsic_rotation_bound_rad, config.extrinsic_translation_bound_m,
               config.model_mass_floor)
    if (any(isinstance(x, (bool, str)) for x in numeric)
            or not np.all(np.isfinite(numeric)) or not 0. < config.risk_budget < 1.
            or not isinstance(config.sample_count, (int, np.integer))
            or isinstance(config.sample_count, bool) or config.sample_count < 1
            or config.covariance_scale <= 0. or config.empirical_scale <= 0.
            or min(numeric[3:7]) < 0. or not 0. <= config.model_mass_floor <= 1.
            or sum(numeric[3:6]) >= math.pi / 2.
            or config.distribution not in ('gaussian', 'moment_bound', 'empirical_envelope')
            or config.attitude_source not in ('planned', 'measured')
            or not isinstance(config.calibrated, bool)
            or not isinstance(config.version, str) or not config.version
            or not isinstance(config.calibration_id, str)
            or not isinstance(config.error_bound_source, str)):
        raise ValueError('INVALID_RISK_CONFIG')
    if any(x > 0. for x in numeric[3:7]) and config.error_bound_source in ('', 'none'):
        raise ValueError('MISSING_ERROR_BOUND_SOURCE')


def _covariances(value, shape):
    array = np.asarray(value, dtype=float)
    if array.shape == (*shape, 9):
        array = array.reshape(*shape, 3, 3)
    if array.shape != (*shape, 3, 3):
        raise ValueError('INVALID_COVARIANCE_SHAPE')
    if not np.all(np.isfinite(array)):
        raise ValueError('NONFINITE_COVARIANCE')
    if not np.allclose(array, np.swapaxes(array, -1, -2), atol=1e-10, rtol=1e-10):
        raise ValueError('INVALID_COVARIANCE_NOT_SYMMETRIC')
    eigenvalues = np.linalg.eigvalsh(array)
    tolerance = 1e-12 * np.maximum(1., np.max(np.abs(eigenvalues), axis=-1))
    if np.any(eigenvalues[..., 0] < -tolerance):
        raise ValueError('INVALID_COVARIANCE_NOT_PSD')
    return array


def _optical_rotation(rotation, extrinsics):
    mount = _rotation(extrinsics.rotation_camera_to_body_flu, 'INVALID_EXTRINSICS')
    return rotation @ _FLU_TO_FRD @ mount @ _OPTICAL_TO_CAMERA_FLU


def _angle_jacobian(optical):
    x, y, z = np.moveaxis(optical, -1, 0)
    hden, vden = x*x + z*z, y*y + z*z
    if np.any(hden <= 1e-18) or np.any(vden <= 1e-18):
        raise ValueError('DEGENERATE_ANGLE_JACOBIAN')
    zero = np.zeros_like(x)
    return np.stack((np.stack((z/hden, zero, -x/hden), axis=-1),
                     np.stack((zero, z/vden, -y/vden), axis=-1)), axis=-2)


def world_target_angle_jacobian(uav_position_ned, body_frd_to_ned,
                                target_position_ned, extrinsics,
                                center_offset_ned=(0., 0., 0.)):
    """
    Return d(horizontal,vertical optical angle)/d(world target position).

    This Jacobian is diagnostic; whole-body robust admission uses exact support
    geometry. Full mount translation and rotation follow the existing P1 kernel.
    """
    position = _vector(uav_position_ned, 'INVALID_UAV_POSITION')
    rotation = _rotation(body_frd_to_ned, 'INVALID_BODY_ROTATION')
    target = _vector(target_position_ned, 'INVALID_TARGET_POSITION')
    offset = _vector(center_offset_ned, 'INVALID_TARGET_GEOMETRY')
    translation = _vector(extrinsics.translation_flu, 'INVALID_EXTRINSICS')
    optical_to_world = _optical_rotation(rotation, extrinsics)
    origin = position + rotation @ _FLU_TO_FRD @ translation
    optical = optical_to_world.T @ (target + offset - origin)
    return _angle_jacobian(optical) @ optical_to_world.T


def _angular_extrema(center, covariance, radius, scale):
    """
    Tangent directions of sphere+ellipsoid in an optical coordinate plane.

    Each support function n.p + radius + scale*sqrt(n.C.n) is exact. All centers
    and uncertainty sets must be strictly ahead of the optical plane. Batched
    bisection avoids sigma-point holes between sampled ellipsoid directions.
    """
    x, z = center[:, :2, None], center[:, 2, None, None]
    cxx = np.diagonal(covariance, axis1=1, axis2=2)[:, :2, None]
    czz, cxz = covariance[:, 2, 2, None, None], covariance[:, :2, 2, None]
    lower = np.full((len(center), 2, 2), -math.pi/2)
    upper = np.full_like(lower, math.pi/2)
    side = np.array((-1., 1.))[None, None, :]
    for _ in range(43):
        theta = (lower + upper)/2
        cosine, sine = np.cos(theta), np.sin(theta)
        variance = np.maximum(0., cxx*cosine**2 + czz*sine**2 - 2*cxz*cosine*sine)
        value = x*cosine - z*sine + side*(radius[:, None, None] + scale*np.sqrt(variance))
        lower = np.where(value > 0., theta, lower)
        upper = np.where(value > 0., upper, theta)
    angles = (lower + upper)/2
    return angles[:, 0, 0], angles[:, 0, 1], angles[:, 1, 0], angles[:, 1, 1]


def _evaluate_samples(rotations, covariance, nominal, intrinsics,
                      extrinsics, sphere, constraints, config, scale):
    optical = np.array([r.target_camera_optical for r in nominal])
    to_world = _optical_rotation(rotations, extrinsics)
    to_optical = np.swapaxes(to_world, -1, -2)
    covariance = config.covariance_scale * (to_optical @ covariance @ to_world)
    maximum_sigma = np.sqrt(np.maximum(0., np.linalg.eigvalsh(covariance)[:, -1]))
    angular_error = (config.yaw_error_bound_rad + config.attitude_error_bound_rad
                     + config.extrinsic_rotation_bound_rad)
    # A rotation displaces any point by <=2*norm(point)*sin(error/2). Including
    # mount lever arm and the uncertain whole sphere bounds full body/mount error.
    radius = (sphere.radius + config.extrinsic_translation_bound_m
              + 2.*(np.linalg.norm(optical, axis=1)
                    + np.linalg.norm(extrinsics.translation_flu)
                    + scale*maximum_sigma + sphere.radius)*math.sin(angular_error/2))
    radius = np.broadcast_to(radius, (len(optical),))
    axial_extent = radius + scale*np.sqrt(np.maximum(0., covariance[:, 2, 2]))
    front = optical[:, 2] - axial_extent > constraints.depth_epsilon
    h0, h1, v0, v1 = _angular_extrema(optical, covariance, radius, scale)
    left, right, up, down = intrinsics.angle_bounds()
    hm = np.minimum(h0-left, right-h1) - constraints.horizontal_margin_rad
    vm = np.minimum(v0-up, down-v1) - constraints.vertical_margin_rad
    if constraints.distance_mode == 'axial':
        near, far = optical[:, 2]-axial_extent, optical[:, 2]+axial_extent
    else:
        distances = np.linalg.norm(optical, axis=1)
        extent = radius + scale*maximum_sigma
        near, far = np.maximum(0., distances-extent), distances+extent
    jacobian = _angle_jacobian(optical)
    angle_covariance = jacobian @ covariance @ np.swapaxes(jacobian, -1, -2)
    samples = []
    for i, original in enumerate(nominal):
        reasons = []
        if not original.whole_target_safe:
            reasons.append('NOMINAL_VISIBILITY_FAILED')
        if not front[i]:
            reasons.append('ROBUST_FRONT_PLANE')
        if hm[i] < -1e-12:
            reasons.append('ROBUST_HORIZONTAL_FOV')
        if vm[i] < -1e-12:
            reasons.append('ROBUST_VERTICAL_FOV')
        if near[i] < constraints.minimum_distance - 1e-12:
            reasons.append('ROBUST_BELOW_MINIMUM_DISTANCE')
        if far[i] > constraints.maximum_distance + 1e-12:
            reasons.append('ROBUST_ABOVE_MAXIMUM_DISTANCE')
        # Angular intervals have no safe interpretation when the set crosses z=0.
        hmargin, vmargin = ((float(hm[i]), float(vm[i])) if front[i]
                            else (-math.inf, -math.inf))
        original_h = original.horizontal_margin_rad
        original_v = original.vertical_margin_rad
        dynamic_h = (max(0., original_h-hmargin) if original_h is not None else math.inf)
        dynamic_v = (max(0., original_v-vmargin) if original_v is not None else math.inf)
        samples.append(RobustVisibilitySample(
            original.whole_target_safe, not reasons, dynamic_h, dynamic_v,
            hmargin, vmargin, tuple(map(tuple, angle_covariance[i])), tuple(reasons)))
    return tuple(samples)


def evaluate_robust_visibility_batch(positions, rotations, targets, target_covariances,
                                     intrinsics, extrinsics, target, constraints, config,
                                     *, model_positions=None, model_covariances=None,
                                     model_probabilities=None):
    """
    Evaluate explicit N-sample trajectory inputs without sources, ROS or clocks.

    Position covariance is world/NED N x 3 x 3 (or N x 9). Conditionals are
    M x N x 3 and M x N x 3 x 3; probabilities are M or N x M. Every supplied
    model is checked, including minor/zero-weight models. Mixture weights must
    already sum to one; no probability mass is silently removed or renormalized.
    Gaussian/moment bounds require a declared calibrated conditional distribution.
    Empirical ellipsoids certify geometry of their envelope only, no analytic risk.
    """
    base = {}
    try:
        p, r, t = (np.asarray(v, dtype=float) for v in (positions, rotations, targets))
        if (p.ndim != 2 or p.shape[1] != 3 or len(p) == 0 or t.shape != p.shape
                or r.shape != (len(p), 3, 3)):
            raise ValueError('INVALID_BATCH_SHAPE')
        nominal = tuple(visibility_batch(p, r, t, intrinsics, extrinsics, target, constraints))
        nominal_ok = all(x.whole_target_safe for x in nominal)
        base.update(nominal_ok=nominal_ok, nominal_results=nominal)
        if not all(x.valid_input for x in nominal):
            raise ValueError('INVALID_NOMINAL_GEOMETRY')
        _validate_config(config)
        base.update(risk_config_fingerprint=config.fingerprint(),
                    risk_config_version=config.version, attitude_source=config.attitude_source,
                    error_bound_source=config.error_bound_source)
        if len(p) > config.sample_count:
            raise ValueError('RISK_SAMPLE_COUNT_EXCEEDED')
        covariance = _covariances(target_covariances, (len(p),))
        if not config.calibrated or not config.calibration_id:
            raise ValueError('UNCALIBRATED_UNCERTAINTY')
        alpha = config.risk_budget/config.sample_count
        if config.distribution == 'gaussian':
            scale = float(np.sqrt(chi2.isf(alpha, 3)))
            guarantee = 'calibrated_conditional_gaussian_ellipsoids'
        elif config.distribution == 'moment_bound':
            scale = math.sqrt(3./alpha)
            guarantee = 'calibrated_conditional_second_moment_ellipsoids'
        else:
            scale = config.empirical_scale
            guarantee = 'calibrated_empirical_envelope_only'
        if not math.isfinite(scale):
            raise ValueError('INVALID_RISK_QUANTILE')
        model_inputs = []
        supplied = (model_positions is not None, model_covariances is not None,
                    model_probabilities is not None)
        if any(supplied):
            if not all(supplied):
                raise ValueError('INCOMPLETE_MODEL_CONDITIONALS')
            mp = np.asarray(model_positions, dtype=float)
            if mp.ndim != 3 or mp.shape[1:] != p.shape or len(mp) == 0:
                raise ValueError('INVALID_MODEL_POSITION_SHAPE')
            mc = _covariances(model_covariances, (len(mp), len(p)))
            probabilities = np.asarray(model_probabilities, dtype=float)
            if probabilities.shape == (len(mp),):
                probabilities = np.broadcast_to(probabilities, (len(p), len(mp)))
            if (probabilities.shape != (len(p), len(mp))
                    or not np.all(np.isfinite(probabilities))
                    or np.any(probabilities < 0.) or np.any(probabilities > 1.)
                    or not np.allclose(np.sum(probabilities, axis=1), 1., atol=1e-10,
                                       rtol=0.)):
                raise ValueError('INVALID_MODEL_PROBABILITIES')
            for index, points in enumerate(mp):
                mn = tuple(visibility_batch(p, r, points, intrinsics, extrinsics,
                                            target, constraints))
                if not all(x.valid_input for x in mn):
                    raise ValueError('INVALID_MODEL_NOMINAL_GEOMETRY')
                model_inputs.append((tuple(probabilities[:, index]),
                                     bool(np.any(probabilities[:, index]
                                                 >= config.model_mass_floor)), mn, mc[index]))
        # Fuse model/sample dimensions for all support-plane root searches. The
        # original scalar nominal authority is retained for every conditional.
        all_covariance = np.concatenate([covariance] + [m[3] for m in model_inputs])
        all_nominal = nominal + tuple(x for m in model_inputs for x in m[2])
        all_rotations = np.tile(r, (1 + len(model_inputs), 1, 1))
        all_samples = _evaluate_samples(
            all_rotations, all_covariance, all_nominal, intrinsics, extrinsics,
            target, constraints, config, scale)
        samples, models = all_samples[:len(p)], []
        for index, (probabilities, major, _, _) in enumerate(model_inputs):
            ms = all_samples[(index+1)*len(p):(index+2)*len(p)]
            models.append(ModelVisibilityReport(index, probabilities, major, True,
                                                all(x.robust_ok for x in ms), ms))
        robust_ok = all(x.robust_ok for x in samples) and all(m.robust_ok for m in models)
        reason = ('NOMINAL_VISIBILITY_FAILED' if not nominal_ok else
                  'MODEL_CONDITIONAL_VISIBILITY_FAILED' if any(not m.robust_ok for m in models)
                  else 'ROBUST_VISIBILITY_FAILED' if not robust_ok else 'OK')
        reasons = tuple(dict.fromkeys(x for s in samples for x in s.reasons))
        return RobustVisibilityReport(
            **base, valid_input=True, robust_ok=robust_ok, reason=reason, reasons=reasons,
            samples=samples, model_reports=tuple(models), per_sample_risk=alpha,
            confidence_scale=scale,
            horizontal_dynamic_margins_rad=tuple(s.horizontal_dynamic_margin_rad for s in samples),
            vertical_dynamic_margins_rad=tuple(s.vertical_dynamic_margin_rad for s in samples),
            probability_bound_available=config.distribution != 'empirical_envelope',
            guarantee_type=guarantee)
    except (ValueError, TypeError, AttributeError) as error:
        return RobustVisibilityReport(**base, reason=str(error), reasons=(str(error),))
    except (FloatingPointError, OverflowError, np.linalg.LinAlgError):
        return RobustVisibilityReport(**base, reason='NUMERICAL_ERROR',
                                      reasons=('NUMERICAL_ERROR',))


def evaluate_robust_visibility(uav_position_ned, body_frd_to_ned, target_position_ned,
                               target_covariance_ned, intrinsics, extrinsics, target,
                               constraints, config):
    """Scalar adapter with the same explicit window risk allocation as the batch."""
    return evaluate_robust_visibility_batch(
        (uav_position_ned,), (body_frd_to_ned,), (target_position_ned,),
        (target_covariance_ned,), intrinsics, extrinsics, target, constraints, config)

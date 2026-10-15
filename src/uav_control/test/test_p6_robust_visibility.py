"""Independent optical rays, support-plane boundaries and conditional risk checks."""

from dataclasses import replace
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_control.guidance.camera_visibility import (
    CameraExtrinsics, CameraIntrinsics, TargetBoundingSphere, VisibilityConstraints,
    evaluate_visibility,
)
from uav_control.guidance.robust_visibility import (
    RobustVisibilityConfig, evaluate_robust_visibility_batch,
    world_target_angle_jacobian,
)


CAMERA = CameraIntrinsics(640, 480, 320., 240., 320., 240.)
MOUNT = CameraExtrinsics.from_sdf_pose((0., 0., 0.), 0., 0., 0.)
LIMITS = VisibilityConstraints(.05, 25.)
RISK = RobustVisibilityConfig(calibrated=True, calibration_id='synthetic-exact-gaussian')


def check(points=((10., 0., 0.),), covariances=None, config=RISK,
          sphere=TargetBoundingSphere(), rotations=None, mount=MOUNT,
          positions=None, limits=LIMITS, **kwargs):
    n = len(points)
    return evaluate_robust_visibility_batch(
        np.zeros((n, 3)) if positions is None else positions,
        np.repeat(np.eye(3)[None], n, axis=0) if rotations is None else rotations,
        points, np.zeros((n, 3, 3)) if covariances is None else covariances,
        CAMERA, mount, sphere, limits, config, **kwargs)


def test_zero_uncertainty_retains_whole_sphere_tangent_boundary():
    result = check(((10., 8., 0.),), sphere=TargetBoundingSphere(math.sqrt(2)))
    assert result.nominal_ok and result.robust_ok
    assert result.samples[0].horizontal_margin_rad == pytest.approx(0., abs=1e-9)
    assert result.horizontal_dynamic_margins_rad == pytest.approx((0.,), abs=1e-9)


def test_center_in_frame_cannot_rescue_nominal_whole_sphere_failure():
    result = check(((10., 8., 0.),), sphere=TargetBoundingSphere(2.))
    assert not result.nominal_ok and not result.robust_ok
    assert result.reason == 'NOMINAL_VISIBILITY_FAILED'


def test_camera_axis_and_nonzero_full_mount_follow_known_physical_ray():
    mount = CameraExtrinsics.from_sdf_pose((.18, 0., .39), 0., math.radians(28), 0.)
    result = check(((9.00947592858927, 0., 4.304715627858908),), mount=mount)
    assert result.robust_ok
    np.testing.assert_allclose(result.nominal_results[0].target_camera_optical,
                               (0., 0., 10.), atol=1e-12)
    yaw = Rotation.from_euler('z', math.pi / 2).as_matrix()
    turned = check(((0., 10., 0.),), rotations=yaw[None])
    assert turned.robust_ok
    np.testing.assert_allclose(turned.nominal_results[0].target_camera_optical,
                               (0., 0., 10.), atol=1e-12)


def test_world_angle_jacobian_matches_known_axis_and_independent_finite_difference():
    j = world_target_angle_jacobian((0., 0., 0.), np.eye(3), (10., 0., 0.), MOUNT)
    np.testing.assert_allclose(j, ((0., .1, 0.), (0., 0., .1)), atol=1e-12)
    rotation = Rotation.from_euler('xyz', (.2, -.1, .7)).as_matrix()
    mount = CameraExtrinsics.from_sdf_pose((.2, -.1, .4), .1, .4, -.2)
    point = np.array((8., 5., 3.))
    j = world_target_angle_jacobian((1., -.3, -2.), rotation, point, mount)
    finite = np.zeros((2, 3))
    for axis in range(3):
        delta = np.eye(3)[axis] * 1e-5
        values = []
        for p in (point + delta, point - delta):
            r = check((p,), mount=mount, positions=((1., -.3, -2.),),
                      rotations=rotation[None])
            x, y, z = r.nominal_results[0].target_camera_optical
            values.append(np.array((math.atan2(x, z), math.atan2(y, z))))
        finite[:, axis] = (values[0] - values[1]) / 2e-5
    np.testing.assert_allclose(j, finite, atol=1e-9)


def test_covariance_orientation_changes_horizontal_and_vertical_margin():
    horizontal = check(covariances=(np.diag((0., .04, 0.)),))
    vertical = check(covariances=(np.diag((0., 0., .04)),))
    assert horizontal.horizontal_dynamic_margins_rad[0] > .04
    assert horizontal.vertical_dynamic_margins_rad[0] < 1e-8
    assert vertical.horizontal_dynamic_margins_rad[0] < 1e-8
    assert vertical.vertical_dynamic_margins_rad[0] > .04


def test_exact_ellipsoid_support_rejects_when_mean_and_linear_jacobian_look_safe():
    # 3-sigma-like lateral radius pushes the body-frame point beyond right=forward.
    result = check(((10., 8., 0.),), covariances=(np.diag((0., 1., 0.)),))
    assert result.nominal_ok and not result.robust_ok
    assert result.samples[0].horizontal_margin_rad < 0.
    assert 'ROBUST_HORIZONTAL_FOV' in result.samples[0].reasons


def test_full_window_risk_allocation_is_more_conservative_without_independence_claim():
    covariance = (np.diag((0., .04, .04)),)
    single = check(covariances=covariance)
    window = check(covariances=covariance, config=replace(RISK, sample_count=17))
    assert window.per_sample_risk == pytest.approx(.05 / 17)
    assert window.confidence_scale > single.confidence_scale
    assert window.horizontal_dynamic_margins_rad[0] > single.horizontal_dynamic_margins_rad[0]
    assert window.risk_scope == 'configured_discrete_window_union_bound'
    assert not window.continuous_time_guarantee
    too_many = check(((10., 0., 0.),) * 2)
    assert too_many.reason == 'RISK_SAMPLE_COUNT_EXCEEDED'


def test_model_conditionals_cannot_hide_low_mass_side_branch_in_fused_mean():
    models = np.array([[[10., 0., 0.]], [[10., 12., 0.]]])
    result = check(model_positions=models, model_covariances=np.zeros((2, 1, 3, 3)),
                   model_probabilities=(.999, .001))
    assert result.nominal_ok and not result.robust_ok
    assert result.reason == 'MODEL_CONDITIONAL_VISIBILITY_FAILED'
    assert len(result.model_reports) == 2
    assert result.model_reports[1].checked and not result.model_reports[1].major
    assert result.unchecked_model_mass == 0.


def test_yaw_error_bound_is_geometric_and_planning_source_remains_explicit():
    nominal = check(((10., 8., 0.),))
    uncertain = check(((10., 8., 0.),), config=replace(
        RISK, yaw_error_bound_rad=.15, attitude_source='planned',
        error_bound_source='engineering-test-envelope'))
    assert nominal.robust_ok and not uncertain.robust_ok
    assert uncertain.attitude_source == 'planned'
    assert uncertain.error_bound_source == 'engineering-test-envelope'
    assert uncertain.horizontal_dynamic_margins_rad[0] > .1
    assert nominal.risk_config_fingerprint != uncertain.risk_config_fingerprint


def test_range_and_front_plane_checks_are_never_relaxed():
    assert not check(((24.9, 0., 0.),), sphere=TargetBoundingSphere(.25)).nominal_ok
    near = check(((1., 0., 0.),), covariances=(np.diag((.25, 0., 0.)),))
    assert near.nominal_ok and not near.robust_ok
    assert 'ROBUST_FRONT_PLANE' in near.samples[0].reasons
    far = check(((24., 0., 0.),), covariances=(np.diag((1., 0., 0.)),))
    assert far.nominal_ok and not far.robust_ok
    assert 'ROBUST_ABOVE_MAXIMUM_DISTANCE' in far.samples[0].reasons


@pytest.mark.parametrize('covariance,reason', [
    (np.diag((-.01, .1, .1)), 'INVALID_COVARIANCE_NOT_PSD'),
    (np.diag((float('nan'), .1, .1)), 'NONFINITE_COVARIANCE'),
    (np.array(((1., .1, 0.), (0., 1., 0.), (0., 0., 1.))),
     'INVALID_COVARIANCE_NOT_SYMMETRIC'),
])
def test_invalid_covariance_cannot_publish_robust_success(covariance, reason):
    result = check(covariances=(covariance,))
    assert result.nominal_ok and not result.robust_ok
    assert result.reason == reason


def test_uncalibrated_probability_model_is_explicit_failure():
    result = check(config=RobustVisibilityConfig())
    assert result.nominal_ok and not result.robust_ok
    assert result.reason == 'UNCALIBRATED_UNCERTAINTY'


def test_empirical_envelope_does_not_claim_analytic_probability_bound():
    config = replace(RISK, distribution='empirical_envelope', empirical_scale=2.)
    result = check(covariances=(np.eye(3) * .01,), config=config)
    assert result.robust_ok
    assert not result.probability_bound_available
    assert result.guarantee_type == 'calibrated_empirical_envelope_only'


def test_batch_matches_individual_with_same_global_risk_allocation():
    points = ((10., 0., 0.), (10., 1., -1.), (10., 7., 0.))
    cov = np.repeat(np.diag((.01, .04, .02))[None], 3, axis=0)
    risk = replace(RISK, sample_count=3)
    batch = check(points, cov, risk)
    for i, point in enumerate(points):
        scalar = check((point,), (cov[i],), risk)
        assert batch.samples[i] == scalar.samples[0]


def test_correlated_ellipsoid_angular_envelope_contains_independent_boundary_samples():
    covariance = np.array(((.08, .03, -.02), (.03, .06, .01), (-.02, .01, .05)))
    point = np.array((10., 1.5, -.8))
    sphere = TargetBoundingSphere(.7)
    report = check((point,), (covariance,), sphere=sphere)
    assert report.robust_ok
    rng = np.random.default_rng(61014)
    directions = rng.normal(size=(240, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    deviations = report.confidence_scale * (directions @ np.linalg.cholesky(covariance).T)
    for deviation in deviations:
        perturbed = evaluate_visibility((0., 0., 0.), np.eye(3), point+deviation,
                                        CAMERA, MOUNT, sphere, LIMITS)
        assert perturbed.whole_target_safe
        assert perturbed.horizontal_margin_rad >= report.samples[0].horizontal_margin_rad-1e-10
        assert perturbed.vertical_margin_rad >= report.samples[0].vertical_margin_rad-1e-10


def test_moment_bound_is_more_conservative_than_gaussian_at_same_window_risk():
    covariance = (np.eye(3)*.01,)
    gaussian = check(covariances=covariance)
    moment = check(covariances=covariance, config=replace(RISK, distribution='moment_bound'))
    assert moment.confidence_scale == pytest.approx(math.sqrt(3/.05))
    assert moment.confidence_scale > gaussian.confidence_scale
    assert moment.probability_bound_available
    assert moment.horizontal_dynamic_margins_rad[0] > gaussian.horizontal_dynamic_margins_rad[0]


def test_covariance_scale_changes_evidence_and_margin_and_flat_input_is_supported():
    covariance = np.diag((.01, .04, .02))
    nominal = check(covariances=(covariance.reshape(9),))
    inflated = check(covariances=(covariance,), config=replace(RISK, covariance_scale=4.))
    assert nominal.robust_ok and inflated.robust_ok
    assert inflated.horizontal_dynamic_margins_rad[0] > nominal.horizontal_dynamic_margins_rad[0]
    assert inflated.risk_config_fingerprint != nominal.risk_config_fingerprint


@pytest.mark.parametrize('changes', [
    dict(risk_budget=0.), dict(risk_budget=float('nan')), dict(sample_count=True),
    dict(covariance_scale=-1.), dict(distribution='unknown'), dict(calibrated='true'),
])
def test_invalid_risk_config_never_claims_probability(changes):
    report = check(config=replace(RISK, **changes))
    assert not report.robust_ok and not report.probability_bound_available
    assert report.reason == 'INVALID_RISK_CONFIG'


def test_minor_model_invalid_covariance_and_nonunit_mass_fail_closed():
    points = np.repeat(np.array([[[10., 0., 0.]]]), 2, axis=0)
    cov = np.zeros((2, 1, 3, 3))
    cov[1, 0, 0, 0] = -.01
    report = check(model_positions=points, model_covariances=cov,
                   model_probabilities=(.999, .001))
    assert report.reason == 'INVALID_COVARIANCE_NOT_PSD'
    report = check(model_positions=points, model_covariances=np.zeros_like(cov),
                   model_probabilities=(.9, .05))
    assert report.reason == 'INVALID_MODEL_PROBABILITIES'

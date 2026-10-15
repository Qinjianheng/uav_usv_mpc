"""Independent integration checks of root-authored short-lease risk scope."""

from dataclasses import replace

import numpy as np
import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.p6_follow import P6Policy, assess_risk


def setup(scope=.45):
    raw = request()
    covariance = np.array([np.eye(3)*(0. if t < .6 else 100.)
                           for t in raw.prediction_times])
    raw = replace(raw, model='BCTRA_BOUNDED_Z',
                  target_covariances=tuple(covariance.reshape(-1, 9).tolist()))
    configured = model()
    configured.p6_policy = P6Policy(
        visibility_mode='robust', allow_nominal_fallback=False,
        risk_horizon_seconds=scope, calibration_id='exact-unit-envelopes',
        calibration_sha256='a'*64, calibration_family='bctra')
    configured.p6_policy.validate()
    return raw, configured


def assess(raw, configured, query):
    query = np.asarray(query)
    return assess_risk(configured, raw, query, np.tile((-5., 0., -5.), (len(query), 1)),
                       np.tile(np.eye(3), (len(query), 1, 1)), np.zeros((len(query), 3)))


def test_prefix_only_claims_checked_window_and_binds_scope_fingerprint():
    raw, configured = setup()
    query = (0., .225, .45, .8, 1.6)
    prefix, detail = assess(raw, configured, query)
    assert prefix.robust_ok
    assert detail['checked_indices'] == (0, 1, 2)
    assert detail['checked_relative_times'] == (0., .225, .45)
    assert not detail['entire_minco_curve_robust']
    assert detail['risk_scope'] == 'execution_lease_prefix_discrete_samples'
    configured.p6_policy = replace(configured.p6_policy, risk_horizon_seconds=0.)
    full, whole = assess(raw, configured, query)
    assert not full.robust_ok
    assert prefix.per_sample_risk == full.per_sample_risk == .05/len(query)
    assert detail['risk_kernel_fingerprint'] == whole['risk_kernel_fingerprint']
    assert detail['risk_config_fingerprint'] != whole['risk_config_fingerprint']


def test_missing_prefix_endpoint_fails_without_marking_unchecked_suffix_safe():
    raw, configured = setup()
    report, detail = assess(raw, configured, (0., .2, .4, .8, 1.6))
    assert report is None
    assert detail['reason'] == 'P6_RISK_ENDPOINT_NOT_SAMPLED'
    assert not detail['robust_ok'] and not detail['used_nominal_fallback']


def test_remaining_lease_is_capped_by_existing_endpoint_and_never_extended_from_now():
    raw, configured = setup()
    report, detail = assess(raw, configured, (.2, .325, .45))
    assert report.robust_ok
    assert max(detail['checked_relative_times']) == .45
    assert not detail['entire_minco_curve_robust']


@pytest.mark.parametrize('batch', [False, True])
def test_prefix_cannot_mask_original_whole_curve_nominal_fov_failure(batch):
    raw, configured = setup()
    query = np.array((0., .225, .45, .8, 1.6))
    p = np.tile((-5., 0., -5.), (len(query), 1))
    p[-1] = (10., 0., -5.)  # The genuine target is behind the final camera.
    zero = np.zeros_like(p)
    _, views, geometry, _ = FollowProblem(raw, configured, 1.6).assess(
        query, p, zero, zero, zero, np.zeros(len(query)), np.zeros(len(query)), batch=batch)
    assert not views[-1].whole_target_safe
    assert min(geometry[-1]) < 0.


def test_candidate_batch_reports_explicit_subset_indices_in_original_order():
    raw, configured = setup()
    query = (0., .8, .225, 1.2, .45)*2
    report, detail = assess(raw, configured, query)
    assert report.robust_ok
    assert len(report.samples) == 6
    assert detail['checked_indices'] == (0, 2, 4, 5, 7, 9)


def test_policy_disallows_shorter_risk_coverage_than_the_execution_lease():
    _, configured = setup()
    with pytest.raises(ValueError, match='INVALID_P6_POLICY'):
        replace(configured.p6_policy, risk_horizon_seconds=.2).validate()

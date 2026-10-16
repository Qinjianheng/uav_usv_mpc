"""Independent calibration provenance and metric checks for offline study."""


import pytest


def study_module():
    from uav_control.analysis import imm_calibration
    return imm_calibration


def rows(seed, squared_mahalanobis):
    return [dict(seed=seed, backend='IMM', horizon=.5, valid=True,
                 mahalanobis_squared=value, conditional_mahalanobis_squared=[value],
                 error=[value ** .5, 0., 0.], covariance=[
                     [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
                 horizontal_error=value ** .5, velocity_error=0.,
                 heading_error=0., compute_time_ms=1., mode='linear',
                 stage='straight', comparison='complete_chain')
            for value in squared_mahalanobis]


def test_calibration_uses_training_residuals_and_cannot_fit_on_heldout_outliers():
    module = study_module()
    calibration = module.fit_calibration(
        rows(1, [1.] * 19 + [6.]), rows(2, [100.] * 20), 'imm',
        {'calibration_seed': 1, 'heldout_seed': 2})
    assert calibration['covariance_scale'] == pytest.approx(1.)
    assert calibration['covariance_floor_m2'] == 0.
    assert calibration['coverage']['heldout'] == 0.
    assert calibration['probability_guarantee'] is False
    assert calibration['distribution'] == 'empirical_envelope'


def test_empirical_radius_matches_actual_3d_training_ellipsoid_and_vertical_residual():
    module = study_module()
    train = rows(1, [1.] * 20)
    for row in train:
        row['error'] = [1., 0., 4.]
    calibration = module.fit_calibration(
        train, rows(2, [1.] * 20), 'imm', {'calibration_seed': 1, 'heldout_seed': 2})
    assert calibration['calibration_domain'] == 'xyz'
    assert calibration['covariance_scale'] * calibration['empirical_scale'] ** 2 >= 17. - 1e-10
    assert calibration['empirical_scale'] == pytest.approx(2.7954834829151074)


def test_calibration_refuses_overlapping_seed_provenance():
    module = study_module()
    with pytest.raises(ValueError, match='disjoint'):
        module.fit_calibration(rows(1, [1.]), rows(1, [1.]), 'imm',
                               {'calibration_seed': 1, 'heldout_seed': 1})


def test_metric_summary_keeps_severe_failures_and_invalid_attempts():
    module = study_module()
    observations = rows(2, [1., 100.])
    observations.append(dict(observations[0], valid=False))
    summary = module.summarize(observations)
    assert summary['attempts'] == 3
    assert summary['valid_count'] == 2
    assert summary['valid_fraction'] == pytest.approx(2. / 3.)
    assert summary['horizontal_rmse'] == pytest.approx((101. / 2.) ** .5)
    assert summary['severe_over_5m'] == 1


def test_retired_predictor_family_cannot_enter_current_calibration_helper():
    with pytest.raises(ValueError, match='only current IMM'):
        study_module().fit_calibration(rows(1, [1.]), rows(2, [1.]), 'bctra',
                                       {'calibration_seed': 1, 'heldout_seed': 2})

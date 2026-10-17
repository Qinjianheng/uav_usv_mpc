"""Independent kinematic and statistical checks for the raw-observation IMM."""

import math
import time

import numpy as np
import pytest

from uav_control.tracking.imm_predictor import ImmConfig, ImmPredictor
from uav_control.tracking.imm_predictor import mix_models, propagate_model


def noiseless_config(**kwargs):
    return ImmConfig(acceleration_noise=(0., 0., 0.),
                     turn_rate_noise=(0., 0., 0.),
                     longitudinal_jerk_noise=0., turn_jerk_noise=0.,
                     **kwargs)


def test_markov_interaction_uses_destination_probability_and_between_state_spread():
    states = np.zeros((3, 7))
    states[:, 0] = (0., 10., 20.)
    covariances = np.tile(np.eye(7), (3, 1, 1))
    transition = np.array(((.8, .1, .1), (.2, .7, .1), (.1, .2, .7)))
    mixed, covariance, prior = mix_models(
        states, covariances, np.array((.6, .3, .1)), transition)
    np.testing.assert_allclose(prior, (.55, .29, .16))
    assert mixed[0, 0] == pytest.approx(.8 / .55)
    assert covariance[0, 0, 0] == pytest.approx(
        1. + 10. / .55 - (.8 / .55) ** 2)


@pytest.mark.parametrize('model', ('CV', 'CTRV', 'CTRA'))
def test_zero_turn_motion_has_straight_line_limit(model):
    state = np.array((1., 2., 3., -4., 1e-12, 0., 0.))
    future, covariance = propagate_model(
        state, np.eye(7), 2., model, noiseless_config())
    np.testing.assert_allclose(future[:4], (7., -6., 3., -4.), atol=1e-9)
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)
    assert np.linalg.eigvalsh(covariance).min() >= -1e-10


def test_cv_covariance_matches_independent_linear_transition():
    state = np.array((1., 2., 3., 4., .6, 2., 1.))
    covariance = np.diag((2., 2., 4., 4., .1, .1, .1))
    future, propagated = propagate_model(
        state, covariance, 2., 'CV', noiseless_config())
    np.testing.assert_allclose(future, (7., 10., 3., 4., 0., 0., 0.))
    assert propagated[0, 0] == pytest.approx(18.)
    assert propagated[0, 2] == pytest.approx(8.)


def test_ctrv_quarter_circle_has_analytic_position_and_velocity():
    state = np.array((0., 0., 2., 0., math.pi / 2., .8, .9))
    future, _ = propagate_model(state, np.eye(7), 1., 'CTRV', noiseless_config())
    np.testing.assert_allclose(future[:4], (4 / math.pi, 4 / math.pi, 0., 2.),
                               atol=1e-9)
    np.testing.assert_allclose(future[4:], (math.pi / 2., 0., 0.))


def test_ctra_accelerates_and_propagates_turn_acceleration():
    state = np.array((0., 0., 2., 0., 0., 1., 0.))
    future, _ = propagate_model(state, np.eye(7), 2., 'CTRA', noiseless_config())
    np.testing.assert_allclose(future[:4], (6., 0., 4., 0.), atol=1e-8)
    state = np.array((0., 0., 2., 0., -.3, .3, .4))
    future, _ = propagate_model(state, np.eye(7), 1.5, 'CTRA', noiseless_config())
    # Independent fine trapezoid quadrature of the continuous kinematics.
    t = np.linspace(0., 1.5, 20001)
    heading = -.3 * t + .2 * t * t
    expected = (np.trapz((2. + .3 * t) * np.cos(heading), t),
                np.trapz((2. + .3 * t) * np.sin(heading), t))
    np.testing.assert_allclose(future[:2], expected, atol=2e-9)
    assert future[4] == pytest.approx(.3)


def test_ctra_braking_stops_without_reversing_and_is_a_consistent_flow():
    initial = np.array((0., 0., 2., 0., 0., -1., 0.))
    config = noiseless_config()
    covariance = np.eye(7) * .01
    stopped, _ = propagate_model(initial, covariance, 3., 'CTRA', config)
    np.testing.assert_allclose(stopped[:4], (2., 0., 0., 0.), atol=1e-10)
    middle, intermediate_covariance = propagate_model(initial, covariance, 1.9, 'CTRA', config)
    composed, _ = propagate_model(middle, intermediate_covariance, 1.1, 'CTRA', config)
    np.testing.assert_allclose(composed, stopped, atol=1e-9)
    tiny, tiny_covariance = propagate_model(initial, covariance, 1.999999, 'CTRA', config)
    composed, _ = propagate_model(tiny, tiny_covariance, 1.000001, 'CTRA', config)
    np.testing.assert_allclose(composed, stopped, atol=1e-8)


def test_ctra_stopped_forecast_acceleration_and_covariance_are_consistent():
    predictor = ImmPredictor(noiseless_config(
        transition_matrix=np.eye(3), initial_probabilities=(0., 0., 1.)))
    predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
    predictor.states[2] = (0., 0., 2., 0., .2, -1., .1)
    forecast = predictor.predict((1.9, 2., 2.1, 3., 4.))
    assert np.linalg.norm(forecast.velocity[0, :2]) == pytest.approx(.1)
    np.testing.assert_allclose(forecast.velocity[1:, :2], 0., atol=1e-12)
    np.testing.assert_allclose(forecast.acceleration[1:, :2], 0., atol=1e-12)
    assert np.isfinite(forecast.position_covariance).all()
    for covariance in forecast.position_covariance:
        assert np.linalg.eigvalsh(covariance).min() >= -1e-10


def test_ctra_stopping_position_jacobian_matches_independent_clipped_ode():
    initial = np.array((.4, -.2, 2., 1., .3, -1., .1))
    covariance = np.diag((.02, .03, .04, .05, .006, .007, .008))

    def independent(value):
        speed = math.hypot(value[2], value[3])
        end = min(4., -speed / value[5])
        times = np.linspace(0., end, 8001)
        heading = math.atan2(value[3], value[2]) + value[4] * times + .5 * value[6] * times ** 2
        speed_at_time = np.maximum(speed + value[5] * times, 0.)
        return np.array((value[0] + np.trapz(speed_at_time * np.cos(heading), times),
                         value[1] + np.trapz(speed_at_time * np.sin(heading), times),
                         0., 0., value[4] + 4. * value[6], value[5], value[6]))

    numerical = np.empty((7, 7))
    for index in range(7):
        delta = np.zeros(7)
        delta[index] = 1e-5
        numerical[:, index] = (independent(initial + delta) - independent(initial - delta)) / 2e-5
    state, propagated = propagate_model(initial, covariance, 4., 'CTRA', noiseless_config())
    np.testing.assert_allclose(state, independent(initial), atol=1e-8)
    np.testing.assert_allclose(propagated, numerical @ covariance @ numerical.T, atol=2e-8)


def test_ctra_unobservable_tiny_velocity_cannot_create_positive_acceleration_heading():
    initial = np.array((0., 0., 1e-12, 0., .1, 1., .2))
    state, covariance = propagate_model(initial, np.eye(7) * .01, 1., 'CTRA')
    assert np.linalg.norm(state[2:4]) <= 1e-11
    assert np.linalg.norm(state[:2]) <= 1e-11
    assert np.linalg.eigvalsh(covariance).max() < 100.
    predictor = ImmPredictor(ImmConfig(
        transition_matrix=np.eye(3), initial_probabilities=(0., 0., 1.)))
    predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
    predictor.states[2] = initial
    forecast = predictor.predict((.1, 1.))
    assert np.linalg.norm(forecast.acceleration[:, :2], axis=1).max() < 1e-10


def test_ctra_stop_preserves_cartesian_process_noise_and_observation_recovery():
    config = ImmConfig(transition_matrix=np.eye(3), initial_probabilities=(0., 0., 1.),
                       turn_rate_noise=(0., 0., 0.), longitudinal_jerk_noise=0.,
                       turn_jerk_noise=0.)
    _, covariance = propagate_model(
        np.array((0., 0., 2., 0., 0., -1., 0.)), np.zeros((7, 7)), 3., 'CTRA', config)
    q = .35 ** 2
    assert covariance[0, 0] == pytest.approx(q * 3. ** 3 / 3.)
    assert covariance[2, 2] == pytest.approx(q * 3.)
    assert covariance[0, 2] == pytest.approx(q * 3. ** 2 / 2.)
    predictor = ImmPredictor(config)
    predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
    predictor.states[2] = (0., 0., 1e-10, 0., 0., -1., 0.)
    predictor.covariances[2] = np.eye(7) * .01
    for index in range(1, 401):
        assert predictor.update(10. + .05 * index, (.2 * index, 0., 0.),
                                np.eye(3) * .01).accepted
    forecast = predictor.predict((0., .5, 1.6, 4.))
    assert abs(forecast.position[0, 0] - 80.) < .5
    assert abs(forecast.velocity[0, 0] - 4.) < .5
    assert np.abs(forecast.position).max() < 110.


def test_stopped_cartesian_noise_covariance_matches_independent_monte_carlo():
    config = ImmConfig(turn_rate_noise=(0., 0., 0.), longitudinal_jerk_noise=0.,
                       turn_jerk_noise=0.)
    _, covariance = propagate_model(
        np.array((0., 0., 2., 0., .2, -1., .1)), np.zeros((7, 7)), 3., 'CTRA', config)
    rng = np.random.default_rng(430)
    p, v = np.zeros(20000), np.zeros(20000)
    for _ in range(300):
        dv = rng.normal(0., .35 * math.sqrt(.01), len(v))
        p += v * .01 + .5 * dv * .01
        v += dv
    measured = np.cov(np.stack((p, v)))
    np.testing.assert_allclose(covariance[np.ix_((0, 2), (0, 2))], measured, rtol=.04)
    assert np.linalg.eigvalsh(covariance).min() >= -1e-10


def test_latent_engineering_projection_inflates_covariance_by_mean_shift():
    predictors = []
    for bounds in ((1e6, 1e6, 1e6), (.7, 2., 1.5)):
        config = ImmConfig(
            transition_matrix=np.eye(3), initial_probabilities=(0., 0., 1.),
            maximum_turn_rate=bounds[0], maximum_longitudinal_acceleration=bounds[1],
            maximum_turn_acceleration=bounds[2])
        predictor = ImmPredictor(config)
        predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
        predictor.states[2] = (0., 0., 4., 0., 3., 10., 4.)
        predictor.covariances[2] = np.eye(7) * .01
        assert predictor.update(10.05, (.2, 0., 0.), np.eye(3) * .01).accepted
        predictors.append(predictor)
    free, bounded = predictors
    np.testing.assert_allclose(bounded.states[2, 4:], (.7, 2., 1.5))
    shift = free.states[2] - bounded.states[2]
    np.testing.assert_allclose(bounded.covariances[2] - free.covariances[2],
                               np.outer(shift, shift), atol=1e-10)
    assert np.linalg.eigvalsh(bounded.covariances[2] - free.covariances[2]).min() >= -1e-9
    assert bounded.projection_count == 1


@pytest.mark.parametrize('model_index', (0, 1, 2))
def test_current_horizontal_speed_prior_is_shared_and_preserves_covariance(model_index):
    predictors = []
    for speed_bound in (1e6, 5.):
        probabilities = np.zeros(3)
        probabilities[model_index] = 1.
        predictor = ImmPredictor(ImmConfig(
            transition_matrix=np.eye(3), initial_probabilities=probabilities,
            maximum_horizontal_speed=speed_bound))
        predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
        predictor.states[model_index] = (0., 0., 6., 8., 0., 0., 0.)
        predictor.covariances[model_index] = np.eye(7) * .01
        predictor.update(10.05, (.3, .4, 0.), np.eye(3) * .01)
        predictors.append(predictor)
    free, bounded = predictors
    np.testing.assert_allclose(bounded.states[model_index, 2:4], (3., 4.), atol=1e-12)
    shift = free.states[model_index] - bounded.states[model_index]
    np.testing.assert_allclose(
        bounded.covariances[model_index] - free.covariances[model_index],
        np.outer(shift, shift), atol=1e-10)
    difference = bounded.covariances[model_index] - free.covariances[model_index]
    assert np.linalg.eigvalsh(difference).min() >= -1e-9


def test_legal_four_metre_per_second_input_is_unchanged_by_speed_prior():
    forecasts = []
    for speed_bound in (1e6, 5.):
        predictor = ImmPredictor(ImmConfig(maximum_horizontal_speed=speed_bound))
        predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
        predictor.states[:, 2] = 4.
        predictor.update(10.05, (.2, 0., 0.), np.eye(3) * .01)
        forecasts.append(predictor.predict((0., .5, 1.6, 4.)))
    np.testing.assert_array_equal(forecasts[0].position, forecasts[1].position)
    np.testing.assert_array_equal(forecasts[0].state_covariance, forecasts[1].state_covariance)


def test_continuous_longitudinal_jerk_contributes_position_velocity_and_cross_covariance():
    config = ImmConfig(acceleration_noise=(0., 0., 0.),
                       turn_rate_noise=(0., 0., 0.),
                       longitudinal_jerk_noise=1., turn_jerk_noise=0.)
    _, covariance = propagate_model(
        np.array((0., 0., 4., 0., 0., 0., 0.)),
        np.zeros((7, 7)), 1., 'CTRA', config)
    # Integrating white longitudinal jerk: Ppp=t^5/20, Pvv=t^3/3,
    # Ppv=t^4/8, Ppa=t^3/6, Pva=t^2/2 for unit spectral density.
    assert covariance[0, 0] == pytest.approx(1. / 20.)
    assert covariance[2, 2] == pytest.approx(1. / 3.)
    assert covariance[0, 2] == pytest.approx(1. / 8.)
    assert covariance[0, 5] == pytest.approx(1. / 6.)
    assert covariance[2, 5] == pytest.approx(1. / 2.)


@pytest.mark.parametrize('model', ('CV', 'CTRV', 'CTRA'))
def test_ekf_covariance_agrees_with_independent_numerical_kinematic_jacobian(model):
    state = np.array((.4, -.2, 3., 1., .35, .6, -.2))
    covariance = np.diag((.02, .03, .04, .05, .006, .007, .008))

    def independent_kinematics(value):
        t = np.linspace(0., .7, 4001)
        omega = value[4] if model != 'CV' else 0.
        a = value[5] if model == 'CTRA' else 0.
        alpha = value[6] if model == 'CTRA' else 0.
        heading = math.atan2(value[3], value[2]) + omega * t + .5 * alpha * t * t
        speed = math.hypot(value[2], value[3]) + a * t
        vx, vy = speed * np.cos(heading), speed * np.sin(heading)
        return np.array((value[0] + np.trapz(vx, t), value[1] + np.trapz(vy, t),
                         vx[-1], vy[-1], omega + alpha * .7, a, alpha))

    numerical = np.empty((7, 7))
    for index in range(7):
        delta = np.zeros(7)
        delta[index] = 1e-5
        numerical[:, index] = (independent_kinematics(state + delta)
                               - independent_kinematics(state - delta)) / 2e-5
    _, predicted = propagate_model(state, covariance, .7, model, noiseless_config())
    np.testing.assert_allclose(predicted, numerical @ covariance @ numerical.T,
                               atol=2e-8, rtol=2e-6)


def test_stationary_target_does_not_create_heading_or_nan():
    for model in ('CV', 'CTRV', 'CTRA'):
        future, covariance = propagate_model(
            np.array((1., 2., 0., 0., .4, .8, .5)),
            np.eye(7), 4., model, ImmConfig())
        assert np.isfinite(future).all()
        assert np.isfinite(covariance).all()
        np.testing.assert_allclose(future[:4], (1., 2., 0., 0.))


def test_first_raw_observation_has_its_measurement_covariance():
    predictor = ImmPredictor()
    assert predictor.update(10., (1., 2., -.15), np.diag((.04, .09, .01))).accepted
    forecast = predictor.predict((0., .5, 1.6))
    np.testing.assert_allclose(forecast.position[0], (1., 2., -.15))
    np.testing.assert_allclose(forecast.position_covariance[0],
                               np.diag((.04, .09, .01)))
    assert forecast.conditional_position.shape == (3, 3, 3)
    assert forecast.state_covariance.shape == (7, 7)


@pytest.mark.parametrize('stamp,reason', (
    (10., 'DUPLICATE_OBSERVATION'), (9.9, 'OUT_OF_ORDER_OBSERVATION')))
def test_duplicate_and_out_of_order_observations_cannot_reduce_covariance(stamp, reason):
    predictor = ImmPredictor()
    predictor.update(10., (1., 2., .1), np.eye(3) * .04)
    before = predictor.predict((0.,))
    result = predictor.update(stamp, (999., 999., 999.), np.eye(3) * 1e-9)
    assert not result.accepted
    assert result.reason == reason
    after = predictor.predict((0.,))
    np.testing.assert_array_equal(after.state, before.state)
    np.testing.assert_array_equal(after.state_covariance, before.state_covariance)


@pytest.mark.parametrize('position,covariance', (
    ((math.nan, 0., 0.), np.eye(3)),
    ((0., 0., 0.), np.diag((1., -1., 1.))),
    ((0., 0., 0.), np.ones((2, 2))),
    ((0., 0., 0.), np.full((3, 3), math.inf)),
    ((0., 0., 0.), np.array(((1., 1., 0.), (0., 1., 0.), (0., 0., 1.)))),
))
def test_invalid_measurement_rejects_without_corrupting_filter(position, covariance):
    predictor = ImmPredictor()
    predictor.update(10., (0., 0., 0.), np.eye(3))
    before = predictor.predict((0.,))
    assert not predictor.update(10.1, position, covariance).accepted
    np.testing.assert_array_equal(predictor.predict((0.,)).state, before.state)


def test_singular_covariance_is_regularized_without_negative_eigenvalues():
    predictor = ImmPredictor()
    for index in range(10):
        assert predictor.update(10. + .05 * index, (0., 0., 0.),
                                np.zeros((3, 3))).accepted
    result = predictor.predict((0., .2, .8, 1.6, 4.))
    for covariance in result.position_covariance:
        assert np.linalg.eigvalsh(covariance).min() >= 0.


def test_cross_vertical_covariance_is_conservatively_separated():
    predictor = ImmPredictor()
    measurement_covariance = np.array(((.04, .01, .005),
                                       (.01, .09, -.006),
                                       (.005, -.006, .02)))
    predictor.update(10., (1., 2., .1), measurement_covariance)
    actual = predictor.predict((0.,)).position_covariance[0]
    assert np.linalg.eigvalsh(actual - measurement_covariance).min() >= -1e-12


def test_gap_reinitializes_with_uncertain_velocity_instead_of_extrapolating_old_track():
    predictor = ImmPredictor()
    predictor.update(10., (1., 2., .1), np.eye(3) * .01)
    result = predictor.update(11., (3., 2., .1), np.eye(3) * .01)
    assert result.accepted
    assert result.reason == 'REINITIALIZED_AFTER_GAP'
    forecast = predictor.predict((0.,))
    np.testing.assert_allclose(forecast.position[0], (3., 2., .1))
    np.testing.assert_allclose(forecast.velocity[0], (0., 0., 0.))
    assert forecast.state_covariance[2, 2] >= 9.


@pytest.mark.parametrize('stamp', (0., -1., math.nan, math.inf))
def test_nonpositive_or_nonfinite_acquisition_stamp_is_rejected(stamp):
    predictor = ImmPredictor()
    assert not predictor.update(stamp, (0., 0., 0.), np.eye(3)).accepted
    assert predictor.stamp is None


def test_log_likelihood_extreme_residual_does_not_reset_probabilities_to_uniform():
    predictor = ImmPredictor(ImmConfig(initial_probabilities=(.8, .15, .05)))
    predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
    assert predictor.update(10.05, (1e8, -1e8, .1), np.eye(3) * .01).accepted
    probabilities = predictor.predict((0.,)).model_probabilities
    assert np.isfinite(probabilities).all()
    assert probabilities.sum() == pytest.approx(1.)
    assert np.max(probabilities) > .5


def test_forecast_at_preserves_filter_epoch_and_propagates_before_horizons():
    predictor = ImmPredictor(ImmConfig(
        transition_matrix=np.eye(3), initial_probabilities=(1., 0., 0.)))
    for index in range(30):
        predictor.update(10. + index * .05, (index * .2, 0., 0.), np.eye(3) * .0025)
    before = predictor.predict((0.,))
    future = predictor.forecast_at(11.5, (0., .5))
    assert predictor.stamp == pytest.approx(11.45)
    assert future.source_stamp == pytest.approx(11.5)
    assert future.observation_stamp == pytest.approx(11.45)
    assert future.position[0, 0] > before.position[0, 0]
    assert future.position[1, 0] - future.position[0, 0] == pytest.approx(2., abs=.02)
    np.testing.assert_array_equal(predictor.predict((0.,)).state, before.state)
    np.testing.assert_array_equal(future.model_probabilities, (1., 0., 0.))


def test_heading_crossing_pi_uses_cartesian_velocity_without_spurious_reversal():
    predictor = ImmPredictor()
    for index in range(80):
        t = index * .05
        heading = 2.9 + .2 * t
        position = (20. * (math.sin(heading) - math.sin(2.9)),
                    20. * (-math.cos(heading) + math.cos(2.9)), 0.)
        assert predictor.update(10. + t, position, np.eye(3) * .0025).accepted
    result = predictor.predict((0., .8))
    assert result.velocity[0, 0] < -3.
    assert result.state[4] == pytest.approx(.2, abs=.1)
    assert np.isfinite(result.state_covariance).all()


def test_steady_turn_observations_select_turning_models_and_finite_covariance():
    predictor = ImmPredictor()
    for index in range(100):
        t = index * .05
        predictor.update(
            10. + t, (10. * math.sin(.4 * t), 10. * (1. - math.cos(.4 * t)), .1),
            np.eye(3) * .0025)
    forecast = predictor.predict((0., .5, 1.6, 4.))
    assert forecast.model_probabilities[1:].sum() > .6
    for covariance in forecast.position_covariance:
        assert np.linalg.eigvalsh(covariance).min() > 0.
    assert np.trace(forecast.position_covariance[-1]) > np.trace(
        forecast.position_covariance[0])
    assert np.trace(forecast.model_disagreement[-1]) > 0.


def test_vertical_displacement_is_bounded_without_importing_truth_or_kf():
    predictor = ImmPredictor(ImmConfig(maximum_vertical_displacement=.3))
    for index in range(30):
        predictor.update(10. + .05 * index, (0., 0., index * .1), np.eye(3) * .001)
    result = predictor.predict((0., 1., 4.))
    assert abs(result.position[-1, 2] - result.position[0, 2]) <= .3 + 1e-12
    assert result.position_covariance[-1, 2, 2] > result.position_covariance[0, 2, 2]


@pytest.mark.parametrize('kwargs', (
    {'transition_matrix': np.ones((3, 3))},
    {'initial_probabilities': (0., 0., 0.)},
    {'acceleration_noise': (1., -1., 1.)},
    {'covariance_floor': 0.},
    {'max_update_gap': -1.},
))
def test_configuration_rejects_invalid_statistical_parameters(kwargs):
    with pytest.raises(ValueError):
        ImmPredictor(ImmConfig(**kwargs))


def test_prediction_batch_does_not_mutate_and_records_computation_budget():
    predictor = ImmPredictor()
    predictor.update(10., (0., 0., 0.), np.eye(3) * .01)
    timings = []
    for _ in range(30):
        started = time.perf_counter()
        forecast = predictor.predict(np.linspace(0., 4., 41))
        timings.append((time.perf_counter() - started) * 1000.)
    assert forecast.conditional_position_covariance.shape == (3, 41, 3, 3)
    # A very loose failure guard avoids asserting a machine-specific <10ms claim.
    assert np.percentile(timings, 95) < 1000.

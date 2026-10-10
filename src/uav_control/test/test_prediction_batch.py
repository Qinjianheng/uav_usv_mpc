"""Shared BCTRA integration must preserve the established scalar forecasts."""
import math

import numpy as np
import pytest

from uav_control.tracking.maneuvering_target_predictor import ManeuveringTargetPredictor


@pytest.mark.parametrize('turn,speed', [(0., 0.), (.7, 1.5), (-1.1, -2.), (.2, -.1)])
@pytest.mark.parametrize('period', [.1, .07, .02])
def test_batch_preserves_scalar_with_clamps_and_vertical_history(turn, speed, period):
    p = ManeuveringTargetPredictor()
    p.valid_turn_updates = 10
    p.turn_rate, p.turn_acceleration, p.speed_acceleration = turn, -1.2, speed
    p.update_vertical(-.1, .4, 100.)
    times = [0., *np.arange(period, 4., period), 4., .031, 2.03, .1]
    state = (10., -3., -.1, 3., -2., .4)
    expected = [p.predict(*state, t) for t in times]
    actual = p.predict_many(*state, times)
    assert np.allclose(actual, expected, rtol=0., atol=1e-11)
    assert p.turn_rate == turn and p.turn_acceleration == -1.2


def test_batch_known_constant_velocity_and_empty_series():
    p = ManeuveringTargetPredictor()
    actual = p.predict_many(1., 2., 3., 4., -2., 0., [0., 1., 2.])
    assert actual == ((1., 2., 3., 4., -2., 0.), (5., 0., 3., 4., -2., 0.),
                      (9., -2., 3., 4., -2., 0.))
    assert p.predict_many(1., 2., 3., 4., -2., 0., []) == ()


@pytest.mark.parametrize('times', [[-.1], [math.nan], [math.inf]])
def test_batch_rejects_invalid_prediction_times(times):
    with pytest.raises(ValueError):
        ManeuveringTargetPredictor().predict_many(0., 0., 0., 1., 0., 0., times)

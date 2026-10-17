"""Independent known extrema of planned thrust and tilt, including between-sample peaks."""
import numpy as np
import pytest
from uav_control.guidance import follow_attitude_bounds as bounds
from uav_control.guidance.planned_attitude import AttitudeConfig


def test_quadratic_acceleration_interior_peak_is_not_missed():
    # ax(t)=16*t*(1-t), peak4 at .5; g is constant, so exact tilt=atan(4/g).
    xyz = np.zeros((1, 6, 3))
    xyz[0, 3, 0] = 16/6
    xyz[0, 4, 0] = -16/12
    result = bounds.attitude_extrema(xyz, (1.,), AttitudeConfig())
    g = AttitudeConfig().gravity
    assert result['maximum_tilt'] == pytest.approx(np.arctan(4/g), abs=1e-9)
    assert result['maximum_thrust'] == pytest.approx(np.hypot(4, g), abs=1e-9)
    assert result['minimum_thrust'] == pytest.approx(g, abs=1e-9)


def test_simultaneous_horizontal_vertical_acceleration_fails_true_tilt_limit():
    xyz = np.zeros((1, 6, 3))
    xyz[0, 2] = (2.25, 0., 1.5)
    result = bounds.attitude_extrema(xyz, (1.,), AttitudeConfig())
    assert result['maximum_tilt'] > .55 and not result['valid']

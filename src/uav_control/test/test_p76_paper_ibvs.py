"""Pixel units and dimensionally consistent, explicitly adapted equation14."""
import numpy as np
import pytest

from uav_control.control.p76_paper_ibvs import image_yaw_rate, fov_holding_velocity


def test_pd_uses_actual_pixel_error_and_derivative():
    assert image_yaw_rate(10., -2., .03, .01) == pytest.approx(.28)
    assert image_yaw_rate(-10., 2., .03, .01) == pytest.approx(-.28)


def test_equation14_engineering_interpretation_scales_with_dt_not_control_count():
    a = fov_holding_velocity((4., 0., 0.), (0., 1., 0.), .005, 2., 7., 2.)
    assert np.linalg.norm(np.asarray(a)-(4., 0., 0.)) <= .01+1e-12
    b = fov_holding_velocity((4., 0., 0.), (1., 0., 0.), .005, 2., 7., 2.)
    assert b[0] == pytest.approx(4.01)
    assert abs(b[2]) <= 2.

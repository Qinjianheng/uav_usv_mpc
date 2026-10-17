"""Native estimator audits must not extrapolate or fabricate time alignment."""

import numpy as np
import pytest


from uav_control.analysis import native_time as AUDIT


def test_brackets_reject_outside_and_missing_history():
    values = np.array([[0., 0.], [1., 2.], [10., 20.]])
    result = AUDIT.interpolate_inside([-1, 0, 25000, 50000, 200000, 500001],
                                      [0, 50000, 500000], values)
    assert np.isnan(result[[0, 4, 5]]).all()
    np.testing.assert_allclose(result[1:4], [[0, 0], [.5, 1], [1, 2]])


def test_native_clock_reset_is_explicitly_rejected():
    with pytest.raises(ValueError):
        AUDIT.interpolate_inside([0], [1, 0], [[0., 0.], [1., 1.]])


def test_current_gnss_given_old_sample_time_creates_forward_motion_error():
    times = np.arange(0, 1000001, 10000)
    positions = np.column_stack([4 * times / 1e6, np.zeros(len(times))])
    current = AUDIT.interpolate_inside([500000], times, positions)
    wrongly_backdated = AUDIT.interpolate_inside([500000 - 110000], times, positions)
    np.testing.assert_allclose(current - wrongly_backdated, [[.44, 0]], atol=1e-12)


def test_gnss_projection_origin_and_axis_scales():
    result = AUDIT.project_gnss(np.array([0, 1e-5, 0]), np.array([0, 0, 1e-5]), 0, 0)
    np.testing.assert_allclose(result, [[0, 0], [1.111949266, 0], [0, 1.111949266]],
                               atol=1e-8)

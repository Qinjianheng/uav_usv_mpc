"""Current offline pairing rejects absent and wide acquisition brackets."""
import numpy as np

from uav_control.analysis.observation_pairing import bounded_interpolate


def test_kf_interpolation_requires_two_close_samples():
    values = [[0., 0., 0., 2., 0., 0.], [.2, 0., 0., 2., 0., 0.]]
    assert np.allclose(bounded_interpolate(1.05, [1., 1.1], values),
                       [.1, 0., 0., 2., 0., 0.])
    for stamp in (.99, 1.2):
        assert bounded_interpolate(stamp, [1., 1.1], values) is None
    assert bounded_interpolate(1.05, [1., 1.5], values) is None

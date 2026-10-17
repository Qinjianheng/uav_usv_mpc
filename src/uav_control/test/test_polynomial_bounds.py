"""Independent known polynomials and conservative derivative convex hull bounds."""
import numpy as np
import pytest

from uav_control.guidance.polynomial_bounds import derivative_upper_bound


def test_known_linear_speed_and_constant_acceleration():
    coefficients = np.zeros((1, 6, 3))
    coefficients[0, 1] = (1., 0., 0.)
    coefficients[0, 2] = (0., 1., 0.)
    assert derivative_upper_bound(coefficients, (2.,), 1, (0, 1)) == pytest.approx(17**.5)
    assert derivative_upper_bound(coefficients, (2.,), 2, (0, 1)) == pytest.approx(2.)
    assert derivative_upper_bound(coefficients, (2.,), 3, (0, 1)) == pytest.approx(0., abs=1e-9)


def test_convex_hull_is_upper_bound_not_claimed_actual_peak():
    # p(t)=t-t², velocity=1-2t, acceleration=-2
    # Known max |v|=1 on [0,1].
    coefficients = np.zeros((1, 6, 3))
    coefficients[0, 1, 0] = 1
    coefficients[0, 2, 0] = -1
    assert derivative_upper_bound(coefficients, (1.,), 1, (0,)) >= 1.
    # Tiny highest degree is retained even on a long interval (no noise trimming).
    coefficients[0, 5, 0] = 1e-12
    assert derivative_upper_bound(coefficients, (1000.,), 1, (0,)) >= 3.


def test_random_dense_samples_are_all_below_bound():
    rng = np.random.default_rng(91)
    c = rng.normal(size=(3, 6, 3))
    for d in (1, 2, 3):
        bound = derivative_upper_bound(c, (.8, .3, 1.6), d, (0, 1))
        for piece, duration in enumerate((.8, .3, 1.6)):
            polynomial = np.polynomial.polynomial.polyder(c[piece], m=d, axis=0)
            values = np.polynomial.polynomial.polyval(np.linspace(0, duration, 1001), polynomial).T
            assert np.max(np.linalg.norm(values[:, :2], axis=1)) <= bound

"""Paper equation9 uses previous measured velocity and wrapped LOS differences."""
import math

import numpy as np
import pytest

from uav_control.control.p76_paper_png import DualPlanePng, direction_from_angles


def test_dual_plane_matches_equation9_and_repeated_estimate_does_not_integrate_twice():
    png = DualPlanePng(3., 3.)
    first = direction_from_angles(.1, .2)
    velocity = 4*direction_from_angles(.05, .1)
    png.initialize(first, velocity, 10.)
    current = direction_from_angles(.12, .23)
    expected = direction_from_angles(.05+3*.02, .1+3*.03)
    result = png.update(current, velocity, 10.02)
    assert result == pytest.approx(expected)
    assert png.update(current, velocity, 10.02) == pytest.approx(result)
    assert np.linalg.norm(result) == pytest.approx(1.)


def test_wrap_crossing_and_zero_speed_have_deterministic_finite_output():
    png = DualPlanePng(3., 3.)
    png.initialize(direction_from_angles(0., math.pi-.01), (0., 0., 0.), 1.)
    result = png.update(direction_from_angles(0., -math.pi+.01), (0., 0., 0.), 1.02)
    assert np.isfinite(result).all() and np.linalg.norm(result) == pytest.approx(1.)
    with pytest.raises(ValueError, match='PNG_EPOCH'):
        png.update(result, (0., 0., 0.), .5)

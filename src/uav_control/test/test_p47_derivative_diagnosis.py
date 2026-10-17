"""Known quintic derivative oracle exercises current MINCO, without retired optimizers."""
import numpy as np
import pytest

from uav_control.guidance.minco_trajectory import MincoS3Trajectory
from uav_control.guidance.polynomial_extrema import derivative_peak


def test_exact_quintic_jerk_energy_and_peak_are_independently_known():
    trajectory = MincoS3Trajectory(
        (0., 0., 0.), (0., 0., 0.), (0., 0., 0.),
        (1., 0., 0.), (0., 0., 0.), (0., 0., 0.),
        intermediate_positions=((17/81, 0., 0.), (64/81, 0., 0.)),
        durations=(1/3,)*3)
    # p(t)=10t^3-15t^4+6t^5; integrate (60-360t+360t^2)^2 analytically.
    nodes, weights = np.polynomial.legendre.leggauss(4)
    jerks = np.asarray([trajectory.sample((node+1)/2).jerk for node in nodes])
    integral = np.dot(weights, np.sum(jerks**2, axis=1)) / 2
    assert integral == pytest.approx(720., rel=1e-8)
    # Preserve the pre-existing diagnostic negative result, without changing MINCO.
    assert trajectory.control_effort() == pytest.approx(2231.1111111111)
    assert trajectory.control_effort() != pytest.approx(integral)
    peak = derivative_peak(trajectory.coefficients, trajectory.durations, 3, (0, 1))
    assert peak['value'] == pytest.approx(60., rel=1e-8)
    assert min(abs(peak['time']), abs(peak['time']-1.)) < 1e-8
    for t in np.linspace(0., 1., 13):
        sample = trajectory.sample(t)
        assert sample.position[0] == pytest.approx(10*t**3-15*t**4+6*t**5, abs=1e-10)
        assert sample.velocity[0] == pytest.approx(30*t**2-60*t**3+30*t**4, abs=1e-10)
    assert trajectory.sample(0.).acceleration == pytest.approx((0., 0., 0.), abs=1e-10)
    assert trajectory.sample(1.).acceleration == pytest.approx((0., 0., 0.), abs=1e-10)

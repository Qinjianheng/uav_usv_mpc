"""Rejected measurements cannot poison the estimator epoch or numerical state."""

import numpy as np
import pytest

from test_visual_delivery_latency import filter_node, observation


@pytest.mark.parametrize('initialized', [False, True])
def test_future_image_does_not_block_the_next_current_image(initialized):
    now = [10.]
    node, states = filter_node(now)
    if initialized:
        node.measurement_callback(observation(10.))
    node.measurement_callback(observation(100.))
    assert node.last_measurement_time_ns != 100_000_000_000
    now[0] = 10.1
    node.measurement_callback(observation(10.1))
    assert node.last_measurement_time_ns == 10_100_000_000
    assert states[-1].valid


def test_rejected_covariance_does_not_mutate_state_or_advance_epoch():
    now = [10.]
    node, states = filter_node(now)
    node.measurement_callback(observation(10.))
    now[0] = 10.1
    node.measurement_callback(observation(10.1))
    before_state, before_cov = node.filter.state.copy(), node.filter.covariance.copy()
    count = len(states)
    now[0] = 10.2
    bad = observation(10.2)
    bad.covariance = [-1., 0., 0., 0., -1., 0., 0., 0., -1.]
    node.measurement_callback(bad)
    assert node.filter_time_ns == 10_100_000_000
    assert node.last_measurement_time_ns == 10_100_000_000
    assert np.array_equal(node.filter.state, before_state)
    assert np.array_equal(node.filter.covariance, before_cov)
    assert len(states) == count
    node.measurement_callback(observation(10.15))
    assert node.filter_time_ns == node.last_measurement_time_ns == 10_150_000_000


def test_numerical_update_failure_is_transactional(monkeypatch):
    from uav_control.tracking.constant_velocity_kalman import ConstantVelocityKalmanFilter
    now = [10.]
    node, _ = filter_node(now)
    node.measurement_callback(observation(10.))
    before = node.filter.covariance.copy()

    def failed_update(self, *args):
        self.state[:] = 999.
        raise np.linalg.LinAlgError('singular innovation')

    monkeypatch.setattr(ConstantVelocityKalmanFilter, 'update', failed_update)
    now[0] = 10.1
    node.measurement_callback(observation(10.1))
    assert np.array_equal(node.filter.covariance, before)
    assert np.max(node.filter.state) < 999.
    assert node.filter_time_ns == node.last_measurement_time_ns == 10_000_000_000

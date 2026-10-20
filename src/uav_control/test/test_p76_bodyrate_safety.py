"""Sea stopping distance uses the actual lower envelope and calibrated response delay."""
import numpy as np
import pytest

from uav_control.control.p76_bodyrate_safety import BodyrateSeaBarrier


def test_two_mps_descent_brakes_before_origin_boundary():
    barrier = BodyrateSeaBarrier([[0., 0., .1]], .33, .1, 4.)
    assert barrier.check([0., 0., -2.], [0., 0., 2.], np.eye(3)) == 'CLEAR'
    assert barrier.check([0., 0., -1.05], [0., 0., 2.], np.eye(3)) == 'BRAKE'
    assert barrier.check([0., 0., -.9], [0., 0., 2.], np.eye(3)) == 'UNRECOVERABLE'
    assert barrier.check([0., 0., -.32], [0., 0., 0.], np.eye(3)) == 'UNRECOVERABLE'
    assert barrier.check([0., 0., -1.05], [0., 0., 2.], np.eye(3),
                         sample_age=.125) == 'UNRECOVERABLE'
    assert barrier.check([0., 0., -.34], [0., 0., 0.], np.eye(3),
                         desired_acceleration=[0., 0., 3.]) == 'UNRECOVERABLE'


def test_tilted_envelope_and_missing_calibration_cannot_be_ignored():
    barrier = BodyrateSeaBarrier([[1., 0., .1]], .33, .1, 4.)
    rotation = np.array(((0., 0., -1.), (0., 1., 0.), (1., 0., 0.)))
    assert barrier.check([0., 0., -.9], [0., 0., 0.], rotation) == 'UNRECOVERABLE'
    with pytest.raises(ValueError):
        BodyrateSeaBarrier([[0., 0., .1]], .33, 0., 0.)

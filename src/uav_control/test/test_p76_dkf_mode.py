"""Delayed replay, source authority, and independent attitude-compensated image tests."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_control.tracking.delayed_spatial_kalman import DelayedSpatialKalman
from uav_control.tracking.delayed_image_kalman import DelayedImageKalman


def test_late_update_replays_identically_to_chronological_measurements():
    first, delayed = DelayedSpatialKalman(), DelayedSpatialKalman()
    covariance = np.eye(3)*.01
    for filter_ in (first, delayed):
        assert filter_.update((0., 0., 0.), covariance, 1., 1.)
    assert first.update((.08, -.02, .01), covariance, 1.02, 1.02)
    assert first.update((.16, -.04, .02), covariance, 1.04, 1.04)
    assert delayed.update((.16, -.04, .02), covariance, 1.04, 1.10)
    assert delayed.update((.08, -.02, .01), covariance, 1.02, 1.11)
    a, b = first.predict(1.11), delayed.predict(1.11)
    assert a.valid and b.valid
    assert a.state == pytest.approx(b.state)
    assert a.covariance == pytest.approx(b.covariance)
    assert b.source_stamp == 1.04
    assert np.linalg.eigvalsh(b.covariance).min() > 0


def test_prediction_and_duplicates_do_not_extend_real_measurement_authority():
    filter_ = DelayedSpatialKalman()
    assert filter_.update((0., 0., 0.), np.eye(3), 10., 10.05)
    assert not filter_.update((1., 1., 1.), np.eye(3), 10., 10.1)
    assert filter_.predict(10.125).valid
    assert not filter_.predict(10.126).valid
    assert not filter_.update((0., 0., 0.), np.eye(3), 10.01, 10.2)
    filter_.reset()
    assert not filter_.predict(10.2).valid


def test_image_dkf_compensates_camera_rotation_without_false_target_velocity():
    filter_ = DelayedImageKalman((320., 240., 300., 300.))
    identity = np.eye(3)
    assert filter_.update((320., 240.), np.eye(2), 1., 1., identity)
    camera = Rotation.from_euler('y', .1).as_matrix()
    ray = camera.T@np.array((0., 0., 1.))
    pixel = (320.+300.*ray[0]/ray[2], 240.)
    assert filter_.update(pixel, np.eye(2), 1.05, 1.08, camera)
    result = filter_.predict(1.1, camera, orientation_covariance=np.eye(3)*1e-6)
    assert result.valid and result.state[0] == pytest.approx(pixel[0]-320., abs=1e-6)
    assert abs(result.state[2]) < 1e-6
    assert result.source_stamp == 1.05
    uncertain = filter_.predict(1.1, camera, orientation_covariance=np.eye(3)*.001)
    assert uncertain.covariance[0, 0] > result.covariance[0, 0]

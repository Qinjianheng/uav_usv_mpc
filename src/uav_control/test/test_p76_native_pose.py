"""Native attitude history never extrapolates or replaces acquisition epochs."""
from types import SimpleNamespace as Packet

import numpy as np
import pytest

from uav_control.common.p76_native import (
    NativePoseHistory, NativeReferenceHistory, odometry_values,
)


def test_odometry_requires_sample_epoch_and_real_variances():
    message = Packet(timestamp_sample=42, pose_frame=1, q=[1., 0., 0., 0.],
                     angular_velocity=[0., 0., 0.], orientation_variance=[.01]*3)
    assert odometry_values(message)[0] == 42
    message.timestamp_sample = 0
    with pytest.raises(ValueError, match='SAMPLE'):
        odometry_values(message)
    message.timestamp_sample = 42
    message.orientation_variance[0] = float('nan')
    with pytest.raises(ValueError, match='VARIANCE'):
        odometry_values(message)


def test_native_pose_brackets_and_rejects_extrapolation():
    history = NativePoseHistory(.05)
    history.add(1., [1., 0., 0., 0.], [0., 0., 0.], [.01]*3)
    history.add(1.04, [1., 0., 0., 0.], [0., 0., 0.], [.01]*3)
    pose = history.at(1.02)
    np.testing.assert_allclose(pose.rotation, np.eye(3))
    assert pose.stamp == 1.02
    with pytest.raises(ValueError, match='BRACKET'):
        history.at(1.041)
    history.add(1.12, [1., 0., 0., 0.], [0., 0., 0.], [.01]*3)
    with pytest.raises(ValueError, match='BRACKET'):
        history.at(1.08)


def test_sea_datum_uses_native_reference_brackets_and_rejects_frame_change():
    history = NativeReferenceHistory()
    history.add(1., 4.)
    history.add(1.04, 4.)
    assert history.at(1.02) == 4.
    with pytest.raises(ValueError, match='BRACKET'):
        history.at(1.041)
    history.add(1.08, 4.2)
    with pytest.raises(ValueError, match='CHANGED'):
        history.at(1.06)

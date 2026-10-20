"""Preserve the initialized image posterior across Y authority handover."""
from collections import deque

import numpy as np

from uav_control.tracking.delayed_image_kalman import DelayedImageKalman
from uav_control.tracking.image_dkf_node import ImageDkfNode


def test_image_filter_does_not_reset_when_handover_becomes_active():
    node = ImageDkfNode.__new__(ImageDkfNode)
    node.filter = DelayedImageKalman((320., 240., 270., 270.))
    assert node.filter.update([320., 240.], np.eye(2), 10.02, 10.05, np.eye(3))
    node.filter_epoch = node.p76_epoch = 1
    node.p76_phase = 'PAPER_INTERCEPT_ACTIVE'
    node.mode_changed()
    assert node.filter.filter.source_stamp == 10.02
    node.cached = deque(['cached-only'], maxlen=12)
    node.p76_phase = 'SAFE_RECOVERY'
    node.mode_changed()
    assert node.filter.filter.source_stamp == 0.
    assert not node.pending


def test_invalid_initial_delayed_image_does_not_seed_a_camera_chart():
    estimator = DelayedImageKalman((320., 240., 270., 270.))
    assert not estimator.update([320., 240.], np.eye(2), 1., 1.126, np.eye(3))
    assert estimator.anchor_rotation is None

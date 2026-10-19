"""Native capture refuses missing, future, mismatched and over-wide pose brackets."""

from dataclasses import replace
from types import SimpleNamespace

import pytest

from uav_control.evaluation.gazebo_capture_geometry import native_capture_sample
from uav_control.evaluation.gazebo_entity_pose import TimedPoseHistory


def trackers():
    result = []
    for offset, height in ((.3, .35), (0., .42)):
        history = TimedPoseHistory()
        for t in (0., .025, .05):
            history.add(1.+t, 10.+t, (0., 4.*t+offset, height))
        result.append(SimpleNamespace(last_mapped_stamp=10.05, query_at=history.query_at))
    return result


def test_native_pair_preserves_epoch_frame_reference_and_velocity():
    uav, target = trackers()
    sample = native_capture_sample(uav, target, 10.06, .42)
    assert sample.stamp == 10.05 and sample.native_stamp == 1.05
    assert sample.uav.position == pytest.approx((.5, 0., -.35))
    assert sample.target.position == pytest.approx((.2, 0., 0.))
    assert sample.uav.velocity == pytest.approx((4., 0., 0.))
    assert sample.target.velocity == pytest.approx((4., 0., 0.))


@pytest.mark.parametrize('now', (10.04, 10.176, float('nan')))
def test_native_pair_rejects_future_old_and_invalid_time(now):
    assert native_capture_sample(*trackers(), now, .42) is None


@pytest.mark.parametrize('defect', ('gap', 'native_mismatch', 'missing', 'extrapolation'))
def test_native_pair_refuses_untrusted_geometry_without_estimate_fallback(defect):
    uav, target = trackers()
    original = target.query_at

    def query(t):
        value = original(t)
        if defect == 'gap':
            return replace(value, interval=.051)
        if defect == 'native_mismatch':
            return replace(value, left_sim_stamp=value.left_sim_stamp+.001,
                           right_sim_stamp=value.right_sim_stamp+.001)
        return replace(value, status='AFTER_HISTORY', value=None)

    target.query_at = query
    if defect == 'missing':
        target = None
    assert native_capture_sample(uav, target, 10.06, .42) is None

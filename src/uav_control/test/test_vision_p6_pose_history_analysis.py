"""Current localization reports acquisition/history ordering without retiming."""
from uav_control.perception.rgbd_target_localizer import pose_history_rejection_reason


def test_pose_history_gap_is_reported_by_current_localizer():
    assert pose_history_rejection_reason(10., (9., 9.98), (9., 10.01)) == (
        'POSITION_TIMESTAMP_AFTER_HISTORY')
    assert pose_history_rejection_reason(10., (9., 10.01), (9., 9.98)) == (
        'ATTITUDE_TIMESTAMP_AFTER_HISTORY')
    assert pose_history_rejection_reason(10., (9., 10.01), (9., 10.01)) == ''
    assert pose_history_rejection_reason(8., (9., 10.01), (9., 10.01)) == (
        'POSITION_TIMESTAMP_BEFORE_HISTORY')

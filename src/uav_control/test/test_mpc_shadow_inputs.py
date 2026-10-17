"""Causal full attitude pairing uses physical navigation epochs."""

import importlib
import math

import pytest
from px4_msgs.msg import TimesyncStatus, VehicleAttitude
from uav_usv_interfaces.msg import UavState


def adapter(**kwargs):
    module = importlib.import_module(
        'uav_control.controllers.mpc_shadow_inputs'
    )
    defaults = dict(
        maximum_sample_age=.5, maximum_receipt_age=.5,
        maximum_attitude_bracket_gap=.3, pose_wait_timeout=.1,
    )
    defaults.update(kwargs)
    return module.ShadowInputAdapter(**defaults)


def timesync(stamp=100., offset=-90.):
    message = TimesyncStatus()
    message.timestamp = round(stamp * 1e6)
    message.estimated_offset = round(offset * 1e6)
    return message


def attitude(source=100.1, yaw=0., reset_counter=0):
    message = VehicleAttitude()
    message.timestamp_sample = round(source * 1e6)
    message.q = [math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)]
    message.quat_reset_counter = reset_counter
    return message


def navigation(stamp=1000.2, native=10.1, source=100.1):
    message = UavState()
    message.stamp.sec = int(stamp)
    message.stamp.nanosec = round((stamp % 1) * 1e9)
    message.frame_id = 'local_ned'
    message.valid = True
    message.position.x, message.position.y, message.position.z = 1., 2., -5.
    message.velocity.x, message.velocity.y, message.velocity.z = 3., 4., .1
    message.acceleration.x = .5
    message.px4_timestamp_sample = round(source * 1e6)
    message.native_timestamp_sample = round(native * 1e6)
    return message


def clocks(subject):
    # Independent trusted anchors: sim runs at half the system clock rate.
    assert subject.add_clock_anchor(10., 1000., 1000.01, 1.)
    assert subject.add_clock_anchor(10.2, 1000.4, 1000.41, 1.4)


def ready(subject):
    clocks(subject)
    assert subject.add_timesync(timesync())
    assert subject.add_attitude(attitude(100.05), 1000.41, 1.41)
    assert subject.add_attitude(attitude(100.15, math.pi / 2),
                                1000.42, 1.42)
    assert subject.add_navigation(navigation(), 1000.43, 1.43)


def test_full_pose_uses_native_sample_epoch_and_keeps_provenance():
    subject = adapter()
    ready(subject)

    result = subject.pair_latest(1000.44, 1.44, observation_stamp=1000.39)

    assert result.status == 'PAIRED'
    pose = result.snapshot
    assert pose.stamp == pytest.approx(1000.2)
    assert pose.position == (1., 2., -5.)
    assert pose.velocity == (3., 4., .1)
    assert pose.acceleration == (.5, 0., 0.)
    assert pose.quaternion_wxyz == pytest.approx(
        (math.cos(math.pi / 8), 0., 0., math.sin(math.pi / 8)), abs=1e-7
    )
    assert pose.attitude_left_stamp == pytest.approx(1000.1)
    assert pose.attitude_right_stamp == pytest.approx(1000.3)
    assert pose.native_timestamp_sample == 10100000
    assert pose.px4_timestamp_sample == 100100000
    assert pose.attitude_left_px4_timestamp_sample == 100050000
    assert pose.attitude_right_native_timestamp_sample == 10150000
    assert pose.navigation_receipt_stamp == pytest.approx(1000.43)
    assert pose.observation_stamp == 1000.39


def test_causal_offset_does_not_use_a_newer_timesync_sample():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync(100., -90.))
    subject.add_timesync(timesync(100.18, -80.))
    subject.add_attitude(attitude(100.05), 1000.41, 1.41)
    subject.add_attitude(attitude(100.15, math.pi / 2), 1000.42, 1.42)
    subject.add_navigation(navigation(), 1000.43, 1.43)

    result = subject.pair_latest(1000.44, 1.44)

    assert result.status == 'PAIRED'
    assert result.snapshot.attitude_left_timesync_stamp == 100.
    assert result.snapshot.attitude_right_timesync_stamp == 100.
    assert result.snapshot.attitude_right_stamp == pytest.approx(1000.3)


def test_late_delivery_changes_age_but_never_changes_pose_epoch():
    early, late = adapter(), adapter()
    ready(early)
    clocks(late)
    late.add_timesync(timesync())
    late.add_attitude(attitude(100.05), 1000.48, 1.48)
    late.add_attitude(attitude(100.15, math.pi / 2), 1000.49, 1.49)
    late.add_navigation(navigation(), 1000.5, 1.5)

    first = early.pair_latest(1000.51, 1.51).snapshot
    second = late.pair_latest(1000.51, 1.51).snapshot

    assert first.stamp == second.stamp == pytest.approx(1000.2)
    assert first.quaternion_wxyz == second.quaternion_wxyz
    assert first.navigation_receipt_stamp != second.navigation_receipt_stamp


def test_latest_pairable_navigation_can_precede_latest_received_navigation():
    subject = adapter()
    ready(subject)
    subject.add_navigation(navigation(1000.36, 10.18, 100.18), 1000.45, 1.45)

    result = subject.pair_latest(1000.46, 1.46)

    assert result.status == 'PAIRED'
    assert result.snapshot.stamp == pytest.approx(1000.2)
    subject.add_attitude(attitude(100.2, math.pi), 1000.47, 1.47)
    assert subject.pair_latest(1000.48, 1.48).snapshot.stamp == (
        pytest.approx(1000.36)
    )


def test_navigation_waits_for_attitude_right_bracket_without_extrapolation():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync())
    subject.add_attitude(attitude(100.05), 1000.41, 1.41)
    subject.add_navigation(navigation(), 1000.42, 1.42)

    assert subject.pair_latest(1000.44, 1.44).status == (
        'ATTITUDE_AFTER_HISTORY'
    )
    subject.add_attitude(attitude(100.15), 1000.46, 1.46)
    assert subject.pair_latest(1000.47, 1.47).status == 'PAIRED'


def test_clock_bracket_wait_does_not_relabel_attitude_receipt_time():
    subject = adapter()
    subject.add_clock_anchor(10., 1000., 1000.01, 1.)
    subject.add_timesync(timesync())
    subject.add_attitude(attitude(100.05), 1000.31, 1.31)
    subject.add_attitude(attitude(100.15), 1000.32, 1.32)
    subject.add_navigation(navigation(), 1000.33, 1.33)

    assert subject.pair_latest(1000.34, 1.34).snapshot is None
    subject.add_clock_anchor(10.2, 1000.4, 1000.41, 1.4)

    pose = subject.pair_latest(1000.42, 1.42).snapshot
    assert pose.stamp == pytest.approx(1000.2)
    assert pose.attitude_left_stamp == pytest.approx(1000.1)
    assert pose.attitude_right_stamp == pytest.approx(1000.3)


def test_late_right_bracket_cannot_resurrect_a_timed_out_navigation():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync())
    subject.add_attitude(attitude(100.05), 1000.41, 1.41)
    subject.add_navigation(navigation(), 1000.42, 1.42)
    assert subject.pair_latest(1000.53, 1.53).status == 'POSE_WAIT_TIMEOUT'

    subject.add_attitude(attitude(100.15), 1000.54, 1.54)

    assert subject.pair_latest(1000.55, 1.55).status == 'POSE_WAIT_TIMEOUT'


@pytest.mark.parametrize('mutation, expected', [
    ('invalid', 'INVALID_NAVIGATION'),
    ('frame', 'INVALID_NAVIGATION'),
    ('nonfinite', 'INVALID_NAVIGATION'),
    ('zero_native', 'INVALID_NAVIGATION_TIMESTAMP'),
    ('zero_source', 'INVALID_NAVIGATION_TIMESTAMP'),
    ('future', 'NAVIGATION_FUTURE'),
])
def test_navigation_invalid_inputs_fail_closed(mutation, expected):
    subject = adapter()
    ready(subject)
    message = navigation()
    if mutation == 'invalid':
        message.valid = False
    elif mutation == 'frame':
        message.frame_id = 'map'
    elif mutation == 'nonfinite':
        message.velocity.x = math.nan
    elif mutation == 'zero_native':
        message.native_timestamp_sample = 0
    elif mutation == 'zero_source':
        message.px4_timestamp_sample = 0
    else:
        message = navigation(1000.5)

    assert not subject.add_navigation(message, 1000.45, 1.45)
    assert subject.pair_latest(1000.46, 1.46).status == expected


@pytest.mark.parametrize('which', ['sample', 'receipt'])
def test_navigation_sample_and_receipt_have_separate_age_limits(which):
    limits = dict(maximum_sample_age=.5, maximum_receipt_age=.5)
    limits['maximum_' + which + '_age'] = .05
    subject = adapter(**limits)
    ready(subject)

    result = subject.pair_latest(1000.49, 1.49)

    assert result.status == ('NAVIGATION_SAMPLE_STALE' if which == 'sample'
                             else 'NAVIGATION_RECEIPT_STALE')
    assert result.snapshot is None


@pytest.mark.parametrize('stamp, status', [
    (1000.5, 'OBSERVATION_FUTURE'),
    (1000., 'OBSERVATION_STALE'),
    (math.nan, 'INVALID_OBSERVATION_TIMESTAMP'),
])
def test_observation_stamp_is_checked_and_never_overwritten(stamp, status):
    subject = adapter(maximum_observation_age=.125)
    ready(subject)

    result = subject.pair_latest(1000.44, 1.44, observation_stamp=stamp)

    assert result.status == status
    assert result.snapshot is None


def test_missing_timesync_never_uses_attitude_receipt_as_a_clock():
    subject = adapter()
    clocks(subject)
    subject.add_attitude(attitude(), 1000.41, 1.41)
    subject.add_navigation(navigation(), 1000.42, 1.42)

    assert subject.pair_latest(1000.44, 1.44).status == 'TIMESYNC_UNAVAILABLE'
    assert subject.pair_latest(1000.55, 1.55).status == 'POSE_WAIT_TIMEOUT'


def test_native_navigation_and_trusted_clock_mapping_must_agree():
    subject = adapter()
    ready(subject)
    subject.add_navigation(navigation(1000.21), 1000.45, 1.45)

    result = subject.pair_latest(1000.46, 1.46)

    assert result.status == 'NAVIGATION_CLOCK_MISMATCH'
    assert result.snapshot is None


@pytest.mark.parametrize('reset_kind', [
    'sim', 'system', 'timesync', 'attitude',
])
def test_reset_clears_old_pose_and_increments_generation(reset_kind):
    subject = adapter()
    ready(subject)
    old_pose = subject.pair_latest(1000.44, 1.44).snapshot
    if reset_kind == 'sim':
        subject.add_clock_anchor(1., 1000.5, 1000.51, 1.5)
    elif reset_kind == 'system':
        subject.add_clock_anchor(10.3, 1002., 1002.01, 1.5)
    elif reset_kind == 'timesync':
        subject.add_timesync(timesync(99., -89.))
    else:
        subject.add_attitude(attitude(100.16, reset_counter=1), 1000.45, 1.45)

    result = subject.pair_latest(1000.52, 1.52)

    assert result.snapshot is None
    assert 'RESET' in result.status
    assert subject.clock_generation == old_pose.clock_generation + 1


def test_attitude_reset_requires_new_navigation_before_pairing():
    subject = adapter()
    ready(subject)
    subject.add_attitude(attitude(100.05, reset_counter=1), 1000.45, 1.45)
    subject.add_attitude(attitude(100.15, math.pi / 2, 1), 1000.46, 1.46)
    assert subject.pair_latest(1000.47, 1.47).snapshot is None

    subject.add_navigation(navigation(), 1000.48, 1.48)

    pose = subject.pair_latest(1000.49, 1.49).snapshot
    assert pose.clock_generation == 1
    assert pose.attitude_quat_reset_counter == 1


@pytest.mark.parametrize('which', ['quaternion', 'source'])
def test_invalid_attitude_cannot_leave_a_cached_valid_pair(which):
    subject = adapter()
    ready(subject)
    message = attitude(100.16)
    if which == 'quaternion':
        message.q = [0., 0., 0., 0.]
    else:
        message.timestamp_sample = 0

    assert not subject.add_attitude(message, 1000.45, 1.45)
    assert subject.pair_latest(1000.46, 1.46).status.startswith(
        'INVALID_ATTITUDE'
    )


def test_clock_staleness_rejects_even_a_previously_paired_pose():
    subject = adapter(maximum_clock_reference_age=.05)
    ready(subject)

    assert subject.pair_latest(1000.46, 1.46).status == 'CLOCK_REFERENCE_STALE'


def test_attitude_gap_is_bounded_even_when_navigation_is_inside_history():
    subject = adapter(maximum_attitude_bracket_gap=.1)
    ready(subject)

    assert subject.pair_latest(1000.44, 1.44).status == 'ATTITUDE_BRACKET_GAP'


def test_attitude_before_history_is_not_backwards_extrapolated():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync())
    subject.add_attitude(attitude(100.12), 1000.41, 1.41)
    subject.add_attitude(attitude(100.15), 1000.42, 1.42)
    subject.add_navigation(navigation(), 1000.43, 1.43)

    assert subject.pair_latest(1000.44, 1.44).status == (
        'ATTITUDE_BEFORE_HISTORY'
    )


def test_late_bracket_is_rejected_even_without_a_timer_poll_during_wait():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync())
    subject.add_attitude(attitude(100.05), 1000.41, 1.41)
    subject.add_navigation(navigation(), 1000.42, 1.42)

    subject.add_attitude(attitude(100.15), 1000.54, 1.54)

    assert subject.pair_latest(1000.55, 1.55).status == 'POSE_WAIT_TIMEOUT'


def test_roll_and_pitch_survive_shortest_arc_interpolation():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync())
    left, right = attitude(100.05), attitude(100.15)
    left.q = [1., 0., 0., 0.]
    # 90 degrees around the equal roll/pitch axis. Opposite quaternion sign
    # describes the same rotation and must choose the same shortest arc.
    right.q = [-math.sqrt(.5), -.5, -.5, 0.]
    subject.add_attitude(left, 1000.41, 1.41)
    subject.add_attitude(right, 1000.42, 1.42)
    subject.add_navigation(navigation(), 1000.43, 1.43)

    pose = subject.pair_latest(1000.44, 1.44).snapshot

    assert pose.quaternion_wxyz == pytest.approx(
        (.9238795325112867, .2705980500730985, .2705980500730985, 0.),
        abs=1e-7,
    )


@pytest.mark.parametrize('which', ['ros', 'monotonic'])
def test_receipt_clock_rewind_invalidates_old_warm_start(which):
    subject = adapter()
    ready(subject)
    subject.pair_latest(1000.44, 1.44)

    result = subject.pair_latest(1000.43 if which == 'ros' else 1000.45,
                                 1.43 if which == 'monotonic' else 1.45)

    assert result.status == 'ROS_TIME_RESET'
    assert result.snapshot is None
    assert subject.clock_generation == 1


def test_input_messages_are_copied_before_callback_owner_reuses_them():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync())
    q = attitude(100.05)
    subject.add_attitude(q, 1000.41, 1.41)
    q.q = [0., 0., 0., 0.]
    subject.add_attitude(attitude(100.15), 1000.42, 1.42)
    nav = navigation()
    subject.add_navigation(nav, 1000.43, 1.43)
    nav.position.x = 99.
    nav.native_timestamp_sample = 99

    pose = subject.pair_latest(1000.44, 1.44).snapshot

    assert pose.position == (1., 2., -5.)
    assert pose.native_timestamp_sample == 10100000
    assert pose.quaternion_wxyz == (1., 0., 0., 0.)


def test_timesync_newer_than_every_attitude_is_not_a_causal_offset():
    subject = adapter()
    clocks(subject)
    subject.add_timesync(timesync(100.2, -90.))
    subject.add_attitude(attitude(100.05), 1000.41, 1.41)
    subject.add_attitude(attitude(100.15), 1000.42, 1.42)
    subject.add_navigation(navigation(), 1000.43, 1.43)

    assert subject.pair_latest(1000.44, 1.44).status == 'TIMESYNC_UNAVAILABLE'


def test_missing_clock_and_future_native_sample_have_explicit_rejections():
    subject = adapter()
    subject.add_navigation(navigation(), 1000.42, 1.42)
    assert subject.pair_latest(1000.43, 1.43).status == (
        'CLOCK_REFERENCE_UNAVAILABLE'
    )
    clocks(subject)
    subject.add_navigation(navigation(1000.43, 10.21), 1000.44, 1.44)
    assert subject.pair_latest(1000.45, 1.45).status == 'CLOCK_BRACKET_AFTER'


def test_expired_pending_attitude_is_not_revived_by_a_late_clock_anchor():
    subject = adapter()
    subject.add_clock_anchor(10., 1000., 1000.01, 1.)
    subject.add_timesync(timesync())
    subject.add_attitude(attitude(100.05), 1000.2, 1.2)
    subject.add_attitude(attitude(100.15), 1000.21, 1.21)
    subject.add_clock_anchor(10.2, 1000.4, 1000.41, 1.4)
    subject.add_navigation(navigation(), 1000.42, 1.42)

    assert subject.pair_latest(1000.43, 1.43).status == 'ATTITUDE_WAIT_TIMEOUT'


@pytest.mark.parametrize('which', ['navigation', 'attitude'])
def test_delayed_older_same_stream_samples_do_not_replace_newer_history(which):
    subject = adapter()
    ready(subject)
    if which == 'navigation':
        accepted = subject.add_navigation(
            navigation(1000.18, 10.09, 100.09), 1000.45, 1.45,
        )
    else:
        accepted = subject.add_attitude(attitude(100.08), 1000.45, 1.45)

    assert not accepted
    pose = subject.pair_latest(1000.46, 1.46).snapshot
    assert pose.stamp == pytest.approx(1000.2)
    assert pose.quaternion_wxyz == pytest.approx(
        (.9238795325112867, 0., 0., .3826834323650898), abs=1e-7,
    )

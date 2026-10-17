"""The actual accepted start must wake the sole Tracker inside the unchanged window."""
from dataclasses import replace
from types import SimpleNamespace

from test_follow_sitl_execution import candidate, ctx
from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.guidance.follow_minco_execution import NominalFollowReceiver


def fixture():
    node = object.__new__(TrajectoryTrackerNode)
    node.follow_receiver = NominalFollowReceiver()
    node.follow_minco_requested = True
    curve = candidate(1, 100.15)
    assert node.follow_receiver.propose(curve, ctx(100.)).state == 'ACCEPTED'
    node.epoch = 100.
    node._ros_seconds = lambda: node.epoch
    node.last_timer_stamp = 100.
    node.test_wakeups, node.calls = [], []

    def create(period, callback):
        timer = SimpleNamespace(period=period, callback=callback, cancelled=False)
        timer.cancel = lambda: setattr(timer, 'cancelled', True)
        node.test_wakeups.append(timer)
        return timer

    node.create_timer = create
    node.destroy_timer = lambda timer: timer.cancel()

    def control():
        node.calls.append(node.epoch)
        # The real receiver still checks actual validation completion time.
        node.follow_receiver.tick(ctx(node.epoch+.003), True)

    node.timer_callback = control
    return node, curve


def test_accepted_start_wakes_same_control_path_and_preserves_lease():
    node, curve = fixture()
    node._schedule_follow_activation()
    assert abs(node.test_wakeups[-1].period-.15) < 1e-9
    node.epoch = curve.start
    node.test_wakeups[-1].callback()
    assert node.calls == [curve.start]
    assert node.follow_receiver.active == curve
    assert node.follow_receiver.deadline == curve.start+.45
    assert node.test_wakeups[-1].cancelled


def test_cancelled_or_replaced_pending_cannot_be_activated_by_old_wakeup():
    node, curve = fixture()
    node._schedule_follow_activation()
    old = node.test_wakeups[-1]
    node.follow_receiver.pending = replace(curve, plan_id=2, start=100.2)
    node.epoch = 100.15
    old.callback()
    assert not node.calls and node.follow_receiver.active is None


def test_regular_control_activation_before_wakeup_does_not_send_an_extra_command():
    node, curve = fixture()
    node._schedule_follow_activation()
    node.follow_receiver.tick(ctx(100.151), True)
    node.epoch = 100.152
    node.test_wakeups[-1].callback()
    assert not node.calls


def test_late_wakeup_still_rejects_missed_start_without_faking_ack_time():
    node, curve = fixture()
    node._schedule_follow_activation()
    node.epoch = curve.start+.06
    node.test_wakeups[-1].callback()
    assert node.follow_receiver.active is None
    assert any('START_MISSED' in ack.reasons for ack in node.follow_receiver.drain_events())


def test_nearby_regular_tick_defers_wakeup_to_existing_minimum_control_dt():
    node, curve = fixture()
    node._schedule_follow_activation()
    node.epoch, node.last_timer_stamp = curve.start, curve.start-.0002
    node.test_wakeups[-1].callback()
    assert not node.calls
    assert abs(node.test_wakeups[-1].period-.0008) < 1e-8
    node.epoch += .001
    node.test_wakeups[-1].callback()
    assert node.calls and node.follow_receiver.active == curve


def test_regular_callback_immediately_after_wakeup_does_not_use_default_fifty_ms_dt():
    node, _ = fixture()
    # Returning before state processing proves no command can receive the fallback dt.
    node.last_timer_stamp, node.epoch = 100.15, 100.1502
    node._process_pending_trajectory = lambda: (_ for _ in ()).throw(
        AssertionError('SUB_MILLISECOND_CONTROL_SHOULD_BE_SKIPPED'))
    TrajectoryTrackerNode.timer_callback(node)

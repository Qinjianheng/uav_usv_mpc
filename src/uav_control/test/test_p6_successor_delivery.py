"""Real admission gates remain separate from bounded successor dispatch opportunities."""
from collections import deque
from types import SimpleNamespace

import pytest

from test_p6_input_aligned_dispatch import runner, receive, finish
from test_p51_follow_admission import planner_fixture, actual_ack


def delivery_runner():
    r, pool, events = runner()
    r.successor_scheduling = 'delivery'
    r.current_receiver_epoch = ('receiver', 0, 1, 2)
    r.execution_status = dict(stamp=100.23, active=7, pending=0, deadline=100.60)
    r.next_solve = 10.277
    receive(r, 100.2, 100.235, 10.235, 2)
    return r, pool, events


def test_active_parent_dispatches_before_periodic_cooldown():
    r, pool, _ = delivery_runner()
    r.tick(100.24, 10.24)
    assert len(pool.requests) == 1
    assert r.pending['dispatch_timing']['policy'] == 'successor_delivery'


def test_failed_delivery_retries_only_on_new_acquisition_and_at_most_once():
    r, pool, _ = delivery_runner()
    r.tick(100.28, 10.28)
    finish(r, pool, 100.29, 10.29, valid=False)
    r.tick(100.30, 10.30)
    assert len(pool.requests) == 1
    receive(r, 100.25, 100.285, 10.285, 3)
    r.tick(100.30, 10.30)
    assert len(pool.requests) == 2
    finish(r, pool, 100.31, 10.31, valid=False)
    receive(r, 100.30, 100.335, 10.335, 4)
    r.tick(100.34, 10.34)
    assert len(pool.requests) == 2


def test_successful_delivery_waits_for_receiver_and_new_active_parent():
    r, pool, _ = delivery_runner()
    r.tick(100.28, 10.28)
    finish(r, pool, 100.29, 10.29)
    receive(r, 100.30, 100.335, 10.335, 3)
    r.execution_status.update(stamp=100.34, pending=8)
    r.tick(100.34, 10.34)
    assert len(pool.requests) == 1
    r.execution_status.update(stamp=100.44, pending=0, active=8, deadline=100.88)
    receive(r, 100.40, 100.435, 10.435, 4)
    r.tick(100.44, 10.44)
    assert len(pool.requests) == 2


def test_no_successor_work_with_stale_receiver_or_insufficient_activation_margin():
    r, pool, _ = delivery_runner()
    r.tick(100.44, 10.44)  # status is now older than 200ms
    assert not pool.requests
    r.execution_status.update(stamp=100.44)
    r.tick(100.44, 10.44)  # start .59 cannot be strictly before deadline-.05
    assert not pool.requests


def test_delivery_budget_counts_whole_worker_once_and_actual_prepare_queue():
    r, _, _ = delivery_runner()
    r.queue_samples = deque([.003]*8, maxlen=8)
    r.solver.online_budget_guard = True
    r.solver.follow_estimate = .030
    from uav_control.guidance.follow_cost_evidence import evidence_for
    evidence_for(r.solver).observe('follow', .030/1.2, 100.2)
    r.solver.fallback_enabled = False
    r.solver.fast = SimpleNamespace(publication_reserve=.020)
    r.solver.wall_clock = lambda: 100.245
    request, _ = r._prepare(100.24, 10.24)
    assert request.context.observation_stamp == 100.2
    assert r.delivery_budget['available_worker_seconds'] == pytest.approx(.040)
    r.solver.wall_clock = lambda: 100.26
    with pytest.raises(ValueError, match='SUCCESSOR_AWAITING_FRESH_INPUT'):
        r._prepare(100.24, 10.24)


def test_active_ack_updates_delivery_status_without_heartbeat_delay():
    n = planner_fixture()
    n.runner.successor_scheduling = 'delivery'
    n.receiver_status = dict(stamp=100.04, active=0, pending=1, deadline=0.,
                             allowed=True, converged=True, state='FOLLOW', inhibition='')
    actual_ack(n, 1, 100.05)
    assert n.runner.execution_status['active'] == 1
    assert n.runner.execution_status['pending'] == 0
    assert n.runner.execution_status['deadline'] == pytest.approx(100.49)


def test_invalid_active_ack_cannot_update_delivery_status():
    n = planner_fixture()
    n.runner.successor_scheduling = 'delivery'
    actual_ack(n, 2, 100.01)  # before curve start
    assert n.accepted_curve is None
    assert getattr(n.runner, 'execution_status', None) is None


def test_heartbeat_older_than_actual_active_ack_cannot_roll_back_status():
    from uav_usv_interfaces.msg import FollowReceiverState
    from uav_control.controllers.follow_transport import stamp
    n = planner_fixture()
    n.runner.successor_scheduling = 'delivery'
    n.receiver_status = dict(stamp=100.04, active=0, pending=1, deadline=0.)
    actual_ack(n, 1, 100.05)
    m = FollowReceiverState()
    m.receiver, m.receiver_boot_id, m.mission_id = 'trajectory_tracker_node', 'receiver', 1
    m.stamp, m.active_plan_id = stamp(100.045), 0
    n.epoch_callback(m)
    assert n.runner.execution_status['active'] == 1
    m.stamp, m.active_plan_id = stamp(100.06), 1
    n.epoch_callback(m)
    assert n.receiver_status['stamp'] == pytest.approx(100.06)


def test_terminal_ack_still_clears_actual_accepted_curve_in_delivery_mode():
    n = planner_fixture()
    n.runner.successor_scheduling = 'delivery'
    actual_ack(n, 1, 100.05)
    actual_ack(n, 1, 100.06, 'EXPIRED')
    assert n.accepted_curve is None
    actual_ack(n, 1, 100.05)
    assert n.accepted_curve is None


def test_delivery_exception_is_recorded_and_does_not_break_timer():
    r, pool, events = delivery_runner()
    r.tick(100.24, 10.24)
    pool.future.set_exception(RuntimeError('worker failed'))
    r.tick(100.25, 10.25)
    assert r.pending is None
    assert events[-1]['output']['reason'] == 'SOLVER_EXCEPTION'


def test_delivery_budget_recovers_from_aged_timing_spike_without_erasing_recent_cost():
    from uav_control.guidance.follow_cost_evidence import evidence_for
    r, _, _ = delivery_runner()
    r.queue_samples = deque([.003]*8, maxlen=8)
    r.solver.online_budget_guard = True
    r.solver.follow_estimate, r.solver.fallback_enabled = .12, False
    r.solver.fast = SimpleNamespace(publication_reserve=.020)
    evidence_for(r.solver).observe('follow', .10, 100.2)
    r.solver.wall_clock = lambda: 100.24
    with pytest.raises(ValueError, match='SUCCESSOR_AWAITING_FRESH_INPUT'):
        r._prepare(100.24, 10.24)
    # Hints may age once per acquired image; protected recent whole-worker
    # measurements still impose a floor. No actual input or lease is extended.
    accepted = False
    for i in range(1, 41):
        acquired = 100.2+.05*i
        receive(r, acquired, acquired+.035, acquired-90.+.035, 2+i)
        r.solver.wall_clock = lambda: acquired+.04
        try:
            r._prepare(acquired+.04, acquired-90.+.04)
            accepted = True
            break
        except ValueError as error:
            assert str(error) == 'SUCCESSOR_AWAITING_FRESH_INPUT'
    assert accepted


def test_delivery_margin_uses_configured_execution_lead():
    from functools import partial
    from uav_control.controllers.follow_mpc_shadow_node import make_shadow_request
    r, pool, _ = delivery_runner()
    r.execution_lead = .35
    r.request_factory = partial(make_shadow_request, execution_lead=.35)
    r.tick(100.24, 10.24)
    assert not pool.requests


def test_accepted_ack_blocks_duplicate_delivery_without_granting_active_control():
    from uav_usv_interfaces.msg import FollowPlanAck
    from uav_control.controllers.follow_transport import stamp
    n = planner_fixture()
    n.runner.successor_scheduling = 'delivery'
    n.receiver_status = dict(stamp=100.04, active=0, pending=0, deadline=0.)
    m = FollowPlanAck()
    m.receiver = 'trajectory_tracker_node'
    m.receiver_boot_id, m.planner_boot_id = 'receiver', 'planner'
    m.mission_id, m.plan_id, m.pending_plan_id = 1, 1, 1
    m.state, m.stamp = 'ACCEPTED', stamp(100.05)
    n.ack_callback(m)
    assert n.accepted_curve is None
    assert n.runner.execution_status['active'] == 0
    assert n.runner.execution_status['pending'] == 1


def test_completion_checks_original_handover_without_building_a_later_unused_request():
    from uav_control.controllers.follow_mpc_shadow_node import make_shadow_request
    r, pool, events = delivery_runner()
    r.execution_status['deadline'] = 100.48
    constructions = []

    def factory(*args):
        request = make_shadow_request(*args, execution_lead=.15)
        constructions.append(request)
        if request.context.execution_start_stamp >= r.execution_status['deadline']-.05:
            raise ValueError('NO_SUCCESSOR_ACTIVATION_MARGIN')
        return request
    r.request_factory = factory
    r.tick(100.24, 10.24)
    finish(r, pool, 100.30, 10.30)
    assert events[-1]['output']['valid']
    assert len(constructions) == 1
    assert events[-1]['request']['context']['execution_start_stamp'] == pytest.approx(100.39)

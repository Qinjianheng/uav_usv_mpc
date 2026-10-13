"""Receiver-owned entry and recovery policy; no simulated ACK grants authority."""
from dataclasses import replace

import pytest

from test_follow_sitl_execution import candidate, ctx


def policy(**changes):
    from uav_control.guidance.follow_admission import AdmissionConfig, FollowAdmission
    return FollowAdmission(AdmissionConfig(**changes))


def metrics(error=.2, relative=.1, height=.02):
    return dict(horizontal_error=error, relative_speed=relative, height_error=height,
                settled=error <= 1. and relative <= .5 and height <= .1)


def stable(g, start, seconds=5.1):
    for i in range(round(seconds/.05)+1):
        g.update(start+i*.05, metrics(), True)


def probes(g, now):
    for i in range(3):
        g.probe(now-.1+i*.05, i+1, True)


def test_first_entry_requires_continuous_convergence_and_fresh_probe_population():
    g = policy()
    g.update(100., metrics(8., 4.), True)
    assert not g.allowed(100.)
    stable(g, 100.05)
    assert not g.allowed(105.15)
    probes(g, 105.15)
    assert g.allowed(105.15)
    g.update(105.2, metrics(1.01), True)
    assert not g.allowed(105.2)


def test_active_execution_ignores_entry_threshold_noise_but_has_performance_hysteresis():
    g = policy()
    for t in (100., 100.1, 100.2, 100.3):
        assert not g.update(t, metrics(1.2, .6), True, active=True)
        assert g.state == 'MINCO_ACTIVE'
    assert not g.update(100.4, metrics(1.6, .8), True, active=True)
    assert g.update(100.91, metrics(1.6, .8), True, active=True) == 'PERFORMANCE_DEVIATION'


def test_recovery_requires_cooldown_then_new_five_seconds_and_recent_probes():
    g = policy()
    g.lost(100., 'LEASE_EXPIRED_NO_SUCCESSOR', candidate())
    stable(g, 100., 6.9)
    probes(g, 106.9)
    assert not g.allowed(106.9)
    stable(g, 106.95, .3)
    probes(g, 107.25)
    assert g.allowed(107.25)
    assert g.loss['reason'] == 'LEASE_EXPIRED_NO_SUCCESSOR'
    assert g.loss['plan_id'] == 1


def test_invalid_input_gap_failed_probe_and_clock_reversal_reset_qualification():
    for failure in ('input', 'gap', 'probe', 'clock'):
        g = policy()
        stable(g, 100.)
        probes(g, 105.1)
        assert g.allowed(105.1)
        if failure == 'input':
            g.update(105.15, metrics(), False)
        elif failure == 'gap':
            g.update(105.5, metrics(), True)
        elif failure == 'probe':
            g.probe(105.15, 9, False)
        else:
            g.update(104., metrics(), True)
        assert not g.allowed(g.last_now)


def test_repeated_actual_entry_failures_are_bounded_until_new_identity():
    g = policy(maximum_entries=2, stable_seconds=.1, cooldown_seconds=.1)
    for i in range(2):
        start = 100.+i
        stable(g, start, .3)
        probes(g, start+.3)
        assert g.allowed(start+.3)
        g.consume()
        g.lost(start+.35, 'START_MISSED', candidate(i+1))
    stable(g, 103., .3)
    probes(g, 103.3)
    assert not g.allowed(103.3)


def test_receiver_rejects_entry_without_gate_and_shadow_probe_never_executes():
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    g = policy()
    r = NominalFollowReceiver(admission=g)
    g.update(100.01, metrics(8., 4.), True)
    ack = r.propose(candidate(), ctx())
    assert ack.state == 'REJECTED' and 'FOLLOW_ENTRY_INHIBITED' in ack.reasons
    r.propose(candidate(2), ctx(), probe=True)
    assert r.active is None and r.pending is None


def test_expired_parent_cannot_activate_pending_child_and_both_acks_survive():
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    r = NominalFollowReceiver()
    old = candidate()
    r.propose(old, ctx())
    r.tick(ctx(old.start), nominal_valid=True)
    new = replace(candidate(2, old.start+.44, 1),
                  xyz=((.44, 0., -5.), *old.xyz[1:]),
                  navigation_stamp=100.4, attitude_stamp=100.4,
                  observation_stamp=100.4, source_stamp=100.4, input_until=100.525)
    assert r.propose(new, ctx(100.42)).state == 'ACCEPTED'
    r.tick(ctx(old.start+.46), nominal_valid=True)
    assert r.active is None and r.pending is None
    events = r.drain_events()
    assert any(a.plan_id == 1 and a.state == 'EXPIRED' for a in events)
    assert any(a.plan_id == 2 and 'PARENT_EXPIRED_AT_ACTIVATION' in a.reasons for a in events)


def test_timely_successor_does_not_enter_recovery_or_reset_command_history():
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    r = NominalFollowReceiver()
    old = candidate()
    r.propose(old, ctx())
    r.tick(ctx(old.start), nominal_valid=True)
    new = replace(candidate(2, 100.1, 1), xyz=((.06, 0., -5.), *old.xyz[1:]))
    assert r.propose(new, ctx(100.06)).state == 'ACCEPTED'
    assert r.tick(ctx(100.11), nominal_valid=True).state == 'ACTIVE'
    assert r.active == new and r.deadline == pytest.approx(100.55)


def test_real_tracker_refreshes_gate_before_candidate_admission(monkeypatch):
    from types import SimpleNamespace
    from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
    from uav_usv_interfaces.msg import FollowTrajectory
    node = object.__new__(TrajectoryTrackerNode)
    calls = []
    node.follow_minco_requested = True
    node.follow_receiver = SimpleNamespace(active=None)
    node._follow_context = lambda: ctx()
    node._update_follow_admission = lambda context: calls.append(context.now)
    node.follow_ack_pub = SimpleNamespace(publish=lambda a: None)
    node.follow_debug_pub = SimpleNamespace(publish=lambda a: None)
    # Invalid coefficients still cannot prevent refreshing the authority gate.
    node.follow_trajectory_callback(FollowTrajectory())
    assert calls == [100.01]


def test_planner_publication_path_is_blocked_without_fresh_receiver_eligibility():
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    node = object.__new__(P4FollowPlannerNode)
    node.follow_execution_enabled = True
    node.follow_admission_required = True
    node.receiver_status = None
    assert node._execution_publication_policy(100.) == 'RECEIVER_ADMISSION_UNAVAILABLE'
    node.receiver_status = dict(stamp=99.99, state='FOLLOW_APPROACH', allowed=False,
                                active=0, pending=0)
    assert node._execution_publication_policy(100.) == 'FOLLOW_ENTRY_INHIBITED'
    node.receiver_status.update(state='FOLLOW_STABLE')
    assert node._execution_publication_policy(100.) == 'SHADOW_PROBE'
    node.receiver_status.update(allowed=True)
    assert node._execution_publication_policy(100.) == ''
    node.receiver_status.update(state='MINCO_ACTIVE', active=7, allowed=False)
    assert node._execution_publication_policy(100.) == ''
    assert node._execution_publication_policy(100.3) == 'RECEIVER_ADMISSION_UNAVAILABLE'


def test_online_budget_guard_primes_only_target_independent_geometry():
    from test_p47_follow_guided_solver import solver
    from test_follow_research_algorithms import request
    from uav_control.guidance.follow_problem import future_request
    from uav_control.guidance.follow_terminal_correction import linear_geometry
    linear_geometry.cache_clear()
    solver(future_request(request(2.), .15), online_budget_guard=True, refinement=False)
    assert linear_geometry.cache_info().currsize == 1


@pytest.mark.parametrize('remaining,expected_fallback', [(.020, True), (.008, False)])
def test_budget_guard_does_not_start_follow_or_fallback_without_stage_budget(
        remaining, expected_fallback, monkeypatch):
    from test_p47_follow_guided_solver import solver
    from test_follow_research_algorithms import request
    from uav_control.guidance.follow_problem import future_request
    from uav_control.guidance.fast_follow_minco import freshness
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    req = future_request(request(2.), .15)
    s = solver(req, online_budget_guard=True, refinement=False, clock=lambda: 0.)
    until, _ = freshness(req, req.now_stamp)
    s.wall_clock = lambda: until-.02-remaining
    called = []
    monkeypatch.setattr('uav_control.controllers.follow_guided_feasible.rollout_follow_fast',
                        lambda *a, **k: pytest.fail('too little budget for FOLLOW/QVA'))

    def fallback(r):
        called.append(r)
        return MpcSeedResult(r.context, valid=True)

    s.fallback._solve = fallback
    result = s.solve(req)
    assert bool(called) == expected_fallback
    assert result.valid == expected_fallback


@pytest.mark.parametrize('active,fast_retry', [(7, True), (0, False)])
def test_failed_successor_has_one_deadline_driven_retry_without_a_queue(active, fast_retry):
    from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner
    from uav_control.controllers.follow_mpc_seed import MpcConfig, MpcSeedResult
    from test_follow_mpc_shadow_node import _Adapter, _Pool, _mission, _prediction
    pool, events = _Pool(), []
    r = P4ResearchRunner(MpcConfig(), _Adapter(), pool, events.append, rate_hz=5.)
    r.deadline_scheduling = True
    r.execution_status = dict(stamp=100.04, active=active, deadline=100.35)
    r.update_mission(_mission(), 100.01, 1.)
    r.update_prediction(_prediction(), 100.01, 1.)
    r.tick(100.02, 1.02)
    pool.future.set_result(MpcSeedResult(pool.requests[0].context, valid=False))
    r.tick(100.04, 1.04)
    assert (r.next_solve < 1.1) == fast_retry
    assert len(pool.requests) == 1
    if fast_retry:
        from concurrent.futures import Future
        pool.future = Future()
        r.tick(100.06, 1.06)
        pool.future.set_result(MpcSeedResult(pool.requests[-1].context, valid=False))
        r.tick(100.08, 1.08)
        assert r.next_solve >= 1.2  # at most one extra retry for this active parent


def test_receiver_full_validation_failure_breaks_the_probe_success_streak(monkeypatch):
    from types import SimpleNamespace
    from test_p4_receiver_node import event
    from test_follow_mpc_shadow_node import _prediction
    from uav_control.controllers.follow_transport import proposal_from_event
    from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    node = object.__new__(TrajectoryTrackerNode)
    g = policy()
    probes(g, 100.01)
    assert len(g.probes) == 3
    node.follow_receiver = NominalFollowReceiver(admission=g)
    node.follow_minco_requested, node.follow_stable_ticks = True, 10
    node._follow_context = lambda: ctx()
    node._update_follow_admission = lambda c: ''
    node.follow_model = None
    node.latest_prediction = _prediction()
    node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='p4_follow_planner_node')]
    node.follow_ack_pub = node.follow_debug_pub = SimpleNamespace(publish=lambda m: None)
    msg = proposal_from_event(event(), 100.)
    msg.reason = 'SHADOW_PROBE'
    monkeypatch.setattr('uav_control.guidance.follow_minco_execution.nominal_attitude',
                        lambda *args: False)
    node.follow_trajectory_callback(msg)
    assert not g.probes


@pytest.mark.parametrize('ack_stamp,owner,active,expected', [
    (100.02, 'MINCO_FOLLOW', 1, False), (100.05, 'ORIGINAL_FOLLOW', 1, False),
    (100.05, 'MINCO_FOLLOW', 999, False), (100.6, 'MINCO_FOLLOW', 1, False),
    (100.05, 'MINCO_FOLLOW', 1, True)])
def test_active_ack_requires_physical_timing_and_actual_owner(ack_stamp, owner, active, expected):
    from io import StringIO
    import json
    from types import SimpleNamespace
    from uav_usv_interfaces.msg import FollowPlanAck
    from uav_control.controllers.follow_transport import stamp
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    n = object.__new__(P4FollowPlannerNode)
    n._ros_seconds = lambda: ack_stamp+.01
    n.follow_execution_enabled = True
    n.planner_boot_id = 'planner'
    n.receiver_epoch = (('receiver', 0, 1, 0), ack_stamp)
    n.runner = SimpleNamespace(mission={'mission_id': 1})
    key = ('receiver', 'planner', 1, 0, 1)
    n.expected_plans, n.execution_curves = {key: 100.01}, {key: candidate()}
    n.accepted_curve, n.ack_file = None, StringIO()
    n.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='trajectory_tracker_node')]
    message = FollowPlanAck()
    message.receiver_boot_id, message.planner_boot_id = 'receiver', 'planner'
    message.receiver, message.mission_id, message.plan_id = 'trajectory_tracker_node', 1, 1
    message.stamp, message.state = stamp(ack_stamp), 'ACTIVE'
    message.control_owner, message.active_plan_id = owner, active
    n.ack_callback(message)
    assert (n.accepted_curve is not None) == expected
    assert json.loads(n.ack_file.getvalue())['identity_valid'] == expected


@pytest.mark.parametrize('callback', ['heartbeat', 'proposal'])
def test_brief_context_loss_observed_between_ticks_cannot_disappear(callback):
    from types import SimpleNamespace
    from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    from uav_usv_interfaces.msg import FollowTrajectory
    node = object.__new__(TrajectoryTrackerNode)
    g = policy()
    node.follow_receiver = r = NominalFollowReceiver(admission=g)
    r.active, r.deadline = candidate(), 100.49
    g.update(100.04, metrics(), True, active=True)
    node.follow_minco_requested = True
    node.follow_admission_identity = ('receiver', 0, 1)
    node.follow_navigation_history = [dict(valid=True, stamp=100.04,
                                           p=[0., 0., -5.], v=[1., 0., 0.])]
    node.follow_target_history = [dict(valid=True, stamp=100.04, source=100.04,
                                       p=[5., 0., 0.], v=[1., 0., 0.])]
    node.flight_guidance = SimpleNamespace(follow_distance=5., flight_altitude=-5.)
    node.vehicle_status_stamp, node.vehicle_status_timeout = 100.04, .75
    context = ctx(100.06, offboard=False)
    node._follow_context = lambda: context
    node.follow_clock = SimpleNamespace(boot_id='receiver')
    messages = []
    node.follow_epoch_pub = node.follow_ack_pub = node.follow_debug_pub = SimpleNamespace(
        publish=messages.append)
    if callback == 'heartbeat':
        node._publish_follow_epoch()
    else:
        node.follow_trajectory_callback(FollowTrajectory())
    context = ctx(100.08)
    node._update_follow_admission(context)
    r.tick(context, nominal_valid=True)
    assert r.active is None and r.pending is None
    assert g.loss['reason'] == 'EXECUTION_CONTEXT_INVALID'
    assert g.state == 'FOLLOW_RECOVERY' and not g.allowed(100.08)
    assert any(getattr(m, 'state', '') == 'REVOKED' for m in messages)


def planner_fixture():
    from io import StringIO
    from types import SimpleNamespace
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    n = object.__new__(P4FollowPlannerNode)
    n._ros_seconds = lambda: 100.12
    n.follow_execution_enabled, n.planner_boot_id = True, 'planner'
    n.receiver_epoch = (('receiver', 0, 1, 0), 100.04)
    n.receiver_status = None
    n.adapter = SimpleNamespace(clock_generation=0)
    n.runner = SimpleNamespace(mission={'mission_id': 1})
    curves = [candidate(), candidate(2, 100.1, 1)]
    n.execution_curves = {('receiver', 'planner', 1, 0, c.plan_id): c for c in curves}
    n.expected_plans = {k: 100.01 for k in n.execution_curves}
    n.accepted_curve, n.ack_file = None, StringIO()
    n.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='trajectory_tracker_node')]
    return n


def actual_ack(node, plan, epoch, state='ACTIVE'):
    from uav_usv_interfaces.msg import FollowPlanAck
    from uav_control.controllers.follow_transport import stamp
    m = FollowPlanAck()
    m.receiver_boot_id, m.planner_boot_id = 'receiver', 'planner'
    m.receiver, m.mission_id, m.plan_id = 'trajectory_tracker_node', 1, plan
    m.stamp, m.state = stamp(epoch), state
    m.control_owner, m.active_plan_id = 'MINCO_FOLLOW', plan
    node.ack_callback(m)


def test_old_active_ack_cannot_roll_back_a_newer_parent_or_resurrect_a_terminal_plan():
    n = planner_fixture()
    actual_ack(n, 1, 100.05)
    actual_ack(n, 2, 100.11)
    actual_ack(n, 1, 100.05)
    assert n.accepted_curve.plan_id == 2
    actual_ack(n, 2, 100.115, 'REVOKED')
    actual_ack(n, 2, 100.11)
    assert n.accepted_curve is None
    # Equal-stamp terminal events dominate even a reordered ACTIVE.
    n = planner_fixture()
    actual_ack(n, 1, 100.05, 'EXPIRED')
    actual_ack(n, 1, 100.05)
    assert n.accepted_curve is None


def test_older_heartbeat_cannot_override_new_receiver_state_in_the_same_identity():
    from uav_usv_interfaces.msg import FollowReceiverState
    from uav_control.controllers.follow_transport import stamp
    n = planner_fixture()
    m = FollowReceiverState()
    m.receiver, m.receiver_boot_id, m.mission_id = 'trajectory_tracker_node', 'receiver', 1
    m.stamp, m.active_plan_id = stamp(100.11), 2
    n.epoch_callback(m)
    m.stamp, m.active_plan_id = stamp(100.05), 1
    n.epoch_callback(m)
    assert n.receiver_status['active'] == 2
    assert n.receiver_epoch[1] == pytest.approx(100.11)

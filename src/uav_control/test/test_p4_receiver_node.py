"""Real ROS receiver boundary, default-off isolation and wire layouts."""
from dataclasses import asdict, replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest

from uav_usv_interfaces.msg import FollowTrajectory
from uav_control.controllers.follow_transport import (
    curve_from_message, proposal_from_event, ack_message,
)
from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.guidance.follow_contract import FollowReceiver, ReceiverContext

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from follow_minco_experiment import scenario  # noqa: E402
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig  # noqa: E402
from uav_control.controllers.progress_follow_solver import ProgressFollowSolver  # noqa: E402


def event():
    from uav_control.guidance.follow_problem import future_request
    base = scenario('constant_velocity')
    base = replace(base, context=replace(base.context, cycle_id=1))
    req = future_request(base, .08)
    m = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
    out = ProgressFollowSolver(m, wall_clock=lambda: req.now_stamp).solve(req)
    assert out.valid
    return dict(output=asdict(out), request=asdict(req))


def test_complete_wire_roundtrip_and_explicit_zero_holding():
    e = event()
    msg = proposal_from_event(e, 100.)
    c = curve_from_message(msg)
    assert c.holding_until == 0. and not c.holding_model
    assert c.source_stamp != c.start
    assert np.allclose(c.sample(c.start)[:9], e['request']['state'][:9])
    assert len(msg.xyz_coefficients) == 54 and len(msg.yaw_coefficients) == 12
    assert proposal_from_event(e, 101.) is None


def test_actual_tracker_callback_rejects_without_touching_control_or_yaw():
    e = event()
    msg = proposal_from_event(e, 100.)
    n = object.__new__(TrajectoryTrackerNode)
    n.follow_receiver = FollowReceiver()
    n._follow_context = lambda: ReceiverContext(
        100., 1, 0, msg.prediction_sequence_id, True, True, True, True)
    published = []
    n.follow_ack_pub = type('P', (), {'publish': lambda _, x: published.append(x)})()
    n.follow_trajectory_callback(msg)
    assert len(published) == 1
    a = published[0]
    assert a.state == 'REJECTED' and a.plan_id == msg.plan_id
    assert a.receiver == 'trajectory_tracker_node' and not a.replaced
    assert a.control_owner == 'ORIGINAL_FOLLOW'
    assert 'NO_QUALIFIED_HOLDING_MODEL' in a.reasons
    assert 'INITIAL_BRIDGE_UNAVAILABLE' in a.reasons
    assert n.follow_receiver.active is None


def test_malformed_and_nan_wire_have_actual_rejection_ack():
    n = object.__new__(TrajectoryTrackerNode)
    n.follow_receiver = FollowReceiver()
    n._follow_context = lambda: ReceiverContext(100., 1, 0, 1, True, True, True, True)
    pub = []
    n.follow_ack_pub = type('P', (), {'publish': lambda _, x: pub.append(x)})()
    msg = FollowTrajectory()
    msg.plan_id = 4
    n.follow_trajectory_callback(msg)
    assert pub[-1].state == 'REJECTED'
    good = proposal_from_event(event(), 100.)
    good.xyz_coefficients[0] = math.nan
    n.follow_trajectory_callback(good)
    assert 'INVALID_COEFFICIENTS' in pub[-1].reasons


def test_ack_preserves_zero_active_and_never_implies_replacement():
    msg = proposal_from_event(event(), 100.)
    ctx = ReceiverContext(100., 1, 0, msg.prediction_sequence_id, True, True, True, True)
    ack = ack_message(FollowReceiver().propose(curve_from_message(msg), ctx))
    assert ack.active_plan_id == 0 and ack.pending_plan_id == 0 and not ack.replaced


def test_p4_math_cache_survives_transient_wait_but_clears_clock_and_mission():
    from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner
    from test_follow_mpc_shadow_node import _Adapter, _Pool
    from uav_control.controllers.follow_mpc_seed import MpcConfig
    from types import SimpleNamespace
    cache = SimpleNamespace(hint=(SimpleNamespace(mission_id=7, clock_generation=2),),
                            clear_warm_start=lambda: setattr(cache, 'hint', None))
    adapter = _Adapter()
    adapter.clock_generation = 2
    runner = P4ResearchRunner(MpcConfig(), adapter, _Pool(), lambda x: None, solver=cache)
    runner.mission.update(mission_id=7, state=3)
    runner._invalidate_warm()
    assert cache.hint is not None
    adapter.clock_generation = 3
    runner._invalidate_warm()
    assert cache.hint is None


def test_epoch_encoding_avoids_large_epoch_multiplication_rounding():
    from uav_control.controllers.follow_transport import stamp, seconds
    value = 1791528000.1234567
    msg = stamp(value)
    expected_ns = round((value-int(value))*1e9)
    assert msg.sec == int(value) and msg.nanosec == expected_ns
    assert seconds(msg) == value


def test_finite_extreme_wire_cannot_interrupt_the_unique_tracker():
    n = object.__new__(TrajectoryTrackerNode)
    n.follow_receiver = FollowReceiver()
    n._follow_context = lambda: ReceiverContext(100., 1, 0, 1, True, True, True, True)
    pub = []
    n.follow_ack_pub = type('P', (), {'publish': lambda _, x: pub.append(x)})()
    message = proposal_from_event(event(), 100.)
    for i in range(3, len(message.xyz_coefficients), 3):
        message.xyz_coefficients[i] = 1e200
    n.follow_trajectory_callback(message)
    assert pub[-1].state == 'REJECTED'
    assert 'INVALID_COEFFICIENTS' in pub[-1].reasons
    # A later normal callback still runs, and the original control state remains untouched.
    n.follow_trajectory_callback(proposal_from_event(event(), 100.))
    assert len(pub) == 2 and n.follow_receiver.active is None


def test_missing_progress_diagnostics_are_unknown_not_zero():
    from p4_follow_replay import progress_counts
    missing = progress_counts([{'metrics': {}}])
    assert missing == dict(progress_C=None, zero_jerk=None, warm_hits=None)
    measured = progress_counts([{'metrics': {'progress': dict(
        stage_C_progress=False, zero_jerk=False, warm_used=False)}}])
    assert measured == dict(progress_C=0, zero_jerk=0, warm_hits=0)


def test_receiver_handshake_is_graph_owned_and_rejects_future_heartbeat():
    from types import SimpleNamespace
    from uav_usv_interfaces.msg import FollowReceiverState
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    from uav_control.controllers.follow_transport import stamp
    node = object.__new__(P4FollowPlannerNode)
    node._ros_seconds = lambda: 100.
    node.adapter = SimpleNamespace(clock_generation=7)
    node.runner = SimpleNamespace(current_receiver_epoch=None)
    node.receiver_epoch = None
    node.get_publishers_info_by_topic = lambda _: [SimpleNamespace(
        node_name='trajectory_tracker_node')]
    heartbeat = FollowReceiverState()
    heartbeat.stamp = stamp(100.)
    heartbeat.receiver, heartbeat.receiver_boot_id = 'trajectory_tracker_node', 'boot-a'
    heartbeat.clock_generation, heartbeat.mission_id = 3, 1
    node.epoch_callback(heartbeat)
    assert node.receiver_epoch[0] == ('boot-a', 3, 1, 7)
    heartbeat.stamp = stamp(101.)
    node.epoch_callback(heartbeat)
    assert node.receiver_epoch is None and node.runner.current_receiver_epoch is None


def test_prediction_policy_A_rejects_and_B_fully_checks_new_version():
    from types import SimpleNamespace
    from io import StringIO
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    from uav_control.controllers.follow_mpc_shadow_node import ShadowPrediction
    e = event()
    c = e['request']['context']
    pred = ShadowPrediction(c['mission_id'], c['prediction_sequence_id']+1,
                            c['prediction_source_stamp'], c['observation_stamp'], 100.,
                            c['prediction_valid_until'], c['frame_id'], c['prediction_source'],
                            'fixture', 3., tuple(e['request']['prediction_times']),
                            tuple(map(tuple, e['request']['target_positions'])),
                            tuple(map(tuple, e['request']['target_velocities'])))
    node = object.__new__(P4FollowPlannerNode)
    node.runner = SimpleNamespace(prediction=pred, solver=SimpleNamespace(model=FollowMpcSeed(
        MpcConfig(allow_synthetic_predictions=True))))
    node.log_file = StringIO()
    node._ros_seconds = lambda: 100.
    node.prediction_policy = 'latest_only'
    assert node._prediction_event(e) is None
    node.prediction_policy = 'full'
    revised = node._prediction_event(e)
    assert revised['request']['context']['prediction_sequence_id'] == pred.sequence_id
    assert revised['request']['context']['navigation_stamp'] == c['navigation_stamp']
    assert e['request']['context']['prediction_sequence_id'] != pred.sequence_id
    node.runner.prediction = replace(pred, target_positions=tuple(
        (x-100, y, z) for x, y, z in pred.target_positions))
    assert node._prediction_event(e) is None


def test_wrong_ack_boot_or_publisher_cannot_be_validated():
    from io import StringIO
    import json
    from types import SimpleNamespace
    from uav_usv_interfaces.msg import FollowPlanAck
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    from uav_control.controllers.follow_transport import stamp
    node = object.__new__(P4FollowPlannerNode)
    node._ros_seconds = lambda: 100.
    node.planner_boot_id = 'planner-a'
    node.receiver_epoch = (('receiver-a', 3, 1, 7), 100.)
    node.runner = SimpleNamespace(mission={'mission_id': 1})
    node.expected_plans = {('receiver-a', 'planner-a', 1, 3, 9): 99.99}
    node.ack_file = StringIO()
    node.get_publishers_info_by_topic = lambda _: [SimpleNamespace(node_name='spoof')]
    ack = FollowPlanAck()
    ack.stamp, ack.plan_id, ack.mission_id, ack.clock_generation = stamp(100.), 9, 1, 3
    ack.receiver = 'trajectory_tracker_node'
    ack.receiver_boot_id, ack.planner_boot_id, ack.state = 'receiver-a', 'planner-a', 'REJECTED'
    node.ack_callback(ack)
    assert not json.loads(node.ack_file.getvalue().splitlines()[-1])['identity_valid']
    node.get_publishers_info_by_topic = lambda _: [SimpleNamespace(
        node_name='trajectory_tracker_node')]
    ack.receiver_boot_id = 'old-receiver'
    node.ack_callback(ack)
    assert not json.loads(node.ack_file.getvalue().splitlines()[-1])['identity_valid']


def test_full_revalidation_cannot_publish_after_whole_cycle_deadline(monkeypatch):
    from types import SimpleNamespace
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    from uav_control.controllers.follow_mpc_seed import MpcConfig
    node = object.__new__(P4FollowPlannerNode)
    e = event()
    e['cycle_started_monotonic'] = 10.
    node.adapter = SimpleNamespace(clock_generation=e['request']['context']['clock_generation'])
    node.runner = SimpleNamespace(config=replace(MpcConfig(), solve_budget=.02),
                                  mission={'mission_id': 1}, _follow=lambda: True)
    monkeypatch.setattr('uav_control.controllers.p4_follow_planner_node.time.monotonic',
                        lambda: 10.03)
    assert node._proposal_rejection(e, 100.) == 'CYCLE_DEADLINE_EXCEEDED'
    node._ros_seconds = lambda: 100.
    assert node._publish_proposal(e) == dict(published=False, reason='CYCLE_DEADLINE_EXCEEDED')


@pytest.mark.parametrize('during_publication', (False, True))
def test_final_profile_keeps_worker_on_empty_rejection_output(monkeypatch, during_publication):
    from io import StringIO
    import json
    import time
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    from uav_control.controllers.follow_mpc_shadow_node import FollowMpcShadowNode
    node = object.__new__(P4FollowPlannerNode)
    node.log_file = StringIO()
    node._ros_seconds = lambda: 100.
    worker = dict(cycle_id=4, stages={}, counts={}, total_seconds=.01)
    e = dict(request={'context': {'cycle_id': 4}},
             output=dict(valid=False, solver_status='SHADOW_REJECTED'),
             candidate_output={'metrics': {'cycle_profile': worker}},
             cycle_started_monotonic=time.monotonic(), expires_at_ros_stamp=100.125)
    if during_publication:
        e['output'] = dict(valid=True, metrics={'cycle_profile': worker})

    def parent_publish(_, payload):
        if during_publication:
            payload['output'] = dict(valid=False, solver_status='SHADOW_REJECTED')

    monkeypatch.setattr(FollowMpcShadowNode, '_publish_event', parent_publish)
    node._publish_event(e)
    row = json.loads(node.log_file.getvalue())
    assert not row['published'] and row['worker_profile'] == worker
    assert row['reason'] == 'SHADOW_REJECTED'

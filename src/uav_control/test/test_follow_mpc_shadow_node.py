# Copyright 2026 UAV-USV Project Contributors
# Licensed under the Apache License, Version 2.0

"""Exercise shadow admission, publication deadlines, callbacks, and fixed ROS authority."""

import ast
from concurrent.futures import Future
from dataclasses import asdict
from importlib import import_module
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from uav_control.controllers.follow_mpc_seed import MpcConfig, MpcSeedResult
from uav_control.controllers.mpc_shadow_inputs import PairResult, ShadowPoseInput


def _module():
    return import_module('uav_control.controllers.follow_mpc_shadow_node')


def _time(value):
    seconds = math.floor(value)
    return SimpleNamespace(sec=seconds, nanosec=round((value - seconds) * 1e9))


def _vector(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def _prediction(**changes):
    values = dict(valid=True, source='tracking', frame_id='local_ned', mission_id=7,
                  sequence_id=12, source_stamp=_time(100.0), observation_stamp=_time(99.99),
                  generated_stamp=_time(100.01), valid_until=_time(100.125),
                  invalid_reason='', model='bctra', prediction_horizon=4.0,
                  samples=[SimpleNamespace(relative_time=_time(t), position=_vector(5 + t, 0, 0),
                                           velocity=_vector(1, 0, 0)) for t in (0.0, 2.0, 4.0)])
    values.update(changes)
    return SimpleNamespace(**values)


def _pose(**changes):
    values = dict(stamp=100.0, position=(0.0, 0.0, -5.0), velocity=(0.0, 0.0, 0.0),
                  acceleration=(0.0, 0.0, 0.0), quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
                  native_timestamp_sample=10_000_000, px4_timestamp_sample=99_900_000,
                  navigation_receipt_stamp=100.01, attitude_left_stamp=99.99,
                  attitude_right_stamp=100.01, attitude_left_native_timestamp_sample=9_990_000,
                  attitude_right_native_timestamp_sample=10_010_000,
                  attitude_left_px4_timestamp_sample=99_890_000,
                  attitude_right_px4_timestamp_sample=99_910_000,
                  attitude_left_timesync_stamp=99.98, attitude_right_timesync_stamp=99.98,
                  attitude_quat_reset_counter=0, clock_generation=2, observation_stamp=99.99)
    values.update(changes)
    return ShadowPoseInput(**values)


def _mission(state=3, completed=False, mission_id=7):
    return SimpleNamespace(state=state, completed=completed, mission_id=mission_id,
                           state_name='FOLLOW', stamp=_time(100.0))


def _request():
    module = _module()
    prediction = module.prediction_from_message(_prediction())
    return module.make_shadow_request(_pose(), prediction, 7, 1, 100.02, MpcConfig())


def test_request_preserves_navigation_image_and_prediction_epochs_without_receipt_substitution():
    """Catch replacing physical source timestamps with callback or solve receipt time."""
    request = _request()
    assert request.context.execution_start_stamp == 100.0
    assert request.context.navigation_stamp == request.context.attitude_stamp == 100.0
    assert request.context.prediction_source_stamp == 100.0
    assert request.context.observation_stamp == 99.99
    assert request.now_stamp == 100.02
    assert request.context.clock_generation == 2
    assert request.state == (0, 0, -5, 0, 0, 0, 0, 0, 0, 0)


def test_request_uses_complete_measured_quaternion_to_initialize_yaw():
    """Catch using absent UavState roll/pitch or assuming a north-facing aircraft."""
    module = _module()
    pose = _pose(quaternion_wxyz=(math.sqrt(.5), 0, 0, math.sqrt(.5)))
    request = module.make_shadow_request(pose, module.prediction_from_message(_prediction()),
                                         7, 1, 100.02, MpcConfig())
    assert request.state[-1] == pytest.approx(math.pi / 2)
    assert request.actual_rotation[0] == pytest.approx((0, -1, 0), abs=1e-12)


@pytest.mark.parametrize('changes', [
    {'valid': False}, {'source': 'truth'}, {'source': 'synthetic'}, {'frame_id': 'map'},
    {'observation_stamp': _time(0)}, {'observation_stamp': _time(100.1)},
    {'samples': []}, {'prediction_horizon': math.inf},
])
def test_prediction_rejects_invalid_source_frame_epoch_and_missing_samples(changes):
    """Catch accepting forbidden target sources or a malformed immutable prediction."""
    with pytest.raises(ValueError):
        _module().prediction_from_message(_prediction(**changes))


@pytest.mark.parametrize('changes,reason', [
    ({'now': 100.126}, 'STALE_INPUT'),
    ({'now': 100.02, 'mission_id': 8}, 'MISSION_CHANGED'),
    ({'now': 100.02, 'generation': 3}, 'CLOCK_GENERATION_CHANGED'),
    ({'now': 100.02, 'elapsed': .501}, 'CYCLE_DEADLINE_EXCEEDED'),
    ({'now': 100.02, 'follow': False}, 'MISSION_NOT_FOLLOW'),
])
def test_completion_rechecks_live_freshness_mission_generation_and_whole_cycle(changes, reason):
    """Catch treating a once-valid solve as publishable after its input or budget expires."""
    values = dict(now=100.02, mission_id=7, generation=2, elapsed=.04, follow=True)
    values.update(changes)
    actual = _module().completion_rejection(_request(), MpcConfig(), **values)
    assert actual == reason


def test_completion_gate_accepts_fresh_same_mission_and_generation():
    """Catch discarding an independently valid result within all live publication limits."""
    assert not _module().completion_rejection(
        _request(), MpcConfig(), 100.03, 7, 2, .04, True)


@pytest.mark.parametrize('prediction,now,reason', [
    (_prediction(observation_stamp=_time(99.8)), 100.02, 'STALE_INPUT'),
    (_prediction(source_stamp=_time(100.03), generated_stamp=_time(100.03)),
     100.02, 'PREDICTION_FUTURE'),
    (_prediction(prediction_horizon=1.0, samples=[
        SimpleNamespace(relative_time=_time(t), position=_vector(5, 0, 0),
                        velocity=_vector(0, 0, 0)) for t in (0.0, 1.0)
    ]), 100.02, 'PREDICTION_HORIZON'),
])
def test_prepare_rejects_stale_future_or_short_prediction_coverage(prediction, now, reason):
    """Catch extrapolating target coverage or relying on the receipt epoch for freshness."""
    module = _module()
    with pytest.raises(ValueError, match=reason):
        module.make_shadow_request(_pose(), module.prediction_from_message(prediction),
                                   7, 1, now, MpcConfig())


class _Pool:
    def __init__(self):
        self.future = Future()
        self.requests = []

    def submit(self, function, request):
        self.requests.append(request)
        return self.future


class _Adapter:
    def __init__(self):
        self.clock_generation = 2
        self.status = 'PAIRED'

    def pair_latest(self, now, monotonic_now, observation_stamp):
        return PairResult(self.status, _pose() if self.status == 'PAIRED' else None)


def _runner():
    module = _module()
    pool, events, adapter = _Pool(), [], _Adapter()
    runner = module.ShadowResearchRunner(MpcConfig(), adapter, pool, events.append)
    runner.update_mission(_mission(), 100.01, 1.0)
    runner.update_prediction(_prediction(), 100.01, 1.0)
    return runner, pool, events, adapter


def test_runner_has_one_inflight_job_without_accumulating_new_prediction_jobs():
    """Catch enqueueing every subscription or timer tick behind a slow optimization."""
    runner, pool, events, _ = _runner()
    runner.tick(100.02, 1.02)
    runner.update_prediction(_prediction(sequence_id=13), 100.03, 1.03)
    runner.tick(100.04, 1.04)
    assert len(pool.requests) == 1
    assert not any(event['output']['valid'] for event in events)


def test_late_solver_result_is_logged_but_clears_the_published_trajectory():
    """Catch publishing stale optimizer output as valid after it finishes asynchronously."""
    runner, pool, events, _ = _runner()
    runner.tick(100.02, 1.02)
    result = MpcSeedResult(pool.requests[0].context, valid=True, solver_status='SUCCESS',
                           positions=((0, 0, -5),), relative_times=(0.0,))
    pool.future.set_result(result)
    runner.tick(100.2, 1.2)
    event = events[-1]
    assert not event['output']['valid'] and event['output']['positions'] == []
    assert event['output']['reason'] == 'STALE_INPUT'
    assert event['candidate_output'] == asdict(result)
    assert event['request'] == asdict(pool.requests[0])
    assert event['input_provenance']['paired_navigation'] == asdict(_pose())
    assert event['accepted_by_tracker'] is False


def test_invalid_prediction_clears_valid_output_and_blocks_inflight_completion():
    """Catch retaining a valid research trajectory after an explicit prediction rejection."""
    runner, pool, events, _ = _runner()
    runner.tick(100.02, 1.02)
    runner.update_prediction(_prediction(valid=False, invalid_reason='vision stale'), 100.03, 1.03)
    assert not events[-1]['output']['valid'] and events[-1]['output']['positions'] == []
    pool.future.set_result(MpcSeedResult(pool.requests[0].context, valid=True))
    runner.tick(100.04, 1.04)
    assert not events[-1]['output']['valid']


def test_clock_generation_change_rejects_a_finished_candidate():
    """Catch accepting attitude or pose from a previous simulation clock generation."""
    runner, pool, events, adapter = _runner()
    runner.tick(100.02, 1.02)
    adapter.clock_generation = 3
    pool.future.set_result(MpcSeedResult(pool.requests[0].context, valid=True))
    runner.tick(100.04, 1.04)
    assert events[-1]['output']['reason'] == 'CLOCK_GENERATION_CHANGED'


def test_node_source_has_only_fixed_research_string_publishers_and_no_command_interfaces():
    """Catch introducing a PX4 setpoint publisher, mission publisher, or truth subscriber."""
    module = _module()
    tree = ast.parse(Path(module.__file__).read_text())
    publishers = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Attribute)
                  and node.func.attr == 'create_publisher']
    assert {(node.args[0].id, node.args[1].value) for node in publishers} == {
        ('String', '/research/mpc_seed/diagnostic'),
        ('String', '/research/mpc_seed/trajectory'),
    }
    assert len(publishers) == 2
    forbidden = {'TrajectorySetpoint', 'OffboardControlMode', 'VehicleCommand'}
    assert not any(isinstance(node, ast.Name) and node.id in forbidden for node in ast.walk(tree))
    assert not any(isinstance(node, ast.Constant) and isinstance(node.value, str)
                   and ('/fmu/in/' in node.value or node.value == '/target/state')
                   for node in ast.walk(tree))


def test_callbacks_forward_original_messages_and_separate_receipt_clocks(monkeypatch):
    """Catch replacing acquisition stamps or hiding receipt time in adapter callbacks."""
    module = _module()
    node = object.__new__(module.FollowMpcShadowNode)
    calls = []
    node.adapter = SimpleNamespace(
        add_navigation=lambda *args: calls.append(('nav', args)) or True,
        add_attitude=lambda *args: calls.append(('att', args)) or True,
        add_timesync=lambda msg: calls.append(('sync', (msg,))) or True,
        add_clock_anchor=lambda *args: calls.append(('clock', args)) or True)
    node._ros_seconds = lambda: 100.02
    monkeypatch.setattr(module.time, 'monotonic', lambda: 5.0)
    nav, att, sync = object(), object(), object()
    node.navigation_callback(nav)
    node.attitude_callback(att)
    node.timesync_callback(sync)
    node.gazebo_clock_callback(SimpleNamespace(
        sim=SimpleNamespace(sec=10, nsec=0), system=SimpleNamespace(sec=100, nsec=0)))
    assert calls == [('nav', (nav, 100.02, 5.0)), ('att', (att, 100.02, 5.0)),
                     ('sync', (sync,)), ('clock', (10.0, 100.0, 100.02, 5.0))]


def test_shadow_yaml_and_standalone_launch_keep_synthetic_gate_closed():
    """Catch default synthetic admission or adding the shadow node to a flight launch graph."""
    import yaml
    module = _module()
    workspace = Path(module.__file__).resolve().parents[4]
    path = workspace / 'src/uav_usv_bringup/config/mpc_seed_shadow.yaml'
    config = yaml.safe_load(path.read_text())['follow_mpc_shadow_node']['ros__parameters']
    assert config['allow_synthetic_predictions'] is False
    assert config['maximum_input_age'] == .125
    assert config['shadow_rate_hz'] == 1.0
    assert config['solve_budget'] == .5
    launch_path = workspace / 'src/uav_usv_bringup/launch/mpc_seed_shadow.launch.py'
    tree = ast.parse(launch_path.read_text())
    nodes = [item for item in ast.walk(tree) if isinstance(item, ast.Call)
             and isinstance(item.func, ast.Name) and item.func.id == 'Node']
    assert len(nodes) == 1
    assert next(kw.value.value for kw in nodes[0].keywords
                if kw.arg == 'executable') == 'follow_mpc_shadow_node'


def test_log_separates_core_feasibility_from_final_admission_and_saves_replay_config():
    """Catch labeling a late geometric solution as deliverable or omitting replay settings."""
    runner, pool, events, _ = _runner()
    runner.tick(100.02, 1.02)
    result = MpcSeedResult(pool.requests[0].context, valid=True, solver_status='SUCCESS')
    pool.future.set_result(result)
    runner.tick(100.2, 1.2)
    event = events[-1]
    assert event['core_result']['valid'] is True
    assert event['admission_status'] == 'STALE_INPUT'
    assert event['input_provenance']['model_config']['mpc'] == asdict(MpcConfig())
    assert event['input_provenance']['model_config']['attitude']['gravity'] == 9.80665
    from uav_control.controllers.follow_mpc_seed import MpcRequest, PlanningContext
    values = dict(event['request'])
    replay = MpcRequest(**{**values, 'context': PlanningContext(**values['context'])})
    assert replay == pool.requests[0]


def test_publication_rechecks_freshness_after_serialization_and_logs_total_time(monkeypatch):
    """Catch serializing a fresh result and publishing it after its image epoch expires."""
    module = _module()
    runner, pool, _, adapter = _runner()
    runner.tick(100.02, 1.02)
    result = MpcSeedResult(pool.requests[0].context, valid=True, solver_status='SUCCESS')
    event = runner.event(asdict(result), 100.04, 1.04, asdict(result), 1.02)
    node = object.__new__(module.FollowMpcShadowNode)
    messages = []
    node.runner, node.adapter = runner, adapter
    node.trajectory_pub = SimpleNamespace(publish=lambda message: messages.append(message.data))
    node.diagnostic_pub = SimpleNamespace(publish=lambda message: messages.append(message.data))
    node.log_file = io.StringIO()
    node._ros_seconds = lambda: 100.14
    monkeypatch.setattr(module.time, 'monotonic', lambda: 1.17)
    node._publish_event(event)
    assert all(not json.loads(message)['output']['valid'] for message in messages)
    record = json.loads(node.log_file.getvalue())
    assert record['admission_status'] == 'STALE_INPUT'
    assert record['core_result']['valid'] is True
    assert record['whole_cycle_time'] == pytest.approx(.15)


def test_fresh_completed_result_is_admitted_only_as_a_research_candidate():
    """Catch discarding usable fresh results or claiming tracker acceptance."""
    runner, pool, events, _ = _runner()
    runner.tick(100.02, 1.02)
    result = MpcSeedResult(pool.requests[0].context, valid=True, solver_status='SUCCESS')
    pool.future.set_result(result)
    runner.tick(100.04, 1.04)
    assert events[-1]['output']['valid'] is True
    assert events[-1]['admission_status'] == 'ADMITTED_RESEARCH'
    assert events[-1]['accepted_by_tracker'] is False


def test_core_rejection_is_distinct_from_late_input_rejection():
    """Catch converting geometric infeasibility into a timing failure or valid output."""
    runner, pool, events, _ = _runner()
    runner.tick(100.02, 1.02)
    result = MpcSeedResult(pool.requests[0].context, solver_status='VISIBILITY_INFEASIBLE',
                           reason='whole target outside safe field of view')
    pool.future.set_result(result)
    runner.tick(100.04, 1.04)
    assert events[-1]['core_result']['valid'] is False
    assert events[-1]['output']['solver_status'] == 'VISIBILITY_INFEASIBLE'
    assert events[-1]['admission_status'] == result.reason


def test_publish_time_is_in_whole_cycle_budget_and_crossing_it_clears_output(monkeypatch):
    """Catch accounting only solve/serialization while a slow publisher exceeds the budget."""
    module = _module()
    runner, pool, _, adapter = _runner()
    runner.tick(100.02, 1.02)
    result = MpcSeedResult(pool.requests[0].context, valid=True, solver_status='SUCCESS')
    event = runner.event(asdict(result), 100.04, 1.04, asdict(result), 1.02)
    node = object.__new__(module.FollowMpcShadowNode)
    messages = []
    node.runner, node.adapter = runner, adapter
    wall_time = [1.04]

    def publish(message):
        messages.append(message.data)
        if len(messages) == 2:
            wall_time[0] = 1.6

    node.trajectory_pub = SimpleNamespace(publish=publish)
    node.diagnostic_pub = SimpleNamespace(publish=publish)
    node.log_file = io.StringIO()
    node._ros_seconds = lambda: 100.04
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall_time[0])
    node._publish_event(event)
    assert json.loads(messages[-1])['output']['valid'] is False
    record = json.loads(node.log_file.getvalue())
    assert record['admission_status'] == 'CYCLE_DEADLINE_EXCEEDED'
    assert record['whole_cycle_time'] >= .58
    assert [attempt['valid'] for attempt in record['publication_attempts']] == [
        True, True, False, False]
    assert record['published_valid'] is True
    assert record['final_marker_valid'] is False
    assert all(attempt['stamp'] == 100.04 and attempt['elapsed'] >= .02 - 1e-9
               for attempt in record['publication_attempts'])


def test_follow_exit_and_reentry_does_not_revive_an_inflight_candidate():
    """Catch reviving work prepared before the same mission temporarily left FOLLOW."""
    runner, pool, events, _ = _runner()
    runner.tick(100.02, 1.02)
    runner.update_mission(_mission(state=14), 100.025, 1.025)
    runner.update_mission(_mission(state=3), 100.03, 1.03)
    pool.future.set_result(MpcSeedResult(pool.requests[0].context, valid=True))
    runner.tick(100.04, 1.04)
    assert events[-1]['output']['valid'] is False
    assert events[-1]['admission_status'] == 'MISSION_STATE_CHANGED'


def test_prediction_subscription_matches_the_current_best_effort_offering():
    """Catch ROS QoS incompatibility that silently prevents receiving target predictions."""
    module = _module()
    tree = ast.parse(Path(module.__file__).read_text())
    prediction = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
                      and isinstance(node.func, ast.Attribute)
                      and node.func.attr == 'create_subscription'
                      and isinstance(node.args[0], ast.Name)
                      and node.args[0].id == 'TargetPrediction')
    assert isinstance(prediction.args[3], ast.Name) and prediction.args[3].id == 'sensor_qos'
    qos = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
               and any(isinstance(target, ast.Name) and target.id == 'sensor_qos'
                       for target in node.targets))
    reliability = next(keyword.value for keyword in qos.keywords
                       if keyword.arg == 'reliability')
    assert isinstance(reliability, ast.Attribute) and reliability.attr == 'BEST_EFFORT'


def test_out_of_order_mission_message_cannot_revive_an_old_follow_state():
    """Catch a delayed older mission message replacing a later inactive mission state."""
    runner, _, events, _ = _runner()
    newer = _mission(state=14)
    newer.stamp = _time(100.02)
    runner.update_mission(newer, 100.025, 1.025)
    runner.update_mission(_mission(state=3), 100.03, 1.03)
    assert not runner._follow()
    assert runner.mission['state'] == 14 and runner.mission['stamp'] == 100.02
    assert events[-1]['admission_status'] == 'MISSION_OUT_OF_ORDER'


def test_prediction_generated_epoch_cannot_precede_its_source_epoch():
    """Catch accepting a logically reversed generated/source/image acquisition sequence."""
    with pytest.raises(ValueError, match='INVALID_PREDICTION'):
        _module().prediction_from_message(_prediction(generated_stamp=_time(99.999)))

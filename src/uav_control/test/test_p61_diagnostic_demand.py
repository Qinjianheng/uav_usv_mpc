"""Shadow-only model payloads must not occupy the BCTRA consumer's critical callbacks."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from test_p6_imm_node import real_sequence_780
from test_progress_follow import model
from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
from uav_control.tracking.imm_target_predictor_node import prediction_to_message
from uav_control.guidance import p6_follow


def test_bctra_keeps_bounded_raw_evidence_without_decoding_until_imm_needs_it(monkeypatch):
    configured = model()
    configured.p6_policy = p6_follow.P6Policy()
    runtime = p6_follow.P6Runtime(configured)
    engine, _, _, result = real_sequence_780()
    encoded = p6_follow.json.dumps(engine.diagnostics(result))
    calls = []
    decode = p6_follow.json.loads

    def recorded_decode(value):
        calls.append(value)
        return decode(value)

    monkeypatch.setattr(p6_follow.json, 'loads', recorded_decode)
    runtime.diagnostic(SimpleNamespace(data=encoded))
    parsed = prediction_from_message(prediction_to_message(result, .001))
    bctra = replace(parsed, model='BCTRA_BOUNDED_Z')
    assert runtime.prediction(bctra) is bctra
    assert not calls
    attached = runtime.prediction(parsed)
    assert len(calls) == 1
    assert np.asarray(attached.model_positions).shape == (3, 41, 3)
    assert attached.uncertainty_identity == (1, 780, result.source_stamp,
                                             result.observation_stamp)


def test_deferred_bad_or_mismatched_diagnostics_cannot_grant_a_model_identity():
    configured = model()
    configured.p6_policy = p6_follow.P6Policy()
    runtime = p6_follow.P6Runtime(configured)
    _, _, _, result = real_sequence_780()
    parsed = prediction_from_message(prediction_to_message(result, .001))
    for _ in range(100):
        runtime.diagnostic(SimpleNamespace(data='not JSON'))
    assert len(runtime.pending_diagnostics) <= 16
    runtime.diagnostic(SimpleNamespace(data='{"mission_id":1,"sequence_id":780}'))
    assert runtime.prediction(parsed) is parsed
    assert not parsed.uncertainty_identity
    assert not runtime.pending_diagnostics


def test_nominal_execution_keeps_validated_wire_mean_without_decoding_unused_models(monkeypatch):
    configured = model()
    configured.p6_policy = p6_follow.P6Policy()
    runtime = p6_follow.P6Runtime(configured)
    engine, _, _, result = real_sequence_780()
    parsed = prediction_from_message(prediction_to_message(result, .001))
    for _ in range(100):
        runtime.diagnostic(SimpleNamespace(data=p6_follow.json.dumps(engine.diagnostics(result))))

    def forbidden_decode(_):
        raise AssertionError('Nominal receiver does not consume IMM model diagnostics')

    monkeypatch.setattr(p6_follow.json, 'loads', forbidden_decode)
    assert runtime.execution_prediction(parsed) is parsed
    assert len(runtime.pending_diagnostics) == 16
    assert not parsed.uncertainty_identity


def test_robust_execution_still_binds_full_model_identity_and_rejects_mismatch():
    configured = model()
    configured.p6_policy = p6_follow.P6Policy(visibility_mode='robust')
    runtime = p6_follow.P6Runtime(configured)
    engine, _, _, result = real_sequence_780()
    parsed = prediction_from_message(prediction_to_message(result, .001))
    runtime.diagnostic(SimpleNamespace(data=p6_follow.json.dumps(engine.diagnostics(result))))
    attached = runtime.execution_prediction(parsed)
    assert attached.uncertainty_identity == (1, 780, result.source_stamp, result.observation_stamp)
    assert np.asarray(attached.model_positions).shape == (3, 41, 3)
    assert not runtime.execution_prediction(replace(parsed, sequence_id=781)).uncertainty_identity


def test_nominal_planner_callback_and_request_keep_mean_without_unused_json(monkeypatch):
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    configured = model()
    configured.p6_policy = p6_follow.P6Policy()
    node = object.__new__(P4FollowPlannerNode)
    node.p6_runtime = p6_follow.P6Runtime(configured)
    engine, _, _, result = real_sequence_780()
    parsed = prediction_from_message(prediction_to_message(result, .001))
    node.runner = SimpleNamespace(prediction=parsed)
    payload = SimpleNamespace(data=p6_follow.json.dumps(engine.diagnostics(result)))

    def forbidden_decode(_):
        raise AssertionError('Nominal planner does not consume conditional model JSON')

    monkeypatch.setattr(p6_follow.json, 'loads', forbidden_decode)
    for _ in range(100):
        node.p6_diagnostic_callback(payload)
    assert node.runner.prediction is parsed
    assert node.p6_runtime.planning_prediction(parsed) is parsed
    assert len(node.p6_runtime.pending_diagnostics) == 16


@pytest.mark.parametrize('visibility,adaptive', [
    ('robust', False), ('robust', True), ('nominal', True),
])
def test_planner_model_consumers_keep_full_strict_identity(visibility, adaptive):
    from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
    configured = model()
    configured.p6_policy = p6_follow.P6Policy(
        visibility_mode=visibility, adaptive_viewpoint=adaptive)
    node = object.__new__(P4FollowPlannerNode)
    node.p6_runtime = p6_follow.P6Runtime(configured)
    engine, _, _, result = real_sequence_780()
    parsed = prediction_from_message(prediction_to_message(result, .001))
    node.runner = SimpleNamespace(prediction=parsed)
    node.p6_diagnostic_callback(SimpleNamespace(
        data=p6_follow.json.dumps(engine.diagnostics(result))))
    assert node.runner.prediction.uncertainty_identity == (
        1, 780, result.source_stamp, result.observation_stamp)
    assert np.asarray(node.runner.prediction.model_positions).shape == (3, 41, 3)
    assert not node.p6_runtime.planning_prediction(
        replace(parsed, sequence_id=781)).uncertainty_identity


@pytest.mark.parametrize('visibility,expected_decodes', [('nominal', 0), ('robust', 1)])
def test_actual_proposal_callback_demands_models_only_for_robust_policy(
        monkeypatch, visibility, expected_decodes):
    from test_follow_sitl_execution import ctx
    from test_p4_receiver_node import event
    from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
    from uav_control.controllers.follow_transport import proposal_from_event
    from uav_control.guidance.follow_minco_execution import NominalFollowReceiver
    n = object.__new__(TrajectoryTrackerNode)
    configured = model()
    configured.p6_policy = p6_follow.P6Policy(visibility_mode=visibility)
    n.p6_runtime = p6_follow.P6Runtime(configured)
    engine, _, _, result = real_sequence_780()
    n.latest_prediction = prediction_to_message(result, .001)
    n.p6_runtime.diagnostic(SimpleNamespace(data=p6_follow.json.dumps(engine.diagnostics(result))))
    n.follow_minco_requested, n.follow_stable_ticks = True, 10
    n.follow_receiver = NominalFollowReceiver()
    n.follow_model = configured
    n._follow_context = lambda: ctx()
    n._ros_seconds = lambda: ctx().now
    n._update_follow_admission = lambda _: ''
    n.get_publishers_info_by_topic = lambda _: [SimpleNamespace(
        node_name='p4_follow_planner_node')]
    published = []
    n.follow_ack_pub = n.follow_debug_pub = SimpleNamespace(publish=published.append)
    message = proposal_from_event(event(), 100.)
    calls = []
    decode = p6_follow.json.loads
    monkeypatch.setattr(p6_follow.json, 'loads',
                        lambda v: (calls.append(v), decode(v))[1])
    monkeypatch.setattr('uav_control.guidance.follow_minco_execution.nominal_attitude',
                        lambda *_: False)
    n.follow_trajectory_callback(message)
    assert len(calls) == expected_decodes
    assert next(m for m in published if hasattr(m, 'state')).state == 'REJECTED'

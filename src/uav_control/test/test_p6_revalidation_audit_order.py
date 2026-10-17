"""The complete revalidation audit is written only after the wire attempt."""
from dataclasses import dataclass
from io import StringIO
import json
from types import SimpleNamespace

from uav_control.controllers.follow_mpc_shadow_node import FollowMpcShadowNode
from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode


@dataclass(frozen=True)
class Prediction:
    sequence_id: int = 2
    observation_stamp: float = 100.0


def node_with_log(events):
    node = object.__new__(P4FollowPlannerNode)

    class TrackedFile(StringIO):
        def write(self, value):
            events.append(('log', json.loads(value)['event']))
            return super().write(value)

    node.log_file = TrackedFile()
    node._ros_seconds = lambda: 100.05
    return node


def test_revalidation_rejection_keeps_full_audit_without_early_write():
    events = []
    node = node_with_log(events)
    node.runner = SimpleNamespace(prediction=Prediction())
    node.prediction_policy = 'latest_only'
    event = dict(request=dict(context=dict(
        cycle_id=9, prediction_sequence_id=1)), output=dict(valid=True))
    assert node._prediction_event(event) is None
    assert node.log_file.getvalue() == ''
    node._flush_revalidation_audit()
    rows = [json.loads(line) for line in node.log_file.getvalue().splitlines()]
    assert len(rows) == 1
    assert rows[0]['event'] == 'prediction_revalidation'
    assert rows[0]['cycle_id'] == 9
    assert rows[0]['report']['valid'] is False
    assert rows[0]['new_prediction']['sequence_id'] == 2
    assert node._pending_revalidation_audit is None
    node._flush_revalidation_audit()
    assert len(node.log_file.getvalue().splitlines()) == 1


def test_critical_publish_precedes_audit_and_research_marker(monkeypatch):
    events = []
    node = node_with_log(events)
    node.follow_execution_enabled = True
    node.follow_admission_required = True
    node.runner = SimpleNamespace(last_completed_valid=False)

    def wire(_):
        events.append(('wire', 'attempted'))
        node._pending_revalidation_audit = (
            9, 100.05, dict(valid=True, reason=''), Prediction())
        return dict(published=True, reason='')

    node._publish_proposal = wire
    monkeypatch.setattr(
        FollowMpcShadowNode, '_publish_event',
        lambda self, event: events.append(('research', 'marker')))
    monkeypatch.setattr(
        'uav_control.controllers.p4_follow_planner_node.time.monotonic',
        lambda: 10.1)
    event = dict(
        event='completion',
        request=dict(context=dict(cycle_id=9)),
        output=dict(valid=True, metrics={}),
        candidate_output=dict(metrics={}),
        cycle_started_monotonic=10.0,
        expires_at_ros_stamp=100.125,
    )
    node._publish_event(event)
    assert events[:3] == [
        ('wire', 'attempted'),
        ('log', 'prediction_revalidation'),
        ('research', 'marker'),
    ]
    assert node.runner.last_completed_valid is True


def test_failed_wire_keeps_revalidation_audit(monkeypatch):
    events = []
    node = node_with_log(events)
    node.follow_execution_enabled = True
    node.follow_admission_required = True
    node.runner = SimpleNamespace(last_completed_valid=True)

    def reject(_):
        node._pending_revalidation_audit = (
            12, 100.05, dict(valid=False, reason='PREDICTION_CHANGED'),
            Prediction())
        return dict(published=False, reason='PREDICTION_CHANGED')

    node._publish_proposal = reject
    monkeypatch.setattr(FollowMpcShadowNode, '_publish_event',
                        lambda self, event: None)
    event = dict(event='reject', request=dict(context=dict(cycle_id=12)),
                 output=dict(valid=True, metrics={}), candidate_output=None,
                 cycle_started_monotonic=None)
    node._publish_event(event)
    audits = [json.loads(x) for x in node.log_file.getvalue().splitlines()]
    assert len(audits) == 1
    assert audits[0]['report']['valid'] is False
    assert node.runner.last_completed_valid is True

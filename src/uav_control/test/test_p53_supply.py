"""Measured cycle79 audit cost must not precede a lawful critical wire publication."""
from dataclasses import asdict
from io import StringIO
import json
from types import SimpleNamespace

from test_follow_research_algorithms import request
from uav_control.controllers.follow_mpc_seed import MpcConfig
from uav_control.controllers.follow_mpc_shadow_node import (
    FollowMpcShadowNode, completion_rejection)
from uav_control.controllers.p4_follow_planner_node import P4FollowPlannerNode
from uav_control.guidance.follow_problem import future_request


def run(monkeypatch, now=100.102, valid=True):
    req = future_request(request(2.), .15)
    node = object.__new__(P4FollowPlannerNode)
    clock, wire, calls = [now], [], []
    node.log_file = StringIO()
    node.follow_execution_enabled = node.follow_admission_required = True
    node._ros_seconds = lambda: clock[0]
    monkeypatch.setattr('uav_control.controllers.p4_follow_planner_node.time.monotonic',
                        lambda: 10.+clock[0]-now)
    event = dict(event='completion', request=asdict(req),
                 output=dict(valid=valid, solver_status='VALID'),
                 candidate_output=dict(metrics={}), cycle_started_monotonic=10.,
                 expires_at_ros_stamp=100.115)

    def research(_, payload):
        calls.append('research')
        clock[0] += .013253  # Recorded cycle79 serialization cost, explicitly injected.
        payload['output'] = dict(valid=False, solver_status='STALE_INPUT')

    def critical(payload):
        calls.append('critical')
        reason = completion_rejection(req, MpcConfig(), clock[0], 1, 0,
                                      clock[0]-now, True)
        if not reason:
            wire.append(clock[0])
        return dict(published=not reason, reason=reason,
                    wire_raw_margin=100.115-clock[0])

    monkeypatch.setattr(FollowMpcShadowNode, '_publish_event', research)
    node._publish_proposal = critical
    node.runner = SimpleNamespace(last_completed_valid=True)
    node._publish_event(event)
    row = json.loads(node.log_file.getvalue())
    row['test_runner_success'] = node.runner.last_completed_valid
    return wire, calls, row


def test_measured_audit_delay_cannot_expire_a_ready_wire_candidate(monkeypatch):
    wire, calls, row = run(monkeypatch)
    assert wire and wire[0] < 100.115
    assert calls == ['critical', 'research']
    assert row['published'] and row['reason'] == ''
    assert row['research_marker_reason'] == 'STALE_INPUT'


def test_already_expired_input_still_never_publishes_before_audit(monkeypatch):
    wire, _, row = run(monkeypatch, now=100.116)
    assert not wire and not row['published'] and row['reason'] == 'STALE_INPUT'


def test_infeasible_solver_output_is_never_sent_as_a_wire_candidate(monkeypatch):
    wire, calls, _ = run(monkeypatch, valid=False)
    assert not wire and calls == ['research']


def test_valid_solver_but_failed_wire_keeps_existing_single_early_retry_eligible(monkeypatch):
    _, _, row = run(monkeypatch, now=100.116)
    assert not row['test_runner_success']


def test_late_research_marker_does_not_request_retry_after_lawful_wire(monkeypatch):
    _, _, row = run(monkeypatch)
    assert row['test_runner_success']

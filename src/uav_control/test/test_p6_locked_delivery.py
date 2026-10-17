"""Locked delivery route cannot silently upgrade or fall back when a worker starts early."""
from dataclasses import asdict, replace

import pytest

from test_follow_research_algorithms import request
from test_p47_follow_guided_solver import solver
from uav_control.guidance.follow_delivery_budget import make_plan
from uav_control.guidance.follow_minco_execution import delivery_request, request_from_payload
from uav_control.guidance.follow_problem import future_request


def locked(path='follow'):
    req = future_request(request(2.), .15)
    c = req.context
    p = make_plan((c.mission_id, c.clock_generation, c.cycle_id), req.now_stamp,
                  c.observation_stamp+.125, c.execution_start_stamp, 0, 0.,
                  costs={path: .001}, queue=0., publication=.010, receiver=.020)
    return delivery_request(req, p)


def test_locked_request_roundtrip_retains_deadline_and_typed_plan():
    req = locked()
    restored = request_from_payload(asdict(req))
    assert restored == req
    assert restored.delivery_plan.path == 'follow'
    assert restored.context == req.context


def test_locked_follow_failure_never_calls_fallback(monkeypatch):
    req = locked()
    s = solver(req, online_budget_guard=True, refinement=False)
    s.fallback_estimate = .001  # Even a cheap fallback cannot change the selected route.

    def failed(*_args, **_kwargs):
        raise ValueError('TEST_INITIALIZATION_FAILURE')
    monkeypatch.setattr('uav_control.controllers.follow_guided_feasible.rollout_follow_fast',
                        failed)
    s.fallback._solve = lambda *_: pytest.fail('Unbudgeted fallback was executed')
    r = s.solve(req)
    assert not r.valid
    assert not r.metrics['follow_guided']['fallback_used']
    assert r.metrics['follow_guided']['delivery_plan']['path'] == 'follow'


def test_locked_fallback_cannot_attempt_follow_with_more_internal_budget(monkeypatch):
    req = locked('fallback')
    s = solver(req, online_budget_guard=True, refinement=False)
    s.follow_estimate = s.fallback_estimate = .001
    monkeypatch.setattr('uav_control.controllers.follow_guided_feasible.rollout_follow_fast',
                        lambda *_args, **_kw: pytest.fail('Unbudgeted P47 was attempted'))
    r = s.solve(req)
    assert r.metrics['follow_guided']['fallback_used']
    assert r.metrics['follow_guided']['initial_status'] == 'LOCKED_P44_PATH'


def test_delivery_plan_wrong_context_or_elapsed_deadline_rejects_before_solve():
    req = locked()
    s = solver(req, online_budget_guard=True)
    wrong = replace(req, delivery_plan=replace(req.delivery_plan, identity=(9, 0, 1)))
    assert not s.solve(wrong).valid
    s.wall_clock = lambda: req.delivery_plan.worker_until+.001
    assert not s.solve(req).valid


def test_runner_prepares_shared_budget_before_submit():
    from test_p6_successor_delivery import delivery_runner
    from uav_control.guidance.follow_delivery_budget import DeliveryCosts
    from types import SimpleNamespace
    r, _, _ = delivery_runner()
    r.solver.delivery_costs = DeliveryCosts()
    r.solver.wall_clock = lambda: 100.245
    r.solver.fast = SimpleNamespace(publication_reserve=.020)
    req, _ = r._prepare(100.24, 10.24)
    assert req.delivery_plan.path == 'follow'
    assert req.delivery_plan.worker_until == pytest.approx(100.295)
    assert req.delivery_plan.accept_until == pytest.approx(100.325)
    assert r.delivery_budget['receiver_reserve_seconds'] == .020


def test_budget_wait_records_current_image_without_consuming_dispatch():
    from test_p6_successor_delivery import delivery_runner
    from uav_control.guidance.follow_delivery_budget import DeliveryCosts
    r, pool, events = delivery_runner()
    r.solver.delivery_costs = DeliveryCosts()
    r.solver.wall_clock = lambda: 100.30
    r.tick(100.30, 10.30)
    assert not pool.requests
    assert events[-1]['delivery_budget']['available_worker_seconds'] < 0.
    assert events[-1]['request']['context']['observation_stamp'] == 100.2
    assert events[-1]['budget_rejection_before_submit'] is True

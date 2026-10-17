"""Do not spend the one parent retry on input the existing cost guard cannot use."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from test_follow_research_algorithms import request
from uav_control.controllers.follow_mpc_shadow_node import ShadowResearchRunner
from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner
from uav_control.guidance.follow_problem import future_request


def runner(monkeypatch):
    req = future_request(request(2.), .15)
    r = object.__new__(P4ResearchRunner)
    r.cycle_id, r.pending = 0, None
    r.retry_waiting_parent = 83
    r.execution_status = dict(active=83)
    r.solver = SimpleNamespace(online_budget_guard=True, follow_estimate=.0221203,
                               fallback_estimate=.0190372, fallback_enabled=True,
                               fast=SimpleNamespace(publication_reserve=.02))
    r.test_request = req
    monkeypatch.setattr(ShadowResearchRunner, '_prepare', lambda self, *args:
                        (self.test_request, None))
    return r, req


def test_insufficient_freshness_defers_without_spending_parent_retry(monkeypatch):
    r, _ = runner(monkeypatch)
    # Same available budget as actual cycle85: 16.79ms after the 20ms reserve.
    for now in [100.07821, 100.08, 100.085]:
        with pytest.raises(ValueError, match='RETRY_AWAITING_FRESH_INPUT'):
            r._prepare(now, 10.)
        assert r.retry_waiting_parent == 83 and r.pending is None


def test_fresh_arrival_with_enough_budget_allows_same_single_retry(monkeypatch):
    r, req = runner(monkeypatch)
    c = req.context
    r.test_request = replace(req, context=replace(
        c, navigation_stamp=100.05, attitude_stamp=100.05, prediction_source_stamp=100.05,
        observation_stamp=100.04, prediction_valid_until=100.165))
    assert r._prepare(100.085, 10.)[0] is r.test_request


def test_wait_guard_does_not_delay_regular_work_or_apply_to_another_parent(monkeypatch):
    r, req = runner(monkeypatch)
    r.retry_waiting_parent = None
    assert r._prepare(100.08, 10.)[0] is req
    r.retry_waiting_parent, r.execution_status = 82, dict(active=83)
    assert r._prepare(100.08, 10.)[0] is req


def test_disabled_fallback_cannot_make_unusable_retry_look_ready(monkeypatch):
    r, _ = runner(monkeypatch)
    r.solver.fallback_enabled = False
    r.solver.fallback_estimate = .001
    with pytest.raises(ValueError, match='RETRY_AWAITING_FRESH_INPUT'):
        r._prepare(100.08, 10.)


def test_real_runner_waits_then_dispatches_only_one_retry_after_new_input(monkeypatch):
    from concurrent.futures import Future
    from test_follow_mpc_shadow_node import _Adapter, _Pool, _mission, _prediction
    from uav_control.controllers.follow_mpc_seed import MpcConfig, MpcSeedResult
    req = future_request(request(2.), .15)
    pool, events = _Pool(), []
    adapter = _Adapter()
    adapter.clock_generation = 0
    r = P4ResearchRunner(MpcConfig(), adapter, pool, events.append, rate_hz=5.)
    r.deadline_scheduling = True
    r.execution_status = dict(stamp=100.06, active=83, deadline=100.44)
    r.solver.online_budget_guard = True
    r.solver.follow_estimate, r.solver.fallback_estimate = .0221203, .0190372
    r.solver.fallback_enabled, r.solver.fast = True, SimpleNamespace(publication_reserve=.02)
    r.test_request = req
    monkeypatch.setattr(ShadowResearchRunner, '_prepare', lambda self, *args:
                        (self.test_request, None))
    r.update_mission(_mission(mission_id=1), 100.01, 1.01)
    r.update_prediction(_prediction(mission_id=1), 100.01, 1.01)
    r.tick(100.02, 1.02)
    pool.future.set_result(MpcSeedResult(req.context, valid=False))
    r.tick(100.06, 1.06)
    assert r.retry_waiting_parent == 83
    pool.future = Future()
    r.tick(100.07821, 1.07821)
    assert len(pool.requests) == 1
    c = req.context
    r.test_request = replace(req, context=replace(
        c, navigation_stamp=100.05, attitude_stamp=100.05, prediction_source_stamp=100.05,
        observation_stamp=100.04, prediction_valid_until=100.165))
    r.tick(100.085, 1.085)
    assert len(pool.requests) == 2 and r.retry_waiting_parent is None
    pool.future.set_result(MpcSeedResult(r.test_request.context, valid=False))
    r.tick(100.09, 1.09)
    assert r.next_solve >= 1.28 and len(pool.requests) == 2

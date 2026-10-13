"""Actual F2 low-budget phases must wait without consuming the sole worker retry."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from test_p52_retry_freshness import runner


@pytest.mark.parametrize('now,follow_cost,fallback_cost,waiting', [
    (100.0688317, .0203261, .0278958, False),  # regular request63
    (100.0799561, .0162609, .0278958, True),   # sole retry64, no work attempted
    (100.0714524, .0359154, .0328215, True),   # sole retry108, no work attempted
])
def test_recorded_successor_phases_wait_for_budget_before_worker_dispatch(
        monkeypatch, now, follow_cost, fallback_cost, waiting):
    r, _ = runner(monkeypatch)
    r.test_request = replace(
        r.test_request, context=replace(r.test_request.context, observation_stamp=100.,
                                        prediction_valid_until=100.25))
    r.deadline_scheduling = True
    r.retry_waiting_parent = 83 if waiting else None
    r.solver.follow_estimate, r.solver.fallback_estimate = follow_cost, fallback_cost
    with pytest.raises(ValueError, match='SUCCESSOR_AWAITING_FRESH_INPUT'):
        r._prepare(now, 10.)
    assert r.pending is None
    assert r.retry_waiting_parent == (83 if waiting else None)


def test_latest_image_after_wait_keeps_original_parent_and_ttl(monkeypatch):
    r, req = runner(monkeypatch)
    r.test_request = replace(
        req, context=replace(req.context, observation_stamp=100., prediction_valid_until=100.25))
    r.deadline_scheduling = True
    r.solver.follow_estimate, r.solver.fallback_estimate = .0359154, .0328215
    with pytest.raises(ValueError, match='SUCCESSOR_AWAITING_FRESH_INPUT'):
        r._prepare(100.0714524, 10.)
    r.test_request = replace(
        req, context=replace(req.context, navigation_stamp=100.06, attitude_stamp=100.06,
                             prediction_source_stamp=100.05, observation_stamp=100.05,
                             prediction_valid_until=100.175))
    result = r._prepare(100.085, 10.02)[0]
    assert result is r.test_request and result.context.observation_stamp == 100.05
    assert r.retry_waiting_parent == 83


def test_wait_does_not_apply_to_shadow_or_initial_entry(monkeypatch):
    r, req = runner(monkeypatch)
    r.deadline_scheduling = True
    r.retry_waiting_parent = None
    r.execution_status = dict(active=0)
    assert r._prepare(100.08, 10.)[0] is req
    r.execution_status = dict(active=83)
    r.deadline_scheduling = False
    assert r._prepare(100.08, 10.)[0] is req


def test_real_runner_does_not_consume_one_retry_until_fresh_image(monkeypatch):
    from concurrent.futures import Future
    from test_follow_mpc_shadow_node import _Adapter, _Pool, _mission, _prediction
    from test_follow_research_algorithms import request
    from uav_control.controllers.follow_mpc_shadow_node import ShadowResearchRunner
    from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner
    from uav_control.controllers.follow_mpc_seed import MpcConfig, MpcSeedResult
    from uav_control.guidance.follow_problem import future_request
    req = future_request(request(2.), .15)
    req = replace(
        req, context=replace(req.context, observation_stamp=100., prediction_valid_until=100.25))
    pool, events = _Pool(), []
    adapter = _Adapter()
    adapter.clock_generation = 0
    r = P4ResearchRunner(MpcConfig(), adapter, pool, events.append, rate_hz=5.)
    r.deadline_scheduling = True
    r.execution_status = dict(stamp=100.06, active=83, deadline=100.44)
    r.solver.online_budget_guard = True
    r.solver.follow_estimate, r.solver.fallback_estimate = .0162609, .0278958
    r.solver.fallback_enabled, r.solver.fast = True, SimpleNamespace(publication_reserve=.02)
    r.test_request = req
    monkeypatch.setattr(ShadowResearchRunner, '_prepare', lambda self, *args:
                        (self.test_request, None))
    r.update_mission(_mission(mission_id=1), 100.01, 1.01)
    r.update_prediction(_prediction(mission_id=1), 100.01, 1.01)
    r.tick(100.02, 1.02)
    pool.future.set_result(MpcSeedResult(req.context, valid=False))
    r.tick(100.06, 1.06)
    pool.future = Future()
    r.tick(100.0799561, 1.0799561)
    assert len(pool.requests) == 1 and r.retry_waiting_parent == 83
    r.test_request = replace(
        req, context=replace(req.context, navigation_stamp=100.08, attitude_stamp=100.08,
                             prediction_source_stamp=100.08, observation_stamp=100.08,
                             prediction_valid_until=100.205))
    r.tick(100.10, 1.10)
    assert len(pool.requests) == 2 and r.retry_waiting_parent is None
    pool.future.set_result(MpcSeedResult(r.test_request.context, valid=False))
    r.tick(100.12, 1.12)
    assert len(pool.requests) == 2 and r.next_solve >= 1.30


def test_cost_spike_recovers_with_new_images_but_not_repeated_timer_ticks(monkeypatch):
    from concurrent.futures import Future
    from test_follow_mpc_shadow_node import _Adapter, _Pool, _mission, _prediction
    from test_follow_research_algorithms import request
    from uav_control.controllers.follow_mpc_shadow_node import ShadowResearchRunner
    from uav_control.controllers.p4_follow_planner_node import P4ResearchRunner
    from uav_control.controllers.follow_mpc_seed import MpcConfig, MpcSeedResult
    from uav_control.guidance.follow_problem import future_request
    req = future_request(request(2.), .15)
    req = replace(
        req, context=replace(req.context, observation_stamp=100., prediction_valid_until=100.25))
    pool, adapter = _Pool(), _Adapter()
    adapter.clock_generation = 0
    r = P4ResearchRunner(MpcConfig(), adapter, pool, lambda _: None, rate_hz=5.)
    r.deadline_scheduling = True
    r.execution_status = dict(stamp=100.04, active=83, deadline=100.44)
    r.solver.online_budget_guard = True
    r.solver.follow_estimate, r.solver.fallback_estimate = .015, .012
    r.solver.fallback_enabled, r.solver.fast = True, SimpleNamespace(publication_reserve=.02)
    r.test_request = req
    monkeypatch.setattr(ShadowResearchRunner, '_prepare', lambda self, *args:
                        (self.test_request, None))
    r.update_mission(_mission(mission_id=1), 100.01, 1.01)
    r.update_prediction(_prediction(mission_id=1), 100.01, 1.01)
    r.tick(100.02, 1.02)
    pool.future.set_result(MpcSeedResult(req.context, valid=False))
    r.solver.follow_estimate, r.solver.fallback_estimate = .060, .055
    r.tick(100.04, 1.04)
    pool.future = Future()
    for offset in (.05, .051, .052, .053, .054, .055):
        r.tick(100.+offset, 1.+offset)
        assert len(pool.requests) == 1 and r.retry_waiting_parent == 83
    r.test_request = replace(
        req, context=replace(req.context, navigation_stamp=100.1, attitude_stamp=100.1,
                             prediction_source_stamp=100.1, observation_stamp=100.1,
                             prediction_valid_until=100.225))
    r.tick(100.135, 1.135)
    assert len(pool.requests) == 2 and r.retry_waiting_parent is None

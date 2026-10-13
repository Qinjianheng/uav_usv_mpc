"""Recent measured successful work must survive short fresh-image budget waits."""
from dataclasses import replace
import pytest
from uav_control.guidance.follow_cost_evidence import CostEvidence
from test_p52_retry_freshness import runner


def test_recorded_f2_path_costs_do_not_decay_to_warm_priors_before_next_attempt():
    evidence = CostEvidence()
    evidence.observe('fallback', .027707078, 100.)
    evidence.observe('fallback', .019623330, 100.2)
    evidence.observe('follow', .036, 100.2)
    assert evidence.floor('fallback', 100.4) == pytest.approx(.0283982448)
    assert evidence.floor('follow', 100.4) == pytest.approx(.0432)


def test_recent_evidence_defers_recorded114_without_consuming_parent_retry(monkeypatch):
    r, req = runner(monkeypatch)
    r.deadline_scheduling = True
    r.test_request = replace(req, context=replace(
        req.context, observation_stamp=100.4, navigation_stamp=100.448,
        attitude_stamp=100.448, prediction_source_stamp=100.426,
        prediction_valid_until=100.525))
    r.solver._cost_evidence = CostEvidence()
    r.solver._cost_evidence.observe('fallback', .027707078, 100.)
    r.solver._cost_evidence.observe('fallback', .019623330, 100.2)
    r.solver._cost_evidence.observe('follow', .036, 100.2)
    r.solver.follow_estimate, r.solver.fallback_estimate = .015, .014689426
    with pytest.raises(ValueError, match='SUCCESSOR_AWAITING_FRESH_INPUT'):
        r._prepare(100.4701766, 10.)
    assert r.pending is None and r.retry_waiting_parent == 83
    r.test_request = replace(r.test_request, context=replace(
        r.test_request.context, observation_stamp=100.5, navigation_stamp=100.52,
        attitude_stamp=100.52, prediction_source_stamp=100.526,
        prediction_valid_until=100.625))
    assert r._prepare(100.535, 10.065)[0] is r.test_request


def test_one_success_cost_spike_does_not_freeze_future_dispatch_forever():
    evidence = CostEvidence()
    evidence.observe('follow', .08, 100.)
    assert evidence.floor('follow', 100.2) == pytest.approx(.096)
    assert evidence.floor('follow', 102.5) == .015


def test_rolling_median_uses_recent_work_instead_of_one_outlier():
    evidence = CostEvidence()
    for i, value in enumerate((.02, .021, .02, .021, .08)):
        evidence.observe('fallback', value, 100.+i*.05)
    assert evidence.floor('fallback', 100.3) == pytest.approx(.0252)


def test_clock_reset_cannot_reuse_cost_history_from_a_future_epoch():
    evidence = CostEvidence()
    evidence.observe('follow', .04, 100.)
    evidence.observe('follow', .02, 99.)
    assert evidence.floor('follow', 99.) == pytest.approx(.024)


def test_scheduler_clock_reset_clears_history_before_any_new_solve_can_run():
    evidence = CostEvidence()
    evidence.observe('follow', .08, 100.)
    evidence.observe('fallback', .08, 100.)
    assert evidence.floor('follow', 100.1) == pytest.approx(.096)
    assert evidence.floor('follow', 1.) == .015
    assert evidence.floor('fallback', 1.) == .012


def test_invalid_cost_does_not_enter_timing_evidence():
    evidence = CostEvidence()
    for value in (0., -1., float('nan')):
        with pytest.raises(ValueError):
            evidence.observe('follow', value, 100.)
    assert evidence.floor('follow', 100.) == .015


def test_input_wait_may_clear_geometry_warm_start_without_erasing_timing_evidence():
    from test_follow_research_algorithms import request
    from test_p47_follow_guided_solver import solver
    from uav_control.guidance.follow_problem import future_request
    from uav_control.guidance.follow_cost_evidence import evidence_for
    s = solver(future_request(request(2.), .15), online_budget_guard=True)
    evidence_for(s).observe('follow', .03, 100.)
    s.clear_warm_start()
    assert evidence_for(s).floor('follow', 100.2) == pytest.approx(.036)

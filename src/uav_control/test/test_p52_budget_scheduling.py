"""Budget heuristics cannot permanently starve warmed P47 after a slow infeasible search."""
from test_follow_research_algorithms import request
from test_p47_follow_guided_solver import solver
from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.guidance.follow_problem import future_request
from uav_control.guidance.fast_follow_minco import freshness


def guarded(remaining):
    req = future_request(request(2.), .15)
    s = solver(req, online_budget_guard=True, refinement=False, clock=lambda: 0.)
    until, _ = freshness(req, req.now_stamp)
    s.wall_clock = lambda: until-.02-remaining
    return req, s


def test_warmed_follow_qva_can_run_inside_a_32ms_budget():
    req, s = guarded(.032)
    r = s.solve(req)
    assert r.valid
    assert r.metrics['follow_guided']['initial_status'] != 'SHORT_BUDGET_P44_PRIORITY'


def test_fallback_that_cannot_fit_does_not_reserve_budget_and_starve_p47():
    req, s = guarded(.032)
    s.follow_estimate, s.fallback_estimate = .015, .06
    r = s.solve(req)
    assert r.valid
    assert r.metrics['follow_guided']['independent_valid']


def test_slow_infeasible_fallback_is_not_the_estimate_of_a_successful_candidate():
    req, s = guarded(.04)
    clock = [0.]
    s.clock = lambda: clock[0]
    s.follow_estimate = 1.  # this case selects the existing P44 priority branch

    def exhaustive_failure(r):
        clock[0] += .03
        return MpcSeedResult(r.context, valid=False, solver_status='VISIBILITY_INFEASIBLE')

    s.fallback._solve = exhaustive_failure
    s.solve(req)
    assert s.fallback_estimate <= .012
    assert s.fallback_failed_cost >= .03


def test_a_follow_timeout_cannot_permanently_inhibit_later_normal_cost_work(monkeypatch):
    import uav_control.controllers.follow_guided_feasible as module
    req, s = guarded(.04)
    clock = [0.]
    s.clock = lambda: clock[0]
    original = module.rollout_follow_fast

    def delayed(*args, **kwargs):
        clock[0] += .08  # deterministic pause past the actual original request cutoff
        return original(*args, **kwargs)

    monkeypatch.setattr(module, 'rollout_follow_fast', delayed)
    first = s.solve(req)
    assert not first.valid and first.solver_status == 'DEADLINE_EXCEEDED'
    assert s.follow_estimate >= .08
    monkeypatch.setattr(module, 'rollout_follow_fast', original)
    results = []
    for _ in range(12):
        clock[0] = 0.  # subsequent independent, fresh-budget calls have normal work cost
        results.append(s.solve(req))
    assert any(v.metrics['follow_guided']['independent_valid'] for v in results)


def test_skipped_fallback_estimate_recovers_after_an_isolated_success_cost_outlier():
    req, s = guarded(.01)
    s.fallback_estimate = .08
    for _ in range(12):
        s.solve(req)
    assert s.fallback_estimate <= .012

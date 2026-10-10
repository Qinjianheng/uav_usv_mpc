"""P44 equivalent candidates, fallback provenance and unchanged safety checks."""
from test_follow_revalidation import inputs
from test_progress_follow import model
from uav_control.controllers.p43_follow_solver import P43FollowSolver


def test_scalar_candidate_provenance_and_exclusive_stages():
    req, _, _ = inputs()
    result = P43FollowSolver(
        model(), refinement='none', wall_clock=lambda: req.now_stamp).solve(req)
    audit = result.metrics['p43']
    assert audit['generated'] == 5
    assert len(audit['ranked_candidates']) == 5
    assert audit['attempts'][0]['strict_seconds'] > 0
    assert audit['attempts'][0]['scale'] in (0., .5, 1.)
    assert audit['attempts'][0]['candidate_index'] in range(5)
    stages = result.metrics['cycle_profile']['stages']
    for name in ('candidate_reference', 'candidate_proxy_geometry', 'candidate_proxy_score',
                 'candidate_sort', 'candidate_strict_validation', 'attitude_extrema'):
        assert stages[name]['calls'] > 0


def test_shared_batch_lazy_candidates_match_scalar_oracle():
    import numpy as np
    from dataclasses import replace
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    from uav_control.guidance.follow_problem import FollowProblem, future_request
    from test_follow_research_algorithms import request
    req = request(2.)
    rng = np.random.default_rng(44)
    for yaw in (0., np.pi-.01, -np.pi+.01, 7.):
        for speed in (0., 1.5):
            state = list(req.state)
            state[6:9] = rng.uniform(-.5, .5, 3)
            state[9] = yaw
            raw = replace(req, state=tuple(state), target_velocities=tuple(
                (speed, .1*speed, 0.) for _ in req.target_velocities))
            raw = future_request(raw, .15)
            object.__setattr__(raw, 'reference_yaw_rate', .3)
            problem = FollowProblem(raw, model(), 1.2)
            scalar = P43FollowSolver(model(), refinement='none').candidates(
                raw, problem, np.zeros(3))
            for ablation in 'BCD':
                solver = P44FollowSolver(model(), ablation=ablation)
                batch = solver.candidates(raw, problem, np.zeros(3))
                assert [x[5]['candidate_index'] for x in sorted(scalar, key=lambda r: r[:2])] == [
                    x[5]['candidate_index'] for x in sorted(batch, key=lambda r: r[:2])]
                for a, b in zip(scalar, batch):
                    assert a[3] == b[3]
                    assert np.allclose(a[4], b[4], atol=2e-14)
                    assert abs(a[1]-b[1]) < 1e-11
                    if b[2] is not None:
                        assert np.allclose(a[2].q, b[2].q, atol=2e-14)


def test_lazy_seed_constructs_only_strict_candidate_and_all_five_fallback(monkeypatch):
    from uav_control.controllers import p43_follow_solver as parent
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    from uav_control.guidance.follow_profile import CycleProfile
    req, _, _ = inputs()
    with CycleProfile(1) as cycle:
        result = P44FollowSolver(
            model(), ablation='D', wall_clock=lambda: req.now_stamp).solve(req)
    assert result.valid
    assert cycle.report()['stages']['candidate_seed_construction']['calls'] == 2
    seen = []

    def reject(request, seed, *args):
        seen.append(seed)
        return MpcSeedResult(request.context, solver_status='UNKNOWN_FAILURE')
    monkeypatch.setattr(parent, 'validate_seed', reject)
    result = P44FollowSolver(model(), ablation='F', wall_clock=lambda: req.now_stamp).solve(req)
    assert not result.valid and len(seen) == 5
    assert len(result.metrics['p43']['attempts']) == 5
    assert len({a['candidate_index'] for a in result.metrics['p43']['attempts']}) == 5


def test_ordering_does_not_discard_candidates_or_relabel_unattempted():
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    from uav_control.guidance.follow_problem import FollowProblem
    import numpy as np
    req, _, _ = inputs()
    s = P44FollowSolver(model(), ablation='E', wall_clock=lambda: req.now_stamp)
    candidates = s.candidates(req, FollowProblem(req, model(), 1.2), np.zeros(3))
    for reason in ('DYNAMIC_INFEASIBLE', 'YAW_RATE_LIMIT', 'VISIBILITY_INFEASIBLE',
                   'ATTITUDE_LIMIT',
                   'UNKNOWN_FAILURE'):
        reordered = s.reorder(candidates, MpcSeedResult(req.context, solver_status=reason))
        assert sorted(r[5]['candidate_index'] for r in reordered) == list(range(5))
    expired = P44FollowSolver(model(), wall_clock=lambda: req.now_stamp+.126).solve(req)
    assert not expired.valid and not expired.metrics['p43']['attempts']
    assert expired.solver_status != 'DYNAMIC_INFEASIBLE'


def test_nonfinite_inputs_and_clamping_do_not_gain_proxy_admission():
    from dataclasses import replace
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    req, _, _ = inputs()
    for value in (float('nan'), float('inf'), -float('inf')):
        state = list(req.state)
        state[0] = value
        raw = replace(req, state=tuple(state))
        result = P44FollowSolver(model(), wall_clock=lambda: req.now_stamp).solve(raw)
        assert not result.valid and not result.metrics['p43']['attempts']


def test_known_equilibrium_ties_and_large_jerk_clamp_match_scalar():
    from dataclasses import replace
    import numpy as np
    from test_follow_research_algorithms import request
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    from uav_control.guidance.follow_problem import FollowProblem, future_request
    for distant in (0., 200.):
        raw = request()
        raw = replace(raw, target_positions=tuple((x+distant, y, z)
                                                  for x, y, z in raw.target_positions))
        req = future_request(raw, .15)
        problem = FollowProblem(req, model(), 1.2)
        reference = P43FollowSolver(model()).candidates(req, problem, np.zeros(3))
        for e in 'BCD':
            rows = P44FollowSolver(model(), ablation=e).candidates(req, problem, np.zeros(3))
            assert [r[5]['candidate_index'] for r in sorted(rows, key=lambda r: r[:2])] == [
                r[5]['candidate_index'] for r in sorted(reference, key=lambda r: r[:2])]
            if distant == 0:
                assert rows[0][1] == rows[1][1] == rows[2][1]
            for a, b in zip(reference, rows):
                assert np.allclose(a[4], b[4], atol=1e-14)
                assert np.linalg.norm(b[4][:2]) <= 6.+1e-12
                assert np.linalg.norm(np.asarray(req.state[6:8])+b[4][:2]*1.2) <= 3.+1e-12


def test_attitude_fallback_orders_acceleration_rather_than_jerk_size():
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    from uav_control.controllers.follow_mpc_seed import MpcSeedResult
    import numpy as np
    req, _, _ = inputs()
    rows = [(0, 0., None, 0., np.zeros(3), dict(end_acceleration=(3., 0., 0.))),
            (0, 1., None, 0., np.array((-2.5, 0., 0.)), dict(end_acceleration=(0., 0., 0.)))]
    got = P44FollowSolver(model(), ablation='E').reorder(
        rows, MpcSeedResult(req.context, solver_status='ATTITUDE_LIMIT'))
    assert got[0] is rows[1]

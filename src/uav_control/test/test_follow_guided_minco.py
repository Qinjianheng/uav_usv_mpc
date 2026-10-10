"""Original FOLLOW forecast rollout and strict MINCO admission, without flight I/O."""
from dataclasses import replace
import math

import numpy as np
import pytest

from test_follow_research_algorithms import request
from test_progress_follow import model
from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState
from uav_control.guidance.follow_problem import FollowProblem, future_request


def rolling(raw=None, **kwargs):
    from uav_control.guidance.follow_rollout import rollout_follow
    req = future_request(raw or request(2.), .15)
    return req, rollout_follow(FollowProblem(req, model(), 1.2), **kwargs)


@pytest.mark.parametrize('height', [-5., -3., -7.])
def test_first_command_is_original_follow_including_altitude_and_shaping(height):
    raw = replace(request(2.), state=(-8., 1., height, 1., 0., 0., .2, 0., 0., 0.))
    command = (1.2, -.1, .1)
    req, ref = rolling(raw, initial_command=command)
    problem = FollowProblem(req, model(), 1.2)
    p, v = problem.target_state((0.,))
    target = FlightKinematicState(tuple(p[0]), tuple(v[0]))
    core = FlightGuidanceCore(reserve_clearance=.5)
    core.previous_velocity = command
    expected = core.command('FOLLOW', FlightKinematicState(req.state[:3], req.state[3:6]), target)
    assert ref.commands[0] == pytest.approx(expected.velocity)
    assert np.asarray(ref.accelerations)[0] == pytest.approx(req.state[6:9])
    assert ref.metrics['command_source'] == 'provided'


def test_virtual_velocity_cannot_jump_to_command_and_heave_cannot_change_altitude():
    raw = replace(request(2.), state=(-9., 0., -5., 0., 0., 0., 0., 0., 0., 0.))
    req, ref = rolling(raw)
    assert np.linalg.norm(np.asarray(ref.velocities)[1]-req.state[3:6]) < .01
    assert ref.metrics['command_source'] == 'execution_velocity_initialization'
    heave = replace(raw, target_positions=tuple((p[0], p[1], math.sin(t)) for p, t in
                                                zip(raw.target_positions, raw.prediction_times)))
    _, other = rolling(heave)
    assert np.asarray(other.positions)[:, 2] == pytest.approx(-5.)
    assert other.positions == ref.positions


@pytest.mark.parametrize('speed,omega', [(2., 0.), (2., .3), (0., 0.), (2., -.3)])
def test_forecast_cases_generate_finite_minco_boundaries(speed, omega):
    from uav_control.controllers.direct_reference_minco import follow_guided_seed
    raw = request(speed)
    if omega:
        ts = np.asarray(raw.prediction_times)
        raw = replace(raw, target_positions=tuple(map(tuple, np.c_[
            speed*np.sin(omega*ts)/omega, speed*(1-np.cos(omega*ts))/omega, ts*0])),
                      target_velocities=tuple(map(tuple, np.c_[speed*np.cos(omega*ts),
                                                               speed*np.sin(omega*ts), ts*0])))
    req, ref = rolling(raw)
    seed = follow_guided_seed(req, FollowProblem(req, model(), 1.2), ref)
    from uav_control.guidance.follow_minco_optimizer import _trajectory
    curve = _trajectory(seed, seed.q, seed.durations)
    assert seed.start == req.state
    assert np.r_[curve.sample(0.).position, curve.sample(0.).velocity,
                 curve.sample(0.).acceleration] == pytest.approx(req.state[:9])
    last = curve.sample(1.2)
    assert np.r_[last.position, last.velocity, last.acceleration] == pytest.approx(seed.end)
    assert np.all(np.isfinite(curve.coefficients))
    assert seed.q[0] == pytest.approx(ref.sample(.4)[0])
    assert np.max(np.abs(np.diff(seed.yaw))) < math.pi


@pytest.mark.parametrize('refinement', ['none', 'q', 'qt'])
def test_guided_solver_reports_own_feasibility_and_strict_validation(refinement):
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    req = future_request(request(2.), .15)
    result = FollowGuidedMinco(model(), refinement=refinement,
                               wall_clock=lambda: req.now_stamp).solve(req)
    assert result.valid
    detail = result.metrics['follow_guided']
    assert detail['independent_valid'] and not detail['fallback_used']
    assert result.metrics['full_projection_validated']
    assert detail['initial_fit']['position_rmse'] < 1e-7
    assert detail['fit']['position_rmse'] < 1e-4
    assert detail['boundary_measurement_position_error'] > 0
    assert result.context == req.context


def test_expired_and_truth_source_never_produce_valid_guided_output():
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    req = future_request(request(2.), .15)
    solver = FollowGuidedMinco(model(), wall_clock=lambda: req.now_stamp+.126)
    assert not solver.solve(req).valid
    solver.wall_clock = lambda: req.now_stamp
    assert not solver.solve(replace(req, context=replace(
        req.context, prediction_source='truth'))).valid


def test_explicit_follow_objective_gradients_and_q_only_dimension():
    from uav_control.controllers.direct_reference_minco import follow_guided_seed
    from uav_control.guidance.p43_minco_objective import TrackingMincoObjective
    from uav_control.guidance.follow_minco_optimizer import MincoConfig
    raw = replace(request(2.), state=(-7., .2, -4.8, 1.8, 0., 0., .1, 0., -.1, 0.))
    req, ref = rolling(raw)
    problem = FollowProblem(req, model(), 1.2)
    seed = follow_guided_seed(req, problem, ref)
    for mode, size in (('q', 6), ('qt', 11)):
        objective = TrackingMincoObjective(problem, seed, MincoConfig(mode=mode), reference=ref,
                                           tracking_weights=(.16, 2/9, 0.),
                                           yaw_optimize=mode == 'qt')
        x = objective.initial()
        assert len(x) == size
        value, gradient = objective(x)
        numerical = []
        for i in range(len(x)):
            direction = np.zeros(len(x))
            direction[i] = 1e-5
            numerical.append((objective(x+direction)[0]-objective(x-direction)[0])/2e-5)
        assert np.isfinite(value)
        assert gradient == pytest.approx(numerical, rel=3e-4, abs=2e-4)
        assert objective.references[0] == pytest.approx(np.asarray(ref.positions))


def test_refinement_attempts_invalid_initial_and_reports_fallback_separately():
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    raw = replace(request(2.), state=(-5., 0., -5., 2., 0., 0., 2.8, 0., 0., 0.))
    req = future_request(raw, .15)
    result = FollowGuidedMinco(model(), refinement='q',
                               wall_clock=lambda: req.now_stamp).solve(req)
    detail = result.metrics['follow_guided']
    assert detail['initial_status'] != 'SAMPLED_FEASIBLE'
    assert detail['optimization_attempted']
    if detail['fallback_used']:
        assert not detail['independent_valid']
        assert result.candidate_kind == 'p44_fallback'
        assert detail['fallback_valid'] == result.valid


def test_turn_across_pi_and_numerical_response_convergence():
    from uav_control.guidance.follow_rollout import FollowRolloutConfig
    raw = request(2.)
    ts = np.asarray(raw.prediction_times)
    angles = math.pi-.05+.2*ts
    raw = replace(raw, state=(5., 0., -4., -2., 0., 0., 0., 0., 0., math.pi-.05),
                  target_positions=tuple(map(tuple, np.c_[-2*ts, -.2*ts**2, ts*0])),
                  target_velocities=tuple(map(tuple, np.c_[2*np.cos(angles), 2*np.sin(angles),
                                                           ts*0])))
    _, coarse = rolling(raw)
    _, fine = rolling(raw, config=FollowRolloutConfig(integration_dt=.005))
    assert coarse.sample(1.2)[0] == pytest.approx(fine.sample(1.2)[0], abs=.006)
    assert coarse.sample(1.2)[1] == pytest.approx(fine.sample(1.2)[1], abs=.006)


def test_engine_registration_selects_guided_and_preserves_p44():
    from types import SimpleNamespace
    from uav_control.controllers.follow_research_shadow_node import FollowResearchShadowNode
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    from uav_control.controllers.p44_follow_solver import P44FollowSolver
    for engine, cls in (('follow_guided_minco', FollowGuidedMinco),
                        ('p44_adaptive', P44FollowSolver)):
        parameters = dict(research_mode='greedy_minco', minco_engine=engine,
                          shadow_rate_hz=5., follow_guided_refinement='q')
        fake = SimpleNamespace(
            declare_parameter=lambda name, value: parameters.setdefault(name, value),
            get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=100_000_000_000)),
            adapter=None, pool=None, _publish_event=lambda event: None)
        runner = FollowResearchShadowNode.make_runner(
            fake, replace(model().config, allow_synthetic_predictions=False))
        assert isinstance(runner.solver, cls)


def test_accepted_parent_boundary_cannot_be_rewritten_to_hide_measurement_error():
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    from uav_control.guidance.follow_minco_execution import accepted_request
    from test_follow_sitl_execution import candidate
    req = future_request(request(2.), .15)
    parent = candidate()
    req = replace(req, context=replace(req.context, execution_start_stamp=parent.start+.1))
    accepted = accepted_request(req, parent)
    original = accepted.prior_curve
    solver = FollowGuidedMinco(model(), wall_clock=lambda: req.now_stamp)
    result = solver.solve(accepted)
    assert accepted.prior_curve == original
    assert result.metrics['follow_guided']['boundary_measurement_position_error'] > 1.
    if result.valid:
        assert result.positions[0] == pytest.approx(accepted.state[:3])
        assert result.velocities[0] == pytest.approx(accepted.state[3:6])
        assert result.accelerations[0] == pytest.approx(accepted.state[6:9])
    bad = replace(accepted, state=(99.,)*10)
    assert not solver.solve(bad).valid


def test_overspeed_measurement_is_preserved_with_legal_command_initialization():
    raw = replace(request(2.), state=(-5., 0., -5., 6.3, 0., 0., 0., 0., 0., 0.))
    req, ref = rolling(raw)
    assert ref.velocities[0] == pytest.approx(req.state[3:6])
    assert np.linalg.norm(ref.commands[0][:2]) <= 6.2
    assert ref.metrics['command_initialization_clipped']


@pytest.mark.parametrize('change', ['behind', 'sea', 'acceleration'])
def test_whole_target_and_hard_physics_still_reject_guided_output(change):
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    raw = request(2.)
    if change == 'behind':
        raw = replace(raw, target_positions=tuple((x-100., y, z)
                                                  for x, y, z in raw.target_positions))
    elif change == 'sea':
        raw = replace(raw, state=(*raw.state[:2], -.1, *raw.state[3:]))
    else:
        raw = replace(raw, state=(*raw.state[:6], 3.5, 0., 0., raw.state[9]))
    req = future_request(raw, .15)
    result = FollowGuidedMinco(model(), wall_clock=lambda: req.now_stamp).solve(req)
    assert not result.valid


def test_new_prediction_version_is_bound_to_new_rollout_and_revalidated():
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    req = future_request(request(2.), .15)
    solver = FollowGuidedMinco(model(), wall_clock=lambda: req.now_stamp)
    assert solver.solve(req).valid
    changed = replace(req, context=replace(req.context, prediction_sequence_id=2),
                      target_positions=tuple((x-100., y, z) for x, y, z in req.target_positions))
    result = solver.solve(changed)
    assert result.context.prediction_sequence_id == 2
    assert result.metrics['follow_guided']['rollout']['prediction_sequence_id'] == 2
    assert not result.valid


def test_final_deadline_check_discards_previously_valid_nominal_curve():
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    req = future_request(request(2.), .15)
    stamps = iter((req.now_stamp, req.now_stamp+.126))
    result = FollowGuidedMinco(model(), clock=lambda: 0.,
                               wall_clock=lambda: next(stamps)).solve(req)
    assert not result.valid and result.solver_status == 'DEADLINE_EXCEEDED'
    assert result.metrics['full_projection_validated']
    assert not result.metrics['follow_guided']['independent_valid']


def test_all_rolling_commands_reuse_original_core_and_sea_guard():
    raw = replace(request(2.), state=(-7., .5, -.3, 1., 0., 1., 0., 0., 0., 0.))
    req, ref = rolling(raw)
    problem = FollowProblem(req, model(), 1.2)
    core = FlightGuidanceCore(reserve_clearance=.5)
    core.previous_velocity = req.state[3:6]
    for i, command in enumerate(ref.commands):
        t = i*.05
        p, v, _ = ref.sample(t)
        tp, tv = problem.target_state((t,))
        expected = core.command('FOLLOW', FlightKinematicState(tuple(p), tuple(v)),
                                FlightKinematicState(tuple(tp[0]), tuple(tv[0])))
        assert command == pytest.approx(expected.velocity)
    assert ref.metrics['safety_states'] != ('SAFE',)


def test_rollout_interrupts_inside_fine_integration_at_explicit_deadline():
    from uav_control.guidance.follow_rollout import rollout_follow, FollowRolloutConfig
    req = future_request(request(2.), .15)
    ticks = iter(np.arange(0., .1, .001))
    with pytest.raises(ValueError, match='DEADLINE_EXCEEDED'):
        rollout_follow(FollowProblem(req, model(), 1.2),
                       FollowRolloutConfig(integration_dt=.0005),
                       deadline=.005, clock=lambda: next(ticks))


def test_solver_stops_rollout_before_coefficient_work_when_budget_expires():
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    from uav_control.guidance.follow_rollout import FollowRolloutConfig
    req = future_request(request(2.), .15)
    ticks = [0.]

    def clock():
        ticks[0] += .001
        return ticks[0]

    limited = model()
    limited.config = replace(limited.config, solve_budget=.03)
    result = FollowGuidedMinco(limited, clock=clock, wall_clock=lambda: req.now_stamp,
                               rollout_config=FollowRolloutConfig(integration_dt=.0005)).solve(req)
    assert not result.valid
    assert result.solver_status == 'DEADLINE_EXCEEDED'
    assert result.timing['validation'] == 0.
    assert result.timing['initial_coefficient_construction'] == 0.
    assert result.metrics['cycle_profile']['counts'].get('minco_builds', 0) == 0


@pytest.mark.parametrize('field', ['control_dt', 'integration_dt'])
def test_tiny_step_is_rejected_before_control_array_allocation(field):
    from uav_control.guidance.follow_rollout import rollout_follow, FollowRolloutConfig
    req = future_request(request(2.), .15)
    config = dict(integration_dt=1e-12)
    if field == 'control_dt':
        config['control_dt'] = 1e-10
    with pytest.raises(ValueError, match='FOLLOW_ROLLOUT_STEP_LIMIT'):
        rollout_follow(FollowProblem(req, model(), 1.2), FollowRolloutConfig(**config))


def test_real_failed_seed_uses_bounded_p44_and_cannot_count_as_independent_success():
    from uav_control.controllers.direct_reference_minco import FollowGuidedMinco
    raw = replace(request(2.), state=(-5., 0., -5., 2., 0., 0., 2.8, 0., 0., 0.))
    req = future_request(raw, .15)
    outputs = []
    for enabled in (False, True):
        solver = FollowGuidedMinco(
            model(), fallback_enabled=enabled, wall_clock=lambda: req.now_stamp, clock=lambda: 0.)
        outputs.append(solver.solve(req))
    assert not outputs[0].valid
    assert not outputs[0].metrics['follow_guided']['fallback_used']
    assert outputs[1].valid
    detail = outputs[1].metrics['follow_guided']
    assert detail['fallback_used'] and detail['fallback_valid']
    assert not detail['independent_valid']
    assert outputs[1].candidate_kind == 'p44_fallback'


def test_sitl_cli_records_the_refinement_actually_selected_for_each_engine(tmp_path, monkeypatch):
    import json
    import sys
    from types import SimpleNamespace
    import yaml
    import p45_minco_sitl as driver

    def prepare(output, condition):
        (output/'config').mkdir(parents=True)
        (output/'config/research.yaml').write_text(yaml.safe_dump({
            'p4_follow_planner_node': {'ros__parameters': {'minco_engine': condition[3]}}}))
        (output/'condition.json').write_text('{}')

    monkeypatch.setattr(driver, 'prepare', prepare)
    monkeypatch.setattr(driver.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=0))
    for engine in ('follow_guided_minco', 'p44_adaptive'):
        output = tmp_path/engine
        argv = ['p45', str(output), '--engine', engine, '--refinement', 'q']
        monkeypatch.setattr(sys, 'argv', argv)
        with pytest.raises(SystemExit) as exited:
            driver.main()
        assert exited.value.code == 0
        params = yaml.safe_load((output/'config/research.yaml').read_text())[
            'p4_follow_planner_node']['ros__parameters']
        key = 'p44_refinement' if engine == 'p44_adaptive' else 'follow_guided_refinement'
        assert params[key] == json.loads((output/'condition.json').read_text())['refinement']

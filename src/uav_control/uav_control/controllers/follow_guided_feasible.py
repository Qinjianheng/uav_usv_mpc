"""P4.7 bounded FOLLOW/QVA research with original strict admission and P44/D fallback."""
from dataclasses import replace

import numpy as np

from uav_control.controllers.direct_reference_minco import follow_guided_seed
from uav_control.controllers.follow_mpc_seed import MpcSeedResult
from uav_control.controllers.p44_follow_solver import P44FollowSolver
from uav_control.guidance.fast_follow_minco import freshness
from uav_control.guidance.follow_fast_validation import validate_seed
from uav_control.guidance.follow_limits import constraint_snapshot
from uav_control.guidance.follow_problem import FollowProblem
from uav_control.guidance.follow_profile import stage
from uav_control.guidance.follow_rollout import FollowRolloutConfig, rollout_follow_fast
from uav_control.guidance.follow_terminal_correction import TerminalCorrectionObjective


class P47FollowGuidedMinco(P44FollowSolver):
    """Separate engine; never grants execution authority, changes limits or renews a lease."""

    def __init__(self, model, terminal_mode='qva', refinement=True, scheduling='adaptive',
                 rollout_config=FollowRolloutConfig(), fallback_enabled=True, **kwargs):
        super().__init__(model, refinement='none', **kwargs)
        if terminal_mode not in ('a', 'va', 'qva') or scheduling not in ('legacy', 'adaptive'):
            raise ValueError('INVALID_P47_CONFIG')
        if not isinstance(refinement, bool) or not isinstance(fallback_enabled, bool):
            raise ValueError('INVALID_P47_CONFIG')
        rollout_config.validate()
        self.terminal_mode, self.refinement = terminal_mode, refinement
        self.scheduling = scheduling
        self.rollout_config, self.fallback_enabled = rollout_config, fallback_enabled
        self.fallback = P44FollowSolver(model, duration=self.duration, rolling=self.rolling,
                                        ablation='D', refinement='none', clock=self.clock,
                                        wall_clock=self.wall_clock)
        self.mode = 'p47_follow_guided'
        # P44 measured P99 is a scheduling estimate, never an execution guarantee.
        self.fallback_estimate = .010
        self.follow_estimate = .010
        self.validation_estimate = .004

    def clear_warm_start(self):
        super().clear_warm_start()
        self.fallback.clear_warm_start()

    def _solve(self, request):
        started, now = self.clock(), self.wall_clock()
        until, _ = freshness(request, now)
        end = started+min(self.config.solve_budget, until-now-self.fast.publication_reserve)
        timing = dict(initialization=0., coefficient_construction=0., optimization=0.,
                      validation=0., fallback=0., fit_analysis=0.)
        detail = dict(independent_valid=False, independent_status='NOT_ATTEMPTED',
                      initial_status='NOT_ATTEMPTED', boundary_status='NOT_ATTEMPTED',
                      refinement_status='NOT_ATTEMPTED', fallback_used=False,
                      fallback_valid=False, fallback_status='NOT_ATTEMPTED',
                      terminal_mode=self.terminal_mode, refinement=self.refinement,
                      initial_fit=None, fit=None, fit_analysis_outside_solver=True,
                      holding_qualified=False, terminal_correction={},
                      budget=dict(scheduling=self.scheduling,
                                  initial_gate=.025 if self.scheduling == 'legacy' else None,
                                  publication_reserve=self.fast.publication_reserve,
                                  fresh_until=until, effective_seconds=end-started,
                                  fallback_estimate=self.fallback_estimate,
                                  estimate_is_guarantee=False))
        best = MpcSeedResult(request.context, solver_status='NO_FRESHNESS_BUDGET')

        def remaining():
            current = self.wall_clock()
            if not np.isfinite(current) or current < request.now_stamp-1e-9:
                return -1.
            return min(end-self.clock(), until-current-self.fast.publication_reserve)

        def validate(seed):
            stamp = self.clock()
            left = remaining()
            if left <= 0:
                raise TimeoutError()
            with stage('p47_strict_validation'):
                result = validate_seed(request, seed, self.model, self.invariant_cache,
                                       left, self.clock)
            timing['validation'] += self.clock()-stamp
            return result

        try:
            if (not np.isfinite(now) or now < request.now_stamp-1e-9 or remaining() <= 0
                    or (self.scheduling == 'legacy' and remaining() < .025)):
                raise ValueError('NO_FRESHNESS_BUDGET')
            problem = FollowProblem(request, self.model, self.duration)
            reserve = self.fallback_estimate if self.fallback_enabled else 0.
            try_follow = (self.scheduling == 'legacy'
                          or remaining() > self.follow_estimate+reserve)
            if try_follow:
                stamp = self.clock()
                try:
                    with stage('p47_follow_initialization'):
                        reference = rollout_follow_fast(problem, self.rollout_config,
                                                        deadline=end-reserve, clock=self.clock)
                        seed = follow_guided_seed(request, problem, reference)
                    detail['rollout'] = reference.metrics
                finally:
                    timing['initialization'] += self.clock()-stamp
                stamp = self.clock()
                with stage('p47_terminal_quadratic'):
                    objective = TerminalCorrectionObjective(problem, seed, reference,
                                                            mode=self.terminal_mode)
                    x = objective.fixed()
                    selected = objective.seed_from(x)
                timing['coefficient_construction'] += self.clock()-stamp
                correction = detail['terminal_correction']
                correction.update(mode=self.terminal_mode, coordinates=objective.names,
                                  component_bounds=objective.bounds, delta=x.tolist(),
                                  original_end=seed.end, original_q=seed.q,
                                  end=selected.end, q=selected.q, durations=selected.durations,
                                  yaw=selected.yaw, start=selected.start,
                                  terminal_position_fixed=True,
                                  bounds_are_safety_thresholds=False)
                best = validate(selected)
                detail['initial_status'] = detail['boundary_status'] = best.solver_status
                correction['boundary_validation'] = dict(
                    valid=best.valid, status=best.solver_status)
                # A fixed yaw violation cannot be repaired through XYZ variables.
                if (self.refinement and best.solver_status != 'YAW_RATE_LIMIT'
                        and remaining() > reserve+self.validation_estimate+.004):
                    stamp = self.clock()
                    detail['optimization_attempted'] = True
                    optimize_end = min(end-reserve-self.validation_estimate, stamp+.006)
                    try:
                        improved_x, info = objective.repair(x, optimize_end, self.clock)
                        detail['refinement_status'] = info['status']
                        correction['optimization'] = info
                        if remaining() > self.validation_estimate:
                            candidate = objective.seed_from(improved_x)
                            tested = validate(candidate)
                            correction['repair_validation'] = dict(
                                valid=tested.valid, status=tested.solver_status)
                            if tested.valid and (not best.valid or info['objective_final']
                                                 <= info['objective_initial']):
                                best, selected, x = tested, candidate, improved_x
                    except TimeoutError:
                        detail['refinement_status'] = 'BOUNDED_STOP_RETAIN_VALID'
                    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
                        detail['refinement_status'] = 'RETAIN_VALID:'+str(error)
                    finally:
                        timing['optimization'] += self.clock()-stamp
                correction.update(delta=x.tolist(), end=selected.end, q=selected.q,
                                  validation=dict(valid=best.valid, status=best.solver_status))
                detail['independent_valid'] = best.valid
                detail['independent_status'] = best.solver_status
                detail['independent_violations'] = best.constraint_violations
                measured = np.asarray(getattr(request, 'measurement_state', ()) or request.state)
                projected = measured.copy()
                dt = request.context.execution_start_stamp-request.context.navigation_stamp
                projected[:3] += measured[3:6]*dt+measured[6:9]*dt**2/2
                projected[3:6] += measured[6:9]*dt
                detail['boundary_policy'] = getattr(request, 'boundary_policy', 'measured')
                for name, part in (('position', slice(0, 3)), ('velocity', slice(3, 6)),
                                   ('acceleration', slice(6, 9))):
                    detail['boundary_measurement_'+name+'_error'] = float(np.linalg.norm(
                        np.asarray(request.state)[part]-measured[part]))
                    detail['boundary_projected_measurement_'+name+'_error'] = float(np.linalg.norm(
                        np.asarray(request.state)[part]-projected[part]))
            else:
                detail['initial_status'] = 'SHORT_BUDGET_P44_PRIORITY'
        except TimeoutError:
            detail['independent_status'] = 'DEADLINE_EXCEEDED'
        except (ValueError, TypeError, FloatingPointError, OverflowError,
                np.linalg.LinAlgError) as error:
            detail['independent_status'] = str(error)
            if not best.valid:
                best = MpcSeedResult(request.context, solver_status=str(error), reason=str(error))

        fallback_gate = .025 if self.scheduling == 'legacy' else .003
        if (not best.valid and self.fallback_enabled and remaining() > fallback_gate
                and not (self.scheduling == 'legacy' and end-started < .025)):
            detail['fallback_used'] = True
            stamp = self.clock()
            self.fallback.wall_clock = self.wall_clock
            self.fallback.config = replace(self.config, solve_budget=remaining())
            with stage('p47_p44_fallback'):
                best = self.fallback._solve(request)
            timing['fallback'] += self.clock()-stamp
            detail['fallback_valid'], detail['fallback_status'] = best.valid, best.solver_status
        snapshot = constraint_snapshot(self.model)
        detail['budget']['remaining_seconds_at_finish'] = remaining()
        if remaining() <= 0:
            best = replace(best, valid=False, solver_status='DEADLINE_EXCEEDED')
            detail['independent_valid'] = detail['fallback_valid'] = False
        elapsed = self.clock()-started
        return replace(best, solve_time=elapsed,
                       candidate_kind='p44_fallback' if detail['fallback_used'] else self.mode,
                       timing=dict(timing, total=elapsed), metrics=dict(
                           best.metrics, follow_guided=detail,
                           constraint_snapshot=snapshot.payload,
                           constraint_fingerprint=snapshot.fingerprint))

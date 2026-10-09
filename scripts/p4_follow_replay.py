#!/usr/bin/env python3
"""Same-record ablations and causal ideal-model rollouts; no execution/holding authority."""
import argparse
import cProfile
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import pstats
import time

import numpy as np

import follow_minco_experiment as base
from uav_control.controllers.progress_follow_solver import ProgressFollowSolver
from uav_control.controllers.short_follow_solver import ShortHorizonFollowSolver
from uav_control.controllers.follow_research_solver import FollowResearchSolver
from uav_control.guidance.fast_follow_minco import FastConfig
from uav_control.guidance.follow_contract import FollowCurve
from uav_control.guidance.follow_minco_optimizer import MincoConfig
from uav_control.guidance.follow_problem import FollowProblem, hypothetical_request
from uav_control.guidance.adaptive_follow_initializer import GreedyConfig


def quantiles(values):
    """Report SI seconds and null for unavailable data."""
    return dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(values, (50, 95, 99))))) if (
        values) else None


def load(paths):
    """Exact source path/line/hash accompanies every immutable request."""
    rows = []
    for path in paths:
        for line, text in enumerate(path.open(), 1):
            row = json.loads(text)
            if row.get('event') == 'completion' and row.get('request'):
                request, model = base.restore_record(row)
                rows.append((str(path), line, hashlib.sha256(text.encode()).hexdigest(),
                             request, model))
    return rows


def to_curve(request, result):
    """A hypothetical coefficient snapshot, never an accepted contract."""
    c, m = request.context, result.metrics
    end = c.execution_start_stamp+sum(m['durations'])
    return FollowCurve(max(1, c.cycle_id), c.mission_id, c.clock_generation,
                       c.prediction_sequence_id, 0, c.navigation_stamp, c.attitude_stamp,
                       c.observation_stamp, c.prediction_source_stamp,
                       min(c.navigation_stamp+.125, c.observation_stamp+.125,
                           c.prediction_source_stamp+.125, c.prediction_valid_until),
                       c.prediction_source_stamp+request.prediction_times[-1],
                       c.execution_start_stamp, end, 0., tuple(m['durations']),
                       tuple(map(tuple, np.asarray(m['xyz_coefficients']).reshape(-1, 3))),
                       tuple(map(tuple, np.asarray(m['yaw_coefficients'])[::-1].T)))


def solver(model, kind, duration, now):
    """Same camera/limits; original 2.4s P31 problem is explicitly separate."""
    if kind == 'p31':
        return FollowResearchSolver(fast_config=FastConfig(), greedy_config=GreedyConfig(),
                                    optimizer_config=MincoConfig(batch_validation=True),
                                    config=model.config, initialization='adaptive')
    return (ShortHorizonFollowSolver if kind == 'p32' else ProgressFollowSolver)(
        model, duration=duration, rolling=kind == 'warm', wall_clock=now)


def progress_counts(results):
    """Missing diagnostics are unknown, distinct from a measured zero count."""
    progress = [r['metrics']['progress'] for r in results if 'progress' in r['metrics']]
    return dict(progress_C=sum(r['stage_C_progress'] for r in progress) if progress else None,
                zero_jerk=sum(r.get('zero_jerk', False) for r in progress) if progress else None,
                warm_hits=sum(r['warm_used'] for r in progress) if progress else None)


def replay(paths, output):
    """All requests, three horizons, cold/warm plus P32 and P31; no state replacement."""
    output.mkdir(exist_ok=False, parents=True)
    cases = load(paths)
    totals, cache, profiles = {}, {}, cProfile.Profile()
    with (output/'same_input.jsonl').open('x') as stream:
        for index, (path, line, fingerprint, req, model) in enumerate(cases):
            for duration in (.8, 1.2, 1.6):
                for kind in ('p32', 'cold', 'warm'):
                    key = (path, kind, duration)
                    current = [req.now_stamp]
                    if key not in cache:
                        cache[key] = solver(model, kind, duration, lambda c=current: c[0])
                    instance = cache[key]
                    instance.wall_clock = lambda: req.now_stamp
                    started = time.perf_counter()
                    result = instance.solve(req)
                    elapsed = time.perf_counter()-started
                    group = totals.setdefault(f'{kind}/{duration}', [])
                    row = dict(path=path, line=line, request_sha256=fingerprint, kind=kind,
                               duration=duration, result=asdict(result), seconds=elapsed,
                               comparison='same_input_local_free_terminal',
                               accepted_by_tracker=False)
                    stream.write(json.dumps(base.base.json_safe(row), allow_nan=False)+'\n')
                    group.append(row)
            # Original P31 has different endpoint/horizon; never rank as same-boundary victory.
            result = solver(model, 'p31', 2.4, lambda: req.now_stamp).solve(req)
            row = dict(path=path, line=line, request_sha256=fingerprint, kind='p31',
                       duration=2.4, result=asdict(result), seconds=result.solve_time,
                       comparison='different_terminal_and_horizon', accepted_by_tracker=False)
            stream.write(json.dumps(base.base.json_safe(row), allow_nan=False)+'\n')
            totals.setdefault('p31/2.4', []).append(row)
            if index % 100 == 0:
                print('replay', index, 'of', len(cases), flush=True)
    summary = {}
    for label, rows in totals.items():
        results = [r['result'] for r in rows]
        summary[label] = dict(
            count=len(rows), feasible=sum(r['valid'] for r in results),
            timing=quantiles([r['seconds'] for r in rows]),
            initialization=quantiles([r['timing'].get('initialization', 0.) for r in results]),
            validation=quantiles([r['timing']['validation'] for r in results
                                  if 'validation' in r['timing']]),
            **progress_counts(results),
            errors={s: sum(r['solver_status'] == s for r in results)
                    for s in set(r['solver_status'] for r in results)})
    # Profiling is separate from wall-time distributions above.
    for _, _, _, req, model in cases[:20]:
        profiles.enable()
        solver(model, 'cold', 1.2, lambda: req.now_stamp).solve(req)
        profiles.disable()
    profiles.dump_stats(str(output/'progress.pstats'))
    with (output/'profile.txt').open('x') as stream:
        pstats.Stats(profiles, stream=stream).strip_dirs().sort_stats('cumtime').print_stats(45)
    (output/'same_input_summary.json').write_text(json.dumps(summary, indent=2))
    return cases


def rolling(cases, output):
    """Each episode advances its OWN accepted-in-this-model curve; raw future UAV never reused."""
    summary = {}
    with (output/'hypothetical_rolling.jsonl').open('x') as stream:
        for rate in (2., 5., 10.):
            for duration in (.8, 1.2, 1.6):
                for kind in ('p32', 'cold', 'warm'):
                    previous, instance, last = None, None, -float('inf')
                    episode, records, reset = 0, [], True
                    for path, line, fingerprint, raw, model in cases:
                        if raw.now_stamp-last < 1/rate-1e-7:
                            continue
                        if previous and (previous.mission_id != raw.context.mission_id or
                                         previous.generation != raw.context.clock_generation):
                            previous = None
                        if instance is None or reset:
                            instance = solver(model, kind, duration, lambda: raw.now_stamp)
                        instance.wall_clock = lambda: raw.now_stamp
                        now = raw.context.execution_start_stamp
                        # At a data gap/expiration terminate; never fabricate continuation.
                        if previous is not None and now > previous.end:
                            previous, reset = None, True
                        boundary = 'episode_initial_recorded_projection'
                        residual = None
                        req = raw
                        if previous is not None:
                            req = hypothetical_request(raw, previous, now)
                            boundary = 'own_hypothetical_previous_curve'
                        else:
                            episode += 1
                        result = instance.solve(req)
                        sampled = None
                        if result.valid:
                            proposed = to_curve(req, result)
                            if previous:
                                delta = np.asarray(proposed.sample(now))-np.asarray(
                                    previous.sample(now))
                                delta[9] = np.arctan2(np.sin(delta[9]), np.cos(delta[9]))
                                residual = float(np.max(np.abs(delta)))
                            if residual is not None and residual > 1e-6:
                                raise ValueError('HYPOTHETICAL_PVA_YAW_HANDOVER_FAILED')
                            previous, reset = proposed, False
                        if previous and previous.start <= now <= previous.end:
                            sampled = previous.sample(now)
                        if sampled:
                            problem = FollowProblem(raw, model, duration)
                            target, target_v = problem.target_state((0.,))
                            heading = np.arctan2(target_v[0, 1], target_v[0, 0])
                            ref = target[0, :2]-5*np.array((np.cos(heading), np.sin(heading)))
                            error = float(np.linalg.norm(np.asarray(sampled[:2])-ref))
                            speed_error = float(np.linalg.norm(np.asarray(sampled[3:5]) -
                                                               target_v[0, :2]))
                        else:
                            error, speed_error = None, None
                        row = dict(
                            path=path, line=line, request_sha256=fingerprint,
                            rate=rate, duration=duration, kind=kind, episode=episode,
                            boundary_policy=boundary, epoch=now, sampled_state=sampled,
                            position_error=error, velocity_error=speed_error,
                            continuous_residual=residual, result_valid=result.valid,
                            reason=result.solver_status, metrics=result.metrics.get('progress'),
                            hypothetical_only=True, accepted_by_tracker=False)
                        stream.write(json.dumps(base.base.json_safe(row), allow_nan=False)+'\n')
                        records.append(row)
                        last = raw.now_stamp
                    errors = [r['position_error'] for r in records
                              if r['position_error'] is not None]
                    effective = np.diff([r['epoch'] for r in records])
                    summary[f'{kind}/{duration}/{rate}'] = dict(
                        count=len(records),
                        episodes=episode, feasible=sum(r['result_valid'] for r in records),
                        sampled_reference_error_rmse=float(np.sqrt(np.mean(np.square(errors))))
                        if errors else None, error_quantiles=quantiles(errors),
                        longest_episode=max((sum(r['episode'] == e for r in records)
                                             for e in range(1, episode+1)), default=0),
                        maximum_continuity_residual=max((r['continuous_residual'] or 0.
                                                        for r in records), default=0.),
                        median_observed_interval=(float(np.median(effective))
                                                  if len(effective) else None),
                        real_closed_loop=False)
    (output/'hypothetical_summary.json').write_text(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base.base._core()
    cases = replay(args.input, args.output)
    rolling(cases, args.output)

#!/usr/bin/env python3
"""Matched original requests and own-curve rolling ablations; never actual flight authority."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

import follow_minco_experiment as base
from p4_follow_replay import load, quantiles, to_curve
from uav_control.controllers.short_follow_solver import ShortHorizonFollowSolver
from uav_control.controllers.progress_follow_solver import (
    ProgressFollowSolver, TrackingFollowSolver, ProgressWeights,
)
from uav_control.guidance.follow_problem import FollowProblem, hypothetical_request
from uav_control.guidance.follow_reference import forecast_viewpoint


VARIANTS = ('p32', 'p33', 'reference', 'bounded', 'none', 'rear', 'gains', 'qt')


def make(model, variant, horizon, now, warm=False):
    """Change one named objective/policy at a time; extended old objective labelled explicitly."""
    kwargs = dict(duration=horizon, rolling=warm, wall_clock=now)
    if variant == 'p32':
        return ShortHorizonFollowSolver(model, **kwargs)
    if variant == 'p33' and horizon <= 1.6:
        return ProgressFollowSolver(model, **kwargs)
    mode = dict(p33='zero', reference='reference', bounded='bounded', none='none').get(
        variant, 'bounded')
    weights = ProgressWeights(position=16., velocity=6.) if variant in ('gains', 'qt') else (
        ProgressWeights())
    return TrackingFollowSolver(model, acceleration_target=mode,
                                rear_priority=variant in ('rear', 'gains', 'qt'),
                                refinement=variant == 'qt', weights=weights, **kwargs)


def errors(request, model, state, beta, horizon):
    """Separate fixed rear, selected constant-beta reference and actual target distance."""
    problem = FollowProblem(request, model, horizon)
    fixed, _, _ = problem.reference((0.,))
    tp, tv = problem.target_state((0.,))
    selected, sv, sa, _, _ = forecast_viewpoint(problem, 0., beta)
    p, v = np.asarray(state[:3]), np.asarray(state[3:6])
    return dict(fixed=float(np.linalg.norm(p[:2]-fixed[0, :2])),
                selected=float(np.linalg.norm(p[:2]-selected[:2])),
                distance=float(np.linalg.norm(p[:2]-tp[0, :2])-5.),
                relative_velocity=float(np.linalg.norm(v[:2]-sv[:2])),
                target_position=tp[0].tolist(), target_velocity=tv[0].tolist(),
                reference_position=selected.tolist(), reference_velocity=sv.tolist(),
                reference_acceleration=sa.tolist())


def summarize(rows):
    """Absent evidence stays null; episode resets cannot count as new-control convergence."""
    pop = [r for r in rows if r['error'] is not None]
    groups = {}
    for episode in sorted(set(r['episode'] for r in rows)):
        rs = [r for r in pop if r['episode'] == episode]
        if not rs:
            continue
        first, last = rs[0]['epoch'], rs[-1]['epoch']
        phases = dict(first5=[r for r in rs if r['epoch'] <= first+5],
                      middle=[r for r in rs if first+(last-first)/3 <= r['epoch']
                              <= first+2*(last-first)/3],
                      last5=[r for r in rs if r['epoch'] >= last-5],
                      after20=[r for r in rs if r['epoch'] >= first+20])
        groups[str(episode)] = dict(
            count=len(rs), duration=last-first,
            initial=rs[0]['error'], final=rs[-1]['error'],
            phases={k: {metric: float(np.sqrt(np.mean([
                r['error'][metric]**2 for r in ps]))) if ps else None
                for metric in ('fixed', 'selected', 'distance')}
                for k, ps in phases.items()})
    return dict(attempted_episodes=len(set(r['episode'] for r in rows)),
                sampled_episodes=len(groups), count=len(rows),
                feasible=sum(r['result']['valid'] for r in rows),
                errors={k: float(np.sqrt(np.mean([r['error'][k]**2 for r in pop])))
                        if pop else None for k in (
                            'fixed', 'selected', 'distance', 'relative_velocity')},
                seconds=quantiles([r['seconds'] for r in rows]),
                failures=dict(Counter(r['result']['solver_status'] for r in rows
                                      if not r['result']['valid'])),
                maximum_jerk=max((r['result']['metrics'].get('maximum_horizontal_jerk', 0.)
                                  for r in rows if r['result']['valid']), default=None),
                minimum_horizontal_margin=min((r['result']['metrics'][
                    'minimum_horizontal_margin'] for r in rows if r['result']['valid']),
                    default=None),
                minimum_vertical_margin=min((r['result']['metrics'][
                    'minimum_vertical_margin'] for r in rows if r['result']['valid']),
                    default=None),
                beta_switches=sum(a['beta'] != b['beta'] for a, b in zip(pop, pop[1:])
                                  if a['episode'] == b['episode']),
                zero_jerk=sum(r['result']['metrics'].get('progress', {}).get('zero_jerk', False)
                              for r in rows),
                warm_hits=sum(r['result']['metrics'].get('progress', {}).get('warm_used', False)
                              for r in rows), episodes=groups,
                max_handover_residual=max((r.get('residual') or 0. for r in rows), default=None),
                median_interval=float(np.median(np.diff([r['epoch'] for r in rows])))
                if len(rows) > 1 else None, real_closed_loop=False)


def run(paths, root, variants, horizons, rolling, warm):
    """Retain every source identity, request, coefficient, failure and actual recorded cadence."""
    root.mkdir(parents=True, exist_ok=False)
    cases = load(paths)
    totals = {}
    for variant in variants:
        for horizon in horizons:
            if variant == 'p32' and horizon > 1.6:
                continue
            label = f'{variant}/{horizon}/{"warm" if warm else "cold"}'
            rows, previous, instance, episode, last, beta = [], None, None, 0, -float('inf'), 0.
            with (root/(label.replace('/', '_')+'.jsonl')).open('x') as stream:
                for path, line, sha, raw, model in cases:
                    if rolling and raw.now_stamp-last < .2-1e-7:
                        continue
                    epoch = raw.context.execution_start_stamp
                    if previous and (epoch > previous.end or
                                     previous.mission_id != raw.context.mission_id or
                                     previous.generation != raw.context.clock_generation):
                        previous, instance = None, None
                    if instance is None:
                        instance = make(model, variant, horizon, lambda: raw.now_stamp, warm)
                    instance.wall_clock = lambda: raw.now_stamp
                    if not previous or not rolling:
                        episode += 1
                    req = (hypothetical_request(raw, previous, epoch)
                           if previous and rolling else raw)
                    before = time.perf_counter()
                    result = instance.solve(req)
                    elapsed = time.perf_counter()-before
                    residual = None
                    if result.valid:
                        proposed = to_curve(req, result)
                        if previous and rolling:
                            delta = (np.asarray(proposed.sample(epoch))
                                     - np.asarray(previous.sample(epoch)))
                            delta[9] = np.arctan2(np.sin(delta[9]), np.cos(delta[9]))
                            residual = float(np.max(np.abs(delta)))
                            if residual > 1e-6:
                                raise ValueError('OWN_REFERENCE_DISCONTINUITY')
                        previous = proposed
                        beta = result.metrics.get('progress', {}).get('beta', 0.)
                    sample = (previous.sample(epoch) if previous
                              and previous.start <= epoch <= previous.end else None)
                    row = dict(path=path, line=line, source_sha256=sha, variant=variant,
                               duration=horizon, warm=warm, episode=episode, epoch=epoch,
                               boundary_policy=getattr(req, 'boundary_policy', 'measured'),
                               request=asdict(req), sampled_state=sample, result=asdict(result),
                               seconds=elapsed, residual=residual, beta=beta,
                               error=errors(raw, model, sample, beta, horizon) if sample else None,
                               hypothetical_only=True, tracker_accepted=False)
                    stream.write(json.dumps(base.base.json_safe(row), allow_nan=False)+'\n')
                    rows.append(row)
                    last = raw.now_stamp
                    if not rolling:
                        previous = None
            totals[label] = summarize(rows)
            (root/'summary.json').write_text(json.dumps(totals, indent=2))
            print(label, len(rows), totals[label]['feasible'], totals[label]['errors'], flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, nargs='+', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--variants', nargs='+', choices=VARIANTS, default=VARIANTS)
    p.add_argument('--horizons', nargs='+', type=float, default=[1.2])
    p.add_argument('--rolling', action='store_true')
    p.add_argument('--warm', action='store_true')
    args = p.parse_args()
    base.base._core()
    run(args.input, args.output, args.variants, args.horizons, args.rolling, args.warm)

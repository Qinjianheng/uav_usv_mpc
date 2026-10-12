#!/usr/bin/env python3
"""P43 matched frozen-request research. Own curves only; no flight authority."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import json
from pathlib import Path
import time

import numpy as np

import p41_follow_replay as prior
from p4_follow_replay import load, to_curve
from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
from uav_control.guidance.follow_limits import ResearchConfig, constraint_snapshot
from uav_control.guidance.follow_problem import hypothetical_request
from uav_control.guidance.follow_profile import CycleProfile


def research_model(original, acceleration, yaw):
    """Only two research limits change; camera, jerk, sea, thrust, tilt and TTL retained."""
    config = replace(ResearchConfig(**asdict(original.config)),
                     maximum_horizontal_acceleration=acceleration, maximum_yaw_rate=yaw)
    return FollowMpcSeed(config, original.intrinsics, original.extrinsics,
                         original.target, original.visibility, original.attitude_config)


def make(model, engine, horizon, now, refinement='none'):
    """Independent opt-in strategy; original P41 remains selectable."""
    if engine == 'p41':
        return prior.make(model, 'rear', horizon, now)
    from uav_control.controllers.p43_follow_solver import P43FollowSolver
    return P43FollowSolver(model, duration=horizon, wall_clock=now, rolling=False,
                           refinement=refinement)


def run_case(cases, path, acceleration=3., yaw=1., horizon=1.2,
             rolling=False, engine='p41', refinement='none'):
    """All population/failures/initialization and episode transitions are retained."""
    rows, previous, instance, episode, last, beta = [], None, None, 0, -float('inf'), 0.
    models = {}
    with path.open('x') as stream:
        for source, line, sha, raw, original in cases:
            if rolling and raw.now_stamp-last < .2-1e-7:
                continue
            if previous and (raw.context.execution_start_stamp > previous.end
                             or previous.mission_id != raw.context.mission_id
                             or previous.generation != raw.context.clock_generation):
                previous, instance = None, None
            key = constraint_snapshot(original).fingerprint
            if key not in models:
                models[key] = research_model(original, acceleration, yaw)
            model = models[key]
            if instance is None:
                instance = make(model, engine, horizon, lambda: raw.now_stamp, refinement)
            instance.wall_clock = lambda: raw.now_stamp
            if not previous or not rolling:
                episode += 1
            epoch = raw.context.execution_start_stamp
            req = hypothetical_request(raw, previous, epoch) if previous and rolling else raw
            started = time.perf_counter()
            with CycleProfile(req.context.cycle_id) as profile:
                result = instance.solve(req)
            elapsed = time.perf_counter()-started
            residual = None
            if result.valid:
                proposed = to_curve(req, result)
                if previous and rolling:
                    delta = np.asarray(proposed.sample(epoch))-np.asarray(previous.sample(epoch))
                    delta[9] = np.arctan2(np.sin(delta[9]), np.cos(delta[9]))
                    residual = float(np.max(np.abs(delta)))
                    if residual > 1e-6:
                        raise ValueError('OWN_REFERENCE_DISCONTINUITY')
                previous = proposed
                beta = result.metrics.get('progress', {}).get('beta', 0.)
            sample = (previous.sample(epoch) if previous
                      and previous.start <= epoch <= previous.end else None)
            row = dict(path=source, line=line, source_sha256=sha, episode=episode, epoch=epoch,
                       request=asdict(req), result=asdict(result),
                       seconds=elapsed, residual=residual,
                       beta=beta, sampled_state=sample, profile=profile.report(),
                       constraint_fingerprint=constraint_snapshot(model).fingerprint,
                       error=prior.errors(raw, model, sample, beta, horizon) if sample else None,
                       hypothetical_only=True, tracker_accepted=False)
            stream.write(json.dumps(prior.base.base.json_safe(row), allow_nan=False)+'\n')
            rows.append(row)
            last = raw.now_stamp
            if not rolling:
                previous = None
    summary = prior.summarize(rows)
    summary.update(acceleration=acceleration, yaw=yaw, horizon=horizon, rolling=rolling,
                   engine=engine, refinement=refinement,
                   beta_counts=dict(Counter(r['beta'] for r in rows if r['result']['valid'])),
                   maximum_tilt=max((r['result']['metrics'].get('maximum_tilt_rad', 0.)
                                     for r in rows if r['result']['valid']), default=None),
                   maximum_thrust=max((r['result']['metrics'].get('maximum_specific_thrust', 0.)
                                       for r in rows if r['result']['valid']), default=None),
                   violations=dict(Counter(
                       k for r in rows for k, v in r['result']['constraint_violations'].items()
                       if v > 1e-6)))
    return summary


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--phase', choices=('limits', 'horizons', 'optimizer'), required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    prior.base.base._core()
    args.output.mkdir(parents=True, exist_ok=False)
    base = Path('data/experiments/20261009_p32_realtime_follow')
    scenes = {scene: load(list((base/folder/'shadow').glob('mpc*.jsonl')))
              for scene, folder in (('f2', 'f2_retry'), ('f3', 'f3_short'))}
    if args.phase == 'limits':
        # Single factors first, then all 12 joint combinations, with identical complete population.
        combinations = [(a, 1., 1.2, 'p41', 'none') for a in (3., 3.5, 4., 4.5)]
        combinations += [(3., y, 1.2, 'p41', 'none') for y in (1.25, 1.5)]
        combinations += [(a, y, 1.2, 'p41', 'none')
                         for a in (3., 3.5, 4., 4.5) for y in (1., 1.25, 1.5)
                         if (a, y) not in [(x[0], x[1]) for x in combinations]]
    elif args.phase == 'horizons':
        combinations = [(a, y, h, 'p43', 'none') for a, y in ((3., 1.), (3.5, 1.25))
                        for h in (.8, 1.2, 1.6, 2.4)]
    else:
        combinations = [(3., 1., h, e, r) for h in (1.2, 1.6) for e, r in
                        (('p41', 'none'), ('p43', 'none'), ('p43', 'q'), ('p43', 'qt'))]
    summaries = {}
    for a, y, h, engine, refine in combinations:
        for scene, cases in scenes.items():
            for rolling in (False, True):
                kind = 'rolling' if rolling else 'single'
                name = f'{scene}_a{a}_y{y}_h{h}_{engine}_{refine}_{kind}'
                summaries[name] = run_case(
                    cases, args.output/(name+'.jsonl'), a, y, h, rolling, engine, refine)
                (args.output/'summary.json').write_text(json.dumps(summaries, indent=2))
                s = summaries[name]
                print(name, s['count'], s['feasible'], s['errors'], s['seconds'], flush=True)


if __name__ == '__main__':
    main()

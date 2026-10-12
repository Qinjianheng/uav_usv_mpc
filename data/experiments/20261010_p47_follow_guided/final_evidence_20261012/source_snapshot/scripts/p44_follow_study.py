#!/usr/bin/env python3
"""Matched A-F frozen ablations; actual request clocks preserved, no execution authority."""
import argparse
import json
import time
import resource
from pathlib import Path

import p43_follow_study as study
from p4_follow_replay import load
from uav_control.controllers.p43_follow_solver import P43FollowSolver
from uav_control.controllers.p44_follow_solver import P44FollowSolver


def make(model, engine, horizon, now, refinement='none'):
    """One selector for the study; baseline stays original scalar P43."""
    cls = P43FollowSolver if engine == 'A' else P44FollowSolver
    extra = {} if engine == 'A' else dict(ablation=engine)
    return cls(model, duration=horizon, wall_clock=now, rolling=False,
               refinement=refinement, **extra)


def main():
    """Every independent configuration writes all inputs, attempts, failures and coefficients."""
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    study.prior.base.base._core()
    args.output.mkdir(exist_ok=False, parents=True)
    study.make = make  # Study-only factory injection; production selector is independent.
    base = Path('data/experiments/20261009_p32_realtime_follow')
    summary = {}
    for scene, folder in (('f2', 'f2_retry'), ('f3', 'f3_short')):
        cases = load(list((base/folder/'shadow').glob('mpc*.jsonl')))
        assert len(cases) == (118 if scene == 'f2' else 489)
        prior_root = Path('data/experiments/20261009_p43_follow_optimization')
        initial = json.loads((prior_root/f'common_initial/{scene}_initial.json').read_text())
        first = next(i for i, c in enumerate(cases) if c[2] == initial['sha']
                     and c[1] == initial['line'])
        for horizon in (1.2, 1.6):
            for engine, refine in [(e, 'none') for e in 'ABCDEF']+[('F', 'q'), ('F', 'qt')]:
                for rolling in (False, True):
                    kind = 'rolling' if rolling else 'single'
                    name = f'{scene}_h{horizon}_{engine}_{refine}_{kind}'
                    subset = cases[first:] if rolling else cases
                    cpu_start, wall_start = time.process_time(), time.perf_counter()
                    summary[name] = study.run_case(
                        subset, args.output/(name+'.jsonl'), horizon=horizon,
                        rolling=rolling, engine=engine, refinement=refine)
                    summary[name]['resources'] = dict(
                        cpu_seconds=time.process_time()-cpu_start,
                        wall_seconds=time.perf_counter()-wall_start,
                        cumulative_max_rss_mib=(
                            resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024))
                    (args.output/'summary.json').write_text(json.dumps(
                        study.prior.base.base.json_safe(summary), indent=2))
                    s = summary[name]
                    print(name, s['count'], s['feasible'], s['seconds'], flush=True)


if __name__ == '__main__':
    main()

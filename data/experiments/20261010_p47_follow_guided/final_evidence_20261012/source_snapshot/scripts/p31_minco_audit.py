#!/usr/bin/env python3
"""Reconstruct frozen P3 trajectories without amending their historical admission."""
import argparse
from collections import Counter, defaultdict
import cProfile
import json
from pathlib import Path
import pstats
import time
from unittest.mock import patch


import follow_minco_experiment as base


def main():
    """Record every F3 completion and a separately timed legacy profile."""
    base.base._core()
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.follow_minco_optimizer import FollowMincoOptimizer, MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.polynomial_extrema import derivative_peak
    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    parser.add_argument('output')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    records = [json.loads(line) for line in Path(args.input).read_text().splitlines()]
    counts = Counter()
    raw_assess = FollowProblem.assess
    with (output / 'per_cycle.jsonl').open('w') as stream:
        for record in records:
            if record.get('event') != 'completion' or not record.get('request'):
                continue
            request, model = base.restore_record(record)
            candidates = []

            def capture(self, times, p, v, a, j, yaw, rate):
                assessed = raw_assess(self, times, p, v, a, j, yaw, rate)
                margins, views, geometry, _ = assessed
                candidates.append(dict(time=float(times[0]), p=p[0].tolist(),
                                       v=v[0].tolist(), a=a[0].tolist(), yaw=float(yaw[0]),
                                       margins={k: float(x[0]) for k, x in margins.items()},
                                       fov=geometry[0].tolist(),
                                       visible=views[0].whole_target_safe))
                return assessed

            with patch.object(FollowProblem, 'assess', capture):
                seed = GreedyFollowInitializer(model).build(request)
            # Previous retained direction was not saved in P3 request; reconstruct zero,
            # never describe these candidates as historical worker internals.
            initial = FollowMincoOptimizer(MincoConfig(mode='fixed', budget=2.), model).solve(
                request, seed)
            original = record.get('core_result', record.get('output', {}))
            metrics = original.get('metrics', {})
            peaks = {}
            for name, value in (('initial', initial.metrics), ('historical_final', metrics)):
                if value.get('xyz_coefficients'):
                    peaks[name] = {kind: derivative_peak(value['xyz_coefficients'],
                                                         value['durations'], derivative, axes)
                                   for kind, derivative, axes in (
                                       ('horizontal_jerk', 3, (0, 1)),
                                       ('vertical_jerk', 3, (2,)),
                                       ('horizontal_acceleration', 2, (0, 1)))}
            final_valid = bool(original.get('valid'))
            category = ('A' if not seed.valid_input else 'B' if initial.valid and not final_valid
                        else 'C' if not initial.valid and final_valid else
                        'D' if not initial.valid and not final_valid else 'E')
            if original.get('solver_status') in ('TIMEOUT', 'DEADLINE_EXCEEDED'):
                category = 'F'
            counts[category] += 1
            stream.write(json.dumps(base.base.json_safe(dict(
                request=request, source_record=record, reconstructed_seed=seed,
                reconstruction_initial=initial, reconstructed_candidates=candidates,
                reconstruction_assumption=(
                    'previous direction=0; costs/retained historical unknown'),
                category=category, analytic_peaks=peaks)), allow_nan=False) + '\n')
    eligible = [r for r in records if r.get('event') == 'completion' and r.get('request')][:8]
    profiler = cProfile.Profile()
    timings = defaultdict(list)
    for record in eligible:
        request, model = base.restore_record(record)
        seed = GreedyFollowInitializer(model).build(request, method='simple')
        for mode in ('fixed', 'q', 'qt'):
            start = time.perf_counter()
            profiler.enable()
            FollowMincoOptimizer(MincoConfig(mode=mode, budget=3.), model).solve(request, seed)
            profiler.disable()
            timings[mode].append(time.perf_counter() - start)
    profiler.dump_stats(str(output / 'legacy.prof'))
    with (output / 'profile.txt').open('w') as stream:
        pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumtime').print_stats(70)
    stats = pstats.Stats(profiler)
    functions = {str(key): dict(calls=value[1], self_s=value[2], cumulative_s=value[3])
                 for key, value in stats.stats.items() if any(s in str(key) for s in (
                     'evaluate', 'approx_derivative', '_minimum_control_system', 'inv',
                     'control_effort', 'planned_attitude', 'evaluate_visibility', 'construct',
                     'assessment', 'line_search', 'sample', 'solve'))}
    (output / 'summary.json').write_text(json.dumps(dict(categories=counts,
                                                         profiled_seconds=timings,
                                                         functions=functions), indent=2))
    print(dict(categories=counts, profiled_seconds=timings))


if __name__ == '__main__':
    main()

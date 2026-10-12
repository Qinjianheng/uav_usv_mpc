#!/usr/bin/env python3
"""P3.2 same-input replay and initialization profile, never flight-control admission."""
import argparse
from collections import Counter
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import time

import numpy as np

import follow_minco_experiment as base


def load(path):
    """Keep every request-bearing completion and its exact source line/hash."""
    cases = []
    for line, text in enumerate(Path(path).read_text().splitlines(), 1):
        row = json.loads(text)
        if row.get('event') == 'completion' and row.get('request') is not None:
            cases.append((line, *base.restore_record(row)))
    return cases


def stats(values):
    """Unavailable measurements remain null, never zero-filled."""
    return dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(values, (50, 95, 99))))) if (
        values) else None


def run(path, output, profile=False):
    """Compare isolated changes and free terminal separately at original immutable epochs."""
    base.base._core()
    from uav_control.guidance.adaptive_follow_initializer import (
        AdaptiveFollowInitializer, GreedyConfig,
    )
    from uav_control.guidance.fast_follow_minco import FastFollowMinco, FastConfig
    from uav_control.guidance.follow_minco_optimizer import MincoConfig
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.guidance.start_jerk import adjust_start
    from uav_control.controllers.realtime_follow_solver import RealtimeFollowSolver
    out = base.base.validate_output_path(output)
    out.mkdir(exist_ok=False, parents=True)
    rows, profiles = [], {}
    variants = [
        ('p31', GreedyConfig(), None),
        ('batch', GreedyConfig(batch=True), None),
        ('two_level', GreedyConfig(batch=True, two_level=True), None),
        ('feasible_first', GreedyConfig(batch=True, feasible_first=True), None),
        ('bounded', GreedyConfig(batch=True, maximum_angles=3, maximum_distances=2), None),
        ('start_score', GreedyConfig(batch=True, start_score=True), None),
        ('time_allocation', GreedyConfig(batch=True), 'time'),
        ('q1', GreedyConfig(batch=True), 'q'),
        ('q1_t1', GreedyConfig(batch=True), 'combined'),
        ('full_initializer', GreedyConfig(
            batch=True, two_level=True, feasible_first=True, start_score=True,
            maximum_angles=3, maximum_distances=2),
         'combined'),
    ]
    if profile:
        variants += [('batch_compact', GreedyConfig(batch=True, detailed=False), None)]
    with (out/'records.jsonl').open('x') as stream:
        for number, request, model in load(path):
            simple = GreedyFollowInitializer(model).build(request, method='simple')
            boundaries = ('same_boundary', 'free_terminal') if not profile else ('free_terminal',)
            for boundary in boundaries:
                for label, config, adjustment in variants:
                    profiler = profiles.setdefault(label, cProfile.Profile())
                    if profile:
                        profiler.enable()
                    before = time.perf_counter()
                    seed = AdaptiveFollowInitializer(model, config).build(
                        request, common_endpoint=(
                            simple.end if boundary == 'same_boundary' else None))
                    if adjustment:
                        seed = adjust_start(seed, mode=adjustment)
                    init = time.perf_counter()-before
                    if profile:
                        profiler.disable()
                    ser = time.perf_counter()
                    serialized = json.dumps(base.base.json_safe(seed), allow_nan=False)
                    serialization = time.perf_counter()-ser
                    result = None if profile else FastFollowMinco(
                        MincoConfig(budget=.5, batch_validation=label != 'p31'), model,
                        FastConfig()).solve(request, seed)
                    row = base.base.json_safe(dict(
                        line=number, hash=base.base.fingerprint(request), variant=label,
                        comparison=boundary, seed=seed, result=result, initialization=init,
                        serialization=serialization, json_bytes=len(serialized),
                        elapsed=time.perf_counter()-before, accepted_by_tracker=False))
                    rows.append(row)
                    stream.write(json.dumps(row, allow_nan=False)+'\n')
                if not profile:
                    # Realistic advancing epoch, rather than resetting stamps after initialization.
                    for label in ('early_budget', 'full_p32'):
                        before = time.perf_counter()
                        solver = RealtimeFollowSolver(model, wall_clock=lambda: (
                            request.now_stamp+time.perf_counter()-before), detailed=True)
                        if label == 'early_budget':
                            # A single full initializer with the original budget computed first.
                            from uav_control.guidance.fast_follow_minco import freshness
                            _, available = freshness(request, request.now_stamp)
                            deadline = before+available-.02-.025
                            seed = AdaptiveFollowInitializer(model, GreedyConfig()).build(
                                request, common_endpoint=simple.end if boundary == 'same_boundary'
                                else None, deadline=deadline)
                            init = time.perf_counter()-before
                            result = FastFollowMinco(MincoConfig(
                                budget=max(1e-6, available-.02-init)), model).solve(request, seed)
                        elif boundary == 'free_terminal':
                            result = solver.solve(request)
                            init = result.timing['initialization']
                            seed = None
                        else:
                            # This comparison constrains every seed to one common endpoint.
                            seed = AdaptiveFollowInitializer(model, variants[-1][1]).build(
                                request, common_endpoint=simple.end)
                            seed = adjust_start(seed)
                            init = time.perf_counter()-before
                            from uav_control.guidance.fast_follow_minco import freshness
                            _, left = freshness(request, request.now_stamp+init)
                            cfg = MincoConfig(budget=max(1e-6, left-.02), batch_validation=True)
                            result = FastFollowMinco(cfg, model).solve(request, seed)
                        row = base.base.json_safe(dict(
                            line=number, hash=base.base.fingerprint(request), variant=label,
                            comparison=boundary, seed=seed, result=result, initialization=init,
                            elapsed=time.perf_counter()-before, accepted_by_tracker=False))
                        rows.append(row)
                        stream.write(json.dumps(row, allow_nan=False)+'\n')
            if number % 20 == 0:
                print('line', number, flush=True)
    summary = {}
    for boundary in sorted(set(r['comparison'] for r in rows)):
        summary[boundary] = {}
        for label in sorted(set(r['variant'] for r in rows)):
            group = [r for r in rows if r['comparison'] == boundary and r['variant'] == label]
            results = [r['result'] for r in group if r['result'] is not None]
            summary[boundary][label] = dict(
                count=len(group), feasible=sum(r['valid'] for r in results),
                status=dict(Counter(r['solver_status'] for r in results)),
                initialization=stats([r['initialization'] for r in group]),
                elapsed=stats([r['elapsed'] for r in group]),
                serialization=stats([r['serialization'] for r in group if 'serialization' in r]),
                candidate_count=stats([r['seed']['metrics'].get('candidate_count', len(
                    r['seed']['metrics'].get('candidates', []))) for r in group if r['seed']]),
                metric={key: stats([r['metrics'][key] for r in results if key in r['metrics']])
                        for key in ('follow_rmse', 'visible_fraction', 'minimum_horizontal_margin',
                                    'minimum_vertical_margin', 'maximum_horizontal_jerk',
                                    'jerk_integral')})
    (out/'summary.json').write_text(json.dumps(summary, indent=2))
    if profile:
        functions = {}
        for label, profiler in profiles.items():
            profiler.dump_stats(str(out/(label+'.pstats')))
            entries = []
            for (file, line, name), (_, calls, self_time, cumulative, _) in pstats.Stats(
                    profiler).stats.items():
                if 'uav_control' in file:
                    entries.append(dict(file=file, line=line, function=name, calls=calls,
                                        self_seconds=self_time, cumulative_seconds=cumulative))
            functions[label] = sorted(entries, key=lambda e: -e['cumulative_seconds'])
        (out/'functions.json').write_text(json.dumps(functions, indent=2))
    (out/'provenance.json').write_text(json.dumps(dict(
        input=str(Path(path).resolve()),
        sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        source=base.base._provenance(model.config, model), profile=profile), indent=2))
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    parser.add_argument('output')
    parser.add_argument('--profile', action='store_true')
    arguments = parser.parse_args()
    run(arguments.input, arguments.output, arguments.profile)

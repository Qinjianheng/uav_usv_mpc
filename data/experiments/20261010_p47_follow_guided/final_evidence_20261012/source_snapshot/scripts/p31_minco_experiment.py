#!/usr/bin/env python3
"""Sequential frozen-input P3.1 ablations; changed terminal/horizon rows are labelled."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.linalg import lu_factor, lu_solve, solve_banded

import follow_minco_experiment as base


def main():
    """Record pure offline results, independent gradients and small-system timings."""
    base.base._core()
    from uav_control.guidance.adaptive_follow_initializer import (
        AdaptiveFollowInitializer, GreedyConfig,
    )
    from uav_control.guidance.fast_follow_minco import FastFollowMinco, FastConfig
    from uav_control.guidance.fast_minco_objective import FastMincoObjective
    from uav_control.guidance.follow_minco_optimizer import (
        FollowMincoOptimizer, MincoConfig, _trajectory,
    )
    from uav_control.guidance.follow_problem import FollowProblem
    from uav_control.guidance.follow_reference import forecast_viewpoint
    from uav_control.guidance.greedy_follow_initializer import GreedyFollowInitializer
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed, MpcConfig
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    parser.add_argument('--input')
    parser.add_argument('--post-only', action='store_true')
    parser.add_argument('--maximum', type=int, default=20)
    parser.add_argument('--suite', choices=('solver', 'strategy', 'cost', 'fastpaths'),
                        default='solver')
    args = parser.parse_args()
    out = base.base.validate_output_path(args.output)
    out.mkdir(parents=True, exist_ok=args.post_only)
    if args.input:
        if not 1 <= args.maximum <= 100:
            raise ValueError('P31 audit limit must be 1..100')
        cases = []
        for number, line in enumerate(Path(args.input).read_text().splitlines(), 1):
            record = json.loads(line)
            if record.get('event') == 'completion' and record.get('request') is not None:
                cases.append(dict(line_number=number, record=record))
            if len(cases) == args.maximum:
                break
        requests = [(str(c['line_number']), *base.restore_record(c['record'])) for c in cases]
    else:
        requests = [(name, base.scenario(name), FollowMpcSeed(MpcConfig(
            allow_synthetic_predictions=True))) for name in base.SCENARIOS]
    records = ([json.loads(line) for line in (out/'records.jsonl').read_text().splitlines()]
               if args.post_only else [])
    with (out/'records.jsonl').open('a' if args.post_only else 'w') as stream:
        for name, request, model in ([] if args.post_only else requests):
            legacy = GreedyFollowInitializer(model).build(request)
            simple = GreedyFollowInitializer(model).build(request, method='simple')
            start = time.perf_counter()
            adaptive = AdaptiveFollowInitializer(model).build(request)
            adaptive_init_s = time.perf_counter()-start
            common_c = AdaptiveFollowInitializer(model, GreedyConfig('C')).build(
                request, common_endpoint=simple.end if simple.valid_input else None)
            common_d = AdaptiveFollowInitializer(model).build(
                request, common_endpoint=simple.end if simple.valid_input else None)
            dynamic = legacy
            if legacy.valid_input:
                p, v, a, _, _ = forecast_viewpoint(FollowProblem(request, model), 2.4)
                dynamic = replace(legacy, end=tuple(np.r_[p, v, a]))
            config = MincoConfig(budget=.5)
            fast = FastConfig()
            variants = [
                ('A_legacy', legacy, config, None, 'same_boundary'),
                ('B_priority', legacy, replace(config, feasible_priority=True),
                 None, 'same_boundary'),
                ('C_dynamic_end', dynamic, config, None, 'changed_terminal'),
                ('D_adaptive_angle', common_c, config, None, 'same_boundary'),
                ('E_adaptive_distance', common_d, config, None, 'same_boundary'),
                ('F_adaptive_terminal', adaptive, config, None, 'changed_terminal'),
                ('G_gradient', legacy, config, replace(fast, fast_feasible_seed=False,
                                                       exact_precheck=False), 'same_boundary'),
                ('H_fast_validation', legacy, config, fast, 'same_boundary'),
                ('I_all', adaptive, config, fast, 'changed_terminal'),
                ('simple_fast', simple, config, fast, 'same_boundary'),
            ]
            if args.suite == 'fastpaths':
                both = replace(config, feasible_priority=True, fast_feasible_seed=True)
                variants = [variants[0], variants[1],
                            ('J_legacy_fast', legacy, replace(config, fast_feasible_seed=True),
                             None, 'same_boundary'),
                            ('K_legacy_both', legacy, both, None, 'same_boundary'),
                            ('L_both_dynamic', dynamic, both, None, 'changed_terminal'),
                            ('M_original_mpc', simple, config, None, 'old_mpc_interface')]
            if args.suite == 'strategy':
                variants = [(strategy, AdaptiveFollowInitializer(
                    model, GreedyConfig(strategy)).build(request), config, fast,
                    'changed_terminal') for strategy in ('A', 'B', 'C', 'D')]
                variants += [('D_horizon_2.8', AdaptiveFollowInitializer(
                    model, GreedyConfig(duration=2.8)).build(request), config, fast,
                              'changed_horizon')]
            if args.suite == 'cost':
                common = replace(fast, fast_feasible_seed=False, exact_precheck=False)
                variants = [(label, simple, cfg, f, 'same_boundary') for label, cfg, f in (
                    ('jerk_dynamic', replace(config, follow_weight=0, visibility_weight=0),
                     common),
                    ('plus_follow', replace(config, visibility_weight=0), common),
                    ('horizontal', config, replace(common, visibility_mode='horizontal')),
                    ('full', config, replace(common, yaw_optimize=False)),
                    ('joint_yaw', config, common),
                    ('visibility_scale_2', replace(config, visibility_weight=2), common))]
            for label, seed, cfg, f, comparison in variants:
                before = time.perf_counter()
                optimizer = (FollowMincoOptimizer(cfg, model) if f is None else
                             FastFollowMinco(cfg, model, f))
                result = (model.solve(request) if label == 'M_original_mpc' else
                          optimizer.solve(request, seed))
                record = dict(case=name, variant=label, comparison=comparison,
                              request_sha256=base.base.fingerprint(request), request=request,
                              seed=seed, result=result, elapsed=time.perf_counter()-before,
                              adaptive_initialization_s=adaptive_init_s, accepted_by_tracker=False)
                record = base.base.json_safe(record)
                records.append(record)
                stream.write(json.dumps(record, allow_nan=False)+'\n')
            print(name, flush=True)
    summary = {}
    for label in sorted(set(r['variant'] for r in records)):
        rows = [r for r in records if r['variant'] == label]
        summary[label] = dict(count=len(rows), feasible=sum(r['result']['valid'] for r in rows),
                              initialized=sum(r['seed']['valid_input'] for r in rows),
                              status=Counter(r['result']['solver_status'] for r in rows),
                              seconds=base.base.percentiles([r['elapsed'] for r in rows]),
                              adaptive_initialization_seconds=base.base.percentiles([
                                 r['adaptive_initialization_s'] for r in rows]),
                              optimizer_calls=sum(r['result']['metrics'].get('fast', {}).get(
                                 'optimization_calls', 0) for r in rows))
    (out/'summary.json').write_text(json.dumps(summary, indent=2))
    provenance = dict(head=base.base._provenance(requests[0][2].config, requests[0][2]),
                      suite=args.suite, input=args.input, actual_cases=len(requests))
    (out/'provenance.json').write_text(json.dumps(provenance, indent=2))
    if not args.input:
        gradients = []
        for name in ('constant_velocity', 'right_turn', 'figure_eight', 'fov_edge', 'wrong_yaw'):
            request = base.scenario(name)
            model = FollowMpcSeed(MpcConfig(allow_synthetic_predictions=True))
            seed = GreedyFollowInitializer(model).build(request, method='simple')
            objective = FastMincoObjective(FollowProblem(request, model), seed, MincoConfig())
            x = objective.initial()
            x[6:8] = (.12, -.2)
            _, grad = objective(x)
            numeric = []
            for i in range(len(x)):
                left, right = x.copy(), x.copy()
                left[i] -= 1e-5
                right[i] += 1e-5
                numeric.append((objective(right)[0]-objective(left)[0])/2e-5)
            absolute = abs(grad-numeric)
            gradients.append(dict(case=name, analytic=grad.tolist(), central=numeric,
                                  maximum_abs=float(max(absolute)),
                                  maximum_scaled_relative=float(max(absolute/np.maximum(
                                      1., np.maximum(abs(grad), abs(np.asarray(numeric))))))))
        (out/'gradients.json').write_text(json.dumps(gradients, indent=2))
        trajectory = _trajectory(seed, seed.q, seed.durations)
        m, rhs = trajectory._minimum_control_system(
            *[seed.start[i:i+3] for i in (0, 3, 6)],
            *[seed.end[i:i+3] for i in (0, 3, 6)], seed.q)
        nz = np.nonzero(m)
        lower, upper = int(max(nz[0]-nz[1])), int(max(nz[1]-nz[0]))
        band = np.zeros((lower+upper+1, 18))
        band[upper+nz[0]-nz[1], nz[1]] = m[nz]
        methods = {'inverse_uncached': lambda: np.linalg.inv(m)@rhs,
                   'numpy_solve': lambda: np.linalg.solve(m, rhs),
                   'scipy_lu': lambda: lu_solve(lu_factor(m), rhs),
                   'banded': lambda: solve_banded((lower, upper), band, rhs)}
        inverse = np.linalg.inv(m)
        factor = lu_factor(m)
        methods.update(inverse_cached=lambda: inverse@rhs,
                       lu_cached=lambda: lu_solve(factor, rhs))
        bench = {}
        for name, call in methods.items():
            times = []
            for _ in range(300):
                start = time.perf_counter()
                coefficients = call()
                times.append(time.perf_counter()-start)
            bench[name] = dict(seconds=base.base.percentiles(times),
                               residual=float(np.max(abs(m@coefficients-rhs))))
        (out/'matrix.json').write_text(json.dumps(bench, indent=2))
    paths = [p for p in out.rglob('*') if p.is_file()]
    (out/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest() +
                                          '  '+str(p.relative_to(out))+'\n' for p in paths))
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()

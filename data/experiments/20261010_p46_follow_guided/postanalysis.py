"""Recompute paired windows and initial-failure causes from the new immutable run."""
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import platform
import sys
from types import SimpleNamespace

import numpy as np
import scipy

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS/'scripts'))
from p46_follow_study import matched, distribution, safe  # noqa: E402
from p4_follow_replay import load  # noqa: E402
from uav_control.guidance.follow_minco_optimizer import _trajectory  # noqa: E402
from uav_control.guidance.follow_problem import FollowProblem  # noqa: E402
from uav_control.guidance.follow_rollout import (  # noqa: E402
    rollout_follow, FollowRolloutConfig,
)
from uav_control.guidance.polynomial_extrema import derivative_peak  # noqa: E402

ROOT = Path(__file__).resolve().parent
RUN = ROOT/'comparison'


def rows(path):
    for line in path.open():
        yield json.loads(line)


def main():
    report = dict(environment=dict(python=sys.version, numpy=np.__version__, scipy=scipy.__version__,
                                   platform=platform.platform(), blas_threads=1),
                  paired_fixed={}, failures={}, convergence={})
    for scene in ('f2', 'f3'):
        for horizon in (1.2, 1.6):
            key = f'{scene}_h{horizon}'
            paired = {}
            for group in ('A', 'B'):
                paired[group] = [{k: r[k] for k in (
                    'line', 'source_sha256', 'epoch', 'error', 'sampled_state', 'continuous_age')}
                    for r in rows(RUN/f'{key}_{group}_rolling.jsonl')]
            report['paired_fixed'][key] = matched(paired)
            status, causes, peaks, examples = Counter(), Counter(), [], []
            for r in rows(RUN/f'{key}_B_single.jsonl'):
                d = r['result']['metrics']['follow_guided']
                status[d.get('initial_status', r['result']['solver_status'])] += 1
                e = r.get('evaluation') or {}
                if d.get('initial_status') != 'DYNAMIC_INFEASIBLE' or 'initial_seed' not in e:
                    continue
                seed, limits = e['initial_seed'], r['model_config']['mpc']
                curve = _trajectory(SimpleNamespace(start=seed['start'], end=seed['end']),
                                    seed['q'], seed['durations'])
                values = {}
                for name, derivative, axes in (
                        ('horizontal_speed', 1, (0, 1)), ('vertical_speed', 1, (2,)),
                        ('horizontal_acceleration', 2, (0, 1)),
                        ('vertical_acceleration', 2, (2,)),
                        ('horizontal_jerk', 3, (0, 1)), ('vertical_jerk', 3, (2,))):
                    value = derivative_peak(curve.coefficients, seed['durations'],
                                            derivative, axes)['value']
                    values[name] = value
                    if value > limits['maximum_'+name]+1e-6:
                        causes[name] += 1
                peaks.append(values['horizontal_jerk'])
                if len(examples) < 3:
                    examples.append(dict(source_line=r['line'], final_status=r['result'][
                        'solver_status'], independent=d['independent_valid'],
                        fallback=d['fallback_used'], peaks=values, initial_fit=d['initial_fit']))
            rolling = list(rows(RUN/f'{key}_B_rolling.jsonl'))
            stopped = next((r for r in rolling if r['stop_reason']), None)
            report['failures'][key] = dict(initial_statuses=dict(status),
                                           initial_dynamic_causes=dict(causes),
                                           rejected_seed_horizontal_jerk=distribution(peaks),
                                           examples=examples,
                                           first_stopped_source_line=stopped['line'] if stopped
                                           else None)
        folder = 'f2_retry' if scene == 'f2' else 'f3_short'
        cases = load(list((WS/'data/experiments/20261009_p32_realtime_follow'/folder/'shadow')
                          .glob('mpc_seed_shadow*.jsonl')))
        checks = []
        for _, line, sha, req, model in cases:
            try:
                problem = FollowProblem(req, model, 1.2)
                coarse = rollout_follow(problem)
                fine = rollout_follow(problem, FollowRolloutConfig(integration_dt=.005))
                wanted = [fine.sample(t) for t in coarse.times]
                differences = []
                for i, field in enumerate((coarse.positions, coarse.velocities,
                                            coarse.accelerations)):
                    delta = np.asarray(field)-np.asarray([s[i] for s in wanted])
                    differences.append(float(np.linalg.norm(delta, axis=1).max()))
                checks.append(dict(source_line=line, line_sha256=sha, maximum_pva=differences))
                if len(checks) == 10:
                    break
            except ValueError:
                continue
        report['convergence'][scene] = dict(count=len(checks), cases=checks,
                                            maximum_pva=np.max([c['maximum_pva'] for c in checks],
                                                               axis=0).tolist(),
                                            settings=asdict(FollowRolloutConfig()))
    with (ROOT/'postanalysis.json').open('x') as stream:
        json.dump(safe(report), stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()

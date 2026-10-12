#!/usr/bin/env python3
"""Fingerprint/reconstruct saved flights and profile three exact P5 inputs, without SITL."""
import argparse
from collections import Counter
import cProfile
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import pstats
import time

import numpy as np

from follow_minco_experiment import restore_record
from p47_follow_study import make
from p47_execution_timeline import clean, reconstruct, stats

WORKSPACE = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(output):
    """Use original producer clocks for handover delay, monotonic clocks for profiling."""
    output.mkdir(parents=True, exist_ok=False)
    sources, history = {}, {}
    for name in ('b_f2_1', 'b_f2_2', 'c_f2_1', 'c_f2_2'):
        path = (WORKSPACE/'data/experiments/20261010_p45_minco_execution'
                / name/'offline_evidence.jsonl')
        sources[str(path)] = digest(path)
        rows = [json.loads(line) for line in path.open()]
        result = reconstruct(rows)
        history[name] = {k: result[k] for k in
                         ('direct_causes', 'upstream_update_states', 'cadence', 'lifecycle')}
        (output/(name+'_timeline.json')).write_text(json.dumps(clean(result), indent=2)+'\n')
        ack_by_plan = {}
        for row in rows:
            if row.get('kind') == 'follow_ack':
                ack_by_plan.setdefault(row['plan'], {})[row['state']] = row
        delays = []
        for row in rows:
            if row.get('kind') == 'proposal':
                ack = ack_by_plan.get(row['plan'], {})
                if 'ACTIVE' in ack:
                    delays.append(ack['ACTIVE']['stamp']-row['stamp'])
        history[name]['publication_to_active_seconds'] = stats(delays)
    run = WORKSPACE/'data/experiments/20261012_p5_revalidation_02/f2_h16_01'
    path = next((run/'shadow').glob('mpc_seed_shadow_*.jsonl'))
    sources[str(path)] = digest(path)
    requests = []
    for line, text in enumerate(path.open(), 1):
        row = json.loads(text)
        if row.get('event') != 'completion':
            continue
        request, model = restore_record(row)
        started = [0.]

        def wall():
            return request.now_stamp+time.perf_counter()-started[0]

        solver = make(model, dict(engine='p47', terminal_mode='qva', refinement=False,
                                  scheduling='adaptive'), 1.6, wall)
        profile = cProfile.Profile()
        started[0] = time.perf_counter()
        result = profile.runcall(solver.solve, request)
        elapsed = time.perf_counter()-started[0]
        stream = io.StringIO()
        pstats.Stats(profile, stream=stream).sort_stats('cumulative').print_stats(30)
        (output/f'cycle_{request.context.cycle_id}_profile.txt').write_text(stream.getvalue())
        (output/f'cycle_{request.context.cycle_id}_replay.json').write_text(
            json.dumps(clean(asdict(result)), indent=2)+'\n')
        c = request.context
        until = min(c.navigation_stamp+.125, c.observation_stamp+.125,
                    c.prediction_source_stamp+.125, c.prediction_valid_until)
        requests.append(dict(cycle=c.cycle_id, source=str(path), line=line,
                             line_sha256=hashlib.sha256(text.encode()).hexdigest(),
                             original_context=asdict(c), dispatch_stamp=request.now_stamp,
                             original_compute_budget=until-request.now_stamp-.02,
                             original_timing=row['candidate_output']['timing'],
                             original_profile=row['candidate_output']['metrics']['cycle_profile'],
                             profiled_elapsed=elapsed, replay_valid=result.valid,
                             replay_status=result.solver_status, replay_timing=result.timing,
                             caution='instrumented exact-input replay; three startup requests, '
                             'not an online latency percentile population'))
    combined = Counter()
    for result in history.values():
        combined.update(result['upstream_update_states'])
    summary = dict(no_new_sitl=True, historical=history,
                   historical_upstream=dict(combined), exact_request_profiles=requests,
                   sources=sources, clocks='ROS/PX4 physical epochs for ages; perf_counter '
                   'for processing durations; never subtract the two domains')
    (output/'analysis.json').write_text(json.dumps(clean(summary), indent=2)+'\n')
    assert all(digest(Path(p)) == value for p, value in sources.items())
    plot(output, requests)
    return summary


def plot(output, requests):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), constrained_layout=True)
    for r in requests:
        value = json.loads((output/f"cycle_{r['cycle']}_replay.json").read_text())
        if not value['valid']:
            continue
        t, p = np.asarray(value['relative_times']), np.asarray(value['positions'])
        axes[0].plot(t, p[:, 2], label=f"saved request {r['cycle']}")
        axes[1].plot(t, np.linalg.norm(p[:, :2]-p[0, :2], axis=1), label=str(r['cycle']))
    axes[0].axhline(-5., color='black', linestyle='--', label='configured NED altitude')
    axes[0].set(ylabel='reference z [m NED]', title='Exact-input offline replay; no new flight')
    axes[1].set(xlabel='curve relative time [s]', ylabel='horizontal displacement [m]')
    axes[0].legend()
    fig.savefig(output/'replay_reference.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    result = analyze(parser.parse_args().output.resolve())
    excluded = ('sources', 'historical', 'exact_request_profiles')
    print(json.dumps(clean({k: v for k, v in result.items() if k not in excluded})))

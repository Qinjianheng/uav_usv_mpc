#!/usr/bin/env python3
"""Matched actual requests and recorded worker-start delay; no zero-cost solver clock."""
import argparse
import copy
from dataclasses import asdict
import json
from pathlib import Path
import time

from mpc_seed_replay import restore_case
from p5_release import verify_manifest
from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
from uav_control.guidance.follow_minco_execution import request_from_payload


def restore_actual(record):
    """Restore model from recorded calibration and keep the full accepted boundary."""
    stripped = copy.deepcopy(record)
    for key in ('measurement_state', 'boundary_policy', 'prior_curve', 'reference_yaw_rate'):
        stripped['request'].pop(key, None)
    _, model = restore_case(stripped)
    request = request_from_payload(record['request'])
    assert json.loads(json.dumps(asdict(request))) == record['request']
    return request, model


def replay(root, output, cycles, fallback=True):
    verify_manifest(root)
    output.mkdir(parents=True, exist_ok=False)
    records = [(path, n, line, json.loads(line))
               for path in (root/'shadow').glob('mpc_seed_shadow_*.jsonl')
               for n, line in enumerate(path.open(), 1)]
    report = []
    for source, line_number, line, record in records:
        if record.get('event') != 'completion' or not record.get('request'):
            continue
        cycle = record['request']['context']['cycle_id']
        if cycle not in cycles:
            continue
        request, model = restore_actual(record)
        metrics = record['candidate_output']['metrics']
        budget = metrics['follow_guided']['budget']
        delay = metrics['worker_timeline']['solve_start_ros']-request.now_stamp
        started = [0.]
        solver = P47FollowGuidedMinco(
            model, duration=1.6, terminal_mode='qva', refinement=False,
            scheduling='adaptive', online_budget_guard=True, fallback_enabled=fallback,
            wall_clock=lambda: request.now_stamp+delay+time.perf_counter()-started[0])
        solver.follow_estimate = budget['follow_estimate']
        solver.fallback_estimate = budget['fallback_estimate']
        started[0] = time.perf_counter()
        value = solver.solve(request)
        elapsed = time.perf_counter()-started[0]
        report.append(dict(cycle=cycle, source=str(source), line=line_number,
                           line_sha256=sha_line(line), recorded_worker_start_delay=delay,
                           elapsed=elapsed, original_status=record['output']['solver_status'],
                           status=value.solver_status, valid=value.valid,
                           fallback_enabled=fallback,
                           scope='Current wall cost; counterfactual if fallback disabled. '
                                 'Original input/epochs/parent; no actual supply claim'))
        (output/f'cycle_{cycle}.json').write_text(json.dumps(asdict(value), indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def sha_line(line):
    import hashlib
    return hashlib.sha256(line.encode()).hexdigest()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--cycles', type=int, nargs='+', required=True)
    parser.add_argument('--without-fallback', action='store_true')
    args = parser.parse_args()
    print(json.dumps(replay(args.root, args.output, args.cycles, not args.without_fallback)))

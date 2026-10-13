#!/usr/bin/env python3
"""Current guarded solver on the three saved P5 requests; no flight or TTL rewriting."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

from follow_minco_experiment import restore_record
from p47_execution_timeline import clean
from uav_control.controllers.follow_guided_feasible import P47FollowGuidedMinco
from uav_control.guidance.follow_terminal_correction import linear_geometry

WORKSPACE = Path(__file__).resolve().parents[1]


def analyze(output):
    output.mkdir(parents=True, exist_ok=False)
    source = next((WORKSPACE/'data/experiments/20261012_p5_revalidation_02/f2_h16_01/shadow')
                  .glob('mpc_seed_shadow_*.jsonl'))
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    records = [(i, text, json.loads(text)) for i, text in enumerate(source.open(), 1)
               if json.loads(text).get('event') == 'completion']
    comparisons = []
    for guarded in (False, True):
        linear_geometry.cache_clear()
        solver, current, started = None, None, [0.]

        def wall():
            return current.now_stamp+time.perf_counter()-started[0]

        for line, text, row in records:
            current, model = restore_record(row)
            construction = 0.
            if solver is None:
                init = time.perf_counter()
                solver = P47FollowGuidedMinco(
                    model, duration=1.6, terminal_mode='qva', refinement=False,
                    scheduling='adaptive', online_budget_guard=guarded, wall_clock=wall)
                construction = time.perf_counter()-init
            started[0] = time.perf_counter()
            result = solver.solve(current)
            elapsed = time.perf_counter()-started[0]
            c = current.context
            ttl = min(c.navigation_stamp+.125, c.attitude_stamp+.125,
                      c.observation_stamp+.125, c.prediction_source_stamp+.125,
                      c.prediction_valid_until)-current.now_stamp
            comparisons.append(dict(
                guarded=guarded, cycle=c.cycle_id, line=line,
                line_sha256=hashlib.sha256(text.encode()).hexdigest(),
                request_context=asdict(c), dispatch=current.now_stamp,
                raw_ttl_at_dispatch=ttl, construction_wall_seconds=construction,
                observed_wall_seconds=elapsed, remaining_raw_ttl=ttl-elapsed,
                valid=result.valid, status=result.solver_status, timing=result.timing,
                metrics=result.metrics, original_status=row['output']['solver_status'],
                note='Exact saved startup request, original epochs. Three cases cannot '
                     'establish online percentiles or actual admission.'))
            (output/f"{'guarded' if guarded else 'legacy'}_cycle_{c.cycle_id}.json").write_text(
                json.dumps(clean(asdict(result)), indent=2)+'\n')
    assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
    report = dict(no_sitl=True, source=str(source), source_sha256=digest,
                  source_unchanged=True, exact_request_comparisons=comparisons,
                  causal_limit='Saved events do not prove CPU/executor contention or online '
                  'steady-state successor rate. Producer and observer delays remain separate.')
    (output/'analysis.json').write_text(json.dumps(clean(report), indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    report = analyze(parser.parse_args().output.resolve())
    print(json.dumps([dict(guarded=r['guarded'], cycle=r['cycle'], valid=r['valid'],
                           status=r['status'], wall_ms=1000*r['observed_wall_seconds'])
                      for r in report['exact_request_comparisons']], indent=2))

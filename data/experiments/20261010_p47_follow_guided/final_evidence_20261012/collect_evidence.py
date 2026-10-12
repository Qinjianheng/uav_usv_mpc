"""Assemble P47 reporting evidence from completed immutable formal outputs."""
import json
import sys
from collections import Counter
from pathlib import Path
import hashlib
import shutil
import numpy as np

WS = Path('/home/qin/data/uav_usv_mpc')
ROOT = WS/'data/experiments/20261010_p47_follow_guided'
FORMAL = ROOT/'comparison/formal_20261012_2'
OUT = ROOT/'final_evidence_20261012'


def read_rows(path):
    return [json.loads(line) for line in path.open()]


def write_json(path, payload):
    path.write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')


def main():
    summary = json.loads((FORMAL/'summary.json').read_text())
    manifest = json.loads((FORMAL/'manifest.json').read_text())
    assert len(summary['configurations']) == 104
    assert all(v['all_identical'] for v in summary['same_input_checks'].values())
    OUT.mkdir(exist_ok=False)
    source_ok = {p: hashlib.sha256((WS/p).read_bytes()).hexdigest() == sha
                 for p, sha in manifest['source_version']['sha256'].items()}
    assert all(source_ok.values())
    write_json(OUT/'source_after.json', source_ok)
    for relative in source_ok:
        target = OUT/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(WS/relative, target)
    for horizon in (1.2, 1.6):
        historical = read_rows(ROOT/f'derivative_diagnosis/f3_h{horizon}.jsonl')
        population = {r['line'] for r in historical if r['original_p46_horizontal_jerk_population']}
        result = dict(horizon=horizon, historical_population=len(population), groups={})
        for arm in ('C', 'D'):
            raw = read_rows(FORMAL/f'f3_h{horizon}_main_{arm}_single.jsonl')
            analysis = read_rows(FORMAL/f'f3_h{horizon}_main_{arm}_single_analysis.jsonl')
            selected = [(r, a) for r, a in zip(raw, analysis) if r['line'] in population]
            counts = Counter()
            witnesses = []
            for row, diagnostic in selected:
                detail = row['result']['metrics']['follow_guided']
                counts['population'] += 1
                counts['boundary_attempted'] += diagnostic['boundary_candidate'] is not None
                counts['independent_attempted'] += diagnostic['independent_candidate'] is not None
                candidate = diagnostic['independent_candidate']
                counts['horizontal_jerk_resolved'] += bool(candidate and
                    candidate['violations']['horizontal_jerk'] <= 1e-6)
                counts['independent_valid'] += detail['independent_valid']
                counts['fallback_used'] += detail['fallback_used']
                counts['final_valid'] += row['result']['valid']
                witnesses.append(dict(line=row['line'], source_sha256=row['source_sha256'],
                                      detail=detail, analysis=diagnostic))
            result['groups'][arm] = dict(counts)
            write_json(OUT/f'historical_jerk_h{horizon}_{arm}_witness.json', witnesses)
        write_json(OUT/f'historical_jerk_h{horizon}_summary.json', result)
    # All method comparisons use immutable line SHA + epoch, not their own windows.
    write_json(OUT/'matched_rolling_acd.json', summary['matched_rolling_acd'])
    repairs = {}
    margins = {}
    for scene in ('f2', 'f3'):
        for horizon in (1.2, 1.6):
            for mode in ('single', 'rolling'):
                counts = Counter()
                rows = read_rows(FORMAL/f'{scene}_h{horizon}_main_D_{mode}.jsonl')
                for row in rows:
                    if not row.get('result'):
                        continue
                    detail = row['result']['metrics'].get('follow_guided', {})
                    correction = detail.get('terminal_correction', {})
                    repair = correction.get('repair_candidate')
                    counts['attempted'] += bool(detail.get('optimization_attempted'))
                    counts['returned'] += repair is not None
                    counts['retained'] += bool(repair and repair['retained'])
                    counts['strict_valid'] += bool(repair and repair['validation']['valid'])
                    counts[detail.get('refinement_status', 'NOT_ATTEMPTED')] += 1
                repairs[f'{scene}_h{horizon}_{mode}'] = dict(counts)
                for arm in ('C', 'D'):
                    observed = read_rows(FORMAL/f'{scene}_h{horizon}_main_{arm}_{mode}.jsonl')
                    values = [r['result']['metrics']['follow_guided']['budget'][
                        'remaining_seconds_at_finish'] for r in observed
                        if r.get('result') and r['result']['valid']]
                    margins[f'{scene}_h{horizon}_{arm}_{mode}'] = dict(
                        valid_count=len(values), minimum_seconds=min(values),
                        quantiles_seconds=np.percentile(values, (5, 50, 95)).tolist(),
                        publication_reserve_seconds=.02,
                        note='Solver-side remaining, not measured receiver end-to-end margin')
    write_json(OUT/'repair_outcomes.json', repairs)
    write_json(OUT/'solver_freshness_margins.json', margins)
    # Preserve the concrete regression witness that prompted the initialization revision.
    old = read_rows(ROOT/'comparison/formal_20261012_1/f3_h1.2_main_C_single.jsonl')
    old_analysis = read_rows(ROOT/'comparison/formal_20261012_1/f3_h1.2_main_C_single_analysis.jsonl')
    new = read_rows(FORMAL/'f3_h1.2_main_C_single.jsonl')
    new_analysis = read_rows(FORMAL/'f3_h1.2_main_C_single_analysis.jsonl')
    witness = {}
    for label, rows, diagnostics in (('first', old, old_analysis), ('second', new, new_analysis)):
        row, analysis = next((r, a) for r, a in zip(rows, diagnostics) if r['line'] == 436)
        witness[label] = dict(raw_request=row['raw_request'], request=row['request'],
                              source_sha256=row['source_sha256'], line=row['line'],
                              correction=row['result']['metrics']['follow_guided']['terminal_correction'],
                              analysis=analysis)
    assert witness['first']['request'] == witness['second']['request']
    write_json(OUT/'initializer_regression_witness_line436.json', witness)
    # Machine-readable main tables, including unrounded values.
    write_json(OUT/'main_tables.json', {k: v for k, v in summary['configurations'].items()
                                      if '_main_' in k})
    shutil.copyfile(__file__, OUT/'collect_evidence.py')
    print('created', OUT)
    print('source hashes unchanged', len(source_ok))


if __name__ == '__main__':
    main()

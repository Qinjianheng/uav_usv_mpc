#!/usr/bin/env python3
"""Re-audit frozen P47 evidence without solving; qualify only the H1.6 candidate."""
import argparse
import ast
import hashlib
import json
from pathlib import Path

from follow_minco_experiment import restore_record
from p46_follow_study import distribution, matched, summarize
from uav_control.guidance.follow_fast_validation import assess_curve
from uav_control.guidance.follow_limits import constraint_snapshot
from uav_control.guidance.follow_problem import HypotheticalRequest
from uav_control.guidance.follow_contract import FollowCurve

WORKSPACE = Path(__file__).resolve().parents[1]
P47 = WORKSPACE/'data/experiments/20261010_p47_follow_guided'
CANDIDATE = dict(engine='p47_follow_guided', horizon=1.6, terminal_mode='qva',
                 refinement=False, scheduling='adaptive')


def sha(path):
    """Stream large evidence; do not allocate whole JSONL files."""
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def verify_manifest(root):
    """Reject missing, changed or out-of-directory evidence before consuming any metrics."""
    root = Path(root).resolve()
    entries = []
    for line in (root/'SHA256SUMS').read_text().splitlines():
        digest, relative = line.split(maxsplit=1)
        path = (root/relative.lstrip('*')).resolve()
        if not path.is_relative_to(root) or sha(path) != digest:
            raise ValueError('EVIDENCE_HASH_MISMATCH: '+relative)
        entries.append(dict(path=str(path.relative_to(root)), sha256=digest,
                            bytes=path.stat().st_size))
    if not entries:
        raise ValueError('EMPTY_MANIFEST')
    return entries


def semantic_tree(path):
    """Ignore docstrings only; all executable AST differences invalidate replay equivalence."""
    tree = ast.parse(Path(path).read_text())
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.body and isinstance(node.body[0], ast.Expr) and isinstance(
                    node.body[0].value, ast.Constant) and isinstance(
                        node.body[0].value.value, str):
                node.body.pop(0)
    return ast.dump(tree, include_attributes=False)


def qualify(evidence):
    """Offline permission to attempt research is distinct from real receiver admission."""
    gates = dict(candidate=all(evidence.get(k) == v for k, v in CANDIDATE.items()),
                 integrity=evidence.get('integrity') is True,
                 source_equivalent=evidence.get('source_equivalent') is True,
                 counterexample=evidence.get('h12_counterexample_retained') is True)
    scenes = evidence.get('scenes', {})
    for scene in ('f2', 'f3'):
        v = scenes.get(scene, {})
        try:
            gates[scene] = bool(
                v['same_inputs'] and v['candidate_total'] >= v['baseline_total']
                and v['independent']+v['fallback'] == v['candidate_total']
                and 0 < v['candidate_total'] <= v['attempted']
                and v['steady_count'] > 0 and v['candidate_xy'] < v['baseline_xy']
                and v['full_coverage'] and v['validation_failures'] == 0
                and v['revalidated'] >= v['candidate_total']
                and 0 <= v['height_rmse'] < .1 and 0 < v['p95'] <= v['p99'] <= .025
                and v['minimum_margin_after_reserve'] > 0)
        except (KeyError, TypeError):
            gates[scene] = False
    passed = all(gates.values())
    return dict(offline_candidate_pass=passed, h16_scoped_gate_pass=passed, gates=gates,
                old_all_horizon_gate_changed=False, sitl_validated=False,
                receiver_margin_measured=False, production_qualified=False,
                scope='H1.6 C, ideal rolling + solver TTL after 20ms reserve only',
                runtime_admission_required=True)


def audit(formal, output):
    """Recompute selected metrics and fully revalidate accepted A/C curves, never optimize."""
    formal, output = Path(formal).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    entries = verify_manifest(formal)
    final = P47/'final_evidence_20261012'
    final_entries = verify_manifest(final)
    manifest = json.loads((formal/'manifest.json').read_text())
    recorded = json.loads((formal/'summary.json').read_text())
    if manifest['arms']['main']['C'] != dict(engine='p47', terminal_mode='qva',
                                             refinement=False, scheduling='adaptive'):
        raise ValueError('FORMAL_CANDIDATE_MISMATCH')
    versions = manifest['source_version']['sha256']
    source_checks = []
    for relative, digest in versions.items():
        snapshot = final/'source_snapshot'/relative
        current = WORKSPACE/relative
        if sha(snapshot) != digest:
            raise ValueError('SNAPSHOT_HASH_MISMATCH: '+relative)
        exact = sha(current) == digest
        equivalent = exact or (current.suffix == '.py' and
                               semantic_tree(current) == semantic_tree(snapshot))
        source_checks.append(dict(path=relative, recorded_sha256=digest,
                                  current_sha256=sha(current), equivalent=equivalent))
    inputs = []
    for scene, source in manifest['inputs'].items():
        for path, digest in source['sources'].items():
            if sha(path) != digest:
                raise ValueError('INPUT_HASH_MISMATCH: '+path)
        inputs.append(dict(scene=scene, **source))
    evidence = dict(CANDIDATE, integrity=True,
                    source_equivalent=all(v['equivalent'] for v in source_checks), scenes={},
                    h12_counterexample_retained=False)
    compact, matched_rows = {}, []
    for scene in ('f2', 'f3'):
        groups = {}
        totals = {}
        validations = []
        margins = []
        reference_height_errors = []
        input_checks = []
        for mode in ('single', 'rolling'):
            populations = {}
            raw_populations = []
            for arm in ('A', 'C', 'D') if mode == 'rolling' else ('A', 'C'):
                name = f'{scene}_h1.6_main_{arm}_{mode}'
                rows = [json.loads(line) for line in (formal/(name+'.jsonl')).open()]
                raw_populations.append([(r['source_sha256'], r['line'], r['raw_request'],
                                         r['constraint_fingerprint']) for r in rows])
                origin, end = rows[0]['epoch'], rows[-1]['epoch']
                report = summarize(rows, arm, origin, end)
                for key in ('attempted', 'feasible', 'independent_valid', 'fallback_valid',
                            'fallback_used', 'coverage_fraction', 'solver_wall_seconds'):
                    if report[key] != recorded['configurations'][name][key]:
                        raise ValueError('SUMMARY_MISMATCH: '+name+'/'+key)
                compact[name] = {k: report[k] for k in (
                    'count', 'attempted', 'feasible', 'independent_valid', 'fallback_valid',
                    'fallback_used', 'coverage_fraction', 'continuous_survival_seconds',
                    'stop_reason', 'solver_wall_seconds', 'handover', 'solver_dynamic_and_fov')}
                if mode == 'single':
                    totals[arm] = report
                if mode == 'rolling':
                    populations[arm] = rows
                if arm == 'C':
                    for row in rows:
                        if not row['result'] or not row['result']['valid']:
                            continue
                        req, model = restore_record(dict(
                            schema_version=1, accepted_by_tracker=False,
                            request=row['raw_request'],
                            input_provenance=dict(model_config=row['model_config'])))
                        if row['request'].get('prior_curve') is not None:
                            payload = dict(row['request'], context=req.context)
                            payload['prior_curve'] = FollowCurve(**payload['prior_curve'])
                            req = HypotheticalRequest(**payload)
                        if constraint_snapshot(model).fingerprint != row['constraint_fingerprint']:
                            raise ValueError('CONSTRAINT_FINGERPRINT_MISMATCH')
                        v, details = assess_curve(req, row['result']['metrics'], model, budget=1.)
                        reference_height_errors.extend(abs(details[1][0][:, 2]+5.))
                        validations.append(dict(mode=mode, line=row['line'],
                                                source_sha256=row['source_sha256'], **v))
                        c = req.context
                        until = min(c.navigation_stamp+.125, c.observation_stamp+.125,
                                    c.prediction_source_stamp+.125, c.prediction_valid_until)
                        margins.append(until-req.now_stamp-row['seconds']-.02)
            input_checks.append(all(v == raw_populations[0] for v in raw_populations))
            if mode == 'rolling':
                groups = populations
        comparison = matched(groups)
        if comparison != recorded['matched_rolling_acd'][scene+'_h1.6_main']:
            raise ValueError('MATCHED_SUMMARY_MISMATCH')
        compact[scene+'_matched'] = comparison
        for arm, rows in groups.items():
            for row in rows:
                matched_rows.append({k: row[k] for k in (
                    'source_sha256', 'line', 'epoch', 'continuous_age', 'error', 'sampled_state')}
                                    | dict(scene=scene, arm=arm))
        a, c = totals['A'], totals['C']
        roll = compact[f'{scene}_h1.6_main_C_rolling']
        fov_margins = [min(v['minimum_horizontal_margin'], v['minimum_vertical_margin'])
                       for v in validations]
        evidence['scenes'][scene] = dict(
            same_inputs=all(input_checks),
            baseline_total=a['feasible'], candidate_total=c['feasible'],
            independent=c['independent_valid'], fallback=c['fallback_valid'],
            attempted=c['attempted'], steady_count=comparison['steady_count'],
            baseline_xy=comparison['groups']['A']['after20']['fixed'],
            candidate_xy=comparison['groups']['C']['after20']['fixed'],
            full_coverage=roll['coverage_fraction'] == 1. and roll['stop_reason'] is None,
            revalidated=len(validations),
            validation_failures=sum(not v['valid'] for v in validations),
            reference_height_max_error=float(max(reference_height_errors)),
            height_rmse=comparison['groups']['C']['after20']['height'],
            p95=max(c['solver_wall_seconds']['p95'], roll['solver_wall_seconds']['p95']),
            p99=max(c['solver_wall_seconds']['p99'], roll['solver_wall_seconds']['p99']),
            minimum_margin_after_reserve=min(margins),
            margin_after_reserve=distribution(margins),
            minimum_full_sphere_fov=min(fov_margins))
    counter_groups = {arm: [json.loads(line) for line in (
        formal/f'f2_h1.2_main_{arm}_rolling.jsonl').open()] for arm in ('A', 'C', 'D')}
    counter = matched(counter_groups)
    if counter != recorded['matched_rolling_acd']['f2_h1.2_main']:
        raise ValueError('H12_COUNTEREXAMPLE_MISMATCH')
    evidence['h12_counterexample_retained'] = (
        counter['groups']['C']['after20']['fixed'] > counter['groups']['A']['after20']['fixed'])
    payload = dict(candidate=CANDIDATE, evidence=evidence, decision=qualify(evidence),
                   compact=compact, h12_counterexample=counter,
                   source_checks=source_checks, inputs=inputs,
                   formal_directory=str(formal), formal_sha256=entries,
                   final_sha256=final_entries,
                   original_qualification=json.loads((final/'qualification.json').read_text()))
    (output/'release.json').write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')
    with (output/'matched_samples.jsonl').open('x') as stream:
        for row in matched_rows:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
    with (output/'SHA256SUMS').open('x') as stream:
        for path in sorted(output.iterdir()):
            if path.name != 'SHA256SUMS':
                stream.write(sha(path)+'  '+path.name+'\n')
    print(json.dumps(payload['decision'], indent=2))
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--formal', type=Path, default=P47/'comparison/formal_20261012_2')
    args = parser.parse_args()
    audit(args.formal, args.output)


if __name__ == '__main__':
    main()

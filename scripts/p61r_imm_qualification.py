#!/usr/bin/env python3
"""Replay immutable raw observations and bind a scoped IMM pilot offline qualification."""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from uav_control.controllers.follow_mpc_shadow_node import prediction_from_message
from uav_control.guidance.p6_follow import attach_model_diagnostics
from uav_control.tracking.imm_target_predictor_node import (
    ImmPredictionEngine, RawObservation, prediction_to_message,
)

KIND = 'P61R_IMM_OFFLINE_BOOTSTRAP'
WORKSPACE = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sources():
    return [WORKSPACE / path for path in (
        'scripts/p61r_imm_qualification.py',
        'src/uav_control/uav_control/tracking/imm_predictor.py',
        'src/uav_control/uav_control/tracking/imm_target_predictor_node.py',
        'src/uav_control/uav_control/tracking/target_prediction.py',
        'src/uav_control/uav_control/tracking/ros_time.py',
        'src/uav_control/uav_control/controllers/follow_mpc_shadow_node.py',
        'src/uav_control/uav_control/guidance/p6_follow.py')]


def replay_rows(rows):
    """
    Run the current engine, wire conversion and strict attach on every saved row.

    Recorded receipt epochs drive causal gates, while elapsed costs use only
    perf_counter. No truth or simulated optimizer outcome enters this replay.
    """
    engine = ImmPredictionEngine()
    rejected, samples = Counter(), Counter()
    elapsed, matches, expired, unsafe, accepted = [], 0, 0, 0, 0
    for sequence, row in enumerate(rows, 1):
        observation = RawObservation(
            row['stamp'], tuple(row['p']), tuple(row['covariance']), row['frame'], row['source'],
            row['valid'], row['reason'],
            (row['image_clock_reset_count'], row['px4_clock_reset_count']),
            row['received'], row['image_measurement_stamp'])
        started = time.perf_counter()
        status = engine.update(observation, row['receipt'])
        if not status.accepted:
            rejected[status.reason] += 1
            continue
        accepted += 1
        result = engine.generate(row['receipt'], 1, sequence)
        if not result.valid:
            unsafe += 1
            continue
        diagnostic = engine.diagnostics(result)
        parsed = prediction_from_message(prediction_to_message(result, 0.))
        attached = attach_model_diagnostics(parsed, diagnostic)
        elapsed.append(time.perf_counter()-started)
        samples[str(len(result.samples))] += 1
        matches += int(attached.uncertainty_identity == (
            1, sequence, result.source_stamp, observation.stamp))
        unsafe += int(result.observation_stamp != observation.stamp
                      or result.valid_until != observation.stamp+.125
                      or result.generated_stamp != row['receipt']
                      or any(np.min(np.linalg.eigvalsh(
                          np.asarray(s.position_covariance).reshape(3, 3))) < -1e-9
                             for s in result.samples))
        # Isolate the negative case completely from the next recorded input.
        stale = deepcopy(engine).generate(observation.stamp+.126, 1, sequence)
        expired += int(not stale.valid and not stale.samples
                       and stale.invalid_reason == 'OBSERVATION_STALE')
    return dict(attempts=len(rows), accepted=accepted, rejections=dict(rejected),
                strict_identity_matches=matches, expired_rejections=expired,
                unsafe_output_count=unsafe, sample_counts=dict(samples),
                compute_seconds=dict(zip(('p50', 'p95', 'p99', 'maximum'),
                                         map(float, np.percentile(elapsed, (50, 95, 99, 100)))))
                if elapsed else {},
                timing_scope='real engine update+forecast+diagnostics+wire+strict attach; '
                'offline load, excluding extra negative/PSD tests and DDS',
                clock_domains='recorded ROS receipt for causality; perf_counter for costs')


def replay_pass(report):
    n = report.get('accepted', 0)
    return bool(n >= 1000 and n >= .99*report.get('attempts', float('inf'))
                and report.get('strict_identity_matches') == n
                and report.get('expired_rejections') == n
                and report.get('unsafe_output_count') == 0
                and report.get('sample_counts') == {'41': n}
                and 0 < report.get('compute_seconds', {}).get('p95', float('inf')) < .02)


def validate_primary_evidence(release, metadata, required_sources=None):
    """A shadow boolean cannot grant IMM primary or any follow-on experiment."""
    try:
        reference = release['imm_primary_evidence']
        path = Path(reference['path'])
        if sha(path) != reference['sha256']:
            raise ValueError('IMM_PRIMARY_EVIDENCE_CHANGED')
        report = json.loads(path.read_text())
        if report.get('kind') != KIND or report.get('passed') is not True:
            raise ValueError('IMM_PRIMARY_EVIDENCE_REQUIRED')
        if (report.get('scope') != 'linear_nominal_pilot'
                or any(metadata.get(k) != v for k, v in dict(
                    role='pilot', motion_mode='linear', group='B', prediction_model='imm',
                    visibility_mode='nominal').items())):
            raise ValueError('IMM_PRIMARY_SCOPE_MISMATCH')
        for evidence in report['evidence']:
            if sha(evidence['path']) != evidence['sha256']:
                raise ValueError('IMM_PRIMARY_EVIDENCE_CHANGED')
        evidence = {r['role']: json.loads(Path(r['path']).read_text())
                    for r in report['evidence']}
        if (not replay_pass(evidence['replay'])
                or evidence['review'].get('review_pass') is not True
                or evidence['verification'].get('passed') is not True
                or evidence['verification'].get('independent_review_pass') is not True):
            raise ValueError('IMM_PRIMARY_EVIDENCE_REQUIRED')
        rows = {Path(r['path']).resolve(): r['sha256'] for r in report['source_checks']}
        if any(rows.get(p.resolve()) != sha(p) for p in (required_sources or sources())):
            raise ValueError('IMM_PRIMARY_SOURCE_CHANGED')
        return report
    except (OSError, KeyError, TypeError) as exc:
        raise ValueError('IMM_PRIMARY_EVIDENCE_REQUIRED') from exc


def replay_case(case, output):
    """Create a new analysis alongside immutable history, retaining all window inputs."""
    case, output = Path(case).resolve(), Path(output).resolve()
    if output.is_relative_to(case) or case.is_relative_to(output):
        raise ValueError('NEW_EXTERNAL_REPLAY_DIRECTORY_REQUIRED')
    summary_path = case/'flight/p5_run_summary.json'
    raw_path = case/'flight/offline_evidence.jsonl'
    start = json.loads(summary_path.read_text())['first_minco_receipt']
    if start is None:
        raise ValueError('ORIGINAL_WINDOW_REQUIRED')
    rows = [json.loads(line) for line in raw_path.open()]
    selected = [r for r in rows if r.get('kind') == 'p6_observation'
                and start <= r['receipt'] <= start+60.]
    report = replay_rows(selected)
    report.update(passed=replay_pass(report), scope='offline raw-observation compatibility only',
                  evidence=[dict(path=str(p), sha256=sha(p)) for p in (summary_path, raw_path)],
                  source_checks=[dict(path=str(p), sha256=sha(p)) for p in sources()])
    output.mkdir(parents=True, exist_ok=False)
    (output/'replay.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'SHA256SUMS').write_text(sha(output/'replay.json')+'  replay.json\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(replay_case(args.case, args.output), indent=2))


if __name__ == '__main__':
    main()

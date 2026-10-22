#!/usr/bin/env python3
"""Decompose preserved P764 performance without claiming an unmeasured improvement."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path

from p764_diagnostic_collect import stats

ROOT = Path(__file__).resolve().parents[1]


def seconds(stamp):
    return stamp['sec']+stamp['nanosec']*1e-9


def analyze(session):
    """Keep measured quantities separate from intervals the old trace cannot identify."""
    path = session/'synchronized_packets.jsonl.gz'
    with gzip.open(path, 'rt') as stream:
        rows = [json.loads(line) for line in stream]
    yolo = [json.loads(r['message']['data']) for r in rows if r['kind'] == 'yolo']
    observations = [r['message'] for r in rows if r['kind'] == 'observation']
    detections = [r for r in rows if r['kind'] == 'detection']
    timing = json.loads((session/'e1_tracker_shadow_timing.json').read_text())
    performance = json.loads((session/'e1_dkf_performance.json').read_text())
    callback = performance['callback_seconds']
    return dict(
        schema_version=1, evidence_class='P764_HISTORICAL_TIMING_RECOMPUTED',
        source=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        actual_new_sessions=0, measured_performance_improvement=False,
        yolo=dict(count=len(yolo), inference_executed=sum(
            r['inference_seconds'] > 0 for r in yolo),
                  inference_seconds=stats([r['inference_seconds'] for r in yolo
                                           if r['inference_seconds'] > 0]),
                  completion_source_age_seconds=stats([r['source_age'] for r in yolo
                                                       if r['source_age'] is not None]),
                  reason_and_inference_counts=dict(Counter(
                      r['reason']+':inference='+str(r['inference_seconds'] > 0) for r in yolo)),
                  unattributed_pipeline_seconds=stats([
                      r['source_age']-r['inference_seconds'] for r in yolo
                      if r['source_age'] is not None and r['inference_seconds'] > 0]),
                  detector_complete_to_collector_receipt_seconds=stats([
                      r['receipt']-seconds(r['message']['detection_stamp']) for r in detections])),
        localizer=dict(paired_receipt_to_processed_seconds=stats([
            r['processed_stamp']['sec']+r['processed_stamp']['nanosec']*1e-9
            - max(r['rgb_receipt_stamp'], r['depth_receipt_stamp']) for r in observations]),
            valid_observations=sum(r['valid'] for r in observations)),
        dkf=dict(callback_seconds=callback,
                 image_valid=sum(r['message']['valid'] for r in rows if r['kind'] == 'image'),
                 spatial_valid=sum(r['message']['valid'] for r in rows if r['kind'] == 'spatial'),
                 caveat='Old callback times include synchronous instrumentation/serialization; '
                        'nested image receive/tick durations overlap and must not be summed.'),
        shadow=timing, process=dict(
            cpu_seconds=performance['process_cpu_seconds'],
            wall_seconds=performance['wall_seconds'],
            core_fraction=performance['process_cpu_core_fraction'],
            rss_bytes=performance['rss_bytes']),
        proven_confounder='PX4 temporary stdin pipe closed; child EOF console busy-loop. '
                          'Fixed wrapper subprocess test verifies stdin lifetime only.',
        unresolved=['Separate DDS wait from RGB bridge transport: no per-process monotonic '
                    'enqueue/start timestamps in old trace.',
                    'Separate recorder serialization from actual DKF method compute: old wrapper '
                    'times include serialization.',
                    'Attribute each of 157 missed cycles: raw wakeup samples/callback spans and '
                    'scheduler/CPU timelines were not saved.',
                    'Quantify PX4 EOF fix or single-worker improvement: requires one separately '
                    'authorized same-scene unarmed session.'],
        new_instrumentation=['single inference worker; latest pending frame replacement',
                             'receipt/selection/inference/publication source ages',
                             'bounded asynchronous serializer with explicit overflow failure',
                             'raw timer intervals, costs, graph, transitions, source identities'],
        limits_unchanged=dict(source_age=.125, period=.005),
        e1_status='NO_GO_NO_VALID_3D_AND_157_MISSED_CYCLES')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, default=(
        ROOT/'data/baselines/20261021_p764_validation_01/nonflight_01'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.session)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(yolo=result['yolo']['count'],
                          inferred=result['yolo']['inference_executed'],
                          improvement_measured=False)))


if __name__ == '__main__':
    main()

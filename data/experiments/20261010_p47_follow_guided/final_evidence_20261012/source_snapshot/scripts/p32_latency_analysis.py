#!/usr/bin/env python3
"""Causal epoch latency and scalar/batch BCTRA benchmarks, without fitted time shifts."""
import argparse
import json
from pathlib import Path
import time

import numpy as np


def percentiles(values):
    """Retain milliseconds and null for genuinely missing evidence."""
    return dict(zip(('p50_ms', 'p95_ms', 'p99_ms'), map(float, np.percentile(
        np.asarray(values)*1000, (50, 95, 99))))) if values else None


def arrival(root):
    """Only unique valid predictions are counted, preserving original native epochs."""
    seen = {}
    for path in (root/'shadow').glob('*.jsonl'):
        for text in path.open():
            row = json.loads(text)
            p = (row.get('input_provenance') or {}).get('prediction') or {}
            if p.get('valid') and (p.get('snapshot') or p['sequence_id'] not in seen):
                seen[p['sequence_id']] = p
    columns = {k: [] for k in ('image_to_kf_epoch', 'kf_epoch_to_generation',
                               'generation_to_shadow_receipt', 'image_to_shadow_receipt')}
    for p in seen.values():
        values = (p['source_stamp']-p['observation_stamp'],
                  p['generated_stamp']-p['source_stamp'],
                  p['receipt_ros_stamp']-p['generated_stamp'],
                  p['receipt_ros_stamp']-p['observation_stamp'])
        for key, value in zip(columns, values):
            columns[key].append(value)
    result = dict(unique_seen_predictions=len(seen),
                  **{k: percentiles(v) for k, v in columns.items()})
    raw_path = root/'offline_evidence.jsonl'
    if raw_path.exists():
        raw = [json.loads(s) for s in raw_path.open()]
        obs = {round(r['stamp'], 6): r for r in raw if r['kind'] == 'observation' and r['valid']}
        columns = {k: [] for k in ('image_to_received', 'received_to_processed',
                                   'processed_to_publish', 'publish_to_generation',
                                   'prediction_python_compute', 'image_to_prediction_receipt')}
        matched = 0
        for p in raw:
            if p['kind'] != 'prediction' or not p['valid']:
                continue
            columns['prediction_python_compute'].append(p['compute'])
            columns['image_to_prediction_receipt'].append(p['receipt']-p['observation'])
            o = obs.get(round(p['observation'], 6))
            if o is not None:
                matched += 1
                for key, value in zip(list(columns)[:4], (
                        o['received']-o['stamp'], o['processed']-o['received'],
                        o['published']-o['processed'], p['generated']-o['published'])):
                    columns[key].append(value)
        result['matched_observation_prediction'] = matched
        result['stages'] = {k: percentiles(v) for k, v in columns.items()}
        clocks = [r for r in raw if r['kind'] == 'clock']
        poses = [r for r in raw if r['kind'] == 'rendered_target']
        if clocks and poses:
            native = np.array([c['sim'] for c in clocks])
            epochs = np.array([c['system'] for c in clocks])
            covered = [p for p in poses if native[0] <= p['sim'] <= native[-1]]
            mapped = np.interp([p['sim'] for p in covered], native, epochs)
            xy = np.array([p['p'][:2] for p in covered])
            monotonic = bool(np.all(np.diff(native) >= 0) and np.all(np.diff(mapped) > 0))
            errors = {h: [] for h in (.8, 1.2, 1.6, 2.4, 4.)}
            if monotonic and covered:
                for p in seen.values():
                    source, snap = p['source_stamp'], p.get('snapshot')
                    if snap is None:
                        continue
                    if source+.8 < mapped[0] or source+4. > mapped[-1]:
                        continue
                    for h, values in errors.items():
                        predicted = [np.interp(h, snap['prediction_times'],
                                               np.asarray(snap['target_positions'])[:, i])
                                     for i in (0, 1)]
                        actual = [np.interp(source+h, mapped, xy[:, i]) for i in (0, 1)]
                        values.append(float(np.linalg.norm(np.asarray(predicted)-actual)))
            result['forecast_error_horizontal'] = dict(
                evaluation_only=True, monotonic=monotonic, common_population=True,
                truth_source='native Gazebo entity pose, original clock mapping',
                horizons={str(h): dict(count=len(v), rmse_m=float(np.sqrt(np.mean(np.square(v)))),
                                       p95_m=float(np.percentile(v, 95))) if v else None
                          for h, v in errors.items()})
    (root/'prediction_latency.json').write_text(json.dumps(result, indent=2))
    return result


def benchmark(root):
    """Compare unchanged scalar predictions, shared prefixes and actual ROS construction."""
    from uav_control.tracking.maneuvering_target_predictor import ManeuveringTargetPredictor
    from uav_control.tracking.target_prediction import PredictionEngine, TargetKinematicState
    from uav_control.tracking.target_predictor_node import prediction_to_message
    p = ManeuveringTargetPredictor(max_turn_rate=.7)
    p.valid_turn_updates = 10
    p.turn_rate, p.turn_acceleration, p.speed_acceleration = .6, -.8, .5
    p.update_vertical(.1, .2, 100.)
    engine = PredictionEngine(p)
    engine.latest_state = TargetKinematicState(100., (3., 2., .1), (4., -2., .2))
    times, state = engine._sample_times(), (3., 2., .1, 4., -2., .2)
    old, new, conversion = [], [], []
    error = float(np.max(np.abs(np.asarray([p.predict(*state, t) for t in times]) -
                                np.asarray(p.predict_many(*state, times)))))
    result = engine.generate(100.05, 1, 1)
    for _ in range(400):
        start = time.perf_counter()
        [p.predict(*state, t) for t in times]
        old.append(time.perf_counter()-start)
        start = time.perf_counter()
        p.predict_many(*state, times)
        new.append(time.perf_counter()-start)
        start = time.perf_counter()
        prediction_to_message(result, 0.)
        conversion.append(time.perf_counter()-start)
    results = dict(count=400, horizon=4., samples=len(times),
                   maximum_position_velocity_error=error,
                   scalar_series=percentiles(old), batch_series=percentiles(new),
                   ros_message_construction=percentiles(conversion),
                   model_changes=False, compute_includes_dds=False)
    (root/'prediction_compute_benchmark.json').write_text(json.dumps(results, indent=2))
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--benchmark', action='store_true')
    args = parser.parse_args()
    print(json.dumps(benchmark(args.root) if args.benchmark else arrival(args.root)))

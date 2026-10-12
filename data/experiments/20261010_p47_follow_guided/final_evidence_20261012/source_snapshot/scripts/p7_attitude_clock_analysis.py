#!/usr/bin/env python3
"""Offline P7.3 attitude/clock audit; never publishes or fits target error.

PX4 SITL hrt follows Gazebo sim time (GZBridge::clockCallback). Validate that
identity independently with logged groundtruth versus the captured model at
raw image time. Keep precisely the existing P7.3 validated frame population.
Quaternion arrays use scipy xyzw; capture fields explicitly use wxyz.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation, Slerp

from kf_truth_capture import convert_rows


def quaternion_at(stamp, times, quaternions, resets, maximum_gap=0.05):
    """SLERP a close bracket within one reset epoch, with no extrapolation."""
    times = np.asarray(times)
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and abs(times[index] - stamp) < 1e-8:
        return Rotation.from_quat(quaternions[index])
    if index == 0 or index == len(times):
        raise ValueError('attitude_not_bracketed')
    left, right = times[index - 1:index + 1]
    if max(stamp - left, right - stamp) > maximum_gap or right <= left:
        raise ValueError('attitude_bracket_too_wide')
    if resets[index - 1] != resets[index]:
        raise ValueError('attitude_reset_crossed')
    return Slerp([left, right], Rotation.from_quat(
        quaternions[index - 1:index + 1]))(stamp)


def decompose(body, online, image_ekf, image_truth, image_model):
    """Exact telescoping vector decomposition; component RMSEs do not add."""
    p, e, g, m = (rotation.apply(body) for rotation in
                  (online, image_ekf, image_truth, image_model))
    return {'total': p - m, 'timing': p - e,
            'estimation': e - g, 'reference': g - m}


def series(dataset):
    """Extract a finite, monotonic quaternion stream and reset counters."""
    data = dataset.data
    times = np.asarray(data['timestamp_sample'], dtype=float) * 1e-6
    quaternions = np.column_stack([data[f'q[{i}]'] for i in (1, 2, 3, 0)])
    valid = np.isfinite(times) & np.all(np.isfinite(quaternions), axis=1)
    valid &= np.linalg.norm(quaternions, axis=1) > 0.9
    times, indices = np.unique(times[valid], return_index=True)
    return (times, quaternions[valid][indices],
            np.asarray(data['quat_reset_counter'])[valid][indices])


def statistics(values):
    """Summarize vector norms without adding component RMSEs."""
    values = np.asarray(values)
    norms = np.linalg.norm(values, axis=1)
    return {'n': len(norms), 'rmse': float(np.sqrt(np.mean(norms ** 2))),
            'p95': float(np.percentile(norms, 95)),
            'max': float(norms.max()),
            'mean_xyz': values.mean(axis=0).tolist()}


def analyze(path):
    """Split each existing P7.3 validated frame into three vector terms."""
    previous = json.loads(path.read_text())
    with Path(previous['source_csv']).open() as stream:
        raw = list(csv.DictReader(stream))
    reference = convert_rows(raw)
    lookup = {ref['timestamp']: (row, ref)
              for row, ref in zip(raw, reference)
              if ref['frame_valid']}
    pairs = [json.loads(line) for line in
             path.with_name('paired_frames.jsonl').read_text().splitlines()]
    log = ULog(previous['source_ulog'], message_name_filter_list=[
        'vehicle_attitude', 'vehicle_attitude_groundtruth'])
    streams = {data.name: series(data) for data in log.data_list}
    records, rejected = [], Counter()
    for pair in pairs:
        stamp = pair['timestamp']
        row, ref = lookup[stamp]
        image_time = float(row['image_rgb_raw_stamp'])
        used_time = stamp - previous['ros_minus_ulog_boot_offset_s']
        try:
            image_ekf = quaternion_at(image_time, *streams['vehicle_attitude'])
            image_truth = quaternion_at(
                image_time, *streams['vehicle_attitude_groundtruth'])
            used_ekf = quaternion_at(used_time, *streams['vehicle_attitude'])
            # A reset between the two queries invalidates attribution.
            times, _, resets = streams['vehicle_attitude']
            lo, hi = sorted((image_time, used_time))
            reset_slice = resets[max(0, np.searchsorted(times, lo) - 1):
                                 np.searchsorted(times, hi) + 1]
            if len(np.unique(reset_slice)) > 1:
                raise ValueError('image_to_online_reset_crossed')
        except ValueError as exc:
            rejected[str(exc)] += 1
            continue
        online = Rotation.from_quat(np.asarray(
            ref['px4_uav_attitude_ned_frd_wxyz'])[[1, 2, 3, 0]])
        model = Rotation.from_quat(np.asarray(
            ref['gazebo_uav_attitude_ned_frd_wxyz'])[[1, 2, 3, 0]])
        body = np.array([float(row[f'target_body_flu_{axis}'])
                         for axis in 'xyz']) * [1, -1, -1]
        terms = decompose(body, online, image_ekf, image_truth, model)
        raw_error = np.asarray(pair['raw_operational_error_ned'])
        terms.update({
            'self_check': online.apply(body) - used_ekf.apply(body),
            'raw': raw_error,
            'timing_only_counterfactual': raw_error - terms['timing'],
            'perfect_attitude_counterfactual': raw_error - terms['total'],
            'closure': terms['total'] - terms['timing']
            - terms['estimation'] - terms['reference'],
            'p7_component_check': terms['total'] - np.asarray(
                pair['px4_attitude_contribution_ned']),
            'estimation_rotvec_deg': (
                image_ekf * image_truth.inv()).as_rotvec() * 180 / np.pi,
        })
        records.append({
            'timestamp': stamp, 'image_sim_stamp': image_time,
            'online_ulog_query_stamp': used_time,
            'online_minus_image_s': used_time - image_time,
            'relative_seconds': stamp - pairs[0]['timestamp'],
            'body_range_m': float(np.linalg.norm(body)),
            **{key: value.tolist() for key, value in terms.items()},
        })
    if not records:
        raise ValueError('no_valid_frames')
    summaries = {}
    for name, subset in (
        ('full', records),
        ('first_10s', [r for r in records if r['relative_seconds'] < 10]),
        ('after_10s', [r for r in records if r['relative_seconds'] >= 10]),
    ):
        if not subset:
            continue
        summaries[name] = {
            **{key: statistics([r[key] for r in subset])
               for key in terms},
            'body_range_p50_p95_m': np.percentile(
                [r['body_range_m'] for r in subset], [50, 95]).tolist(),
            'online_minus_image_p50_p95_s': np.percentile(
                [r['online_minus_image_s'] for r in subset],
                [50, 95]).tolist(),
        }
    summary = {
        'scenario': previous['scenario'],
        'source_analysis': str(path.resolve()),
        'source_ulog': previous['source_ulog'],
        'input_count': len(pairs), 'accepted_count': len(records),
        'rejections': dict(rejected),
        'phase_origin': 'first validated P7.3 frame (phase_comparison)',
        'units': 'vectors: m; estimation_rotvec_deg: degrees',
        'parameters': {key: log.initial_parameters.get(key) for key in (
            'EKF2_MAG_TYPE', 'EKF2_MAG_DECL', 'EKF2_DECL_TYPE',
            'EKF2_GPS_CTRL',
            'EKF2_EV_CTRL', 'SIM_GZ_EN_MAG')},
        'phases': summaries,
    }
    return summary, records


def main():
    """Write per-frame evidence and summaries for all validated scenarios."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(
        'data/experiments/current/p7_kf_20260929'))
    parser.add_argument('--output', type=Path, default=Path(
        'data/experiments/current/p7_attitude_clock_20260929'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for path in sorted(args.root.glob('**/analysis_validated/analysis.json')):
        summary, records = analyze(path)
        results.append(summary)
        output = args.output / (summary['scenario'] + '_frames.jsonl')
        output.write_text(''.join(json.dumps(r) + '\n' for r in records))
        phase = summary['phases']['first_10s']
        print(summary['scenario'], summary['accepted_count'], {
            key: round(phase[key]['rmse'], 6)
            for key in ('raw', 'total', 'estimation', 'timing', 'reference')})
    if not results:
        raise ValueError('no P7.3 validated analyses found')
    (args.output / 'summary.json').write_text(
        json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
    main()

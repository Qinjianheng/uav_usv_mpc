#!/usr/bin/env python3
"""Match sampled pose/info model positions to PX4 groundtruth ULog rows."""

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.spatial import cKDTree


def percentiles(values):
    values = np.asarray(values, dtype=float)
    return dict(zip(('p05', 'p50', 'p95', 'max'),
                    np.percentile(values, [5, 50, 95, 100]).tolist()))


def analyze(pose_path, ulog_path, scenario):
    pose = [json.loads(line) for line in pose_path.read_text().splitlines()]
    model = np.asarray([
        (row['model_position_enu'][1], row['model_position_enu'][0],
         -row['model_position_enu'][2]) for row in pose
    ])
    sim = np.asarray([row['sim_stamp'] for row in pose], dtype=float)
    log = ULog(str(ulog_path), message_name_filter_list=[
        'vehicle_local_position_groundtruth', 'vehicle_local_position',
    ])
    data = {dataset.name: dataset.data for dataset in log.data_list}
    gt = data['vehicle_local_position_groundtruth']
    gt_time = gt['timestamp_sample'].astype(float) * 1e-6
    gt_position = np.stack([gt[axis] for axis in 'xyz'], axis=1)
    gt_velocity = np.stack([gt['v' + axis] for axis in 'xyz'], axis=1)
    in_capture = (gt_time >= sim[0]) & (gt_time <= sim[-1])
    tree = cKDTree(model)
    distances, indices = tree.query(gt_position[in_capture], k=2)
    speed = np.linalg.norm(gt_velocity[in_capture], axis=1)
    # Dynamic samples must have a unique model pose. Repeated positions while
    # stationary otherwise match arbitrary samples and create false delays.
    selected = ((speed > 0.3) & (distances[:, 0] < 1e-4)
                & (distances[:, 1] > 1e-3))
    matches = indices[selected, 0]
    aligned_gt_time = gt_time[in_capture][selected]
    source_sim = sim[matches]
    px4 = data['vehicle_local_position']
    px4_time = px4['timestamp_sample'].astype(float) * 1e-6
    px4_est = np.stack([
        np.interp(aligned_gt_time, px4_time, px4[axis])
        for axis in 'xyz'
    ], axis=1) if len(matches) else np.empty((0, 3))
    residual = px4_est - model[matches]
    receipt_minus_mapped = np.asarray([
        pose[index]['receipt_ros_stamp'] - pose[index]['mapped_stamp']
        for index in matches
    ])
    result = {
        'scenario': scenario,
        'pose_receipt_file': str(pose_path),
        'ulog_file': str(ulog_path),
        'pose_rows': len(pose),
        'groundtruth_rows_in_capture': int(in_capture.sum()),
        'unique_dynamic_exact_position_matches': len(matches),
        'matching_rule': 'GT speed >0.3m/s; nearest pose <0.1mm; '
                         'second-nearest pose >1mm',
        'matched_position_distance_m': (
            percentiles(distances[selected, 0]) if len(matches) else None
        ),
        'px4_groundtruth_timestamp_sample_minus_pose_header_sim_s': (
            percentiles(aligned_gt_time - source_sim)
            if len(matches) else None
        ),
        'px4_groundtruth_timestamp_minus_timestamp_sample_s': (
            percentiles((gt['timestamp'][in_capture][selected]
                         - gt['timestamp_sample'][in_capture][selected])
                        * 1e-6) if len(matches) else None
        ),
        'evaluator_gz_transport_receipt_minus_mapped_stamp_s': (
            percentiles(receipt_minus_mapped) if len(matches) else None
        ),
        'px4_estimate_minus_exact_model_ned_rmse_m': (
            float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
            if len(matches) else None
        ),
        'px4_estimate_minus_exact_model_ned_mean_xyz_m': (
            residual.mean(axis=0).tolist() if len(matches) else None
        ),
        'px4_bridge_wall_receipt_time_observable': False,
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', action='append', required=True,
                        help='name=pose_receipt.jsonl=PX4.ulg')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    for item in args.scenario:
        name, pose_path, ulog_path = item.split('=', 2)
        result[name] = analyze(Path(pose_path), Path(ulog_path), name)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    for name, item in result.items():
        print(name, item['unique_dynamic_exact_position_matches'],
              item['px4_groundtruth_timestamp_sample_minus_pose_header_sim_s'])


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Audit P8.1 pose starvation against PX4's own DDS clock diagnostics.

No target position, truth pose, or target error enters this time comparison.
Timesync samples are held causally, never fitted to an error minimum.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def clock_budget(row, times, estimated_offsets):
    """Split apparent pose age into clock-domain difference and remainder."""
    query = float(row['rgb_raw_stamp'])
    index = int(np.searchsorted(times, query, side='right')) - 1
    if index < 0:
        return None
    # DDS adds -estimated_offset. The image mapper uses system-sim offset.
    clock_lag = (float(row['image_clock_offset'])
                 + float(estimated_offsets[index]) * 1e-6)
    age = (float(row['processed_stamp'])
           - float(row['position_history_end_stamp']))
    return {
        'clock_lag_s': clock_lag,
        'apparent_pose_age_s': age,
        'residual_age_s': age - clock_lag,
        'timesync_sample_age_s': query - times[index],
        'image_lead_position_s': float(row['image_measurement_stamp'])
        - float(row['position_history_end_stamp']),
        'receipt_to_process_s': float(row['processed_stamp'])
        - float(row['receipt_stamp']),
    }


def percentiles(values):
    """Report finite min, median, P95 and max without hiding invalid values."""
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    return {'n': len(values), 'finite_n': len(finite),
            'min_p50_p95_max': np.percentile(
                finite, [0, 50, 95, 100]).tolist() if len(finite) else []}


def analyze(csv_path, ulog_path):
    """Compare a vision CSV with the independently recorded clock stream."""
    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    log = ULog(str(ulog_path), message_name_filter_list=['timesync_status'])
    streams = [d for d in log.data_list if d.name == 'timesync_status'
               and np.all(d.data['source_protocol'] == 2)]
    if len(streams) != 1:
        raise ValueError('expected one DDS timesync stream')
    data = streams[0].data
    times = np.asarray(data['timestamp'], dtype=float) * 1e-6
    if np.any(np.diff(times) <= 0):
        raise ValueError('timesync reset: split log into epochs first')
    records = []
    for row in rows:
        budget = clock_budget(row, times, data['estimated_offset'])
        if budget is None:
            continue
        records.append({'stamp': float(row['measurement_stamp']),
                        'reason': row['rejection_reason'], **budget})
    groups = {}
    for name, predicate in (
        ('valid', lambda r: not r['reason']),
        ('position_after_history', lambda r:
         r['reason'] == 'POSITION_TIMESTAMP_AFTER_HISTORY'),
    ):
        selected = [r for r in records if predicate(r)]
        groups[name] = {
            key: percentiles([r[key] for r in selected])
            for key in ('clock_lag_s', 'apparent_pose_age_s',
                        'residual_age_s', 'timesync_sample_age_s',
                        'image_lead_position_s', 'receipt_to_process_s')}
        groups[name]['lead_exceeds_existing_150ms_gate'] = sum(
            r['image_lead_position_s'] > 0.15 for r in selected)
        groups[name]['receipt_wait_exceeds_existing_150ms'] = sum(
            r['receipt_to_process_s'] > 0.15 for r in selected)
    return {
        'source_csv': str(csv_path.resolve()),
        'source_ulog': str(ulog_path.resolve()),
        'rows': len(rows), 'paired_clock_rows': len(records),
        'rejection_histogram': dict(Counter(r['rejection_reason'] for r in rows)),
        'caveat': '1 Hz held timesync; residual is not a measured DDS latency',
        'groups': groups,
    }, records


def main():
    """Write an auditable report without publishing or running simulation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, required=True)
    parser.add_argument('--ulog', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summary, records = analyze(args.csv, args.ulog)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    (args.output / 'frames.jsonl').write_text(
        ''.join(json.dumps(row) + '\n' for row in records))
    print(json.dumps(summary['groups'], indent=2))


if __name__ == '__main__':
    main()

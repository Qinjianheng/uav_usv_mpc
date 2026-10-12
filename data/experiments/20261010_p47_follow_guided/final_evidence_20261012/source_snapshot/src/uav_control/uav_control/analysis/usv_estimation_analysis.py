"""Summarize camera, Kalman, and future-position errors from an experiment."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np


ERROR_FIELDS = (
    ('camera', 'camera_position_error'),
    ('kalman', 'kf_position_error'),
    ('guidance_0.5s', 'guidance_prediction_0p5_error'),
    ('guidance_1.0s', 'guidance_prediction_1p0_error'),
    ('guidance_2.0s', 'guidance_prediction_2p0_error'),
    ('kalman_0.5s', 'kf_prediction_0p5_error'),
    ('kalman_1.0s', 'kf_prediction_1p0_error'),
    ('kalman_2.0s', 'kf_prediction_2p0_error'),
)


def finite_column(rows, field):
    """Return finite floating-point values from one CSV field."""
    values = []
    for row in rows:
        text = row.get(field, '')
        if text in ('', None):
            continue
        try:
            value = float(text)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return np.asarray(values, dtype=float)


def error_statistics(rows, field):
    """Calculate count, mean, RMSE, median, and 95th percentile."""
    values = finite_column(rows, field)
    if values.size == 0:
        return None
    return {
        'count': int(values.size),
        'mean': float(np.mean(values)),
        'rmse': float(math.sqrt(np.mean(values * values))),
        'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95.0)),
        'maximum': float(np.max(values)),
    }


def analyze_rows(rows):
    """Return availability and error summaries for parsed CSV rows."""
    rows = list(rows)
    total = len(rows)
    summary = {
        label: error_statistics(rows, field)
        for label, field in ERROR_FIELDS
    }
    for label, valid_field in (
        ('camera_availability', 'camera_measurement_valid'),
        ('kalman_availability', 'kf_state_valid'),
    ):
        valid = sum(row.get(valid_field) == '1' for row in rows)
        summary[label] = valid / total if total else 0.0
    summary['row_count'] = total
    return summary


def analyze_csv(path):
    """Read an interception CSV and return estimation statistics."""
    path = Path(path).expanduser()
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    return analyze_rows(rows)


def print_summary(path, summary):
    """Print a compact terminal report suitable for experiment review."""
    print(f'USV estimation report: {Path(path).expanduser()}')
    print(f'CSV rows: {summary["row_count"]}')
    print(
        'Availability: '
        f'camera={100.0 * summary["camera_availability"]:.1f}% | '
        f'Kalman={100.0 * summary["kalman_availability"]:.1f}%'
    )
    print('Error metrics (metres):')
    for label, _ in ERROR_FIELDS:
        statistics = summary[label]
        if statistics is None:
            print(f'  {label:14s} no valid samples')
            continue
        print(
            f'  {label:14s} n={statistics["count"]:4d} | '
            f'mean={statistics["mean"]:.3f} | '
            f'RMSE={statistics["rmse"]:.3f} | '
            f'median={statistics["median"]:.3f} | '
            f'P95={statistics["p95"]:.3f} | '
            f'max={statistics["maximum"]:.3f}'
        )


def main(args=None):
    """Run CSV estimation analysis from the command line."""
    parser = argparse.ArgumentParser(
        description='Analyze UAV prediction and camera/Kalman USV errors.',
    )
    parser.add_argument('csv_path', help='trajectory_impact_sim CSV path')
    parsed = parser.parse_args(args=args)
    summary = analyze_csv(parsed.csv_path)
    print_summary(parsed.csv_path, summary)


if __name__ == '__main__':
    main()

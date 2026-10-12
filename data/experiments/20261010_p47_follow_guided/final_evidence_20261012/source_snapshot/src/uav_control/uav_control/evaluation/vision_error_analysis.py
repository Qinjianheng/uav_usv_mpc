"""Reproducible offline attribution metrics for event-based vision CSVs."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def _is_true(value):
    return str(value).strip().lower() in ('1', 'true', 'yes')


def _finite(row, key):
    try:
        value = float(row.get(key, ''))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _percentile(values, fraction):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = min(max(float(fraction), 0.0), 1.0) * (
        len(ordered) - 1
    )
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] + weight * (ordered[upper] - ordered[lower])


def _metrics(values, signed=False):
    values = [float(value) for value in values]
    result = {
        'count': len(values),
        'mean': statistics.fmean(values) if values else None,
        'p50': _percentile(values, 0.50),
        'p95': _percentile(values, 0.95),
        'rmse': (
            math.sqrt(statistics.fmean(value * value for value in values))
            if values else None
        ),
    }
    if signed:
        result['p95_abs'] = _percentile(
            [abs(value) for value in values], 0.95
        )
    return result


def _histogram(rows, key, include_empty=False):
    counts = {}
    for row in rows:
        value = str(row.get(key, '')).strip()
        if value or include_empty:
            counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items()))


def _motion_errors(samples):
    along = []
    cross = []
    speeds = []
    for index, sample in enumerate(samples):
        left = samples[max(index - 1, 0)]
        right = samples[min(index + 1, len(samples) - 1)]
        delta_time = right['stamp'] - left['stamp']
        delta_x = right['truth'][0] - left['truth'][0]
        delta_y = right['truth'][1] - left['truth'][1]
        distance = math.hypot(delta_x, delta_y)
        if delta_time <= 1e-9 or distance <= 1e-6:
            continue
        direction_x = delta_x / distance
        direction_y = delta_y / distance
        error_x, error_y, _ = sample['error']
        along.append(error_x * direction_x + error_y * direction_y)
        cross.append(-error_x * direction_y + error_y * direction_x)
        speeds.append(distance / delta_time)
    return {
        'count': len(speeds),
        'speed_median': statistics.median(speeds) if speeds else None,
        'along': _metrics(along, signed=True),
        'cross': _metrics(cross, signed=True),
    }


def analyze_vision_csv(path):
    """Analyze one vision event CSV without changing or imputing samples."""
    path = Path(path)
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))

    samples = []
    for row in rows:
        if not (
            _is_true(row.get('valid'))
            and _is_true(row.get('truth_available'))
        ):
            continue
        stamp = _finite(row, 'measurement_stamp')
        truth = tuple(_finite(row, f'truth_{axis}') for axis in 'xyz')
        error = tuple(_finite(row, f'error_{axis}') for axis in 'xyz')
        if stamp is None or None in truth or None in error:
            continue
        samples.append({
            'stamp': stamp,
            'truth': truth,
            'error': error,
            'error_3d': (
                _finite(row, 'position_3d_error')
                or math.sqrt(sum(value * value for value in error))
            ),
            'age': _finite(row, 'observation_age'),
            'skew': _finite(row, 'rgb_depth_acquisition_skew'),
        })
    samples.sort(key=lambda sample: sample['stamp'])

    skew_groups = {}
    for label, predicate in (
        ('zero', lambda value: abs(value) <= 1e-9),
        ('nonzero', lambda value: abs(value) > 1e-9),
    ):
        selected = [
            sample for sample in samples
            if sample['skew'] is not None and predicate(sample['skew'])
        ]
        skew_groups[label] = {
            'count': len(selected),
            'signed_error': {
                axis: _metrics(
                    [sample['error'][index] for sample in selected],
                    signed=True,
                )
                for index, axis in enumerate('xyz')
            },
            'position_3d_error': _metrics(
                [sample['error_3d'] for sample in selected]
            ),
        }

    return {
        'path': str(path),
        'observations': len(rows),
        'valid_truth_observations': len(samples),
        'signed_error': {
            axis: _metrics(
                [sample['error'][index] for sample in samples],
                signed=True,
            )
            for index, axis in enumerate('xyz')
        },
        'position_3d_error': _metrics([
            sample['error_3d'] for sample in samples
        ]),
        'observation_age': _metrics([
            sample['age'] for sample in samples if sample['age'] is not None
        ]),
        'motion': _motion_errors(samples),
        'skew_groups': skew_groups,
        'clock_status_histogram': _histogram(rows, 'image_clock_status'),
        'rejection_histogram': _histogram(rows, 'rejection_reason'),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Analyze UAV-USV vision event CSV files.',
    )
    parser.add_argument('vision_csv', nargs='+', type=Path)
    parser.add_argument('--indent', type=int, default=2)
    arguments = parser.parse_args(argv)
    reports = [analyze_vision_csv(path) for path in arguments.vision_csv]
    print(json.dumps(reports, indent=arguments.indent, ensure_ascii=False))


if __name__ == '__main__':
    main()

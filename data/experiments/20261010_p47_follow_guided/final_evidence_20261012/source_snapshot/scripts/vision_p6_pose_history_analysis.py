#!/usr/bin/env python3
"""Summarize evaluation-only pose-history availability from a capture CSV."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path


def finite(row, name):
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def distribution(values):
    values = sorted(value for value in values if value is not None)
    if not values:
        return {'count': 0, 'p50': None, 'p95': None, 'max': None}

    def percentile(fraction):
        index = (len(values) - 1) * fraction
        left = int(index)
        right = min(left + 1, len(values) - 1)
        return values[left] + (values[right] - values[left]) * (index - left)

    return {
        'count': len(values), 'p50': percentile(0.5),
        'p95': percentile(0.95), 'max': values[-1],
    }


def unique_stamp_steps(rows, name):
    """Observed stamp steps at image rate; not the PX4 publish cadence."""
    stamps = sorted({value for row in rows
                     if (value := finite(row, name)) is not None})
    return distribution([right - left for left, right
                         in zip(stamps, stamps[1:]) if right > left])


def summarize_group(rows):
    differences = {
        'position_future_gap': ('measurement_stamp',
                                'position_history_end_stamp'),
        'attitude_future_gap': ('measurement_stamp',
                                'attitude_history_end_stamp'),
        'position_latest_age': ('measurement_stamp',
                                'position_mapped_stamp'),
        'attitude_latest_age': ('measurement_stamp',
                                'attitude_mapped_stamp'),
    }
    result = {'count': len(rows)}
    for name, (later, earlier) in differences.items():
        result[name] = distribution([
            finite(row, later) - finite(row, earlier)
            for row in rows
            if finite(row, later) is not None
            and finite(row, earlier) is not None
        ])
    for kind in ('position', 'attitude'):
        result[f'{kind}_source_observed_step'] = unique_stamp_steps(
            rows, f'{kind}_source_stamp')
        result[f'{kind}_mapped_observed_step'] = unique_stamp_steps(
            rows, f'{kind}_mapped_stamp')
        result[f'{kind}_bracket_sample_interval'] = distribution([
            finite(row, f'px4_{kind}_interval') for row in rows
        ])
    return result


def analyze(rows):
    valid = [row for row in rows if row.get('valid', '').lower() == 'true']
    after = [row for row in rows if row.get('rejection_reason')
             == 'POSITION_TIMESTAMP_AFTER_HISTORY']
    total = len(rows)
    reasons = Counter(row.get('rejection_reason') or 'VALID' for row in rows)
    return {
        'total_observations': total,
        'valid_count': len(valid),
        'valid_rate': len(valid) / total if total else None,
        'rejection_reason_counts': dict(sorted(reasons.items())),
        'position_timestamp_after_history_count': len(after),
        'position_timestamp_after_history_rate': (
            len(after) / total if total else None
        ),
        'note': ('Observed source/mapped stamp steps are sampled at image '
                 'rate. Bracket intervals directly measure adjacent PX4 '
                 'pose samples where independent queries are available.'),
        'valid': summarize_group(valid),
        'position_timestamp_after_history': summarize_group(after),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.csv_path.open(newline='', encoding='utf-8') as stream:
        result = analyze(list(csv.DictReader(stream)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(args.output)


if __name__ == '__main__':
    main()

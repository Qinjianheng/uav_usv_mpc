#!/usr/bin/env python3
"""
Diagnose visual position bias offline, without fitting or online correction.

Heading references must already be causal, in the same NED frame, and aligned
with the image acquisition measurement_stamp. This program does not infer a
reference from target truth direction, PX4 quaternions, or receipt times.
All angles are radians, positions/ranges metres, and timestamps seconds.
"""

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import csv
import json
import math
from pathlib import Path


MAX_MATCH_TOLERANCE = 1e-6


def _number(row, key):
    try:
        value = float(row.get(key, ''))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _true(value):
    return str(value).strip().lower() in ('true', '1', '1.0', 'yes')


def _mean(values):
    if not values:
        return None
    return math.fsum(value / len(values) for value in values)


def _rmse(values):
    if not values or not all(math.isfinite(value) for value in values):
        return None
    scale = max(abs(value) for value in values)
    if scale == 0.0:
        return 0.0
    return scale * math.sqrt(
        math.fsum((value / scale) ** 2 for value in values) / len(values)
    )


def _summary(values):
    return {'mean': _mean(values), 'rmse': _rmse(values)}


def _correlation(first, second):
    if len(first) < 2:
        return None
    first_scale = max(abs(value) for value in first)
    second_scale = max(abs(value) for value in second)
    if first_scale == 0.0 or second_scale == 0.0:
        return None
    first = [value / first_scale for value in first]
    second = [value / second_scale for value in second]
    first_mean, second_mean = _mean(first), _mean(second)
    centered = [(a - first_mean, b - second_mean)
                for a, b in zip(first, second)]
    left = math.fsum(a * a for a, _ in centered)
    right = math.fsum(b * b for _, b in centered)
    if left <= 0.0 or right <= 0.0:
        return None
    covariance = math.fsum(a * b for a, b in centered)
    return max(-1.0, min(1.0, covariance / math.sqrt(left * right)))


def _bearing(row):
    if not _true(row.get('geometry_diagnostics_enabled')):
        return None
    u, fx, cx = (_number(row, key) for key in (
        'mask_centroid_u', 'camera_fx', 'camera_cx'))
    if None in (u, fx, cx) or fx <= 0.0:
        return None
    return math.atan((u - cx) / fx)


def _heading_diagnostic(error_rows, heading_rows, tolerance):
    provided = heading_rows is not None
    references = []
    for row in heading_rows or ():
        stamp, px4, reference = (_number(row, key) for key in (
            'measurement_stamp', 'px4_heading', 'reference_heading'))
        if None not in (stamp, px4, reference) and stamp > 0.0:
            references.append((stamp, px4, reference))
    references.sort()
    stamps = [item[0] for item in references]
    yaw_errors, scalar_proxies, predicted_x, predicted_y = [], [], [], []
    actual_x, actual_y, residuals = [], [], []
    unmatched = ambiguous = missing_geometry = matched = 0
    for row, error_x, error_y in error_rows:
        stamp = _number(row, 'measurement_stamp')
        matches = references[bisect_left(stamps, stamp - tolerance):
                             bisect_right(stamps, stamp + tolerance)]
        matches = [item for item in matches
                   if abs(item[0] - stamp) <= tolerance]
        if not matches:
            unmatched += 1
            continue
        if len(matches) != 1:
            ambiguous += 1
            continue
        matched += 1
        geometry = [_number(row, key) for key in (
            'target_range', 'position_x', 'position_y',
            'interpolated_uav_x', 'interpolated_uav_y')]
        if (not _true(row.get('geometry_diagnostics_enabled'))
                or None in geometry or geometry[0] <= 0.0):
            missing_geometry += 1
            continue
        target_range, x, y, uav_x, uav_y = geometry
        _, px4_heading, reference_heading = matches[0]
        period = 2.0 * math.pi
        delta = ((px4_heading % period) - (reference_heading % period)
                 + math.pi) % period - math.pi
        relative_x, relative_y = x - uav_x, y - uav_y
        cosine, sine = math.cos(delta), math.sin(delta)
        # estimated_rel - Rz(-delta) * estimated_rel; no fitted offset.
        induced_x = relative_x - (cosine * relative_x + sine * relative_y)
        induced_y = relative_y - (-sine * relative_x + cosine * relative_y)
        if not all(math.isfinite(value) for value in (induced_x, induced_y)):
            missing_geometry += 1
            continue
        yaw_errors.append(delta)
        scalar_proxies.append(target_range * sine)
        predicted_x.append(induced_x)
        predicted_y.append(induced_y)
        actual_x.append(error_x)
        actual_y.append(error_y)
        residuals.append(math.hypot(error_x - induced_x, error_y - induced_y))
    count = len(yaw_errors)
    return {
        'status': 'evaluated_diagnostic_only' if count else 'not_evaluated',
        'reason': '' if count else 'insufficient_same_time_reference',
        'reference_source': (
            'supplied_acquisition_aligned_csv' if provided else 'none'),
        'reference_requirements': (
            'causal acquisition-aligned headings in the same NED frame'),
        'match_tolerance_s': tolerance,
        'reference_rows': len(heading_rows or ()),
        'valid_reference_rows': len(references),
        'matched_reference_rows': matched,
        'unmatched_rows': unmatched,
        'ambiguous_reference_rows': ambiguous,
        'missing_geometry_rows': missing_geometry,
        'sample_count': count,
        'yaw_error_rad': _summary(yaw_errors),
        'range_sin_yaw_error_m': _summary(scalar_proxies),
        'signed_ned_error_m': {
            'x_mean': _mean(predicted_x), 'y_mean': _mean(predicted_y),
            'x_error_correlation': _correlation(predicted_x, actual_x),
            'y_error_correlation': _correlation(predicted_y, actual_y),
        },
        'matched_raw_horizontal_rmse_m': _rmse([
            math.hypot(x, y) for x, y in zip(actual_x, actual_y)]),
        'residual_horizontal_rmse_m': _rmse(residuals),
        'root_cause': 'not_confirmed',
        'interpretation': (
            'correlation and residuals are offline diagnostics, '
            'not causal proof'),
    }


def _embedded_references(rows):
    references = []
    for row in rows:
        if not (
            _true(row.get('geometry_diagnostics_enabled'))
            and _true(row.get('gazebo_uav_diagnostics_enabled'))
            and row.get('heading_diagnostics_status') == 'VALID'
            and row.get('heading_reference_source') == 'gazebo_uav_model_pose'
            and row.get('uav_pose_status') in ('EXACT', 'INTERPOLATED')
        ):
            continue
        stamp, query, left, right = (_number(row, key) for key in (
            'measurement_stamp', 'uav_pose_query_ros_stamp',
            'uav_pose_left_ros_stamp', 'uav_pose_right_ros_stamp'))
        if (None in (stamp, query, left, right) or stamp <= 0.0
                or abs(query - stamp) > MAX_MATCH_TOLERANCE
                or not left - MAX_MATCH_TOLERANCE <= stamp
                <= right + MAX_MATCH_TOLERANCE):
            continue
        references.append(row)
    return references


def _vector_layer(rows, prefix, axes, units='m'):
    samples = []
    for row in rows:
        if not _true(row.get('geometry_diagnostics_enabled')):
            continue
        values = tuple(_number(row, prefix + '_' + axis) for axis in axes)
        if None not in values:
            samples.append(values)
    return {
        'status': 'evaluated_diagnostic_only' if samples else 'not_evaluated',
        'sample_count': len(samples), 'units': units,
        'norm_rmse': _rmse([math.hypot(*value) for value in samples]),
        'signed_axes': {axis: _summary([value[i] for value in samples])
                        for i, axis in enumerate(axes)},
    }


def _geometry_layers(valid_rows):
    layers = {
        'projection_pixel': _vector_layer(
            valid_rows, 'projection_error', ('u', 'v'), 'px'),
        'camera_center': _vector_layer(
            valid_rows, 'camera_center_error', ('x', 'y', 'z')),
        'body_flu': _vector_layer(
            valid_rows, 'body_flu_error', ('x', 'y', 'z')),
        'vision_to_entity': _vector_layer(
            valid_rows, 'vision_to_entity', ('x', 'y', 'z')),
        'entity_to_truth': _vector_layer(
            valid_rows, 'entity_to_truth', ('x', 'y', 'z')),
        'independent_physical_projection': _vector_layer(
            valid_rows, 'physical_projection_error', ('u', 'v'), 'px'),
        'independent_physical_camera': _vector_layer(
            valid_rows, 'physical_camera_center_error', ('x', 'y', 'z')),
        'attitude_only_counterfactual': _vector_layer(
            [row for row in valid_rows
             if row.get('attitude_counterfactual_status')
             == 'PX4_POSITION_HELD_FIXED'],
            'attitude_counterfactual_error', ('x', 'y', 'z')),
    }
    layers['attitude_only_counterfactual']['conditioning'] = (
        'PX4 position held fixed; independent UAV attitude replaces PX4 '
        'attitude; model-origin position equivalence not established')
    evaluated = any(value['sample_count'] for value in layers.values())
    return {
        'status': (
            'evaluated_diagnostic_only' if evaluated else 'not_evaluated'),
        'reason': '' if evaluated else 'insufficient_geometry_fields',
        'conditioning': (
            'projection_pixel/camera_center/body_flu expected values use PX4 '
            'position and attitude; residuals cannot alone distinguish pose '
            'error from mask/depth/extrinsics. Rotated camera/body residual '
            'norms are coupled and are not independent proof.'),
        'independent_physical_provenance': (
            'legacy physical_* columns require their original capture config '
            'and exact camera/model pose bracket audit; these statistics '
            'alone do not certify alignment'),
        'root_cause': 'not_confirmed', **layers,
    }


def analyze_rows(rows, heading_rows=None, match_tolerance=MAX_MATCH_TOLERANCE):
    """Return finite JSON-ready diagnostics from acquisition-timed CSV rows."""
    tolerance = float(match_tolerance)
    if (not math.isfinite(tolerance)
            or not 0.0 <= tolerance <= MAX_MATCH_TOLERANCE):
        raise ValueError(
            'heading match tolerance must be between 0 and 1e-6 seconds')
    rows = list(rows)
    heading_rows = list(heading_rows) if heading_rows is not None else None
    valid_rows = [row for row in rows if _true(row.get('observation_valid'))]
    error_rows, bearings = [], []
    for row in valid_rows:
        bearing = _bearing(row)
        if bearing is not None:
            bearings.append(bearing)
        stamp = _number(row, 'measurement_stamp')
        error_x, error_y = _number(row, 'error_x'), _number(row, 'error_y')
        if (_true(row.get('truth_available'))
                and stamp is not None and stamp > 0.0
                and error_x is not None and error_y is not None):
            error_rows.append((row, error_x, error_y))
    embedded = _embedded_references(valid_rows) if heading_rows is None else []
    references = (
        heading_rows if heading_rows is not None else (embedded or None))
    heading_diagnostic = _heading_diagnostic(error_rows, references, tolerance)
    if embedded:
        heading_diagnostic['reference_source'] = (
            'embedded_gazebo_uav_model_pose')
    reasons = Counter(str(row.get('rejection_reason', '')).strip()
                      for row in rows)
    return {
        'offline_only': True,
        'online_correction': 'none',
        'root_cause': 'not_confirmed',
        'units': {'position_range_error': 'm', 'bearing_heading': 'rad',
                  'timestamp': 's'},
        'total_rows': len(rows),
        'valid_observation_rows': len(valid_rows),
        'valid_rate': len(valid_rows) / len(rows) if rows else None,
        'position_timestamp_after_history_rows': sum(
            count for reason, count in reasons.items()
            if 'POSITION_TIMESTAMP_AFTER_HISTORY' in reason),
        'rejection_reason_counts': dict(reasons),
        'raw_error': {
            'sample_count': len(error_rows),
            'selection': (
                'observation_valid and truth_available with positive '
                'acquisition stamp and finite XY error'),
            'horizontal_rmse_m': _rmse([
                math.hypot(x, y) for _, x, y in error_rows]),
            'y_mean_m': _mean([y for _, _, y in error_rows]),
            'x_mean_m': _mean([x for _, x, _ in error_rows]),
        },
        'image_bearing': {
            'status': 'evaluated' if bearings else 'not_evaluated',
            'reason': '' if bearings else 'insufficient_geometry_fields',
            'sample_count': len(bearings),
            'mean_rad': _mean(bearings), 'rmse_rad': _rmse(bearings),
            'definition': 'atan((mask_centroid_u - camera_cx) / camera_fx)',
        },
        'geometry_layers': _geometry_layers(valid_rows),
        'heading_diagnostic': heading_diagnostic,
    }


def _read_csv(path):
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def main(argv=None):
    """Read input CSVs and write diagnostic JSON without middleware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision', required=True, type=Path)
    parser.add_argument('--heading-reference', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    headings = (_read_csv(args.heading_reference)
                if args.heading_reference else None)
    report = analyze_rows(_read_csv(args.vision), headings)
    report['inputs'] = {
        'vision': str(args.vision.resolve()),
        'heading_reference': (str(args.heading_reference.resolve())
                              if args.heading_reference else None),
    }
    serialized = json.dumps(
        report, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    if args.output:
        args.output.write_text(serialized, encoding='utf-8')
    else:
        print(serialized, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

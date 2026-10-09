"""
Summarize interception outcomes and terminal-approach geometry from CSV logs.

This is the P0 batch-evaluation tool.  It reads every experiment CSV in a
directory, counts success and failure reasons, and decomposes the terminal
approach of each run into the geometry that actually decides the result:

* planner usage (MINCO versus direct-pursuit fallback),
* the closest-approach geometry (horizontal, vertical, and 3D distance),
* the altitude-tracking error against ``guidance_altitude_reference``,
* the radial and perpendicular relative velocity near closest approach,
* whether the sea-contact event happened before the capture sphere.

The tool only reads logs.  It never publishes, commands, or writes anything.
Cause attribution stays with the reader: a run that reaches the sea with a
large horizontal gap has a horizontal-closure problem, while a run that
reaches the sea with a small horizontal gap has a vertical-descent problem.
"""

import argparse
import csv
import math
from pathlib import Path


CAPTURE_RADIUS_DEFAULT = 0.25
TERMINAL_RADIUS_DEFAULT = 3.0
TERMINAL_PURSUIT_MODES = (
    'PURSUIT',
    'TERMINAL_PURSUIT',
    'FAR_GUIDANCE',
    'MINCO_SAFE_WAIT',
)


def finite_value(row, field):
    """Return one finite float from a CSV row, or None."""
    text = row.get(field)
    if text in (None, ''):
        return None
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def finite_column(rows, field):
    """Return all finite floats from one CSV column."""
    values = []
    for row in rows:
        value = finite_value(row, field)
        if value is not None:
            values.append(value)
    return values


def finite_column_alias(rows, *fields):
    """Read the first available numeric field from a log schema."""
    for field in fields:
        values = finite_column(rows, field)
        if values:
            return values
    return []


def percentile(values, fraction):
    """Return a nearest-rank percentile, or None for an empty sequence."""
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    index = min(
        max(int(math.ceil(float(fraction) * len(ordered))) - 1, 0),
        len(ordered) - 1,
    )
    return ordered[index]


def load_intercept_rows(path):
    """Return the intercept-phase rows of one experiment CSV."""
    with Path(path).open(newline='', encoding='utf-8') as stream:
        rows = [
            row
            for row in csv.DictReader(stream)
            if row.get('phase') == 'intercept'
        ]
    return rows


def planner_field(rows):
    """Return the planner-type column name used by this log version."""
    if not rows:
        return None
    for name in ('trajectory_planner', 'planner'):
        if name in rows[0]:
            return name
    return None


def radial_components(row):
    """
    Return range, radial closing rate, and perpendicular speed.

    The radial closing rate is positive while the range is shrinking.
    """
    target_x = finite_value(row, 'target_x')
    target_y = finite_value(row, 'target_y')
    uav_x = finite_value(row, 'uav_x')
    uav_y = finite_value(row, 'uav_y')
    if None in (target_x, target_y, uav_x, uav_y):
        return None
    relative_x = target_x - uav_x
    relative_y = target_y - uav_y
    horizontal = math.hypot(relative_x, relative_y)
    if horizontal < 1e-9:
        return 0.0, 0.0, 0.0
    line_x = relative_x / horizontal
    line_y = relative_y / horizontal
    relative_vx = (finite_value(row, 'uav_vx') or 0.0) - (
        finite_value(row, 'target_vx') or 0.0
    )
    relative_vy = (finite_value(row, 'uav_vy') or 0.0) - (
        finite_value(row, 'target_vy') or 0.0
    )
    closing = relative_vx * line_x + relative_vy * line_y
    perpendicular = abs(-relative_vx * line_y + relative_vy * line_x)
    return horizontal, closing, perpendicular


def planner_histogram(rows):
    """Count rows per planner type."""
    field = planner_field(rows)
    if field is None:
        return {}
    histogram = {}
    for row in rows:
        key = (row.get(field) or '').strip() or 'UNKNOWN'
        histogram[key] = histogram.get(key, 0) + 1
    return histogram


def text_histogram(rows, field):
    """Count non-empty categorical values in one field."""
    histogram = {}
    for row in rows:
        value = (row.get(field) or '').strip()
        if value:
            histogram[value] = histogram.get(value, 0) + 1
    return histogram


def minco_share(rows):
    """Return the fraction of intercept rows planned by MINCO."""
    histogram = planner_histogram(rows)
    total = sum(histogram.values())
    if not total:
        return None
    minco = sum(
        count
        for key, count in histogram.items()
        if key.startswith('MINCO')
    )
    return minco / total


def terminal_metrics(rows, capture_radius=CAPTURE_RADIUS_DEFAULT):
    """Return closest-approach and terminal-tracking metrics."""
    if not rows:
        return None
    best_index = None
    best_horizontal = None
    for index, row in enumerate(rows):
        horizontal = finite_value(row, 'horizontal_distance')
        if horizontal is None:
            continue
        if best_horizontal is None or horizontal < best_horizontal:
            best_horizontal = horizontal
            best_index = index
    if best_index is None:
        return None

    closest = rows[best_index]
    vertical = abs(finite_value(closest, 'vertical_error') or 0.0)
    distance = finite_value(closest, 'distance')
    horizontal_allowance = math.sqrt(
        max(capture_radius * capture_radius - vertical * vertical, 0.0)
    )
    radial = radial_components(closest)

    # Sea contact versus capture: the last row carries the verdict.
    final = rows[-1]
    altitude_reserve = [
        (finite_value(row, 'uav_z') or 0.0)
        - (finite_value(row, 'target_z') or 0.0)
        for row in rows
    ]
    tracking_error = []
    reference_delta = []
    previous_reference = None
    for row in rows:
        reference = finite_value(row, 'guidance_altitude_reference')
        altitude = finite_value(row, 'uav_z')
        if reference is not None and altitude is not None:
            tracking_error.append(abs(reference - altitude))
        if reference is not None and previous_reference is not None:
            reference_delta.append(abs(reference - previous_reference))
        if reference is not None:
            previous_reference = reference

    closing_rates = [
        radial_components(row)[1]
        for row in rows
        if radial_components(row) is not None
    ]
    perpendicular_speeds = [
        radial_components(row)[2]
        for row in rows
        if radial_components(row) is not None
    ]
    control_steps = finite_column(rows, 'sim_dt')
    if not control_steps:
        control_steps = finite_column(rows, 'control_dt')
    wall_steps = finite_column(rows, 'wall_dt')
    callback_times = finite_column(rows, 'callback_compute_time')
    planner_times = finite_column(rows, 'planner_compute_time')
    horizontal_min_times = finite_column_alias(
        rows,
        't_horizontal_min',
        'horizontal_min_time',
    )
    vertical_min_times = finite_column_alias(
        rows,
        't_vertical_min',
        'vertical_min_time',
    )
    retained_plan_ages = finite_column(rows, 'retained_plan_age')
    planner_counts = planner_histogram(rows)
    planner_total = sum(planner_counts.values())
    held_samples = sum(
        count
        for planner, count in planner_counts.items()
        if planner.endswith('_HOLD')
    )
    planner_attempts = sum(
        1 for row in rows if finite_value(row, 'planner_attempted') == 1.0
    )
    planner_successes = sum(
        1 for row in rows if finite_value(row, 'planner_succeeded') == 1.0
    )
    prediction_errors = {}
    for model in ('guidance', 'kf'):
        for label in ('0p5', '1p0', '2p0'):
            field = f'{model}_prediction_{label}_error'
            values = finite_column(rows, field)
            prediction_errors[field] = {
                'count': len(values),
                'rmse': (
                    math.sqrt(
                        sum(value * value for value in values) / len(values)
                    )
                    if values else None
                ),
                'p95': percentile(values, 0.95),
            }

    return {
        'row_count': len(rows),
        'outcome': (final.get('outcome') or '').strip(),
        'failure_reason': (final.get('failure_reason') or '').strip(),
        'closest_horizontal': best_horizontal,
        'closest_vertical': vertical,
        'closest_distance': distance,
        'closest_horizontal_allowance': horizontal_allowance,
        'closest_planner': (
            (closest.get(planner_field(rows)) or '').strip()
            if planner_field(rows)
            else ''
        ),
        'closest_time': finite_value(closest, 'intercept_elapsed_time'),
        'closest_closing_rate': radial[1] if radial else None,
        'closest_perpendicular_speed': radial[2] if radial else None,
        'final_horizontal': finite_value(final, 'horizontal_distance'),
        'final_altitude': finite_value(final, 'uav_z'),
        'final_vertical': abs(finite_value(final, 'vertical_error') or 0.0),
        'minimum_altitude_reserve': min(altitude_reserve),
        'maximum_altitude_tracking_error': (
            max(tracking_error) if tracking_error else None
        ),
        'maximum_reference_step': (
            max(reference_delta) if reference_delta else None
        ),
        'minimum_closing_rate': min(closing_rates) if closing_rates else None,
        'mean_perpendicular_speed': (
            sum(perpendicular_speeds) / len(perpendicular_speeds)
            if perpendicular_speeds
            else None
        ),
        'minco_share': minco_share(rows),
        'planner_histogram': planner_counts,
        'planner_failure_histogram': text_histogram(
            rows,
            'planner_failure_reason',
        ),
        'guidance_phase_histogram': text_histogram(rows, 'guidance_phase'),
        'sea_safety_histogram': text_histogram(rows, 'sea_safety_state'),
        'planner_attempts': planner_attempts,
        'planner_successes': planner_successes,
        'planner_success_rate': (
            planner_successes / planner_attempts if planner_attempts else None
        ),
        'prediction_errors': prediction_errors,
        'held_plan_share': (
            held_samples / planner_total if planner_total else None
        ),
        'mean_control_dt': (
            sum(control_steps) / len(control_steps)
            if control_steps
            else None
        ),
        'p95_control_dt': percentile(control_steps, 0.95),
        'p95_wall_dt': percentile(wall_steps, 0.95),
        'p95_callback_compute_time': percentile(callback_times, 0.95),
        'p95_planner_compute_time': percentile(planner_times, 0.95),
        'minimum_horizontal_min_time': (
            min(horizontal_min_times) if horizontal_min_times else None
        ),
        'minimum_vertical_min_time': (
            min(vertical_min_times) if vertical_min_times else None
        ),
        'maximum_retained_plan_age': (
            max(retained_plan_ages) if retained_plan_ages else None
        ),
    }


def analyze_file(path, capture_radius=CAPTURE_RADIUS_DEFAULT):
    """Return terminal metrics for one experiment CSV."""
    rows = load_intercept_rows(path)
    metrics = terminal_metrics(rows, capture_radius)
    if metrics is None:
        return None
    metrics['file'] = Path(path).name
    return metrics


def analyze_directory(directory, capture_radius=CAPTURE_RADIUS_DEFAULT):
    """Return metrics for every experiment CSV in a directory."""
    paths = sorted(Path(directory).expanduser().glob('*.csv'))
    results = []
    for path in paths:
        metrics = analyze_file(path, capture_radius)
        if metrics is not None:
            results.append(metrics)
    return results


def outcome_counts(results):
    """Count runs per outcome and failure reason."""
    counts = {}
    for metrics in results:
        if not metrics['outcome']:
            continue
        key = (metrics['outcome'], metrics['failure_reason'])
        counts[key] = counts.get(key, 0) + 1
    return counts


def format_optional(value, digits=2):
    """Format a possibly missing float."""
    if value is None:
        return '  n/a'
    return f'{value:.{digits}f}'


def print_directory_summary(results):
    """Print the batch table and aggregate counts."""
    print(f'{"file":<48}{"outcome":>9}{"reason":>20}'
          f'{"hd_min":>8}{"|ve|":>7}{"allow":>7}{"t_min":>7}'
          f'{"minco":>7}{"hold":>7}{"dt95":>7}{"alt_err":>8}')
    for metrics in results:
        ending = f"{metrics['outcome']}/{metrics['failure_reason']}"
        if ending == '/':
            ending = '(unfinished)'
        print(
            f"{metrics['file'][-48:]:<48}"
            f"{metrics['outcome']:>9}"
            f"{metrics['failure_reason']:>20}"
            f"{format_optional(metrics['closest_horizontal']):>8}"
            f"{format_optional(metrics['closest_vertical']):>7}"
            f"{format_optional(metrics['closest_horizontal_allowance']):>7}"
            f"{format_optional(metrics['closest_time'], 1):>7}"
            f"{format_optional(metrics['minco_share'], 2):>7}"
            f"{format_optional(metrics['held_plan_share'], 2):>7}"
            f"{format_optional(metrics['p95_control_dt'], 3):>7}"
            f"{format_optional(metrics['maximum_altitude_tracking_error']):>8}"
        )
    print()
    print('Outcome counts:')
    for (outcome, reason), count in sorted(
        outcome_counts(results).items(),
        key=lambda item: (-item[1], item[0]),
    ):
        label = f'{outcome}/{reason}' if reason else outcome
        print(f'  {label:<32} {count:4d}')
    print()
    print('Notes: hd_min = closest horizontal separation')
    print('       |ve| = vertical error at that instant')
    print('       allow = horizontal separation still inside the capture')
    print('               sphere given |ve| (0 means capture was')
    print('               geometrically impossible at that instant)')
    print('       minco = fraction of intercept rows planned by MINCO')
    print('       hold = fraction executing a retained trajectory')
    print('       dt95 = 95th-percentile measured control period')
    print('       alt_err = max |guidance_altitude_reference - uav_z|')


def print_terminal_detail(metrics, rows):
    """Print the terminal-approach table of one run."""
    print(f"Terminal approach: {metrics['file']}")
    header = (
        f"{'t':>6}{'range':>7}{'|ve|':>7}{'closing':>9}{'perp':>7}"
        f"{'ref':>8}{'uav_z':>8}{'ref-uz':>8}{'planner':>18}"
    )
    print(header)
    for row in rows[-20:]:
        radial = radial_components(row)
        reference = finite_value(row, 'guidance_altitude_reference')
        altitude = finite_value(row, 'uav_z')
        if None in (reference, altitude):
            altitude_offset = None
        else:
            altitude_offset = reference - altitude
        planned_time = finite_value(row, 'intercept_elapsed_time')
        vertical_error = abs(finite_value(row, 'vertical_error') or 0.0)
        planner = ''
        field = planner_field(rows)
        if field:
            planner = (row.get(field) or '').strip()
        print(
            f'{format_optional(planned_time, 2):>6}'
            f'{format_optional(finite_value(row, "horizontal_distance")):>7}'
            f'{format_optional(vertical_error):>7}'
            f'{format_optional(radial[1] if radial else None):>9}'
            f'{format_optional(radial[2] if radial else None):>7}'
            f'{format_optional(reference):>8}'
            f'{format_optional(altitude):>8}'
            f'{format_optional(altitude_offset):>8}'
            f'{planner[:18]:>18}'
        )


def main(args=None):
    """Run batch outcome analysis from the command line."""
    parser = argparse.ArgumentParser(
        description='Summarize interception outcomes and terminal geometry.',
    )
    parser.add_argument(
        'log_directory',
        help='directory of trajectory_impact_sim CSV logs',
    )
    parser.add_argument(
        '--capture-radius',
        type=float,
        default=CAPTURE_RADIUS_DEFAULT,
        help='capture radius in metres (default: 0.25)',
    )
    parser.add_argument(
        '--detail',
        default=None,
        help='file name (or unique suffix) to print a terminal table for',
    )
    parsed = parser.parse_args(args=args)

    results = analyze_directory(parsed.log_directory, parsed.capture_radius)
    if not results:
        print(f'No experiment CSV found in {parsed.log_directory}')
        return
    print_directory_summary(results)

    if parsed.detail:
        matching = [
            metrics
            for metrics in results
            if parsed.detail in metrics['file']
        ]
        if not matching:
            print(f'No log matching {parsed.detail!r}')
            return
        path = Path(parsed.log_directory).expanduser() / matching[-1]['file']
        print()
        print_terminal_detail(matching[-1], load_intercept_rows(path))


if __name__ == '__main__':
    main()

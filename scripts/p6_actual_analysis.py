#!/usr/bin/env python3
"""Read actual P6 records and score bounded offline futures; never grant execution."""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np

HORIZONS = (.2, .5, .8, 1.2, 1.6, 2.4, 4.)
PRIMARY = '/planning/target_prediction'
SHADOW = '/planning/imm_target_prediction'
EPOCH_TOLERANCE = 1e-6


def sha(path):
    """Hash source evidence without changing it."""
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _array(value, shape=None):
    try:
        result = np.asarray(value, dtype=float)
        if not np.isfinite(result).all() or (shape is not None and result.shape != shape):
            return None
        return result
    except (TypeError, ValueError):
        return None


def _statistics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    result = dict(count=len(values), rmse=None, min=None, p50=None, p95=None, p99=None, max=None)
    if len(values):
        result.update(rmse=float(np.sqrt(np.mean(values ** 2))), min=float(values.min()),
                      max=float(values.max()),
                      **dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(
                          values, (50, 95, 99))))))
    return result


class BoundedTruth:
    """Interpolate recorded epochs only; reject gaps and ambiguous clock histories."""

    def __init__(self, rows, maximum_gap=.2):
        self.maximum_gap = maximum_gap
        self.clock_reversals = 0
        self.conflicting_duplicates = 0
        self.invalid_records = 0
        points = []
        ordered = sorted(enumerate(rows), key=lambda item: item[1].get('receipt', item[0]))
        for _, row in ordered:
            if not row.get('valid', True):
                continue
            p, v = _array(row.get('p'), (3,)), _array(row.get('v'), (3,))
            if not _finite(row.get('stamp')) or p is None or v is None:
                self.invalid_records += 1
                continue
            stamp = float(row['stamp'])
            if points and stamp < points[-1][0] - EPOCH_TOLERANCE:
                self.clock_reversals += 1
            if points and abs(stamp - points[-1][0]) <= EPOCH_TOLERANCE:
                if not (np.allclose(p, points[-1][1], atol=1e-10, rtol=0.)
                        and np.allclose(v, points[-1][2], atol=1e-10, rtol=0.)):
                    self.conflicting_duplicates += 1
                continue
            points.append((stamp, p, v))
        self.times = np.array([point[0] for point in points])
        self.positions = np.array([point[1] for point in points])
        self.velocities = np.array([point[2] for point in points])

    def at(self, epoch):
        if self.clock_reversals or self.conflicting_duplicates:
            return None, 'truth_clock_ambiguous'
        if not len(self.times) or not _finite(epoch):
            return None, 'truth_unavailable'
        if epoch < self.times[0] - EPOCH_TOLERANCE:
            return None, 'left_censored'
        if epoch > self.times[-1] + EPOCH_TOLERANCE:
            return None, 'right_censored'
        index = int(np.searchsorted(self.times, epoch))
        for candidate in (index, index - 1):
            if 0 <= candidate < len(self.times) and abs(
                    self.times[candidate] - epoch) <= EPOCH_TOLERANCE:
                return dict(p=self.positions[candidate], v=self.velocities[candidate]), ''
        if not 0 < index < len(self.times):
            return None, 'truth_unavailable'
        gap = self.times[index] - self.times[index - 1]
        if gap > self.maximum_gap:
            return None, 'interpolation_gap'
        weight = (epoch - self.times[index - 1]) / gap
        position = (1. - weight) * self.positions[index - 1] + weight * self.positions[index]
        velocity = (1. - weight) * self.velocities[index - 1] + weight * self.velocities[index]
        return dict(p=position, v=velocity), ''


def _coverage(rows, start, end, maximum_gap=1.5, edge=1.5):
    stamps = sorted(float(row['receipt']) for row in rows if _finite(row.get('receipt')))
    return bool(stamps and stamps[0] <= start + edge and stamps[-1] >= end - edge
                and (len(stamps) == 1 or np.max(np.diff(stamps)) <= maximum_gap))


def _position_metrics(rows, truth):
    positions, velocities, reasons = [], [], Counter()
    for row in rows:
        if not row.get('valid'):
            reasons['invalid_record'] += 1
            continue
        actual, reason = truth.at(row.get('stamp'))
        position = _array(row.get('p'), (3,))
        if actual is None or position is None:
            reasons[reason or 'malformed_position'] += 1
            continue
        positions.append(float(np.linalg.norm(position[:2] - actual['p'][:2])))
        velocity = _array(row.get('v'), (3,))
        if velocity is not None:
            velocities.append(float(np.linalg.norm(velocity - actual['v'])))
    return dict(attempts=len(rows), horizontal=_statistics(positions),
                velocity=_statistics(velocities), unscored_reasons=dict(reasons))


def _forecast_arrays(row):
    times = _array(row.get('times'))
    if (times is None or times.ndim != 1 or len(times) < 2
            or abs(times[0]) > 1e-9 or np.any(np.diff(times) <= 0.)):
        return None
    values = {name: _array(row.get(name), (len(times), 3)) for name in ('p', 'v', 'a')}
    if any(value is None for value in values.values()):
        return None
    return dict(times=times, **values)


def _at_forecast(arrays, horizon):
    if arrays is None or not arrays['times'][0] <= horizon <= arrays['times'][-1] + 1e-9:
        return None
    return {name: np.array([np.interp(horizon, arrays['times'], arrays[name][:, i])
                            for i in range(3)]) for name in ('p', 'v', 'a')}


def _psd(value, dimension=3, count=None):
    matrix = _array(value)
    expected = (dimension, dimension) if count is None else (count, dimension, dimension)
    if matrix is None or matrix.size != int(np.prod(expected)):
        return False
    matrix = matrix.reshape(expected)
    return bool(np.allclose(matrix, np.swapaxes(matrix, -1, -2), atol=1e-8, rtol=1e-8)
                and np.linalg.eigvalsh(matrix).min() >= -1e-8)


def _lease_valid(row):
    names = ('source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until')
    if not all(_finite(row.get(name)) for name in names):
        return False
    source, observation, generated, expiry = (float(row[name]) for name in names)
    return bool(observation > 0. and observation <= source + EPOCH_TOLERANCE
                and source <= generated + EPOCH_TOLERANCE
                and 0. < expiry - observation <= .125 + EPOCH_TOLERANCE
                and generated <= expiry + EPOCH_TOLERANCE
                and row.get('source') == 'tracking' and row.get('frame') == 'local_ned')


def _raw_observation_valid(row):
    times = ('stamp', 'received', 'processed', 'published', 'image_measurement_stamp')
    return bool(row.get('valid') and row.get('frame') == 'local_ned'
                and not any(token in str(row.get('source', '')).lower()
                            for token in ('truth', '/target/state', 'ground_truth'))
                and row.get('source') and all(_finite(row.get(name)) for name in times)
                and abs(row['image_measurement_stamp'] - row['stamp']) <= EPOCH_TOLERANCE
                and _array(row.get('p'), (3,)) is not None
                and _psd(row.get('covariance'))
                and all(isinstance(row.get(name), int) for name in (
                    'image_clock_reset_count', 'px4_clock_reset_count')))


class _ObservationIndex:
    """Bounded epoch lookup preserving aliases and rejecting ambiguous acquisitions."""

    def __init__(self, rows):
        self.buckets = defaultdict(list)
        for row in rows:
            if _raw_observation_valid(row):
                self.buckets[round(row['stamp'] / EPOCH_TOLERANCE)].append(row)

    def match(self, prediction, diagnostic):
        if (diagnostic is None or not _finite(prediction.get('observation_stamp'))
                or not _finite(diagnostic.get('raw_input_stamp'))
                or not diagnostic.get('observation_source')):
            return None, 'RAW_IDENTITY_MISSING'
        epoch = float(prediction['observation_stamp'])
        bucket = round(epoch / EPOCH_TOLERANCE)
        candidates = []
        # Large epoch division has its own rounding error. Two adjacent buckets
        # on each side cover that representation effect; actual differences
        # must still satisfy the original 1us tolerance, with no nearest choice.
        for key in range(bucket - 2, bucket + 3):
            for row in self.buckets.get(key, ()):
                if (abs(row['stamp'] - epoch) <= EPOCH_TOLERANCE
                        and abs(row['stamp'] - diagnostic['raw_input_stamp'])
                        <= EPOCH_TOLERANCE
                        and row['source'] == diagnostic['observation_source']
                        and row['frame'] == diagnostic.get('frame_id')
                        and [row['image_clock_reset_count'], row['px4_clock_reset_count']]
                        == diagnostic.get('clock_identity')):
                    candidates.append(row)
        if len(candidates) != 1:
            return None, ('RAW_OBSERVATION_AMBIGUOUS' if candidates
                          else 'RAW_OBSERVATION_NOT_FOUND')
        return candidates[0], 'MATCHED'


def _imm_contract(prediction, diagnostic, observation, arrays):
    if diagnostic is None or observation is None or arrays is None:
        return False, False, None
    identity = (diagnostic.get('valid') is True and _raw_observation_valid(observation)
                and prediction.get('model') == 'IMM_CV_CTRV_CTRA'
                and diagnostic.get('schema_version') == 'p6_imm_v1')
    for name, counterpart in (('mission', 'mission_id'), ('sequence', 'sequence_id')):
        identity = identity and prediction.get(name) == diagnostic.get(counterpart)
    for name in ('source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until'):
        identity = identity and _finite(prediction.get(name)) and _finite(
            diagnostic.get(name)) and abs(
            prediction[name] - diagnostic[name]) <= EPOCH_TOLERANCE
    clock_identity = [observation.get('image_clock_reset_count'),
                      observation.get('px4_clock_reset_count')]
    identity = (identity and diagnostic.get('frame_id') == prediction.get('frame')
                and diagnostic.get('source') == prediction.get('source')
                and diagnostic.get('model') == prediction.get('model')
                and diagnostic.get('clock_identity') == clock_identity
                and diagnostic.get('observation_source') == observation.get('source')
                and _finite(diagnostic.get('raw_input_stamp'))
                and abs(diagnostic['raw_input_stamp'] - prediction['observation_stamp'])
                <= EPOCH_TOLERANCE
                and abs(observation['stamp'] - diagnostic['raw_input_stamp'])
                <= EPOCH_TOLERANCE
                and abs(observation['stamp'] - prediction['observation_stamp'])
                <= EPOCH_TOLERANCE)
    probabilities = _array(diagnostic.get('model_probabilities'), (3,))
    times = _array(diagnostic.get('relative_times'), arrays['times'].shape)
    models = diagnostic.get('model_predictions', [])
    models = (models if isinstance(models, list)
              and all(isinstance(m, dict) for m in models) else [])
    identity = bool(identity and probabilities is not None and (probabilities >= 0.).all()
                    and abs(probabilities.sum() - 1.) <= 1e-6 and times is not None
                    and np.allclose(times, arrays['times'], atol=1e-9, rtol=0.)
                    and _array(diagnostic.get('state'), (7,)) is not None
                    and len(models) == 3
                    and {model.get('model') for model in models} == {'CV', 'CTRV', 'CTRA'})
    covariance_valid = _psd(diagnostic.get('state_covariance'), 7)
    for model in models:
        identity = identity and all(_array(model.get(name), arrays['p'].shape) is not None
                                    for name in ('positions', 'velocities', 'accelerations'))
        covariance_valid = covariance_valid and _psd(
            model.get('position_covariances'), count=len(arrays['times']))
    if identity and covariance_valid:
        model_by_name = {model['model']: model for model in models}
        ordered = [model_by_name[name] for name in ('CV', 'CTRV', 'CTRA')]
        identity = all(_finite(model.get('probability')) and abs(
            model['probability'] - probabilities[index]) <= 1e-8
            for index, model in enumerate(ordered))
        means = np.asarray([model['positions'] for model in ordered])
        fused = np.einsum('m,mni->ni', probabilities, means)
        differences = means - fused
        within = np.asarray([model['position_covariances'] for model in ordered]).reshape(
            3, len(arrays['times']), 3, 3)
        covariance = np.einsum('m,mnij->nij', probabilities, within) + np.einsum(
            'm,mni,mnj->nij', probabilities, differences, differences)
        message_covariance = _array(prediction.get('covariance'))
        identity = bool(identity and np.allclose(fused, arrays['p'], atol=1e-6, rtol=1e-9)
                        and message_covariance is not None
                        and message_covariance.size == covariance.size
                        and np.allclose(covariance, message_covariance.reshape(covariance.shape),
                                        atol=1e-6, rtol=1e-8))
    identity = bool(identity and 'compute_time_scope' in diagnostic)
    return identity, bool(covariance_valid), probabilities


def _stage_at(motion, source):
    if not motion or not _finite(source):
        return 'unknown'
    index = np.searchsorted([row[0] for row in motion], source + EPOCH_TOLERANCE) - 1
    return (str(motion[index][1].get('stage', 'unknown')) if index >= 0
            and 0. <= source - motion[index][0] <= .15 else 'unknown')


def _score_forecast(row, arrays, truth, stage):
    scores = []
    source_truth, source_reason = truth.at(row.get('source_stamp'))
    initial = _at_forecast(arrays, 0.)
    for horizon in HORIZONS:
        value = dict(topic=row.get('topic', 'unknown'), mission=row.get('mission'),
                     sequence=row.get('sequence'), source_stamp=row.get('source_stamp'),
                     observation_stamp=row.get('observation_stamp'), horizon=horizon,
                     stage=stage, scored=False, valid=bool(row.get('valid')))
        future_epoch = (float(row['source_stamp']) + horizon
                        if _finite(row.get('source_stamp')) else float('nan'))
        future_truth, reason = truth.at(future_epoch)
        future = _at_forecast(arrays, horizon)
        if not row.get('valid'):
            value.update(reason='invalid_forecast', invalid_reason=row.get('reason', 'unknown'))
        elif initial is None or future is None:
            value['reason'] = 'prediction_samples_unavailable'
        elif source_truth is None or future_truth is None:
            value['reason'] = source_reason if source_truth is None else reason
        else:
            initial_error = initial['p'][:2] - source_truth['p'][:2]
            initial_velocity_error = initial['v'][:2] - source_truth['v'][:2]
            total = future['p'][:2] - future_truth['p'][:2]
            increment = ((future['p'][:2] - initial['p'][:2])
                         - (future_truth['p'][:2] - source_truth['p'][:2]))
            cv_effect = initial_error + horizon * initial_velocity_error
            speeds = np.linalg.norm(future['v'][:2]), np.linalg.norm(future_truth['v'][:2])
            heading = (math.atan2(future['v'][1], future['v'][0])
                       - math.atan2(future_truth['v'][1], future_truth['v'][0]))
            value.update(
                scored=True, reason='', horizontal_error=float(np.linalg.norm(total)),
                velocity_error=float(np.linalg.norm(future['v'] - future_truth['v'])),
                heading_error_rad=math.atan2(math.sin(heading), math.cos(heading))
                if min(speeds) >= .2 else None,
                source_position_error=float(np.linalg.norm(initial_error)),
                source_velocity_error=float(np.linalg.norm(initial_velocity_error)),
                displacement_increment_error=float(np.linalg.norm(increment)),
                initial_cv_effect_error=float(np.linalg.norm(cv_effect)),
                beyond_initial_cv_error=float(np.linalg.norm(total - cv_effect)),
                error_vector=total.tolist(), source_position_error_vector=initial_error.tolist(),
                displacement_increment_error_vector=increment.tolist(),
                initial_cv_effect_vector=cv_effect.tolist(),
                beyond_initial_cv_vector=(total - cv_effect).tolist())
        scores.append(value)
    return scores


def _score_summary(scores):
    valid = [row for row in scores if row['scored']]
    result = dict(attempts=len(scores), scored_count=len(valid),
                  invalid_count=sum(not row['valid'] for row in scores),
                  censor_reasons=dict(Counter(row['reason'] for row in scores
                                              if not row['scored'] and row['valid'])))
    for field, label in (('horizontal_error', 'horizontal'), ('velocity_error', 'velocity'),
                         ('heading_error_rad', 'heading'),
                         ('source_position_error', 'source_position'),
                         ('source_velocity_error', 'source_velocity'),
                         ('displacement_increment_error', 'displacement_increment'),
                         ('initial_cv_effect_error', 'initial_cv_effect'),
                         ('beyond_initial_cv_error', 'beyond_initial_cv')):
        result[label] = _statistics([row[field] for row in valid if row[field] is not None])
    return result


def _actual_healthy(summary, execution, metadata):
    authority, lifecycle = execution.get('authority', {}), execution.get('lifecycle', {})
    native = execution.get('native_command', {}).get('period_seconds') or {}
    checks = summary.get('p51_audit', {}).get('checks', {})
    return bool(metadata.get('role') == 'pilot' and metadata.get('motion_mode') == 'linear'
                and metadata.get('prediction_model') == 'bctra'
                and summary.get('completed') is True and summary.get('healthy') is True
                and summary.get('observation_seconds', 0.) >= 59.5
                and summary.get('diagnostic_coverage', 0.) >= .95
                and summary.get('p51_audit', {}).get('passed') is True
                and all(checks.get(name) is True for name in (
                    'reference_height', 'entry_recovery', 'execution_gates',
                    'evidence_coverage', 'switching', 'handovers'))
                and authority.get('discovered_samples', 0) > 0
                and authority.get('all_discovered_single_tracker') is True
                and not authority.get('conflicts') and lifecycle.get('accepted_unique', 0) > 0
                and lifecycle.get('active_unique', 0) > 0 and native.get('count', 0) > 0)


def analyze_rows(rows, run_summary, execution_analysis=None, case_metadata=None,
                 malformed_lines=()):
    """Analyze the predefined 60s receipt cohort, preserving every score/rejection."""
    execution, metadata = execution_analysis or {}, case_metadata or {}
    first = run_summary.get('first_minco_receipt')
    start = float(first) if _finite(first) else 0.
    end = start + 60.
    window = [row for row in rows if _finite(row.get('receipt'))
              and start <= float(row['receipt']) <= end]
    groups = defaultdict(list)
    for row in window:
        groups[row.get('kind', 'unknown')].append(row)
    truth = BoundedTruth([row for row in rows if row.get('kind') == 'ideal'
                          and _finite(row.get('receipt')) and row['receipt'] <= end])
    motion = sorted(((row['stamp_ns'] * 1e-9, row) for row in rows
                     if row.get('kind') == 'p6_motion_truth' and _finite(row.get('stamp_ns'))),
                    key=lambda value: value[0])
    valid_raw = [row for row in groups['p6_observation'] if _raw_observation_valid(row)]
    # Select predictions by the fixed receipt window, then join their original
    # acquisitions and sequence-keyed diagnostics across DDS delivery boundaries.
    observations = _ObservationIndex(
        row for row in rows if row.get('kind') == 'p6_observation'
        and _finite(row.get('receipt')) and row['receipt'] <= end)
    diagnostics = defaultdict(list)
    for row in rows:
        if row.get('kind') == 'p6_imm':
            diagnostics[row.get('mission_id'), row.get('sequence_id')].append(row)
    scores, forecast_counts = [], Counter()
    identities, covariance_checks, leases = [], [], []
    full_counts, probabilities, computes = [], [], []
    unique_ids, unique_observations = set(), set()
    duplicate_ids = 0
    invalid_reasons, observation_joins = Counter(), Counter()
    for row in groups['p6_prediction']:
        arrays = _forecast_arrays(row)
        stage = _stage_at(motion, row.get('source_stamp'))
        scores.extend(_score_forecast(row, arrays, truth, stage))
        forecast_counts[row.get('topic', 'unknown')] += 1
        if not row.get('valid'):
            invalid_reasons[row.get('reason', 'unknown')] += 1
        if row.get('topic') != SHADOW or not row.get('valid'):
            continue
        key = row.get('mission'), row.get('sequence')
        duplicate_ids += key in unique_ids
        unique_ids.add(key)
        obs_key = (round(float(row['observation_stamp']) / EPOCH_TOLERANCE)
                   if _finite(row.get('observation_stamp')) else None)
        unique_observations.add((row.get('mission'), obs_key))
        matched = diagnostics.get(key, [])
        diagnostic = matched[0] if len(matched) == 1 else None
        observation, join_reason = observations.match(row, diagnostic)
        observation_joins[join_reason] += 1
        identity, covariance_ok, probability = _imm_contract(row, diagnostic, observation, arrays)
        identities.append(identity)
        covariance_checks.append(covariance_ok and arrays is not None and _psd(
            row.get('covariance'), count=len(arrays['times'])))
        leases.append(_lease_valid(row))
        full_counts.append(bool(arrays is not None and len(arrays['times']) >= 41
                                and arrays['times'][-1] >= 4. - 1e-9))
        if identity and probability is not None:
            probabilities.append(probability)
        computes.append(float(row['compute']) * 1000. if _finite(row.get('compute'))
                        and row['compute'] >= 0. else float('nan'))
    prediction_report = {}
    for topic in forecast_counts:
        selected = [row for row in scores if row['topic'] == topic]
        prediction_report[topic] = dict(
            forecasts=forecast_counts[topic],
            horizons={str(h): _score_summary([row for row in selected if row['horizon'] == h])
                      for h in HORIZONS},
            stages={stage: {str(h): _score_summary([
                row for row in selected if row['stage'] == stage and row['horizon'] == h])
                for h in HORIZONS} for stage in sorted({row['stage'] for row in selected})})
    primary_node = ('imm_target_predictor_node' if metadata.get('prediction_model') == 'imm'
                    else 'target_predictor_node')
    graph = groups['graph']
    primary_unique = bool(graph and all(
        len(row.get('publishers', {}).get(PRIMARY, [])) == 1
        and row['publishers'][PRIMARY][0].get('node') == primary_node for row in graph)
        and _coverage(graph, start, end, maximum_gap=2., edge=1.5))
    native_coverage = _coverage(groups['reference'], start, end, maximum_gap=.15, edge=.15)
    valid_kf = [row for row in groups['kf'] if row.get('valid')
                and _array(row.get('p'), (3,)) is not None
                and _array(row.get('v'), (3,)) is not None]
    valid_ideal = [row for row in groups['ideal'] if row.get('valid')
                   and _array(row.get('p'), (3,)) is not None
                   and _array(row.get('v'), (3,)) is not None]
    prediction_coverage = {topic: _coverage([
        row for row in groups['p6_prediction'] if row.get('topic') == topic],
        start, end, maximum_gap=.5, edge=.5) for topic in (PRIMARY, SHADOW)}
    raw_complete = bool(not malformed_lines and valid_raw and groups['kf'] and len(truth.times)
                        and groups['p6_imm'] and forecast_counts[PRIMARY] > 0
                        and all(prediction_coverage.values())
                        and _coverage(valid_raw, start, end, maximum_gap=.5, edge=.5)
                        and _coverage(valid_kf, start, end, maximum_gap=.5, edge=.5)
                        and _coverage(valid_ideal, start, end, maximum_gap=.2, edge=.2)
                        and native_coverage and not truth.clock_reversals
                        and not truth.conflicting_duplicates and not truth.invalid_records)
    probability_values = np.array(probabilities)
    probability_range = (np.ptp(probability_values, axis=0).tolist()
                         if len(probability_values) else [])
    compute_stats = _statistics(computes)
    full_forecasts = sum(full_counts)
    checks = dict(
        actual_healthy_60s_pilot=_actual_healthy(run_summary, execution, metadata),
        raw_recording_complete=raw_complete,
        more_than_200_full_shadow_forecasts=(
            full_forecasts > 200 and len(unique_observations) > 200),
        matching_full_imm_diagnostics=bool(identities and all(identities) and not duplicate_ids),
        original_acquisition_lease=bool(leases and all(leases)),
        conditional_covariance_psd=bool(covariance_checks and all(covariance_checks)),
        unique_primary_publisher=primary_unique,
        model_probabilities_vary=bool(probability_range and max(probability_range) > 1e-6),
        compute_p95_under_20ms=bool(
            computes and np.isfinite(computes).all() and compute_stats['p95'] <= 20.))
    report = dict(
        schema='p6-actual-analysis-v1', evaluation_only=True, qualification_generated=False,
        predefined_window=dict(start_receipt=start, end_receipt=end, seconds=60.,
                               reason='first actual MINCO receipt; never selected for low errors'),
        record_counts=dict(Counter(row.get('kind', 'unknown') for row in rows)),
        window_record_counts={kind: len(value) for kind, value in groups.items()},
        prediction_recording_coverage_60s=prediction_coverage,
        malformed_lines=list(malformed_lines),
        truth=dict(samples=len(truth.times), clock_reversals=truth.clock_reversals,
                   conflicting_duplicates=truth.conflicting_duplicates,
                   invalid_records=truth.invalid_records, maximum_interpolation_gap=.2,
                   no_extrapolation=True,
                   cutoff='receipt <= predefined 60s end; no post-stop freeze'),
        observation=_position_metrics(groups['p6_observation'], truth),
        kf=_position_metrics(groups['kf'], truth), prediction=prediction_report,
        invalid_prediction_reasons=dict(invalid_reasons),
        imm=dict(valid_shadow_forecasts=len(identities), full_shadow_forecasts=full_forecasts,
                 unique_acquisitions=len(unique_observations), duplicate_identities=duplicate_ids,
                 observation_join_reasons=dict(observation_joins),
                 probability_range=probability_range, compute_ms=compute_stats,
                 compute_scope='raw update+forecast+message; excludes JSON/DDS; 20ms engineering '
                 'budget inside 20Hz/50ms period does not replace original 125ms input lease'),
        actual_control_audit=dict(run_summary=run_summary, authority=execution.get('authority'),
                                  lifecycle=execution.get('lifecycle'),
                                  native_command=execution.get('native_command'),
                                  native_receipt_coverage_60s=native_coverage,
                                  note='Original recorded owner/native audit retained verbatim'),
        decomposition_scope='Exact horizontal translation plus displacement increment; initial '
        'CV position/velocity effect is also reported. Increment contains latent-state errors; '
        'it is not a matched-initial-state pure model causal error.',
        shadow_gate=dict(passed=all(checks.values()), checks=checks,
                         scope='Actual shadow evidence diagnostic only; no qualification or '
                         'main-chain authorization is created'))
    return report, scores


def analyze_case(case, output):
    """Write only a new report directory outside the immutable actual case."""
    case, output = Path(case).resolve(), Path(output).resolve()
    if output == case or output.is_relative_to(case) or case.is_relative_to(output):
        raise ValueError('P6_NEW_EXTERNAL_ANALYSIS_DIRECTORY_REQUIRED')
    if output.exists():
        raise FileExistsError(str(output))
    flight = case / 'flight'
    evidence = flight / 'offline_evidence.jsonl'
    rows, malformed = [], []
    with evidence.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
                if not isinstance(value, dict) or 'kind' not in value:
                    raise ValueError('record must be an object with kind')
                rows.append(value)
            except (json.JSONDecodeError, ValueError):
                malformed.append(number)
    metadata_path, summary_path = case / 'case.json', flight / 'p5_run_summary.json'
    execution_path = flight / 'execution_analysis.json'
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    execution = json.loads(execution_path.read_text()) if execution_path.exists() else {}
    report, scores = analyze_rows(rows, summary, execution, metadata, malformed)
    sources = [path for path in (evidence, metadata_path, summary_path, execution_path)
               if path.exists()]
    report['source_evidence'] = {str(path): sha(path) for path in sources}
    report['analyzer_sha256'] = sha(Path(__file__))
    output.mkdir(parents=True, exist_ok=False)
    (output / 'analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    with (output / 'forecast_scores.jsonl').open('x') as stream:
        for score in scores:
            stream.write(json.dumps(score, allow_nan=False, separators=(',', ':')) + '\n')
    (output / 'SHA256SUMS').write_text(''.join(
        f'{sha(path)}  {path.name}\n' for path in sorted(output.iterdir()) if path.is_file()))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args(argv)
    report = analyze_case(args.case, args.output)
    print(json.dumps(report['shadow_gate'], indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

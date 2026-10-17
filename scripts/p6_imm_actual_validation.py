#!/usr/bin/env python3
"""Evaluate saved IMM residuals and covariance; never fit or grant flight qualification."""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import shutil

import numpy as np

import p6_actual_analysis as actual

CHI_SQUARE_95 = {2: 5.991464547107979, 3: 7.814727903251179}
DATA_ROLES = ('diagnostic', 'calibration_train', 'heldout_validation')


def serialize(value, **options):
    """Represent nonfinite rejected input epochs by null; raw input remains immutable."""
    def clean(item):
        if isinstance(item, dict):
            return {key: clean(part) for key, part in item.items()}
        if isinstance(item, (list, tuple)):
            return [clean(part) for part in item]
        if isinstance(item, float) and not np.isfinite(item):
            return None
        return item

    return json.dumps(clean(value), allow_nan=False, **options)


def _wire_identity_mismatches(prediction, diagnostic):
    """Compare representable recorded floats on the online ROS nanosecond grid."""
    if diagnostic is None:
        return ['diagnostic_missing_or_ambiguous']

    def nanoseconds(value):
        if not isinstance(value, (int, float)) or not actual._finite(value):
            return None
        second = math.floor(value)
        return second * 1_000_000_000 + round((value - second) * 1e9)

    mismatches = []
    for field, counterpart in (('mission', 'mission_id'), ('sequence', 'sequence_id'),
                               ('source', 'source'), ('model', 'model'), ('frame', 'frame_id')):
        if prediction.get(field) != diagnostic.get(counterpart):
            mismatches.append(field)
    for field in ('source_stamp', 'observation_stamp', 'generated_stamp', 'valid_until'):
        expected, recorded = nanoseconds(prediction.get(field)), nanoseconds(diagnostic.get(field))
        if expected is None or recorded is None or expected != recorded:
            mismatches.append(field)
    expected_times, recorded_times = prediction.get('times'), diagnostic.get('relative_times')
    expected = (tuple(map(nanoseconds, expected_times))
                if isinstance(expected_times, (list, tuple)) else ())
    recorded = (tuple(map(nanoseconds, recorded_times))
                if isinstance(recorded_times, (list, tuple)) else ())
    if not expected or None in expected or expected != recorded:
        mismatches.append('relative_times')
    return mismatches


def covariance_score(error, covariance, dimension):
    """Whiten position error with full covariance, without regularizing singular matrices."""
    if dimension not in CHI_SQUARE_95:
        raise ValueError('P6_POSITION_DIMENSION_REQUIRED')
    result = dict(scored=False, reason='', dimension=dimension,
                  threshold_95=CHI_SQUARE_95[dimension], covered_95=None)
    residual = actual._array(error, (3,))
    matrix = actual._array(covariance)
    if matrix is None or matrix.size != 9:
        return dict(result, reason='covariance_nonfinite_or_shape')
    matrix = matrix.reshape(3, 3)[:dimension, :dimension]
    if residual is None:
        return dict(result, reason='residual_nonfinite_or_shape')
    if not np.allclose(matrix, matrix.T, atol=1e-8, rtol=1e-8):
        return dict(result, reason='covariance_not_symmetric')
    try:
        factor = np.linalg.cholesky(matrix)
        whitened = np.linalg.solve(factor, residual[:dimension])
    except np.linalg.LinAlgError:
        return dict(result, reason='covariance_not_positive_definite')
    squared = float(whitened @ whitened)
    if not np.isfinite(squared):
        return dict(result, reason='standardized_residual_nonfinite')
    return dict(result, scored=True, covariance=matrix.tolist(), reason='',
                mahalanobis_squared=squared, standardized_norm=float(np.sqrt(squared)),
                whitened_error=whitened.tolist(),
                marginal_standardized_error=(residual[:dimension]
                                             / np.sqrt(np.diag(matrix))).tolist(),
                covered_95=bool(squared <= CHI_SQUARE_95[dimension]))


def _unscored_covariance(dimension, reason):
    return dict(scored=False, reason=reason, dimension=dimension,
                threshold_95=CHI_SQUARE_95[dimension], covered_95=None)


def _vector_summary(vectors):
    values = np.asarray(vectors, dtype=float)
    if not values.size:
        return dict(count=0, mean_xyz=None, component_rmse_xyz=None,
                    horizontal=actual._statistics([]), spatial=actual._statistics([]))
    return dict(count=len(values), mean_xyz=np.mean(values, axis=0).tolist(),
                component_rmse_xyz=np.sqrt(np.mean(values ** 2, axis=0)).tolist(),
                horizontal=actual._statistics(np.linalg.norm(values[:, :2], axis=1)),
                spatial=actual._statistics(np.linalg.norm(values, axis=1)))


def _covariance_summary(rows, field):
    attempted = [row[field] for row in rows]
    scored = [row for row in attempted if row['scored']]
    covered = sum(row['covered_95'] for row in scored)
    return dict(attempts=len(attempted), scored_count=len(scored), covered_count=covered,
                coverage_95=covered / len(scored) if scored else None,
                unscored_reasons=dict(Counter(
                    row['reason'] for row in attempted if not row['scored'])),
                mahalanobis_squared=actual._statistics([
                    row['mahalanobis_squared'] for row in scored]),
                standardized_norm=actual._statistics([
                    row['standardized_norm'] for row in scored]),
                mean_mahalanobis_squared=float(np.mean([
                    row['mahalanobis_squared'] for row in scored])) if scored else None,
                mean_whitened_error=np.mean([
                    row['whitened_error'] for row in scored], axis=0).tolist() if scored else None,
                mean_marginal_standardized_error=np.mean([
                    row['marginal_standardized_error'] for row in scored], axis=0).tolist()
                if scored else None)


def _forecast_summary(rows):
    scored = [row for row in rows if row['scored']]
    return dict(attempts=len(rows), scored_count=len(scored),
                invalid_forecasts=sum(not row['valid'] for row in rows),
                unscored_reasons=dict(Counter(
                    row['reason'] for row in rows if not row['scored'])),
                position=_vector_summary([row['error_xyz'] for row in scored]),
                velocity=_vector_summary([row['velocity_error_xyz'] for row in scored]),
                source_position=_vector_summary([row['source_error_xyz'] for row in scored]),
                source_velocity=_vector_summary([
                    row['source_velocity_error_xyz'] for row in scored]),
                increment=_vector_summary([row['increment_error_xyz'] for row in scored]),
                initial_cv_effect=_vector_summary([
                    row['initial_cv_effect_xyz'] for row in scored]),
                beyond_initial_cv=_vector_summary([
                    row['beyond_initial_cv_xyz'] for row in scored]),
                covariance_2d=_covariance_summary(rows, 'covariance_2d'),
                covariance_3d=_covariance_summary(rows, 'covariance_3d'))


def _source_score(kind, epoch, position, velocity, truth, **identity):
    value = dict(kind=kind, epoch=epoch, scored=False, **identity)
    reference, reason = truth.at(epoch)
    position = actual._array(position, (3,))
    if position is None or reference is None:
        return dict(value, reason=reason or 'position_unavailable')
    value.update(scored=True, reason='', position_error_xyz=(position - reference['p']).tolist())
    velocity = actual._array(velocity, (3,))
    if velocity is not None:
        value['velocity_error_xyz'] = (velocity - reference['v']).tolist()
    return value


def _source_summary(rows):
    scored = [row for row in rows if row['scored']]
    return dict(attempts=len(rows), scored_count=len(scored),
                unscored_reasons=dict(Counter(
                    row['reason'] for row in rows if not row['scored'])),
                position=_vector_summary([row['position_error_xyz'] for row in scored]),
                velocity=_vector_summary([row['velocity_error_xyz'] for row in scored
                                         if 'velocity_error_xyz' in row]))


def _probability_summary(rows):
    values = np.asarray([row['probabilities'] for row in rows])
    return dict(count=len(rows), model_order=['CV', 'CTRV', 'CTRA'],
                mean=np.mean(values, axis=0).tolist() if len(values) else None,
                min=np.min(values, axis=0).tolist() if len(values) else None,
                max=np.max(values, axis=0).tolist() if len(values) else None,
                dominant_counts=dict(Counter(['CV', 'CTRV', 'CTRA'][int(np.argmax(value))]
                                             for value in values)))


def _forecast_covariance(row, arrays, horizon):
    if arrays is None:
        return None
    indices = np.flatnonzero(np.isclose(arrays['times'], horizon, atol=1e-9, rtol=0.))
    covariance = actual._array(row.get('covariance'))
    if (len(indices) != 1 or covariance is None
            or covariance.size != len(arrays['times']) * 9):
        return None
    return covariance.reshape(-1, 3, 3)[indices[0]]


def analyze_rows(rows, summary, execution=None, metadata=None, malformed_lines=(),
                 data_role='diagnostic'):
    """Keep the existing fixed receipt cohort and epoch-bounded scoring protocol."""
    if data_role not in DATA_ROLES:
        raise ValueError('P6_DATA_ROLE_INVALID')
    if not actual._finite(summary.get('first_minco_receipt')):
        raise ValueError('P6_FIRST_MINCO_RECEIPT_REQUIRED')
    baseline, baseline_scores = actual.analyze_rows(
        rows, summary, execution, metadata, malformed_lines)
    start = float(summary['first_minco_receipt'])
    end = start + 60.
    window = [row for row in rows if actual._finite(row.get('receipt'))
              and start <= row['receipt'] <= end]
    truth = actual.BoundedTruth([row for row in rows if row.get('kind') == 'ideal'
                                and actual._finite(row.get('receipt')) and row['receipt'] <= end])
    kf = actual.BoundedTruth([row for row in rows if row.get('kind') == 'kf'
                             and actual._finite(row.get('receipt')) and row['receipt'] <= end])
    motion = sorted(((row['stamp_ns'] * 1e-9, row) for row in rows
                     if row.get('kind') == 'p6_motion_truth'
                     and actual._finite(row.get('stamp_ns'))),
                    key=lambda item: item[0])
    observations = actual._ObservationIndex(
        row for row in rows if row.get('kind') == 'p6_observation'
        and actual._finite(row.get('receipt')) and row['receipt'] <= end)
    diagnostics = defaultdict(list)
    for row in rows:
        if row.get('kind') == 'p6_imm':
            diagnostics[row.get('mission_id'), row.get('sequence_id')].append(row)
    predictions = [row for row in window if row.get('kind') == 'p6_prediction']
    identity_counts = Counter((row.get('topic'), row.get('mission'), row.get('sequence'))
                              for row in predictions if row.get('valid'))
    sources, scores, probabilities = [], [], []
    protocol_reasons, wire_mismatches = Counter(), Counter()
    wire_attempts, wire_exact = 0, 0
    for row in window:
        if row.get('kind') in ('p6_observation', 'kf'):
            kind = 'observation' if row['kind'] == 'p6_observation' else 'kf'
            if row.get('valid'):
                sources.append(_source_score(kind, row.get('stamp'), row.get('p'), row.get('v'),
                                             truth, receipt=row['receipt'],
                                             source_line=row.get('_source_line'),
                                             acquisition_stamp=row.get('source') if kind == 'kf'
                                             else row.get('stamp')))
            else:
                sources.append(dict(kind=kind, epoch=row.get('stamp'), scored=False,
                                    reason='invalid_record', receipt=row['receipt'],
                                    source_line=row.get('_source_line'),
                                    invalid_reason=row.get('reason', 'unknown')))
    for prediction_index, row in enumerate(predictions):
        arrays = actual._forecast_arrays(row)
        initial = actual._at_forecast(arrays, 0.)
        source_truth, _ = truth.at(row.get('source_stamp'))
        protocol_valid = bool(row.get('valid') and actual._lease_valid(row)
                              and identity_counts[row.get('topic'), row.get('mission'),
                                                  row.get('sequence')] == 1)
        diagnostic = None
        if row.get('model') == 'IMM_CV_CTRV_CTRA' and row.get('valid'):
            matched = diagnostics[row.get('mission'), row.get('sequence')]
            diagnostic = matched[0] if len(matched) == 1 else None
            mismatches = _wire_identity_mismatches(row, diagnostic)
            wire_attempts += 1
            wire_exact += not mismatches
            wire_mismatches.update(mismatches)
            observation, _ = observations.match(row, diagnostic)
            identity, covariance_valid, probability = actual._imm_contract(
                row, diagnostic, observation, arrays)
            protocol_valid = bool(protocol_valid and identity and covariance_valid
                                  and not mismatches)
            if identity and probability is not None:
                probabilities.append(dict(probabilities=probability.tolist(),
                                          stage=actual._stage_at(motion, row.get('source_stamp'))))
            if initial is not None:
                sources.append(_source_score(
                    'imm_source', row.get('source_stamp'), initial['p'], initial['v'], truth,
                    receipt=row['receipt'], mission=row.get('mission'),
                    sequence=row.get('sequence'),
                    source_line=row.get('_source_line'),
                    acquisition_stamp=row.get('observation_stamp'), protocol_valid=protocol_valid))
                kf_value, kf_reason = kf.at(row.get('source_stamp'))
                pair = dict(kind='same_epoch_imm_kf', epoch=row.get('source_stamp'),
                            mission=row.get('mission'), sequence=row.get('sequence'),
                            scored=kf_value is not None, reason=kf_reason,
                            protocol_valid=protocol_valid)
                if kf_value is not None:
                    pair.update(position_delta_xyz=(initial['p'] - kf_value['p']).tolist(),
                                velocity_delta_xyz=(initial['v'] - kf_value['v']).tolist())
                sources.append(pair)
        if row.get('valid') and not protocol_valid:
            protocol_reasons['prediction_protocol_invalid'] += 1
        for horizon_index, horizon in enumerate(actual.HORIZONS):
            old = baseline_scores[prediction_index * len(actual.HORIZONS) + horizon_index]
            value = dict(old, protocol_valid=protocol_valid, source_line=row.get('_source_line'))
            if old['scored']:
                future = actual._at_forecast(arrays, horizon)
                future_truth, _ = truth.at(row['source_stamp'] + horizon)
                error = future['p'] - future_truth['p']
                source_error = initial['p'] - source_truth['p']
                source_velocity_error = initial['v'] - source_truth['v']
                increment = ((future['p'] - initial['p'])
                             - (future_truth['p'] - source_truth['p']))
                initial_cv = source_error + horizon * source_velocity_error
                value.update(error_xyz=error.tolist(), source_error_xyz=source_error.tolist(),
                             source_velocity_error_xyz=source_velocity_error.tolist(),
                             velocity_error_xyz=(future['v'] - future_truth['v']).tolist(),
                             increment_error_xyz=increment.tolist(),
                             initial_cv_effect_xyz=initial_cv.tolist(),
                             beyond_initial_cv_xyz=(error - initial_cv).tolist())
                covariance = _forecast_covariance(row, arrays, horizon)
                for dimension in (2, 3):
                    value[f'covariance_{dimension}d'] = (
                        covariance_score(error, covariance, dimension) if protocol_valid
                        else _unscored_covariance(dimension, 'prediction_protocol_invalid'))
            else:
                for dimension in (2, 3):
                    value[f'covariance_{dimension}d'] = _unscored_covariance(
                        dimension, old['reason'])
            scores.append(value)
    prediction_report = {topic: {
        str(horizon): _forecast_summary([
            row for row in scores if row['topic'] == topic and row['horizon'] == horizon])
        for horizon in actual.HORIZONS} for topic in sorted({row['topic'] for row in scores})}
    paired = [row for row in sources if row['kind'] == 'same_epoch_imm_kf']
    paired_valid = [row for row in paired if row['scored']]
    report = dict(
        schema='p6-imm-actual-validation-v1', evaluation_only=True, qualification_generated=False,
        main_chain_eligible=False, malformed_lines=list(malformed_lines),
        predefined_window=baseline['predefined_window'], truth=baseline['truth'],
        record_counts=baseline['record_counts'],
        window_record_counts=baseline['window_record_counts'],
        prediction=prediction_report, protocol_reasons=dict(protocol_reasons),
        current_state={kind: _source_summary([row for row in sources if row['kind'] == kind])
                       for kind in ('observation', 'kf', 'imm_source')},
        same_epoch_imm_kf=dict(
            attempts=len(paired), scored_count=len(paired_valid),
            unscored_reasons=dict(Counter(row['reason'] for row in paired if not row['scored'])),
            position_delta=_vector_summary([row['position_delta_xyz'] for row in paired_valid]),
            velocity_delta=_vector_summary([row['velocity_delta_xyz'] for row in paired_valid]),
            scope='Retrospective interpolation of recorded KF P/V to immutable IMM source epoch; '
            'not a causal online-availability or matched-acquisition/pure-model comparison. '
            'Maximum KF bracket gap 0.2s; no extrapolation or acquisition stamp rewriting.'),
        model_probabilities=dict(overall=_probability_summary(probabilities), by_stage={
            stage: _probability_summary([row for row in probabilities if row['stage'] == stage])
            for stage in sorted({row['stage'] for row in probabilities})}),
        imm_protocol=baseline['imm'], existing_shadow_diagnostic=baseline['shadow_gate'],
        wire_identity=dict(attempts=wire_attempts, exact_matches=wire_exact,
                           mismatched_fields=dict(wire_mismatches),
                           scope='Recorded float epochs compared on exact ROS 1ns grid, matching '
                           'online attach identity. Does not recover nanoseconds lost by float '
                           'logging. Offline raw-observation join retains its 1us bound.'),
        recorded_control=dict(minco_fraction=summary.get('minco_fraction'),
                              longest_minco_seconds=summary.get('longest_minco_seconds'),
                              fallback_count=summary.get('ownership', {}).get('fallback_count'),
                              lease_expiry_count=summary.get('p51_audit', {}).get(
                                  'lease_expiry_count'),
                              f3_eligible=summary.get('f3_eligible')),
        independence=dict(data_role=data_role, fitted_calibration=False, online_truth_used=False,
                          source_stamp_rewritten=False,
                          note='Selecting a role never establishes independence. Flight/source '
                          'identity and disjoint calibration/validation runs need separate review.'
                          ),
        nonfinite_serialization='Rejected nonfinite values serialize as null; original source '
        'file and source line retain the raw value. No finite replacement enters scoring.',
        limits=[
            'One receipt cohort; overlapping horizons/frames are correlated. No IID confidence '
            'interval, joint trajectory probability or continuous-time safety claim.',
            'Coverage concerns raw position covariance against recorded interpolation truth. '
            'It is not innovation NIS and excludes truth/reference uncertainty.',
            '2D marginal and 3D covariance coverage use different chi-square thresholds. '
            'Singular covariance is rejected without jitter or pseudoinversion.',
            'BCTRA covariance is its declared engineering model; this analysis does not make it '
            'statistically calibrated. IMM mixture is not necessarily Gaussian.',
            'State/increment vector identities are algebraic; increment and beyond-CV errors '
            'still contain latent initial-state error and are not pure model causal attribution.',
            'Existing shadow diagnostic does not gate on MINCO fraction or lease expiries. '
            'A diagnostic pass cannot authorize IMM primary or supersede supply/F2 qualification.',
            'No five-mode actual claim, model-stage identification claim or main-chain '
            'qualification can be generated by this offline tool.'])
    return report, scores, sources


def analyze_case(case, output, data_role='diagnostic'):
    """Write a fresh external directory and hash immutable inputs and executing analyzers."""
    if data_role not in DATA_ROLES:
        raise ValueError('P6_DATA_ROLE_INVALID')
    case, output = Path(case).resolve(), Path(output).resolve()
    if output == case or output.is_relative_to(case) or case.is_relative_to(output):
        raise ValueError('P6_NEW_EXTERNAL_ANALYSIS_DIRECTORY_REQUIRED')
    if output.exists():
        raise FileExistsError(str(output))
    evidence = case / 'flight/offline_evidence.jsonl'
    rows, malformed = [], []
    with evidence.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                if not isinstance(row, dict) or 'kind' not in row:
                    raise ValueError('record must be object with kind')
                row['_source_line'] = number
                rows.append(row)
            except (ValueError, json.JSONDecodeError):
                malformed.append(number)
    paths = [case / 'case.json', case / 'flight/p5_run_summary.json',
             case / 'flight/execution_analysis.json']
    metadata, summary, execution = [json.loads(path.read_text()) if path.exists() else {}
                                    for path in paths]
    report, scores, sources = analyze_rows(
        rows, summary, execution, metadata, malformed, data_role)
    inputs = [path for path in [evidence, *paths, *sorted((case / 'config').glob('*.yaml'))]
              if path.exists()]
    report['source_evidence'] = {str(path): actual.sha(path) for path in inputs}
    report['case_metadata'] = metadata
    report['analyzer_sources'] = {str(path): actual.sha(path) for path in (
        Path(__file__).resolve(), Path(actual.__file__).resolve())}
    output.mkdir(parents=True, exist_ok=False)
    snapshot = output / 'source_snapshot'
    snapshot.mkdir()
    for path in (Path(__file__).resolve(), Path(actual.__file__).resolve()):
        shutil.copyfile(path, snapshot / path.name)
    (output / 'analysis.json').write_text(serialize(report, indent=2) + '\n')
    for filename, records in (('forecast_validation.jsonl', scores),
                              ('source_validation.jsonl', sources)):
        with (output / filename).open('x') as stream:
            for row in records:
                stream.write(serialize(row, separators=(',', ':')) + '\n')
    (output / 'SHA256SUMS').write_text(''.join(
        f'{actual.sha(path)}  {path.relative_to(output)}\n'
        for path in sorted(output.rglob('*')) if path.is_file()))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--data-role', choices=DATA_ROLES, default='diagnostic')
    args = parser.parse_args(argv)
    report = analyze_case(args.case, args.output, args.data_role)
    print(json.dumps(dict(recorded_control=report['recorded_control'],
                          imm_protocol=report['imm_protocol'],
                          existing_shadow_diagnostic=report['existing_shadow_diagnostic'],
                          qualification_generated=False), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

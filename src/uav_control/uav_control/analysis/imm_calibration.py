"""
IMM-only offline empirical calibration; heldout residuals cannot fit parameters.

Migrated pure functions from historical tools at 55c1003; no CLI or online wiring.
"""
import math

import numpy as np

CHI_SQUARED_95_2D = 5.991464547107979
CHI_SQUARED_95_3D = 7.814727903251179


def summarize(rows):
    """Count all attempts and keep severe valid residuals in every metric."""
    valid = [item for item in rows if item['valid']]
    result = dict(attempts=len(rows), valid_count=len(valid),
                  invalid_count=len(rows) - len(valid),
                  valid_fraction=len(valid) / max(len(rows), 1))
    if not valid:
        return result
    for key, label in (('horizontal_error', 'horizontal'), ('velocity_error', 'velocity')):
        values = np.array([item[key] for item in valid])
        result[label + '_rmse'] = float(np.sqrt(np.mean(values ** 2)))
        result[label + '_p95'] = float(np.percentile(values, 95))
    result['severe_over_5m'] = sum(item['horizontal_error'] > 5. for item in valid)
    normalized = [item['mahalanobis_squared'] for item in valid
                  if item['mahalanobis_squared'] is not None]
    result['nominal_95_coverage'] = float(np.mean(
        np.asarray(normalized) <= CHI_SQUARED_95_2D)) if normalized else None
    heading = [item['heading_error'] for item in valid if item['heading_error'] is not None]
    result['heading_rmse_rad'] = float(np.sqrt(np.mean(np.square(heading)))) if heading else None
    result['compute_ms'] = dict(zip(('p50', 'p95', 'p99'), map(float, np.percentile(
        [item['compute_time_ms'] for item in valid], (50, 95, 99)))))
    for name in ('initial_state_effect', 'model_increment', 'pure_model_increment',
                 'current_turn_rate_error'):
        values = [item[name] for item in valid if name in item and item[name] is not None]
        if values:
            result[name + '_rmse'] = float(np.sqrt(np.mean(np.square(values))))
    return result


def fit_calibration(calibration_rows, heldout_rows, family, provenance):
    """Fit only calibration residuals; heldout coverage never adjusts the scale."""
    if {item['seed'] for item in calibration_rows} & {item['seed'] for item in heldout_rows}:
        raise ValueError('calibration and heldout seeds must be disjoint')
    if family.lower() != 'imm':
        raise ValueError('only current IMM calibration is supported')
    name = family.upper()
    train = [item for item in calibration_rows if item['backend'] == name and item['valid']
             and item['comparison'] == 'complete_chain']
    test = [item for item in heldout_rows if item['backend'] == name and item['valid']
            and item['comparison'] == 'complete_chain']
    if not train or not test:
        raise ValueError('both independent splits require valid residuals')

    def distances(item, conditional=False):
        error = np.asarray(item['error'], dtype=float)
        covariance = np.asarray(item['covariance'], dtype=float)
        if not conditional:
            return [float(error @ np.linalg.solve(covariance, error))]
        # IMM conditionals share the independent vertical filter; existing
        # evidence records conditional XY distances and the common z block.
        if np.max(np.abs(covariance[:2, 2])) > 1e-12:
            raise ValueError('conditional xyz reconstruction requires independent vertical block')
        vertical = float(error[2] ** 2 / covariance[2, 2])
        return [value + vertical for value in item.get('conditional_mahalanobis_squared', [])]

    scales = []
    for horizon in sorted({item['horizon'] for item in train}):
        group = [item for item in train if item['horizon'] == horizon]
        normalized = [distances(item)[0] for item in group]
        conditional = [value for item in group for value in distances(item, True)]
        quantile = float(np.quantile(normalized, .95))
        conditional_quantile = float(np.quantile(conditional, .95)) if conditional else quantile
        scales.append(dict(horizon=horizon, covariance_scale=max(
            1., quantile / CHI_SQUARED_95_3D, conditional_quantile / CHI_SQUARED_95_3D),
            calibration_samples=len(group), q95_mahalanobis_squared=quantile,
            q95_conditional_mahalanobis_squared=conditional_quantile))
    scale = max(item['covariance_scale'] for item in scales)

    def coverage(items, conditional=False):
        values = [value for item in items for value in distances(item, conditional)]
        return float(np.mean(np.asarray(values) <= CHI_SQUARED_95_3D * scale)) if values else None

    for item in scales:
        item['calibration_coverage'] = coverage([
            row for row in train if row['horizon'] == item['horizon']])
        item['heldout_coverage'] = coverage([
            row for row in test if row['horizon'] == item['horizon']])
    return dict(
        schema='p6-calibration-v1', id=f'p6_{family}_synthetic_{provenance["calibration_seed"]}',
        family=family, distribution='empirical_envelope', covariance_scale=scale,
        covariance_floor_m2=0., empirical_scale=math.sqrt(CHI_SQUARED_95_3D),
        probability_guarantee=False, calibration_domain='xyz',
        normalization_basis='chi_squared_95_3d_radius_used_as_empirical_reference_only',
        synthetic_only=True, suggested_policy='nominal_fallback',
        calibrated_horizons=[item['horizon'] for item in scales], per_horizon=scales,
        source_provenance=dict(provenance, split='calibration',
                               seed=provenance['calibration_seed']),
        heldout_provenance=dict(provenance, split='heldout', seed=provenance['heldout_seed']),
        coverage=dict(calibration=coverage(train), heldout=coverage(test),
                      calibration_samples=len(train), heldout_samples=len(test),
                      conditional_calibration=coverage(train, True),
                      conditional_heldout=coverage(test, True),
                      nominal_pointwise=.95, joint_trajectory_claim=False),
        limitations=[
            'Synthetic iid position noise; shared motion paths; disjoint noise seeds.',
            'Overlapping horizons are dependent; no iid confidence interval or joint guarantee.',
            'No attitude, depth bias, dropout bursts or real RGB-D residual calibration.',
            'IMM conditional models with posterior probability >=.05 are included in calibration.',
            'No heldout value is used to fit noise, scale, floor, or model parameters.'])

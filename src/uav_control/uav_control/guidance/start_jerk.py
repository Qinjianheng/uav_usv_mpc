"""Bounded fixed-endpoint Q1/T1 experiments; these seeds still require full validation."""
from dataclasses import replace

import numpy as np

from uav_control.guidance.follow_minco_optimizer import _trajectory
from uav_control.guidance.polynomial_extrema import derivative_peak


def start_jerk(seed):
    """Quintic p'''(0)=6*c3, using the established MINCO mapping."""
    return 6*_trajectory(seed, seed.q, seed.durations).coefficients[0, 3]


def adjust_start(seed, mode='combined', radius=.75, deadline=None, clock=None):
    """
    Compare uniform/increased T1 and bounded Q1 correction at fixed total/PVA.

    A common scalar linear coefficient maps each Q1 axis into c3. Correction
    targets initial jerk, but selection also checks the global analytic jerk
    peak. This is a heuristic seed search, never an admission or impossibility proof.
    """
    if mode not in ('time', 'q', 'combined') or not np.isfinite(radius) or radius < 0:
        raise ValueError('INVALID_START_ADJUSTMENT')
    if not seed.valid_input:
        return seed
    original = float(np.linalg.norm(start_jerk(seed)[:2]))
    total = sum(seed.durations)
    times = [seed.durations]
    if mode in ('time', 'combined'):
        times += [(first, (total-first)/2, (total-first)/2)
                  for first in (total*.42, total*.5)]
    options = []
    for durations in times:
        if deadline is not None and clock is not None and clock() >= deadline:
            break
        base = replace(seed, durations=tuple(durations))
        variants = [base]
        if mode in ('q', 'combined'):
            q = np.asarray(base.q).copy()
            shifted = q.copy()
            shifted[0, 0] += 1.
            sensitivity = (start_jerk(replace(base, q=tuple(map(tuple, shifted))))[0]
                           - start_jerk(base)[0])
            if abs(sensitivity) > 1e-9:
                delta = -start_jerk(base)/sensitivity
                delta *= min(1., radius/max(float(np.linalg.norm(delta)), 1e-12))
                q[0] += delta
                variants.append(replace(base, q=tuple(map(tuple, q))))
        for candidate in variants:
            trajectory = _trajectory(candidate, candidate.q, candidate.durations)
            initial = float(np.linalg.norm(6*trajectory.coefficients[0, 3, :2]))
            if initial > original+1e-9:
                continue
            peak = derivative_peak(trajectory.coefficients, trajectory.durations, 3, (0, 1))
            options.append(((peak['value'], trajectory.control_effort(), initial), candidate))
    chosen = min(options, key=lambda pair: pair[0])[1] if options else seed
    return replace(chosen, metrics=dict(chosen.metrics, start_adjustment=dict(
        mode=mode, original_initial_jerk=original,
        adjusted_initial_jerk=float(np.linalg.norm(start_jerk(chosen)[:2])),
        tested=len(options), fixed_total=True, fixed_endpoint=True)), final_validated=False)

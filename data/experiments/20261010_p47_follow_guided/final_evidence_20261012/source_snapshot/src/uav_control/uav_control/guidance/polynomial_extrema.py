"""Exact real-root extrema of polynomial derivative norms (no time grid)."""
from uav_control.guidance.follow_profile import profiled, count

import math

import numpy as np


@profiled('derivative_extrema')
def derivative_peak(coefficients, durations, derivative, axes):
    """Return the largest derivative norm, physical time, piece and endpoint class."""
    c = np.asarray(coefficients, dtype=float)
    ts = np.asarray(durations, dtype=float)
    if (c.shape != (len(ts), 6, 3) or not np.all(np.isfinite(c))
            or not np.all(np.isfinite(ts)) or np.any(ts <= 0)
            or derivative not in (1, 2, 3) or not axes
            or any(axis not in (0, 1, 2) for axis in axes)):
        raise ValueError('INVALID_POLYNOMIAL_EXTREMA')
    best = {'value': -1., 'time': 0., 'piece': 0, 'local_time': 0., 'location': 'start'}
    offset = 0.
    for index, (piece, duration) in enumerate(zip(c, ts)):
        factors = np.array([math.factorial(k) / math.factorial(k - derivative)
                            for k in range(derivative, 6)])
        polys = piece[derivative:, list(axes)] * factors[:, None]
        squared = np.zeros(2 * len(polys) - 1)
        for poly in polys.T:
            squared += np.convolve(poly, poly)
        slope = np.polynomial.polynomial.polyder(squared)
        roots = _counted_roots(slope)
        candidates = [0., float(duration)] + [float(r.real) for r in roots
                                              if abs(r.imag) < 1e-8 and 0 < r.real < duration]
        for t in candidates:
            value = float(np.linalg.norm(np.polynomial.polynomial.polyval(t, polys)))
            if value > best['value']:
                location = ('interior' if 0 < t < duration else
                            'start' if index == 0 and t == 0 else
                            'end' if index == len(ts) - 1 and t == duration else 'junction')
                best = dict(value=value, time=offset + t, piece=index,
                            local_time=t, location=location)
        offset += duration
    return best


def _counted_roots(coefficients):
    count('root_searches')
    return np.polynomial.polynomial.polyroots(coefficients)

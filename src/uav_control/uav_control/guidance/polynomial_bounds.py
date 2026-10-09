"""Conservative derivative norm upper bounds from exact power-to-Bernstein conversion."""
import math

import numpy as np


def derivative_upper_bound(coefficients, durations, derivative, axes):
    """Convex hull of Bernstein vectors bounds every value; retains all polynomial terms."""
    c, ts = np.asarray(coefficients, dtype=float), np.asarray(durations, dtype=float)
    if (c.ndim != 3 or c.shape[1:] != (6, 3) or ts.shape != (len(c),)
            or derivative not in (1, 2, 3) or not axes
            or any(k not in (0, 1, 2) for k in axes) or np.any(ts <= 0)
            or not np.all(np.isfinite(c)) or not np.all(np.isfinite(ts))):
        raise ValueError('INVALID_POLYNOMIAL_BOUND')
    degree = 5-derivative
    factors = np.array([math.factorial(i+derivative)/math.factorial(i)
                        for i in range(degree+1)])
    power = c[:, derivative:, :]*factors[None, :, None]
    scaled = power*ts[:, None, None]**np.arange(degree+1)[None, :, None]
    transform = np.array([[math.comb(k, i)/math.comb(degree, i) if i <= k else 0.
                           for i in range(degree+1)] for k in range(degree+1)])
    bernstein = np.einsum('ki,pia->pka', transform, scaled)
    peak = float(np.max(np.linalg.norm(bernstein[:, :, axes], axis=2)))
    # Inflate roundoff by the coefficient sum, not only by a possibly cancelled result.
    padding = 1e-12*(1.+float(np.max(np.sum(np.abs(scaled), axis=1))))
    result = peak+padding
    if not math.isfinite(result):
        raise ValueError('NONFINITE_POLYNOMIAL_BOUND')
    return result

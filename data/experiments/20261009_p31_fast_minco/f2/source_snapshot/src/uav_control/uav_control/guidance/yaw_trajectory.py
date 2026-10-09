"""Continuous unwrapped yaw with analytic cubic rate extrema."""

import numpy as np
from scipy.interpolate import CubicSpline


class YawTrajectory:
    """Use a C2 cubic spline, with explicit clamped start/end yaw rates."""

    def __init__(self, times, yaw, start_rate=0., end_rate=0.):
        """Reject invalid knots and unwrap all yaw values relative to the first knot."""
        self.times = np.asarray(times, dtype=float)
        values = np.asarray(yaw, dtype=float)
        if (self.times.ndim != 1 or len(self.times) < 2 or values.shape != self.times.shape
                or not np.all(np.isfinite(self.times)) or not np.all(np.isfinite(values))
                or np.any(np.diff(self.times) <= 0) or self.times[0] != 0.
                or not np.all(np.isfinite((start_rate, end_rate)))):
            raise ValueError('INVALID_YAW_KNOTS')
        self.values = np.unwrap(values)
        self.spline = CubicSpline(self.times, self.values,
                                  bc_type=((1, start_rate), (1, end_rate)))

    def sample(self, times):
        """Sample yaw/rate only inside the trajectory, never extrapolate."""
        t = np.asarray(times, dtype=float)
        if not np.all(np.isfinite(t)) or np.any(t < 0) or np.any(t > self.times[-1] + 1e-9):
            raise ValueError('YAW_TIME_OUTSIDE')
        return self.spline(t), self.spline(t, 1)

    def maximum_rate(self):
        """Find the maximum absolute quadratic rate at endpoints and interior vertices."""
        queries = list(self.times)
        for i, duration in enumerate(np.diff(self.times)):
            cubic, quadratic = self.spline.c[:2, i]
            if abs(cubic) > 1e-12:
                local = -quadratic / (3 * cubic)
                if 0 < local < duration:
                    queries.append(self.times[i] + local)
        return float(np.max(np.abs(self.spline(queries, 1))))

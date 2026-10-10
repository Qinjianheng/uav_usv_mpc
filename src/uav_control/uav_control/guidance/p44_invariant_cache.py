"""Content cache with sufficient Bernstein continuous force/tilt proof, exact fallback."""
import math

import numpy as np

from uav_control.guidance.follow_attitude_bounds import attitude_extrema
from uav_control.guidance.follow_fast_validation import InvariantCache
from uav_control.guidance.follow_profile import profiled, count


@profiled('attitude_bounds')
def bounded_attitude(xyz, durations, config):
    """Convex force hull gives an upper norm, positive z lower bound and tilt upper."""
    xyz, ts = np.asarray(xyz), np.asarray(durations)
    if (ts.ndim != 1 or len(ts) == 0 or xyz.shape != (len(ts), 6, 3)
            or np.any(ts <= 0) or not np.all(np.isfinite(np.r_[xyz.ravel(), ts]))):
        raise ValueError('INVALID_COEFFICIENTS')
    force = -np.polynomial.polynomial.polyder(xyz, 2, axis=1)
    force[:, 0, 2] += config.gravity
    scaled = force*ts[:, None, None]**np.arange(4)[None, :, None]
    transform = np.array([[math.comb(k, i)/math.comb(3, i) if i <= k else 0.
                           for i in range(4)] for k in range(4)])
    hull = np.einsum('ki,pia->pka', transform, scaled)
    pad = 1e-12*(1.+float(np.max(np.sum(np.abs(scaled), axis=1))))
    minimum = float(np.min(hull[:, :, 2]))-pad
    maximum = float(np.max(np.linalg.norm(hull, axis=2)))+pad
    horizontal = float(np.max(np.linalg.norm(hull[:, :, :2], axis=2)))+pad
    tilt = math.atan2(horizontal, minimum) if minimum > 0 else math.pi
    if (np.isfinite((minimum, maximum, tilt)).all()
            and minimum > config.minimum_specific_thrust+1e-7
            and maximum < config.maximum_specific_thrust-1e-7
            and tilt < config.maximum_tilt_rad-1e-7):
        count('attitude_bernstein_proofs')
        return dict(valid=True, minimum_thrust=minimum, maximum_thrust=maximum,
                    maximum_tilt=tilt, continuous_nominal_only=True, method='BERNSTEIN_SUFFICIENT')
    count('attitude_exact_fallbacks')
    return attitude_extrema(xyz, ts, config)


class P44InvariantCache(InvariantCache):
    """Same full content/policy/epoch key; no target, time or admission conclusion cache."""

    attitude_check = staticmethod(bounded_attitude)

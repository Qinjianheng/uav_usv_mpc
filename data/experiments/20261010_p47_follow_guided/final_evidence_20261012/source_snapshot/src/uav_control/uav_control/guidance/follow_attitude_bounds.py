"""Continuous nominal thrust/tilt extrema from complete polynomial acceleration."""
import math
import numpy as np
from uav_control.guidance.follow_profile import profiled, count


def roots_inside(polynomial, duration):
    """Numerically real stationary points plus endpoints; no sampling-grid shortcut."""
    count('root_searches')
    roots = np.polynomial.polynomial.polyroots(polynomial)
    return [0., duration]+[float(r.real) for r in roots
                           if abs(r.imag) < 1e-8 and 0 < r.real < duration]


@profiled('attitude_extrema')
def attitude_extrema(xyz, durations, config):
    """Evaluate squared force and horizontal/vertical force ratio extrema exactly in model."""
    poly = np.polynomial.polynomial
    acceleration = poly.polyder(np.asarray(xyz), 2, axis=1)
    force = -acceleration
    force[:, 0, 2] += config.gravity
    minimum, maximum, tilt = float('inf'), 0., 0.
    for f, duration in zip(force, durations):
        square = np.zeros(7)
        for c in f.T:
            product = poly.polymul(c, c)
            square[:len(product)] += product
        # polymul may trim trailing zeros: pad each component before adding.
        values = [float(np.linalg.norm(poly.polyval(t, f)))
                  for t in roots_inside(poly.polyder(square), duration)]
        minimum, maximum = min(minimum, min(values)), max(maximum, max(values))
        z = f[:, 2]
        z_min = min(float(poly.polyval(t, z)) for t in roots_inside(poly.polyder(z), duration))
        if z_min <= 0:
            tilt = math.pi
            continue
        horizontal = poly.polyadd(poly.polymul(f[:, 0], f[:, 0]),
                                  poly.polymul(f[:, 1], f[:, 1]))
        numerator = poly.polysub(poly.polymul(poly.polyder(horizontal), z),
                                 2*poly.polymul(horizontal, poly.polyder(z)))
        for t in roots_inside(numerator, duration):
            value = poly.polyval(t, f)
            tilt = max(tilt, math.atan2(np.linalg.norm(value[:2]), value[2]))
    valid = (minimum >= config.minimum_specific_thrust-1e-6
             and maximum <= config.maximum_specific_thrust+1e-6
             and tilt <= config.maximum_tilt_rad+1e-6)
    return dict(valid=valid, minimum_thrust=minimum, maximum_thrust=maximum,
                maximum_tilt=tilt, continuous_nominal_only=True)

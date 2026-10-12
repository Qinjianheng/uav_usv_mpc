"""Fit the marked validation sphere using known radius and ToF range noise."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import least_squares


@dataclass(frozen=True)
class SphereFit:
    """Camera FLU center and conservative largest-axis standard deviation."""

    center: tuple
    position_std: float


def fit_tof_sphere(points, radius, *, noise_std, range_noise_scale):
    """Reject incompatible surfaces and unobservable fits without using truth."""
    points = np.asarray(points, dtype=float)
    parameters = (radius, noise_std, range_noise_scale)
    if (not all(math.isfinite(value) for value in parameters)
            or radius <= 0 or noise_std <= 0 or range_noise_scale < 0):
        raise ValueError('radius/noise must be finite, positive and ordered')
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 12
            or not np.isfinite(points).all() or np.any(points[:, 0] <= 0)):
        return None
    points = points[::max(1, math.ceil(len(points) / 512))]
    ranges = np.linalg.norm(points, axis=1)
    sigma = noise_std + range_noise_scale * ranges
    surface = np.median(points, axis=0)
    initial = surface + radius * surface / np.linalg.norm(surface)

    def residual(center):
        return (np.linalg.norm(center - points, axis=1) - radius) / sigma

    def jacobian(center):
        delta = center - points
        lengths = np.maximum(np.linalg.norm(delta, axis=1), 1e-12)
        return delta / (lengths * sigma)[:, None]

    fit = least_squares(
        residual, initial, jac=jacobian, loss='soft_l1', f_scale=1.,
        max_nfev=30, ftol=1e-5, xtol=1e-6, gtol=1e-5,
    )
    if not fit.success or not np.isfinite(fit.x).all() or fit.x[0] <= 0:
        return None
    unit_rays = points / ranges[:, None]

    def radial_residual(center):
        along = unit_rays @ center
        discriminant = radius ** 2 - (center @ center - along ** 2)
        predicted = along - np.sqrt(np.maximum(discriminant, 0.))
        # A missed sphere intersection contributes an explicit geometry cost;
        # never discard inconvenient rays from the model comparison.
        missed = np.maximum(-discriminant, 0.) / (radius * sigma)
        return np.concatenate(((predicted - ranges) / sigma, missed))

    radial = least_squares(
        radial_residual, fit.x, loss='soft_l1', f_scale=1.,
        max_nfev=30, ftol=1e-5, xtol=1e-6, gtol=1e-5,
    )
    if not radial.success or not np.isfinite(radial.x).all() or radial.x[0] <= 0:
        return None
    all_residuals = np.abs(radial_residual(radial.x))
    normalized = all_residuals[:len(points)]
    # Gaussian measurement budget: tolerate isolated outliers, reject mixtures.
    if (np.percentile(normalized, 95) > 3.
            or np.percentile(all_residuals[len(points):], 95) > 3.):
        return None
    # A first-return surface must lie in front of its center along the rays.
    if np.median(np.sum((radial.x - points) * unit_rays, axis=1)) <= 0:
        return None
    # Compare competing three-parameter surfaces on the same radial samples.
    # A plane can fit a small / distant cap when curvature is below the noise;
    # demand evidence for the sphere rather than just compatibility with it.
    weight = ranges ** 2 / sigma
    plane_initial = np.linalg.lstsq(
        unit_rays * weight[:, None], weight / ranges, rcond=None,
    )[0]
    plane = least_squares(
        lambda normal: (1. / np.maximum(unit_rays @ normal, 1e-6) - ranges) / sigma,
        plane_initial, loss='soft_l1', f_scale=1., max_nfev=30,
    )
    if not plane.success or not np.isfinite(plane.cost):
        return None
    if 2. * (plane.cost - radial.cost) < 9.:
        return None
    singular = np.linalg.svd(radial.jac[:len(points)], compute_uv=False)
    if singular[-1] <= 1e-6 * singular[0]:
        return None
    # Range sigma bounds the surface-normal error; isotropic covariance using
    # its largest axis stays conservative after the camera-to-NED rotation.
    inflation = max(1., float(np.sqrt(np.mean(np.minimum(normalized, 3.) ** 2))))
    position_std = inflation / singular[-1]
    if position_std > radius / 2.:
        return None
    return SphereFit(tuple(float(value) for value in radial.x), position_std)

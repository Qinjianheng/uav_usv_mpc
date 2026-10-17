"""Known-radius localization must accept modeled ToF and reject wrong geometry."""

import importlib
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from uav_control.perception.tof_depth_model import TofDepthModel


def fitting():
    name = 'uav_control.perception.tof_sphere_localization'
    assert importlib.util.find_spec(name), 'noise-aware sphere fitting is missing'
    return importlib.import_module(name).fit_tof_sphere


def sphere(center):
    path = Path(__file__).resolve().parents[3] / 'scripts/vision_p4_synthetic_sphere.py'
    spec = importlib.util.spec_from_file_location('synthetic_sphere', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.render(center)


def points(mask, depth, rays):
    valid = mask & np.isfinite(depth)
    return depth[valid, None] * rays[valid]


@pytest.mark.parametrize('center', [
    (1., 0., 0.), (3., .4, -.1), (8., 2.5, .3), (15., -3., .4),
])
def test_modeled_noise_and_missing_returns_recover_center(center):
    mask, depth, rays = sphere(center)
    model = TofDepthModel(seed=73)
    errors = []
    for _ in range(8):
        fit = fitting()(points(mask, model.measure(depth), rays), .25,
                        noise_std=.01, range_noise_scale=.001)
        assert fit is not None
        errors.append(np.linalg.norm(np.asarray(fit.center) - center))
        assert 0 < fit.position_std <= .125
    assert np.sqrt(np.mean(np.square(errors))) < .05


def test_sparse_far_observations_require_curvature_evidence():
    center = (24., 0., 0.)
    mask, depth, rays = sphere(center)
    model = TofDepthModel(seed=73)
    errors = []
    # Only 21 ideal pixels cover this sphere. Some noisy frames carry too
    # little curvature evidence; do not require them all to produce a pose.
    for _ in range(8):
        fit = fitting()(points(mask, model.measure(depth), rays), .25,
                        noise_std=.01, range_noise_scale=.001)
        if fit is not None:
            errors.append(np.linalg.norm(np.asarray(fit.center) - center))
    assert 0 < len(errors) < 8
    assert max(errors) < .05


def test_clipped_close_sphere_remains_usable_without_complete_silhouette():
    center = (1., .5, .3)
    mask, depth, rays = sphere(center)
    cutoff = int(np.median(np.where(mask)[1]))
    mask[:, :cutoff] = False
    fit = fitting()(points(mask, TofDepthModel(seed=9).measure(depth), rays), .25,
                    noise_std=.01, range_noise_scale=.001)
    assert fit is not None
    assert fit.center == pytest.approx(center, abs=.025)


@pytest.mark.parametrize('kind', ['plane', 'tiny_cap', 'wrong_radius', 'mixture'])
def test_wrong_or_unobservable_geometry_is_rejected(kind):
    mask, depth, rays = sphere((5., 0., 0.))
    if kind == 'plane':
        depth[mask] = 4.75
    elif kind == 'tiny_cap':
        y, z = np.meshgrid(np.linspace(-1e-4, 1e-4, 10),
                           np.linspace(-1e-4, 1e-4, 10))
        cloud = np.column_stack((np.full(100, 4.75), y.ravel(), z.ravel()))
        assert fitting()(cloud, .25, noise_std=.01,
                         range_noise_scale=.001) is None
        return
    elif kind == 'wrong_radius':
        fit = fitting()(points(mask, depth, rays), .1,
                        noise_std=.01, range_noise_scale=.001)
        assert fit is None
        return
    else:
        rows, cols = np.where(mask)
        depth[rows[::3], cols[::3]] += .5
    fit = fitting()(points(mask, depth, rays), .25,
                    noise_std=.01, range_noise_scale=.001)
    assert fit is None


def test_integration_preserves_ideal_path_and_exposes_fit_uncertainty():
    from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
    mask, depth, _ = sphere((8., 0., 0.))
    ideal = target_geometry_from_rgbd(
        mask, depth, 1.74, .05, 25., .25, require_valid_sphere=True,
    )
    assert ideal is not None
    noisy = TofDepthModel(seed=4).measure(depth)
    assert target_geometry_from_rgbd(
        mask, noisy, 1.74, .05, 25., .25, require_valid_sphere=True,
    ) is None
    fit = target_geometry_from_rgbd(
        mask, noisy, 1.74, .05, 25., .25, require_valid_sphere=True,
        sphere_fit_mode='tof', sphere_noise_std=.01, sphere_range_noise_scale=.001,
    )
    assert fit is not None
    assert fit.center_camera == pytest.approx((8., 0., 0.), abs=.03)
    assert fit.fit_position_std > 0


@pytest.mark.parametrize('distance', [12., 15., 20., 24.])
def test_far_noisy_plane_cannot_masquerade_as_validation_sphere(distance):
    mask, depth, rays = sphere((distance, 0., 0.))
    depth[mask] = distance - .25
    model = TofDepthModel(seed=0)
    for _ in range(20):
        fit = fitting()(points(mask, model.measure(depth), rays), .25,
                        noise_std=.01, range_noise_scale=.001)
        assert fit is None


@pytest.mark.parametrize('distance', [8., 12., 15.])
def test_eight_percent_wrong_depth_mixture_is_rejected(distance):
    mask, depth, rays = sphere((distance, 0., 0.))
    rows, cols = np.where(mask)
    depth[rows[::12], cols[::12]] += .5
    fit = fitting()(points(mask, depth, rays), .25,
                    noise_std=.01, range_noise_scale=.001)
    assert fit is None

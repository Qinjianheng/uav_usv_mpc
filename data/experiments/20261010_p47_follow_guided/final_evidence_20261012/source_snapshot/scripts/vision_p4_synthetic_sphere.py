#!/usr/bin/env python3
"""Check X-depth sphere-center approximations against ideal rays."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from uav_control.perception.front_tof_monitor import camera_intrinsics
from uav_control.perception.rgbd_target_localizer import (
    target_geometry_from_rgbd,
)

WIDTH, HEIGHT, FOV, RADIUS = 640, 480, 1.74, 0.25
OUTPUT = (Path(__file__).resolve().parents[1]
          / 'data/experiments/current/p4_20260927_synthetic_sphere.json')


def render(center):
    fx, fy, cx, cy = camera_intrinsics(WIDTH, HEIGHT, FOV)
    rows, cols = np.indices((HEIGHT, WIDTH))
    rays = np.stack((np.ones((HEIGHT, WIDTH)),
                     -(cols - cx) / fx, -(rows - cy) / fy), axis=-1)
    center = np.asarray(center, dtype=float)
    dot = rays @ center
    ray_norm2 = np.sum(rays * rays, axis=-1)
    disc = dot * dot - ray_norm2 * (center @ center - RADIUS ** 2)
    mask = (disc >= 0) & (dot > 0)
    depth = np.full((HEIGHT, WIDTH), np.inf)
    depth[mask] = (dot[mask] - np.sqrt(disc[mask])) / ray_norm2[mask]
    return mask, depth, rays


def evaluate(center):
    mask, depth, rays = render(center)
    geometry = target_geometry_from_rgbd(
        mask, depth, FOV, 0.2, 25.0, RADIUS,
    )
    fx, fy, cx, cy = geometry.intrinsics
    u, v = geometry.projection_center
    central_ray = np.array((1.0, -(u - cx) / fx, -(v - cy) / fy))
    # Reproduce P4's historical A independently of the current online rule.
    old = (geometry.depth_median + RADIUS) * central_ray
    ray_corrected_forward = (
        geometry.depth_median + RADIUS / np.linalg.norm(central_ray)
    )
    ray_corrected = ray_corrected_forward * central_ray
    points = (depth[mask, None] * rays[mask])
    fit = least_squares(
        lambda candidate: np.linalg.norm(points - candidate, axis=1)
        - RADIUS,
        old, loss='soft_l1', f_scale=0.01,
        max_nfev=100,
    )
    actual = np.asarray(center, dtype=float)
    return {
        'true_center_camera_flu_m': actual.tolist(),
        'visible_pixels': int(mask.sum()),
        'depth_median_m': geometry.depth_median,
        'depth_mad_m': geometry.depth_mad,
        'median_plus_radius': {
            'estimate_m': old.tolist(),
            'error_3d_m': float(np.linalg.norm(old - actual)),
        },
        'median_plus_ray_radius': {
            'estimate_m': ray_corrected.tolist(),
            'error_3d_m': float(np.linalg.norm(ray_corrected - actual)),
        },
        'known_radius_surface_fit': {
            'estimate_m': fit.x.tolist(),
            'error_3d_m': float(np.linalg.norm(fit.x - actual)),
            'converged': bool(fit.success),
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    scenarios = {
        'center_5m': (5.0, 0.0, 0.0),
        'center_7m': (7.0, 0.0, 0.0),
        'left_7m': (7.0, 2.5, 0.0),
        'right_7m': (7.0, -2.5, 0.0),
        'left_15m': (15.0, 3.0, 0.0),
        'right_15m': (15.0, -3.0, 0.0),
    }
    result = {name: evaluate(center) for name, center in scenarios.items()}
    with args.output.open('x') as file:
        json.dump(result, file, indent=2, ensure_ascii=False,
                  allow_nan=False)
        file.write('\n')
    print(args.output)
    for name, item in result.items():
        print(name, item['visible_pixels'], *(
            round(item[key]['error_3d_m'], 6)
            for key in ('median_plus_radius', 'median_plus_ray_radius',
                        'known_radius_surface_fit')
        ))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Compare RGB-D sphere recovery and acquisition skew on saved real ROIs."""

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares

from uav_control.evaluation.pose_frame_diagnostics import quaternion_rotation
from uav_control.perception.rgbd_target_localizer import (
    target_geometry_from_rgbd,
)


CAMERA_TRANSLATION = np.array((0.35, 0.0, 0.19))
CAMERA_PITCH = 0.20944


def camera_vector(model, entity):
    """Physical Gazebo camera vector in FLU using the actual SDF mount."""
    rotation = quaternion_rotation(model[3:7])
    pitch = CAMERA_PITCH
    mount = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)],
    ])
    camera = np.asarray(model[:3]) + rotation @ CAMERA_TRANSLATION
    target = np.array((entity[1], entity[0], -entity[2]))
    return mount.T @ rotation.T @ (target - camera)


def projected_pixel(center, intrinsics):
    fx, fy, cx, cy = intrinsics
    return np.array((cx - center[1] * fx / center[0],
                     cy - center[2] * fy / center[0]))


def roi_points(mask, depth, bounds, intrinsics, minimum=0.2,
               maximum=25.0):
    left, top, _, _ = bounds
    fx, fy, cx, cy = intrinsics
    rows, cols = np.nonzero(mask & np.isfinite(depth)
                            & (depth >= minimum) & (depth <= maximum))
    if len(rows) == 0:
        return np.empty((0, 3)), np.empty((0, 2))
    z = depth[rows, cols].astype(float)
    u = cols.astype(float) + left
    v = rows.astype(float) + top
    points = np.stack((z, -(u - cx) * z / fx, -(v - cy) * z / fy), axis=1)
    return points, np.stack((u, v), axis=1)


def sphere_surface_x(center, pixels, intrinsics, radius):
    fx, fy, cx, cy = intrinsics
    u, v = pixels.T
    ray = np.stack((np.ones(len(u)), -(u - cx) / fx,
                    -(v - cy) / fy), axis=1)
    center = np.asarray(center, dtype=float)
    dot = ray @ center
    scale = np.einsum('ij,ij->i', ray, ray)
    discriminant = dot ** 2 - scale * (center @ center - radius ** 2)
    result = np.full(len(u), np.nan)
    valid = (discriminant >= 0) & (dot > 0)
    result[valid] = (dot[valid] - np.sqrt(discriminant[valid])) / scale[valid]
    return result


def fit_sphere(points, initial, radius, robust=False):
    """Known-radius surface fit using only RGB-D points and radius."""
    if len(points) < 12:
        return None
    if len(points) > 600:
        indices = np.linspace(0, len(points) - 1, 600).astype(int)
        points = points[indices]
    result = least_squares(
        lambda center: np.linalg.norm(points - center, axis=1) - radius,
        initial, loss='soft_l1' if robust else 'linear',
        f_scale=0.02, max_nfev=50,
    )
    if not result.success or not np.all(np.isfinite(result.x)):
        return None
    if result.x[0] <= 0:
        return None
    return result.x


def reconstruct(mask_roi, depth_roi, shape, bounds, fov):
    left, top, right, bottom = bounds
    mask = np.zeros(shape, dtype=bool)
    depth = np.full(shape, np.nan, dtype=np.float32)
    mask[top:bottom, left:right] = mask_roi
    depth[top:bottom, left:right] = depth_roi
    return mask, depth


def metrics(errors, timings, attempted):
    if not errors:
        return {'valid': 0, 'attempted': attempted, 'valid_rate': 0.0}
    e = np.asarray(errors, dtype=float)
    norms = np.linalg.norm(e, axis=1)
    t = np.asarray(timings, dtype=float)
    return {
        'valid': len(e), 'attempted': attempted,
        'valid_rate': len(e) / attempted if attempted else math.nan,
        'rmse_3d_m': float(np.sqrt(np.mean(norms ** 2))),
        'p50_3d_m': float(np.percentile(norms, 50)),
        'p95_3d_m': float(np.percentile(norms, 95)),
        'max_3d_m': float(norms.max()),
        'signed_mean_xyz_m': e.mean(axis=0).tolist(),
        'compute_p50_ms': float(np.percentile(t, 50) * 1000),
        'compute_p95_ms': float(np.percentile(t, 95) * 1000),
    }


def analyze_capture(directory, scenario):
    records = [json.loads(line) for line in
               (directory / 'roi.jsonl').read_text().splitlines()]
    errors = {name: [] for name in ('A_median_radius', 'B_ray_radius',
                                    'C_sphere_fit', 'D_robust_fit')}
    timings = {name: [] for name in errors}
    skew = []
    projection = []
    projection_depth = []
    surface = []
    online_reconstruction_differences = []
    online_b_differences = []
    invalid_roi_diagnostics = []
    counts = {'saved': len(records), 'online_valid': 0,
              'complete_pose': 0, 'mad_high': 0}
    for item in records:
        counts['mad_high'] += item['rejection_reason'] == 'DEPTH_MAD_HIGH'
        if not item['valid']:
            if (item['intrinsics_fx_fy_cx_cy'][0] <= 0
                    or item['intrinsics_fx_fy_cx_cy'][1] <= 0
                    or item['mask_bbox_ltrb_inclusive'] == [0, 0, 0, 0]):
                invalid_roi_diagnostics.append({
                    'file': item['file'],
                    'rejection_reason': item['rejection_reason'],
                    'usable': False,
                    'reason': (
                        'historical_invalid_observation_missing_geometry'
                    ),
                })
                continue
            with np.load(directory / item['file']) as roi:
                bad_mask, bad_depth = roi['mask'], roi['depth']
            bad_entity = item['entity_at_depth']['value']
            bad_model = item['model_at_depth']['value']
            if bad_entity is not None and bad_model is not None:
                bad_points, bad_pixels = roi_points(
                    bad_mask, bad_depth, item['roi_bounds_ltrb'],
                    item['intrinsics_fx_fy_cx_cy'],
                )
                bad_expected = sphere_surface_x(
                    camera_vector(bad_model, bad_entity), bad_pixels,
                    item['intrinsics_fx_fy_cx_cy'], item['target_radius'],
                )
                good_bad = np.isfinite(bad_expected)
                invalid_roi_diagnostics.append({
                    'file': item['file'],
                    'rejection_reason': item['rejection_reason'],
                    'valid_depth_points': len(bad_points),
                    'depth_mad_m': item['depth_mad'],
                    'depth_range_m': (
                        [float(bad_points[:, 0].min()),
                         float(bad_points[:, 0].max())]
                        if len(bad_points) else []
                    ),
                    'fraction_rays_hitting_true_sphere_at_depth': (
                        float(np.mean(good_bad)) if len(good_bad) else 0.0
                    ),
                    'fraction_points_within_10cm_true_surface': (
                        float(np.mean(np.abs(
                            bad_points[good_bad, 0]
                            - bad_expected[good_bad]
                        ) <= 0.1)) if good_bad.any() else 0.0
                    ),
                })
            continue
        counts['online_valid'] += 1
        entity_r = item['entity_at_rgb']['value']
        entity_d = item['entity_at_depth']['value']
        model_r = item['model_at_rgb']['value']
        model_d = item['model_at_depth']['value']
        if any(x is None for x in (entity_r, entity_d, model_r, model_d)):
            continue
        counts['complete_pose'] += 1
        true_r = camera_vector(model_r, entity_r)
        true_d = camera_vector(model_d, entity_d)
        intrinsics = item['intrinsics_fx_fy_cx_cy']
        radius = float(item['target_radius'])
        with np.load(directory / item['file']) as roi:
            roi_mask, roi_depth = roi['mask'], roi['depth']
        mask, depth = reconstruct(roi_mask, roi_depth,
                                  item['image_shape'],
                                  item['roi_bounds_ltrb'], 1.74)
        start = time.perf_counter()
        geom = target_geometry_from_rgbd(mask, depth, 1.74, 0.2, 25.0,
                                         radius)
        elapsed = time.perf_counter() - start
        if geom is None:
            continue
        u, v = geom.projection_center
        fx, fy, cx, cy = intrinsics
        ray = np.array((1.0, -(u - cx) / fx, -(v - cy) / fy))
        # Keep the historical online A baseline explicit after the live
        # localizer changes to B; both use the same decoded ROI and pixels.
        old_forward = geom.depth_median + radius
        old = np.array((
            old_forward,
            -(u - cx) * old_forward / fx,
            -(v - cy) * old_forward / fy,
        ))
        online_reconstruction_differences.append(float(np.linalg.norm(
            old - np.asarray(item['online_center_camera_flu'])
        )))
        errors['A_median_radius'].append(old - true_r)
        timings['A_median_radius'].append(elapsed)
        start = time.perf_counter()
        candidate = (geom.depth_median + radius / np.linalg.norm(ray)) * ray
        online_b_differences.append(float(np.linalg.norm(
            candidate - np.asarray(item['online_center_camera_flu'])
        )))
        timings['B_ray_radius'].append(time.perf_counter() - start)
        errors['B_ray_radius'].append(candidate - true_r)
        points, pixels = roi_points(roi_mask, roi_depth,
                                    item['roi_bounds_ltrb'], intrinsics)
        for name, robust in (('C_sphere_fit', False),
                             ('D_robust_fit', True)):
            start = time.perf_counter()
            candidate = fit_sphere(points, old, radius, robust)
            elapsed = time.perf_counter() - start
            if candidate is not None:
                errors[name].append(candidate - true_r)
                timings[name].append(elapsed)
        projection.append((np.array(geom.projection_center)
                           - projected_pixel(true_r, intrinsics)).tolist())
        projection_depth.append((
            np.array(geom.projection_center)
            - projected_pixel(true_d, intrinsics)
        ).tolist())
        expected_x = sphere_surface_x(true_d, pixels, intrinsics, radius)
        expected_rgb_x = sphere_surface_x(true_r, pixels, intrinsics, radius)
        good = np.isfinite(expected_x)
        if good.any():
            surface.append({
                'median_observed_minus_tdepth_surface_x_m': float(np.median(
                    points[good, 0] - expected_x[good])),
                'median_abs_observed_minus_tdepth_surface_x_m': float(
                    np.median(np.abs(points[good, 0] - expected_x[good]))),
                'median_abs_observed_minus_trgb_surface_x_m': float(
                    np.median(np.abs(points[np.isfinite(expected_rgb_x), 0]
                                     - expected_rgb_x[np.isfinite(
                                         expected_rgb_x)]))
                    if np.isfinite(expected_rgb_x).any() else math.nan),
                'inlier_fraction_10cm': float(np.mean(
                    np.abs(points[good, 0] - expected_x[good]) <= 0.1)),
                'ray_coverage': float(np.mean(good)),
            })
        rotation = quaternion_rotation(model_r[3:7])
        cp, sp = math.cos(CAMERA_PITCH), math.sin(CAMERA_PITCH)
        mount = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)))
        world_to_camera = (rotation @ mount).T
        target_delta_enu = np.array((entity_d[1] - entity_r[1],
                                     entity_d[0] - entity_r[0],
                                     -(entity_d[2] - entity_r[2])))
        uav_delta_enu = np.asarray(model_d[:3]) - np.asarray(model_r[:3])
        target_contribution = world_to_camera @ target_delta_enu
        uav_translation_contribution = -world_to_camera @ uav_delta_enu
        x_rgb = sphere_surface_x(true_r, pixels, intrinsics, radius)
        x_depth = sphere_surface_x(true_d, pixels, intrinsics, radius)
        common = np.isfinite(x_rgb) & np.isfinite(x_depth)
        theoretical = math.nan
        if common.any():
            theoretical = float(np.median(x_depth[common])
                                - np.median(x_rgb[common]))
        skew.append({
            'rgb_depth_skew_s': item['rgb_depth_skew'],
            'target_translation_camera_flu_m':
                target_contribution.tolist(),
            'uav_translation_camera_flu_m':
                uav_translation_contribution.tolist(),
            'relative_translation_camera_flu_m':
                (target_contribution + uav_translation_contribution).tolist(),
            'relative_camera_change_flu_m': (true_d - true_r).tolist(),
            'uav_rotation_remainder_camera_flu_m':
                (true_d - true_r - target_contribution
                 - uav_translation_contribution).tolist(),
            'theoretical_median_depth_x_shift_m': theoretical,
            'theoretical_center_shift_camera_flu_m': (
                (theoretical * ray).tolist()
                if math.isfinite(theoretical) else [math.nan] * 3
            ),
        })
    summary = {'scenario': scenario, 'directory': str(directory),
               'counts': counts,
               'candidate_metrics': {
                   name: metrics(errors[name], timings[name],
                                 counts['complete_pose'])
                   for name in errors},
               'projection_error_at_rgb_px': projection,
               'projection_error_at_depth_px': projection_depth,
               'depth_surface_checks': surface,
               'invalid_roi_diagnostics': invalid_roi_diagnostics,
               'max_online_A_reconstruction_difference_m': (
                   max(online_reconstruction_differences)
                   if online_reconstruction_differences else None
               ),
               'max_online_B_reconstruction_difference_m': (
                   max(online_b_differences)
                   if online_b_differences else None
               ),
               'skew_per_frame': skew}
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', action='append', required=True,
                        help='name=/absolute/path/to/capture_ROI_directory')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {}
    for entry in args.scenario:
        name, path = entry.split('=', 1)
        result[name] = analyze_capture(Path(path), name)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False,
                  allow_nan=True)
        stream.write('\n')
    for name, item in result.items():
        print(name, item['counts'])
        for method, value in item['candidate_metrics'].items():
            print(' ', method, value)


if __name__ == '__main__':
    main()

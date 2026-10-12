#!/usr/bin/env python3
"""Offline native-time GNSS/EKF audit. Never feeds estimates to the vehicle."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def project_gnss(latitude, longitude, ref_lat, ref_lon):
    """PX4 spherical azimuthal equidistant projection (earth radius 6371000)."""
    lat, lon = np.radians(latitude), np.radians(longitude)
    lat0, lon0 = np.radians(ref_lat), np.radians(ref_lon)
    dlon = lon - lon0
    cos_c = np.sin(lat0) * np.sin(lat) + np.cos(lat0) * np.cos(lat) * np.cos(dlon)
    c = np.arccos(np.clip(cos_c, -1, 1))
    scale = np.ones_like(c)
    np.divide(c, np.sin(c), out=scale, where=np.abs(c) > 1e-12)
    return 6371000 * scale[:, None] * np.column_stack([
        np.cos(lat0) * np.sin(lat) - np.sin(lat0) * np.cos(lat) * np.cos(dlon),
        np.cos(lat) * np.sin(dlon)])


def interpolate_inside(times, source_times, values, maximum_gap_us=100000):
    """Require a bounded bracket; return NaN for missing, distant or reset data."""
    times = np.asarray(times, dtype=float)
    source_times = np.asarray(source_times, dtype=float)
    values = np.asarray(values, dtype=float)
    if len(source_times) < 2 or np.any(np.diff(source_times) <= 0):
        raise ValueError('native source times must be strictly increasing')
    right = np.searchsorted(source_times, times, side='right')
    right = np.clip(right, 1, len(source_times) - 1)
    left = right - 1
    gap = source_times[right] - source_times[left]
    valid = ((times >= source_times[0]) & (times <= source_times[-1])
             & (gap <= maximum_gap_us))
    exact_left = times == source_times[left]
    exact_right = times == source_times[right]
    valid |= exact_left | exact_right
    fraction = (times - source_times[left]) / gap
    result = values[left] + fraction[:, None] * (values[right] - values[left])
    result[exact_left] = values[left[exact_left]]
    result[exact_right] = values[right[exact_right]]
    result[~valid] = np.nan
    return result


def vector(data, keys):
    return np.column_stack([data[key] for key in keys])


def metrics(error, velocity, mask):
    mask = mask & np.isfinite(error).all(axis=1) & np.isfinite(velocity).all(axis=1)
    e, v = error[mask], velocity[mask]
    if not len(e):
        return {'samples': 0}
    speed = np.linalg.norm(v[:, :2], axis=1)
    along = np.sum(e[:, :2] * v[:, :2], axis=1) / np.maximum(speed, 1e-9)
    return {
        'samples': len(e),
        'horizontal_rmse_m': float(np.sqrt(np.mean(np.sum(e[:, :2] ** 2, axis=1)))),
        'mean_xyz_m': e.mean(axis=0).tolist(),
        'horizontal_p95_m': float(np.percentile(np.linalg.norm(e[:, :2], axis=1), 95)),
        'along_velocity_mean_m': float(np.mean(along)),
    }


def audit(path):
    ulog = ULog(str(path))
    data = {item.name: item.data for item in ulog.data_list if item.multi_id == 0}
    gps, pose, truth = (data[name] for name in (
        'vehicle_gps_position', 'vehicle_local_position', 'vehicle_local_position_groundtruth'))
    gt_t = truth['timestamp_sample'].astype(float)
    gt_p = vector(truth, ['x', 'y', 'z'])
    gt_v = vector(truth, ['vx', 'vy', 'vz'])
    gps_p = np.column_stack([
        project_gnss(gps['latitude_deg'], gps['longitude_deg'],
                     truth['ref_lat'][0], truth['ref_lon'][0]),
        truth['ref_alt'][0] - gps['altitude_msl_m']])
    gps_t = gps['timestamp'].astype(float)
    gps_sample_t = gps['timestamp_sample'].astype(float)
    raw_truth = interpolate_inside(gps_t, gt_t, gt_p)
    sample_truth = interpolate_inside(gps_sample_t, gt_t, gt_p)
    gps_v = interpolate_inside(gps_t, gt_t, gt_v)
    common = np.isfinite(raw_truth).all(axis=1) & np.isfinite(sample_truth).all(axis=1)
    moving = common & (np.linalg.norm(gps_v[:, :2], axis=1) > 3)
    pose_t = pose['timestamp_sample'].astype(float)
    pose_v = interpolate_inside(pose_t, gt_t, gt_v)
    pose_error = vector(pose, ['x', 'y', 'z']) - interpolate_inside(pose_t, gt_t, gt_p)
    origin_delta = np.column_stack([
        project_gnss(pose['ref_lat'], pose['ref_lon'], truth['ref_lat'][0], truth['ref_lon'][0]),
        truth['ref_alt'][0] - pose['ref_alt']])
    counts = np.unique((gps_t - gps_sample_t) / 1000, return_counts=True)
    return {
        'ulog': str(path.resolve()),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'method': 'Native sample time; bounded brackets <=100ms; no extrapolation or fitted lag. '
                  'GNSS errors use the GT map origin; EKF errors retain the actual local origin. '
                  'Moving threshold >3m/s; stationary <0.1m/s. GPS comparisons share samples.',
        'parameters': {key: ulog.initial_parameters.get(key) for key in (
            'SENS_GPS0_ID', 'SENS_GPS1_ID', 'SENS_GPS0_DELAY', 'SENS_GPS1_DELAY',
            'EKF2_GPS_P_NOISE', 'EKF2_GPS_V_NOISE', 'EKF2_TAU_POS', 'EKF2_TAU_VEL')},
        'gps_devices': np.unique(gps['device_id']).astype(int).tolist(),
        'gps_timestamp_minus_sample_ms': dict(zip(
            (str(x) for x in counts[0]), (int(x) for x in counts[1]))),
        'gps_raw_moving': metrics(gps_p - raw_truth, gps_v, moving),
        'gps_sample_moving': metrics(gps_p - sample_truth, gps_v, moving),
        'ekf_moving': metrics(pose_error, pose_v, np.linalg.norm(pose_v[:, :2], axis=1) > 3),
        'ekf_stationary': metrics(pose_error, pose_v,
                                  np.linalg.norm(pose_v[:, :2], axis=1) < .1),
        'ekf_all': metrics(pose_error, pose_v, np.ones(len(pose_t), dtype=bool)),
        'ekf_common_origin_moving': metrics(
            pose_error + origin_delta, pose_v, np.linalg.norm(pose_v[:, :2], axis=1) > 3),
        'ekf_origin_offset_median_xyz_m': np.nanmedian(origin_delta, axis=0).tolist(),
        'xy_reset_counters': np.unique(pose['xy_reset_counter']).astype(int).tolist(),
        'heading_reset_counters': np.unique(pose['heading_reset_counter']).astype(int).tolist(),
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.ulog)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))

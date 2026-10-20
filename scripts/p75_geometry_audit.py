#!/usr/bin/env python3
"""Audit fixed-layout sphere/capture incompatibility and a bounded 38-degree alternative."""
from dataclasses import fields
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from p75_offline_probe import attitude, sensor, INTR  # noqa: E402
from uav_control.control.p7_spatial_guidance import capture_endpoint_reference  # noqa: E402
from uav_control.guidance.camera_visibility import (  # noqa: E402
    CameraExtrinsics, TargetBoundingSphere, VisibilityConstraints, evaluate_visibility,
)
from uav_control.usv_motion_profiles import MotionParameters, UsvMotionProfile  # noqa: E402


def distance_upper_bound(rotation, pitch, radius=.25, center_depth=0.):
    """Bound camera range over the full capture ball intersected with positive depth."""
    rotation = np.asarray(rotation)
    center = rotation@np.array((.18, 0., -.39))+np.array((0., 0., .42))
    forward = rotation@np.array((math.cos(pitch), 0., math.sin(pitch)))
    projection = float(center@forward)
    plane = projection+center_depth
    norm = float(np.linalg.norm(center))
    if plane > radius:
        return 0.  # Empty half-ball, already impossible before sea/pixel checks.
    if norm < 1e-12 or plane <= -radius or -radius*projection/norm >= plane:
        return radius+norm
    perpendicular = float(np.linalg.norm(center-projection*forward))
    minimum_dot = plane*projection-math.sqrt(max(0., radius**2-plane**2))*perpendicular
    return math.sqrt(max(0., radius**2+norm**2-2*minimum_dot))


def run(output, angles=(28., 38.)):
    """Compare two mounts on one unchanged S cycle, with ideal matched-velocity poses."""
    output.mkdir(exist_ok=False, parents=True)
    config_path = ROOT/'src/uav_usv_bringup/config/p7_spatial_candidate.yaml'
    config = yaml.safe_load(config_path.read_text())
    p = config['moving_target']['ros__parameters']
    args = {f.name: p[f.name] for f in fields(MotionParameters) if f.name in p}
    args.update(linear_vx=p['velocity_x'], linear_vy=p['velocity_y'], velocity_z=p['velocity_z'])
    profile = UsvMotionProfile('variable_speed_s', MotionParameters(**args))
    samples = [profile.advance(.05) for _ in range(646)]
    result = []
    for degrees in angles:
        pitch = math.radians(degrees)
        extr = CameraExtrinsics.from_sdf_pose((.18, 0., .39), 0., pitch, 0.)
        c, s = math.cos(pitch), math.sin(pitch)
        mount = np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))
        stats = dict(angle=degrees, samples=480, whole_follow=0, valid_follow=0,
                     empty_full_capture_ball_by_bound=0, bound_max=0.,
                     finite_endpoint=0, valid_endpoint_rgbd=0, nominal_window=0, witnesses=[])
        for i in range(160, 640):
            target = samples[i]
            rotation = attitude(np.asarray(target.acceleration), target.yaw)
            bound = distance_upper_bound(rotation, pitch)
            stats['bound_max'] = max(stats['bound_max'], bound)
            stats['empty_full_capture_ball_by_bound'] += int(bound < .27-1e-9)
            relative = np.array((5*math.cos(target.yaw), 5*math.sin(target.yaw),
                                 5+target.position[2]))
            view = evaluate_visibility((0., 0., 0.), rotation, relative-(0., 0., .42),
                                       INTR, extr, TargetBoundingSphere(.25),
                                       VisibilityConstraints(.05, 25.))
            stats['whole_follow'] += int(view.whole_target_safe)
            stats['valid_follow'] += int(sensor(view.target_camera_flu)['valid'])
            endpoint = capture_endpoint_reference(
                rotation, target.position[2], relative, camera_pitch_down=pitch)
            if endpoint is None:
                continue
            stats['finite_endpoint'] += 1
            offset = rotation@np.array((.18, 0., -.39))+np.array((0., 0., .42))
            ray = mount.T@np.diag((1., -1., -1.))@rotation.T@(endpoint-offset)
            if not sensor(ray)['valid']:
                continue
            stats['valid_endpoint_rgbd'] += 1
            height_z = target.position[2]-endpoint[2]
            valid, minimum_gap = True, float('inf')
            for step in (0, 3, 6):
                future = samples[i+step]
                delta = future.yaw-target.yaw
                xy = np.array(((math.cos(delta), -math.sin(delta)),
                               (math.sin(delta), math.cos(delta))))@endpoint[:2]
                r = np.array((*xy, future.position[2]-height_z))
                pose = attitude(np.asarray(future.acceleration), future.yaw)
                offset = pose@np.array((.18, 0., -.39))+np.array((0., 0., .42))
                ray = mount.T@np.diag((1., -1., -1.))@pose.T@(r-offset)
                observation = sensor(ray)
                minimum_gap = min(minimum_gap, observation['surface_gap'])
                valid &= (observation['valid'] and observation['surface_gap'] >= .02
                          and np.linalg.norm(r) <= .24+1e-9 and height_z <= -.33+1e-9)
            stats['nominal_window'] += int(valid)
            if valid and len(stats['witnesses']) < 2:
                stats['witnesses'].append(dict(profile_time=(i+1)*.05,
                                               relative=endpoint.tolist(),
                                               minimum_gap=minimum_gap))
        result.append(stats)
    document = dict(scope='480 finite nominal steady-match S poses and 3-point .3s windows. '
                    'No scene flight, no global impossibility proof over all transient attitudes.',
                    bound='max ||r-c||, ||r||<=.25, n.r>=n.c; sea/pixel constraints omitted '
                    'so a bound below .27 proves empty at that fixed pose even before them.',
                    results=result)
    (output/'geometry.json').write_text(json.dumps(document, indent=2)+'\n')
    print(json.dumps(document, indent=2))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--angle', type=float, nargs='+', default=(28., 38.))
    options = parser.parse_args()
    run(options.output, options.angle)

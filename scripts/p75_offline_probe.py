#!/usr/bin/env python3
"""Run a bounded exact-sensor variable-S probe, never a flight qualification."""
import argparse
from collections import Counter
from dataclasses import asdict, fields
import importlib.util
import inspect
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/uav_control'))
from uav_control.control import p7_spatial_guidance as spatial  # noqa: E402
from uav_control.control.flight_guidance import (  # noqa: E402
    FlightGuidanceCore, FlightKinematicState,
)
from uav_control.control.p7_visual_guidance import GuidanceConfig  # noqa: E402
from uav_control.guidance.camera_visibility import (  # noqa: E402
    CameraIntrinsics, CameraExtrinsics, TargetBoundingSphere, VisibilityConstraints,
    evaluate_visibility,
)
from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd  # noqa: E402
from uav_control.usv_motion_profiles import MotionParameters, UsvMotionProfile  # noqa: E402

INTR = CameraIntrinsics.from_horizontal_fov(640, 480, 1.74)
YY, XX = np.indices((480, 640))
RAYS = np.stack((np.ones_like(XX), (INTR.cx-XX)/INTR.fx, (INTR.cy-YY)/INTR.fy), -1)
RAY_NORM2 = np.sum(RAYS*RAYS, axis=-1)


def sensor(center):
    """Render RGB independently of depth clipping and call the actual ideal fitter."""
    center = np.asarray(center)
    dot = RAYS@center
    disc = dot*dot-RAY_NORM2*(center@center-.25**2)
    depth = (dot-np.sqrt(np.maximum(0., disc)))/RAY_NORM2
    projected = (disc >= 0.) & (dot > 0.) & (depth > 0.)
    rgb = projected & (depth >= .05) & (depth <= 60.)
    valid = rgb & (depth >= .05) & (depth <= 25.)
    pixels, depths = int(rgb.sum()), int(valid.sum())
    ratio = depths/max(1, pixels)
    fit = None
    if pixels >= 3 and ratio >= .5:
        fit = target_geometry_from_rgbd(
            rgb, np.where(rgb, depth, np.nan), 1.74, .05, 25., .25,
            require_valid_sphere=True)
    reason = ('RGB_NEAR_CLIP' if pixels < 3 and np.count_nonzero(projected) >= 3
              else 'RGB_NOT_VISIBLE' if pixels < 3 else 'DEPTH_UNUSABLE' if ratio < .5
              else 'FIT_QUALITY' if fit is None else 'VALID')
    return dict(valid=fit is not None, reason=reason, projected_pixels=int(projected.sum()),
                rgb_pixels=pixels,
                depth_pixels=depths, depth_ratio=ratio,
                center=list(fit.center_camera) if fit else None,
                surface_gap=float(np.linalg.norm(center))-.25)


def attitude(acceleration, yaw):
    """Quasistatic thrust tilt from measured model acceleration, not nav extrapolation."""
    z = np.array((-acceleration[0], -acceleration[1], 9.81))
    z /= np.linalg.norm(z)
    x = np.array((math.cos(yaw), math.sin(yaw), 0.))
    x -= z*(z@x)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def run(output, start=32., frozen=None, config_file=None):
    """Evaluate one fixed lawful S phase through actual final FlightGuidance shaping."""
    output.mkdir(parents=True, exist_ok=False)
    module = spatial
    if frozen is not None:
        name = 'uav_control.control.p75_frozen'
        spec = importlib.util.spec_from_file_location(name, frozen)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    config_path = config_file or ROOT/'src/uav_usv_bringup/config/p7_spatial_candidate.yaml'
    config = yaml.safe_load(config_path.read_text())
    p = config['moving_target']['ros__parameters']
    args = {f.name: p[f.name] for f in fields(MotionParameters) if f.name in p}
    args.update(linear_vx=p['velocity_x'], linear_vy=p['velocity_y'], velocity_z=p['velocity_z'])
    profile = UsvMotionProfile('variable_speed_s', MotionParameters(**args))
    for _ in range(round(start/.05)):
        target = profile.advance(.05)
    tp, tv = np.array(target.position), np.array(target.velocity)
    up = tp.copy()
    up[:2] -= 5*tv[:2]/np.linalg.norm(tv[:2])
    up[2] = -5.
    uv = tv.copy()
    uv[2] = 0.
    acc, yaw = np.array(target.acceleration), math.atan2(tv[1], tv[0])
    cp = config['trajectory_tracker_node']['ros__parameters']
    guide = module.P7SpatialGuidance(
        GuidanceConfig(**{f.name: cp['p7_'+f.name] for f in fields(GuidanceConfig)
                          if 'p7_'+f.name in cp}),
        camera_pitch_down=cp['p7_camera_pitch_down'], capture_radius=.25,
        project_capture_reference=True, minimum_center_depth=.01, fast_capture_enabled=True,
        **(dict(precision_capture_enabled=cp.get('p7_precision_capture_enabled', False))
           if frozen is None else {}))
    guide.reset(uv)
    final = FlightGuidanceCore(
        maximum_horizontal_speed=cp['guidance_maximum_horizontal_speed'],
        maximum_horizontal_acceleration=cp['guidance_maximum_horizontal_acceleration'],
        maximum_vertical_speed=cp['maximum_vertical_speed'],
        maximum_vertical_acceleration=cp['maximum_vertical_acceleration'],
        reserve_clearance=cp['reserve_clearance'],
        safety_response_delay=cp['safety_response_delay'],
        vertical_braking_acceleration=cp['vertical_braking_acceleration'])
    final.previous_velocity = tuple(uv)
    pitch = cp['p7_camera_pitch_down']
    extr = CameraExtrinsics.from_sdf_pose((.18, 0., .39), 0., pitch, 0.)
    # Read-only tracing captures P7.4 intermediate assignments without editing its guidance.
    source = inspect.getsourcelines(module.P7SpatialGuidance._fast_step)
    lines = {i+source[1]: text.strip() for i, text in enumerate(source[0])}
    stages = {}

    def trace(frame, event, arg):
        if frame.f_code.co_name != '_fast_step':
            return None
        values, text = frame.f_locals, lines.get(frame.f_lineno, '')
        if event == 'line' and 'limit' in values:
            if text.startswith('reserve_speed ='):
                stages['braking_limit'] = values['limit']
            elif text.startswith('forecast ='):
                stages['after_depth'] = values['limit']
            elif text.startswith('if partial_hold:') and 'forecast' in values:
                stages.setdefault('after_view', values['limit'])
        if event == 'return':
            for name in ('blend', 'measured', 'commanded', 'boundary_distance', 'down',
                         'distance', 'limit'):
                if name in values:
                    stages[name] = float(values[name])
            for name in ('pn', 'approach', 'relative_request'):
                if name in values:
                    stages[name] = list(values[name])
        return trace

    reason, minimum, first = 'TIME_BOUND', float('inf'), {}
    counts, total, previous = Counter(), 0, np.array(uv)
    stats = dict(surface_gap=float('inf'), minimum_height=5., maximum_xy_speed=0.,
                 maximum_xy_acceleration=0., maximum_vz=0., maximum_yaw_rate=0.,
                 endpoint_samples=0, tracker_clipped_samples=0, maximum_matching_error=0.)
    elapsed_steps = []
    with (output/'trace.jsonl').open('w') as stream:
        for i in range(800):
            t, now = i*.05, 10.+i*.05
            tp, tv = np.array(target.position), np.array(target.velocity)
            rotation = attitude(acc, yaw)
            q = Rotation.from_matrix(rotation).as_quat()
            view = evaluate_visibility(up, rotation, tp-(0., 0., .42), INTR, extr,
                                       TargetBoundingSphere(.25), VisibilityConstraints(.05, 25.))
            image = sensor(view.target_camera_flu)
            distance = float(np.linalg.norm(tp-up))
            minimum = min(minimum, distance)
            for radius in (1., .5, .25):
                if distance <= radius:
                    first.setdefault(str(radius), t)
            stats['surface_gap'] = min(stats['surface_gap'], image['surface_gap'])
            stats['minimum_height'] = min(stats['minimum_height'], float(-up[2]))
            if distance <= .25:
                reason = 'IDEAL_CAPTURE_NOT_NATIVE'
                break
            if not image['valid']:
                reason = image['reason']
                break
            sample = module.ImageSample(now, tuple(image['center']), (q[3], *q[:3]),
                                        tuple(up), tuple(tp))
            stages.clear()
            if frozen is not None:
                sys.settrace(trace)
            try:
                began = time.perf_counter()
                result = guide.step(now, .05, up, uv, tp, tv, sample, capture=True,
                                    feedback_stamp=now)
                elapsed_steps.append(time.perf_counter()-began)
            finally:
                sys.settrace(None)
            if not result.valid:
                reason = 'CONTROL_'+(result.preview_reason or 'INVALID')
                break
            command = final._velocity_command(FlightKinematicState(tuple(up), tuple(uv)),
                                              result.velocity, .05, True, True)
            velocity = np.array(command.velocity)
            if not result.capture_reference_feasible:
                counts['NO_CAPTURE_ENDPOINT'] += 1
            stats['endpoint_samples'] += int(result.capture_reference_feasible)
            stats['tracker_clipped_samples'] += int(np.linalg.norm(
                velocity-np.asarray(result.velocity)) > 1e-6)
            stats['maximum_xy_speed'] = max(
                stats['maximum_xy_speed'], float(np.linalg.norm(velocity[:2])))
            stats['maximum_xy_acceleration'] = max(
                stats['maximum_xy_acceleration'],
                float(np.linalg.norm(velocity[:2]-previous[:2]))/.05)
            stats['maximum_vz'] = max(stats['maximum_vz'], abs(velocity[2]))
            stats['maximum_yaw_rate'] = max(stats['maximum_yaw_rate'], abs(result.yaw_rate))
            stats['maximum_matching_error'] = max(
                stats['maximum_matching_error'], float(np.linalg.norm(uv[:2]-tv[:2])))
            if stages.get('after_depth', 2.) < stages.get('braking_limit', 2.)-1e-6:
                counts['DEPTH_REDUCTION'] += 1
            if stages.get('after_view', 2.) < stages.get('after_depth', 2.)-1e-6:
                counts['VIEW_REDUCTION'] += 1
            if result.braking_active:
                counts['BRAKING'] += 1
            row = dict(t=t, up=up.tolist(), uv=uv.tolist(), tp=tp.tolist(), tv=tv.tolist(),
                       target_acceleration=list(target.acceleration),
                       target_longitudinal=target.longitudinal_acceleration,
                       target_lateral=target.lateral_acceleration,
                       image=image, stages=dict(stages), p7=asdict(result),
                       final_velocity=velocity.tolist(), lag=(velocity-uv).tolist())
            stream.write(json.dumps(row)+'\n')
            previous = velocity.copy()
            old = uv.copy()
            uv += (velocity-uv)*(.05/.15)
            acc = (uv-old)/.05
            up += uv*.05
            yaw += result.yaw_rate*.05
            target = profile.advance(.05)
            total += 1
    failure = dict(t=t, distance=distance, up=up.tolist(), tp=tp.tolist(),
                   rotation=rotation.tolist(), ray=list(view.target_camera_flu), image=image)
    if reason.startswith('CONTROL_'):
        failure['controller_rejection'] = asdict(result)
    summary = dict(scope='Exact-sensor variable S + .15s velocity plant; not PX4/native evidence.',
                   motion='variable_speed_s', profile_at_y=start, outcome=reason, seconds=t,
                   minimum=minimum, first=first, samples=total, stats=stats,
                   constraint_samples=dict(counts), failure_or_stop=failure)
    if elapsed_steps:
        summary['controller_seconds_p95'] = float(np.percentile(elapsed_steps, 95))
        summary['controller_seconds_max'] = max(elapsed_steps)
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--start', type=float, default=32.)
    parser.add_argument('--frozen', type=Path)
    parser.add_argument('--config', type=Path)
    options = parser.parse_args()
    run(options.output, options.start, options.frozen, options.config)

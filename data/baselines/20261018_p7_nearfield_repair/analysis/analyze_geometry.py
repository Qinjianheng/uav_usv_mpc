"""Offline causal pose-hold, rectangular-frustum and measured-closure attribution."""
import bisect
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WS = Path('/home/qin/data/uav_usv_mpc')
RUN = WS/'data/experiments/20261018_p7_nearfield_repair_s_01'
OUT = Path(__file__).parent
raw = RUN/'offline_evidence.jsonl'
rows = [json.loads(s) for s in raw.open()]
summary = json.loads((RUN/'summary.json').read_text())
end = summary['capture_result']['stamp']
vp = next((RUN/'evaluator').glob('*_vision.csv'))
vision = list(csv.DictReader(vp.open()))
config = yaml.safe_load((RUN/'config/flight.yaml').read_text())
cal = config['rgbd_target_localizer']['ros__parameters']
translation = np.array([cal['camera_translation_'+a] for a in 'xyz'])
pitch = cal['camera_pitch_down']
radius = cal['target_radius']
model = RUN/'source_snapshot/src/uav_usv_bringup/models/x500_mono_cam/model.sdf'
cam_sdf = ET.parse(model).find(".//sensor[@name='front_tof_camera']/camera")
width = int(cam_sdf.findtext('image/width')); height = int(cam_sdf.findtext('image/height'))
hfov = float(cam_sdf.findtext('horizontal_fov'))
near = float(cam_sdf.findtext('clip/near')); far = float(cam_sdf.findtext('clip/far'))
focal = width/(2*np.tan(hfov/2)); bottom_slope = height/(2*focal)


def ray_hits(center, radius, width=640, height=480, fx=None, clip=.05):
    if fx is None:
        fx = focal
    u, v = np.meshgrid(np.arange(width)+.5, np.arange(height)+.5)
    y, z = (width/2-u)/fx, (height/2-v)/fx
    a = 1+y*y+z*z
    c = np.array(center)
    b = c[0]+y*c[1]+z*c[2]
    disc = b*b-a*(c@c-radius*radius)
    distance_x = (b-np.sqrt(np.maximum(0, disc)))/a
    hit = (disc >= 0) & (distance_x > 0)
    valid = hit & (distance_x >= clip) & (distance_x <= far)
    return int(hit.sum()), int(valid.sum())


def bottom_margin(c, radius, slope=bottom_slope):
    # Halfspace z + slope*x >= 0 is the rectangular sensor's bottom plane.
    return float((c[2]+slope*c[0])/np.sqrt(1+slope*slope)+radius)


# Independent literal 4:3, fx=320 fixtures: plane normal length=1.25.
assert abs(bottom_margin((1, 0, -1.125), .3, .75)) < 1e-12
assert bottom_margin((1, 0, -1.126), .3, .75) < 0
assert ray_hits((1, 0, -1.126), .3, fx=320, clip=0)[0] == 0
assert ray_hits((1, 0, -1.), .3, fx=320, clip=0)[0] > 0
assert ray_hits((2, 0, 0), .25, fx=320, clip=.05)[1] > 0
assert ray_hits((.03, 0, 0), .01, fx=320, clip=0)[0] > 0
assert ray_hits((.03, 0, 0), .01, fx=320, clip=.05)[1] == 0

nav = sorted([r for r in rows if r['kind'] == 'rendered_uav'], key=lambda r:r['sim'])
boat = sorted([r for r in rows if r['kind'] == 'rendered_target'], key=lambda r:r['sim'])
ns, ts = [r['sim'] for r in nav], [r['sim'] for r in boat]
geometry = []
for v in vision:
    epoch = float(v['measurement_stamp'])
    if not end-1. <= epoch <= end:
        continue
    t = float(v['rgb_raw_stamp'])
    ni, ti = bisect.bisect_right(ns, t), bisect.bisect_right(ts, t)
    if not (0 < ni < len(nav) and 0 < ti < len(boat)):
        continue
    a,b = nav[ni-1:ni+1]
    if not (0 < b['sim']-a['sim'] <= .05 and 0 <= t-ts[ti-1] <= .05
            and 0 < ts[ti]-ts[ti-1] <= .05):
        continue
    f = (t-a['sim'])/(b['sim']-a['sim'])
    p = (1-f)*np.array(a['p'])+f*np.array(b['p'])
    rot = Slerp([a['sim'],b['sim']], Rotation.from_quat([
        [q['q'][j] for j in (1,2,3,0)] for q in (a,b)]))(t)
    before, after = boat[ti-1:ti+1]
    g = (t-before['sim'])/(after['sim']-before['sim'])

    def camera(point, rotation=rot):
        d = np.array(point)-p
        flu = rotation.inv().apply([d[1],d[0],-d[2]])-translation
        return np.array([np.cos(pitch)*flu[0]-np.sin(pitch)*flu[2],flu[1],
                         np.sin(pitch)*flu[0]+np.cos(pitch)*flu[2]])

    held = camera(before['p'])
    linear = camera((1-g)*np.array(before['p'])+g*np.array(after['p']))
    observed = np.array([float(v['center_camera_'+a]) for a in 'xyz'])
    valid = v['observation_valid'].lower() == 'true'
    yaw = np.arctan2(rot.as_matrix()[1,0],rot.as_matrix()[0,0])
    level = camera(before['p'], Rotation.from_euler('z',yaw))
    optical_axis = rot.apply([np.cos(pitch),0,-np.sin(pitch)])
    ray_count, clipped_count = ray_hits(held, radius, clip=near)
    geometry.append(dict(
        stamp=epoch, before_result=epoch-end, raw_stamp=t, valid=valid,
        reason=v['rejection_reason'], red_pixels=int(v['red_pixel_count']),
        input_age=float(v['receipt_stamp'])-epoch,
        rgb_depth_skew=float(v['rgb_depth_acquisition_skew']),
        native_nav_bracket=[a['sim'],b['sim']], native_target_source_stamp=before['sim'],
        target_pose_age=t-before['sim'], target_next_stamp=after['sim'],
        camera_held=held.tolist(), camera_linear=linear.tolist(),
        observed_center=observed.tolist() if valid else None,
        hold_residual=float(np.linalg.norm(held-observed)) if valid else None,
        linear_residual=float(np.linalg.norm(linear-observed)) if valid else None,
        bottom_plane_margin=bottom_margin(held,radius),
        linear_bottom_margin=bottom_margin(linear,radius),
        level_body_counterfactual_bottom_margin=bottom_margin(level,radius),
        visible_pixel_rays_without_near=ray_count, visible_pixel_rays_with_near=clipped_count,
        effective_optical_pitch_down_deg=float(np.rad2deg(np.arctan2(
            -optical_axis[2],np.hypot(*optical_axis[:2]))))))

invalid = [g for g in geometry if not g['valid']]
valid = [g for g in geometry if g['valid']]

for g in geometry:
    c = g['camera_held']
    g['center_in_frame'] = bool(c[0] > 0 and abs(c[1]/c[0]) <= width/(2*focal)
                               and abs(c[2]/c[0]) <= height/(2*focal))
result = dict(source_run=str(RUN), source_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
              result_epoch=end, analytic_fixture_checks_passed=7,
              matched_images=len(geometry), invalid_images=len(invalid),
              center_in_frame_count=sum(g['center_in_frame'] for g in geometry),
              minimum_sphere_bottom_margin_m=min(g['bottom_plane_margin'] for g in geometry),
              minimum_rendered_red_pixels=min(g['red_pixels'] for g in geometry),
              hold_residual_p95_m=float(np.percentile([g['hold_residual'] for g in valid],95)),
              geometry=geometry,
              method='Full native attitude with <=50ms brackets; native target causal hold; no offset, extrapolation or online truth.')
(OUT/'geometry_validation.json').write_text(json.dumps(result,indent=2)+'\n')
with (OUT/'geometry_validation.csv').open('w') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(geometry[0]))
    writer.writeheader();writer.writerows(geometry)
fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True)
ax[0].plot([g['before_result'] for g in geometry],[g['bottom_plane_margin'] for g in geometry],'.-')
ax[0].axhline(0.,color='red',linestyle='--');ax[0].set_ylabel('Sphere bottom margin (m)')
ax[1].plot([g['before_result'] for g in geometry],[g['red_pixels'] for g in geometry],'.-')
ax[1].set(xlabel='Time before capture (s)',ylabel='Actual red pixels')
fig.tight_layout();fig.savefig(OUT/'geometry_validation.png',dpi=160)
print(json.dumps({k:v for k,v in result.items() if k!='geometry'},indent=2))

"""Finite analytical screening only; this module never produces flight commands."""
import math

import numpy as np
from scipy.spatial.transform import Rotation


def geometry_cases(contract, dense=False):
    """Generate 15 FOLLOW poses and a sampled continuous native-radius approach per heave."""
    follow = contract['follow']
    mount = Rotation.from_euler('y', math.radians(contract['camera']['pitch_down_degrees']))
    for heave in (-.15, 0., .15):
        settings = [(follow['distance'], follow['height'], roll, pitch, 'FOLLOW')
                    for roll, pitch in ((0., 0.), (.25, 0.), (-.25, 0.), (0., .25), (0., -.25))]
        distances = np.linspace(5., .18, 161) if dense else (3., 1., .5, .35, .25, .18)
        settings += [(d, max(.16, .8*d)+heave*min(1., (5.-d)/4.8),
                      0., 0., 'TERMINAL') for d in distances]
        for distance, height, roll, pitch, phase in settings:
            rotation = Rotation.from_euler('xyz', (roll, pitch, 0.))
            origin = np.array((-distance, 0., height))
            ball = np.array((0., 0., contract['camera']['marker_reference_z_offset']+heave))
            delta = ball-origin-rotation.apply(contract['camera']['translation_flu'])
            center = mount.inv().apply(rotation.inv().apply(delta))
            yield dict(origin=origin, ball=ball, rotation=rotation, center=center,
                       case=dict(phase=phase, heave=heave, roll=roll, pitch=pitch,
                                 origin_distance=float(np.linalg.norm(origin-(0., 0., heave)))))


def ray_box_entry(origin, directions, box):
    """Return first forward OBB intersection, with exact parallel-axis rejection."""
    rotation = np.asarray(box['rotation_flu'])
    local = (np.asarray(origin)-box['center_flu'])@rotation
    rays = np.asarray(directions)@rotation
    half = np.asarray(box['size'])/2
    parallel = np.abs(rays) < 1e-12
    safe = np.where(parallel, 1., rays)
    lower, upper = (-half-local)/safe, (half-local)/safe
    near, far = np.minimum(lower, upper), np.maximum(lower, upper)
    near = np.where(parallel, -np.inf, near)
    far = np.where(parallel, np.inf, far)
    entry, exit_ = np.maximum(0., np.max(near, axis=1)), np.min(far, axis=1)
    miss = np.any(parallel & (np.abs(local) > half), axis=1) | (exit_ < entry)
    return np.where(miss, np.inf, entry)


def boxes_overlap(first, second):
    """Use the separating-axis theorem, including cross axes, for rigid OBBs."""
    a, b = np.asarray(first['rotation_flu']), np.asarray(second['rotation_flu'])
    delta = np.asarray(second['center_flu'])-first['center_flu']
    axes = [*a.T, *b.T, *(np.cross(x, y) for x in a.T for y in b.T)]
    for axis in axes:
        if np.linalg.norm(axis) < 1e-10:
            continue
        radius = (np.abs(axis@a)@(np.asarray(first['size'])/2)
                  + np.abs(axis@b)@(np.asarray(second['size'])/2))
        if abs(axis@delta) > radius+1e-9:
            return False
    return True


def mount_conflicts(envelope):
    """Allow the specified support attachment to the body, never to rotating propellers."""
    boxes = envelope['boxes']
    camera = [b for b in boxes if b['link'] == 'front_camera_link']
    support = [b for b in boxes if b['link'] == 'p761_camera_support']
    others = [b for b in boxes if b['link'] not in ('front_camera_link', 'p761_camera_support')]
    return [(a['link'], b['link']) for a in camera+support for b in others
            if (a in camera or b['swept_rotor']) and boxes_overlap(a, b)]


def visibility(contract, envelope, pose, scale=4):
    """Ray trace the near sphere surface against physical boxes; rendering remains mandatory."""
    camera = contract['camera']
    height, width = camera['height'], camera['width']
    v, u = np.mgrid[scale/2:height:scale, scale/2:width:scale]
    focal = width/(2*math.tan(camera['horizontal_fov']/2))
    rays = np.stack((np.ones_like(u), -(u-width/2)/focal, -(v-height/2)/focal), axis=-1)
    center = pose['center']
    squared = np.sum(rays*rays, axis=-1)
    projection = rays@center
    discriminant = projection**2-squared*(center@center-camera['marker_radius']**2)
    depth = (projection-np.sqrt(np.maximum(0., discriminant)))/squared
    ideal = (discriminant >= 0) & (depth >= camera['rgb_clip'][0])
    directions = Rotation.from_euler('y', math.radians(
        camera['pitch_down_degrees'])).apply(rays[ideal])
    obstruction = np.full(len(directions), np.inf)
    for box in envelope['boxes']:
        obstruction = np.minimum(obstruction, ray_box_entry(
            camera['translation_flu'], directions, box))
    visible = int(np.sum(obstruction > depth[ideal]+1e-6))*scale**2
    flu = np.asarray(envelope['vertices_frd'])*[1., -1., -1.]
    lowest = pose['origin'][2]+np.min(pose['rotation'].apply(flu)[:, 2])
    return dict(visible_pixels_estimate=visible, ideal_pixels_estimate=int(ideal.sum())*scale**2,
                lens_surface_gap=float(np.linalg.norm(center)-camera['marker_radius']),
                body_clearance=float(lowest-contract['candidate']['sea_surface_z']))

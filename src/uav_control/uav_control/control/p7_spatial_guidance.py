"""Lightweight P7 spatial PNG and image feedback; no truth or planner ownership."""

from dataclasses import dataclass
from dataclasses import replace
import bisect
import math

import numpy as np

from ..guidance.camera_visibility import body_frd_to_ned_from_quaternion
from .p7_visual_guidance import GuidanceConfig, GuidanceOutput, _limit, image_errors


_FLU_FRD = np.diag((1., -1., -1.))


def common_epoch_feedback(navigation, tracking, now):
    """Pair KF with delivered native navigation at one epoch, without extrapolation."""
    def valid(row):
        return (row.get('valid') and all(len(row.get(k, ())) == 3 for k in ('p', 'v'))
                and all(math.isfinite(x) for k in ('p', 'v') for x in row[k]))

    if (not math.isfinite(now) or not navigation or not tracking
            or not valid(navigation[-1]) or not valid(tracking[-1])
            or not 0 < tracking[-1].get('source', 0.) <= tracking[-1]['stamp'] <= now
            or not 0 <= now-navigation[-1]['stamp'] <= .125):
        return None
    nav = sorted((r for r in navigation if valid(r) and 0 < r['stamp'] <= now
                  and now-r['stamp'] <= .125), key=lambda r: r['stamp'])
    stamps = [r['stamp'] for r in nav]
    for target in sorted(tracking, key=lambda r: r['stamp'], reverse=True):
        if (not valid(target) or not 0 < target.get('source', 0.) <= target['stamp'] <= now
                or not 0 <= now-target['source'] <= .125):
            continue
        i = bisect.bisect_left(stamps, target['stamp'])
        if i < len(nav) and stamps[i] == target['stamp']:
            a = b = nav[i]
        elif 0 < i < len(nav) and 0 < stamps[i]-stamps[i-1] <= .05:
            a, b = nav[i-1:i+1]
        else:
            continue
        fraction = ((target['stamp']-a['stamp'])/(b['stamp']-a['stamp'])
                    if b['stamp'] > a['stamp'] else 0.)
        return dict(epoch=target['stamp'], source_stamp=target['source'],
                    navigation_bracket=(a['stamp'], b['stamp']),
                    **{key: tuple(x+fraction*(y-x) for x, y in zip(a[key], b[key]))
                       for key in ('p', 'v')},
                    target_p=tuple(target['p']), target_v=tuple(target['v']))
    return None


def configure_p7_follow_geometry(node, model):
    """Wire P7 mount/altitude into the existing canonical contract on both sides."""
    from ..guidance.camera_visibility import CameraExtrinsics
    if not node.has_parameter('p7_follow_geometry_enabled'):
        node.declare_parameter('p7_follow_geometry_enabled', False)
    if not node.get_parameter('p7_follow_geometry_enabled').value:
        return
    for key, default in (('p7_camera_pitch_down', math.radians(42.)),
                         ('flight_altitude', model.config.flight_altitude)):
        if not node.has_parameter(key):
            node.declare_parameter(key, default)
    model.config = replace(model.config, flight_altitude=float(
        node.get_parameter('flight_altitude').value))
    model.config.validate()
    model.extrinsics = CameraExtrinsics.from_sdf_pose(
        (.18, 0., .39), 0., float(node.get_parameter('p7_camera_pitch_down').value), 0.)


def _unit(value):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError('INVALID_SPATIAL_VECTOR')
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError('ZERO_SPATIAL_VECTOR')
    return vector/norm


def shape_image_error(error, deadband=.03, transition=.03):
    """Signed Liu (4.27)/(4.28) shaping, in normalized pinhole tangent units."""
    if not all(math.isfinite(v) for v in (error, deadband, transition)):
        raise ValueError('INVALID_IMAGE_ERROR')
    if deadband < 0 or transition <= 0:
        raise ValueError('INVALID_IMAGE_TRANSITION')
    magnitude = abs(error)
    return error*max(0., min(1., (magnitude-deadband)/transition))


def depth_factors(camera_range, radius, vertical_half_fov=.7267008566948965):
    """Liu (4.30), using RGB-D sphere extent rather than a cropped pixel box."""
    if (not all(math.isfinite(v) for v in (camera_range, radius, vertical_half_fov))
            or not camera_range > radius > 0 or not 0 < vertical_half_fov < math.pi/2):
        raise ValueError('INVALID_DEPTH_SCALE')
    scale = radius/(math.sqrt(camera_range**2-radius**2)*math.tan(vertical_half_fov))
    cube = min(1., scale**3)
    return 1.-cube, 1.+cube


def shortest_los_quaternion(previous_los, current_los):
    """Hamilton wxyz shortest rotation, with a nonnegative scalar hemisphere."""
    a, b = map(_unit, (previous_los, current_los))
    cross = np.cross(a, b)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(a@b, -1., 1.))
    if sine < 1e-10:
        if cosine > 0:
            return np.array((1., 0., 0., 0.))
        # Deterministic shortest half-turn axis, including exactly antiparallel LOS.
        basis = np.eye(3)[int(np.argmin(np.abs(a)))]
        axis = _unit(np.cross(a, basis))
    else:
        axis = cross/sine
    half_angle = .5*math.atan2(sine, cosine)
    return np.array((math.cos(half_angle), *(axis*math.sin(half_angle))))


def scaled_los_direction(previous_los, current_los, previous_direction, gain):
    """Apply Liu (4.23)-(4.25) to the previous desired direction, without Euler angles."""
    direction = _unit(previous_direction)
    if not math.isfinite(gain) or gain < 0:
        raise ValueError('INVALID_ROTATION_GAIN')
    quaternion = shortest_los_quaternion(previous_los, current_los)
    sine = float(np.linalg.norm(quaternion[1:]))
    if sine < 1e-10:
        return direction.copy()
    axis = quaternion[1:]/sine
    angle = gain*2.*math.atan2(sine, quaternion[0])
    # Rodrigues applies the scaled quaternion without Euler-angle decomposition.
    return _unit(direction*math.cos(angle)+np.cross(axis, direction)*math.sin(angle)
                 + axis*(axis@direction)*(1.-math.cos(angle)))


def _mount(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))


def capture_entry_reference(rotation, *, camera_pitch_down=math.radians(42.),
                            camera_translation_flu=(.18, 0., .39), target_radius=.25,
                            visual_height=.42, minimum_center_depth=.02):
    """Closed-form origin entry point at the camera surface/depth corridor intersection."""
    rotation = np.asarray(rotation, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7)
            or not math.isfinite(minimum_center_depth)
            or not 0 < minimum_center_depth < target_radius+.02):
        raise ValueError('INVALID_CAMERA_ENTRY_ROTATION')
    offset = rotation@_FLU_FRD@np.asarray(camera_translation_flu)
    forward = rotation@_FLU_FRD@_mount(camera_pitch_down)[:, 0]
    # r is UAV origin -> USV reference. The marker center adds -visual_height in NED.
    center = offset+np.array((0., 0., visual_height))
    perpendicular = center-forward*float(center@forward)
    if np.linalg.norm(perpendicular) < 1e-9:
        basis = np.eye(3)[int(np.argmin(np.abs(forward)))]
        perpendicular = np.cross(forward, basis)
    direction = -_unit(perpendicular)
    if direction[2] < 0:
        direction = -direction
    axial, radius = minimum_center_depth, target_radius+.02
    return center+axial*forward+math.sqrt(radius**2-axial**2)*direction


def horizontal_capture_reference(relative_z, rotation, *, camera_pitch_down,
                                 capture_radius=.25, preferred_xy=(0., 0.),
                                 minimum_center_depth=.02):
    """Closest horizontal point in the capture/depth/surface intersection, or None."""
    rotation = np.asarray(rotation, dtype=float)
    preferred = np.asarray(preferred_xy, dtype=float)
    if (not math.isfinite(relative_z) or not math.isfinite(capture_radius)
            or capture_radius <= 0 or not math.isfinite(camera_pitch_down)
            or rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7)
            or preferred.shape != (2,) or not np.isfinite(preferred).all()
            or not math.isfinite(minimum_center_depth) or not 0 < minimum_center_depth < .27):
        raise ValueError('INVALID_HORIZONTAL_CAPTURE_GEOMETRY')
    if abs(relative_z) >= capture_radius:
        return None
    center = rotation@_FLU_FRD@np.array((.18, 0., .39))+np.array((0., 0., .42))
    forward = rotation@_FLU_FRD@_mount(camera_pitch_down)[:, 0]
    normal, origin = forward[:2], center[:2]
    limit = minimum_center_depth+forward@center-forward[2]*relative_z
    depth_norm = float(normal@normal)
    capture_xy = math.sqrt(capture_radius**2-relative_z**2)
    surface_sq = .27**2-(relative_z-center[2])**2
    candidates = [np.zeros(2)]
    if depth_norm > 1e-12:
        foot = limit*normal/depth_norm
        candidates.append(foot)
        side = np.array((-normal[1], normal[0]))/math.sqrt(depth_norm)
        remainder = capture_xy**2-float(foot@foot)
        if remainder >= -1e-10:
            for sign in (-1, 1):
                candidates.append(foot+sign*math.sqrt(max(0., remainder))*side)
    if surface_sq > 0:
        surface = math.sqrt(surface_sq)
        distance = float(np.linalg.norm(origin))
        if distance > 1e-12:
            candidates.append(origin-surface*origin/distance)
            along = (capture_xy**2-surface_sq+distance**2)/(2.*distance)
            remainder = capture_xy**2-along**2
            if remainder >= -1e-10:
                side = np.array((-origin[1], origin[0]))/distance
                for sign in (-1, 1):
                    candidates.append(along*origin/distance
                                      + sign*math.sqrt(max(0., remainder))*side)
        if depth_norm > 1e-12:
            foot = origin+(limit-normal@origin)*normal/depth_norm
            remainder = surface_sq-float((foot-origin)@(foot-origin))
            if remainder >= -1e-10:
                side = np.array((-normal[1], normal[0]))/math.sqrt(depth_norm)
                for sign in (-1, 1):
                    candidates.append(foot+sign*math.sqrt(max(0., remainder))*side)
    legal = [xy for xy in candidates if (
        xy@xy <= capture_xy**2+1e-10 and normal@xy >= limit-1e-10
        and (xy-origin)@(xy-origin) >= surface_sq-1e-10)]
    if not legal:
        return None

    def rank(xy):
        return round(float(xy@xy), 12), float((xy-preferred)@(xy-preferred))

    chosen = min(legal, key=rank)
    return tuple(float(x) for x in chosen)


def spatial_capture_reference(uav_position, target_position, rotation, *,
                              camera_pitch_down=math.radians(42.),
                              camera_translation_flu=(.18, 0., .39), target_radius=.25,
                              visual_height=.42, capture_radius=.25, sea_surface_z=0.,
                              contact_clearance=.33, relative_velocity=(0., 0., 0.),
                              target_velocity=(0., 0., 0.), response_delay=.15,
                              minimum_center_depth=.02, include_entry_floor=True):
    """
    Origin capture corridor with a camera-surface barrier, using measured states.

    The exact-image pose owns camera geometry. A bounded relative-motion preview
    reserves space; it never replaces a navigation timestamp or history sample.
    The sea floor stays unchanged, including during an infeasible heave phase.
    """
    up, tp, rv, tv = [np.asarray(v, dtype=float) for v in (
        uav_position, target_position, relative_velocity, target_velocity)]
    rotation = np.asarray(rotation, dtype=float)
    if (any(v.shape != (3,) or not np.isfinite(v).all() for v in (up, tp, rv, tv))
            or rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7)
            or not all(math.isfinite(v) for v in (
                camera_pitch_down, target_radius, visual_height, capture_radius,
                sea_surface_z, contact_clearance, response_delay, minimum_center_depth))
            or contact_clearance < .33 or target_radius <= 0 or capture_radius <= 0
            or not 0 <= response_delay <= .15 or not 0 < minimum_center_depth < .27):
        raise ValueError('INVALID_SPATIAL_CAPTURE_REFERENCE')
    camera_offset = rotation@_FLU_FRD@np.asarray(camera_translation_flu)
    forward = rotation@_FLU_FRD@_mount(camera_pitch_down)[:, 0]
    horizontal = (tp-up-camera_offset)[:2]
    delta = rv[:2]*response_delay
    fraction = (float(np.clip(-horizontal@delta/(delta@delta), 0., 1.))
                if delta@delta > 1e-12 else 0.)
    closest_xy = float(np.linalg.norm(horizontal+fraction*delta))
    # Keep the optical origin outside the surface, with 2 cm geometric margin.
    surface_gap = visual_height+camera_offset[2]+math.sqrt(max(
        0., (target_radius+.02)**2-closest_xy**2))
    entry = capture_entry_reference(
        rotation, camera_pitch_down=camera_pitch_down,
        camera_translation_flu=camera_translation_flu, target_radius=target_radius,
        visual_height=visual_height, minimum_center_depth=minimum_center_depth)
    gap = max(float(entry[2]), surface_gap) if include_entry_floor else surface_gap
    reason = 'CAMERA_CORRIDOR' if surface_gap >= entry[2] else 'CAPTURE_CORRIDOR'
    if np.linalg.norm(entry) >= capture_radius:
        reason += '_ATTITUDE_ENTRY_WAIT'
    if forward[2] > 1e-6:
        axial_gap = (visual_height+camera_offset[2]
                     + (minimum_center_depth-forward[:2]@horizontal)/forward[2])
        gap = max(gap, float(axial_gap))
    else:
        raise ValueError('CAMERA_NOT_DOWNWARD')
    # Upward-moving target reduces camera surface clearance over the response window.
    goal = float(tp[2]-gap-max(0., -tv[2])*response_delay)
    floor = sea_surface_z-contact_clearance
    if goal > floor:
        reason += '_SEA_PHASE_WAIT'
    return min(goal, floor), reason


def capture_endpoint_reference(rotation, target_z, relative, *, camera_pitch_down,
                               capture_radius=.25, minimum_center_depth=.01):
    """Joint sea-safe endpoint with a 1 cm interior capture margin, or None."""
    relative = np.asarray(relative, dtype=float)
    if relative.shape != (3,) or not np.isfinite(relative).all() or not math.isfinite(target_z):
        raise ValueError('INVALID_CAPTURE_ENDPOINT')
    interior = capture_radius-.01
    lower = max(.001, target_z+.33)
    if lower >= interior:
        return None
    points = []
    for gap in np.linspace(lower, interior-1e-8, 13):
        xy = horizontal_capture_reference(
            float(gap), rotation, camera_pitch_down=camera_pitch_down,
            capture_radius=interior, preferred_xy=relative[:2],
            minimum_center_depth=minimum_center_depth)
        if xy is not None:
            points.append(np.array((*xy, gap)))
    return min(points, key=lambda p: float((p-relative)@(p-relative))) if points else None


def visibility_reference(relative_z, rotation, direction, *, camera_pitch_down):
    """Nearest sea-phase waiting point whose center stays inside the inset frustum."""
    direction = _unit((*direction, 0.))
    rotation = np.asarray(rotation, dtype=float)
    center = rotation@np.array((.18, 0., -.39))+np.array((0., 0., .42))
    camera = _mount(camera_pitch_down).T@_FLU_FRD@rotation.T
    a = camera@direction
    b = camera@(np.array((0., 0., relative_z))-center)
    lower, upper = 0., 100.
    # 0.1 rad center inset for waiting; terminal partial-sphere geometry is separate.
    normals = [(np.array((1., 0., 0.)), .01)]
    for axis, half in ((1, .87-.1), (2, .7267008566948965-.1)):
        for sign in (-1., 1.):
            normal = np.zeros(3)
            normal[0], normal[axis] = math.tan(half), sign
            normals.append((normal, 0.))
    for normal, limit in normals:
        slope, offset = float(normal@a), float(normal@b)-limit
        if abs(slope) < 1e-10:
            if offset < 0:
                return None
        elif slope > 0:
            lower = max(lower, -offset/slope)
        else:
            upper = min(upper, -offset/slope)
    if lower > upper:
        return None
    aa, bb, cc = float(a@a), float(2*a@b), float(b@b-.27**2)
    disc = bb*bb-4*aa*cc
    if disc > 0 and np.linalg.norm(a*lower+b) < .27:
        lower = max(lower, (-bb+math.sqrt(disc))/(2*aa))
    return tuple((lower*direction)[:2]) if lower <= upper else None


def stopping_closure(distance, measured_closure, command_closure, target_acceleration,
                     response_time=.15):
    """Measured-speed stopping envelope; response bound is not a timestamp correction."""
    values = (distance, measured_closure, command_closure, target_acceleration, response_time)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('INVALID_STOPPING_INPUT')
    available = max(0., distance)
    acceleration = max(0., 2.-abs(target_acceleration))
    if acceleration <= 1e-8:
        return 0., measured_closure > 0
    closure = max(0., measured_closure)
    lag = max(0., measured_closure-command_closure)/2.
    response = response_time+lag
    limit = min(2., math.sqrt((acceleration*response)**2
                              + 2*acceleration*available)-acceleration*response)
    braking = closure*response+closure**2/(2*acceleration) > available
    return (0. if braking else limit), braking


def braking_visibility_reference(relative_z, rotation, direction, *, camera_pitch_down,
                                 maximum_acceleration):
    """
    Return the waiting boundary at the bounded horizontal braking thrust attitude.

    This is a constraint preview, not a replacement or extrapolation of image pose.
    The current full-pose constraint is also applied by the caller. Gravity and the
    existing acceleration limit set the tilt; no fitted camera-angle offset is used.
    """
    direction = _unit((*direction, 0.))
    vertical = _unit((*((maximum_acceleration*direction)[:2]), 9.81))
    forward = np.asarray(rotation)[:, 0].copy()
    forward[2] = 0.
    forward -= vertical*float(forward@vertical)
    forward = _unit(forward)
    braking_rotation = np.column_stack((forward, np.cross(vertical, forward), vertical))
    return visibility_reference(relative_z, braking_rotation, direction[:2],
                                camera_pitch_down=camera_pitch_down)


@dataclass(frozen=True)
class ImageSample:
    """Validated RGB-D ray and its original causal, same-image EKF pose."""

    stamp: float
    ray: tuple
    quaternion: tuple
    uav_position: tuple
    target_position: tuple


@dataclass(frozen=True)
class SpatialOutput(GuidanceOutput):
    velocity: tuple = (0., 0., 0.)
    vertical_velocity: float = 0.
    distance_3d: float = 0.
    closing_request: float = 0.
    desired_direction: tuple = (0., 0., 0.)
    los_direction: tuple = (0., 0., 0.)
    omega_los: tuple = (0., 0., 0.)
    new_observation: bool = False
    geometry_reason: str = ''
    goal_z: float = 0.
    view_margin: float = 0.
    depth_down: float = 1.
    entry_relative: tuple = (0., 0., 0.)
    horizontal_reference: tuple = (0., 0.)
    capture_horizontal_endpoint: tuple = (0., 0.)
    capture_reference_feasible: bool = False
    braking_active: bool = False
    closing_limit: float = 0.
    reference_rate: tuple = (0., 0.)
    matching_velocity: tuple = (0., 0., 0.)


class P7SpatialGuidance:
    """One bounded velocity controller; the existing Tracker retains final authority."""

    def __init__(self, config=GuidanceConfig(), *, camera_pitch_down=math.radians(42.),
                 capture_radius=.25, sea_surface_z=0., contact_clearance=.33,
                 project_capture_reference=False, minimum_center_depth=.02,
                 fast_capture_enabled=False):
        self.config = config
        self.camera_pitch_down = camera_pitch_down
        self.capture_radius = capture_radius
        self.sea_surface_z = sea_surface_z
        self.contact_clearance = contact_clearance
        self.project_capture_reference = project_capture_reference
        self.minimum_center_depth = minimum_center_depth
        self.fast_capture_enabled = fast_capture_enabled
        self.reset()

    def reset(self, velocity=None, yaw_rate=0.):
        self.previous = None
        if velocity is not None:
            value = np.asarray(velocity, dtype=float)
            if value.shape == (2,):
                value = np.append(value, 0.)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError('INVALID_P7_INITIAL_VELOCITY')
            self.previous = value
        self.yaw_rate = float(yaw_rate) if math.isfinite(yaw_rate) else 0.
        self.last_stamp = self.last_image_stamp = None
        self.previous_los = self.desired_direction = self.filtered_velocity = None
        self.previous_target_velocity = None
        self.previous_margin = None
        self.margin_rate = 0.
        self.approach_origin = None
        self.fast_reference = self.fast_relative_velocity = None
        self.fast_target_acceleration = np.zeros(2)
        self.previous_target_stamp = None

    def step(self, now, dt, uav_position, uav_velocity, target_position, target_velocity,
             image, capture=False, feedback_stamp=None):
        config = self.config
        try:
            up, uv, tp, tv = [np.asarray(v, dtype=float) for v in (
                uav_position, uav_velocity, target_position, target_velocity)]
            if (self.previous is None or any(v.shape != (3,) or not np.isfinite(v).all()
                                             for v in (up, uv, tp, tv))
                    or not all(math.isfinite(v) for v in (now, dt, image.stamp))
                    or not 0 < dt <= config.maximum_dt
                    or not 0 <= now-image.stamp <= config.maximum_input_age
                    or (self.last_stamp is not None and not 0 < now-self.last_stamp <= .25)
                    or (self.last_image_stamp is not None
                        and image.stamp < self.last_image_stamp)):
                raise ValueError('INVALID_SPATIAL_INPUT')
            eu, ev = image_errors(image.ray)
            camera_range = float(np.linalg.norm(image.ray))
            down, gain_up = depth_factors(camera_range, .25)
            rotation = body_frd_to_ned_from_quaternion(image.quaternion)
            entry = capture_entry_reference(
                rotation, camera_pitch_down=self.camera_pitch_down,
                minimum_center_depth=self.minimum_center_depth)
            r, relative_velocity = tp-up, tv-uv
            distance = float(np.linalg.norm(r))
            if distance < max(.01, .1*self.capture_radius):
                raise ValueError('SPATIAL_RANGE_TOO_SMALL')
            los = _unit(r)
            omega = np.cross(r, relative_velocity)/distance**2
            goal, geometry_reason = (spatial_capture_reference(
                image.uav_position, image.target_position, rotation,
                camera_pitch_down=self.camera_pitch_down, capture_radius=self.capture_radius,
                sea_surface_z=self.sea_surface_z, contact_clearance=self.contact_clearance,
                relative_velocity=relative_velocity, target_velocity=tv,
                minimum_center_depth=self.minimum_center_depth) if capture else
                (float(up[2]), 'SAFE_STANDOFF'))
        except (ValueError, TypeError, AttributeError):
            self.reset()
            return SpatialOutput()
        xy = float(np.linalg.norm(r[:2]))
        direction_xy = r[:2]/xy if xy > 1e-8 else np.zeros(2)
        # Both signed sides use the finite sphere's visible angular interval.
        # Whole-sphere containment is a separate diagnostic, never inferred here.
        ray = np.asarray(image.ray)
        margins = []
        for side, half_fov in ((1, .87), (2, .7267008566948965)):
            angle = math.atan2(-ray[side], ray[0])
            extent = math.asin(min(1., .25/math.hypot(ray[0], ray[side])))
            margins.append(min(half_fov-(angle-extent), angle+extent+half_fov))
        margin = min(margins)
        new = self.last_image_stamp is None or image.stamp > self.last_image_stamp
        if new:
            image_dt = .05 if self.last_image_stamp is None else image.stamp-self.last_image_stamp
            alpha = image_dt/(image_dt+.05*gain_up)
            if self.previous_los is None:
                self.previous_los = los.copy()
                self.desired_direction = los.copy()
            else:
                filtered = scaled_los_direction(self.previous_los, los, self.previous_los, alpha)
                self.desired_direction = scaled_los_direction(
                    self.previous_los, filtered, self.desired_direction,
                    config.navigation_constant)
                # Bounded image-directed restoration prevents accumulated PN direction drift.
                self.desired_direction = scaled_los_direction(
                    self.desired_direction, los, self.desired_direction,
                    min(1., config.velocity_gain*image_dt))
                self.previous_los = filtered
            self.margin_rate = (0. if self.previous_margin is None else
                                (margin-self.previous_margin)/image_dt)
            self.previous_margin, self.last_image_stamp = margin, image.stamp
        if capture and self.fast_capture_enabled:
            return self._fast_step(now, dt, up, uv, tp, tv, image, rotation, r,
                                   los, omega, eu, ev, margin, down, gain_up, entry, new,
                                   now if feedback_stamp is None else feedback_stamp)
        horizontal = entry[:2]
        feasible = True
        if capture and self.project_capture_reference:
            projected = horizontal_capture_reference(
                float(image.target_position[2]-goal), rotation,
                camera_pitch_down=self.camera_pitch_down, capture_radius=self.capture_radius,
                preferred_xy=r[:2], minimum_center_depth=self.minimum_center_depth)
            feasible = projected is not None
            if feasible:
                horizontal = np.asarray(projected)
            else:
                geometry_reason += '_NO_CAPTURE_INTERSECTION'
        endpoint = tuple(horizontal)
        if capture and self.project_capture_reference:
            if not feasible:
                # No capture intersection: hold a visible position at the actual
                # vertical gap, rather than pursuing the unreachable final entry.
                center = rotation@_FLU_FRD@np.array((.18, 0., .39))+np.array((0., 0., .42))
                forward = rotation@_FLU_FRD@_mount(self.camera_pitch_down)[:, 0]
                horizontal = (center+max(0., (r[2]-center[2])/forward[2])*forward)[:2]
                self.approach_origin = r.copy()
            else:
                if self.approach_origin is None:
                    self.approach_origin = r.copy()
                remaining = float(self.approach_origin[2]-entry[2])
                progress = (float(np.clip((r[2]-entry[2])/remaining, 0., 1.))
                            if remaining > 1e-6 else 0.)
                horizontal = horizontal+progress*(self.approach_origin[:2]-horizontal)
        near_velocity = (config.closing_gain*(r[:2]-horizontal) if capture else
                         config.closing_gain*(xy-4.5)*direction_xy)
        near_velocity = _limit(near_velocity, config.maximum_closing_speed)
        # Continuous alignment reduction; no abrupt reversal from a center-pixel threshold.
        near_velocity *= min(1., config.alignment_limit/max(abs(math.atan(eu)), 1e-9))
        forecast = margin+min(0., self.margin_rate)/config.velocity_gain
        view_scale = float(np.clip(forecast/.1, 0., 1.))
        if capture and near_velocity@direction_xy > 0:
            near_velocity *= view_scale
        closing = float(near_velocity@direction_xy)
        # Near-field matching is smoothly blended across the existing 1 m boundary.
        blend = float(np.clip((xy-.75)/.5, 0., 1.))
        blend = blend*blend*(3.-2.*blend)
        pn_xy = self.desired_direction[:2]
        norm_xy = float(np.linalg.norm(pn_xy))
        pn_xy = pn_xy/norm_xy if norm_xy > 1e-8 else direction_xy
        desired_xy = tv[:2]+(1.-blend)*near_velocity+blend*closing*pn_xy
        if capture and self.project_capture_reference and not feasible:
            # A scalar LOS projection discards the cross-track position hold.
            desired_xy = tv[:2]+near_velocity
        if not capture:
            goal = float(up[2])
        reference_vz = config.velocity_gain*(goal-up[2])
        image_vz = 0.
        if capture and margin < .1:
            image_down = rotation@_FLU_FRD@_mount(self.camera_pitch_down)@np.array((0., 0., -1.))
            image_vz = (.1-margin)/.1*max(.5, down)*config.velocity_gain*float(
                image_down[2])*ray[0]*shape_image_error(ev)
        png_vz = blend*closing*(self.desired_direction[2]-los[2])
        desired_vz = float(np.clip(reference_vz+image_vz+png_vz, -.6, .6))
        # Geometry is an altitude ceiling in NED. Image centering cannot push through it.
        if capture:
            desired_vz = min(desired_vz, float(np.clip((goal-up[2])/.15, -.6, .6)))
        desired = np.array((*desired_xy, desired_vz))
        if self.filtered_velocity is None:
            self.filtered_velocity = self.previous.copy()
        alpha = dt/(dt+.05*gain_up)
        self.filtered_velocity += alpha*(desired-self.filtered_velocity)
        # For closure=k*r, the integrated command has Kp=damping*k and Kd=damping.
        # damping >= 4*k gives a critically damped nominal position response;
        # the legacy 1.2/0.6 pair is underdamped and passes an unready target.
        damping = max(config.velocity_gain, 4.*config.closing_gain)
        acceleration = damping*(self.filtered_velocity[:2]-uv[:2])
        if capture and self.previous_target_velocity is not None:
            acceleration += (tv[:2]-self.previous_target_velocity[:2])/dt
        self.previous_target_velocity = tv.copy()
        acceleration = _limit(acceleration, config.maximum_acceleration)
        velocity_xy = _limit(self.previous[:2]+acceleration*dt, config.maximum_speed)
        velocity_xy = self.previous[:2]+_limit(
            velocity_xy-self.previous[:2], config.maximum_acceleration*dt)
        emitted = (velocity_xy-self.previous[:2])/dt
        # Single 3-D LOS feed-forward, with no second horizontal LOS rate.
        desired_yaw = (config.yaw_gain*max(.5, down)*math.atan(shape_image_error(eu))
                       + float(omega[2]))
        self.yaw_rate += float(np.clip(
            desired_yaw-self.yaw_rate, -config.maximum_yaw_acceleration*dt,
            config.maximum_yaw_acceleration*dt))
        self.yaw_rate = float(np.clip(self.yaw_rate, -config.maximum_yaw_rate,
                                      config.maximum_yaw_rate))
        velocity = (*velocity_xy, desired_vz)
        self.previous, self.last_stamp = np.asarray(velocity), now
        mode = ('GEOMETRY_WAIT' if not feasible else
                'CAPTURE_MATCH' if xy < 1. else 'PNG' if blend >= 1. else 'STANDOFF')
        return SpatialOutput(
            valid=True, mode=mode, velocity=tuple(velocity), acceleration=tuple(emitted),
            yaw_rate=self.yaw_rate, distance=xy, los_rate=float(omega[2]),
            closing_speed=float((uv[:2]-tv[:2])@direction_xy),
            lateral_acceleration=float(emitted@np.array((-direction_xy[1], direction_xy[0]))),
            error_u=eu, error_v=ev, vertical_velocity=desired_vz, distance_3d=distance,
            closing_request=closing, desired_direction=tuple(self.desired_direction),
            los_direction=tuple(los), omega_los=tuple(omega), new_observation=new,
            geometry_reason=geometry_reason, goal_z=goal, view_margin=margin, depth_down=down,
            entry_relative=tuple(entry), horizontal_reference=tuple(horizontal),
            capture_horizontal_endpoint=endpoint,
            capture_reference_feasible=feasible)

    def _fast_step(self, now, dt, up, uv, tp, tv, image, rotation, r,
                   los, omega, eu, ev, margin, down, gain_up, entry, new, epoch):
        if not math.isfinite(epoch) or not 0 <= now-epoch <= self.config.maximum_input_age:
            self.reset()
            return SpatialOutput()
        config = self.config
        image_relative = np.subtract(image.target_position, image.uav_position)
        endpoint = capture_endpoint_reference(
            rotation, image.target_position[2], image_relative,
            camera_pitch_down=self.camera_pitch_down, capture_radius=self.capture_radius,
            minimum_center_depth=self.minimum_center_depth)
        direction = r[:2]
        if np.linalg.norm(direction) < 1e-8:
            direction = (rotation@np.array((1., 0., 0.)))[:2]
        visible = visibility_reference(float(image_relative[2]), rotation, direction,
                                       camera_pitch_down=self.camera_pitch_down)
        # A deceleration pitches the camera up before relative speed has stopped.
        # Reserve that frustum before slewing the tracking reference toward it.
        if image_relative[2] > self.capture_radius+.1:
            braking_visible = braking_visibility_reference(
                float(image_relative[2]), rotation, direction,
                camera_pitch_down=self.camera_pitch_down,
                maximum_acceleration=config.maximum_acceleration)
            if braking_visible is None:
                visible = None
            elif visible is not None and np.linalg.norm(braking_visible) > np.linalg.norm(visible):
                visible = braking_visible
        if (visible is None and endpoint is not None
                and image_relative[2] <= self.capture_radius+.1):
            # A valid partial-sphere capture need not put its center in the image.
            visible = tuple(endpoint[:2])
        partial_hold = visible is None
        if visible is None:
            # Valid RGB-D can survive outside the center-only waiting frustum.
            # Match target velocity and retain safety, without inventing visual loss.
            visible = tuple(image_relative[:2])
        desired_reference = np.asarray(visible)
        terminal_blend = 0.
        if endpoint is not None:
            progress = float(np.clip((image_relative[2]-endpoint[2])/.5, 0., 1.))
            progress = progress*progress*(3.-2.*progress)
            terminal_blend = 1.-progress
            desired_reference = endpoint[:2]+progress*(desired_reference-endpoint[:2])
        if self.fast_reference is None:
            self.fast_reference = r[:2].copy()
        reference_delta = _limit(desired_reference-self.fast_reference,
                                 config.maximum_closing_speed*dt)
        self.fast_reference += reference_delta
        reference_rate = reference_delta/dt
        goal, reason = spatial_capture_reference(
            image.uav_position, image.target_position, rotation,
            camera_pitch_down=self.camera_pitch_down, capture_radius=self.capture_radius,
            sea_surface_z=self.sea_surface_z, contact_clearance=self.contact_clearance,
            relative_velocity=tv-uv, target_velocity=tv,
            minimum_center_depth=self.minimum_center_depth, include_entry_floor=False)
        if endpoint is not None:
            goal = min(goal, float(image.target_position[2]-endpoint[2]))
        error = r[:2]-self.fast_reference
        distance = float(np.linalg.norm(error))
        error_direction = error/distance if distance > 1e-8 else np.zeros(2)
        if (self.previous_target_stamp is not None and epoch > self.previous_target_stamp
                and epoch-self.previous_target_stamp <= .125):
            interval = epoch-self.previous_target_stamp
            acceleration = (tv[:2]-self.previous_target_velocity[:2])/interval
            self.fast_target_acceleration += interval/(interval+.15)*(
                acceleration-self.fast_target_acceleration)
        if self.previous_target_stamp is None or epoch > self.previous_target_stamp:
            self.previous_target_stamp = epoch
            self.previous_target_velocity = tv.copy()
        radial = direction/float(np.linalg.norm(direction))
        measured = float((uv[:2]-tv[:2])@radial)
        commanded = float((self.previous[:2]-tv[:2])@radial)
        boundary_distance = max(0., float(np.linalg.norm(r[:2])-np.linalg.norm(desired_reference)))
        limit, braking = stopping_closure(
            boundary_distance, measured, commanded,
            float(np.linalg.norm(self.fast_target_acceleration)),
            response_time=.15+max(0., now-image.stamp, now-epoch))
        # Depth adaptation damps extra approach, never the target's 4 m/s matching part.
        reserve_speed = math.sqrt(2*config.maximum_acceleration*.01)
        limit = min(limit, reserve_speed+down*config.maximum_closing_speed)
        forecast = margin+min(0., self.margin_rate)*(.15+max(0., now-image.stamp))
        limit *= float(np.clip(forecast/.1, 0., 1.))
        if partial_hold:
            limit, braking = 0., True
        blend = float(np.clip((distance-1.)/2., 0., 1.))
        pn = self.desired_direction[:2]
        pn = pn/max(float(np.linalg.norm(pn)), 1e-8)
        approach = (1.-blend)*error_direction+blend*pn
        norm = float(np.linalg.norm(approach))
        approach = approach/max(norm, 1e-8)
        relative_request = _limit(limit*approach-reference_rate, config.maximum_closing_speed)
        relative_request -= max(0., float(relative_request@radial)-limit)*radial
        if partial_hold:
            relative_request = np.zeros(2)
            reason += '_PARTIAL_VIEW_HOLD'
        if braking:
            relative_request -= max(0., float(relative_request@radial))*radial
        if self.fast_relative_velocity is None:
            self.fast_relative_velocity = self.previous[:2]-tv[:2]
        alpha = dt/(dt+.05*gain_up)
        self.fast_relative_velocity += alpha*(relative_request-self.fast_relative_velocity)
        if partial_hold:
            self.fast_relative_velocity[:] = 0.
        if braking:
            self.fast_relative_velocity -= max(0., float(
                self.fast_relative_velocity@radial))*radial
        self.fast_relative_velocity -= max(
            0., float(self.fast_relative_velocity@radial)-limit)*radial
        desired_xy = _limit(tv[:2]+self.fast_relative_velocity, config.maximum_speed)
        velocity_xy = self.previous[:2]+_limit(desired_xy-self.previous[:2],
                                               config.maximum_acceleration*dt)
        vertical = float(np.clip(config.velocity_gain*(goal-up[2]), -.6, .6))
        vertical = min(vertical, float(np.clip((goal-up[2])/.15, -.6, .6)))
        center = rotation@np.array((.18, 0., -.39))+np.array((0., 0., .42))
        reference_ray = _mount(self.camera_pitch_down).T@_FLU_FRD@rotation.T@(
            np.array((*self.fast_reference, image_relative[2]))-center)
        desired_error = (terminal_blend*(-reference_ray[1]/reference_ray[0])
                         if reference_ray[0] > 1e-8 else 0.)
        yaw_request = config.yaw_gain*(math.atan(shape_image_error(eu))
                                       - math.atan(shape_image_error(desired_error)))+omega[2]
        self.yaw_rate += float(np.clip(yaw_request-self.yaw_rate,
                                       -config.maximum_yaw_acceleration*dt,
                                       config.maximum_yaw_acceleration*dt))
        self.yaw_rate = float(np.clip(self.yaw_rate, -config.maximum_yaw_rate,
                                      config.maximum_yaw_rate))
        relative_emitted = np.array((*velocity_xy, vertical))-tv
        if np.linalg.norm(relative_emitted) < .2:
            self.desired_direction = los.copy()
        else:
            self.desired_direction = _unit(relative_emitted)
        emitted = (velocity_xy-self.previous[:2])/dt
        velocity = (*velocity_xy, vertical)
        self.previous, self.last_stamp = np.array(velocity), now
        return SpatialOutput(
            valid=True, mode='GEOMETRY_WAIT' if endpoint is None else 'CAPTURE_MATCH',
            velocity=velocity, acceleration=tuple(emitted), yaw_rate=self.yaw_rate,
            distance=float(np.linalg.norm(r[:2])), closing_speed=measured,
            error_u=eu, error_v=ev, vertical_velocity=vertical,
            distance_3d=float(np.linalg.norm(r)),
            closing_request=float(relative_request@radial), geometry_reason=reason,
            goal_z=goal, view_margin=margin, depth_down=down, entry_relative=tuple(entry),
            desired_direction=tuple(self.desired_direction), los_direction=tuple(los),
            omega_los=tuple(omega), new_observation=new,
            horizontal_reference=tuple(self.fast_reference),
            capture_horizontal_endpoint=tuple(endpoint[:2]) if endpoint is not None else (0., 0.),
            capture_reference_feasible=endpoint is not None, braking_active=braking,
            closing_limit=limit, reference_rate=tuple(reference_rate), matching_velocity=tuple(tv))

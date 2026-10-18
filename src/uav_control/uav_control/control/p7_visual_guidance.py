"""Small vision-only yaw servo and bounded horizontal PNG approach in local NED."""

from dataclasses import dataclass
import math

import numpy as np


def image_errors(camera_flu_ray):
    """Pinhole normalized errors: image right/down equal -FLU y/x and -FLU z/x."""
    ray = np.asarray(camera_flu_ray, dtype=float)
    if ray.shape != (3,) or not np.isfinite(ray).all() or ray[0] <= 0.:
        raise ValueError('INVALID_IMAGE_RAY')
    return float(-ray[1]/ray[0]), float(-ray[2]/ray[0])


def ibvs_yaw_rate(error_u, gain=1., deadband=.03, maximum=.6):
    """Continuous deadband, positive image right -> positive clockwise NED yaw."""
    if not all(math.isfinite(x) for x in (error_u, gain, deadband, maximum)):
        raise ValueError('INVALID_IBVS_INPUT')
    if gain <= 0 or deadband < 0 or maximum <= 0:
        raise ValueError('INVALID_IBVS_LIMIT')
    return math.copysign(min(maximum, gain*max(0., abs(error_u)-deadband)), error_u)


@dataclass(frozen=True)
class LosResult:
    valid: bool
    distance: float
    los_rate: float
    closing_speed: float
    acceleration: tuple


def png_los(relative_position, relative_velocity, navigation_constant=3., minimum_range=1.):
    """Compute horizontal LOS PN without commanding opening or near-zero ranges."""
    r, v = (np.asarray(value, dtype=float) for value in (relative_position, relative_velocity))
    if (r.shape != (2,) or v.shape != (2,) or not np.isfinite(r).all()
            or not np.isfinite(v).all() or not math.isfinite(navigation_constant)
            or not math.isfinite(minimum_range) or navigation_constant <= 0 or minimum_range <= 0):
        return LosResult(False, 0., 0., 0., (0., 0.))
    distance = float(np.linalg.norm(r))
    if distance < minimum_range:
        return LosResult(False, distance, 0., 0., (0., 0.))
    rate = float((r[0]*v[1]-r[1]*v[0])/(distance*distance))
    closure = float(-r@v/distance)
    normal = np.array((-r[1], r[0]))/distance
    acceleration = navigation_constant*max(0., closure)*rate*normal
    return LosResult(True, distance, rate, closure, tuple(acceleration))


@dataclass(frozen=True)
class GuidanceConfig:
    navigation_constant: float = 3.
    standoff_distance: float = 4.5
    maximum_closing_speed: float = .75
    closing_gain: float = .6
    velocity_gain: float = 1.2
    maximum_speed: float = 6.2
    maximum_acceleration: float = 2.
    yaw_gain: float = 1.
    yaw_deadband: float = .03
    maximum_yaw_rate: float = .6
    maximum_yaw_acceleration: float = 1.2
    alignment_limit: float = .15
    vertical_alignment_limit: float = .7
    maximum_input_age: float = .125
    maximum_dt: float = .1

    def __post_init__(self):
        if (not all(math.isfinite(v) and v > 0 for v in vars(self).values())
                or self.maximum_input_age > .125 or self.maximum_dt > .1
                or self.standoff_distance < 4.):
            raise ValueError('INVALID_P7_GUIDANCE_CONFIG')


@dataclass(frozen=True)
class GuidanceOutput:
    valid: bool = False
    mode: str = 'INVALID'
    velocity: tuple = (0., 0.)
    acceleration: tuple = (0., 0.)
    yaw_rate: float = 0.
    distance: float = 0.
    los_rate: float = 0.
    closing_speed: float = 0.
    lateral_acceleration: float = 0.
    error_u: float = 0.
    error_v: float = 0.


def _limit(vector, maximum):
    norm = float(np.linalg.norm(vector))
    return vector*min(1., maximum/max(norm, 1e-12))


def capture_reference(uav_position, target_position, target_velocity,
                      sea_surface_z=0., contact_clearance=.33, maximum_descent_speed=.6):
    """
    Coordinate descent with finite camera standoff and a sea-safe contact height.

    Positions are measured navigation/KF states, never simulation truth. The
    existing final sea barrier still shapes the returned vertical request.
    """
    vectors = [np.asarray(v, dtype=float) for v in (
        uav_position, target_position, target_velocity)]
    if (not all(v.shape == (3,) and np.isfinite(v).all() for v in vectors)
            or not all(math.isfinite(v) for v in (
                sea_surface_z, contact_clearance, maximum_descent_speed))
            or contact_clearance < .33 or not 0 < maximum_descent_speed <= .6):
        raise ValueError('INVALID_P7_CAPTURE_REFERENCE')
    up, tp, tv = vectors
    goal_z = min(tp[2]-contact_clearance, sea_surface_z-contact_clearance)
    descent_remaining = max(0., goal_z-up[2])
    # Descend before the final horizontal closure. A 0.30 m horizontal offset
    # at 0.33 m vertical separation is inside the unchanged 0.50 m sphere.
    # The 2:1 corridor leaves room for velocity-loop braking and the fixed
    # forward camera; closing over the target before descent loses its image.
    standoff = min(4.5, .3+2.*descent_remaining)
    vertical = min(maximum_descent_speed,
                   max(-maximum_descent_speed, goal_z-up[2]+min(0., tv[2])))
    return standoff, vertical


class P7VisualGuidance:
    """Integrate PN acceleration using actual dt; initialize from the last emitted command."""

    def __init__(self, config=GuidanceConfig()):
        self.config = config
        self.previous = None
        self.yaw_rate = 0.
        self.last_stamp = None
        self.previous_target_velocity = None

    def reset(self, velocity=None, yaw_rate=0.):
        self.previous = None if velocity is None else np.asarray(velocity[:2], dtype=float)
        if (self.previous is not None and (
                self.previous.shape != (2,) or not np.isfinite(self.previous).all())):
            raise ValueError('INVALID_P7_INITIAL_VELOCITY')
        self.yaw_rate = float(yaw_rate) if math.isfinite(yaw_rate) else 0.
        self.last_stamp = None
        self.previous_target_velocity = None

    def step(self, now, dt, uav_position, uav_velocity, target_position, target_velocity,
             image_stamp, error_u, error_v, approach_distance=None):
        config = self.config
        vectors = [np.asarray(v, dtype=float) for v in (
            uav_position, uav_velocity, target_position, target_velocity)]
        if (self.previous is None or not all(v.shape == (2,) and np.isfinite(v).all()
                                             for v in vectors)
                or not all(math.isfinite(v) for v in (now, dt, image_stamp, error_u, error_v))
                or not 0 < dt <= config.maximum_dt
                or not 0 <= now-image_stamp <= config.maximum_input_age
                or (approach_distance is not None and (
                    not math.isfinite(approach_distance) or approach_distance < 0.))
                or (self.last_stamp is not None and not 0 < now-self.last_stamp <= .25)):
            self.reset()
            return GuidanceOutput()
        up, uv, tp, tv = vectors
        los = png_los(tp-up, tv-uv, config.navigation_constant)
        if not los.valid and approach_distance is None:
            self.reset()
            return GuidanceOutput()
        direction = ((tp-up)/los.distance if los.distance > 1e-8 else np.zeros(2))
        normal = np.array((-direction[1], direction[0]))
        standoff = config.standoff_distance if approach_distance is None else approach_distance
        closing = max(-config.maximum_closing_speed, min(
            config.maximum_closing_speed,
            config.closing_gain*(los.distance-standoff)))
        if abs(error_u) > config.alignment_limit:
            closing = min(0., closing)
        if approach_distance is not None and error_v > config.vertical_alignment_limit:
            # A fixed forward camera cannot pitch independently. Stand back
            # before the target leaves its lower edge; descent stays sea guarded.
            closing = min(closing, max(-config.maximum_closing_speed,
                                       config.closing_gain*(config.vertical_alignment_limit
                                                            - error_v)))
        longitudinal = config.velocity_gain*(float(tv@direction)+closing
                                             - float(self.previous@direction))
        lateral = float(np.asarray(los.acceleration)@normal)
        mode = 'PNG'
        if los.distance <= standoff+.75 or los.closing_speed <= 0.:
            # Approach ends above the sea at finite standoff; keep matching a
            # moving target rather than commanding zero world-frame velocity.
            lateral = config.velocity_gain*float((tv-self.previous)@normal)
            mode = 'STANDOFF' if los.distance <= standoff+.75 else 'CLOSING'
        if approach_distance is not None and los.distance < 1.:
            mode = 'CAPTURE_MATCH'
            desired = tv+closing*direction
            acceleration = config.velocity_gain*(desired-self.previous)
        else:
            acceleration = longitudinal*direction+lateral*normal
        if approach_distance is not None and self.previous_target_velocity is not None:
            # Capture follows a maneuvering target: integrate in its moving
            # velocity frame. Only accepted, causal KF samples contribute;
            # resets discard history and all existing emission bounds follow.
            acceleration += (tv-self.previous_target_velocity)/dt
        self.previous_target_velocity = tv.copy()
        acceleration = _limit(acceleration, config.maximum_acceleration)
        velocity = _limit(self.previous+acceleration*dt, config.maximum_speed)
        # Projection onto the speed ball cannot enlarge a velocity increment
        # when its initial command is already inside that ball.
        velocity = self.previous+_limit(velocity-self.previous, config.maximum_acceleration*dt)
        emitted_acceleration = (velocity-self.previous)/dt
        desired_yaw = ibvs_yaw_rate(error_u, config.yaw_gain, config.yaw_deadband,
                                    config.maximum_yaw_rate)
        yaw_step = config.maximum_yaw_acceleration*dt
        self.yaw_rate += max(-yaw_step, min(yaw_step, desired_yaw-self.yaw_rate))
        self.yaw_rate = max(-config.maximum_yaw_rate, min(config.maximum_yaw_rate, self.yaw_rate))
        self.previous, self.last_stamp = velocity, now
        return GuidanceOutput(True, mode, tuple(velocity), tuple(emitted_acceleration),
                              self.yaw_rate, los.distance, los.los_rate, los.closing_speed,
                              float(emitted_acceleration@normal), error_u, error_v)

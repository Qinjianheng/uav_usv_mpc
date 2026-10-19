"""Near-field pointing and depth-adaptive pursuit, with independent plant geometry."""

import math

import numpy as np
import pytest

from uav_control.control.p7_visual_guidance import P7VisualGuidance, capture_reference
from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState


@pytest.mark.parametrize('sign', (-1., 1.))
def test_capture_tracks_los_rotation_even_at_zero_image_error(sign):
    controller = P7VisualGuidance()
    controller.reset((4., 0.))
    # At r=(2,0), relative lateral speed +/-0.6 gives LOS rate +/-0.3.
    out = controller.step(10., .05, (0., 0.), (4., 0.), (2., 0.),
                          (4., sign*.6), 9.98, 0., .3, approach_distance=.3)
    assert out.valid
    assert sign*out.yaw_rate > 0.
    assert abs(out.yaw_rate) <= 1.2*.05+1e-10


def test_depth_tapers_only_relative_pursuit_not_target_velocity():
    emitted = []
    for distance in (.5, 5.):
        controller = P7VisualGuidance()
        controller.reset((4., 0.))
        out = controller.step(10., .05, (0., 0.), (4., 0.), (distance, 0.), (4., 0.),
                              9.98, 0., .3, approach_distance=distance-.5)
        assert out.valid and out.velocity[0] >= 4.
        emitted.append(out.velocity[0])
    assert emitted[0] < emitted[1]


@pytest.mark.parametrize('turn_rate', (-.45, .45))
def test_turning_capture_preserves_camera_center_with_actuator_lag(turn_rate):
    dt, speed, pitch = .05, 4., math.radians(28.)
    p, v = np.array([-5., 0., -5.]), np.array([4., 0., 0.])
    controller = P7VisualGuidance()
    controller.reset(v)
    flight = FlightGuidanceCore()
    flight.previous_velocity = tuple(v)
    heading = yaw_rate = 0.
    for i in range(1200):
        t = i*dt
        angle = turn_rate*t
        q = np.array([speed/turn_rate*math.sin(angle),
                      speed/turn_rate*(1.-math.cos(angle)), 0.])
        w = np.array([speed*math.cos(angle), speed*math.sin(angle), 0.])
        if np.linalg.norm(q-p) <= .5:
            break
        # Independent fixed-camera FLU projection, including the actual mount.
        delta = q[:2]-p[:2]
        forward = math.cos(heading)*delta[0]+math.sin(heading)*delta[1]-.18
        left = math.sin(heading)*delta[0]-math.cos(heading)*delta[1]
        up = p[2]+.42-.39
        camera_forward = math.cos(pitch)*forward-math.sin(pitch)*up
        camera_up = math.sin(pitch)*forward+math.cos(pitch)*up
        assert camera_forward > 0.
        error_u, error_v = -left/camera_forward, -camera_up/camera_forward
        assert abs(error_u) < math.tan(1.74/2.)
        assert abs(error_v) < math.tan(1.74/2.)*480./640.
        offset, vz = capture_reference(p, q, w)
        out = controller.step(10.+t, dt, p[:2], v[:2], q[:2], w[:2], 10.+t-.04,
                              error_u, error_v, approach_distance=offset)
        assert out.valid
        command = flight._velocity_command(FlightKinematicState(tuple(p), tuple(v)),
                                           (*out.velocity, vz), dt, True, True)
        v += (np.array(command.velocity)-v)*dt/.25
        p += v*dt
        yaw_rate += (out.yaw_rate-yaw_rate)*dt/.15
        heading += yaw_rate*dt
    assert np.linalg.norm(q-p) <= .5

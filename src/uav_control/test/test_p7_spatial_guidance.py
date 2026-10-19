"""Independent geometry, rotation and bounded P7 spatial-control examples."""

import math

import numpy as np
import pytest

from uav_control.control.p7_spatial_guidance import (
    ImageSample, P7SpatialGuidance, depth_factors, shape_image_error,
    scaled_los_direction, spatial_capture_reference,
)
from uav_control.control.p7_visual_guidance import GuidanceConfig


def sample(stamp=10., ray=(5., 0., -.1), quaternion=(1., 0., 0., 0.)):
    return ImageSample(stamp, ray, quaternion, (0., 0., -3.), (5., 0., 0.))


def test_signed_dead_zone_and_transition_are_continuous_in_tangent_units():
    assert shape_image_error(.02) == shape_image_error(-.02) == 0.
    assert shape_image_error(.045) == pytest.approx(.0225)
    for boundary in (.03, .06):
        assert abs(shape_image_error(boundary+1e-9)-shape_image_error(boundary-1e-9)) < 1e-8
    assert shape_image_error(-.045) == -shape_image_error(.045)
    assert shape_image_error(.2) == .2


def test_depth_factors_are_bounded_and_do_not_depend_on_cropped_bounding_box():
    far = depth_factors(10., .25)
    near = depth_factors(.26, .25)
    assert 0 <= near[0] < far[0] <= 1
    assert 1 <= far[1] < near[1] <= 2
    with pytest.raises(ValueError):
        depth_factors(.24, .25)


def test_quaternion_png_rotates_previous_desired_direction_not_replaced_by_los():
    theta = .02
    current = (math.cos(theta), math.sin(theta), 0.)
    direction = scaled_los_direction((1., 0., 0.), current, (1., 0., 0.), 3.)
    assert direction == pytest.approx((math.cos(3*theta), math.sin(3*theta), 0.))
    vertical = scaled_los_direction((1., 0., 0.), (0., 0., 1.), (1., 0., 0.), 1.)
    assert vertical == pytest.approx((0., 0., 1.))
    parallel = scaled_los_direction((1., 0., 0.), (1., 0., 0.), (0., 1., 0.), 3.)
    assert parallel == pytest.approx((0., 1., 0.))
    a = scaled_los_direction((1., 0., 0.), (-1., 0., 0.), (1., 0., 0.), 1.)
    assert a == pytest.approx((-1., 0., 0.))
    with pytest.raises(ValueError):
        scaled_los_direction((0., 0., 0.), current, current, 3.)


def test_capture_reference_respects_sea_floor_phase_and_camera_surface():
    # Reference semantics remain UAV origin to target reference, not marker center.
    up = np.array((0., 0., -.6))
    target = np.array((.18, 0., -.15))
    goal, reason = spatial_capture_reference(up, target, np.eye(3))
    assert goal <= -.33
    # At the camera's horizontal offset, do not descend through the sphere.
    assert target[2]-goal >= .299-1e-6
    assert 'CAMERA' in reason
    # Over the target there is a phase-dependent origin capture region.
    goal, _ = spatial_capture_reference(up, (0., 0., -.15), np.eye(3))
    assert abs(goal+.15) < .25
    goal, reason = spatial_capture_reference(up, (0., 0., .15), np.eye(3))
    assert goal <= -.33
    assert .15-goal > .25
    assert 'SEA' in reason


@pytest.mark.parametrize('age', [.126, -.001])
def test_spatial_invalid_image_time_resets_all_histories(age):
    control = P7SpatialGuidance()
    control.reset((4., 0., 0.))
    out = control.step(10., .05, (0, 0, -3), (4, 0, 0),
                       (5, 0, 0), (4, 0, 0), sample(10.-age), capture=True)
    assert not out.valid and control.previous is None
    assert control.previous_los is None and control.desired_direction is None


def test_spatial_repeated_image_does_not_generate_or_update_los_rotation():
    control = P7SpatialGuidance()
    control.reset((4., 0., 0.))
    image = sample(10.)
    a = control.step(10.02, .05, (0, 0, -3), (4, 0, 0), (5, 0, 0), (4, 0, 0), image)
    assert a.valid
    history = control.previous_los.copy()
    b = control.step(10.07, .05, (0, 0, -3), (4, 0, 0), (5, .1, 0), (4, .1, 0), image)
    assert b.valid and not b.new_observation
    assert control.previous_los == pytest.approx(history)


def test_spatial_actual_steps_and_near_field_mode_change_keep_emission_continuous():
    c = P7SpatialGuidance(GuidanceConfig(maximum_closing_speed=2.))
    previous = np.array((4., 0., 0.))
    c.reset(previous)
    for i, distance in enumerate((1.1, 1.01, .99, 1.001, .9, .1)):
        now = 10.+i*.05
        im = sample(now, ray=(1., 0., -.1))
        out = c.step(now, .05, (0, 0, -.6), previous, (distance, 0, -.1),
                     (4., .1, 0.), im, capture=True)
        assert out.valid
        assert np.linalg.norm(np.subtract(out.velocity, previous)[:2]) <= .1+1e-9
        assert np.linalg.norm(out.velocity[:2]) <= 6.2+1e-9
        assert abs(out.vertical_velocity) <= .6
        assert abs(out.yaw_rate) <= .6
        previous = np.array(out.velocity)


@pytest.mark.parametrize('separation', [0., .0001])
def test_spatial_zero_relative_range_is_rejected_without_division(separation):
    c = P7SpatialGuidance()
    c.reset((0., 0., 0.))
    out = c.step(10., .05, (0, 0, -1), (0, 0, 0),
                 (separation, 0, -1), (0, 0, 0), sample())
    assert not out.valid


def test_measured_velocity_damping_does_not_pass_the_target_while_height_is_not_ready():
    # Independent horizontal plant response; image feedback is held valid to isolate the loop.
    c = P7SpatialGuidance(GuidanceConfig(maximum_closing_speed=2.))
    position, velocity = np.array((0., 0., -.6)), np.array((4., 0., 0.))
    c.reset(velocity)
    for i in range(400):
        now = 10.+i*.05
        target = np.array((5.+4.*i*.05, 0., 0.))
        image = ImageSample(now, (1., 0., -.1), (1., 0., 0., 0.),
                            tuple(position), tuple(target))
        out = c.step(now, .05, position, velocity, target, (4., 0., 0.), image, capture=True)
        assert out.valid
        velocity[:2] += (np.array(out.velocity[:2])-velocity[:2])*(.05/.15)
        position[:2] += velocity[:2]*.05
        assert position[0] <= target[0]+4.*.05+1e-4

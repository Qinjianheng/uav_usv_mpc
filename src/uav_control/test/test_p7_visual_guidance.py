"""Independent image/LOS examples and emitted velocity bounds for safe P7 approach."""

import math

import numpy as np
import pytest


def test_ibvs_camera_flu_ray_and_left_right_sign_deadband():
    from uav_control.control.p7_visual_guidance import image_errors, ibvs_yaw_rate
    assert image_errors((4., -1., -2.)) == pytest.approx((.25, .5))
    assert ibvs_yaw_rate(.25) > 0.
    assert ibvs_yaw_rate(-.25) < 0.
    assert ibvs_yaw_rate(.02) == 0.
    assert abs(ibvs_yaw_rate(100.)) <= .6
    error = .4
    for _ in range(200):
        previous = error
        error = math.tan(math.atan(error)-ibvs_yaw_rate(error)*.05)
        assert 0. <= error <= previous
    assert error < .04


def test_png_independent_crossing_example_and_nonclosing_singularity_guards():
    from uav_control.control.p7_visual_guidance import png_los
    result = png_los((10., 0.), (-2., 1.), 3.)
    assert result.distance == 10.
    assert result.los_rate == pytest.approx(.1)
    assert result.closing_speed == 2.
    assert result.acceleration == pytest.approx((0., .6))
    assert png_los((0., 10.), (1., -2.), 3.).acceleration == pytest.approx((.6, 0.))
    assert png_los((10., 0.), (2., 1.), 3.).acceleration == (0., 0.)
    assert not png_los((1e-8, 0.), (-2., 1.), 3.).valid


def test_guidance_initializes_from_emitted_command_and_bounds_actual_dt():
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    controller = P7VisualGuidance()
    previous = np.array((2., .4))
    controller.reset(previous, .2)
    for i in range(300):
        now = 10.+i*.05
        command = controller.step(now, .05, (0., 0.), (2., 0.),
                                  (12., 2.), (2., .1), now-.02, .01, .3)
        assert command.valid
        emitted = np.array(command.velocity)
        assert np.linalg.norm(emitted-previous) <= 2.*.05+1e-10
        assert np.linalg.norm(emitted) <= 6.2+1e-10
        assert abs(command.yaw_rate) <= .6
        previous = emitted
    invalid = controller.step(now+.3, .3, (0., 0.), (2., 0.),
                              (12., 2.), (2., 0.), now, .1, .1)
    assert not invalid.valid and invalid.yaw_rate == 0.


@pytest.mark.parametrize('age', (.126, -.01, float('nan')))
def test_guidance_rejects_old_future_or_nonfinite_image(age):
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    controller = P7VisualGuidance()
    controller.reset((2., 0.))
    out = controller.step(10., .05, (0., 0.), (2., 0.), (10., 0.), (2., 0.),
                          10.-age, .3, .1)
    assert not out.valid and out.yaw_rate == 0.


def test_standoff_holds_target_speed_and_never_commands_descent():
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    controller = P7VisualGuidance()
    controller.reset((2., 0.))
    out = controller.step(10., .05, (0., 0.), (2., 0.), (4.5, 0.), (2., 0.),
                          9.98, 0., .5)
    assert out.valid and out.mode == 'STANDOFF'
    assert out.velocity == pytest.approx((2., 0.))
    assert len(out.velocity) == 2  # Height is owned by the existing sea-safe Tracker path.


def test_crossing_target_approach_converges_to_finite_standoff_in_ideal_kinematics():
    from uav_control.control.p7_visual_guidance import P7VisualGuidance
    controller = P7VisualGuidance()
    position, velocity = np.array((0., 0.)), np.array((2., 0.))
    target, target_velocity = np.array((14., 2.)), np.array((2., .4))
    controller.reset(velocity)
    yaw, distances = 0., []
    for i in range(1000):
        now = 10.+i*.05
        relative = target-position
        image_error = math.tan(math.atan2(relative[1], relative[0])-yaw)
        out = controller.step(now, .05, position, velocity, target, target_velocity,
                              now-.02, image_error, .4)
        assert out.valid
        velocity = np.array(out.velocity)
        position += velocity*.05
        target += target_velocity*.05
        yaw += out.yaw_rate*.05
        distances.append(np.linalg.norm(target-position))
    assert min(distances) > 4.
    assert distances[-1] == pytest.approx(4.5, abs=.15)

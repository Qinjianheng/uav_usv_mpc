"""P7 relative estimates and feasible horizontal reference, independent of truth."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_control.control import p7_spatial_guidance as spatial


def navigation(stamp):
    return dict(valid=True, stamp=stamp, p=(4.*(stamp-10.), 0., -3.), v=(4., 0., 0.))


def tracking(stamp=10.02, source=10.):
    return dict(valid=True, stamp=stamp, source=source,
                p=(5.+4.*(stamp-10.), 0., 0.), v=(4., 0., 0.))


def test_common_epoch_removes_moving_platform_pairing_error_without_nav_extrapolation():
    nav = [navigation(10.), navigation(10.04), navigation(10.08)]
    target = tracking()
    pair = spatial.common_epoch_feedback(nav, [target], 10.09)
    assert pair['epoch'] == 10.02
    assert pair['navigation_bracket'] == (10., 10.04)
    relative = np.subtract(pair['target_p'], pair['p'])
    assert relative == pytest.approx((5., 0., 3.))
    mixed = np.subtract(target['p'], nav[-1]['p'])
    assert np.linalg.norm(mixed-relative) == pytest.approx(.24)


@pytest.mark.parametrize('case', ['future', 'stale_source', 'wide_bracket', 'no_right',
                                  'latest_invalid', 'stale_navigation'])
def test_common_epoch_rejects_invalid_or_unbracketed_inputs(case):
    nav, targets, now = [navigation(10.), navigation(10.04)], [tracking()], 10.09
    if case == 'future':
        targets[-1]['stamp'] = now+.01
    elif case == 'stale_source':
        targets[-1]['source'] = now-.126
    elif case == 'wide_bracket':
        nav[-1] = navigation(10.06)
    elif case == 'no_right':
        nav = nav[:1]
    elif case == 'latest_invalid':
        targets.append(dict(tracking(10.03), valid=False))
    else:
        now = 10.2
    assert spatial.common_epoch_feedback(nav, targets, now) is None


def test_common_epoch_does_not_change_under_common_position_translation():
    nav, targets = [navigation(10.), navigation(10.04)], [tracking()]
    before = spatial.common_epoch_feedback(nav, targets, 10.09)
    offset = np.array((100., -200., 30.))
    for row in nav+targets:
        row['p'] = tuple(np.asarray(row['p'])+offset)
    after = spatial.common_epoch_feedback(nav, targets, 10.09)
    assert np.subtract(after['target_p'], after['p']) == pytest.approx(
        np.subtract(before['target_p'], before['p']))


@pytest.mark.parametrize('relative_z', [.18, .20, .24, .26])
def test_32_degree_reference_rejects_incompatible_capture_depth_surface_intersection(relative_z):
    assert spatial.horizontal_capture_reference(
        relative_z, np.eye(3), camera_pitch_down=math.radians(32.)) is None


def test_horizontal_reference_is_inside_capture_and_outside_camera_sphere():
    pitch = math.radians(42.)
    xy = spatial.horizontal_capture_reference(.24, np.eye(3), camera_pitch_down=pitch,
                                              preferred_xy=(.1, .1))
    assert xy is not None
    relative = np.array((*xy, .24))
    camera_delta = relative-np.array((.18, 0., .03))
    forward = np.array((math.cos(pitch), 0., math.sin(pitch)))
    assert np.linalg.norm(relative) <= .25+1e-9
    assert np.linalg.norm(camera_delta) >= .27-1e-9
    assert forward@camera_delta >= .02-1e-9


def test_horizontal_reference_rotates_with_heading_without_fixed_axis_correction():
    pitch = math.radians(42.)
    rotation = Rotation.from_euler('z', .7).as_matrix()
    base = spatial.horizontal_capture_reference(.24, np.eye(3), camera_pitch_down=pitch,
                                                preferred_xy=(.1, .1))
    preferred = rotation@np.array((.1, .1, 0.))
    rotated = spatial.horizontal_capture_reference(
        .24, rotation, camera_pitch_down=pitch, preferred_xy=preferred[:2])
    assert rotated == pytest.approx((rotation@np.array((*base, 0.)))[:2], abs=1e-7)


def test_banked_capture_corridor_uses_sea_safe_height_instead_of_level_entry_floor():
    from uav_control.control.p7_visual_guidance import GuidanceConfig
    rotation = Rotation.from_euler('xyz', (10., 0., 0.), degrees=True)
    quaternion = rotation.as_quat()
    image = spatial.ImageSample(
        10., (1., 0., -.1), (quaternion[3], *quaternion[:3]),
        (0., 0., -.6), (.6, 0., -.15))
    control = spatial.P7SpatialGuidance(
        GuidanceConfig(maximum_closing_speed=2.), camera_pitch_down=math.radians(38.),
        project_capture_reference=True)
    control.reset((4., 0., 0.))
    result = control.step(10., .05, image.uav_position, (4., 0., 0.),
                          image.target_position, (4., 0., 0.), image, capture=True)
    assert result.valid and result.capture_reference_feasible
    relative = np.array((*result.capture_horizontal_endpoint,
                         image.target_position[2]-result.goal_z))
    assert np.linalg.norm(relative) <= .25+1e-9
    assert result.goal_z <= -.33


def test_partial_sphere_with_one_centimetre_center_depth_keeps_surface_clearance():
    from synthetic_sphere_fixture import render
    from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
    angle = math.radians(38.)
    entry = spatial.capture_entry_reference(
        np.eye(3), camera_pitch_down=angle, minimum_center_depth=.01)
    forward = np.array((math.cos(angle), 0., math.sin(angle)))
    camera_delta = entry-np.array((.18, 0., .03))
    assert np.linalg.norm(entry) < .25
    assert np.linalg.norm(camera_delta) == pytest.approx(.27)
    assert forward@camera_delta == pytest.approx(.01)
    ray = spatial._mount(angle).T@spatial._FLU_FRD@camera_delta
    mask, depth, _ = render(tuple(ray))
    fit = target_geometry_from_rgbd(mask, depth, 1.74, .05, 25., .25,
                                    require_valid_sphere=True)
    assert fit is not None and fit.center_camera == pytest.approx(ray, abs=1e-6)
    xy = spatial.horizontal_capture_reference(
        entry[2], np.eye(3), camera_pitch_down=angle, minimum_center_depth=.01)
    assert xy is not None


def test_geometry_wait_holds_a_safe_relative_position_instead_of_velocity_only_drift():
    from uav_control.control.p7_visual_guidance import GuidanceConfig
    control = spatial.P7SpatialGuidance(
        GuidanceConfig(maximum_closing_speed=2.), camera_pitch_down=math.radians(38.),
        project_capture_reference=True, minimum_center_depth=.01)
    control.reset((4., 0., 0.))
    image = spatial.ImageSample(10., (1., 0., -.1), (1., 0., 0., 0.),
                                (0., 0., -.5), (.3, 0., 0.))
    out = control.step(10., .05, image.uav_position, (4., 0., 0.),
                       image.target_position, (4., 0., 0.), image, capture=True)
    assert out.valid and out.mode == 'GEOMETRY_WAIT'
    # With an unreachable vertical gap, keep the sphere on the optical axis.
    expected = .18+(.5-.03)/math.tan(math.radians(38.))
    assert out.horizontal_reference == pytest.approx((expected, 0.))
    assert out.goal_z <= -.33
    image = spatial.ImageSample(10.05, (1., 0., -.1), (1., 0., 0., 0.),
                                (0., 0., -.5), (.25, 0., 0.))
    out = control.step(10.05, .05, image.uav_position, (4., 0., 0.),
                       image.target_position, (4., 0., 0.), image, capture=True)
    assert out.closing_request < 0.


def test_geometry_wait_with_zero_horizontal_range_uses_camera_heading_deterministically():
    from uav_control.control.p7_visual_guidance import GuidanceConfig
    control = spatial.P7SpatialGuidance(
        GuidanceConfig(maximum_closing_speed=2.), camera_pitch_down=math.radians(38.),
        project_capture_reference=True, minimum_center_depth=.01)
    control.reset((0., 0., 0.))
    for now in (10., 10.05):
        image = spatial.ImageSample(now, (1., 0., -.1), (1., 0., 0., 0.),
                                    (0., 0., -.5), (0., 0., 0.))
        out = control.step(now, .05, image.uav_position, (0., 0., 0.),
                           image.target_position, (0., 0., 0.), image, capture=True)
        assert out.valid and np.isfinite(out.velocity).all()
        assert np.isfinite(out.horizontal_reference).all()


def test_horizontal_approach_progresses_with_actual_descent_instead_of_only_final_goal():
    from uav_control.control.p7_visual_guidance import GuidanceConfig
    control = spatial.P7SpatialGuidance(
        GuidanceConfig(maximum_closing_speed=2.), camera_pitch_down=math.radians(38.),
        project_capture_reference=True, minimum_center_depth=.01)
    control.reset((4., 0., 0.))
    for i, height in enumerate((5., .5)):
        now = 10.+i*.05
        image = spatial.ImageSample(now, (5., 0., -.1), (1., 0., 0., 0.),
                                    (0., 0., -height), (5., 0., 0.))
        out = control.step(now, .05, image.uav_position, (4., 0., 0.),
                           image.target_position, (4., 0., 0.), image, capture=True)
        assert out.valid
        if i == 0:
            assert out.horizontal_reference[0] > 5.
        else:
            assert .7 < out.horizontal_reference[0] < .9


def test_unreachable_heave_phase_preserves_cross_track_feedback_before_near_field():
    from uav_control.control.p7_visual_guidance import GuidanceConfig
    control = spatial.P7SpatialGuidance(
        GuidanceConfig(maximum_closing_speed=2.), camera_pitch_down=math.radians(38.),
        project_capture_reference=True, minimum_center_depth=.01)
    control.reset((4., 0., 0.))
    image = spatial.ImageSample(10., (2., 0., -.1), (1., 0., 0., 0.),
                                (0., 0., -.5), (0., 2., .15))
    out = control.step(10., .05, image.uav_position, (4., 0., 0.),
                       image.target_position, (4., 0., 0.), image, capture=True)
    assert out.valid and out.mode == 'GEOMETRY_WAIT'
    # Reference requires north opening even though LOS/PNG points due east.
    assert out.horizontal_reference[0] > .9
    assert out.velocity[0] < 4.
    assert out.velocity[1] > 0.

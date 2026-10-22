"""P7.4 geometry, measured-speed braking and candidate-only continuous approach."""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from uav_control.control import p7_spatial_guidance as spatial
from uav_control.control.p7_visual_guidance import GuidanceConfig

ROOT = Path(__file__).resolve().parents[3]
ROTATION = np.array(((-.9626273262954615, .243455605947962, -.11865074210371358),
                     (-.2602205329206144, -.9528515318064082, .15607444563634698),
                     (-.07505934262810146, .18111688564761777, .9805930699416274)))


def image(now, up, target, rotation=np.eye(3)):
    from scipy.spatial.transform import Rotation
    angle = math.radians(28.)
    mount = np.array(((math.cos(angle), 0., math.sin(angle)), (0., 1., 0.),
                      (-math.sin(angle), 0., math.cos(angle))))
    center = rotation@np.array((.18, 0., -.39))+np.array((0., 0., .42))
    ray = mount.T@np.diag((1., -1., -1.))@rotation.T@(np.subtract(target, up)-center)
    q = Rotation.from_matrix(rotation).as_quat()
    return spatial.ImageSample(now, tuple(ray), (q[3], *q[:3]), tuple(up), tuple(target))


def controller(velocity=(4., 0., 0.)):
    control = spatial.P7SpatialGuidance(
        GuidanceConfig(maximum_closing_speed=2.), camera_pitch_down=math.radians(28.),
        capture_radius=.25, project_capture_reference=True, minimum_center_depth=.01,
        fast_capture_enabled=True)
    control.reset(velocity)
    return control


def test_candidate_025_and_fast_path_leave_default_050_legacy_untouched():
    configs = [yaml.safe_load((ROOT/'src/uav_usv_bringup/config'/n).read_text()) for n in
               ('p7_guidance.yaml', 'p7_spatial_candidate.yaml')]
    for cfg, radius, fast in zip(configs, (.5, .25), (False, True)):
        tracker = cfg['trajectory_tracker_node']['ros__parameters']
        assert tracker['p7_capture_radius'] == radius
        assert tracker.get('p7_fast_capture_enabled', False) is fast
        assert tracker['flight_altitude'] == -5.
        ev = cfg['intercept_evaluator_node']['ros__parameters']
        assert ev['evaluation_capture_radius'] == ev['planned_capture_radius'] == radius


def test_joint_reference_is_inside_capture_sphere_and_camera_clearance_at_safe_height():
    point = spatial.capture_endpoint_reference(
        ROTATION, -.15, (-.1, .1, .18), camera_pitch_down=math.radians(28.))
    assert point is not None
    center = ROTATION@np.array((.18, 0., -.39))+np.array((0., 0., .42))
    forward = ROTATION@np.array((math.cos(math.radians(28.)), 0., math.sin(math.radians(28.))))
    assert np.linalg.norm(point) <= .24+1e-9
    assert np.linalg.norm(point-center) >= .27-1e-9
    assert forward@(point-center) >= .01-1e-9
    assert -.15-point[2] <= -.33+1e-9
    assert np.linalg.norm(point[:2]) < .3


@pytest.mark.parametrize('rotation,target_z', [(np.eye(3), -.15), (ROTATION, 0.)])
def test_empty_intersection_never_forces_capture_or_lowers_sea_reference(rotation, target_z):
    assert spatial.capture_endpoint_reference(
        rotation, target_z, (1., 0., .5), camera_pitch_down=math.radians(28.)) is None


def test_visible_reference_allows_closure_before_finishing_five_metre_descent():
    reference = spatial.visibility_reference(5., np.eye(3), (1., 0.),
                                             camera_pitch_down=math.radians(28.))
    assert reference is not None and .3 < reference[0] < 4.
    control = controller()
    sample = image(10., (0., 0., -5.), (5., 0., 0.))
    out = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True)
    assert out.valid and 4. < out.velocity[0] <= 4.1+1e-9
    assert 0 < out.vertical_velocity <= .6
    assert out.goal_z <= -.33


def test_measured_overspeed_and_command_tracking_lag_trigger_early_braking():
    slow, slow_braking = spatial.stopping_closure(.8, .2, .2, 0.)
    fast, fast_braking = spatial.stopping_closure(.8, 2., .5, 0.)
    assert slow > 0 and not slow_braking
    assert fast == 0. and fast_braking
    turning, _ = spatial.stopping_closure(.8, .2, .2, 1.5)
    assert turning < slow


def test_braking_attitude_reserves_view_distance_before_reference_slew():
    from scipy.spatial.transform import Rotation
    pitch = math.radians(28.)
    rotation = Rotation.from_euler('y', -.15).as_matrix()
    current = spatial.visibility_reference(3., rotation, (1., 0.),
                                           camera_pitch_down=pitch)
    braking = spatial.braking_visibility_reference(
        3., rotation, (1., 0.), camera_pitch_down=pitch, maximum_acceleration=2.)
    assert braking is not None and braking[0] > current[0]
    control = controller((5.5, 0., 0.))
    up, target = (0., 0., -3.), (braking[0]+.3, 0., 0.)
    sample = image(10., up, target, rotation)
    out = control.step(10., .05, up, (5.5, 0., 0.), target, (4., 0., 0.),
                       sample, capture=True)
    assert out.valid and out.braking_active
    assert out.closing_request <= 0.
    assert out.velocity[0] <= 5.5


def test_view_stop_caps_reference_rate_feedforward_too():
    control = controller()
    control.previous_margin = 2.
    control.last_image_stamp = 9.95
    sample = image(10., (0., 0., -3.), (2., 0., 0.))
    out = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True)
    assert out.valid and out.closing_limit == 0.
    assert out.closing_request <= 0.


def test_far_field_reference_does_not_cancel_image_yaw_restoration():
    control = controller()
    sample = image(10., (0., 0., -3.), (3., 1., 0.))
    out = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True)
    assert out.valid and out.yaw_rate == pytest.approx(.06)


def test_declared_delayed_velocity_plant_stops_before_the_reference_boundary():
    distance, actual, command = 1.6, 2., 2.
    for _ in range(160):
        desired, _ = spatial.stopping_closure(distance, actual, command, 0.)
        command += float(np.clip(desired-command, -.1, .1))
        actual += (command-actual)*(.05/.15)
        distance -= actual*.05
        assert distance >= -.01


def test_feasibility_changes_slew_references_and_emitted_commands():
    control = controller()
    previous_velocity, previous_reference = np.array((4., 0.)), None
    for i, height in enumerate((5., .34, .8, .34)):
        now = 10.+i*.05
        heading = ROTATION[:2, 0]/np.linalg.norm(ROTATION[:2, 0])
        sample = image(now, (0., 0., -height), (*heading, -.15), ROTATION)
        out = control.step(now, .05, sample.uav_position, (4., 0., 0.),
                           sample.target_position, (4., 0., 0.), sample, capture=True)
        assert out.valid
        assert np.linalg.norm(np.subtract(out.velocity[:2], previous_velocity)) <= .1+1e-9
        reference = np.array(out.horizontal_reference)
        if previous_reference is not None:
            assert np.linalg.norm(reference-previous_reference) <= .1+1e-9
        previous_reference, previous_velocity = reference, np.array(out.velocity[:2])


def test_low_relative_speed_keeps_four_metre_world_speed_matching_and_reference_yaw():
    point = spatial.capture_endpoint_reference(
        ROTATION, -.15, (-.1, .1, .18), camera_pitch_down=math.radians(28.))
    control = controller()
    up = (0., 0., -.15-point[2])
    target = (point[0], point[1], -.15)
    sample = image(10., up, target, ROTATION)
    out = control.step(10., .05, up, (4., 0., 0.), target, (4., 0., 0.), sample, capture=True)
    assert out.valid and np.isfinite(out.velocity).all()
    assert np.linalg.norm(out.velocity[:2]) > 3.8
    assert abs(out.yaw_rate) < .01


def test_valid_partial_rgbd_without_capture_intersection_holds_instead_of_resetting():
    from synthetic_sphere_fixture import render
    from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
    control = controller()
    up, target = (0., 0., -.33), (-.10812968880010489, .11189555852160056, -.10)
    sample = image(10., up, target, ROTATION)
    mask, depth, _ = render(sample.ray)
    mask &= (depth >= .05) & (depth <= 25.)
    fit = target_geometry_from_rgbd(mask, depth, 1.74, .05, 25., .25,
                                    require_valid_sphere=True)
    assert fit is not None
    out = control.step(10., .05, up, (4., 0., 0.), target, (4., 0., 0.), sample, capture=True)
    assert out.valid and out.mode == 'GEOMETRY_WAIT'
    assert out.braking_active and out.closing_limit == 0.
    assert out.velocity[:2] == pytest.approx((4., 0.))
    assert out.goal_z <= -.33


@pytest.mark.parametrize('stale', ['image', 'feedback'])
def test_fast_path_retains_image_and_feedback_freshness_gate(stale):
    control = controller()
    sample = image(10., (0., 0., -5.), (5., 0., 0.))
    now = 10.126 if stale == 'image' else 10.
    out = control.step(now, .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True,
                       feedback_stamp=9.874 if stale == 'feedback' else now)
    assert not out.valid

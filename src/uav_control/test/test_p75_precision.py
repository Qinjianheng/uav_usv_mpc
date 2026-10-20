"""P7.5 exact RGB-D, vector braking and candidate-only direction/terminal guards."""
import math

import numpy as np
import pytest

from uav_control.guidance.rgbd_visibility import evaluate_rgbd_sphere
from uav_control.control import p7_spatial_guidance as spatial
from test_p74_fast_capture import controller, image, ROTATION


def precision(velocity=(4., 0., 0.)):
    result = controller(velocity)
    result.precision_capture_enabled = True
    return result


def test_p74_near_clip_is_not_reported_as_observable():
    result = evaluate_rgbd_sphere((.2230642963568542, .006759608909881817,
                                   -.13304059718474343))
    assert not result.valid and result.projected_pixels > 0
    assert result.rgb_pixels == result.depth_pixels == 0
    assert result.reason == 'RGB_NEAR_CLIP'


def test_depth_range_and_rgb_visibility_are_separate():
    result = evaluate_rgbd_sphere((40., 0., 0.))
    assert result.rgb_pixels > 0 and result.depth_pixels == 0
    assert not result.valid and result.reason == 'DEPTH_UNUSABLE'


def test_partial_interior_sphere_fits_without_a_visible_center():
    result = evaluate_rgbd_sphere((.011, .1769748884079802, -.20493874419998934))
    assert result.valid and result.depth_ratio >= .5
    assert result.depth_pixels >= 12
    assert result.center == pytest.approx((.011, .1769748884079802, -.20493874419998934))


def test_historical_s_lower_edge_exit_is_rgb_loss():
    result = evaluate_rgbd_sphere((.116, -.115, -.445))
    assert not result.valid and result.reason == 'RGB_NOT_VISIBLE'


@pytest.mark.parametrize('lateral', [-1., 1.])
def test_acceleration_disk_reserves_lateral_demand_without_subtracting_its_norm(lateral):
    budget = spatial.projected_braking_acceleration((0., lateral), (1., 0.), 2., 0.)
    assert budget == pytest.approx(math.sqrt(3.))
    assert math.hypot(budget, lateral) <= 2.+1e-9


def test_target_acceleration_and_deceleration_have_opposite_braking_effects():
    away = spatial.projected_braking_acceleration((.5, 1.), (1., 0.), 2., .1)
    toward = spatial.projected_braking_acceleration((-.5, 1.), (1., 0.), 2., .1)
    assert away-toward == pytest.approx(1.)
    assert toward >= 0.
    assert spatial.projected_braking_acceleration((0., 2.1), (1., 0.), 2., 0.) == 0.


def test_png_blend_tracks_actual_range_not_internal_reference_error():
    assert spatial.precision_png_blend(5.) == 1.
    assert spatial.precision_png_blend(.4) == 0.
    assert 0. < spatial.precision_png_blend(1.5) < 1.


def test_safe_far_field_closes_even_when_internal_reference_has_small_error():
    control = precision()
    sample = image(10., (0., 0., -5.), (5., 0., 0.))
    out = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True)
    assert out.valid and out.png_blend == 1.
    assert out.velocity[0] > 4.
    assert out.velocity[2] > 0.
    assert out.allowed_closure > .2
    assert out.depth_limit > .2


def test_no_terminal_at_mean_sea_level_never_forces_closure_or_lower_altitude():
    control = precision()
    sample = image(10., (0., 0., -.33), (.5, 0., 0.))
    out = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True)
    assert out.valid and not out.capture_reference_feasible
    assert out.mode == 'GEOMETRY_WAIT' and out.goal_z <= -.33
    assert out.geometry_limit <= out.kinematic_limit


def test_precision_candidate_keeps_epoch_rejection():
    control = precision()
    sample = image(10., (0., 0., -5.), (5., 0., 0.))
    out = control.step(10.126, .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True)
    assert not out.valid


def test_precision_flag_is_candidate_only_and_keeps_all_limits():
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[3]
    for name, enabled in (('p7_guidance.yaml', False), ('p7_spatial_candidate.yaml', True)):
        config = yaml.safe_load((root/'src/uav_usv_bringup/config'/name).read_text())
        tracker = config['trajectory_tracker_node']['ros__parameters']
        assert tracker.get('p7_precision_capture_enabled', False) is enabled
        assert tracker['p7_maximum_speed'] == 6.2
        assert tracker['p7_maximum_acceleration'] == 2.
        assert tracker['p7_maximum_yaw_rate'] == .6
        assert tracker['p7_maximum_input_age'] == .125


def test_short_corridor_catches_pixel_loss_before_closure():
    result = spatial.precision_corridor(
        (.3, 0., .25), (4.6, 0., 0.), (4., 0., 0.), np.eye(3),
        (4.6, 0., .6), (0., 0., 0.), camera_pitch_down=math.radians(28.))
    assert not result['valid']
    assert result['reason'] in ('CAMERA_CLEARANCE', 'RGB_NOT_VISIBLE', 'RGB_NEAR_CLIP')


def test_precision_endpoint_rejects_unobservable_geometry():
    point = spatial.capture_endpoint_reference(
        ROTATION, -.15, (-.1, .1, .18), camera_pitch_down=math.radians(28.))
    assert point is not None  # Existing finite geometric witness stays available.
    center = ROTATION@np.array((.18, 0., -.39))+np.array((0., 0., .42))
    mount = spatial._mount(math.radians(28.))
    ray = mount.T@np.diag((1., -1., -1.))@ROTATION.T@(point-center)
    observation = evaluate_rgbd_sphere(ray)
    assert observation.valid


def test_rising_target_waiting_reference_reserves_future_camera_clearance():
    rising = spatial.precision_waiting_reference(
        .28, 0., -.22, 0., 0., np.eye(3), (1., 0.), camera_pitch_down=math.radians(28.))
    falling = spatial.precision_waiting_reference(
        .28, 0., .22, 0., 0., np.eye(3), (1., 0.), camera_pitch_down=math.radians(28.))
    assert rising is not None and falling is not None
    assert rising[0] > falling[0]


def test_waiting_for_rising_target_can_open_before_the_camera_boundary():
    control = precision()
    sample = image(10., (0., 0., -.38), (.41, 0., -.10))
    out = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., -.22), sample, capture=True)
    assert out.valid
    assert out.closing_request < 0. and out.velocity[0] < 4.
    assert out.goal_z <= -.33


def test_exact_pixels_match_independent_renderer_and_actual_localizer():
    from synthetic_sphere_fixture import render
    from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
    center = (.011, .1769748884079802, -.20493874419998934)
    projected, depth, _ = render(center)
    rgb = projected & (depth >= .05) & (depth <= 60.)
    fit = target_geometry_from_rgbd(rgb, depth, 1.74, .05, 25., .25,
                                    require_valid_sphere=True)
    result = evaluate_rgbd_sphere(center)
    assert result.rgb_pixels == int(rgb.sum())
    assert result.depth_pixels == fit.valid_depth_count
    assert result.center == pytest.approx(fit.center_camera)


def test_capture_ball_range_certificate_covers_independent_spherical_points():
    from pathlib import Path
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
    from p75_geometry_audit import distance_upper_bound
    for rotation in (np.eye(3), ROTATION):
        center = rotation@np.array((.18, 0., -.39))+np.array((0., 0., .42))
        normal = rotation@np.array((math.cos(math.radians(28.)), 0.,
                                    math.sin(math.radians(28.))))
        bound = distance_upper_bound(rotation, math.radians(28.))
        checked = 0
        for elevation in np.linspace(-math.pi/2, math.pi/2, 13):
            for azimuth in np.linspace(-math.pi, math.pi, 25):
                point = .25*np.array((math.cos(elevation)*math.cos(azimuth),
                                      math.cos(elevation)*math.sin(azimuth),
                                      math.sin(elevation)))
                if normal@(point-center) >= 0:
                    assert np.linalg.norm(point-center) <= bound+1e-9
                    checked += 1
        assert checked
    assert distance_upper_bound(np.eye(3), math.radians(28.)) < .27


def test_precision_repeated_image_keeps_png_update_causal():
    control = precision()
    sample = image(10., (0., 0., -5.), (5., 0., 0.))
    first = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                         sample.target_position, (4., 0., 0.), sample, capture=True)
    repeated = control.step(10.05, .05, sample.uav_position, (4., 0., 0.),
                            sample.target_position, (4., 0., 0.), sample, capture=True)
    assert first.new_observation and not repeated.new_observation
    older = image(9.99, sample.uav_position, sample.target_position)
    assert not control.step(10.1, .05, older.uav_position, (4., 0., 0.),
                            older.target_position, (4., 0., 0.), older, capture=True).valid


def test_precision_png_direction_is_independent_of_descent_speed_allocation():
    control = precision()
    sample = image(10., (0., 0., -5.), (5., 0., 0.))
    out = control.step(10., .05, sample.uav_position, (4., 0., 0.),
                       sample.target_position, (4., 0., 0.), sample, capture=True)
    assert out.valid
    # A slew-limited horizontal speed and independent vz must not rotate the
    # stored 3D PNG direction toward vertical before the next image update.
    assert out.desired_direction == pytest.approx(out.png_direction)


def test_precision_stages_above_the_camera_sphere_before_horizontal_alignment():
    pitch = math.radians(65.)
    control = spatial.P7SpatialGuidance(
        camera_pitch_down=pitch, capture_radius=.25, project_capture_reference=True,
        minimum_center_depth=.01, fast_capture_enabled=True, precision_capture_enabled=True)
    control.reset((4., 0., 0.))
    up, tp = np.array((0., 0., -.55)), np.array((.5, 0., -.15))
    relative = tp-up
    center = np.array((.18, 0., .03))
    ray = spatial._mount(pitch).T@np.diag((1., -1., -1.))@(relative-center)
    sample = spatial.ImageSample(10., tuple(ray), (1., 0., 0., 0.), tuple(up), tuple(tp))
    out = control.step(10., .05, up, (4., 0., 0.), tp, (4., 0., 0.), sample, capture=True)
    assert out.valid
    # At x=.18, a direct crossing at low altitude intersects the .25m marker.
    # Staging above the sphere reserves the same .07m waiting clearance through
    # every horizontal position and any bounded 2m/s² thrust tilt.
    tilt = math.atan2(2., 9.81)
    required = .42-.39*math.cos(tilt)+.18*math.sin(tilt)+.25+.07
    assert out.goal_z <= tp[2]-required+1e-9


def test_65_degree_layout_is_isolated_and_coherent_without_other_config_changes(tmp_path):
    from pathlib import Path
    import yaml
    from p7_follow_sitl import prepare, validate_p7_geometry
    root = Path(__file__).resolve().parents[3]
    folder = root/'src/uav_usv_bringup/config'
    original = yaml.safe_load((folder/'p7_spatial_candidate.yaml').read_text())
    path = folder/'p75_spatial_65_candidate.yaml'
    config = yaml.safe_load(path.read_text())
    validate_p7_geometry(config)
    prepare(tmp_path/'candidate65', 'f2', 'variable_speed_s', capture=True, config_file=path)
    expected = dict(trajectory_tracker_node='p7_camera_pitch_down',
                    p4_follow_planner_node='p7_camera_pitch_down',
                    rgbd_target_localizer='camera_pitch_down',
                    target_bearing_node='camera_pitch_down', front_tof_monitor='camera_pitch_down')
    for node, key in expected.items():
        assert config[node]['ros__parameters'][key] == pytest.approx(math.radians(65.))
        config[node]['ros__parameters'][key] = original[node]['ros__parameters'][key]
    assert config == original


def test_exact_partial_sphere_staging_does_not_require_center_inside_frustum():
    pitch = math.radians(65.)
    control = spatial.P7SpatialGuidance(
        camera_pitch_down=pitch, capture_radius=.25, project_capture_reference=True,
        minimum_center_depth=.01, fast_capture_enabled=True, precision_capture_enabled=True)
    control.reset((4., 0., 0.))
    control.fast_reference = np.zeros(2)
    up, tp = np.array((0., 0., -.6)), np.array((.1, 0., -.15))
    relative = tp-up
    ray = spatial._mount(pitch).T@np.diag((1., -1., -1.))@(
        relative-np.array((.18, 0., .03)))
    sample = spatial.ImageSample(10., tuple(ray), (1., 0., 0., 0.), tuple(up), tuple(tp))
    out = control.step(10., .05, up, (4., 0., 0.), tp, (4., 0., 0.), sample, capture=True)
    assert out.valid
    assert out.horizontal_reference == pytest.approx((0., 0.))


def test_capture_window_checks_target_matching_motion_instead_of_frozen_world_height():
    result = spatial.precision_endpoint_preview(
        (0., 0., .24), (4., 0., 0.), (.2, .4, .37), np.eye(3), -.15,
        camera_pitch_down=math.radians(65.), capture_interior=.24)
    assert result['valid']
    assert result['minimum_gap'] >= .02


def test_aligned_terminal_descent_uses_response_braking_envelope():
    pitch = math.radians(65.)
    control = spatial.P7SpatialGuidance(
        camera_pitch_down=pitch, capture_radius=.25, project_capture_reference=True,
        minimum_center_depth=.01, fast_capture_enabled=True, precision_capture_enabled=True)
    control.reset((4., 0., .4))
    up, tp = np.array((0., 0., -.55)), np.array((0., 0., -.15))
    ray = spatial._mount(pitch).T@np.diag((1., -1., -1.))@(
        tp-up-np.array((.18, 0., .03)))
    sample = spatial.ImageSample(10., tuple(ray), (1., 0., 0., 0.), tuple(up), tuple(tp))
    out = control.step(10., .05, up, (4., 0., .4), tp, (4., 0., 0.), sample, capture=True)
    assert out.valid
    assert .3 < out.velocity[2] <= .6
    distance = out.goal_z-up[2]
    assert .15*out.velocity[2]+out.velocity[2]**2/(2*2.) <= distance+1e-9


def test_terminal_corridor_can_brake_descent_within_actual_slew_limit():
    rotation = np.array(((.9675294365, .2527343151, .0034864716),
                         (-.2527566111, .9674820274, .0096240447),
                         (-.0009407722, -.0101927753, .9999476098)))
    selected = spatial.precision_allocate_velocity(
        (-.00094, -.00226, .33104), (1.70516, 1.13880, .34147),
        (1.70513, 1.13496, -.0725), rotation, (1.70174, 1.12936, .33044),
        (0., 0.), .30, (-.05194, -.09358, .28182), .05,
        camera_pitch_down=math.radians(65.), uav_z=-.47370, maximum_acceleration=2.)
    assert selected is not None
    command, factor, preview = selected
    assert preview['valid'] and preview['minimum_gap'] >= .02
    assert command[2] < .30
    assert abs(command[2]-.33044) <= 2*.05+1e-9


def test_explicit_wide_camera_corridor_uses_calibrated_intrinsics():
    from uav_control.guidance.camera_visibility import CameraIntrinsics
    intr = CameraIntrinsics.from_horizontal_fov(640, 480, 2.3)
    control = spatial.P7SpatialGuidance(camera_pitch_down=math.radians(45.),
                                       camera_horizontal_fov=2.3)
    assert control.intrinsics == intr
    assert spatial.precision_corridor(
        (8., 0., 0.), (0., 0., 0.), (0., 0., 0.), np.eye(3),
        (0., 0., 0.), (0., 0., 0.), camera_pitch_down=math.radians(45.),
        intrinsics=intr)['valid']

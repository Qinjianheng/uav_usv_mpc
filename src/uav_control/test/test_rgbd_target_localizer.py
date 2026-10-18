import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from px4_msgs.msg import TimesyncStatus, VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import Image

from uav_control.perception import rgbd_target_localizer

from uav_control.perception.rgbd_target_localizer import (
    GazeboImageClockMapper,
    Px4RosClockMapper,
    RgbDepthPairBuffer,
    TimestampedVectorHistory,
    body_frd_to_ned_rotation,
    camera_target_to_local_ned,
    due_data_timeout,
    pose_history_rejection_reason,
    quaternion_slerp,
    synchronized_measurement_time,
    target_geometry_from_rgbd,
    target_vector_from_rgbd,
    local_ned_target_to_camera_flu,
    validated_sensor_stamp,
)


def test_gazebo_clock_maps_sim_time_from_server_side_dual_clock_anchors():
    mapper = GazeboImageClockMapper()
    assert mapper.add_anchor(10.0, 1000.0, 1000.04, 50.0)
    assert mapper.add_anchor(10.1, 1000.2, 1000.25, 50.2)

    mapped = mapper.to_ros_time(10.05, now_ros_time=1000.25)

    assert mapped == pytest.approx(1000.1)
    assert mapper.last_status == 'MAPPED_INTERPOLATED'
    assert mapper.mapping_mode == 'GAZEBO_CLOCK_SYSTEM_INTERPOLATION'


def test_gazebo_mapping_does_not_follow_image_bridge_delay_variation():
    mapper = GazeboImageClockMapper()
    mapper.add_anchor(20.0, 2000.0, 2000.01, 60.0)
    mapper.add_anchor(20.2, 2000.2, 2000.22, 60.2)

    early_receipt_mapping = mapper.to_ros_time(20.1, 2000.22)
    late_receipt_mapping = mapper.to_ros_time(20.1, 2000.60)

    assert early_receipt_mapping == pytest.approx(2000.1)
    assert late_receipt_mapping == pytest.approx(2000.1)


def test_gazebo_clock_system_jump_invalidates_old_mapping():
    mapper = GazeboImageClockMapper(maximum_system_clock_step=0.1)
    mapper.add_anchor(5.0, 100.0, 100.01, 10.0)
    mapper.add_anchor(5.1, 100.1, 100.11, 10.1)
    assert mapper.to_ros_time(5.05, 100.11) == pytest.approx(100.05)

    accepted = mapper.add_anchor(5.2, 101.2, 101.21, 10.2)

    assert not accepted
    assert mapper.reset_count == 1
    assert mapper.to_ros_time(5.15, 101.21) is None
    assert mapper.last_status == 'SYSTEM_CLOCK_RESET'


def test_gazebo_clock_pause_resume_and_rewind_are_bounded():
    mapper = GazeboImageClockMapper()
    mapper.add_anchor(1.0, 50.0, 50.01, 1.0)
    mapper.add_anchor(1.1, 50.1, 50.11, 1.1)

    assert not mapper.add_anchor(1.1, 50.3, 50.31, 1.3)
    assert mapper.last_status == 'SIM_TIME_PAUSED'
    assert mapper.add_anchor(1.2, 50.4, 50.41, 1.4)
    assert mapper.to_ros_time(1.15, 50.41) == pytest.approx(50.25)

    assert not mapper.add_anchor(0.1, 50.5, 50.51, 1.5)
    assert mapper.reset_count == 1
    assert mapper.last_status == 'SIM_TIME_RESET'
    assert mapper.to_ros_time(1.15, 50.51) is None


def test_gazebo_image_time_without_trusted_clock_pair_is_rejected():
    mapper = GazeboImageClockMapper()

    assert mapper.to_ros_time(12.0, 100.0) is None
    assert mapper.last_status == 'CLOCK_REFERENCE_UNAVAILABLE'


def test_gazebo_clock_rejects_stale_and_future_mapped_images_separately():
    mapper = GazeboImageClockMapper(maximum_reference_age=0.5)
    mapper.add_anchor(10.0, 100.0, 100.01, 1.0)
    mapper.add_anchor(10.2, 100.2, 100.21, 1.2)

    assert mapper.to_ros_time(10.1, 101.0) is None
    assert mapper.last_status == 'CLOCK_REFERENCE_STALE'
    assert mapper.to_ros_time(10.3, 100.21) is None
    assert mapper.last_status == 'IMAGE_AFTER_CLOCK_REFERENCE'


def test_missing_image_timeout_is_reported_at_a_bounded_rate():
    assert due_data_timeout(10.6, 10.0, -math.inf, 0.5)
    assert not due_data_timeout(10.7, 10.0, 10.6, 0.5)
    assert due_data_timeout(11.2, 10.0, 10.6, 0.5)


@pytest.mark.parametrize('center', [(.45, -.08, .12), (.7, .25, .2)])
def test_rgbd_partial_near_sphere_recovers_geometric_center(center):
    """Use rendered ray/sphere intersections, including a missing mask side."""
    height, width = 120, 160
    fov = 1.74
    fx, fy, cx, cy = rgbd_target_localizer.camera_intrinsics(width, height, fov)
    rows, columns = np.indices((height, width))
    rays = np.stack((np.ones_like(rows), -(columns - cx) / fx,
                     -(rows - cy) / fy), axis=-1)
    center = np.array(center)
    radius = .25
    a = np.sum(rays * rays, axis=-1)
    b = np.sum(rays * center, axis=-1)
    discriminant = b * b - a * (np.dot(center, center) - radius ** 2)
    depth = (b - np.sqrt(np.maximum(discriminant, 0))) / a
    mask = (discriminant > 0) & (depth >= .05) & (columns > width // 3)
    depth[~mask] = np.inf

    estimated = target_vector_from_rgbd(mask, depth, fov, .05, 25., radius)

    assert estimated == pytest.approx(center, abs=1e-4)


@pytest.mark.parametrize('bad_kind', ('tiny', 'invalid', 'background'))
def test_online_sphere_geometry_rejects_unreliable_surfaces(bad_kind):
    mask, depth = _render_forward_depth_sphere((.45, -.08, .12), .25)
    if bad_kind == 'tiny':
        visible = np.argwhere(mask)
        mask[:] = False
        for row, column in visible[:5]:
            mask[row, column] = True
    elif bad_kind == 'invalid':
        depth[mask] = np.nan
    else:
        rows, columns = np.nonzero(mask)
        depth[rows[::5], columns[::5]] = 2.
    geometry = target_geometry_from_rgbd(
        mask, depth, 1.74, .05, 25., .25, require_valid_sphere=True,
    )

    assert geometry is None


def test_online_partial_sphere_remains_precise_after_near_clip():
    expected = np.array((.36, -.07, .08))
    mask, depth = _render_forward_depth_sphere(expected, .25)
    mask &= depth >= .15
    mask[:, :120] = False
    geometry = target_geometry_from_rgbd(
        mask, depth, 1.74, .15, 25., .25, require_valid_sphere=True,
    )

    assert geometry is not None
    assert geometry.center_camera == pytest.approx(expected, abs=.01)


def test_sphere_center_rejects_almost_planar_tiny_cap():
    angle = np.linspace(-.001, .001, 15)
    left, up = np.meshgrid(angle, angle)
    points = np.column_stack((
        .45 - np.sqrt(.25 ** 2 - left.ravel() ** 2 - up.ravel() ** 2),
        left.ravel(), up.ravel(),
    ))

    assert rgbd_target_localizer.validation_sphere_center(points, .25) is None


def test_observable_sphere_fit_tolerates_submillimetre_depth_noise():
    expected = np.array((.45, -.08, .12))
    mask, depth = _render_forward_depth_sphere(expected, .25)
    rng = np.random.default_rng(5)
    depth[mask] += rng.normal(0., .0005, np.count_nonzero(mask))
    geometry = target_geometry_from_rgbd(
        mask, depth, 1.74, .05, 25., .25, require_valid_sphere=True,
    )

    assert geometry is not None
    assert np.linalg.norm(np.array(geometry.center_camera) - expected) < .01


def test_rgbd_center_pixel_recovers_forward_target_center():
    mask = np.zeros((4, 6), dtype=bool)
    mask[2, 3] = True
    depth = np.full((4, 6), math.inf, dtype=float)
    depth[2, 3] = 9.75

    vector = target_vector_from_rgbd(
        mask,
        depth,
        horizontal_fov=math.pi / 2.0,
        minimum_depth=0.2,
        maximum_depth=25.0,
        target_radius=0.25,
    )

    assert vector == pytest.approx((10.0, 0.0, 0.0))


def test_rgbd_geometry_exposes_bounded_mask_and_depth_diagnostics():
    mask = np.zeros((5, 7), dtype=bool)
    mask[1:4, 2:6] = True
    depth = np.full((5, 7), math.inf, dtype=float)
    depth[1:4, 2:6] = np.array((
        (4.9, 4.8, 4.9, math.inf),
        (4.8, 4.75, 4.8, 4.9),
        (4.9, 4.8, 4.9, math.nan),
    ))

    geometry = target_geometry_from_rgbd(
        mask,
        depth,
        horizontal_fov=math.pi / 2.0,
        minimum_depth=0.2,
        maximum_depth=25.0,
        target_radius=0.25,
    )

    assert geometry.red_pixel_count == 12
    assert geometry.valid_depth_count == 10
    assert geometry.mask_bbox == (2, 1, 5, 3)
    assert geometry.mask_center == pytest.approx((3.5, 2.0))
    assert geometry.projection_center == pytest.approx((3.0, 2.0))
    assert geometry.depth_min == pytest.approx(4.75)
    assert geometry.depth_median == pytest.approx(4.85)
    assert geometry.depth_mad == pytest.approx(0.05)
    assert geometry.surface_camera[0] == pytest.approx(4.85)
    fx, fy, cx, cy = geometry.intrinsics
    u, v = geometry.projection_center
    length = math.sqrt(1 + ((u - cx) / fx) ** 2
                       + ((v - cy) / fy) ** 2)
    assert geometry.center_camera[0] == pytest.approx(4.85 + 0.25 / length)


def test_off_axis_depth_is_forward_axis_not_euclidean_range():
    mask = np.zeros((5, 7), dtype=bool)
    mask[2, 5] = True
    depth = np.full((5, 7), math.inf, dtype=float)
    depth[2, 5] = 4.75

    geometry = target_geometry_from_rgbd(
        mask,
        depth,
        horizontal_fov=math.pi / 2.0,
        minimum_depth=0.2,
        maximum_depth=25.0,
        target_radius=0.25,
    )

    # Gazebo Rendering publishes the reconstructed camera-forward X
    # component in the depth image.  Therefore pinhole Y/Z use the same
    # forward depth; treating 4.75 as a slant range would under-project Y.
    fx = geometry.intrinsics[0]
    ray_length = math.sqrt(
        1.0 + ((5.0 - geometry.intrinsics[2]) / fx) ** 2
        + ((2.0 - geometry.intrinsics[3]) / fx) ** 2
    )
    forward = 4.75 + 0.25 / ray_length
    expected_left = -(5.0 - geometry.intrinsics[2]) * forward / fx
    assert geometry.center_camera[0] == pytest.approx(forward)
    assert geometry.center_camera[1] == pytest.approx(expected_left)
    assert np.linalg.norm(geometry.center_camera) > 5.0


def test_off_axis_radius_is_added_along_unit_observation_ray():
    mask = np.zeros((5, 7), dtype=bool)
    mask[2, 5] = True
    depth = np.full((5, 7), math.inf, dtype=float)
    depth[2, 5] = 4.75
    geometry = target_geometry_from_rgbd(
        mask, depth, math.pi / 2.0, 0.2, 25.0, 0.25,
    )
    fx, fy, cx, cy = geometry.intrinsics
    ray = np.array((1.0, -(5.0 - cx) / fx, -(2.0 - cy) / fy))
    expected = (4.75 + 0.25 / np.linalg.norm(ray)) * ray
    assert geometry.surface_camera == pytest.approx(4.75 * ray)
    assert geometry.center_camera == pytest.approx(expected)
    assert np.linalg.norm(
        np.asarray(geometry.center_camera)
        - np.asarray(geometry.surface_camera)
    ) == pytest.approx(0.25)


def test_ray_radius_is_not_claimed_as_exact_for_partial_sphere():
    mask = np.zeros((5, 7), dtype=bool)
    mask[1:4, 3:6] = True
    depth = np.full((5, 7), math.inf, dtype=float)
    # A deliberately asymmetric visible patch has a surface-depth median
    # that is not the optical-axis front point.  The diagnostic preserves
    # this evidence instead of hiding it behind a fixed world-axis offset.
    depth[1:4, 3:6] = np.array((
        (7.86, 7.88, 7.91),
        (7.75, 7.80, 7.86),
        (7.86, 7.88, 7.91),
    ))

    geometry = target_geometry_from_rgbd(
        mask,
        depth,
        horizontal_fov=1.74,
        minimum_depth=0.2,
        maximum_depth=25.0,
        target_radius=0.25,
    )

    assert geometry.depth_median == pytest.approx(7.86)
    fx, fy, cx, cy = geometry.intrinsics
    u, v = geometry.projection_center
    length = math.sqrt(1 + ((u - cx) / fx) ** 2
                       + ((v - cy) / fy) ** 2)
    assert geometry.center_camera[0] == pytest.approx(7.86 + 0.25 / length)
    assert geometry.depth_mad > 0.0


def test_rgbd_projection_uses_camera_flu_image_signs():
    mask = np.zeros((3, 5), dtype=bool)
    mask[0, 4] = True
    depth = np.full((3, 5), math.inf, dtype=float)
    depth[0, 4] = 5.0

    vector = target_vector_from_rgbd(
        mask,
        depth,
        horizontal_fov=math.pi / 2.0,
        minimum_depth=0.2,
        maximum_depth=25.0,
    )

    assert vector[0] == pytest.approx(5.0)
    assert vector[1] < 0.0
    assert vector[2] > 0.0


def test_camera_pose_transform_compensates_mount_pitch_and_translation():
    pitch = math.radians(12.0)
    desired_body_flu = np.array((10.0, 2.0, -3.0))
    translation = np.array((0.35, 0.0, -0.05))
    relative_body = desired_body_flu - translation
    camera_vector = np.array((
        math.cos(pitch) * relative_body[0]
        - math.sin(pitch) * relative_body[2],
        relative_body[1],
        math.sin(pitch) * relative_body[0]
        + math.cos(pitch) * relative_body[2],
    ))

    position = camera_target_to_local_ned(
        camera_vector,
        uav_position_ned=(1.0, 4.0, -5.0),
        attitude_quaternion=(1.0, 0.0, 0.0, 0.0),
        camera_translation_flu=translation,
        camera_pitch_down=pitch,
        target_reference_z_offset=0.42,
    )

    assert position == pytest.approx((11.0, 2.0, -1.58))


def test_default_camera_translation_uses_px4_model_origin():
    # x500_base's merged model has base_link at z=+0.24 m.  The front camera
    # is z=+0.15 m relative to that link, hence z=+0.39 m from model origin.
    mount = rgbd_target_localizer.DEFAULT_CAMERA_TRANSLATION_FLU
    assert mount == pytest.approx((0.18, 0.0, 0.39))
    baseline = (Path(__file__).parents[2]
                / 'uav_usv_bringup/config/baseline.yaml')
    parameters = yaml.safe_load(baseline.read_text())[
        'rgbd_target_localizer']['ros__parameters']
    assert tuple(
        parameters[f'camera_translation_{axis}'] for axis in 'xyz'
    ) == pytest.approx(mount)
    # This independent gate has a radius-derived physical meaning; changing
    # the configured sphere radius must update the ceiling as well.
    assert parameters['maximum_depth_mad'] == pytest.approx(
        parameters['target_radius']
    )


def test_body_to_ned_rotation_applies_yaw():
    half_angle = math.pi / 4.0
    rotation = body_frd_to_ned_rotation((
        math.cos(half_angle),
        0.0,
        0.0,
        math.sin(half_angle),
    ))

    assert rotation @ np.array((10.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 10.0, 0.0),
        abs=1.0e-9,
    )


def _quaternion_from_roll_pitch_yaw(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


@pytest.mark.parametrize(
    'attitude',
    (
        (0.0, 0.0, 0.0),
        (math.radians(8.0), math.radians(-6.0), math.radians(25.0)),
        (math.radians(-12.0), math.radians(10.0), math.radians(-70.0)),
    ),
)
def test_camera_body_ned_transform_round_trip_for_full_attitude(attitude):
    quaternion = _quaternion_from_roll_pitch_yaw(*attitude)
    camera_vector = np.array((8.0, -1.2, 0.7))
    uav_position = np.array((3.0, -4.0, -6.0))
    translation = np.array((0.35, 0.0, -0.05))
    camera_pitch = math.radians(12.0)

    target_ned = camera_target_to_local_ned(
        camera_vector,
        uav_position,
        quaternion,
        translation,
        camera_pitch,
    )
    reconstructed = local_ned_target_to_camera_flu(
        target_ned,
        uav_position,
        quaternion,
        translation,
        camera_pitch,
    )

    assert reconstructed == pytest.approx(camera_vector, abs=1.0e-9)


def _render_forward_depth_sphere(center, radius, width=320, height=240):
    fx, fy, cx, cy = rgbd_target_localizer.camera_intrinsics(
        width, height, 1.74
    )
    columns, rows = np.meshgrid(np.arange(width), np.arange(height))
    ray_y = -(columns - cx) / fx
    ray_z = -(rows - cy) / fy
    center = np.asarray(center, dtype=float)
    a = 1.0 + ray_y * ray_y + ray_z * ray_z
    b = -2.0 * (center[0] + center[1] * ray_y + center[2] * ray_z)
    c = float(center @ center - radius * radius)
    discriminant = b * b - 4.0 * a * c
    mask = discriminant >= 0.0
    depth = np.full((height, width), math.inf, dtype=float)
    depth[mask] = (
        -b[mask] - np.sqrt(discriminant[mask])
    ) / (2.0 * a[mask])
    mask &= depth > 0.0
    depth[~mask] = math.inf
    return mask, depth


@pytest.mark.parametrize('distance', (3.0, 5.0, 8.0, 12.0))
def test_synthetic_sphere_recovers_center_without_median_surface_bias(distance):
    radius = 0.25
    expected = np.array((distance, 0.0, 0.0))
    mask, depth = _render_forward_depth_sphere(expected, radius)

    geometry = target_geometry_from_rgbd(
        mask, depth, 1.74, 0.2, 25.0, radius
    )

    assert geometry is not None
    assert geometry.projection_center == pytest.approx((160.0, 120.0))
    assert geometry.center_camera == pytest.approx(expected, abs=1e-4)


def test_synthetic_off_axis_and_edge_spheres_keep_correct_camera_signs():
    cases = (
        np.array((5.0, -1.0, 0.6)),
        np.array((8.0, 5.0, -2.0)),
    )
    for expected in cases:
        mask, depth = _render_forward_depth_sphere(expected, 0.25)
        geometry = target_geometry_from_rgbd(
            mask, depth, 1.74, 0.2, 25.0, 0.25
        )

        assert geometry is not None
        assert math.copysign(1.0, geometry.center_camera[1]) == (
            math.copysign(1.0, expected[1])
        )
        assert math.copysign(1.0, geometry.center_camera[2]) == (
            math.copysign(1.0, expected[2])
        )
        assert np.linalg.norm(
            np.asarray(geometry.center_camera) - expected
        ) < 0.35


def test_partial_synthetic_sphere_recovers_center_from_visible_surface():
    expected = np.array((5.0, -0.8, 0.4))
    mask, depth = _render_forward_depth_sphere(expected, 0.25)
    visible_columns = np.nonzero(mask)[1]
    mask[:, :int(np.median(visible_columns))] = False

    geometry = target_geometry_from_rgbd(
        mask, depth, 1.74, 0.2, 25.0, 0.25
    )

    assert geometry is not None
    assert geometry.center_camera == pytest.approx(expected, abs=1e-4)


def test_rgbd_localizer_rejects_missing_valid_depth():
    mask = np.ones((2, 2), dtype=bool)
    depth = np.full((2, 2), math.inf, dtype=float)

    assert target_vector_from_rgbd(
        mask,
        depth,
        horizontal_fov=1.74,
        minimum_depth=0.2,
        maximum_depth=25.0,
    ) is None


def test_image_callbacks_only_replace_latest_frames(monkeypatch):
    """Catch image decoding and red segmentation returning to DDS callbacks."""
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    node.latest_color_message = None
    node.latest_depth_message = None
    node.color_time = -1.0
    node.depth_time = -1.0
    node.color_measurement_time = None
    node.depth_measurement_time = None
    node.data_timeout = 0.5
    node.image_clock_mapper = Px4RosClockMapper(0.05)
    node.image_pair_buffer = RgbDepthPairBuffer(0.05)
    node.pending_image_pairs = __import__('collections').deque(maxlen=8)
    node.last_pair_rejection = ''
    node._ros_seconds = lambda: 10.0
    monkeypatch.setattr(
        rgbd_target_localizer,
        'red_pixel_mask',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('heavy color work ran in callback')
        ),
    )
    monkeypatch.setattr(
        rgbd_target_localizer,
        'decode_float32_depth',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('heavy depth work ran in callback')
        ),
    )
    stamp = SimpleNamespace(sec=2, nanosec=0)
    header = SimpleNamespace(stamp=stamp)
    color = SimpleNamespace(header=header)
    depth = SimpleNamespace(header=header)

    node.color_callback(color)
    node.depth_callback(depth)

    assert node.latest_color_message is color
    assert node.latest_depth_message is depth


def test_sensor_stamp_must_be_in_the_ros_clock_domain():
    assert validated_sensor_stamp(
        measurement_stamp=10.0,
        receipt_stamp=10.04,
        maximum_age=0.5,
    ) == pytest.approx(10.0)
    assert validated_sensor_stamp(
        measurement_stamp=2.0,
        receipt_stamp=10.0,
        maximum_age=0.5,
    ) is None
    assert validated_sensor_stamp(
        measurement_stamp=10.2,
        receipt_stamp=10.0,
        maximum_age=0.5,
    ) is None


def test_rgb_depth_pair_uses_acquisition_time_and_rejects_mismatch():
    assert synchronized_measurement_time(10.00, 10.04, 0.05) == (
        pytest.approx(10.00)
    )
    assert synchronized_measurement_time(10.00, 10.06, 0.05) is None


def test_pose_history_interpolates_at_image_acquisition_time():
    history = TimestampedVectorHistory(maximum_age=1.0)
    assert history.add(10.0, (0.0, 0.0, -5.0))
    assert history.add(10.2, (2.0, 4.0, -5.0))

    assert history.value_at(10.1) == pytest.approx((1.0, 2.0, -5.0))
    assert history.value_at(9.9) is None
    assert history.value_at(10.3) is None
    assert not history.add(10.1, (99.0, 99.0, 99.0))


def test_quaternion_history_uses_shortest_path_slerp():
    history = TimestampedVectorHistory(
        maximum_age=1.0,
        interpolator=quaternion_slerp,
    )
    assert history.add(10.0, (1.0, 0.0, 0.0, 0.0))
    assert history.add(10.2, (0.0, 0.0, 0.0, 1.0))

    halfway = history.value_at(10.1)

    assert halfway == pytest.approx(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )
    assert np.linalg.norm(halfway) == pytest.approx(1.0)
    assert quaternion_slerp(
        (1.0, 0.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0, 0.0),
        0.5,
    ) == pytest.approx((1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    'stamp, position_range, attitude_range, expected',
    (
        (10.1, None, (10.0, 10.2), 'POSITION_HISTORY_EMPTY'),
        (10.1, (10.0, 10.2), None, 'ATTITUDE_HISTORY_EMPTY'),
        (
            9.9, (10.0, 10.2), (9.8, 10.2),
            'POSITION_TIMESTAMP_BEFORE_HISTORY',
        ),
        (
            9.9, (9.8, 10.2), (10.0, 10.2),
            'ATTITUDE_TIMESTAMP_BEFORE_HISTORY',
        ),
        (
            10.3, (10.0, 10.2), (10.0, 10.4),
            'POSITION_TIMESTAMP_AFTER_HISTORY',
        ),
        (
            10.3, (10.0, 10.4), (10.0, 10.2),
            'ATTITUDE_TIMESTAMP_AFTER_HISTORY',
        ),
    ),
)
def test_pose_history_rejection_reason_is_stream_specific(
    stamp,
    position_range,
    attitude_range,
    expected,
):
    assert pose_history_rejection_reason(
        stamp,
        position_range,
        attitude_range,
    ) == expected


def test_px4_clock_mapper_rejects_a_time_jump_instead_of_retiming_pose():
    mapper = Px4RosClockMapper(maximum_offset_jump=0.05)

    assert mapper.to_ros_time(2.0, receipt_ros_time=10.0) is None
    assert mapper.to_ros_time(2.02, receipt_ros_time=10.02) is None
    assert mapper.to_ros_time(2.05, receipt_ros_time=10.05) is None
    assert mapper.to_ros_time(
        2.1, receipt_ros_time=10.1
    ) == pytest.approx(10.1)
    assert mapper.to_ros_time(1.0, receipt_ros_time=10.2) is None


def test_shared_px4_mapper_tolerates_cross_topic_interleaving():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
    )
    samples = (
        ('position', 10.050, 100.070),
        ('attitude', 10.040, 100.075),
        ('position', 10.100, 100.120),
        ('attitude', 10.090, 100.125),
    )

    mapped = [
        mapper.to_ros_time(source, receipt, stream_name=stream)
        for stream, source, receipt in samples
    ]

    assert mapper.reset_count == 0
    assert mapped[-1] == pytest.approx(100.110)


def test_px4_reset_requires_consistent_regression_from_both_streams():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
    )
    for stream, source in (
        ('position', 10.00),
        ('attitude', 10.00),
        ('position', 10.05),
        ('attitude', 10.05),
    ):
        mapper.to_ros_time(source, source + 90.0, stream_name=stream)
    before = mapper.reset_count

    assert mapper.to_ros_time(
        1.00, 100.10, stream_name='position'
    ) is None
    assert mapper.reset_count == before
    assert mapper.to_ros_time(
        0.99, 100.11, stream_name='attitude'
    ) is None
    assert mapper.reset_count == before + 1


def test_ros_clock_rollback_resets_shared_mapping_immediately():
    mapper = Px4RosClockMapper(calibration_samples=1)
    assert mapper.to_ros_time(
        10.0, 100.0, stream_name='position'
    ) == pytest.approx(100.0)

    assert mapper.to_ros_time(
        10.1, 99.0, stream_name='attitude'
    ) is None
    assert mapper.reset_count == 1
    assert mapper.last_status == 'ROS_CLOCK_RESET'


def test_single_delayed_px4_message_does_not_reset_shared_mapping():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=2,
    )
    assert mapper.to_ros_time(
        10.0, 100.0, stream_name='position'
    ) is None
    assert mapper.to_ros_time(
        10.0, 100.01, stream_name='attitude'
    ) == pytest.approx(100.0)
    before = mapper.reset_count

    assert mapper.to_ros_time(
        10.05, 100.5, stream_name='position'
    ) == pytest.approx(100.05)
    assert mapper.reset_count == before


def test_single_forward_timestamp_outlier_does_not_reset_shared_mapping():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=2,
    )
    assert mapper.to_ros_time(
        10.0, 100.0, stream_name='position'
    ) is None
    assert mapper.to_ros_time(
        10.0, 100.01, stream_name='attitude'
    ) == pytest.approx(100.0)
    before = mapper.reset_count

    assert mapper.to_ros_time(
        15.0, 100.05, stream_name='position'
    ) is None
    assert mapper.last_status == 'OFFSET_OUTLIER'
    assert mapper.reset_count == before
    assert mapper.to_ros_time(
        10.05, 100.06, stream_name='position'
    ) == pytest.approx(100.05)


@pytest.mark.parametrize('old_offset', (0.507, 0.191))
def test_sustained_two_stream_offset_change_recovers_mapping(old_offset):
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
        offset_recovery_samples=3,
    )
    for stream, source in (
        ('position', 10.00),
        ('attitude', 10.00),
        ('position', 10.05),
        ('attitude', 10.05),
    ):
        mapper.to_ros_time(
            source,
            source + old_offset,
            stream_name=stream,
        )
    assert mapper.offset == pytest.approx(old_offset)

    mapped = []
    for index in range(6):
        source = 20.0 + 0.05 * index
        mapped.append(mapper.to_ros_time(
            source,
            source + 0.004,
            stream_name='position',
        ))
        mapped.append(mapper.to_ros_time(
            source + 0.002,
            source + 0.008,
            stream_name='attitude',
        ))

    assert mapper.offset_recalibration_count == 1
    assert mapper.reset_count == 1
    assert mapper.calibration_count == 2
    assert mapper.offset == pytest.approx(0.004)
    assert mapped[-1] == pytest.approx(20.256)
    assert mapper.last_status == 'MAPPED'


def test_one_stream_offset_outlier_does_not_trigger_recalibration():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
        offset_recovery_samples=3,
    )
    for stream, source in (
        ('position', 10.00),
        ('attitude', 10.00),
        ('position', 10.05),
        ('attitude', 10.05),
    ):
        mapper.to_ros_time(
            source,
            source + 0.507,
            stream_name=stream,
        )

    assert mapper.to_ros_time(
        20.10, 20.105, stream_name='position'
    ) is None
    assert mapper.to_ros_time(
        20.10, 20.607, stream_name='attitude'
    ) == pytest.approx(20.607)
    assert mapper.reset_count == 0
    assert mapper.offset_recalibration_count == 0


def _make_causal_pose_localizer(now):
    """Build only callback state, using independent source/sim/system clocks."""
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    node._ros_seconds = lambda: now[0]
    node.timesync_history = deque(maxlen=16)
    node._pending_raw_pose_samples = deque(maxlen=32)
    node._pending_pose_samples = deque(maxlen=8)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(9.9, 99.9, 99.91, 1.0)
    node.image_clock_mapper.add_anchor(10.4, 100.4, 100.41, 1.5)
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(
        1.0, interpolator=quaternion_slerp,
    )
    node.uav_position = node.uav_attitude = None
    node.uav_position_time = node.uav_attitude_time = -math.inf
    node.position_mapped_stamp = node.attitude_mapped_stamp = math.nan
    node.position_raw_sim_stamp = node.attitude_raw_sim_stamp = math.nan
    node.latest_timesync_stamp = node.latest_timesync_offset = math.nan
    node.waiting_image_pair = None
    _send_timesync(node, 49.9, -40.0)
    return node


def _send_timesync(node, stamp, offset):
    message = TimesyncStatus()
    message.timestamp = round(stamp * 1_000_000)
    message.estimated_offset = round(offset * 1_000_000)
    node.timesync_callback(message)


def _send_pose(node, kind, source, quaternion=(1.0, 0.0, 0.0, 0.0),
               position=(0.0, 0.0, -5.0)):
    if kind == 'position':
        message = VehicleLocalPosition()
        message.x, message.y, message.z = position
        message.xy_valid = message.z_valid = True
        callback = node.position_callback
    else:
        message = VehicleAttitude()
        message.q = list(quaternion)
        callback = node.attitude_callback
    message.timestamp = round(source * 1_000_000)
    callback(message)


def test_causal_timesync_offset_change_preserves_previous_pose_history():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    _send_pose(node, 'position', 50.0)
    _send_pose(node, 'attitude', 50.0)
    _send_timesync(node, 50.1, -40.1)
    _send_timesync(node, 50.3, -40.2)

    # A late-arriving pose must use 50.1, never the newest future sample 50.3.
    _send_pose(node, 'position', 50.2, position=(2.0, 4.0, -5.0))
    _send_pose(node, 'attitude', 50.05)

    assert node.position_raw_sim_stamp == pytest.approx(10.1)
    assert node.position_history.time_range == pytest.approx((100.0, 100.1))
    assert node.position_history.value_at(100.05) == pytest.approx((1.0, 2.0, -5.0))
    assert node.attitude_history.time_range == pytest.approx((100.0, 100.05))
    mapped, raw, causal_stamp = node._map_px4_pose_stamp(50.2, 100.49)
    assert (mapped, raw, causal_stamp) == pytest.approx((100.1, 10.1, 50.1))


def test_px4_pose_without_causal_timesync_does_not_use_future_offset_or_receipt():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    node.timesync_history.clear()
    _send_timesync(node, 50.2, -40.0)

    _send_pose(node, 'position', 50.1)
    _send_pose(node, 'attitude', 50.1)

    assert node.position_history.time_range is None
    assert node.attitude_history.time_range is None
    assert node.uav_position is None and node.uav_attitude is None
    assert not node._pending_raw_pose_samples


def test_px4_pose_waits_for_clock_bracket_then_flushes_at_acquisition_time():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    _send_pose(node, 'attitude', 50.2)
    _send_pose(node, 'position', 50.6, position=(3.0, 4.0, -5.0))
    _send_pose(node, 'attitude', 50.6, quaternion=(-1.0, 0.0, 0.0, 0.0))
    assert node.position_history.time_range is None
    assert node.attitude_history.time_range == pytest.approx((100.2, 100.2))
    assert len(node._pending_raw_pose_samples) == 2

    now[0] = 100.82
    node.image_clock_mapper.add_anchor(10.8, 100.8, 100.81, 1.9)
    node._flush_pending_raw_pose_samples()

    assert not node._pending_raw_pose_samples
    assert node.position_history.time_range == pytest.approx((100.6, 100.6))
    assert node.uav_position == pytest.approx((3.0, 4.0, -5.0))
    assert node.attitude_history.time_range == pytest.approx((100.2, 100.6))
    assert node.uav_attitude == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert node.position_raw_sim_stamp == pytest.approx(10.6)
    assert node.attitude_raw_sim_stamp == pytest.approx(10.6)


def test_gazebo_clock_rewind_clears_pose_and_pending_raw_samples():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    _send_pose(node, 'position', 50.2)
    _send_pose(node, 'attitude', 50.2)
    _send_pose(node, 'position', 50.6)
    node.waiting_image_pair = ('old-color', 'old-depth')

    now[0] = 100.51
    node.gazebo_clock_callback(SimpleNamespace(
        sim=SimpleNamespace(sec=0, nsec=100_000_000),
        system=SimpleNamespace(sec=100, nsec=500_000_000),
    ))

    assert node.image_clock_mapper.reset_count == 1
    assert node._image_clock_reset_pending
    assert node.position_history.time_range is None
    assert node.attitude_history.time_range is None
    assert not node._pending_raw_pose_samples
    assert node.waiting_image_pair is None


def _enable_navigation(node):
    import threading
    messages = []
    node.navigation_state_pub = SimpleNamespace(publish=messages.append)
    node._navigation_lock = threading.RLock()
    node._navigation_samples = deque(maxlen=32)
    node._navigation_last_stamp = -math.inf
    node.pose_wait_timeout = .15
    return messages


def _navigation_pose(source):
    message = VehicleLocalPosition()
    message.timestamp_sample = round(source * 1e6)
    message.x, message.y, message.z = 3., 4., -5.
    message.vx = 4.
    message.ax = 1.
    message.xy_valid = message.z_valid = True
    message.v_xy_valid = message.v_z_valid = True
    return message


def test_navigation_preserves_sample_time_and_raw_clock_provenance():
    node = _make_causal_pose_localizer([100.42])
    messages = _enable_navigation(node)
    node.position_callback(_navigation_pose(50.2))
    assert len(messages) == 1
    message = messages[0]
    assert message.stamp.sec + message.stamp.nanosec * 1e-9 == (
        pytest.approx(100.2)
    )
    assert message.valid and message.frame_id == 'local_ned'
    assert message.px4_timestamp_sample == 50_200_000
    assert message.native_timestamp_sample == 10_200_000
    assert message.position.x == 3. and message.velocity.x == 4.
    assert message.acceleration.x == 1.


def test_navigation_waits_for_clock_bracket_without_receipt_fallback():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    messages = _enable_navigation(node)
    node.position_callback(_navigation_pose(50.6))
    assert not messages
    now[0] = 100.82
    node.image_clock_mapper.add_anchor(10.8, 100.8, 100.81, 1.9)
    node._flush_navigation_states()
    assert len(messages) == 1
    assert messages[0].stamp.nanosec == 600_000_000
    node._reset_pose_time_state()
    assert not messages[-1].valid
    assert not node._navigation_samples


def test_navigation_clock_wait_timeout_drops_sample():
    import time
    node = _make_causal_pose_localizer([100.42])
    messages = _enable_navigation(node)
    node._navigation_samples.append((
        _navigation_pose(50.6), 10.6, time.monotonic() - 1.,
    ))
    node._flush_navigation_states()
    assert not messages and not node._navigation_samples


def test_px4_ros_domain_mode_keeps_true_source_sample_time():
    mapper = Px4RosClockMapper(
        source_is_ros_time=True,
        calibration_samples=4,
    )

    mapped = mapper.to_ros_time(
        100.125,
        receipt_ros_time=100.500,
        stream_name='position',
    )

    assert mapped == pytest.approx(100.125)
    assert mapper.offset == pytest.approx(0.0)
    assert mapper.last_status == 'DIRECT_TIMESTAMP'


def test_duplicate_and_small_out_of_order_sample_are_bounded_per_stream():
    mapper = Px4RosClockMapper(calibration_samples=1)
    mapper.to_ros_time(10.0, 100.0, stream_name='position')
    before = mapper.reset_count

    assert mapper.to_ros_time(
        10.0, 100.01, stream_name='position'
    ) is None
    assert mapper.last_status == 'DUPLICATE_SOURCE_TIMESTAMP'
    assert mapper.to_ros_time(
        9.99, 100.02, stream_name='position'
    ) is None
    assert mapper.last_status == 'OUT_OF_ORDER_SOURCE_TIMESTAMP'
    assert mapper.reset_count == before


def test_cross_topic_interleaving_does_not_clear_pose_histories():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    for kind, source, receipt in (
        ('position', 50.050, 100.42),
        ('attitude', 50.040, 100.43),
        ('position', 50.100, 100.44),
        ('attitude', 50.090, 100.45),
    ):
        now[0] = receipt
        _send_pose(node, kind, source)

    assert node.position_history.time_range == pytest.approx((100.05, 100.1))
    assert node.attitude_history.time_range == pytest.approx((100.04, 100.09))
    assert len(node.timesync_history) == 1


def test_one_bad_position_timestamp_does_not_clear_attitude_history():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    for source in (50.0, 50.05):
        _send_pose(node, 'position', source)
        _send_pose(node, 'attitude', source)
    attitude_range = node.attitude_history.time_range
    position_range = node.position_history.time_range

    now[0] = 100.48
    _send_pose(node, 'position', 1.0)  # No causal timesync sample: discard.
    _send_pose(node, 'position', 50.01)  # Mapped but older than good position.
    _send_pose(node, 'position', 50.6)  # Waiting for future clock anchor.

    assert node.attitude_history.time_range == attitude_range
    assert node.position_history.time_range == position_range
    assert node.uav_position_time == pytest.approx(100.05)
    assert len(node.timesync_history) == 1
    assert len(node._pending_raw_pose_samples) == 1
    assert node._pending_raw_pose_samples[0][1] == pytest.approx(10.6)


def test_confirmed_px4_restart_clears_both_old_pose_histories():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    for source in (50.0, 50.05):
        _send_pose(node, 'position', source)
        _send_pose(node, 'attitude', source)
    _send_pose(node, 'position', 50.6)
    _send_pose(node, 'attitude', 50.6)
    node._pending_pose_samples.append(('position', 50.6, (0.0, 0.0, -5.0)))
    node.waiting_image_pair = ('old-clock-color', 'old-clock-depth')
    assert node.position_history.time_range is not None
    assert node.attitude_history.time_range is not None
    assert len(node._pending_raw_pose_samples) == 2

    # Timesync source rewind is the current P8.1 epoch-reset trigger.
    _send_timesync(node, 1.0, 9.0)

    assert node.position_history.time_range is None
    assert node.attitude_history.time_range is None
    assert node.uav_position is None and node.uav_attitude is None
    assert math.isnan(node.position_mapped_stamp)
    assert math.isnan(node.attitude_mapped_stamp)
    assert node.waiting_image_pair is None
    assert not node._pending_raw_pose_samples
    assert not node._pending_pose_samples
    assert tuple(node.timesync_history) == ((1.0, 9.0),)

    _send_pose(node, 'position', 1.15)
    _send_pose(node, 'attitude', 1.16)
    assert node.position_history.time_range == pytest.approx((100.15, 100.15))
    assert node.attitude_history.time_range == pytest.approx((100.16, 100.16))


def test_rgb_depth_pairs_by_acquisition_time_despite_transport_delay():
    pairs = RgbDepthPairBuffer(maximum_skew=0.02, capacity=4)
    color = object()
    depth = object()

    assert pairs.add('color', color, 5.0, 10.01) == ''
    assert pairs.add('depth', depth, 5.0, 10.08) == ''
    paired_color, paired_depth, reason = pairs.pop_pair()

    assert reason == ''
    assert paired_color.message is color
    assert paired_depth.message is depth
    assert paired_color.raw_stamp == paired_depth.raw_stamp == 5.0


def test_aligned_sensor_config_does_not_consume_previous_depth_frame():
    baseline = (Path(__file__).resolve().parents[2]
                / 'uav_usv_bringup/config/baseline.yaml')
    parameters = yaml.safe_load(baseline.read_text())[
        'rgbd_target_localizer']['ros__parameters']
    pairs = RgbDepthPairBuffer(parameters['maximum_rgb_depth_skew'])
    color, old_depth, matching_depth = object(), object(), object()
    pairs.add('depth', old_depth, 5.0, 10.0)
    pairs.add('color', color, 5.052, 10.06)

    paired_color, paired_depth, reason = pairs.pop_pair()

    assert paired_color is None and paired_depth is None
    assert reason == 'DEPTH_FRAME_UNMATCHED'
    pairs.add('depth', matching_depth, 5.052, 10.07)
    paired_color, paired_depth, reason = pairs.pop_pair()
    assert reason == ''
    assert paired_color.message is color
    assert paired_depth.message is matching_depth
    assert paired_color.raw_stamp == paired_depth.raw_stamp == 5.052


def test_rgb_depth_pairing_selects_closest_acquisition_times():
    pairs = RgbDepthPairBuffer(maximum_skew=0.1, capacity=4)
    early_color = object()
    nearest_color = object()
    depth = object()
    pairs.add('color', early_color, 5.00, 10.01)
    pairs.add('color', nearest_color, 5.09, 10.10)
    pairs.add('depth', depth, 5.08, 10.20)

    paired_color, paired_depth, reason = pairs.pop_pair()

    assert reason == ''
    assert paired_color.message is nearest_color
    assert paired_depth.message is depth
    assert paired_color.raw_stamp - paired_depth.raw_stamp == pytest.approx(
        0.01
    )
    assert len(pairs.color) == 1
    assert pairs.color[0].message is early_color


def test_rgb_depth_pairs_are_consumed_once_and_keep_real_skew():
    pairs = RgbDepthPairBuffer(maximum_skew=0.1, capacity=4)
    pairs.add('color', object(), 8.00, 20.02)
    pairs.add('depth', object(), 8.04, 20.11)

    color, depth, reason = pairs.pop_pair()

    assert reason == ''
    assert depth.raw_stamp - color.raw_stamp == pytest.approx(0.04)
    assert pairs.pop_pair() == (None, None, '')


def test_rgb_depth_buffer_rejects_duplicates_mismatch_and_resets():
    pairs = RgbDepthPairBuffer(maximum_skew=0.02, capacity=4)
    assert pairs.add('color', object(), 5.0, 10.0) == ''
    assert pairs.add('color', object(), 5.0, 10.1) == 'DUPLICATE_FRAME'
    assert pairs.add('depth', object(), 5.2, 10.2) == ''
    assert pairs.pop_pair()[2] == 'RGB_FRAME_UNMATCHED'
    assert pairs.add('color', object(), 1.0, 11.0) == 'TIME_RESET'
    assert len(pairs.depth) == 0


def test_synthetic_synchronized_red_rgbd_produces_valid_observation():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [100.12]
    node._ros_seconds = lambda: now[0]
    node.maximum_rgb_depth_skew = 0.02
    node.data_timeout = 0.5
    node.image_pair_buffer = RgbDepthPairBuffer(0.02, 4)
    node.pending_image_pairs = __import__('collections').deque(maxlen=4)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.px4_clock_mapper = Px4RosClockMapper(0.05, calibration_samples=4)
    node.image_clock_mapper.add_anchor(10.0, 100.0, 100.01, 1.0)
    node.image_clock_mapper.add_anchor(10.2, 100.2, 100.21, 1.2)
    node.image_clock_wait_timeout = 0.15
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(1.0)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.2, (0.0, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.2, (1.0, 0.0, 0.0, 0.0))
    node.color_time = node.depth_time = -math.inf
    node.last_pair_rejection = ''
    node.last_image_timeout_report = -math.inf
    node.last_time_diagnostic = {}
    node.frame_id = 'local_ned'
    node.horizontal_fov = math.pi / 2.0
    node.minimum_red_pixels = 3
    node.minimum_depth = 0.2
    node.maximum_depth = 25.0
    node.minimum_depth_ratio = 0.5
    node.maximum_depth_mad = 0.25
    node.camera_translation_flu = (0.0, 0.0, 0.0)
    node.camera_pitch_down = 0.0
    node.target_radius = 0.0
    node.target_reference_z_offset = 0.0
    node.base_position_std = 0.08
    node.range_position_std_scale = 0.01
    node.geometry_diagnostics_enabled = True
    observations = []
    positions = []
    node.observation_pub = SimpleNamespace(
        publish=lambda message: observations.append(message)
    )
    node.target_position_pub = SimpleNamespace(
        publish=lambda message: positions.append(message)
    )

    color_array = np.zeros((4, 4, 3), dtype=np.uint8)
    color_array[1:3, 1:3, 2] = 255
    depth_array = np.full((4, 4), 5.0, dtype='<f4')
    color = Image()
    color.header.stamp.sec = 10
    color.header.stamp.nanosec = 100_000_000
    color.width = color.height = 4
    color.step = 12
    color.encoding = 'bgr8'
    color.data = color_array.tobytes()
    depth = Image()
    depth.header.stamp.sec = 10
    depth.header.stamp.nanosec = 100_000_000
    depth.width = depth.height = 4
    depth.step = 16
    depth.encoding = '32FC1'
    depth.data = depth_array.tobytes()

    node.color_callback(color)
    now[0] = 100.13
    node.depth_callback(depth)
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].red_pixel_count == 4
    assert observations[0].rejection_reason == ''
    assert observations[0].rgb_depth_acquisition_skew == pytest.approx(0.0)
    assert observations[0].geometry_diagnostics_enabled
    assert observations[0].mask_centroid_u == pytest.approx(1.5)
    assert observations[0].mask_centroid_v == pytest.approx(1.5)
    assert observations[0].mask_bbox_left == 1
    assert observations[0].mask_bbox_right == 2
    assert observations[0].valid_depth_count == 4
    assert observations[0].depth_median == pytest.approx(5.0)
    assert observations[0].center_camera_x == pytest.approx(5.0)
    assert observations[0].interpolated_uav_z == pytest.approx(-5.0)
    assert len(positions) == 1


def _make_synthetic_localizer(now):
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    node._ros_seconds = lambda: now[0]
    node.maximum_rgb_depth_skew = 0.02
    node.data_timeout = 0.5
    node.image_pair_buffer = RgbDepthPairBuffer(0.02, 8)
    node.pending_image_pairs = __import__('collections').deque(maxlen=8)
    node.waiting_image_pair = None
    node.pose_wait_timeout = 0.15
    node.maximum_pose_wait_gap = 0.15
    node.image_clock_wait_timeout = 0.15
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(99.9, 99.9, 99.91, 1.0)
    node.image_clock_mapper.add_anchor(100.7, 100.7, 100.71, 1.8)
    node.px4_clock_mapper = Px4RosClockMapper(0.05, calibration_samples=4)
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(
        1.0,
        interpolator=quaternion_slerp,
    )
    node.color_time = node.depth_time = -math.inf
    node.last_pair_rejection = ''
    node.last_image_timeout_report = -math.inf
    node.last_time_diagnostic = {}
    node.frame_id = 'local_ned'
    node.horizontal_fov = math.pi / 2.0
    node.minimum_red_pixels = 3
    node.minimum_depth = 0.2
    node.maximum_depth = 25.0
    node.minimum_depth_ratio = 0.5
    node.maximum_depth_mad = 0.25
    node.camera_translation_flu = (0.0, 0.0, 0.0)
    node.camera_pitch_down = 0.0
    node.target_radius = 0.0
    node.target_reference_z_offset = 0.0
    node.base_position_std = 0.08
    node.range_position_std_scale = 0.01
    node.position_source_stamp = math.nan
    node.position_mapped_stamp = math.nan
    node.attitude_source_stamp = math.nan
    node.attitude_mapped_stamp = math.nan
    observations = []
    node.observation_pub = SimpleNamespace(
        publish=lambda message: observations.append(message)
    )
    node.target_position_pub = SimpleNamespace(publish=lambda _message: None)
    return node, observations


def _rgbd_messages(stamp):
    color_array = np.zeros((4, 4, 3), dtype=np.uint8)
    color_array[1:3, 1:3, 2] = 255
    depth_array = np.full((4, 4), 5.0, dtype='<f4')
    seconds = int(stamp)
    nanoseconds = round((stamp - seconds) * 1e9)
    color = Image()
    color.header.stamp.sec = seconds
    color.header.stamp.nanosec = nanoseconds
    color.width = color.height = 4
    color.step = 12
    color.encoding = 'bgr8'
    color.data = color_array.tobytes()
    depth = Image()
    depth.header.stamp.sec = seconds
    depth.header.stamp.nanosec = nanoseconds
    depth.width = depth.height = 4
    depth.step = 16
    depth.encoding = '32FC1'
    depth.data = depth_array.tobytes()
    return color, depth


def _enqueue_rgbd(node, now, stamp, color_delay, depth_delay):
    color, depth = _rgbd_messages(stamp)
    now[0] = stamp + color_delay
    node.color_callback(color)
    now[0] = stamp + depth_delay
    node.depth_callback(depth)


def test_large_depth_mad_rejects_observation_with_quality_evidence():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    positions = []
    node.target_position_pub = SimpleNamespace(
        publish=lambda message: positions.append(message)
    )
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.5, (0.0, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.5, (1.0, 0.0, 0.0, 0.0))
    color, depth = _rgbd_messages(100.10)
    depth_array = np.full((4, 4), 5.0, dtype='<f4')
    depth_array[1:3, 1:3] = np.array(((4.0, 4.0), (5.0, 5.0)))
    depth.data = depth_array.tobytes()
    now[0] = 100.12
    node.color_callback(color)
    now[0] = 100.13
    node.depth_callback(depth)
    node.localize()

    assert len(observations) == 1
    message = observations[0]
    assert not message.valid
    assert message.rejection_reason == 'DEPTH_MAD_HIGH'
    assert message.confidence == 0.0
    assert math.isnan(message.position.x)
    assert message.red_pixel_count == 4
    assert message.valid_depth_count == 4
    assert message.valid_depth_ratio == pytest.approx(1.0)
    assert message.depth_median == pytest.approx(4.5)
    assert message.depth_mad == pytest.approx(0.5)
    assert message.geometry_diagnostics_enabled
    assert (message.mask_bbox_left, message.mask_bbox_top,
            message.mask_bbox_right, message.mask_bbox_bottom) == (
                1, 1, 2, 2,
            )
    assert (message.mask_centroid_u, message.mask_centroid_v) == (
        1.5, 1.5,
    )
    assert (message.projection_centroid_u,
            message.projection_centroid_v) == (1.5, 1.5)
    assert message.camera_fx > 0.0
    assert message.camera_fy > 0.0
    assert all(math.isfinite(value) for value in (
        message.surface_camera_x, message.surface_camera_y,
        message.surface_camera_z, message.center_camera_x,
        message.center_camera_y, message.center_camera_z,
    ))
    assert message.camera_translation_x == pytest.approx(0.0)
    assert message.camera_pitch_down == pytest.approx(0.0)
    assert message.target_reference_z_offset == pytest.approx(0.0)
    assert all(math.isnan(value) for value in message.covariance)
    assert not positions


def test_online_node_rejects_sparse_sphere_instead_of_publishing_fallback():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.target_radius = .25
    positions = []
    node.target_position_pub = SimpleNamespace(publish=positions.append)
    for stamp in (100., 100.5):
        node.position_history.add(stamp, (0., 0., -5.))
        node.attitude_history.add(stamp, (1., 0., 0., 0.))
    _enqueue_rgbd(node, now, 100.1, .02, .03)
    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == 'TARGET_VECTOR_INVALID'
    assert not positions


def test_valid_camera_center_is_available_without_optional_geometry_diagnostics():
    """Yaw feedback needs the measured camera ray in the normal launch too."""
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.geometry_diagnostics_enabled = False
    for stamp in (100., 100.5):
        node.position_history.add(stamp, (0., 0., -5.))
        node.attitude_history.add(stamp, (1., 0., 0., 0.))
    _enqueue_rgbd(node, now, 100.1, .02, .03)
    node.localize()
    message = observations[-1]
    assert message.valid
    assert not message.geometry_diagnostics_enabled
    assert (message.center_camera_x, message.center_camera_y,
            message.center_camera_z) == pytest.approx((5., 1.25, 1.25))


def test_localizer_processes_latest_complete_pair_instead_of_fifo_backlog():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.5, (0.0, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.5, (1.0, 0.0, 0.0, 0.0))
    for stamp in (100.10, 100.15, 100.20):
        _enqueue_rgbd(node, now, stamp, 0.02, 0.03)

    now[0] = 100.24
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].stamp.sec == 100
    assert observations[0].stamp.nanosec == 200_000_000
    assert not node.pending_image_pairs


def test_continuous_rgbd_batches_produce_monotonic_valid_observations():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.6, (0.6, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.6, (1.0, 0.0, 0.0, 0.0))
    for first, second, delays in (
        (100.10, 100.15, (0.02, 0.04)),
        (100.20, 100.25, (0.05, 0.03)),
        (100.30, 100.35, (0.01, 0.06)),
    ):
        _enqueue_rgbd(node, now, first, *delays)
        _enqueue_rgbd(node, now, second, *delays)
        now[0] = second + max(delays) + 0.01
        node.localize()

    stamps = [
        item.stamp.sec + item.stamp.nanosec * 1e-9
        for item in observations
    ]
    assert all(item.valid for item in observations)
    assert stamps == pytest.approx((100.15, 100.25, 100.35))


@pytest.mark.parametrize(
    'position_range, attitude_range, expected',
    (
        ((100.0, 100.2), (100.2, 100.4),
         'ATTITUDE_TIMESTAMP_BEFORE_HISTORY'),
        ((100.2, 100.4), (100.0, 100.2),
         'POSITION_TIMESTAMP_BEFORE_HISTORY'),
    ),
)
def test_localizer_reports_which_pose_history_misses_image_time(
    position_range,
    attitude_range,
    expected,
):
    now = [100.13]
    node, observations = _make_synthetic_localizer(now)
    for stamp in position_range:
        node.position_history.add(stamp, (0.0, 0.0, -5.0))
    for stamp in attitude_range:
        node.attitude_history.add(stamp, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == expected


def test_image_waits_for_slightly_late_pose_then_localizes():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.00, (0.0, 0.0, -5.0))
    node.position_history.add(100.08, (0.08, 0.0, -5.0))
    node.attitude_history.add(100.00, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.08, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert observations == []
    assert node.waiting_image_pair is not None

    node.position_history.add(100.12, (0.12, 0.0, -5.0))
    node.attitude_history.add(100.12, (1.0, 0.0, 0.0, 0.0))
    now[0] = 100.16
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].stamp.sec == 100
    assert observations[0].stamp.nanosec == 100_000_000
    assert node.waiting_image_pair is None


def test_image_does_not_wait_for_multi_second_pose_mismatch():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(95.00, (0.0, 0.0, -5.0))
    node.position_history.add(95.10, (0.1, 0.0, -5.0))
    node.attitude_history.add(95.00, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(95.10, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == (
        'POSITION_TIMESTAMP_AFTER_HISTORY'
    )
    assert node.waiting_image_pair is None


def test_image_wait_timeout_reports_original_pose_rejection():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.00, (0.0, 0.0, -5.0))
    node.position_history.add(100.08, (0.08, 0.0, -5.0))
    node.attitude_history.add(100.00, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.08, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)
    node.localize()
    assert observations == []

    now[0] = 100.29
    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == (
        'POSITION_TIMESTAMP_AFTER_HISTORY'
    )
    assert node.waiting_image_pair is None


def test_px4_callbacks_cache_pose_in_mapped_measurement_time():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    _send_pose(node, 'position', 50.0)
    _send_pose(node, 'attitude', 50.0)
    now[0] = 100.48
    _send_pose(node, 'position', 50.2, position=(2.0, 4.0, -5.0))
    _send_pose(node, 'attitude', 50.2)

    assert node.position_history.value_at(100.1) == pytest.approx((1.0, 2.0, -5.0))
    assert node.attitude_history.value_at(100.1) == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert node.position_raw_sim_stamp == pytest.approx(10.2)
    assert node.position_mapped_stamp == pytest.approx(100.2)
    assert node.uav_position_time != now[0]


def test_px4_callbacks_use_timestamp_sample_before_causal_timesync_mapping():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    position = VehicleLocalPosition()
    position.timestamp = 50_200_000
    position.timestamp_sample = 50_100_000
    position.x, position.y, position.z = 1.0, 2.0, -5.0
    position.xy_valid = position.z_valid = True
    attitude = VehicleAttitude()
    attitude.timestamp = 50_210_000
    attitude.timestamp_sample = 50_110_000
    attitude.q = [1.0, 0.0, 0.0, 0.0]

    node.position_callback(position)
    node.attitude_callback(attitude)

    assert node.position_source_stamp == pytest.approx(50.1)
    assert node.attitude_source_stamp == pytest.approx(50.11)
    assert node.position_raw_sim_stamp == pytest.approx(10.1)
    assert node.attitude_raw_sim_stamp == pytest.approx(10.11)
    assert node.position_history.time_range == pytest.approx((100.1, 100.1))
    assert node.attitude_history.time_range == pytest.approx((100.11, 100.11))
    assert node.position_mapped_stamp == pytest.approx(100.1)
    assert node.attitude_mapped_stamp == pytest.approx(100.11)


def test_attitude_cache_normalizes_antipodal_quaternions():
    now = [100.42]
    node = _make_causal_pose_localizer(now)
    _send_pose(node, 'attitude', 50.0)
    for source in (50.05, 50.1, 50.2):
        _send_pose(node, 'attitude', source, quaternion=(-1.0, 0.0, 0.0, 0.0))

    assert node.attitude_history.value_at(100.075) == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert node.uav_attitude == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_image_waits_for_next_clock_anchor_then_localizes():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(100.00, 100.00, 100.01, 1.0)
    node.image_clock_mapper.add_anchor(100.08, 100.08, 100.09, 1.08)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.2, (0.0, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.2, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert observations == []
    assert node.waiting_image_pair is not None

    node.image_clock_mapper.add_anchor(100.20, 100.20, 100.21, 1.2)
    now[0] = 100.14
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid


def test_missing_clock_reference_times_out_without_fabricating_stamp():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    _enqueue_rgbd(node, now, 10.10, 90.02, 90.03)

    node.localize()
    assert observations == []

    now[0] = 100.31
    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == (
        'IMAGE_CLOCK_REFERENCE_UNAVAILABLE'
    )
    assert observations[0].stamp.sec == 0


def test_mapped_image_in_future_has_specific_rejection_reason():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(10.0, 100.5, 100.51, 1.0)
    node.image_clock_mapper.add_anchor(10.2, 100.7, 100.71, 1.2)
    color, depth = _rgbd_messages(10.1)
    now[0] = 100.10
    node.color_callback(color)
    now[0] = 100.11
    node.depth_callback(depth)
    now[0] = 100.12

    node.localize()

    assert observations[-1].rejection_reason == 'IMAGE_TIMESTAMP_IN_FUTURE'


def test_old_mapped_image_has_specific_stale_rejection_reason():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(10.0, 99.0, 99.01, 1.0)
    node.image_clock_mapper.add_anchor(11.0, 100.0, 100.01, 2.0)
    color, depth = _rgbd_messages(10.1)
    now[0] = 100.02
    node.color_callback(color)
    now[0] = 100.03
    node.depth_callback(depth)
    now[0] = 100.04

    node.localize()

    assert observations[-1].rejection_reason == 'IMAGE_TIMESTAMP_STALE'

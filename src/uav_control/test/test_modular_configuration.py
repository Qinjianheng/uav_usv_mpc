from pathlib import Path

import pytest
import yaml


CONFIG_FILE = (
    Path(__file__).parents[2]
    / 'uav_usv_bringup'
    / 'config'
    / 'baseline.yaml'
)


def parameters(config, node):
    return config[node]['ros__parameters']


def test_online_small_matrix_nodes_start_with_one_blas_thread(monkeypatch, tmp_path):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import SetEnvironmentVariable
    from launch_ros.actions import Node

    path = CONFIG_FILE.parent.parent / 'launch/modular_intercept.launch.py'
    spec = importlib.util.spec_from_file_location('modular_runtime_limits', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    context = LaunchContext()
    context.environment['OPENBLAS_NUM_THREADS'] = '12'
    context.environment['OMP_NUM_THREADS'] = '12'
    for action in module.generate_launch_description().entities:
        if isinstance(action, Node):
            break
        if isinstance(action, SetEnvironmentVariable):
            action.execute(context)
    assert context.environment['OPENBLAS_NUM_THREADS'] == '1'
    assert context.environment['OMP_NUM_THREADS'] == '1'


def test_shared_dynamic_and_safety_limits_are_synchronized():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert planner['maximum_horizontal_speed'] == pytest.approx(7.0)
    assert tracker['maximum_horizontal_speed'] == pytest.approx(6.5)
    assert tracker['guidance_maximum_horizontal_speed'] == pytest.approx(6.2)
    assert (
        tracker['maximum_horizontal_speed']
        < planner['maximum_horizontal_speed']
    )

    for name in (
        'maximum_vertical_speed',
        'maximum_horizontal_acceleration',
        'maximum_vertical_acceleration',
    ):
        assert tracker[name] == pytest.approx(planner[name])
    assert tracker['sea_surface_z'] == pytest.approx(planner['sea_surface_z'])
    assert evaluator['sea_surface_z'] == pytest.approx(planner['sea_surface_z'])
    assert evaluator['evaluation_capture_radius'] == pytest.approx(0.50)
    assert planner['planned_capture_radius'] == pytest.approx(0.50)
    assert evaluator['planned_capture_radius'] == pytest.approx(
        planner['planned_capture_radius']
    )


def test_planned_contact_point_stays_above_the_body_contact_threshold():
    """
    Guard the invariant that produced the 2026-09-20 sea-contact failures.

    The planner aims at ``sea_surface_z - preferred_clearance`` while the
    evaluator fails the run as soon as the airframe reaches the water, which
    happens ``body_lower_extent`` below the PX4 reference.  If the planned
    endpoint sits at or below that threshold, flying the plan to completion
    always ends in SEA_CONTACT before capture.
    """
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    planned_contact_z = planner['sea_surface_z'] - planner['preferred_clearance']
    body_contact_z = evaluator['sea_surface_z'] - evaluator['body_lower_extent']

    assert planned_contact_z < body_contact_z
    # The validated ceiling must sit on the hard floor, strictly below the
    # contact altitude, or the planned endpoint rests exactly on the validated
    # boundary and millimetre sag is reported as a clearance violation.
    assert planner['contact_clearance'] < planner['preferred_clearance']
    # The aim sphere must still contain a watered-off contact at the largest
    # downward target heave.
    heave = parameters(config, 'moving_target')[
        'vertical_oscillation_amplitude'
    ]
    assert (
        planner['preferred_clearance'] + heave
        <= planner['planned_capture_radius'] + 1e-9
    )


def test_tracker_owns_position_feedback_with_continuous_replanning():
    """Consume bounded tracker velocity without adding PX4 position feedback."""
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    tracker = parameters(config, 'trajectory_tracker_node')

    assert tracker['use_velocity_control'] is True


def test_terminal_budget_uses_short_prediction_after_lower_preparation():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    assert planner['maximum_duration'] <= 1.5
    # Rest-to-rest quintic descent: max acceleration = 10/sqrt(3) * dz/T^2.
    # The former 1.5 m preparation needed the entire vertical budget, leaving
    # no room for the measured pose/velocity at a rolling handover.
    import math
    descent = tracker['approach_preparation_clearance'] - planner['preferred_clearance']
    peak = 10. / math.sqrt(3.) * descent / planner['maximum_duration'] ** 2
    assert peak < .8 * planner['maximum_vertical_acceleration']
    assert tracker['approach_preparation_clearance'] >= 1.0
    assert tracker['approach_closing_speed'] == (
        planner['approach_preparation_standoff_speed']
    )


def test_y_preparation_has_a_feasible_short_plan_without_slow_closing():
    """A 2 m standoff must admit a 1.5 s interception at 4 m/s target speed."""
    from uav_control.guidance.fast_minco_planner import FastMincoPlanner

    config = yaml.safe_load(CONFIG_FILE.read_text())
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    constructor = {name: planner[name] for name in (
        'minimum_duration', 'maximum_duration', 'duration_margin', 'sample_step',
        'maximum_horizontal_speed', 'maximum_vertical_speed',
        'maximum_horizontal_acceleration', 'maximum_vertical_acceleration',
        'preferred_closing_speed', 'conservative_closing_speed', 'sea_surface_z',
        'contact_clearance', 'preferred_clearance', 'target_curve_weight',
    )}
    constructor.update(capture_radius=planner['planned_capture_radius'],
                       piece_count=3, deadline_seconds=.08)
    solver = FastMincoPlanner(**constructor)
    outcome = solver.plan(
        initial_position=(0., 0., -tracker['approach_preparation_clearance']),
        initial_velocity=(4., 0., 0.), initial_acceleration=(0., 0., 0.),
        target_state_at_time=lambda t: (
            (2. + 4. * t, 0., 0.), (4., 0., 0.), (0., 0., 0.),
        ),
    )
    assert outcome.plan is not None
    assert outcome.plan.duration <= 1.5
    assert outcome.plan.closing_speed >= 1.2
    assert outcome.plan.maximum_horizontal_speed <= tracker['maximum_horizontal_speed']


def test_front_camera_preserves_close_contact_surfaces():
    import xml.etree.ElementTree as ET

    model = CONFIG_FILE.parent.parent / 'models/x500_mono_cam/model.sdf'
    sensor = ET.parse(model).find(".//sensor[@name='front_tof_camera']/camera")
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    localizer = parameters(config, 'rgbd_target_localizer')
    monitor = parameters(config, 'front_tof_monitor')
    rgb_near = float(sensor.find('clip/near').text)
    depth_near = float(sensor.find('depth_camera/clip/near').text)
    # A 0.25 m validation sphere at 0.35 m camera range has a 0.10 m
    # near surface. The old 0.20 m clip removed that surface during Y.
    assert 0 < rgb_near <= .05
    assert depth_near == pytest.approx(rgb_near)
    assert localizer['minimum_depth'] <= .05
    assert monitor['minimum_depth'] <= .05


@pytest.mark.parametrize('name', [
    'baseline.yaml', 'visual_geometry_diagnostics.yaml', 'follow_low_dynamic_diagnostics.yaml',
    'p7_guidance.yaml',
])
def test_camera_calibration_projects_the_physical_optical_axis_to_image_center(name, tmp_path):
    """Each prepared physical mount must match that session's source calibration."""
    import math
    import importlib.util
    import xml.etree.ElementTree as ET
    from uav_control.perception.rgbd_target_localizer import local_ned_target_to_camera_flu

    model = CONFIG_FILE.parent.parent / 'models/x500_mono_cam/model.sdf'
    config = yaml.safe_load((CONFIG_FILE.parent / name).read_text())
    localizer = parameters(config, 'rgbd_target_localizer')
    script = CONFIG_FILE.parents[3]/'scripts/prepare_gz_camera_model.py'
    spec = importlib.util.spec_from_file_location('configuration_camera', script)
    preparer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preparer)
    preparer.prepare(model.parent, tmp_path/'session_camera',
                     front_camera_pitch_down=localizer['camera_pitch_down'])
    mount = ET.parse(tmp_path/'session_camera/model.sdf').find(
        ".//link[@name='front_camera_link']/pose")
    physical_pose = tuple(float(value) for value in mount.text.split())
    physical_pitch = physical_pose[4]
    translation = tuple(localizer['camera_translation_' + a] for a in 'xyz')
    # The merged x500_base places base_link 0.24 m above the PX4 model origin.
    physical_translation = (*physical_pose[:2], physical_pose[2] + .24)
    assert translation == pytest.approx(physical_translation)
    # A point 3 m along the physical camera's forward axis, in level-body NED.
    target = (translation[0] + 3. * math.cos(physical_pitch),
              -translation[1], -translation[2] + 3. * math.sin(physical_pitch))
    ray = local_ned_target_to_camera_flu(
        target, (0., 0., 0.), (1., 0., 0., 0.),
        translation, localizer['camera_pitch_down'])
    assert ray == pytest.approx((3., 0., 0.), abs=1e-10)
    assert parameters(config, 'front_tof_monitor')['camera_pitch_down'] == physical_pitch
    if 'target_bearing_node' in config:
        assert parameters(config, 'target_bearing_node')['camera_pitch_down'] == physical_pitch


def test_front_camera_stays_outside_marker_at_terminal_capture():
    import math
    import xml.etree.ElementTree as ET
    from uav_control.perception.rgbd_target_localizer import local_ned_target_to_camera_flu

    model = CONFIG_FILE.parent.parent / 'models/x500_mono_cam/model.sdf'
    pose = tuple(float(value) for value in ET.parse(model).find(
        ".//link[@name='front_camera_link']/pose").text.split())
    translation = (*pose[:2], pose[2] + .24)
    # Representative physical contact: UAV reference is 0.30 m behind and
    # 0.39 m above USV reference; rendered marker center is 0.42 m above it.
    ray = local_ned_target_to_camera_flu(
        (0., 0., -.42), (-.30, 0., -.39), (1., 0., 0., 0.), translation, pose[4])
    distance = math.sqrt(sum(value * value for value in ray))
    assert distance > .25 + .05  # marker radius plus the RGB/depth near clip
    # A partially visible sphere still provides its geometric center. Its
    # silhouette must intersect the vertical FOV, even at the contact edge.
    vertical_half_fov = math.atan(240. / (320. / math.tan(1.74 / 2.)))
    assert abs(math.atan2(ray[2], ray[0])) < vertical_half_fov + math.asin(.25 / distance)
    # Forward/downward center rays clear the central body envelope.
    assert pose[0] > .35355339059327373 / 2.
    assert pose[2] - .02 > .055


def test_sea_barrier_reserve_covers_the_airframe():
    """Guard the barrier that reported SAFE while the gear was at the water."""
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    tracker = parameters(config, 'trajectory_tracker_node')
    planner = parameters(config, 'intercept_planner_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    reserve = tracker['reserve_clearance']
    body = evaluator['body_lower_extent']

    # The planned contact must stay reachable, so the barrier sits below it.
    assert reserve < planner['preferred_clearance']
    # A token reserve far below the airframe extent lets the vehicle fly with
    # its gear level with the water while the guard still reports SAFE.
    assert reserve >= 0.6 * body


def test_curve_weight_stays_inside_the_lateral_acceleration_budget():
    """Guard the 2026-09-20 jittering first-descent regression."""
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')

    # The figure-eight target turns hard enough that blending the MINCO
    # waypoints toward its curved path breaks the 3.0 m/s^2 planning limit
    # from about 0.2 upward, which rejected every terminal plan while the
    # vehicle hovered near the sea.
    assert planner['target_curve_weight'] <= 0.1


def test_shadow_perception_rates_are_below_control_rate():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    tracker = parameters(config, 'trajectory_tracker_node')
    localizer = parameters(config, 'rgbd_target_localizer')
    front = parameters(config, 'front_tof_monitor')

    assert tracker['control_rate_hz'] == pytest.approx(20.0)
    assert localizer['localization_rate_hz'] == pytest.approx(10.0)
    assert front['analysis_rate_hz'] == pytest.approx(5.0)


def test_terminal_planning_uses_fast_rate_and_short_freeze_window():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')

    assert planner['planning_rate_hz'] == pytest.approx(5.0)
    assert planner['terminal_planning_rate_hz'] == pytest.approx(10.0)
    assert planner['minimum_duration'] == pytest.approx(1.0)
    assert planner['terminal_minimum_duration'] == pytest.approx(0.30)
    assert planner['terminal_freeze_time'] == pytest.approx(0.30)


def test_approach_preparation_uses_existing_horizon_and_safety_limits():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert tracker['approach_horizon'] >= planner['maximum_duration']
    assert tracker['approach_contact_clearance'] == pytest.approx(
        planner['preferred_clearance']
    )
    assert tracker['approach_closing_speed'] == pytest.approx(
        planner['approach_preparation_standoff_speed']
    )
    assert planner['approach_reserve_clearance'] == pytest.approx(
        tracker['reserve_clearance']
    )
    assert evaluator['detailed_diagnostics_enabled'] is False


def test_plan_age_bound_matches_four_mps_endpoint_tolerance():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    target = parameters(config, 'moving_target')
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')

    assert target['horizontal_speed'] == pytest.approx(4.0)
    derived_maximum_age = (
        planner['endpoint_tolerance'] / target['horizontal_speed']
    )
    assert planner['maximum_input_age'] <= derived_maximum_age
    assert tracker['maximum_plan_age'] <= derived_maximum_age


def test_imm_uses_raw_observation_and_tracker_uses_kf_truth_is_evaluation_only():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    predictor = parameters(config, 'imm_target_predictor_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert predictor['observation_topic'] == parameters(
        config, 'rgbd_target_localizer')['observation_topic']
    assert 'simulation_truth_topic' not in predictor
    assert 'target_state_source' not in predictor
    assert 'tracking_topic' not in predictor
    assert tracker['target_state_topic'] == parameters(
        config, 'target_kalman_filter')['state_topic']
    assert evaluator['truth_topic'] == '/target/state'
    assert evaluator['truth_role'] == 'evaluation_only'


def test_prediction_horizon_covers_vertical_intercept_duration():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    predictor = parameters(config, 'imm_target_predictor_node')
    planner = parameters(config, 'intercept_planner_node')

    # Y preparation handles the long descent; terminal prediction stays short.
    assert planner['maximum_duration'] <= 1.5
    assert predictor['prediction_horizon'] >= planner['maximum_duration']


def test_imm_diagnostics_and_kf_state_are_separate_from_main_prediction():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))

    control = parameters(config, 'imm_target_predictor_node')
    kalman = parameters(config, 'target_kalman_filter')
    localizer = parameters(config, 'rgbd_target_localizer')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert control['observation_topic'] == localizer['observation_topic']
    assert control['prediction_topic'] == '/planning/target_prediction'
    assert control['diagnostics_topic'] == '/planning/imm_diagnostics'
    assert control['diagnostics_topic'] != control['prediction_topic']
    assert kalman['state_topic'] != control['prediction_topic']
    assert kalman['input_topic'] == localizer['observation_topic']
    assert evaluator['shadow_prediction_topic'] != control['prediction_topic']
    assert 'target_predictor_node' not in config
    assert 'shadow_target_predictor_node' not in config


@pytest.mark.parametrize('name', (
    'baseline.yaml', 'visual_geometry_diagnostics.yaml',
    'follow_low_dynamic_diagnostics.yaml', 'p6_options.yaml',
))
def test_runtime_configs_materialize_unchanged_imm_defaults_and_primary_topic(name):
    from uav_control.tracking.imm_target_predictor_node import node_parameter_defaults
    config = yaml.safe_load((CONFIG_FILE.parent / name).read_text())
    assert 'target_predictor_node' not in config
    assert 'shadow_target_predictor_node' not in config
    defaults = node_parameter_defaults()
    defaults['prediction_topic'] = '/planning/target_prediction'
    assert parameters(config, 'imm_target_predictor_node') == defaults

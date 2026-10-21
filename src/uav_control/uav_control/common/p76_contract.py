"""Versioned P7.6 parameter bindings; one source for explicit candidate behavior."""
from copy import deepcopy
import math
from pathlib import Path

import yaml

from .p76_body_geometry import navigation_camera_translation


def load_contract(path):
    """Read a contract and bind its absolute path without changing the file."""
    path = Path(path).resolve()
    contract = yaml.safe_load(path.read_text())
    if not isinstance(contract, dict) or contract.get('schema_version') not in (1, 2):
        raise ValueError('P76_CONTRACT_SCHEMA')
    contract['_path'] = str(path)
    _validate(contract)
    return contract


def _validate(c):
    if c.get('schema_version') == 2 and (
            c.get('architecture') != 'parallel_dkf_unified_bodyrate'
            or c.get('qualification_schema_version') != 2):
        raise ValueError('P763_ARCHITECTURE_SCHEMA')
    if c.get('schema_version') == 2:
        outer = c['bodyrate_follow']
        limits = dict(kp=2., kv=3., attitude_gain=4., maximum_acceleration=5.,
                      maximum_jerk=3., maximum_tilt=.45, maximum_thrust_slew=2.)
        if (any(type(outer.get(k)) not in (int, float)
                or not math.isfinite(outer[k]) or not 0 < outer[k] <= limit
                for k, limit in limits.items())
                or outer.get('qualification') != 'OFFLINE_CANDIDATE_NOT_FLIGHT_QUALIFIED'):
            raise ValueError('P763_FOLLOW_CANDIDATE_BOUNDS')
    if c['timing']['maximum_source_age'] != .125:
        raise ValueError('P76_SOURCE_AGE')
    if c['timing']['maximum_pose_bracket'] != .05:
        raise ValueError('P76_POSE_BRACKET')
    if (c['candidate']['capture_radius'] != .25
            or 'minimum_origin_height' in c['candidate']
            or c['candidate']['minimum_body_clearance'] != .02):
        raise ValueError('P76_CAPTURE_SEA_BOUNDARY')
    camera, follow = c['camera'], c['follow']
    layouts = dict(revision32=(32., 5., 4.))
    if ((camera['pitch_down_degrees'], follow['distance'], follow['height'])
            != layouts.get(camera['layout']) or camera['translation_flu'] not in (
                [-.25, 0., .55], [.35, 0., .55], [.45, 0., .55], [.55, 0., .55])):
        raise ValueError('P76_UNAPPROVED_LAYOUT')
    if c['motion'] != dict(mode='variable_speed_s', minimum_speed=2., maximum_speed=4.,
                           speed_period=24., startup_duration=8., vertical_amplitude=.15,
                           vertical_frequency=.25):
        raise ValueError('P76_MOTION_CONTRACT')
    if c['detector']['input_size'] not in (416, 512):
        raise ValueError('P76_DETECTOR_RESOLUTION')
    numeric = [v for group in c.values() if isinstance(group, dict)
               for v in group.values() if isinstance(v, (int, float))]
    if not all(math.isfinite(v) for v in numeric):
        raise ValueError('P76_NONFINITE_CONTRACT')


def parameter_bindings(contract):
    """Return every cross-node P76 binding for generation and prelaunch auditing."""
    _validate(contract)
    c = contract
    camera, f, paper, candidate, timing, detector = (
        c[name] for name in ('camera', 'follow', 'paper', 'candidate', 'timing', 'detector'))
    pitch = math.radians(camera['pitch_down_degrees'])
    navigation_translation = navigation_camera_translation(c)
    shared = dict(detection_source='yolo', detection_topic='/perception/target_detection',
                  detection_model_sha256=detector['model_sha256'],
                  camera_pitch_down=pitch, horizontal_fov=camera['horizontal_fov'])
    localizer = dict(shared, **{'camera_translation_'+a: v for a, v in zip(
        'xyz', navigation_translation)}, target_radius=camera['marker_radius'],
        target_reference_z_offset=camera['marker_reference_z_offset'],
        minimum_red_pixels=camera['minimum_pixels'], minimum_depth=camera['depth_clip'][0],
        maximum_depth=camera['depth_clip'][1], minimum_depth_ratio=camera['minimum_depth_ratio'],
        localization_rate_hz=paper['camera_rate_hz'], p76_mode_topic='/control/p76_mode')
    tracker = dict(p76_enabled=True, p76_contract_file=c['_path'],
                   p76_controller_target_rate_hz=paper['paper_controller_target_rate_hz'],
                   p7_guidance_enabled=True, p7_capture_enabled=False,
                   p7_camera_pitch_down=pitch, p7_camera_horizontal_fov=camera['horizontal_fov'],
                   p7_camera_translation_flu=navigation_translation,
                   flight_altitude=-f['height'], follow_distance=f['distance'],
                   maximum_horizontal_speed=candidate['maximum_horizontal_speed'],
                   guidance_maximum_horizontal_speed=candidate['maximum_horizontal_speed'])
    if c.get('schema_version') == 2:
        tracker.update({'p763_follow_'+k: v for k, v in c['bodyrate_follow'].items()})
    for source in (paper, candidate, timing):
        tracker.update({'p76_'+k: v for k, v in source.items()})
    filters = c['filters']
    filter_shared = dict(contract_file=c['_path'], p76_mode_topic='/control/p76_mode',
                         update_rate_hz=paper['dkf_rate_hz'],
                         maximum_source_age=timing['maximum_source_age'],
                         history_seconds=timing['history_seconds'])
    spatial = dict(filter_shared, acceleration_std=filters['spatial_acceleration_std'],
                   initial_velocity_std=filters['spatial_initial_velocity_std'],
                   input_topic='/perception/front/target_observation',
                   state_topic='/tracking/target_state')
    image = dict(filter_shared, acceleration_std=filters['image_acceleration_std'],
                 initial_velocity_std=filters['image_initial_velocity_std'],
                 measurement_std=filters['image_measurement_std'],
                 input_topic='/perception/target_detection', state_topic='/tracking/image_state')
    return {
        'trajectory_tracker_node': tracker,
        'trajectory_impact_sim': dict(flight_altitude=-f['height']),
        'p4_follow_planner_node': dict(p7_camera_pitch_down=pitch,
                                       p7_camera_translation_flu=navigation_translation,
                                       p7_camera_horizontal_fov=camera['horizontal_fov'],
                                       flight_altitude=-f['height'], follow_distance=f['distance'],
                                       p76_mode_topic='/control/p76_mode'),
        'mission_manager_node': dict(p76_enabled=True),
        'rgbd_target_localizer': localizer,
        'target_bearing_node': dict(shared),
        'front_tof_monitor': dict(
            shared, minimum_target_depth_ratio=camera['minimum_depth_ratio']),
        'yolo_target_detector': dict(contract_file=c['_path'], **detector,
                                     input_topic='/camera/front/image_raw',
                                     output_topic='/perception/target_detection',
                                     camera_info_topic='/camera/front/camera_info'),
        'spatial_dkf_node': spatial, 'image_dkf_node': image,
        'target_predictor_node': dict(p76_mode_topic='/control/p76_mode'),
        'imm_target_predictor_node': dict(p76_mode_topic='/control/p76_mode'),
        'intercept_evaluator_node': dict(evaluation_capture_radius=candidate['capture_radius'],
                                         planned_capture_radius=candidate['capture_radius'],
                                         capture_geometry_source='gazebo_native',
                                         sea_surface_z=candidate['sea_surface_z'],
                                         p76_body_geometry_enabled=True,
                                         p76_contract_file=c['_path']),
        'moving_target': dict(trajectory_type=c['motion']['mode'],
                              variable_speed_min=c['motion']['minimum_speed'],
                              variable_speed_max=c['motion']['maximum_speed'],
                              variable_speed_period=c['motion']['speed_period'],
                              variable_startup_duration=c['motion']['startup_duration'],
                              vertical_oscillation_amplitude=c['motion']['vertical_amplitude'],
                              vertical_oscillation_frequency=c['motion']['vertical_frequency']),
    }


def ros_parameters(contract, base_path):
    """Generate the complete opt-in profile without editing any old configuration."""
    profile = deepcopy(yaml.safe_load(Path(base_path).read_text()))
    for node, values in parameter_bindings(contract).items():
        profile.setdefault(node, {}).setdefault('ros__parameters', {}).update(values)
    return profile


def dump_ros_parameters(profile):
    """Serialize complete ROS parameters without YAML aliases unsupported by rcl."""
    class RosDumper(yaml.SafeDumper):
        def ignore_aliases(self, _data):
            return True
    return yaml.dump(profile, Dumper=RosDumper, sort_keys=False)


def validate_profile(contract, profile):
    """Reject any drift in a parameter that participates in the new global contract."""
    for node, values in parameter_bindings(contract).items():
        actual = profile.get(node, {}).get('ros__parameters', {})
        for key, value in values.items():
            if key not in actual or actual[key] != value:
                raise ValueError(f'P76_PARAMETER_MISMATCH:{node}.{key}')

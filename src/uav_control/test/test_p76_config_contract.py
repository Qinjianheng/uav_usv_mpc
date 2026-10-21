"""P7.6 generated global parameters and model-origin geometry contract."""
from copy import deepcopy
import math
from pathlib import Path

import pytest

from uav_control.common.p76_contract import load_contract, ros_parameters, validate_profile


ROOT = Path(__file__).resolve().parents[3]
CONTRACT = ROOT/'src/uav_usv_bringup/config/p76_contract.yaml'


def test_contract_drives_every_effective_sensor_and_controller_stage():
    contract = load_contract(CONTRACT)
    profile = ros_parameters(contract, ROOT/'src/uav_usv_bringup/config/p7_spatial_candidate.yaml')
    validate_profile(contract, profile)
    tracker = profile['trajectory_tracker_node']['ros__parameters']
    planner = profile['p4_follow_planner_node']['ros__parameters']
    assert tracker['p76_enabled'] and tracker['p76_contract_file'] == str(CONTRACT)
    assert tracker['flight_altitude'] == planner['flight_altitude'] == -4.
    assert tracker['follow_distance'] == planner['follow_distance'] == 5.
    assert tracker['p76_capture_vertical_speed_limit'] == 2.
    assert tracker['p76_controller_target_rate_hz'] == 200.
    assert 'p76_minimum_origin_height' not in tracker
    assert tracker['p76_minimum_body_clearance'] == .02
    for node in ('rgbd_target_localizer', 'target_bearing_node', 'front_tof_monitor'):
        params = profile[node]['ros__parameters']
        assert params['detection_source'] == 'yolo'
        assert params['camera_pitch_down'] == pytest.approx(math.radians(32.))
    localizer = profile['rgbd_target_localizer']['ros__parameters']
    assert [localizer['camera_translation_'+a] for a in 'xyz'] == pytest.approx([-.25, 0., .31])
    assert tracker['p7_camera_translation_flu'] == planner['p7_camera_translation_flu']
    assert tracker['p7_camera_translation_flu'] == pytest.approx([-.25, 0., .31])
    assert profile['intercept_evaluator_node']['ros__parameters'][
        'evaluation_capture_radius'] == .25
    assert profile['spatial_dkf_node']['ros__parameters']['maximum_source_age'] == .125
    assert profile['image_dkf_node']['ros__parameters']['maximum_source_age'] == .125


def test_manually_drifted_profile_fails_before_launch():
    contract = load_contract(CONTRACT)
    profile = ros_parameters(contract, ROOT/'src/uav_usv_bringup/config/p7_spatial_candidate.yaml')
    for node, key, wrong in (('rgbd_target_localizer', 'camera_translation_z', .55),
                             ('trajectory_tracker_node', 'p76_capture_vertical_speed_limit', .6),
                             ('p4_follow_planner_node', 'flight_altitude', -5.),
                             ('intercept_evaluator_node', 'evaluation_capture_radius', .5)):
        changed = deepcopy(profile)
        changed[node]['ros__parameters'][key] = wrong
        with pytest.raises(ValueError, match='P76_PARAMETER_MISMATCH'):
            validate_profile(contract, changed)


def test_contract_rejects_authority_extension_and_unapproved_layouts():
    contract = load_contract(CONTRACT)
    contract['timing']['maximum_source_age'] = .2
    with pytest.raises(ValueError, match='P76_SOURCE_AGE'):
        ros_parameters(contract, ROOT/'src/uav_usv_bringup/config/p7_spatial_candidate.yaml')


def test_session_model_uses_model_origin_mount_and_derived_twr_without_editing_sources(tmp_path):
    import hashlib
    import xml.etree.ElementTree as ET
    from uav_control.common.p76_model import prepare_model, verify_model
    px = tmp_path/'px_models'
    for name in ('x500', 'x500_base'):
        (px/name).mkdir(parents=True)
    (px/'x500_base/model.sdf').write_text(
        '<sdf version="1.9"><model name="x500_base"><pose>0 0 0.24 0 0 0</pose>'
        '<link name="base_link"><inertial><mass>2</mass></inertial>'
        '<collision name="body"><geometry><box><size>.3 .3 .05</size></box></geometry>'
        '</collision></link></model></sdf>')
    plugins = ''.join('<plugin name="motor"><motorConstant>0.00001</motorConstant>'
                      '<maxRotVelocity>1000</maxRotVelocity></plugin>' for _ in range(4))
    (px/'x500/model.sdf').write_text('<sdf version="1.9"><model name="x500">'
                                     + plugins+'</model></sdf>')
    source = ROOT/'src/uav_usv_bringup/models/x500_mono_cam'
    before = hashlib.sha256((source/'model.sdf').read_bytes()).hexdigest()
    contract = load_contract(CONTRACT)
    metadata = prepare_model(source, px, tmp_path/'models', contract)
    camera = ET.parse(tmp_path/'models/x500_mono_cam/model.sdf')
    pose = camera.find(".//link[@name='front_camera_link']/pose")
    values = [float(v) for v in pose.text.split()]
    assert values[:3] == pytest.approx((-.25, 0., .31))
    assert values[4] == pytest.approx(math.radians(32.))
    assert metadata['static_model_twr'] == pytest.approx(3.)
    assert verify_model(tmp_path/'models', contract)['consistent']
    assert hashlib.sha256((source/'model.sdf').read_bytes()).hexdigest() == before
    pose.text = '-0.25 0 0.55 0 1.1344640137963142 0'
    camera.write(tmp_path/'models/x500_mono_cam/model.sdf')
    with pytest.raises(ValueError, match='P76_MODEL'):
        verify_model(tmp_path/'models', contract)


def test_follow_planner_and_receiver_load_actual_new_model_origin_extrinsics():
    from types import SimpleNamespace
    from uav_control.control.p7_spatial_guidance import configure_p7_follow_geometry
    from uav_control.controllers.follow_mpc_seed import FollowMpcSeed
    from uav_control.guidance.camera_visibility import CameraExtrinsics
    from uav_control.guidance.follow_limits import ResearchConfig
    values = dict(p7_follow_geometry_enabled=True, p7_camera_pitch_down=math.radians(65.),
                  p7_camera_translation_flu=[-.25, 0., .55], flight_altitude=-4.,
                  follow_distance=5.5)
    node = SimpleNamespace(has_parameter=lambda key: key in values,
                           declare_parameter=lambda key, value: values.setdefault(key, value),
                           get_parameter=lambda key: SimpleNamespace(value=values[key]))
    for model in (FollowMpcSeed(ResearchConfig()), FollowMpcSeed(ResearchConfig())):
        configure_p7_follow_geometry(node, model)
        assert model.config.follow_distance == 5.5
        assert model.extrinsics == CameraExtrinsics.from_sdf_pose(
            (-.25, 0., .55), 0., math.radians(65.), 0.)

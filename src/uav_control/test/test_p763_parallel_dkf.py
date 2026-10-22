"""Estimator activity cannot stand in for actual guidance ownership."""
from collections import deque
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as Row

import numpy as np
import pytest
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from uav_usv_interfaces.msg import TargetImageDetection, TargetObservation

from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_native import NativePoseHistory
from uav_control.common import p76_phase
from uav_control.tracking.delayed_image_kalman import DelayedImageKalman
from uav_control.tracking.delayed_spatial_kalman import DelayedSpatialKalman
from uav_control.tracking.image_dkf_node import ImageDkfNode
from uav_control.tracking.spatial_dkf_node import SpatialDkfNode

ROOT = Path(__file__).resolve().parents[3]


def ros_stamp(value):
    return Time(nanoseconds=round(value*1e9)).to_msg()


def callback_nodes(now=10.05):
    """Use actual callbacks/messages/math; substitute only ROS clock and transport."""
    clock = Row(value=now)
    nodes = []
    for cls in (SpatialDkfNode, ImageDkfNode):
        node = cls.__new__(cls)
        node.get_clock = lambda: Row(now=lambda: Row(nanoseconds=round(clock.value*1e9)))
        node.states, node.acks = [], []
        node.state_pub = Row(publish=node.states.append)
        node.ack_pub = Row(publish=lambda message, node=node:
                           node.acks.append(json.loads(message.data)))
        node.p76_phase, node.p76_epoch, node.p76_y_stamp = 'FOLLOW_ACTIVE', 0, 0.
        node.navigation_frame_fault, node.last_now = '', None
        nodes.append(node)
    spatial, image = nodes
    spatial.filter = DelayedSpatialKalman()
    image.contract = load_contract(ROOT/'src/uav_usv_bringup/config/p76_contract.yaml')
    image.calibration, image.camera_to_body = (320., 240., 300., 300.), np.eye(3)
    image.filter = DelayedImageKalman(image.calibration)
    image.history = NativePoseHistory()
    image.pending, image.cached = deque(maxlen=12), deque(maxlen=12)
    image.seen_detections = deque(maxlen=256)
    image.latest_packet, image.last_reason, image.filter_epoch = None, 'UNINITIALIZED', 0
    image.navigation_stamp = now
    for stamp in (now-.05, now):
        image.history.add(stamp, [1., 0., 0., 0.], [0., 0., 0.], [1e-6]*3)
    return clock, spatial, image


def detection(source=10.02, frame=1, raw=None):
    message = TargetImageDetection()
    message.stamp, message.raw_stamp = ros_stamp(source), ros_stamp(source if raw is None else raw)
    message.detection_stamp = ros_stamp(source+.01)
    message.frame_sequence, message.image_width, message.image_height = frame, 640, 480
    message.center_uv, message.mask_counts = [330., 242.], [0, 3, 640*480-3]
    message.confidence, message.valid, message.class_name = .9, True, 'red_ball'
    message.model_sha256 = '4fb70adc8fc0989eea52831bfbd62550f48cb21cfb39dd07a4f7f68035c65ef8'
    return message


def observation(source=10.02, x=3.):
    message = TargetObservation()
    message.stamp, message.frame_id, message.valid = ros_stamp(source), 'local_ned', True
    message.position.x, message.covariance = x, (np.eye(3)*.01).reshape(-1).tolist()
    return message


def test_follow_runs_both_estimators_but_only_spatial_can_authorize_control():
    _, spatial, image = callback_nodes()
    spatial.receive(observation())
    image.receive(detection())
    assert spatial.states[-1].valid
    assert image.states and image.states[-1].valid
    for node, authorized in ((spatial, True), (image, False)):
        ack = node.acks[-1]
        assert ack['schema_version'] == 2
        assert ack['active'] and ack['ready']
        assert ack['source_stamp'] == pytest.approx(10.02)
        assert ack['control_authorized'] is authorized


@pytest.mark.parametrize('phase', ['Y_HANDOVER', 'PAPER_INTERCEPT_ACTIVE'])
def test_y_and_epoch_relabels_preserve_image_posterior_and_history(phase):
    clock, spatial, image = callback_nodes()
    image.receive(detection())
    image.receive(detection(10.04, 2))
    assert image.states and image.states[-1].valid
    posterior = image.filter.predict(10.05, np.eye(3))
    history_count = len(image.filter.filter.checkpoints)
    native_count = len(image.history.samples)
    for node in (spatial, image):
        node.p76_phase, node.p76_epoch, node.p76_y_stamp = phase, 7, clock.value
    image.mode_changed()
    image.tick()
    after = image.filter.predict(10.05, np.eye(3))
    assert after.state == pytest.approx(posterior.state)
    assert after.covariance == pytest.approx(posterior.covariance)
    assert len(image.filter.filter.checkpoints) == history_count
    assert len(image.history.samples) == native_count
    assert image.states[-1].valid and image.states[-1].mode_epoch == 7
    assert image.acks[-1]['control_authorized']
    spatial.receive(observation(10.04))
    assert spatial.states[-1].valid
    assert spatial.acks[-1]['active'] and spatial.acks[-1]['ready']
    assert not spatial.acks[-1]['control_authorized']
    assert not p76_phase.follow_active(spatial)


def test_stale_sources_keep_estimators_active_but_remove_all_control_authority():
    clock, spatial, image = callback_nodes()
    spatial.receive(observation())
    image.receive(detection())
    clock.value = 10.146
    for node in (spatial, image):
        node.tick()
        assert node.acks[-1]['active']
        assert not node.acks[-1]['ready']
        assert not node.acks[-1]['control_authorized']


def test_duplicate_original_frame_cannot_be_retimed_to_refresh_image_authority():
    _, _, image = callback_nodes()
    image.p76_phase = 'Y_HANDOVER'
    image.receive(detection())
    image.receive(detection(10.04, 1, raw=10.02))
    assert image.filter.filter.source_stamp == pytest.approx(10.02)
    assert len(image.filter.filter.checkpoints) == 1


def test_clock_rollback_clears_both_posteriors_and_all_image_histories():
    clock, spatial, image = callback_nodes()
    spatial.receive(observation())
    image.p76_phase = 'Y_HANDOVER'
    image.receive(detection())
    clock.value = 2.
    for node in (spatial, image):
        node.tick()
        filter_ = node.filter if node is spatial else node.filter.filter
        assert not filter_.predict(2.).valid
        assert not node.acks[-1]['ready']
    assert not image.history.samples and not image.pending and not image.cached
    assert not image.seen_detections and image.latest_packet is None
    assert image.navigation_stamp == 0.
    spatial.receive(observation(1.99))
    assert spatial.states[-1].valid


def test_frame_fault_clears_filters_and_blocks_later_measurements():
    _, spatial, image = callback_nodes()
    spatial.receive(observation())
    image.p76_phase = 'Y_HANDOVER'
    image.receive(detection())
    for node in (spatial, image):
        node.navigation_frame_fault = 'PX4_FRAME_RESET'
        node.frame_failed(node.navigation_frame_fault)
    spatial.receive(observation(10.04))
    image.receive(detection(10.04, 2))
    spatial.tick()
    image.tick()
    assert spatial.filter.source_stamp == 0.
    assert image.filter.filter.source_stamp == 0.
    assert not image.history.samples and not image.pending and not image.cached
    assert all(not node.acks[-1]['active'] for node in (spatial, image))


def test_fast_camera_rotation_does_not_create_target_motion_in_anchor_chart():
    filter_ = DelayedImageKalman((320., 240., 300., 300.))
    for i, angle in enumerate((0., .12, .24, .36)):
        rotation = Rotation.from_euler('y', angle).as_matrix()
        ray = rotation.T@np.array([0., 0., 1.])
        center = [320.+300.*ray[0]/ray[2], 240.]
        assert filter_.update(center, np.eye(2), 10.+i*.02, 10.02+i*.02, rotation)
    predicted = filter_.predict(10.08, rotation, [0., 6., 0.])
    assert predicted.valid
    assert predicted.state[:2] == pytest.approx([300.*np.tan(-.36), 0.], abs=1e-6)
    assert predicted.state[2] == pytest.approx(-1800./np.cos(.36)**2, rel=1e-7)
    assert abs(filter_.filter.predict(10.08).state[2]) < 1e-6


def validation_module():
    spec = importlib.util.spec_from_file_location(
        'p763_validation', ROOT/'scripts/p761_validation.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_b_qualification_requires_both_live_filters_and_exclusive_spatial_authority():
    validation = validation_module()
    diagnostic = Row(stamp=ros_stamp(10.), status='MINCO_FOLLOW', plan_id=3,
                     remaining_time=.2, source_age=.02, last_valid_observation_age=.02)
    target = Row(valid=True, stamp=ros_stamp(10.), source_stamp=ros_stamp(9.98))
    acks = {name: dict(schema_version=2, active=True, ready=True, epoch=0,
                       phase='FOLLOW_ACTIVE', stamp=10., source_stamp=9.98,
                       control_authorized=(name == 'spatial'))
            for name in ('spatial', 'image')}
    mode = dict(phase='FOLLOW_ACTIVE', epoch=0)
    runtime = dict(schema_version=2, phase='FOLLOW_ACTIVE', guidance_owner='MINCO_FOLLOW',
                   px4_interface='BODY_RATE', stamp=10.,
                   output=dict(stamp=10., rates=[0., 0., 0.], thrust=-.5))
    native = dict(stamp=10., flags=dict(
        offboard=True, rates=True, position=False, velocity=False, attitude=False,
        altitude=False, climb_rate=False))
    assert validation.follow_authority_fresh(
        10.02, diagnostic, target, acks, [10.], mode, runtime, native)
    for name, field, value in [('image', 'active', False), ('image', 'ready', False),
                               ('image', 'source_stamp', 9.),
                               ('image', 'control_authorized', True),
                               ('spatial', 'control_authorized', False),
                               ('spatial', 'schema_version', 1)]:
        changed = deepcopy(acks)
        changed[name][field] = value
        assert not validation.follow_authority_fresh(
            10.02, diagnostic, target, changed, [10.], mode, runtime, native)
    for field, value in [('guidance_owner', 'HOLD'), ('stamp', 9.),
                         ('px4_interface', 'VELOCITY'), ('schema_version', 1), ('output', None)]:
        changed = dict(runtime, **{field: value})
        assert not validation.follow_authority_fresh(
            10.02, diagnostic, target, acks, [10.], mode, changed, native)
    changed_modes = (dict(native, stamp=9.),
                     dict(native, flags=dict(native['flags'], velocity=True)))
    for changed in changed_modes:
        assert not validation.follow_authority_fresh(
            10.02, diagnostic, target, acks, [10.], mode, runtime, changed)
    diagnostic.status = 'FOLLOW'
    assert not validation.follow_authority_fresh(
        10.02, diagnostic, target, acks, [10.], mode, runtime, native)


def replay_module():
    spec = importlib.util.spec_from_file_location('p763_replay', ROOT/'scripts/p763_dkf_replay.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_y_observer_checks_image_and_actual_output_without_spatial_target_consumption():
    validation = validation_module()
    acks = {name: dict(schema_version=2, active=True, ready=(name == 'image'), epoch=1,
                       phase='PAPER_INTERCEPT_ACTIVE', stamp=10., source_stamp=9.98,
                       control_authorized=(name == 'image')) for name in ('spatial', 'image')}
    mode = dict(phase='PAPER_INTERCEPT_ACTIVE', epoch=1)
    runtime = dict(schema_version=2, phase='PAPER_INTERCEPT_ACTIVE', guidance_owner='PAPER',
                   px4_interface='BODY_RATE', stamp=10.,
                   output=dict(stamp=10., rates=[0., 0., 0.], thrust=-.5))
    native = dict(stamp=10., flags=dict(
        offboard=True, rates=True, position=False, velocity=False, attitude=False,
        altitude=False, climb_rate=False))
    assert validation.intercept_authority_fresh(10.02, acks, mode, runtime, native)
    for name, field, value in [('spatial', 'control_authorized', True),
                               ('image', 'source_stamp', 9.), ('image', 'ready', False)]:
        changed = deepcopy(acks)
        changed[name][field] = value
        assert not validation.intercept_authority_fresh(10.02, changed, mode, runtime, native)
    assert not validation.intercept_authority_fresh(
        10.02, acks, mode, dict(runtime, guidance_owner='MINCO_FOLLOW'), native)


def test_replay_never_pairs_yolo_and_navigation_from_unrelated_epochs(tmp_path):
    import gzip
    detector, navigation = tmp_path/'detector.jsonl.gz', tmp_path/'navigation.jsonl.gz'
    with gzip.open(detector, 'wt') as stream:
        stream.write(json.dumps(dict(source_stamp=10., receipt=10.02, valid=True))+'\n')
    with gzip.open(navigation, 'wt') as stream:
        for kind in ('pose', 'state'):
            stream.write(json.dumps(dict(kind=kind, stamp=100.))+'\n')
    coverage = replay_module().audit_real_inputs(detector, navigation)
    assert coverage['detector_packets'] == 1
    assert coverage['overlapping_source_epochs'] == 0
    assert not coverage['matched_real_replay_qualified']
    assert 'NO_SYNCHRONIZED_NATIVE_NAVIGATION' in coverage['blockers']


def test_fixture_replay_exposes_legacy_y_gap_without_qualifying_real_data():
    comparison = replay_module().benchmark_fixture(.4)
    old, new = comparison['legacy_exclusive'], comparison['parallel']
    assert not old['image_valid_at_y']
    assert old['spatial_updates_after_y'] == 0
    assert new['image_valid_at_y']
    assert new['image_same_epoch_y_jump_pixels'] == pytest.approx(0., abs=1e-9)
    assert new['spatial_updates_after_y'] > 0
    assert new['illegal_authorizations'] == 0
    assert new['callbacks']['image.tick']['p99_ms'] >= 0.
    assert comparison['evidence_class'] == 'SYNTHETIC_CALLBACK_FIXTURE'
    assert not comparison['tracker_scheduling_qualified']

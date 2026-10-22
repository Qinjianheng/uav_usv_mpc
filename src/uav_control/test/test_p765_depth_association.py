"""Foreground association against independent surfaces and frozen raw ROI bytes."""
import hashlib
import io
import json
from pathlib import Path
import tarfile

import numpy as np
import pytest
import yaml

from synthetic_sphere_fixture import render
from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd

ROOT = Path(__file__).resolve().parents[3]


def fit(mask, depth, enabled=True, **kwargs):
    details = {}
    result = target_geometry_from_rgbd(
        mask, depth, 1.74, .05, 25., .25, require_valid_sphere=True,
        sphere_depth_component_enabled=enabled, diagnostics=details, **kwargs)
    return result, details


@pytest.mark.parametrize('center', [(3., .4, -.1), (8., -1.2, .3)])
def test_clean_analytic_sphere_and_raw_statistics_are_unchanged(center):
    mask, depth, _ = render(center)
    original, _ = fit(mask, depth, enabled=False)
    associated, details = fit(mask, depth)
    assert associated == original
    assert associated.center_camera == pytest.approx(center, abs=2e-5)
    assert details['fit_input_count'] == details['valid_depth_count']
    assert details['excluded_valid_depth_count'] == 0


def test_far_mask_leak_is_removed_without_mutating_inputs_or_raw_statistics():
    mask, depth, _ = render((5., .2, .1))
    mask[210:213, 390:395] = True
    depth[210:213, 390:395] = 18.
    before_mask, before_depth = mask.copy(), depth.copy()
    assert fit(mask, depth, enabled=False)[0] is None
    result, details = fit(mask, depth)
    assert result.center_camera == pytest.approx((5., .2, .1), abs=2e-5)
    assert result.valid_depth_count == np.count_nonzero(mask)
    assert details['valid_depth_count'] == result.valid_depth_count
    assert details['fit_input_count'] == result.valid_depth_count-15
    assert details['excluded_valid_depth_count'] == 15
    assert details['depth_component_count'] == 2
    assert details['association_policy'] == 'nearest_depth_component_diameter_v1'
    assert np.array_equal(mask, before_mask)
    assert np.array_equal(depth, before_depth)


def test_nearer_occluder_and_equal_or_minority_support_are_rejected():
    mask, depth, _ = render((5., 0., 0.))
    mask[200, 300] = True
    depth[200, 300] = 2.
    assert fit(mask, depth)[0] is None
    # Two equally supported depth layers cannot identify one target.
    mask = np.ones((20, 20), bool)
    depth = np.full(mask.shape, 5.)
    depth[:10] = 2.
    _, details = fit(mask, depth)
    assert details['explicit_fit_rejection_reason'] == 'DEPTH_COMPONENT_SUPPORT_LOW_OR_AMBIGUOUS'
    depth[:11] = 5.
    assert fit(mask, depth)[0] is None


def test_invalid_pixels_count_against_original_mask_support():
    mask, depth, _ = render((5., 0., 0.))
    indices = np.flatnonzero(mask)
    depth.flat[indices[::4]] = np.nan
    depth.flat[indices[1::4]] = np.inf
    depth.flat[indices[2::4]] = 26.
    result, details = fit(mask, depth)
    assert result is None
    assert details['fit_input_ratio'] < .5
    assert details['valid_depth_count'] == len(indices[3::4])


def test_component_support_obeys_existing_stricter_depth_ratio():
    mask, depth, _ = render((5., 0., 0.))
    indices = np.flatnonzero(mask)
    depth.flat[indices[::10]] = 18.
    assert fit(mask, depth, minimum_depth_ratio=.95)[0] is None
    assert fit(mask, depth, minimum_depth_ratio=.5)[0] is not None


@pytest.mark.parametrize('surface', ['plane', 'tiny_cap', 'depth_chain'])
def test_non_spherical_or_unobservable_surface_remains_rejected(surface):
    mask, depth, _ = render((5., 0., 0.))
    if surface == 'plane':
        depth[mask] = 4.75
    elif surface == 'tiny_cap':
        mask, depth, _ = render((.5, 0., 0.))
        yy, xx = np.indices(mask.shape)
        mask &= (xx-320)**2+(yy-240)**2 <= 2**2
    else:
        depth[mask] = np.linspace(4., 6., np.count_nonzero(mask))
    result, details = fit(mask, depth)
    assert result is None
    if surface == 'depth_chain':
        assert details['explicit_fit_rejection_reason'] == 'DEPTH_COMPONENT_SPAN_EXCEEDS_DIAMETER'
    elif surface == 'tiny_cap':
        assert details['explicit_fit_rejection_reason'] == 'FIT_ILL_CONDITIONED'


def test_association_requires_known_ideal_sphere_and_valid_support_contract():
    mask, depth, _ = render((5., 0., 0.))
    for kwargs in (dict(sphere_fit_mode='tof'), dict(minimum_depth_ratio=np.nan),
                   dict(minimum_depth_ratio=1.1)):
        result, details = fit(mask, depth, **kwargs)
        assert result is None
        assert details['explicit_fit_rejection_reason'] == 'DEPTH_COMPONENT_CONTRACT_INVALID'


def test_real_frozen_roi_reproduces_original_failure_then_recovers_strict_sphere():
    archive = ROOT/'data/baselines/20261022_p765_unarmed_01/raw_session.tar.gz'
    with tarfile.open(archive) as stream:
        members = sorted(m for m in stream.getnames() if '/roi/roi_' in m and m.endswith('.json'))
        assert len(members) == 10
        for name in members:
            metadata = json.load(stream.extractfile(name))
            payload = stream.extractfile(str(Path(name).parent/metadata['data_file'])).read()
            assert hashlib.sha256(payload).hexdigest() == metadata['data_sha256']
            with np.load(io.BytesIO(payload), allow_pickle=False) as raw:
                mapping = metadata['mask_mapping']
                shape = (mapping['original_height'], mapping['original_width'])
                depth = np.full(shape, np.nan, dtype=np.float32)
                mask = np.zeros(shape, bool)
                left, top, right, bottom = metadata['roi_xyxy_exclusive']
                depth[top:bottom, left:right] = raw['depth']
                mask[top:bottom, left:right] = raw['mask']
            original, old = fit(mask, depth, enabled=False)
            assert original is None
            assert old['explicit_fit_rejection_reason'] == 'FITTED_RADIUS_MISMATCH'
            result, details = fit(mask, depth)
            assert result is not None
            assert result.valid_depth_count == details['valid_depth_count'] == 268
            assert details['fit_input_count'] == 263
            assert details['fit_input_ratio'] == pytest.approx(263/292)
            assert details['excluded_valid_depth_count'] == 5
            assert details['fitted_radius'] == pytest.approx(.24833417847, abs=1e-8)
            assert details['radius_residual'] < .002
            assert details['fit_position_std'] is None  # Ideal fit has no uncertainty model.


def test_only_opt_in_p76_profile_enables_association_and_contract_detects_drift():
    from uav_control.common.p76_contract import load_contract, ros_parameters, validate_profile
    config = ROOT/'src/uav_usv_bringup/config'
    contract = load_contract(config/'p76_contract.yaml')
    profile = ros_parameters(contract, config/'p7_spatial_candidate.yaml')
    params = profile['rgbd_target_localizer']['ros__parameters']
    assert params['sphere_depth_component_enabled'] is True
    saved = yaml.safe_load((config/'p76_yan_paper.yaml').read_text())
    validate_profile(contract, saved)
    for name in ('baseline.yaml', 'p7_spatial_candidate.yaml'):
        legacy = yaml.safe_load((config/name).read_text())
        assert not legacy['rgbd_target_localizer']['ros__parameters'].get(
            'sphere_depth_component_enabled', False)
    params['sphere_depth_component_enabled'] = False
    with pytest.raises(ValueError, match='P76_PARAMETER_MISMATCH'):
        validate_profile(contract, profile)


@pytest.mark.parametrize('enabled', [False, True])
def test_yolo_localizer_uses_association_without_retiming_or_changing_raw_quality(enabled):
    from builtin_interfaces.msg import Time
    from uav_usv_interfaces.msg import TargetImageDetection
    from test_rgbd_target_localizer import _make_synthetic_localizer, _rgbd_messages
    from uav_control.perception.yolo_detection_contract import DetectionCache
    from uav_control.perception.yolo_inference import encode_mask

    now = [100.13]
    node, observations = _make_synthetic_localizer(now)
    node.target_radius, node.horizontal_fov = .25, 1.74
    node.sphere_depth_component_enabled = enabled
    node.geometry_diagnostics_enabled = True
    node.detection_source, node.yolo_cache = 'yolo', DetectionCache('a'*64)
    for stamp in (100., 100.5):
        node.position_history.add(stamp, (0., 0., -5.))
        node.attitude_history.add(stamp, (1., 0., 0., 0.))
    mask, depth, _ = render((5., .2, .1))
    mask[210:213, 390:395] = True
    depth[210:213, 390:395] = 18.
    color_msg, depth_msg = _rgbd_messages(100.1)
    for message in (color_msg, depth_msg):
        message.width, message.height = 640, 480
    color_msg.step, color_msg.data = 1920, np.zeros((480, 640, 3), np.uint8).tobytes()
    depth_msg.step, depth_msg.data = 2560, depth.astype('<f4').tobytes()
    packet = TargetImageDetection()
    packet.raw_stamp = packet.stamp = Time(sec=100, nanosec=100_000_000)
    packet.detection_stamp = Time(sec=100, nanosec=110_000_000)
    packet.image_width, packet.image_height = 640, 480
    packet.model_sha256, packet.class_name = 'a'*64, 'red_ball'
    packet.valid, packet.frame_sequence, packet.confidence = True, 1, .9
    packet.center_uv, packet.mask_counts = [320., 240.], list(encode_mask(mask))
    node.yolo_cache.add(packet)
    node.color_callback(color_msg)
    node.depth_callback(depth_msg)
    node.localize()
    assert len(observations) == 1
    message = observations[0]
    assert message.valid == enabled
    assert message.rgb_raw_stamp == message.depth_raw_stamp == pytest.approx(100.1)
    if enabled:
        assert message.valid_depth_count == np.count_nonzero(mask)
        assert message.valid_depth_ratio == 1.
        assert message.position.x == pytest.approx(5., abs=2e-5)
        assert message.stamp.sec+message.stamp.nanosec*1e-9 == pytest.approx(100.1)
    else:
        assert message.rejection_reason == 'TARGET_VECTOR_INVALID'


def test_offline_comparison_never_relaxes_a_stricter_recorded_contract(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'association_analysis', ROOT/'scripts/p765_depth_association_analysis.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = ROOT/'data/baselines/20261022_p765_unarmed_01/raw_session.tar.gz'
    changed = tmp_path/'strict_archive.tar.gz'
    with tarfile.open(original) as source, tarfile.open(changed, 'w:gz') as target:
        for name in source.getnames():
            if '/roi/roi_' not in name:
                continue
            payload = source.extractfile(name).read()
            if name.endswith('.json'):
                metadata = json.loads(payload)
                metadata['geometry_contract']['minimum_depth_ratio'] = .95
                payload = json.dumps(metadata).encode()
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            target.addfile(info, io.BytesIO(payload))
    result = module.compare(changed)
    assert result['candidate_accepted'] == 0
    assert all(row['candidate_contract']['minimum_depth_ratio'] == .95
               for row in result['samples'])
    assert all(row['candidate']['diagnostics']['explicit_fit_rejection_reason']
               == 'DEPTH_COMPONENT_SUPPORT_LOW_OR_AMBIGUOUS' for row in result['samples'])

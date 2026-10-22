"""Lossless bounded ROI evidence without a controller callback or source retiming."""
import hashlib
import json

import numpy as np
import pytest


def sample(reason='TARGET_VECTOR_INVALID', key=10_000_000_007):
    # Explicit 3x2 ROI in a 4x5 image includes exact NaN and infinity bits.
    rgb = np.arange(60, dtype=np.uint8).reshape(4, 5, 3)
    depth = np.arange(20, dtype=np.float32).reshape(4, 5)
    depth.view(np.uint32)[1, 1] = 0x7fc00013
    depth[2, 2] = np.inf
    mask = np.zeros((4, 5), bool)
    mask[1:3, 1:4] = True
    from uav_control.perception.yolo_inference import encode_mask
    stamp = dict(sec=10, nanosec=7)
    return dict(rgb=rgb, depth=depth, mask=mask, metadata=dict(
        rgb_raw_stamp=stamp, depth_raw_stamp=stamp, detection_raw_stamp=stamp,
        rgb_receipt_ns=20_000_000_000, depth_receipt_ns=20_000_000_010,
        diagnostic_receipt_ns=20_000_000_050, localization_reason=reason,
        detection=dict(mask_counts=list(encode_mask(mask)), image_width=5, image_height=4,
                       raw_stamp=stamp, frame_sequence=1, model_sha256='b'*64),
        geometry=dict(explicit_fit_rejection_reason='FIT_TOO_FEW_POINTS'),
        raw_key_ns=key))


def test_capture_losslessly_roundtrips_original_float_bits_and_coordinate_mapping(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={'test': True},
                              max_samples=4, max_per_stratum=2, max_bytes=100000)
    packet = sample()
    result = writer.capture(**packet)
    assert result['saved']
    data = np.load(writer.output/result['data_file'])
    assert np.array_equal(data['depth'].view(np.uint32),
                          packet['depth'][1:3, 1:4].view(np.uint32))
    assert np.array_equal(data['rgb'], packet['rgb'][1:3, 1:4])
    metadata = json.loads((writer.output/result['metadata_file']).read_text())
    assert metadata['roi_xyxy_exclusive'] == [1, 1, 4, 3]
    assert metadata['mask_mapping'] == dict(order='C', original_width=5, original_height=4,
                                            roi_origin_uv=[1, 1], roi_scale_uv=[1, 1])
    assert metadata['rgb_raw_stamp']['nanosec'] == 7
    assert metadata['depth_raw_stamp'] == metadata['detection_raw_stamp']
    assert metadata['data_sha256'] == hashlib.sha256(
        (writer.output/result['data_file']).read_bytes()).hexdigest()
    assert metadata['depth_dtype'] == '<f4'
    writer.close()
    manifest = json.loads((writer.output/'manifest.json').read_text())
    assert manifest['runtime_identity'] == {'test': True}
    assert manifest['saved_samples'] == 1


def test_capture_has_independent_success_failure_strata_and_hard_budget(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={},
                              max_samples=2, max_per_stratum=1, max_bytes=100000)
    assert writer.capture(**sample())['saved']
    packet = sample(key=10_000_000_008)
    packet['metadata']['rgb_raw_stamp']['nanosec'] = 8
    assert writer.capture(**packet)['reason'] == 'STRATUM_LIMIT'
    assert writer.capture(**sample(reason=''))['saved']
    assert writer.capture(**sample(reason='DEPTH_RATIO_LOW'))['reason'] == 'SAMPLE_LIMIT'
    writer.close()
    events = [json.loads(line) for line in (writer.output/'events.jsonl').read_text().splitlines()]
    assert len(events) == 4
    assert events[0]['metadata']['localization_reason'] == 'TARGET_VECTOR_INVALID'
    assert events[1]['metadata']['localization_reason'] == 'TARGET_VECTOR_INVALID'


def test_capture_refuses_mismatched_raw_identity_and_wrong_depth_dtype(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={}, max_bytes=100000)
    packet = sample()
    packet['metadata']['depth_raw_stamp'] = dict(sec=10, nanosec=8)
    assert writer.capture(**packet)['reason'] == 'RAW_ACQUISITION_MISMATCH'
    packet = sample()
    packet['depth'] = packet['depth'].astype(np.float64)
    assert writer.capture(**packet)['reason'] == 'DEPTH_NOT_FLOAT32'
    packet = sample()
    packet['metadata']['detection']['mask_counts'] = [20]
    assert writer.capture(**packet)['reason'] == 'MASK_RLE_MISMATCH'
    writer.close()
    assert not list(writer.output.glob('*.npz'))


def test_capture_refuses_overwrite_and_byte_exhaustion(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={}, max_bytes=1)
    assert writer.capture(**sample())['reason'] == 'BYTE_LIMIT'
    writer.close()
    with pytest.raises(FileExistsError):
        BoundedRoiWriter(tmp_path/'capture', runtime_identity={})


def test_runtime_identity_hashes_actual_loaded_functions():
    from uav_control.perception.p765_roi_capture import runtime_identity
    identity = runtime_identity()
    assert len(identity['loaded_code']['target_geometry_from_rgbd']['code_sha256']) == 64
    assert len(identity['loaded_code']['fit_tof_sphere']['source_sha256']) == 64
    assert identity['loaded_code']['fit_tof_sphere']['source_path'].endswith(
        'tof_sphere_localization.py')


def test_matcher_waits_for_exact_sources_and_does_not_substitute_a_nearby_mask(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={}, max_bytes=100000)
    matcher = RoiCaptureMatcher(writer, capacity=2, allow_synthetic=True)
    packet = sample()
    meta = packet['metadata']
    matcher.add_frame('rgb', meta['rgb_raw_stamp'], packet['rgb'], dict(receipt_ns=20_000_000_011))
    matcher.add_frame('depth', meta['depth_raw_stamp'], packet['depth'],
                      dict(receipt_ns=20_000_000_012))
    wrong = dict(meta['detection'], raw_stamp=dict(sec=10, nanosec=6))
    matcher.add_detection(wrong, receipt_ns=20_000_000_013)
    matcher.add_diagnostic(dict(rgb_raw_stamp=meta['rgb_raw_stamp'],
                                depth_raw_stamp=meta['depth_raw_stamp'],
                                localization_reason='TARGET_VECTOR_INVALID',
                                rgb_receipt_stamp=20., depth_receipt_stamp=20.000000001,
                                diagnostic_publish_stamp=20.000000005, clock_epoch=0,
                                geometry=meta['geometry'],
                                runtime_identity={'localizer': 'loaded'}),
                           receipt_ns=20_000_000_014)
    assert writer.samples == 0
    matcher.add_detection(meta['detection'], receipt_ns=20_000_000_015)
    assert writer.samples == 0  # Importing infer in the recorder is not detector runtime evidence.
    matcher.add_detector_status(detector_status(meta['rgb_raw_stamp'], frame=2),
                                receipt_ns=20_000_000_016)
    assert writer.samples == 0
    matcher.add_detector_status(detector_status(meta['rgb_raw_stamp']), receipt_ns=20_000_000_017)
    assert writer.samples == 1
    matcher.close()
    recorded = json.loads(next(writer.output.glob('roi_*.json')).read_text())
    assert recorded['detection_receipt_ns'] == 20_000_000_015
    assert recorded['diagnostic_receipt_ns'] == 20_000_000_014
    assert recorded['localizer_diagnostic']['runtime_identity'] == {'localizer': 'loaded'}
    assert recorded['detector_runtime_identity']['pid'] == 12345
    assert recorded['detector_runtime_identity']['synthetic_only'] is True
    assert recorded['detector_status_receipt_ns'] == 20_000_000_017


def test_matcher_bounds_unmatched_frames_and_retains_missing_pair_failure(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer, capacity=2)
    packet = sample()
    for nanosec in (1, 2, 3):
        stamp = dict(sec=10, nanosec=nanosec)
        matcher.add_frame('rgb', stamp, packet['rgb'], dict(receipt_ns=11))
        matcher.add_diagnostic(dict(rgb_raw_stamp=stamp, depth_raw_stamp=stamp,
                                    localization_reason='TARGET_VECTOR_INVALID'), receipt_ns=12)
    assert len(matcher.frames['rgb']) == len(matcher.pending) == 2
    matcher.close()
    rows = [json.loads(line) for line in (writer.output/'events.jsonl').read_text().splitlines()]
    assert len(rows) == 3
    assert rows[0]['reason'] == 'UNMATCHED_DIAGNOSTIC_EVICTED'
    assert all(row['metadata']['localization_reason'] == 'TARGET_VECTOR_INVALID' for row in rows)
    assert rows[-1]['reason'] == 'UNMATCHED_AT_CLOSE'


def test_raw_image_decode_respects_padding_endianness_and_never_quantizes_depth():
    from types import SimpleNamespace
    from uav_control.perception.p765_roi_capture import image_array
    payload = np.array([0x3f800001, 0x7fc00013, 0x00000000,
                        0x7f800000, 0x40000001, 0x00000000], dtype='>u4').tobytes()
    msg = SimpleNamespace(width=2, height=2, step=12, encoding='32FC1',
                          is_bigendian=1, data=payload)
    result = image_array(msg)
    assert result.dtype.str == '>f4'
    assert np.array_equal(result.view('>u4'),
                          [[0x3f800001, 0x7fc00013], [0x7f800000, 0x40000001]])
    msg = SimpleNamespace(width=1, height=2, step=4, encoding='rgb8',
                          is_bigendian=0, data=bytes([1, 2, 3, 99, 4, 5, 6, 99]))
    assert np.array_equal(image_array(msg), [[[1, 2, 3]], [[4, 5, 6]]])
    msg.encoding = '16UC1'
    with pytest.raises(ValueError, match='ROI_IMAGE_ENCODING'):
        image_array(msg)


def test_saved_roi_replay_uses_original_pixel_intrinsics_and_checks_sha(tmp_path):
    from synthetic_sphere_fixture import render
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, replay_roi
    from uav_control.perception.yolo_inference import encode_mask
    mask, depth, _ = render((3., .4, -.1))
    packet = sample(reason='')
    packet['rgb'] = np.zeros((480, 640, 3), dtype=np.uint8)
    packet['depth'], packet['mask'] = depth.astype(np.float32), mask
    packet['metadata']['detection'].update(
        mask_counts=list(encode_mask(mask)), image_width=640, image_height=480)
    packet['metadata']['geometry_contract'] = dict(
        horizontal_fov=1.74, minimum_depth=.05, maximum_depth=25., target_radius=.25,
        require_valid_sphere=True, sphere_fit_mode='ideal', sphere_noise_std=.01,
        sphere_range_noise_scale=.001)
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    result = writer.capture(**packet)
    writer.close()
    replay = replay_roi(writer.output/result['metadata_file'])
    assert replay['valid_sphere']
    assert replay['center_camera_flu'] == pytest.approx((3., .4, -.1), abs=2e-5)
    assert replay['diagnostics']['valid_depth_count'] == int(mask.sum())
    data_path = writer.output/result['data_file']
    data_path.write_bytes(data_path.read_bytes()+b'corruption')
    with pytest.raises(ValueError, match='ROI_DATA_SHA256'):
        replay_roi(writer.output/result['metadata_file'])


def detector_status(stamp, frame=1):
    """Build an explicitly synthetic remote-process identity, never real detector attestation."""
    identity = dict(pid=12345, executable='/synthetic/python', synthetic_only=True,
                    model_sha256='b'*64, loaded_code={
                        name: dict(code_sha256='c'*64, source_sha256='d'*64)
                        for name in ('YoloTargetDetector.process', 'YoloInference.infer',
                                     'decode_segmentation', 'encode_mask')})
    return dict(raw_stamp=stamp, frame=frame, runtime_identity=identity, clock_epoch=0,
                image_receipt=20., selected=20.000000001, completed=20.000000003,
                published=20.000000004)


@pytest.mark.parametrize('bad', [
    'missing_identity', 'wrong_model', 'float_stamp', 'synthetic_live',
])
def test_live_matcher_rejects_unbound_or_unproven_detector_runtime(tmp_path, bad):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer, allow_synthetic=bad != 'synthetic_live')
    packet = sample()
    metadata = packet['metadata']
    matcher.add_frame('rgb', metadata['rgb_raw_stamp'], packet['rgb'],
                      dict(receipt_ns=20_000_000_001))
    matcher.add_frame('depth', metadata['depth_raw_stamp'], packet['depth'],
                      dict(receipt_ns=20_000_000_002))
    matcher.add_detection(metadata['detection'], receipt_ns=20_000_000_003)
    matcher.add_diagnostic(dict(rgb_raw_stamp=metadata['rgb_raw_stamp'],
                                depth_raw_stamp=metadata['depth_raw_stamp'],
                                localization_reason='TARGET_VECTOR_INVALID'), receipt_ns=4)
    status = detector_status(metadata['rgb_raw_stamp'])
    if bad == 'missing_identity':
        del status['runtime_identity']
    elif bad == 'wrong_model':
        status['runtime_identity']['model_sha256'] = 'e'*64
    elif bad == 'float_stamp':
        status['raw_stamp'] = 10.000000007
    matcher.add_detector_status(status, receipt_ns=20_000_000_015)
    assert writer.samples == 0
    matcher.close()
    events = [json.loads(line) for line in (writer.output/'events.jsonl').read_text().splitlines()]
    assert any(row['reason'].startswith('DETECTOR_') for row in events)
    assert any(row['reason'] == 'UNMATCHED_AT_CLOSE' for row in events)


def test_matcher_terminal_event_follows_all_unmatched_failures(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer)
    packet = sample()['metadata']
    matcher.add_diagnostic(dict(rgb_raw_stamp=packet['rgb_raw_stamp'],
                                depth_raw_stamp=packet['depth_raw_stamp'],
                                localization_reason='TARGET_VECTOR_INVALID'), receipt_ns=1)
    matcher.close(terminal_metadata=dict(duration_s=30., failure=''))
    rows = [json.loads(line) for line in (writer.output/'events.jsonl').read_text().splitlines()]
    assert [row['reason'] for row in rows] == ['UNMATCHED_AT_CLOSE', 'SESSION_END']
    assert rows[-1]['metadata']['duration_s'] == 30.


def feed_complete_matcher(matcher, *, status_epoch=0, diagnostic_receipt=20_000_000_014):
    packet = sample()
    metadata = packet['metadata']
    matcher.add_frame('rgb', metadata['rgb_raw_stamp'], packet['rgb'],
                      dict(receipt_ns=20_000_000_011))
    matcher.add_frame('depth', metadata['depth_raw_stamp'], packet['depth'],
                      dict(receipt_ns=20_000_000_012))
    matcher.add_detection(metadata['detection'], receipt_ns=20_000_000_013)
    status = detector_status(metadata['rgb_raw_stamp'])
    status['clock_epoch'] = status_epoch
    matcher.add_detector_status(status, receipt_ns=20_000_000_015)
    diagnostic = dict(rgb_raw_stamp=metadata['rgb_raw_stamp'],
                      depth_raw_stamp=metadata['depth_raw_stamp'],
                      rgb_receipt_stamp=20., depth_receipt_stamp=20.000000001,
                      diagnostic_publish_stamp=20.000000005, clock_epoch=0,
                      localization_reason='TARGET_VECTOR_INVALID')
    matcher.add_diagnostic(diagnostic, receipt_ns=diagnostic_receipt)
    return diagnostic


def test_raw_frame_consumption_is_one_shot_even_with_repeated_diagnostics(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer, allow_synthetic=True)
    diagnostic = feed_complete_matcher(matcher)
    assert writer.samples == 1
    matcher.add_diagnostic(diagnostic, receipt_ns=20_000_000_020)
    assert writer.samples == 1
    assert not matcher.frames['rgb'] and not matcher.frames['depth']
    matcher.close()
    events = [json.loads(line) for line in (writer.output/'events.jsonl').read_text().splitlines()]
    assert any(row['reason'] == 'DUPLICATE_RAW_ALREADY_CONSUMED' for row in events)


def test_raw_clock_rollback_fails_closed_clears_cache_and_never_resumes(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer, allow_synthetic=True)
    packet = sample()
    matcher.add_frame('rgb', dict(sec=10, nanosec=7), packet['rgb'], dict(receipt_ns=1))
    matcher.add_frame('rgb', dict(sec=9, nanosec=7), packet['rgb'], dict(receipt_ns=2))
    assert matcher.failure == 'RAW_CLOCK_ROLLBACK_FAIL_CLOSED'
    feed_complete_matcher(matcher)
    assert writer.samples == 0
    assert not any(matcher.frames.values()) and not matcher.pending
    matcher.close(terminal_metadata=dict(failure=matcher.failure))


@pytest.mark.parametrize('kind', ['clock_epoch', 'old_receipt', 'reversed_publish'])
def test_matcher_rejects_epoch_or_original_receipt_chronology_violation(tmp_path, kind):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer, allow_synthetic=True)
    if kind == 'clock_epoch':
        feed_complete_matcher(matcher, status_epoch=1)
    elif kind == 'old_receipt':
        feed_complete_matcher(matcher, diagnostic_receipt=21_000_000_000)
    else:
        packet = sample()
        status = detector_status(packet['metadata']['rgb_raw_stamp'])
        status['published'] = 19.
        matcher.add_detector_status(status, receipt_ns=20_000_000_015)
        feed_complete_matcher(matcher)
    assert writer.samples == 0
    assert matcher.failure
    matcher.close()


def test_epoch_change_fails_before_same_key_can_reuse_cached_pixels(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer, allow_synthetic=True)
    packet = sample()['metadata']
    old = detector_status(packet['rgb_raw_stamp'])
    matcher.add_detector_status(old, receipt_ns=20_000_000_015)
    reset = dict(old, clock_epoch=1)
    matcher.add_detector_status(reset, receipt_ns=20_000_000_020)
    assert matcher.failure == 'CLOCK_EPOCH_CHANGED_FAIL_CLOSED'
    assert not matcher.detector_statuses
    assert writer.samples == 0
    matcher.close()


def test_duplicate_source_before_consumption_is_explicit_and_cannot_replace_pixels(tmp_path):
    from uav_control.perception.p765_roi_capture import BoundedRoiWriter, RoiCaptureMatcher
    writer = BoundedRoiWriter(tmp_path/'capture', runtime_identity={})
    matcher = RoiCaptureMatcher(writer)
    packet = sample()
    for receipt in (1, 2):
        matcher.add_frame('rgb', packet['metadata']['rgb_raw_stamp'], packet['rgb'],
                          dict(receipt_ns=receipt))
    assert matcher.failure == 'DUPLICATE_RAW_PENDING_FAIL_CLOSED'
    assert not matcher.frames['rgb']
    assert writer.samples == 0
    matcher.close()

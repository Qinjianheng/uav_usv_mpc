"""Independent known geometry and evidence limits for P7.6.5 diagnostics."""
import json
from pathlib import Path

import numpy as np
import pytest

from uav_control.perception.rgbd_target_localizer import target_geometry_from_rgbd
from uav_control.perception.tof_sphere_localization import fit_tof_sphere

ROOT = Path(__file__).resolve().parents[3]


def test_ideal_diagnostics_preserve_rejection_for_flat_known_surface():
    mask = np.zeros((480, 640), dtype=bool)
    mask[210:270, 290:350] = True
    depth = np.full(mask.shape, 4.75, dtype=np.float32)
    details = {}
    result = target_geometry_from_rgbd(mask, depth, 1.74, .05, 25., .25,
                                       require_valid_sphere=True, diagnostics=details)
    assert result is None
    assert details['valid_depth_count'] == 3600
    assert details['depth_ratio'] == 1.
    assert details['depth_min'] == 4.75
    assert details['depth_median'] == 4.75
    assert details['depth_mad'] == 0.
    assert details['fit_mode'] == 'ideal'
    assert details['fit_rank'] == 3
    assert details['explicit_fit_rejection_reason'] == 'FIT_RANK_DEFICIENT'


@pytest.mark.parametrize('center', [(3., .4, -.1), (8., -1.2, .3)])
def test_forward_x_depth_and_flu_signs_from_independent_ray_intersections(center):
    from synthetic_sphere_fixture import render
    mask, depth, _ = render(center)
    details = {}
    result = target_geometry_from_rgbd(mask, depth.astype(np.float32), 1.74, .05, 25., .25,
                                       require_valid_sphere=True, diagnostics=details)
    assert result is not None
    assert result.center_camera == pytest.approx(center, abs=2e-5)
    assert details['explicit_fit_rejection_reason'] == ''
    assert details['fitted_radius'] == pytest.approx(.25, abs=2e-5)
    assert details['radius_residual'] < 1e-5
    assert details['fit_rank'] == 4
    assert details['fit_condition'] > 1.


def test_tof_diagnostics_explain_sparse_and_planar_rejection():
    details = {}
    assert fit_tof_sphere(np.ones((11, 3)), .25, noise_std=.01,
                          range_noise_scale=.001, diagnostics=details) is None
    assert details['explicit_fit_rejection_reason'] == 'FIT_TOO_FEW_POINTS'
    from synthetic_sphere_fixture import render
    mask, depth, rays = render((5., 0., 0.))
    depth[mask] = 4.75
    cloud = depth[mask, None]*rays[mask]
    details = {}
    assert fit_tof_sphere(cloud, .25, noise_std=.01,
                          range_noise_scale=.001, diagnostics=details) is None
    assert details['explicit_fit_rejection_reason'] == 'RADIAL_RESIDUAL_EXCEEDS_NOISE_BUDGET'
    assert details['fit_rank'] == 3
    assert details['fitted_radius'] > 0
    assert details['radius_residual'] >= 0


def test_geometry_diagnostics_reject_shape_and_keep_nonfinite_depth_counts():
    details = {}
    assert target_geometry_from_rgbd(np.ones((2, 3), bool), np.ones((3, 2)),
                                     1.74, .05, 25., .25, diagnostics=details) is None
    assert details['explicit_fit_rejection_reason'] == 'MASK_DEPTH_SHAPE_MISMATCH'
    depth = np.array([[1., np.nan, np.inf], [0., 26., 1.]], np.float32)
    target_geometry_from_rgbd(np.ones((2, 3), bool), depth, 1.74, .05, 25., .25,
                              require_valid_sphere=True, diagnostics=details)
    assert details['valid_depth_count'] == 2
    assert details['depth_ratio'] == pytest.approx(1/3)
    assert details['explicit_fit_rejection_reason'] == 'FIT_TOO_FEW_POINTS'


def test_frozen_history_preserves_every_failure_and_does_not_invent_pixels():
    from uav_control.perception.p765_geometry import analyze_history
    result = analyze_history(ROOT/'data/baselines/20261021_p764_validation_01/nonflight_01'
                             / 'synchronized_packets.jsonl.gz')
    assert result['observation_count'] == len(result['observations']) == 362
    assert result['reason_counts']['TARGET_VECTOR_INVALID'] == 140
    assert result['valid_observations'] == 0
    assert result['pixel_replay_possible'] is False
    assert result['sphere_failure_subcause'] == 'UNKNOWN_RAW_DEPTH_NOT_RETAINED'
    sphere = [r for r in result['observations'] if r['reason'] == 'TARGET_VECTOR_INVALID']
    assert len(sphere) == 140
    assert all(r['lineage_status'] == 'EXACT_RGB_DEPTH_DETECTION' for r in sphere)
    assert all(r['depth_pixels_retained'] is False for r in sphere)
    assert all(r['rgb_depth_skew_ns'] == 0 for r in sphere)
    assert all(r['mask_mapping_valid'] for r in sphere)
    assert all(r['derived_range_m'] is None for r in sphere)
    assert result['spatial_callback_binding']['matched_observations'] == 362
    assert all(len(row['spatial_dkf_callback_lines']) == 1 for row in result['observations'])


def test_history_does_not_trust_stale_observation_raw_fields():
    from uav_control.perception.p765_geometry import analyze_rows
    rows = [dict(kind='observation', receipt=2., message=dict(
        valid=False, rejection_reason='YOLO_DETECTION_TIMEOUT', rgb_raw_stamp=1.,
        depth_raw_stamp=1., stamp=dict(sec=0, nanosec=0)))]
    result = analyze_rows(rows)
    assert result['observations'][0]['lineage_status'] == 'UNPROVEN_EARLY_REJECTION_RAW_FIELDS'


def test_opt_in_diagnostic_preserves_exact_raw_headers_and_loaded_code_identity():
    from types import SimpleNamespace
    from sensor_msgs.msg import Image
    from uav_control.perception.rgbd_target_localizer import (
        RgbdTargetLocalizer, TimestampedImageFrame,
    )
    sent = []
    node = SimpleNamespace(sphere_fit_diagnostic_pub=SimpleNamespace(publish=sent.append),
                           sphere_fit_runtime_identity={'loaded_code': {'test': 'identity'}},
                           _ros_seconds=lambda: 20.)
    color, depth = Image(), Image()
    color.header.stamp.sec = depth.header.stamp.sec = 10
    color.header.stamp.nanosec = depth.header.stamp.nanosec = 7
    details = dict(explicit_fit_rejection_reason='FIT_TOO_FEW_POINTS')
    RgbdTargetLocalizer._publish_sphere_fit_diagnostic(
        node, details, 'TARGET_VECTOR_INVALID', TimestampedImageFrame(color, 10.000000007, 19.9),
        TimestampedImageFrame(depth, 10.000000007, 19.91), 19.8)
    payload = json.loads(sent[0].data)
    assert payload['rgb_raw_stamp'] == dict(sec=10, nanosec=7)
    assert payload['depth_raw_stamp'] == payload['rgb_raw_stamp']
    assert payload['geometry'] == details
    assert payload['runtime_identity'] == node.sphere_fit_runtime_identity
    assert payload['localization_reason'] == 'TARGET_VECTOR_INVALID'
    assert payload['rgb_receipt_stamp'] == 19.9
    # Default-disabled call is inert without requiring any capture/runtime state.
    RgbdTargetLocalizer._publish_sphere_fit_diagnostic(
        SimpleNamespace(), details, '', None, None, None)


def test_geometry_analysis_cli_exports_all_lineages_and_strict_summary(tmp_path):
    import subprocess
    import sys
    output = tmp_path/'analysis.json'
    completed = subprocess.run([
        sys.executable, str(ROOT/'scripts/p765_geometry_analysis.py'),
        '--input', str(ROOT/'data/baselines/20261021_p764_validation_01/nonflight_01'
                       / 'synchronized_packets.jsonl.gz'), '--output', str(output),
    ], text=True, capture_output=True)
    assert completed.returncode == 0, completed.stderr
    analysis = json.loads(output.read_text())
    assert analysis['sphere_failure_image_statistics']['count'] == 140
    assert analysis['sphere_failure_image_statistics']['measured_range_count'] == 0
    lines = [json.loads(line) for line in (tmp_path/'histlineage.jsonl').read_text().splitlines()]
    assert len(lines) == 362
    assert len([row for row in lines if row['reason'] == 'TARGET_VECTOR_INVALID']) == 140
    assert len(analysis['lineage_file']['sha256']) == 64


def test_consecutive_yolo_timeouts_report_the_current_pair_without_stale_diagnostics():
    from test_rgbd_target_localizer import _make_synthetic_localizer, _rgbd_messages
    from uav_control.perception.rgbd_target_localizer import TimestampedImageFrame
    from uav_control.perception.yolo_detection_contract import DetectionCache
    node, observations = _make_synthetic_localizer([100.5])
    node.detection_source, node.yolo_cache = 'yolo', DetectionCache('a'*64)
    node.last_time_diagnostic = dict(rgb_raw_stamp=99., depth_raw_stamp=99.,
                                     rgb_mapped_stamp=999.)
    for source, receipt in ((100.1, 100.2), (100.2, 100.3)):
        color, depth = _rgbd_messages(source)
        node.pending_image_pairs.append((TimestampedImageFrame(color, source, receipt),
                                         TimestampedImageFrame(depth, source, receipt+.001)))
        node.localize()
    assert [row.rejection_reason for row in observations] == ['YOLO_DETECTION_TIMEOUT']*2
    assert [row.rgb_raw_stamp for row in observations] == [100.1, 100.2]
    assert [row.depth_raw_stamp for row in observations] == [100.1, 100.2]
    assert [row.rgb_receipt_stamp for row in observations] == [100.2, 100.3]
    assert all(np.isnan(row.rgb_mapped_stamp) for row in observations)

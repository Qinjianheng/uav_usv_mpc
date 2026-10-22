"""Recompute the one authorized unarmed capture; never starts ROS/PX4/Gazebo."""
from collections import Counter, defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/uav_control')]
from p764_diagnostic_collect import stats
from p765_qualification import validate_e1_raw, validate_e1_streams
from uav_control.common.p76_readiness import source_fingerprint
from uav_control.perception.rgbd_target_localizer import (
    camera_intrinsics, target_geometry_from_rgbd,
)
from uav_control.perception.yolo_inference import decode_mask

SESSION = ROOT/'data/experiments/p765_offline_validation/nonflight_01'
OLD = ROOT/'data/baselines/20261021_p764_validation_01/nonflight_01'
OUTPUT = Path(__file__).parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def packets(root):
    return [json.loads(line) for line in gzip.open(root/'synchronized_packets.jsonl.gz', 'rt')]


def detection_stats(rows):
    reports = [json.loads(r['message']['data']) for r in rows if r['kind'] == 'yolo']
    detections = [r for r in rows if r['kind'] == 'detection']
    valid = sum(r['message']['valid'] for r in detections)
    result = dict(count=len(detections), valid=valid, valid_fraction=valid/len(detections),
                  inference=stats([r['inference_seconds'] for r in reports
                                   if r['inference_seconds'] > 0]),
                  completion_source_age=stats([r['source_age'] for r in reports
                                               if r['source_age'] is not None]),
                  reasons=dict(Counter(r['reason'] for r in reports)))
    for key in ('pending_wait_seconds', 'compute_seconds', 'source_age_at_receipt',
                'publication_source_age'):
        result[key] = stats([r[key] for r in reports if r.get(key) is not None])
    result['replaced_pending_frames'] = max((r.get('replaced_pending_frames', 0)
                                             for r in reports), default=0)
    return result


def geometry_analysis():
    events = [json.loads(line) for line in (SESSION/'roi/events.jsonl').read_text().splitlines()]
    cases = []
    for path in sorted((SESSION/'roi').glob('roi_*.json')):
        metadata = json.loads(path.read_text())
        data_path = path.parent/metadata['data_file']
        assert digest(data_path) == metadata['data_sha256']
        with np.load(data_path, allow_pickle=False) as data:
            roi, colors = data['depth'], data['rgb']
        packet, contract = metadata['detection'], metadata['geometry_contract']
        mask = decode_mask(packet['mask_counts'], packet['image_width'], packet['image_height'])
        depth = np.full(mask.shape, np.nan, dtype=np.float32)
        left, top, right, bottom = metadata['roi_xyxy_exclusive']
        depth[top:bottom, left:right] = roi
        valid = mask & np.isfinite(depth) & (depth >= contract['minimum_depth']) & (
            depth <= contract['maximum_depth'])
        ordered = np.sort(depth[valid])
        # OFFLINE COUNTERFACTUAL ONLY: fixed known sphere diameter, no optimized threshold.
        groups = np.split(ordered, np.flatnonzero(np.diff(ordered) > 2*contract['target_radius'])+1)
        group = max(groups, key=len)
        foreground = valid & (depth >= group[0]) & (depth <= group[-1])
        background = valid & ~foreground
        before, after = {}, {}
        original = target_geometry_from_rgbd(mask, depth, diagnostics=before, **contract)
        filtered = target_geometry_from_rgbd(foreground, depth, diagnostics=after, **contract)
        y, x = np.where(background)
        values = depth[background].astype(float)
        fx, fy, cx, cy = camera_intrinsics(mask.shape[1], mask.shape[0], contract['horizontal_fov'])
        points = np.c_[values, -(x-cx)*values/fx, -(y-cy)*values/fy]
        center = points.mean(axis=0)
        _, singular, vectors = np.linalg.svd(points-center)
        normal = vectors[-1]
        cases.append(dict(
            raw_stamp=metadata['rgb_raw_stamp'], data_sha256=digest(data_path),
            mask_count=int(mask.sum()), raw_valid_depth_count=int(valid.sum()),
            foreground_count=int(foreground.sum()), background_count=int(background.sum()),
            invalid_count=int((mask & ~valid).sum()),
            depth_components=[dict(count=len(g), minimum=float(g[0]), maximum=float(g[-1]))
                              for g in groups],
            background_rgb=[colors[v-top, u-left].tolist() for v, u in zip(y, x)],
            background_camera_plane=dict(unit_normal=normal.tolist(),
                offset=float(-normal@center),
                maximum_residual=float(np.max(np.abs((points-center)@normal))),
                singular_values=singular.tolist()),
            original_valid=original is not None, original_diagnostics=before,
            counterfactual_valid=filtered is not None, counterfactual_diagnostics=after,
            counterfactual_center_camera=filtered.center_camera if filtered else None,
            foreground_support_over_original_mask=float(foreground.sum()/mask.sum()),
            counterfactual_scope='Changed point association only, original sphere thresholds; '
                                 'not an online source modification or estimator qualification'))
    return dict(
        fit_reasons=dict(Counter(r.get('metadata', {}).get('geometry', {}).get(
            'explicit_fit_rejection_reason') for r in events
            if r.get('metadata', {}).get('geometry'))),
        capture_event_reasons=dict(Counter(r['reason'] for r in events)), cases=cases,
        unique_pixel_payloads=len({r['data_sha256'] for r in cases}),
        caution='Ten source frames in one stationary scene; identical pixel payloads are not '
                'independent scenes. No association policy or covariance has been flight-qualified.')


def process_resources():
    path = SESSION/'process_resources.jsonl'
    samples = [json.loads(line) for line in path.read_text().splitlines()]
    by_pid = defaultdict(list)
    for sample in samples:
        for process in sample['processes']:
            by_pid[(process['pid'], process['start_ticks'])].append((sample['monotonic'], process))
    reports = []
    for key, rows in by_pid.items():
        if len(rows) < 2:
            continue
        a, b = rows[0], rows[-1]
        reports.append(dict(pid=key[0], start_ticks=key[1], command=a[1]['command'],
            seconds=b[0]-a[0], core_fraction=(b[1]['cpu_seconds']-a[1]['cpu_seconds'])/(b[0]-a[0]),
            peak_rss_bytes=max(r[1]['rss_bytes'] for r in rows)))
    cores = defaultdict(list)
    for before, after in zip(samples, samples[1:]):
        for name, end in after['cpu_ticks_per_core'].items():
            if name == 'cpu' or name not in before['cpu_ticks_per_core']:
                continue
            delta = np.asarray(end[:8])-np.asarray(before['cpu_ticks_per_core'][name][:8])
            total, idle = delta.sum(), delta[3]+delta[4]
            if total > 0:
                cores[name].append(float(1-idle/total))
    return dict(processes=reports, host_per_core_busy_fraction={k: stats(v) for k,v in cores.items()},
                scope='Host per-core activity is not wholly attributable to this experiment; '
                      'process samples use exact inherited run-directory environment')


def main():
    current, old = packets(SESSION), packets(OLD)
    raw = json.loads((SESSION/'e1_executor_raw.json').read_text())
    paths = {n: SESSION/n for n in ('synchronized_packets.jsonl.gz',
              'actual_callback_inputs.jsonl.gz', 'phase_labels.json', 'e1_executor_raw.json')}
    validate_e1_streams(paths, raw)
    assert raw['sources'] == source_fingerprint(ROOT)
    e1 = dict(raw_evidence={str(path):digest(path) for path in paths.values()})
    try:
        validate_e1_raw(e1)
        admission = dict(qualified=True)
    except ValueError as error:
        admission = dict(qualified=False, reason=str(error))
    times = np.asarray(raw['timer_times'])
    image_spans = [s for s in raw['callback_spans'] if s['nested_depth'] == 0 and s['method'] in (
        'ImageDkfNode.receive', 'ImageDkfNode.tick')]
    missed_intervals = [(a,b,max(0,round((b-a)/.005)-1)) for a,b in zip(times,times[1:])
                        if max(0,round((b-a)/.005)-1)]
    overlap = [(a,b,n) for a,b,n in missed_intervals
               if any(s['began'] < b and s['ended'] > a for s in image_spans)]
    observations = [r['message'] for r in current if r['kind']=='observation']
    result = dict(schema_version=1, source_head='f5c8338', actual_sessions=1, flight_sessions=0,
        source_inputs={str(p):digest(p) for p in paths.values()},
        raw_capture=json.loads((SESSION/'p765_postflight.json').read_text())['raw_capture'],
        geometry=geometry_analysis(),
        observation_reasons=dict(Counter(r['rejection_reason'] for r in observations)),
        actual_valid_3d=sum(r['valid'] for r in observations),
        valid_2d_outputs=sum(r['message']['valid'] for r in current if r['kind']=='image'),
        old_detection=detection_stats(old), new_detection=detection_stats(current),
        old_shadow=json.loads((OLD/'e1_tracker_shadow_timing.json').read_text()),
        new_shadow=json.loads((SESSION/'e1_tracker_shadow_timing.json').read_text()),
        scheduling=dict(missed_intervals=len(missed_intervals),
                        with_image_callback_overlap=len(overlap),
                        missed_cycles_with_image_callback_overlap=sum(r[2] for r in overlap),
                        caveat='Temporal overlap in the same executor, not independent CPU '
                               'causal attribution; nested callbacks counted once.'),
        resources=process_resources(), independent_admission=admission,
        graph_and_phase=json.loads((SESSION/'e1_real_data_coverage.json').read_text()),
        comparison_limitations=['Same scene/model/hardware but changed worker/EOF/serializer plus '
                                'extra ROI/resource/identity recording and more successful 2D updates.',
                                'Do not attribute one change or declare equal-load scheduling improvement.'],
        decision=dict(E0='BLOCKED',E1='FAIL',E2='FAIL',A1='NO-GO',
                      production_source_changed=False,second_session_authorized=False))
    (OUTPUT/'capture_analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(samples=len(result['geometry']['cases']),
        reasons=result['geometry']['fit_reasons'],actual_3d=result['actual_valid_3d'],
        old_valid=result['old_detection']['valid'],new_valid=result['new_detection']['valid'],
        missed=result['new_shadow']['missed_cycles'],overlap=result['scheduling']),indent=2))


if __name__ == '__main__':
    main()

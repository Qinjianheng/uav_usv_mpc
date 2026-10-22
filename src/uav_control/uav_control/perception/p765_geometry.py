"""Offline frame lineage audit; absent pixels never become inferred sphere evidence."""
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .yolo_inference import decode_mask


def stamp_ns(stamp):
    """Keep the integer identity provided by the source ROS header."""
    return int(stamp['sec'])*1_000_000_000+int(stamp['nanosec'])


def json_finite(value):
    """Use strict JSON null for missing/nonfinite diagnostics, retaining source hashes."""
    if isinstance(value, dict):
        return {str(key): json_finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_finite(item) for item in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(value):
        return None
    if isinstance(value, np.generic):
        return value.item()
    return value


def analyze_rows(rows):
    """Join acquisition keys only; receipt-nearest masks are never substituted."""
    rows = list(rows)
    frames = {kind: defaultdict(list) for kind in ('rgb_frame', 'depth_frame', 'detection')}
    for index, row in enumerate(rows):
        kind, message = row['kind'], row['message']
        if kind not in frames:
            continue
        key = stamp_ns(message['raw_stamp'] if kind == 'detection'
                       else message['header']['stamp'])
        frames[kind][key].append((index, row))
    observations, reasons = [], Counter()
    early = {'YOLO_DETECTION_TIMEOUT', 'YOLO_YOLO_SOURCE_STALE_AFTER_INFERENCE',
             'YOLO_SOURCE_STALE_OR_INVALID'}
    for index, row in enumerate(rows):
        if row['kind'] != 'observation':
            continue
        message = row['message']
        reason = message.get('rejection_reason', '')
        reasons[reason] += 1
        rgb_raw, depth_raw = message.get('rgb_raw_stamp'), message.get('depth_raw_stamp')
        finite = all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
                     for v in (rgb_raw, depth_raw))
        rgb_ns = round(rgb_raw*1e9) if finite else None
        depth_ns = round(depth_raw*1e9) if finite else None
        matched = {kind: frames[kind].get(depth_ns if kind == 'depth_frame' else rgb_ns, [])
                   for kind in frames}
        # These branches return before last_time_diagnostic is refreshed. Its
        # raw fields can describe a previous/waiting frame, so the join is unproven.
        lineage = ('UNPROVEN_EARLY_REJECTION_RAW_FIELDS' if reason in early else
                   'EXACT_RGB_DEPTH_DETECTION' if all(matched.values()) else
                   'INCOMPLETE_ACQUISITION_LINEAGE')
        detection = matched['detection'][-1][1]['message'] if matched['detection'] else None
        mapping_valid, mask_count, bounds, centroid = False, None, None, None
        if detection is not None and detection.get('mask_counts'):
            try:
                mask = decode_mask(detection['mask_counts'], detection['image_width'],
                                   detection['image_height'])
                yy, xx = np.where(mask)
                mask_count = len(xx)
                if len(xx):
                    bounds = [int(xx.min()), int(yy.min()), int(xx.max()), int(yy.max())]
                    centroid = [float(np.mean(xx)), float(np.mean(yy))]
                mapping_valid = all(
                    entry[1]['message']['width'] == detection['image_width'] and
                    entry[1]['message']['height'] == detection['image_height']
                    for kind in ('rgb_frame', 'depth_frame') for entry in matched[kind]
                ) and bool(matched['rgb_frame'] and matched['depth_frame'])
            except ValueError:
                pass
        observations.append(dict(
            source_line=index+1, reason=reason, valid=bool(message.get('valid', False)),
            original_observation=row, rgb_raw_ns=rgb_ns, depth_raw_ns=depth_ns,
            raw_float_identity_note='Float seconds matched against original integer headers.',
            rgb_depth_skew_ns=abs(rgb_ns-depth_ns) if finite else None,
            lineage_status=lineage,
            matched_source_lines={kind: [entry[0]+1 for entry in entries]
                                  for kind, entries in matched.items()},
            mask_mapping_valid=mapping_valid, mask_pixel_count=mask_count, mask_bounds=bounds,
            mask_centroid_uv=centroid,
            detector_center_uv=detection.get('center_uv') if detection else None,
            detector_valid=detection.get('valid') if detection else None,
            detector_reason=detection.get('rejection_reason') if detection else None,
            detector_model_sha256=detection.get('model_sha256') if detection else None,
            rgb_pixels_retained=False, depth_pixels_retained=False, derived_range_m=None,
            detailed_fit_rejection_reason=None,
            spatial_dkf_result=('NO_VALID_MEASUREMENT' if not message.get('valid')
                                else 'REQUIRES_CALLBACK_REPLAY'),
        ))
    spatial = [row for row in rows if row['kind'] == 'spatial']
    failures = [record for record in observations if record['reason'] == 'TARGET_VECTOR_INVALID']

    def distribution(values):
        values = [value for value in values if value is not None]
        return dict(count=len(values), minimum=min(values), maximum=max(values),
                    p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95))) \
            if values else dict(count=0)

    image_statistics = dict(
        count=len(failures), measured_range_count=0,
        unique_raw_acquisitions=len({record['rgb_raw_ns'] for record in failures}),
        mask_pixel_count=distribution(record['mask_pixel_count'] for record in failures),
        detector_center_u=distribution(record['detector_center_uv'][0]
                                       for record in failures if record['detector_center_uv']),
        detector_center_v=distribution(record['detector_center_uv'][1]
                                       for record in failures if record['detector_center_uv']),
        mask_centroid_u=distribution(record['mask_centroid_uv'][0]
                                     for record in failures if record['mask_centroid_uv']),
        mask_centroid_v=distribution(record['mask_centroid_uv'][1]
                                     for record in failures if record['mask_centroid_uv']),
        mask_bbox_width_px=distribution(record['mask_bounds'][2]-record['mask_bounds'][0]+1
                                        for record in failures if record['mask_bounds']),
        mask_bbox_height_px=distribution(record['mask_bounds'][3]-record['mask_bounds'][1]+1
                                         for record in failures if record['mask_bounds']),
        interpretation='Failure image coverage only; zero successes prevents risk comparison.',
    )
    return json_finite(dict(
        schema_version=1, observation_count=len(observations), reason_counts=dict(reasons),
        valid_observations=sum(record['valid'] for record in observations),
        valid_spatial_outputs=sum(bool(row['message'].get('valid')) for row in spatial),
        observation_lineage_counts=dict(Counter(
            record['lineage_status'] for record in observations)),
        sphere_failure_image_statistics=image_statistics,
        pixel_replay_possible=False, sphere_failure_subcause='UNKNOWN_RAW_DEPTH_NOT_RETAINED',
        unknowns=['Rejected fits lack valid_depth_count/depth distribution/point cloud.',
                  'RLE extent is valid; alignment/background contamination need pixels.',
                  'range-dependent failure cannot be measured from invalid target_range or truth',
                  'fit rank/condition/radius/residual/covariance cannot be reconstructed',
                  'No evidence proves a depth/coordinate/numerical implementation defect.'],
        depth_convention='Depth is camera X in metres; camera FLU image right=-Y/down=-Z.',
        failure_geometry_fields='Rejected-fit fields are defaults, not measured zero depth/count.',
        evidence_scope='Unarmed marked sphere; no flight/noncooperative-USV qualification.',
        observations=observations,
    ))


def analyze_history(path):
    """Read immutable historical packets and bind this audit to their SHA256."""
    path = Path(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as stream:
        result = analyze_rows(json.loads(line) for line in stream)
    result['source'] = dict(path=str(path.resolve()),
                            sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    callbacks = path.parent/'actual_callback_inputs.jsonl.gz'
    if callbacks.is_file():
        index = defaultdict(list)

        def fingerprint(message):
            return hashlib.sha256(json.dumps(
                json_finite(message), sort_keys=True, separators=(',', ':')).encode()).hexdigest()

        with gzip.open(callbacks, 'rt') as stream:
            for line, text in enumerate(stream, start=1):
                row = json.loads(text)
                if row['node'] == 'SpatialDkfNode' and row['method'] == 'receive':
                    index[fingerprint(row['arguments'][0])].append(dict(
                        source_line=line, receipt=row['receipt'], phase=row['phase']))
        for record in result['observations']:
            hits = index.get(fingerprint(record['original_observation']['message']), [])
            record['spatial_dkf_callback_lines'] = [hit['source_line'] for hit in hits]
            record['spatial_dkf_callbacks'] = hits
        result['spatial_callback_binding'] = dict(
            path=str(callbacks.resolve()),
            sha256=hashlib.sha256(callbacks.read_bytes()).hexdigest(),
            match_rule='Full original observation payload SHA256; no receipt-time nearest match.',
            matched_observations=sum(bool(record['spatial_dkf_callback_lines'])
                                     for record in result['observations']))
    return result

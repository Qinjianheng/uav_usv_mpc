"""Bounded lossless RGB/depth ROI evidence; used only by a separate capture process."""
from collections import Counter, OrderedDict
import hashlib
import inspect
import io
import json
import marshal
import math
from pathlib import Path
import platform
import sys

import numpy as np

from .p765_geometry import json_finite, stamp_ns
from .yolo_inference import decode_mask


def image_array(message):
    """View raw image rows with source padding/byte order; no depth values are altered."""
    encoding = message.encoding.lower()
    if encoding in ('32fc1', '32fc'):
        dtype, channels = np.dtype('>f4' if message.is_bigendian else '<f4'), 1
    elif encoding in ('rgb8', 'bgr8'):
        dtype, channels = np.dtype('u1'), 3
    else:
        raise ValueError('ROI_IMAGE_ENCODING')
    width, height, step = int(message.width), int(message.height), int(message.step)
    if (width <= 0 or height <= 0 or width*height > 4_000_000
            or step < width*channels*dtype.itemsize or len(message.data) < height*step):
        raise ValueError('ROI_IMAGE_EXTENT')
    shape = (height, width) if channels == 1 else (height, width, channels)
    strides = (step, dtype.itemsize) if channels == 1 else (step, channels, 1)
    return np.ndarray(shape, dtype=dtype, buffer=message.data, strides=strides)


def runtime_identity():
    """Fingerprint code objects actually imported, separately from mutable disk bytes."""
    from .rgbd_target_localizer import (
        RgbdTargetLocalizer, target_geometry_from_rgbd, validation_sphere_center,
    )
    from .front_tof_monitor import decode_float32_depth
    from .tof_sphere_localization import fit_tof_sphere
    from .yolo_detection_contract import detection_mask
    from .yolo_inference import YoloInference, decode_segmentation, encode_mask, letterbox_rgb
    loaded = {}
    for function in (target_geometry_from_rgbd, validation_sphere_center, fit_tof_sphere,
                     RgbdTargetLocalizer.localize, decode_float32_depth, YoloInference.infer,
                     decode_segmentation, encode_mask, decode_mask, letterbox_rgb, detection_mask):
        path = Path(inspect.getfile(function)).resolve()
        loaded[function.__name__] = dict(
            code_sha256=hashlib.sha256(marshal.dumps(function.__code__)).hexdigest(),
            source_path=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
                numpy=np.__version__, loaded_code=loaded)


def replay_roi(metadata_path):
    """Reconstruct original full-image pixel coordinates before fitting saved ROI depths."""
    from .rgbd_target_localizer import target_geometry_from_rgbd
    metadata_path = Path(metadata_path)
    metadata = json.loads(metadata_path.read_text())
    data_path = metadata_path.parent/metadata['data_file']
    if hashlib.sha256(data_path.read_bytes()).hexdigest() != metadata['data_sha256']:
        raise ValueError('ROI_DATA_SHA256')
    contract = metadata['geometry_contract']
    if contract.get('require_valid_sphere') is not True:
        raise ValueError('ROI_VALID_SPHERE_REQUIRED')
    mapping, packet = metadata['mask_mapping'], metadata['detection']
    mask = decode_mask(packet['mask_counts'], packet['image_width'], packet['image_height'])
    if (mapping['order'] != 'C' or mapping['roi_scale_uv'] != [1, 1]
            or [mapping['original_height'], mapping['original_width']] != list(mask.shape)):
        raise ValueError('ROI_MAPPING_INVALID')
    left, top, right, bottom = metadata['roi_xyxy_exclusive']
    if (not (0 <= left < right <= mask.shape[1] and 0 <= top < bottom <= mask.shape[0])
            or mapping['roi_origin_uv'] != [left, top]):
        raise ValueError('ROI_MAPPING_INVALID')
    with np.load(data_path, allow_pickle=False) as arrays:
        roi, roi_mask = arrays['depth'], arrays['mask']
        if (roi.shape != (bottom-top, right-left) or roi.dtype.kind != 'f'
                or roi.dtype.itemsize != 4
                or not np.array_equal(roi_mask, mask[top:bottom, left:right])
                or np.count_nonzero(mask) != np.count_nonzero(roi_mask)):
            raise ValueError('ROI_MASK_OR_DEPTH_INVALID')
        depth = np.full(mask.shape, np.nan, dtype=roi.dtype)
        depth[top:bottom, left:right] = roi
    details = {}
    geometry = target_geometry_from_rgbd(mask, depth, diagnostics=details, **contract)
    return json_finite(dict(metadata_file=str(metadata_path.resolve()),
                            source_data_sha256=metadata['data_sha256'], diagnostics=details,
                            valid_sphere=geometry is not None,
                            center_camera_flu=geometry.center_camera if geometry else None,
                            fit_position_std=geometry.fit_position_std if geometry else None,
                            original_localization_reason=metadata['localization_reason'],
                            qualification='OFFLINE_ONLY_NO_NEW_REAL_OBSERVATIONS'))


class BoundedRoiWriter:
    """Stratify successes/failure reasons and retain explicit metadata for skipped offers."""

    def __init__(self, output, *, runtime_identity, max_samples=60, max_per_stratum=10,
                 max_bytes=64*1024*1024, max_events=10000):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.identity = runtime_identity
        self.maximum, self.per_stratum, self.byte_limit, self.event_limit = (
            max(0, int(value)) for value in (max_samples, max_per_stratum, max_bytes, max_events))
        self.samples, self.bytes_written, self.events_count = 0, 0, 0
        self.strata, self.files = Counter(), {}
        self.events = (self.output/'events.jsonl').open('x')

    def capture(self, *, rgb, depth, mask, metadata):
        """Save a paired ROI without quantization, filtering NaNs, resizing, or retiming."""
        if self.events_count >= self.event_limit:
            return dict(saved=False, reason='EVENT_LIMIT')
        reason = metadata.get('localization_reason', 'UNKNOWN')
        stratum = 'success' if not reason else 'failure:'+reason
        result = dict(saved=False, stratum=stratum)
        try:
            self._validate(rgb, depth, mask, metadata)
            if self.samples >= self.maximum:
                raise ValueError('SAMPLE_LIMIT')
            if self.strata[stratum] >= self.per_stratum:
                raise ValueError('STRATUM_LIMIT')
            yy, xx = np.where(mask)
            if not len(xx):
                raise ValueError('MASK_EMPTY')
            left, top = int(xx.min()), int(yy.min())
            right, bottom = int(xx.max()+1), int(yy.max()+1)
            payload = io.BytesIO()
            np.savez_compressed(payload, rgb=rgb[top:bottom, left:right],
                                depth=depth[top:bottom, left:right],
                                mask=mask[top:bottom, left:right])
            content = payload.getvalue()
            data_name = f'roi_{self.samples:04d}_{stamp_ns(metadata["rgb_raw_stamp"])}.npz'
            info_name = data_name[:-4]+'.json'
            details = dict(metadata, data_file=data_name,
                           data_sha256=hashlib.sha256(content).hexdigest(),
                           roi_xyxy_exclusive=[left, top, right, bottom],
                           mask_mapping=dict(order='C', original_width=mask.shape[1],
                                             original_height=mask.shape[0],
                                             roi_origin_uv=[left, top], roi_scale_uv=[1, 1]),
                           depth_dtype=depth.dtype.str, depth_unit='metre_camera_forward_X',
                           runtime_identity_sha256=hashlib.sha256(json.dumps(
                               self.identity, sort_keys=True).encode()).hexdigest())
            text = json.dumps(json_finite(details), sort_keys=True,
                              allow_nan=False, indent=2).encode()
            if self.bytes_written+len(content)+len(text) > self.byte_limit:
                raise ValueError('BYTE_LIMIT')
            (self.output/data_name).write_bytes(content)
            (self.output/info_name).write_bytes(text)
            self.files[data_name] = details['data_sha256']
            self.files[info_name] = hashlib.sha256(text).hexdigest()
            self.samples += 1
            self.strata[stratum] += 1
            self.bytes_written += len(content)+len(text)
            result.update(saved=True, reason='SAVED', data_file=data_name, metadata_file=info_name)
        except ValueError as error:
            result['reason'] = str(error)
        self.note(metadata, **result)
        return result

    def note(self, metadata, *, reason, saved=False, **fields):
        """Retain original failure metadata when pixels are unavailable or a budget rejects it."""
        if self.events_count >= self.event_limit:
            return
        self.events.write(json.dumps(json_finite(dict(fields, reason=reason, saved=saved,
                                                      metadata=metadata)),
                                     allow_nan=False, sort_keys=True)+'\n')
        self.events_count += 1

    @staticmethod
    def _validate(rgb, depth, mask, metadata):
        keys = [stamp_ns(metadata[name]) for name in
                ('rgb_raw_stamp', 'depth_raw_stamp', 'detection_raw_stamp')]
        if len(set(keys)) != 1 or keys[0] <= 0:
            raise ValueError('RAW_ACQUISITION_MISMATCH')
        if (depth.ndim != 2 or mask.shape != depth.shape or rgb.shape != (*depth.shape, 3)
                or rgb.dtype != np.uint8 or mask.dtype != bool):
            raise ValueError('ROI_ARRAY_SHAPE_OR_ENCODING')
        if depth.dtype.kind != 'f' or depth.dtype.itemsize != 4:
            raise ValueError('DEPTH_NOT_FLOAT32')
        packet = metadata['detection']
        decoded = decode_mask(packet['mask_counts'], packet['image_width'], packet['image_height'])
        if (stamp_ns(packet['raw_stamp']) != keys[0] or decoded.shape != mask.shape
                or not np.array_equal(decoded, mask)):
            raise ValueError('MASK_RLE_MISMATCH')

    def close(self):
        """Freeze manifest and checksums, including every skipped/failure metadata event."""
        if self.events.closed:
            return
        self.events.close()
        self.files['events.jsonl'] = hashlib.sha256(
            (self.output/'events.jsonl').read_bytes()).hexdigest()
        manifest = dict(schema_version=1, runtime_identity=self.identity,
                        saved_samples=self.samples, sample_strata=dict(self.strata),
                        payload_bytes=self.bytes_written, events=self.events_count,
                        limits=dict(samples=self.maximum, per_stratum=self.per_stratum,
                                    payload_bytes=self.byte_limit, events=self.event_limit),
                        scope='ROI pixels only; full original mask and source stamps retained.',
                        files=self.files)
        path = self.output/'manifest.json'
        path.write_text(json.dumps(manifest, sort_keys=True, indent=2))
        hashes = dict(self.files)
        hashes['manifest.json'] = hashlib.sha256(path.read_bytes()).hexdigest()
        (self.output/'SHA256SUMS').write_text(''.join(
            f'{digest}  {name}\n' for name, digest in sorted(hashes.items())))


class RoiCaptureMatcher:
    """Bound exact-key image/mask/fit queues in the recorder process, never in control."""

    def __init__(self, writer, capacity=24, *, allow_synthetic=False):
        self.writer, self.capacity = writer, max(1, int(capacity))
        self.allow_synthetic = bool(allow_synthetic)
        self.frames = {name: OrderedDict() for name in ('rgb', 'depth')}
        self.detections, self.pending = OrderedDict(), OrderedDict()
        self.detector_statuses = OrderedDict()
        self.last_raw, self.completed, self.failure = {}, set(), ''
        self.last_epoch = {}

    def _fail(self, reason, metadata):
        self.failure = reason
        for packet, _ in self.pending.values():
            self.writer.note(packet, reason='FAIL_CLOSED_PENDING')
        self.writer.note(metadata, reason=reason)
        caches = (*self.frames.values(), self.detections, self.detector_statuses, self.pending)
        for cache in caches:
            cache.clear()

    def _accept(self, stream, key, metadata):
        if self.failure:
            self.writer.note(metadata, reason='RECORDER_FAIL_CLOSED')
            return False
        if key < self.last_raw.get(stream, key):
            self._fail('RAW_CLOCK_ROLLBACK_FAIL_CLOSED', metadata)
            return False
        if key in self.completed:
            self.writer.note(metadata, reason='DUPLICATE_RAW_ALREADY_CONSUMED')
            return False
        if key == self.last_raw.get(stream):
            self._fail('DUPLICATE_RAW_PENDING_FAIL_CLOSED', metadata)
            return False
        self.last_raw[stream] = key
        return True

    def _accept_epoch(self, stream, packet):
        epoch = packet.get('clock_epoch')
        if isinstance(epoch, int):
            if epoch != self.last_epoch.get(stream, epoch):
                self._fail('CLOCK_EPOCH_CHANGED_FAIL_CLOSED', packet)
                return False
            self.last_epoch[stream] = epoch
        return True

    @staticmethod
    def _ordered_times(values):
        return (all(isinstance(value, (int, float)) and math.isfinite(value) and value > 0
                    for value in values)
                and all(left <= right+1e-6 for left, right in zip(values, values[1:])))

    def _bound(self, mapping):
        while len(mapping) > self.capacity:
            mapping.popitem(last=False)

    def add_frame(self, name, stamp, array, metadata):
        """Retain a bounded raw source array with its untouched header and receipt time."""
        key = stamp_ns(stamp)
        if not self._accept(name, key, metadata):
            return
        self.frames[name][key] = (array, metadata)
        self._bound(self.frames[name])
        self._flush()

    def add_detection(self, packet, *, receipt_ns):
        """Keep the full original row-major RLE and detector source identity."""
        key = stamp_ns(packet['raw_stamp'])
        if not self._accept('detection', key, packet):
            return
        self.detections[key] = (packet, receipt_ns)
        self._bound(self.detections)
        self._flush()

    def add_diagnostic(self, packet, *, receipt_ns):
        """Localizer outcomes supply trustworthy current frame identities even on fit failure."""
        key = stamp_ns(packet['rgb_raw_stamp'])
        if not self._accept_epoch('diagnostic', packet):
            return
        if not self._accept('diagnostic', key, packet):
            return
        self.pending[key] = (packet, receipt_ns)
        while len(self.pending) > self.capacity:
            _, (old, _) = self.pending.popitem(last=False)
            self.writer.note(old, reason='UNMATCHED_DIAGNOSTIC_EVICTED')
        self._flush()

    def add_detector_status(self, packet, *, receipt_ns):
        """Bind the detector process's own code identity, never this recorder's imports."""
        raw = packet.get('raw_stamp')
        identity, frame = packet.get('runtime_identity', {}), packet.get('frame')
        if (not isinstance(raw, dict) or not isinstance(raw.get('sec'), int)
                or not isinstance(raw.get('nanosec'), int)
                or not 0 <= raw['nanosec'] < 1_000_000_000):
            self.writer.note(packet, reason='DETECTOR_STATUS_RAW_STAMP_REQUIRED')
            return
        required = ('YoloTargetDetector.process', 'YoloInference.infer',
                    'decode_segmentation', 'encode_mask')
        loaded = identity.get('loaded_code', {}) if isinstance(identity, dict) else {}

        def digest(value):
            return (isinstance(value, str) and len(value) == 64
                    and all(character in '0123456789abcdef' for character in value))

        if (not isinstance(identity, dict) or not isinstance(identity.get('pid'), int)
                or identity['pid'] <= 0 or not identity.get('executable')
                or not isinstance(frame, int) or frame <= 0
                or not digest(identity.get('model_sha256'))
                or (identity.get('synthetic_only') and not self.allow_synthetic)
                or not isinstance(loaded, dict) or not all(
                    isinstance(loaded.get(name), dict)
                    and digest(loaded[name].get('code_sha256'))
                    and digest(loaded[name].get('source_sha256')) for name in required)):
            self.writer.note(packet, reason='DETECTOR_RUNTIME_IDENTITY_INVALID')
            return
        key = stamp_ns(raw)
        if not self._accept_epoch('detector_status', packet):
            return
        detection = self.detections.get(key)
        if detection is not None and detection[0].get('frame_sequence') != frame:
            self.writer.note(packet, reason='DETECTOR_STATUS_FRAME_MISMATCH')
            return
        if not self._accept('detector_status', key, packet):
            return
        if not self._ordered_times([packet.get(name) for name in (
                'image_receipt', 'selected', 'completed', 'published')]+[receipt_ns*1e-9]):
            self._fail('DETECTOR_RECEIPT_CHRONOLOGY_FAIL_CLOSED', packet)
            return
        self.detector_statuses[(key, frame)] = (packet, receipt_ns)
        self._bound(self.detector_statuses)
        self._flush()

    def _flush(self):
        for key in list(self.pending):
            packet, receipt = self.pending[key]
            depth_key = stamp_ns(packet['depth_raw_stamp'])
            if not (key in self.frames['rgb'] and depth_key in self.frames['depth']
                    and key in self.detections):
                continue
            rgb, rgb_meta = self.frames['rgb'][key]
            depth, depth_meta = self.frames['depth'][depth_key]
            detection, detection_receipt = self.detections[key]
            status_key = (key, detection.get('frame_sequence'))
            if status_key not in self.detector_statuses:
                continue
            status, status_receipt = self.detector_statuses[status_key]
            identity = status['runtime_identity']
            if identity['model_sha256'] != detection.get('model_sha256'):
                self.writer.note(status, reason='DETECTOR_MODEL_SHA256_MISMATCH')
                del self.detector_statuses[status_key]
                continue
            if (not isinstance(packet.get('clock_epoch'), int)
                    or packet['clock_epoch'] != status.get('clock_epoch')):
                self._fail('CLOCK_EPOCH_MISMATCH_FAIL_CLOSED', packet)
                return
            local_times = [packet.get('rgb_receipt_stamp'), packet.get('depth_receipt_stamp')]
            published = packet.get('diagnostic_publish_stamp')
            if (not all(self._ordered_times([source, published, receipt*1e-9])
                        for source in local_times)
                    or receipt*1e-9-min(local_times) > .5
                    or max(rgb_meta.get('receipt_ns', 0), depth_meta.get('receipt_ns', 0),
                           detection_receipt, status_receipt, receipt)*1e-9-min(local_times) > .5):
                self._fail('ROI_RECEIPT_CHRONOLOGY_FAIL_CLOSED', packet)
                return
            metadata = dict(packet, localizer_diagnostic=packet,
                            rgb_metadata=rgb_meta, depth_metadata=depth_meta,
                            detection=detection, detection_raw_stamp=detection['raw_stamp'],
                            detection_receipt_ns=detection_receipt,
                            diagnostic_receipt_ns=receipt, detector_status=status,
                            detector_runtime_identity=identity,
                            detector_status_receipt_ns=status_receipt,
                            detector_runtime_scope=(
                                'SYNTHETIC_ONLY' if identity.get('synthetic_only')
                                else 'DETECTOR_PROCESS_REPORTED'))
            try:
                mask = decode_mask(detection['mask_counts'], detection['image_width'],
                                   detection['image_height'])
                self.writer.capture(rgb=rgb, depth=depth, mask=mask, metadata=metadata)
            except ValueError as error:
                self.writer.note(metadata, reason='ROI_DECODE_'+str(error))
            del self.pending[key]
            self.completed.add(key)
            self.frames['rgb'].pop(key, None)
            self.frames['depth'].pop(depth_key, None)
            self.detections.pop(key, None)
            for consumed in [entry for entry in self.detector_statuses if entry[0] == key]:
                del self.detector_statuses[consumed]

    def close(self, *, terminal_metadata=None):
        """Persist every still-unmatched fit failure before freezing the recorder manifest."""
        for packet, _ in self.pending.values():
            self.writer.note(packet, reason='UNMATCHED_AT_CLOSE')
        self.pending.clear()
        if terminal_metadata is not None:
            self.writer.note(terminal_metadata, reason='SESSION_END')
        self.writer.close()

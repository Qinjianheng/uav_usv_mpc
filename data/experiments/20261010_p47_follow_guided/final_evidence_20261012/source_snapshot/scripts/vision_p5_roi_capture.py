#!/usr/bin/env python3
"""Bounded evaluation-only RGB-D ROI, pose timing, and KF baseline capture."""

import argparse
from collections import Counter, deque
from datetime import datetime
import json
import math
from pathlib import Path
from queue import Full, Queue
from threading import Thread
import random
import time

import numpy as np
import rclpy
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from uav_usv_interfaces.msg import TargetState

from uav_control.perception.front_tof_monitor import (
    COLOR_PIXEL_FORMATS, red_pixel_mask,
)
from vision_static_capture import (
    StaticVisionCapture, gazebo_stamp_seconds, norm3,
    physical_camera_geometry, stamp_seconds,
)


def choose_trigger(message, physical_error, radius, counts, maximum,
                   random_value):
    """Bound the saved set and retain sparse normal controls."""
    if sum(counts.values()) >= maximum:
        return None
    if message.rejection_reason == 'DEPTH_MAD_HIGH':
        reason = 'mad_high'
    elif math.isfinite(message.depth_mad) and (
            message.depth_mad >= 0.7 * radius):
        reason = 'near_mad_limit'
    elif math.isfinite(physical_error) and physical_error >= 0.3:
        reason = 'large_camera_error'
    elif bool(message.valid) and random_value < 0.10:
        reason = 'normal_control'
    else:
        return None
    limits = {'mad_high': maximum, 'near_mad_limit': maximum // 3,
              'large_camera_error': maximum // 3,
              'normal_control': 3 * maximum // 4}
    if counts.get(reason, 0) >= max(limits[reason], 1):
        return None
    return reason


def roi_bounds(message, width, height, padding=8):
    """Return half-open bbox plus padding, constrained to the image."""
    left = max(0, int(message.mask_bbox_left) - padding)
    top = max(0, int(message.mask_bbox_top) - padding)
    right = min(width, int(message.mask_bbox_right) + padding + 1)
    bottom = min(height, int(message.mask_bbox_bottom) + padding + 1)
    if left >= right or top >= bottom:
        return None
    return left, top, right, bottom


def roi_packed_bytes(data, width, height, step, bounds, bytes_per_pixel):
    """Copy only bounded rows/pixels from a padded packed image."""
    width, height, step = int(width), int(height), int(step)
    left, top, right, bottom = bounds
    if (width <= 0 or height <= 0
            or step < width * bytes_per_pixel
            or not (0 <= left < right <= width and
                    0 <= top < bottom <= height)):
        return None
    raw = np.frombuffer(data, dtype=np.uint8)
    if raw.size < height * step:
        return None
    rows = raw[:height * step].reshape(height, step)
    return np.ascontiguousarray(
        rows[top:bottom, left * bytes_per_pixel:right * bytes_per_pixel])


def red_pixel_mask_roi(data, width, height, step, encoding, bounds):
    """Match the full-frame red mask for one half-open ROI."""
    formats = {value[0]: value[1] for value in COLOR_PIXEL_FORMATS.values()}
    channels = formats.get(encoding)
    if channels is None:
        return None
    packed = roi_packed_bytes(data, width, height, step, bounds, channels)
    if packed is None:
        return None
    left, top, right, bottom = bounds
    roi_width, roi_height = right - left, bottom - top
    return red_pixel_mask(packed.tobytes(), roi_width, roi_height,
                          roi_width * channels, encoding)


def decode_float32_depth_roi(data, width, height, step, bounds):
    """Match full-frame little-endian float32 decode for one ROI."""
    packed = roi_packed_bytes(data, width, height, step, bounds, 4)
    if packed is None:
        return None
    left, top, right, bottom = bounds
    return packed.view('<f4').reshape(bottom - top, right - left)


class RoiWriter:
    """Bounded evaluation writer; never touches ROS node state."""

    def __init__(self, directory, maximum_pending=12):
        if maximum_pending <= 0:
            raise ValueError('maximum_pending must be positive')
        self.directory = Path(directory)
        self.queue = Queue(maxsize=maximum_pending)
        self.metadata = (self.directory / 'roi.jsonl').open(
            'x', encoding='utf-8', buffering=1)
        self.written = 0
        self.error = None
        self.closed = False
        self.thread = Thread(target=self._run, name='p5-roi-writer')
        self.thread.start()

    def submit(self, index, mask, depth, item):
        if self.closed or self.error is not None:
            return False
        filename = f'roi_{index:04d}.npz'
        metadata = dict(item, file=filename)
        work = (filename, mask.copy(), depth.copy(), metadata)
        try:
            self.queue.put_nowait(work)
        except Full:
            return False
        return True

    def _run(self):
        while True:
            work = self.queue.get()
            try:
                if work is None:
                    return
                filename, mask, depth, item = work
                path = self.directory / filename
                before = self.metadata.tell()
                try:
                    with path.open('xb') as stream:
                        np.savez_compressed(stream, mask=mask, depth=depth)
                    self.metadata.write(
                        json.dumps(item, allow_nan=True) + '\n')
                    self.written += 1
                except Exception as exc:
                    self.metadata.seek(before)
                    self.metadata.truncate()
                    path.unlink(missing_ok=True)
                    if self.error is None:
                        self.error = exc
            finally:
                self.queue.task_done()

    def close(self):
        self.closed = True
        self.queue.join()
        self.queue.put(None)
        self.thread.join()
        self.metadata.close()
        if self.error is not None:
            raise RuntimeError('ROI writer failed') from self.error


def pose_metadata(query):
    return {
        'status': query.status,
        'left_sim_stamp': query.left_sim_stamp,
        'right_sim_stamp': query.right_sim_stamp,
        'left_mapped_stamp': query.left_ros_stamp,
        'right_mapped_stamp': query.right_ros_stamp,
        'value': query.value,
    }


class P5Capture(StaticVisionCapture):
    def __init__(self, output_path, visual_height_offset, target_radius,
                 maximum_roi=80):
        super().__init__(output_path, visual_height_offset)
        self.target_radius = float(target_radius)
        self.maximum_roi = int(maximum_roi)
        self.roi_directory = output_path.with_suffix('')
        self.roi_directory.mkdir(exist_ok=False)
        self.roi_writer = RoiWriter(self.roi_directory)
        self.next_roi_index = 0
        self.pose_stream = (self.roi_directory / 'pose_receipt.jsonl').open(
            'x', encoding='utf-8', buffering=1)
        self.truth_stream = (self.roi_directory / 'truth.jsonl').open(
            'x', encoding='utf-8', buffering=1)
        self.kf_stream = (self.roi_directory / 'kf.jsonl').open(
            'x', encoding='utf-8', buffering=1)
        self.observation_stream = (
            self.roi_directory / 'observation.jsonl'
        ).open('x', encoding='utf-8', buffering=1)
        self.colors = deque(maxlen=16)
        self.depths = deque(maxlen=16)
        self.trigger_counts = Counter()
        self.rng = random.Random(20260927)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )
        self.create_subscription(Image, '/camera/front/image_raw',
                                 self.color_callback, sensor_qos)
        self.create_subscription(Image, '/camera/front/depth/image_raw',
                                 self.depth_callback, sensor_qos)
        self.create_subscription(TargetState, '/tracking/target_state',
                                 self.kf_callback, 10)

    def color_callback(self, message):
        self.colors.append((stamp_seconds(message.header.stamp), message))

    def depth_callback(self, message):
        self.depths.append((stamp_seconds(message.header.stamp), message))

    def observation_callback(self, message):
        super().observation_callback(message)
        item = {
            'stamp': stamp_seconds(message.stamp),
            'receipt_stamp': self.now(),
            'valid': bool(message.valid),
            'rejection_reason': str(message.rejection_reason),
            'position_ned': [message.position.x, message.position.y,
                             message.position.z],
            'covariance': list(message.covariance),
        }
        self.observation_stream.write(json.dumps(item) + '\n')

    def truth_callback(self, message):
        super().truth_callback(message)
        item = {
            'stamp': stamp_seconds(message.stamp),
            'source_stamp': stamp_seconds(message.source_stamp),
            'valid': bool(message.valid),
            'position_ned': [message.position.x, message.position.y,
                             message.position.z],
            'velocity_ned': [message.velocity.x, message.velocity.y,
                             message.velocity.z],
        }
        self.truth_stream.write(json.dumps(item) + '\n')

    def kf_callback(self, message):
        item = {
            'stamp': stamp_seconds(message.stamp),
            'source_stamp': stamp_seconds(message.source_stamp),
            'receipt_stamp': self.now(),
            'valid': bool(message.valid),
            'position_ned': [message.position.x, message.position.y,
                             message.position.z],
            'velocity_ned': [message.velocity.x, message.velocity.y,
                             message.velocity.z],
            'covariance': list(message.covariance),
        }
        self.kf_stream.write(json.dumps(item) + '\n')
        self.counts['kf_state'] += 1

    def pose_callback(self, message):
        receipt = self.now()
        mono = time.monotonic()
        super().pose_callback(message)
        for pose in message.pose:
            if (str(pose.name).startswith('x500_mono_cam_')
                    and '::' not in str(pose.name)):
                item = {
                    'sim_stamp': gazebo_stamp_seconds(message.header.stamp),
                    'receipt_ros_stamp': receipt,
                    'receipt_monotonic': mono,
                    'mapped_stamp': self.entity_tracker.last_mapped_stamp,
                    'model_position_enu': [pose.position.x, pose.position.y,
                                           pose.position.z],
                    'model_orientation_wxyz': [
                        pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z,
                    ],
                }
                self.pose_stream.write(json.dumps(item) + '\n')
                self.counts['pose_receipt'] += 1
                break

    def write_observation(self, message, truth, entity_query, model_query,
                          px4_position_query, px4_attitude_query,
                          collector_receipt_stamp, collector_wait_seconds):
        super().write_observation(
            message, truth, entity_query, model_query,
            px4_position_query, px4_attitude_query,
            collector_receipt_stamp, collector_wait_seconds,
        )
        if not message.geometry_diagnostics_enabled:
            return
        physical_error = math.nan
        if message.valid and entity_query.value and model_query.value:
            try:
                physical = physical_camera_geometry(
                    model_query.value, entity_query.value, message,
                )
                physical_error = norm3(np.asarray((
                    message.center_camera_x, message.center_camera_y,
                    message.center_camera_z,
                )) - physical[1])
            except (ValueError, ZeroDivisionError):
                pass
        trigger = choose_trigger(
            message, physical_error, self.target_radius,
            self.trigger_counts, self.maximum_roi, self.rng.random(),
        )
        if trigger is None:
            return
        color = next((image for stamp, image in reversed(self.colors)
                      if abs(stamp - message.rgb_raw_stamp) < 1e-6), None)
        depth = next((image for stamp, image in reversed(self.depths)
                      if abs(stamp - message.depth_raw_stamp) < 1e-6), None)
        if color is None or depth is None:
            self.counts['roi_pair_missing'] += 1
            return
        if (message.camera_fx <= 0 or message.camera_fy <= 0
                or (message.mask_bbox_left, message.mask_bbox_top,
                    message.mask_bbox_right, message.mask_bbox_bottom)
                == (0, 0, 0, 0)):
            self.counts['roi_geometry_unavailable'] += 1
            return
        bounds = roi_bounds(message, color.width, color.height)
        if bounds is None:
            self.counts['roi_bbox_missing'] += 1
            return
        mask = red_pixel_mask_roi(
            color.data, color.width, color.height, color.step,
            color.encoding, bounds)
        depths = decode_float32_depth_roi(
            depth.data, depth.width, depth.height, depth.step, bounds)
        if mask is None or depths is None or mask.shape != depths.shape:
            self.counts['roi_decode_failed'] += 1
            return
        rgb_time = float(message.rgb_mapped_stamp)
        depth_time = float(message.depth_mapped_stamp)
        entity_rgb = self.entity_tracker.query_at(rgb_time)
        entity_depth = self.entity_tracker.query_at(depth_time)
        model_rgb = self.model_history.query_at(rgb_time)
        model_depth = self.model_history.query_at(depth_time)
        item = {
            'trigger': trigger,
            'valid': bool(message.valid),
            'rejection_reason': str(message.rejection_reason),
            'rgb_raw_stamp': message.rgb_raw_stamp,
            'depth_raw_stamp': message.depth_raw_stamp,
            'rgb_mapped_stamp': rgb_time,
            'depth_mapped_stamp': depth_time,
            'rgb_depth_skew': message.rgb_depth_acquisition_skew,
            'image_shape': [color.height, color.width],
            'roi_bounds_ltrb': list(bounds),
            'mask_bbox_ltrb_inclusive': [
                message.mask_bbox_left, message.mask_bbox_top,
                message.mask_bbox_right, message.mask_bbox_bottom,
            ],
            'intrinsics_fx_fy_cx_cy': [
                message.camera_fx, message.camera_fy,
                message.camera_cx, message.camera_cy,
            ],
            'target_radius': self.target_radius,
            'depth_mad': message.depth_mad,
            'depth_median': message.depth_median,
            'physical_camera_error_m': physical_error,
            'online_center_camera_flu': [
                message.center_camera_x, message.center_camera_y,
                message.center_camera_z,
            ],
            'entity_at_rgb': pose_metadata(entity_rgb),
            'entity_at_depth': pose_metadata(entity_depth),
            'model_at_rgb': pose_metadata(model_rgb),
            'model_at_depth': pose_metadata(model_depth),
        }
        if not self.roi_writer.submit(
                self.next_roi_index, mask, depths, item):
            self.counts['roi_writer_queue_drop'] += 1
            return
        self.next_roi_index += 1
        self.trigger_counts[trigger] += 1
        self.counts['roi_queued'] += 1

    def close(self):
        self.drain(force=True)
        try:
            self.roi_writer.close()
        finally:
            self.counts['roi_saved'] = self.roi_writer.written
            super().close()
            for stream in (self.pose_stream, self.truth_stream,
                           self.kf_stream, self.observation_stream):
                stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--visual-height-offset', type=float, default=0.42)
    parser.add_argument('--target-radius', type=float, default=0.25)
    parser.add_argument('--maximum-roi', type=int, default=80)
    args = parser.parse_args()
    if args.target_radius <= 0 or args.maximum_roi <= 0:
        parser.error('radius and maximum ROI count must be positive')
    args.output_directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output = args.output_directory / f'vision_p5_capture_{stamp}.csv'
    rclpy.init()
    node = P5Capture(output, args.visual_height_offset,
                     args.target_radius, args.maximum_roi)
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(output)
    print(dict(node.counts))


if __name__ == '__main__':
    main()

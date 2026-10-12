#!/usr/bin/env python3
"""Capture evaluation-only static vision geometry outside mission logging."""

import argparse
from collections import Counter, deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from uav_usv_interfaces.msg import TargetObservation, TargetState

from uav_control.evaluation.gazebo_entity_pose import (
    GazeboEntityPoseTracker, TimedPoseHistory, entity_geometry_residuals,
)
from uav_control.evaluation.pose_frame_diagnostics import (
    gazebo_model_rotation_ned_frd, interpolate_model_pose,
    interpolate_px4_attitude, interpolate_px4_position,
    quaternion_rotation, query_crosses_reset, query_metadata,
    rotation_angle_between_quaternions, rotation_residual_rpy,
)
from uav_control.evaluation.intercept_evaluator import TimestampedStateHistory
from uav_control.evaluation.intercept_evaluator_node import truth_from_message
from uav_control.perception.rgbd_target_localizer import Px4RosClockMapper


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def gazebo_stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nsec) * 1e-9


def norm3(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def pose_history_diagnostics(message):
    """Copy online pose-history timestamps without changing their meaning."""
    return {
        name: float(getattr(message, name))
        for name in (
            'pose_history_start_stamp', 'pose_history_end_stamp',
            'position_history_start_stamp',
            'position_history_end_stamp',
            'attitude_history_start_stamp',
            'attitude_history_end_stamp',
            'position_source_stamp', 'position_mapped_stamp',
            'attitude_source_stamp', 'attitude_mapped_stamp',
        )
    }


def physical_camera_geometry(model_pose, entity_center_ned, message):
    """Project using Gazebo model pose and SDF camera link, independently."""
    model_world = np.asarray(model_pose[:3], dtype=float)
    model_rotation = quaternion_rotation(model_pose[3:])
    # x500_mono_cam/model.sdf: base_link z=0.24, front link relative
    # base_link=(0.35, 0, -0.05), giving model-local z=0.19.
    link_translation = np.array((0.35, 0.0, 0.19))
    pitch = 0.20944
    link_rotation = quaternion_rotation((
        math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0,
    ))
    camera_world = model_world + model_rotation @ link_translation
    camera_rotation = model_rotation @ link_rotation
    entity_world = np.array((
        entity_center_ned[1], entity_center_ned[0],
        -entity_center_ned[2],
    ))
    expected_camera = camera_rotation.T @ (entity_world - camera_world)
    projection = (
        message.camera_cx
        - expected_camera[1] * message.camera_fx / expected_camera[0],
        message.camera_cy
        - expected_camera[2] * message.camera_fy / expected_camera[0],
    )
    model_heading_ned = math.atan2(
        model_rotation[0, 0], model_rotation[1, 0]
    )
    attitude_rotation = quaternion_rotation((
        message.interpolated_attitude_w,
        message.interpolated_attitude_x,
        message.interpolated_attitude_y,
        message.interpolated_attitude_z,
    ))
    px4_yaw = math.atan2(attitude_rotation[1, 0], attitude_rotation[0, 0])
    return (
        camera_world, expected_camera, projection,
        model_heading_ned, px4_yaw,
    )


class StaticVisionCapture(Node):
    """Read shadow observations and independent truth without publishing."""

    def __init__(
        self, output_path, visual_height_offset,
        px4_timestamp_is_ros_time=True,
    ):
        super().__init__('vision_static_capture')
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output_path.open(
            'x', newline='', encoding='utf-8', buffering=1,
        )
        self.writer = None
        self.counts = Counter()
        self.pending = deque(maxlen=1024)
        self.truth_history = TimestampedStateHistory(maximum_age=5.0)
        self.entity_tracker = GazeboEntityPoseTracker(
            history_duration=5.0, clock_history_duration=5.0,
        )
        self.model_history = TimedPoseHistory(
            maximum_age=5.0, interpolator=interpolate_model_pose,
        )
        self.px4_clock_mapper = Px4RosClockMapper(
            source_is_ros_time=px4_timestamp_is_ros_time,
        )
        self.px4_timestamp_is_ros_time = bool(px4_timestamp_is_ros_time)
        self.px4_position_history = TimedPoseHistory(
            maximum_age=5.0, interpolator=interpolate_px4_position,
        )
        self.px4_attitude_history = TimedPoseHistory(
            maximum_age=5.0, interpolator=interpolate_px4_attitude,
        )
        self.visual_height_offset = float(visual_height_offset)
        self.latest_uav_velocity = (math.nan,) * 3
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            TargetObservation, '/perception/front/target_observation',
            self.observation_callback, 10,
        )
        self.create_subscription(
            TargetState, '/target/state', self.truth_callback, 10,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.uav_callback, sensor_qos,
        )
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_callback, sensor_qos,
        )
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GazeboTransportNode
        self.gazebo_node = GazeboTransportNode()
        if not self.gazebo_node.subscribe(
            Clock, '/world/default/clock', self.clock_callback,
        ):
            raise RuntimeError('Could not subscribe to Gazebo clock')
        if not self.gazebo_node.subscribe(
            Pose_V, '/world/default/pose/info', self.pose_callback,
        ):
            raise RuntimeError('Could not subscribe to Gazebo entity poses')
        self.timer = self.create_timer(0.03, self.drain)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def clock_callback(self, message):
        previous_reset = self.entity_tracker.reset_count
        self.entity_tracker.add_clock_anchor(
            gazebo_stamp_seconds(message.sim),
            gazebo_stamp_seconds(message.system),
            self.now(), time.monotonic(),
        )
        if self.entity_tracker.reset_count != previous_reset:
            self.model_history.clear()

    def pose_callback(self, message):
        sim_stamp = gazebo_stamp_seconds(message.header.stamp)
        target_pose = None
        model_pose = None
        for pose in message.pose:
            if str(pose.name).rsplit('::', 1)[-1] == 'usv_target':
                target_pose = pose
            elif (str(pose.name).startswith('x500_mono_cam_')
                  and '::' not in str(pose.name)):
                model_pose = pose
        if target_pose is None:
            return
        accepted = self.entity_tracker.add_pose(
            sim_stamp,
            (target_pose.position.x, target_pose.position.y,
             target_pose.position.z),
            self.now(),
        )
        if accepted and model_pose is not None:
            self.model_history.add(
                sim_stamp, self.entity_tracker.last_mapped_stamp,
                (model_pose.position.x, model_pose.position.y,
                 model_pose.position.z, model_pose.orientation.w,
                 model_pose.orientation.x, model_pose.orientation.y,
                 model_pose.orientation.z),
            )

    def truth_callback(self, message):
        try:
            state = truth_from_message(message)
        except ValueError:
            self.counts['invalid_truth'] += 1
            return
        self.truth_history.add(stamp_seconds(message.stamp), state)

    def uav_callback(self, message):
        self.latest_uav_velocity = (
            float(message.vx), float(message.vy), float(message.vz),
        )
        self.add_px4_sample('position', message, (
            float(message.x), float(message.y), float(message.z),
            float(message.heading), int(message.xy_reset_counter),
            int(message.z_reset_counter),
            int(message.heading_reset_counter), float(message.delta_heading),
        ))

    def attitude_callback(self, message):
        self.add_px4_sample('attitude', message, (
            *tuple(float(component) for component in message.q),
            int(message.quat_reset_counter),
            *tuple(float(component) for component in message.delta_q_reset),
        ))

    def add_px4_sample(self, stream_name, message, value):
        if not all(math.isfinite(component) for component in value):
            self.counts[f'invalid_px4_{stream_name}'] += 1
            return
        source = float(
            message.timestamp_sample or message.timestamp
        ) * 1e-6
        before_reset = self.px4_clock_mapper.reset_count
        mapped = self.px4_clock_mapper.to_ros_time(
            source, self.now(), stream_name=stream_name,
        )
        if self.px4_clock_mapper.reset_count != before_reset:
            self.px4_position_history.clear()
            self.px4_attitude_history.clear()
        if mapped is None:
            self.counts[f'unmapped_px4_{stream_name}'] += 1
            return
        history = (
            self.px4_position_history if stream_name == 'position'
            else self.px4_attitude_history
        )
        if not history.add(source, mapped, value):
            self.counts[f'out_of_order_px4_{stream_name}'] += 1

    def observation_callback(self, message):
        self.counts['received_observation'] += 1
        if len(self.pending) == self.pending.maxlen:
            self.counts['queue_overflow'] += 1
        self.pending.append((message, time.monotonic(), self.now()))

    def drain(self, force=False):
        while self.pending:
            message, queued_at, collector_receipt_stamp = self.pending[0]
            measurement_stamp = stamp_seconds(message.stamp)
            truth = self.truth_history.state_at(measurement_stamp)
            entity_query = self.entity_tracker.query_at(measurement_stamp)
            model_query = self.model_history.query_at(measurement_stamp)
            px4_position_query = self.px4_position_history.query_at(
                measurement_stamp
            )
            px4_attitude_query = self.px4_attitude_history.query_at(
                measurement_stamp
            )
            entity = entity_query.value
            model = model_query.value
            if (
                not force and message.valid
                and (
                    truth is None or entity is None or model is None
                    or px4_position_query.value is None
                    or px4_attitude_query.value is None
                )
                and time.monotonic() - queued_at < 0.5
            ):
                return
            self.pending.popleft()
            self.write_observation(
                message, truth, entity_query, model_query,
                px4_position_query, px4_attitude_query,
                collector_receipt_stamp, time.monotonic() - queued_at,
            )

    def write_observation(
        self, message, truth, entity_query, model_query,
        px4_position_query, px4_attitude_query,
        collector_receipt_stamp, collector_wait_seconds,
    ):
        entity = entity_query.value
        model = model_query.value
        estimate = (
            float(message.position.x), float(message.position.y),
            float(message.position.z),
        )
        truth_position = truth.position if truth else (math.nan,) * 3
        truth_velocity = truth.velocity if truth else (math.nan,) * 3
        entity_reference = (
            (entity[0], entity[1], entity[2] + self.visual_height_offset)
            if entity else (math.nan,) * 3
        )
        total_error = tuple(a - b for a, b in zip(estimate, truth_position))
        entity_error = tuple(
            a - b for a, b in zip(entity_reference, truth_position)
        )
        vision_error = tuple(
            a - b for a, b in zip(estimate, entity_reference)
        )
        residuals = None
        if entity and message.valid and message.geometry_diagnostics_enabled:
            try:
                residuals = entity_geometry_residuals(
                    entity_center_ned=entity,
                    uav_position_ned=(
                        message.interpolated_uav_x,
                        message.interpolated_uav_y,
                        message.interpolated_uav_z,
                    ),
                    attitude_quaternion=(
                        message.interpolated_attitude_w,
                        message.interpolated_attitude_x,
                        message.interpolated_attitude_y,
                        message.interpolated_attitude_z,
                    ),
                    camera_translation_flu=(
                        message.camera_translation_x,
                        message.camera_translation_y,
                        message.camera_translation_z,
                    ),
                    camera_pitch_down=message.camera_pitch_down,
                    intrinsics=(
                        message.camera_fx, message.camera_fy,
                        message.camera_cx, message.camera_cy,
                    ),
                    observed_projection_center=(
                        message.projection_centroid_u,
                        message.projection_centroid_v,
                    ),
                    observed_center_camera=(
                        message.center_camera_x,
                        message.center_camera_y,
                        message.center_camera_z,
                    ),
                    observed_body_flu=(
                        message.target_body_flu_x,
                        message.target_body_flu_y,
                        message.target_body_flu_z,
                    ),
                )
            except ValueError:
                self.counts['geometry_residual_invalid'] += 1
        physical = None
        if (
            model and entity and message.valid
            and message.geometry_diagnostics_enabled
        ):
            try:
                physical = physical_camera_geometry(model, entity, message)
            except (ValueError, ZeroDivisionError):
                self.counts['physical_geometry_invalid'] += 1
        row = {
            'measurement_stamp': stamp_seconds(message.stamp),
            'valid': bool(message.valid),
            'truth_available': truth is not None,
            'geometry_diagnostics_enabled': bool(
                message.geometry_diagnostics_enabled
            ),
            'gazebo_entity_available': entity is not None,
            'gazebo_model_available': model is not None,
            'rejection_reason': str(message.rejection_reason),
            'image_clock_status': str(message.image_clock_status),
            'image_clock_reset_count': int(message.image_clock_reset_count),
            'gazebo_entity_clock_status': self.entity_tracker.last_status,
            'gazebo_entity_clock_reset_count': self.entity_tracker.reset_count,
            'gazebo_entity_raw_stamp': self.entity_tracker.last_raw_stamp,
            'gazebo_entity_mapped_stamp': (
                self.entity_tracker.last_mapped_stamp
            ),
            'rgb_depth_acquisition_skew': message.rgb_depth_acquisition_skew,
            'observation_age': self.now() - stamp_seconds(message.stamp),
            'depth_median': message.depth_median,
            'depth_mad': message.depth_mad,
            'valid_depth_count': message.valid_depth_count,
            'red_pixel_count': message.red_pixel_count,
            'target_range': message.target_range,
            'view_angle': message.view_angle,
            'projection_centroid_u': message.projection_centroid_u,
            'projection_centroid_v': message.projection_centroid_v,
            'mask_centroid_u': message.mask_centroid_u,
            'mask_centroid_v': message.mask_centroid_v,
            'camera_fx': message.camera_fx,
            'camera_fy': message.camera_fy,
            'camera_cx': message.camera_cx,
            'camera_cy': message.camera_cy,
            'camera_pitch_down': message.camera_pitch_down,
            'target_reference_z_offset': message.target_reference_z_offset,
            'gazebo_model_heading_ned': physical[3] if physical else math.nan,
            'px4_attitude_yaw_ned': physical[4] if physical else math.nan,
        }
        row['model_px4_heading_difference'] = (
            math.atan2(
                math.sin(physical[3] - physical[4]),
                math.cos(physical[3] - physical[4]),
            ) if physical else math.nan
        )
        row.update(query_metadata('entity', entity_query))
        row.update(query_metadata('model', model_query))
        for prefix, query in (
            ('px4_position', px4_position_query),
            ('px4_attitude', px4_attitude_query),
        ):
            metadata = query_metadata(prefix, query)
            row.update({
                key.replace('_sim_stamp', '_source_stamp'): value
                for key, value in metadata.items()
            })
        row.update({
            'entity_clock_reset_count_at_write': (
                self.entity_tracker.reset_count
            ),
            'px4_clock_reset_count_at_write': (
                self.px4_clock_mapper.reset_count
            ),
            'px4_clock_status_at_write': self.px4_clock_mapper.last_status,
            'px4_clock_offset_at_write': (
                self.px4_clock_mapper.offset
                if self.px4_clock_mapper.offset is not None else math.nan
            ),
            'px4_clock_offset_online': message.px4_clock_offset,
            'px4_clock_status_online': str(message.px4_clock_status),
            'px4_clock_reset_count_online': message.px4_clock_reset_count,
            'px4_position_reset_crossed': (
                query_crosses_reset(px4_position_query, 4)
                or query_crosses_reset(px4_position_query, 5)
                or query_crosses_reset(px4_position_query, 6)
            ),
            'px4_attitude_reset_crossed': query_crosses_reset(
                px4_attitude_query, 4,
            ),
            'image_rgb_raw_stamp': message.rgb_raw_stamp,
            'image_depth_raw_stamp': message.depth_raw_stamp,
            'image_rgb_mapped_stamp': message.rgb_mapped_stamp,
            'image_depth_mapped_stamp': message.depth_mapped_stamp,
            'image_rgb_receipt_stamp_online': message.rgb_receipt_stamp,
            'image_depth_receipt_stamp_online': message.depth_receipt_stamp,
            'observation_received_stamp_online': stamp_seconds(
                message.received_stamp
            ),
            'observation_processed_stamp_online': stamp_seconds(
                message.processed_stamp
            ),
            'observation_published_stamp_online': stamp_seconds(
                message.published_stamp
            ),
            'collector_receipt_stamp': collector_receipt_stamp,
            'collector_write_stamp': self.now(),
            'collector_wait_seconds': collector_wait_seconds,
            'online_processing_seconds': (
                stamp_seconds(message.processed_stamp)
                - stamp_seconds(message.stamp)
            ),
            'online_publish_seconds': (
                stamp_seconds(message.published_stamp)
                - stamp_seconds(message.stamp)
            ),
            'collector_transport_seconds': (
                collector_receipt_stamp
                - stamp_seconds(message.published_stamp)
            ),
        })
        row.update(pose_history_diagnostics(message))
        row['collector_write_age_seconds'] = (
            row['collector_write_stamp'] - row['measurement_stamp']
        )
        position_reference = px4_position_query.value
        attitude_reference = px4_attitude_query.value
        if position_reference:
            row.update({
                'px4_heading_ned': position_reference[3],
                'px4_xy_reset_counter': position_reference[4],
                'px4_z_reset_counter': position_reference[5],
                'px4_heading_reset_counter': position_reference[6],
                'px4_delta_heading': position_reference[7],
            })
        else:
            row.update({
                key: math.nan for key in (
                    'px4_heading_ned', 'px4_xy_reset_counter',
                    'px4_z_reset_counter', 'px4_heading_reset_counter',
                    'px4_delta_heading',
                )
            })
        row['px4_quat_reset_counter'] = (
            attitude_reference[4] if attitude_reference else math.nan
        )
        for prefix, values, axes in (
            ('px4_position_independent', (
                position_reference[:3] if position_reference
                else (math.nan,) * 3
            ), 'xyz'),
            ('px4_attitude_independent', (
                attitude_reference[:4] if attitude_reference
                else (math.nan,) * 4
            ), 'wxyz'),
            ('px4_delta_q_reset', (
                attitude_reference[5:9] if attitude_reference
                else (math.nan,) * 4
            ), 'wxyz'),
        ):
            for axis, value in zip(axes, values):
                row[f'{prefix}_{axis}'] = value
        if model and position_reference and attitude_reference:
            model_rotation_ned = gazebo_model_rotation_ned_frd(model[3:])
            px4_rotation_ned = quaternion_rotation(attitude_reference[:4])
            roll, pitch, yaw = rotation_residual_rpy(
                model_rotation_ned, px4_rotation_ned,
            )
            rotation_delta = model_rotation_ned @ px4_rotation_ned.T
            row.update({
                'model_px4_roll_residual': roll,
                'model_px4_pitch_residual': pitch,
                'model_px4_yaw_residual': yaw,
                'model_px4_rotation_angle': math.acos(float(np.clip(
                    (np.trace(rotation_delta) - 1.0) / 2.0,
                    -1.0, 1.0,
                ))),
                'px4_attitude_yaw_independent': math.atan2(
                    px4_rotation_ned[1, 0], px4_rotation_ned[0, 0],
                ),
            })
            row['px4_heading_attitude_difference'] = math.atan2(
                math.sin(
                    row['px4_heading_ned']
                    - row['px4_attitude_yaw_independent']
                ),
                math.cos(
                    row['px4_heading_ned']
                    - row['px4_attitude_yaw_independent']
                ),
            )
            row['online_independent_px4_rotation_angle'] = (
                rotation_angle_between_quaternions((
                    message.interpolated_attitude_w,
                    message.interpolated_attitude_x,
                    message.interpolated_attitude_y,
                    message.interpolated_attitude_z,
                ), attitude_reference[:4])
            )
        else:
            for field in (
                'model_px4_roll_residual', 'model_px4_pitch_residual',
                'model_px4_yaw_residual', 'model_px4_rotation_angle',
                'px4_attitude_yaw_independent',
                'px4_heading_attitude_difference',
                'online_independent_px4_rotation_angle',
            ):
                row[field] = math.nan
        for prefix, values in (
            ('model_px4_position_error', (
                tuple(
                    a - b for a, b in zip(
                        (model[1], model[0], -model[2]),
                        position_reference[:3],
                    )
                ) if model and position_reference else (math.nan,) * 3
            )),
            ('online_independent_px4_position_error', (
                tuple(
                    a - b for a, b in zip((
                        message.interpolated_uav_x,
                        message.interpolated_uav_y,
                        message.interpolated_uav_z,
                    ), position_reference[:3])
                ) if position_reference else (math.nan,) * 3
            )),
        ):
            for axis, value in zip('xyz', values):
                row[f'{prefix}_{axis}'] = value
        for prefix, values in (
            ('position', estimate), ('truth', truth_position),
            ('truth_velocity', truth_velocity),
            ('gazebo_entity_center', entity or (math.nan,) * 3),
            ('gazebo_model_world', model[:3] if model else (math.nan,) * 3),
            ('physical_camera_world', (
                physical[0] if physical else (math.nan,) * 3
            )),
            ('physical_expected_camera', (
                physical[1] if physical else (math.nan,) * 3
            )),
            ('physical_camera_center_error', (
                tuple(np.asarray((message.center_camera_x,
                                  message.center_camera_y,
                                  message.center_camera_z)) - physical[1])
                if physical else (math.nan,) * 3
            )),
            ('gazebo_entity_reference', entity_reference),
            ('error', total_error), ('entity_to_truth', entity_error),
            ('vision_to_entity', vision_error),
            ('interpolated_uav', (
                message.interpolated_uav_x,
                message.interpolated_uav_y,
                message.interpolated_uav_z,
            )),
            ('uav_velocity_latest', self.latest_uav_velocity),
            ('center_camera', (
                message.center_camera_x,
                message.center_camera_y,
                message.center_camera_z,
            )),
            ('target_body_flu', (
                message.target_body_flu_x,
                message.target_body_flu_y,
                message.target_body_flu_z,
            )),
            ('camera_center_error', (
                residuals.camera_error if residuals else (math.nan,) * 3
            )),
            ('body_flu_error', (
                residuals.body_error if residuals else (math.nan,) * 3
            )),
        ):
            for axis, value in zip('xyz', values):
                row[f'{prefix}_{axis}'] = value
        for prefix, values in (
            ('projection_error', (
                residuals.pixel_error if residuals else (math.nan,) * 2
            )),
            ('entity_expected_projection', (
                residuals.expected_projection if residuals else (math.nan,) * 2
            )),
            ('physical_expected_projection', (
                physical[2] if physical else (math.nan,) * 2
            )),
            ('physical_projection_error', (
                (message.projection_centroid_u - physical[2][0],
                 message.projection_centroid_v - physical[2][1])
                if physical else (math.nan,) * 2
            )),
        ):
            for axis, value in zip('uv', values):
                row[f'{prefix}_{axis}'] = value
        for prefix, values in (
            ('error', total_error), ('entity_to_truth', entity_error),
            ('vision_to_entity', vision_error),
            ('camera_center_error', (
                residuals.camera_error if residuals else (math.nan,) * 3
            )),
            ('body_flu_error', (
                residuals.body_error if residuals else (math.nan,) * 3
            )),
            ('physical_camera_center_error', (
                tuple(np.asarray((message.center_camera_x,
                                  message.center_camera_y,
                                  message.center_camera_z)) - physical[1])
                if physical else (math.nan,) * 3
            )),
        ):
            row[f'{prefix}_3d'] = norm3(values)
        if self.writer is None:
            self.writer = csv.DictWriter(self.stream, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.counts['written_observation'] += 1
        if (
            message.valid and truth and entity
            and message.geometry_diagnostics_enabled and residuals
        ):
            self.counts['complete_attribution'] += 1
        if physical:
            self.counts['complete_physical_reference'] += 1

    def close(self):
        self.drain(force=True)
        self.stream.close()
        meta_path = self.output_path.with_suffix('.json')
        with meta_path.open('x', encoding='utf-8') as stream:
            json.dump({
                'csv': str(self.output_path),
                'counts': dict(self.counts),
                'visual_height_offset': self.visual_height_offset,
                'px4_timestamp_is_ros_time': (
                    self.px4_timestamp_is_ros_time
                ),
                'collection_mode': 'evaluation_only_no_publish',
            }, stream, indent=2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--visual-height-offset', type=float, default=0.42)
    parser.add_argument(
        '--px4-timestamp-is-ros-time',
        action=argparse.BooleanOptionalAction, default=True,
    )
    parser.add_argument('--output-directory', type=Path, default=Path(
        'data/experiments/current'
    ))
    arguments = parser.parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output = arguments.output_directory / f'vision_static_capture_{stamp}.csv'
    rclpy.init()
    node = StaticVisionCapture(
        output, arguments.visual_height_offset,
        arguments.px4_timestamp_is_ros_time,
    )
    try:
        deadline = time.monotonic() + arguments.seconds
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

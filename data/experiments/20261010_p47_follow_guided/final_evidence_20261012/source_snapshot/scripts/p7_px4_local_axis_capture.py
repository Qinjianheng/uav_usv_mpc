#!/usr/bin/env python3
"""Read-only P7.2b Gazebo/PX4 axis capture at bounded ROS sample times."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import String

from uav_control.evaluation.gazebo_entity_pose import (
    GazeboEntityPoseTracker, TimedPoseHistory, gazebo_enu_to_ned,
)
from uav_control.evaluation.pose_frame_diagnostics import (
    gazebo_model_rotation_ned_frd, interpolate_model_pose,
    interpolate_px4_attitude, query_crosses_reset, query_metadata,
    quaternion_rotation,
)
from uav_control.perception.rgbd_target_localizer import Px4RosClockMapper


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nsec) * 1e-9


def interpolate_position(left, right, fraction):
    position_velocity = tuple(
        a + fraction * (b - a) for a, b in zip(left[:6], right[:6])
    )
    yaw_delta = math.atan2(
        math.sin(right[6] - left[6]), math.cos(right[6] - left[6]),
    )
    heading = math.atan2(
        math.sin(left[6] + fraction * yaw_delta),
        math.cos(left[6] + fraction * yaw_delta),
    )
    return position_velocity + (heading,) + tuple(left[7:])


def position_sample(message):
    return (
        float(message.x), float(message.y), float(message.z),
        float(message.vx), float(message.vy), float(message.vz),
        float(message.heading), int(message.xy_reset_counter),
        int(message.z_reset_counter), int(message.heading_reset_counter),
        int(message.ref_timestamp), float(message.ref_lat),
        float(message.ref_lon), float(message.ref_alt),
    )


def _clean(value):
    if isinstance(value, dict):
        return {key: _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _valid_status(query):
    return query.value is not None and query.status in (
        'EXACT', 'INTERPOLATED')


class AxisCapture(Node):
    """Observe one experiment, without any PX4 command publisher."""

    def __init__(self, output, sample_rate=10.0, sample_lag=0.06,
                 maximum_gap=0.05):
        super().__init__('p7_px4_local_axis_capture')
        self.output = Path(output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output.open('x', encoding='utf-8', buffering=1)
        self.sample_lag = float(sample_lag)
        self.maximum_gap = float(maximum_gap)
        self.clock_tracker = GazeboEntityPoseTracker(
            history_duration=5.0, clock_history_duration=5.0,
        )
        self.model_history = TimedPoseHistory(
            maximum_age=5.0, interpolator=interpolate_model_pose,
        )
        self.px4_mapper = Px4RosClockMapper(source_is_ros_time=True)
        self.local_history = TimedPoseHistory(
            maximum_age=5.0, interpolator=interpolate_position,
        )
        self.ground_history = TimedPoseHistory(
            maximum_age=5.0, interpolator=interpolate_position,
        )
        self.attitude_history = TimedPoseHistory(
            maximum_age=5.0, interpolator=interpolate_px4_attitude,
        )
        self.phase = 'WAITING'
        self.last_command_event = None
        self.nav_state = None
        self.arming_state = None
        self.failsafe = None
        self.epoch = 0
        self.last_reset_key = None
        self.last_sample_stamp = None
        self.rows = 0
        self.valid_rows = 0
        self.groundtruth_messages = 0
        self.done_samples = 0
        self.capture_complete = False
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            lambda msg: self.add_position('local', msg), sensor_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_groundtruth',
            lambda msg: self.add_position('groundtruth', msg), sensor_qos,
        )
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.add_attitude, sensor_qos,
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.add_status, sensor_qos,
        )
        self.create_subscription(
            String, '/p7/axis/phase', self.add_phase, 10,
        )
        # The standard lab console requires two X-command subscribers.
        self.create_subscription(
            String, '/simulation/impact/command',
            self.record_command, 10,
        )
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GazeboNode
        self.gazebo_node = GazeboNode()
        if not self.gazebo_node.subscribe(
                Clock, '/world/default/clock', self.add_clock):
            raise RuntimeError('Gazebo clock subscription failed')
        if not self.gazebo_node.subscribe(
                Pose_V, '/world/default/pose/info', self.add_pose):
            raise RuntimeError('Gazebo pose subscription failed')
        self.create_timer(1.0 / sample_rate, self.sample)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def add_clock(self, message):
        old = self.clock_tracker.reset_count
        self.clock_tracker.add_clock_anchor(
            stamp_seconds(message.sim), stamp_seconds(message.system),
            self.now(), time.monotonic(),
        )
        if self.clock_tracker.reset_count != old:
            self.model_history.clear()

    def add_pose(self, message):
        model = next((pose for pose in message.pose
                      if str(pose.name).startswith('x500_mono_cam_')
                      and '::' not in str(pose.name)), None)
        if model is None:
            return
        sim_stamp = stamp_seconds(message.header.stamp)
        position = (model.position.x, model.position.y, model.position.z)
        if self.clock_tracker.add_pose(sim_stamp, position, self.now()):
            self.model_history.add(
                sim_stamp, self.clock_tracker.last_mapped_stamp,
                position + (model.orientation.w, model.orientation.x,
                            model.orientation.y, model.orientation.z),
            )

    def _mapped_px4_time(self, stream, message):
        source = float(message.timestamp_sample or message.timestamp) * 1e-6
        old = self.px4_mapper.reset_count
        mapped = self.px4_mapper.to_ros_time(
            source, self.now(), stream_name=stream,
        )
        if self.px4_mapper.reset_count != old:
            self.local_history.clear()
            self.ground_history.clear()
            self.attitude_history.clear()
        return source, mapped

    def add_position(self, stream, message):
        value = position_sample(message)
        if not all(math.isfinite(x) for x in value):
            return
        source, mapped = self._mapped_px4_time(stream, message)
        if mapped is not None:
            if stream == 'groundtruth':
                self.groundtruth_messages += 1
            history = (self.local_history if stream == 'local'
                       else self.ground_history)
            history.add(source, mapped, value)

    def add_attitude(self, message):
        value = tuple(float(x) for x in message.q) + (
            int(message.quat_reset_counter),)
        if not all(math.isfinite(x) for x in value):
            return
        source, mapped = self._mapped_px4_time('attitude', message)
        if mapped is not None:
            self.attitude_history.add(source, mapped, value)

    def add_status(self, message):
        self.nav_state = int(message.nav_state)
        self.arming_state = int(message.arming_state)
        self.failsafe = bool(message.failsafe)

    def add_phase(self, message):
        self.phase = str(message.data)

    def record_command(self, message):
        self.last_command_event = str(message.data)

    def sample(self):
        stamp = self.now() - self.sample_lag
        model = self.model_history.query_at(stamp)
        local = self.local_history.query_at(stamp)
        ground = self.ground_history.query_at(stamp)
        attitude = self.attitude_history.query_at(stamp)
        queries = {'model': model, 'position': local,
                   'groundtruth': ground, 'attitude': attitude}
        row = {
            'ros_stamp': stamp, 'phase': self.phase,
            'last_command_event': self.last_command_event,
            'clock_reset_count': self.clock_tracker.reset_count,
            'px4_clock_reset_count': self.px4_mapper.reset_count,
            'px4_clock_status': self.px4_mapper.last_status,
            'nav_state': self.nav_state,
            'arming_state': self.arming_state,
            'failsafe': self.failsafe,
        }
        for name, query in queries.items():
            row.update(query_metadata(name, query))
        reasons = []
        gaps = []
        for name in ('model', 'position', 'attitude'):
            query = queries[name]
            if not _valid_status(query):
                reasons.append(f'{name}:{query.status}')
            else:
                gaps.extend((stamp - query.left_ros_stamp,
                             query.right_ros_stamp - stamp))
        if gaps and max(gaps) > self.maximum_gap:
            reasons.append('alignment_gap_exceeded')
        if _valid_status(local) and any(
                query_crosses_reset(local, index) for index in (7, 8, 9, 10)):
            reasons.append('position_reset_crossed')
        if _valid_status(attitude) and query_crosses_reset(attitude, 4):
            reasons.append('attitude_reset_crossed')
        if self.phase == 'WAITING':
            reasons.append('no_phase')
        reset_key = [self.clock_tracker.reset_count,
                     self.px4_mapper.reset_count]
        if _valid_status(local):
            reset_key.extend(local.value[7:11])
        if _valid_status(attitude):
            reset_key.append(attitude.value[4])
        if self.last_reset_key is not None and (
                reset_key != self.last_reset_key or
                stamp <= self.last_sample_stamp):
            self.epoch += 1
        if (self.last_sample_stamp is not None
                and stamp <= self.last_sample_stamp):
            reasons.append('timestamp_reversal')
        self.last_reset_key = reset_key
        self.last_sample_stamp = stamp
        row['reset_key'] = reset_key
        row['reset_epoch'] = self.epoch
        row['alignment_gap_s'] = max(gaps) if gaps else None
        row['valid'] = not reasons
        row['invalid_reason'] = '|'.join(reasons)
        if _valid_status(model):
            value = model.value
            row['gazebo_uav_position_enu'] = list(value[:3])
            row['gazebo_uav_position_ned'] = list(gazebo_enu_to_ned(
                value[:3]))
            row['gazebo_uav_attitude_enu_flu_wxyz'] = list(value[3:])
            row['sim_stamp'] = (model.left_sim_stamp + model.fraction
                                * (model.right_sim_stamp
                                   - model.left_sim_stamp))
            rotation = gazebo_model_rotation_ned_frd(value[3:])
            row['gazebo_yaw_ned'] = math.atan2(
                rotation[1, 0], rotation[0, 0])
        if _valid_status(local):
            value = local.value
            row['px4_local_position_ned'] = list(value[:3])
            row['px4_local_velocity_ned'] = list(value[3:6])
            row['px4_heading_ned'] = value[6]
            row['xy_reset_counter'] = value[7]
            row['z_reset_counter'] = value[8]
            row['heading_reset_counter'] = value[9]
            row['px4_ref_timestamp'] = value[10]
            row['px4_ref_lat'] = value[11]
            row['px4_ref_lon'] = value[12]
            row['px4_ref_alt'] = value[13]
        if _valid_status(attitude):
            row['px4_attitude_ned_frd_wxyz'] = list(attitude.value[:4])
            row['quat_reset_counter'] = attitude.value[4]
            rotation = quaternion_rotation(attitude.value[:4])
            row['px4_yaw_ned'] = math.atan2(
                rotation[1, 0], rotation[0, 0])
        if _valid_status(ground):
            row['px4_groundtruth_position_ned'] = list(ground.value[:3])
            row['px4_groundtruth_velocity_ned'] = list(ground.value[3:6])
        self.stream.write(json.dumps(_clean(row), allow_nan=False) + '\n')
        self.rows += 1
        self.valid_rows += row['valid']
        if self.phase == 'DONE':
            self.done_samples += 1
            if self.done_samples >= 10:
                self.capture_complete = True

    def close(self):
        self.stream.close()
        summary = {'rows': self.rows, 'valid_rows': self.valid_rows,
                   'groundtruth_topic_observed': (
                       self.groundtruth_messages > 0)}
        with self.output.with_suffix('.summary.json').open('x') as stream:
            json.dump(summary, stream, indent=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sample-rate', type=float, default=10.0)
    parser.add_argument('--max-gap', type=float, default=0.05)
    args = parser.parse_args()
    rclpy.init()
    node = AxisCapture(args.output, args.sample_rate,
                       maximum_gap=args.max_gap)
    try:
        while rclpy.ok() and not node.capture_complete:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

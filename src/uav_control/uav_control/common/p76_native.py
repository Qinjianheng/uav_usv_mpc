"""Physical odometry attitude validation and bounded, causal interpolation."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def odometry_values(message):
    """Require native sample identity, NED attitude and known FRD angular uncertainty."""
    stamp = int(message.timestamp_sample)
    if stamp <= 0:
        raise ValueError('P76_NATIVE_SAMPLE_REQUIRED')
    if int(message.pose_frame) != 1:
        raise ValueError('P76_NATIVE_NED_REQUIRED')
    q, omega, variance = (np.asarray(v, dtype=float) for v in (
        message.q, message.angular_velocity, message.orientation_variance))
    if (q.shape != (4,) or not np.isfinite(q).all()
            or abs(np.linalg.norm(q)-1.) > .01):
        raise ValueError('P76_NATIVE_QUATERNION')
    if omega.shape != (3,) or not np.isfinite(omega).all():
        raise ValueError('P76_NATIVE_ANGULAR_VELOCITY')
    if variance.shape != (3,) or not np.isfinite(variance).all() or np.any(variance < 0):
        raise ValueError('P76_NATIVE_VARIANCE')
    return stamp, q/np.linalg.norm(q), omega, variance


@dataclass(frozen=True)
class NativePose:
    """Attitude and rates at an actual or bracketed physical acquisition epoch."""

    stamp: float
    rotation: np.ndarray
    quaternion: np.ndarray
    omega: np.ndarray
    variance: np.ndarray


class NativePoseHistory:
    """No latest-offset reconstruction, prediction, or navigation extrapolation."""

    def __init__(self, maximum_bracket=.05):
        self.maximum_bracket = float(maximum_bracket)
        self.samples = deque(maxlen=256)

    def clear(self):
        """Discard all samples across a clock/frame epoch change."""
        self.samples.clear()

    def add(self, stamp, quaternion, omega, variance):
        """Retain ordered, physical epochs only."""
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('P76_NATIVE_EPOCH')
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        values = tuple(np.asarray(v, dtype=float) for v in (quaternion, omega, variance))
        if not all(np.isfinite(v).all() for v in values):
            raise ValueError('P76_NATIVE_VALUES')
        if (values[0].shape != (4,) or abs(np.linalg.norm(values[0])-1) > .01
                or values[1].shape != (3,) or values[2].shape != (3,)
                or np.any(values[2] < 0)):
            raise ValueError('P76_NATIVE_VALUES')
        self.samples.append((float(stamp), *values))
        return True

    def at(self, stamp):
        """Interpolate only inside an existing at-most-50-ms bracket."""
        for sample in self.samples:
            if abs(sample[0]-stamp) < 1e-9:
                q, omega, variance = sample[1:]
                return NativePose(stamp, Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix(),
                                  q.copy(), omega.copy(), variance.copy())
        for left, right in zip(self.samples, list(self.samples)[1:]):
            if left[0] < stamp < right[0] and right[0]-left[0] <= self.maximum_bracket+1e-9:
                weight = (stamp-left[0])/(right[0]-left[0])
                rotations = Rotation.from_quat(np.array([left[1], right[1]])[:, [1, 2, 3, 0]])
                rotation = Slerp([left[0], right[0]], rotations)([stamp])[0]
                return NativePose(stamp, rotation.as_matrix(), rotation.as_quat()[[3, 0, 1, 2]],
                                  (1-weight)*left[2]+weight*right[2],
                                  (1-weight)*left[3]+weight*right[3])
        raise ValueError('P76_NATIVE_POSE_BRACKET')


class NativeReferenceHistory:
    """Retain PX4's actual altitude datum; no sea truth lookup or state extrapolation."""

    def __init__(self):
        self.samples = deque(maxlen=256)

    def add(self, stamp, ref_alt):
        """Save strictly ordered, native-mapped local-frame datum samples."""
        if not all(math.isfinite(v) for v in (stamp, ref_alt)) or stamp <= 0:
            raise ValueError('P76_LOCAL_REFERENCE_INVALID')
        if not self.samples or stamp > self.samples[-1][0]:
            self.samples.append((stamp, ref_alt))

    def at(self, stamp):
        """Require an exact epoch or <=50 ms bracket with an unchanged frame datum."""
        for source, ref_alt in self.samples:
            if abs(source-stamp) < 1e-9:
                return ref_alt
        for left, right in zip(self.samples, list(self.samples)[1:]):
            if left[0] < stamp < right[0] and right[0]-left[0] <= .05+1e-9:
                if abs(left[1]-right[1]) > 1e-6:
                    raise ValueError('P76_LOCAL_REFERENCE_CHANGED')
                return left[1]
        raise ValueError('P76_LOCAL_REFERENCE_BRACKET')


def install_odometry_relay(node):
    """Reuse the localizer's causal clock map without replacing native sample epochs."""
    from px4_msgs.msg import (
        VehicleControlMode, VehicleLocalPosition, VehicleOdometry, VehicleThrustSetpoint,
    )
    from rclpy.qos import qos_profile_sensor_data
    from copy import copy
    from uav_usv_interfaces.msg import NativeAttitude
    from builtin_interfaces.msg import Time
    from std_msgs.msg import String
    import json
    node.native_attitude_pub = node.create_publisher(
        NativeAttitude, '/navigation/native_attitude', 10)

    def receive(message):
        if getattr(node, 'navigation_frame_fault', ''):
            return
        try:
            wire, q, omega, variance = odometry_values(message)
            now = node._ros_seconds()
            mapped, raw, _ = node._map_px4_pose_stamp(wire*1e-6, now)
            if mapped is None or not 0 <= now-mapped <= .125:
                return
            if node._observe_pose_reset('ODOMETRY', message, raw):
                return
            result = NativeAttitude()
            ns = round(mapped*1e9)
            result.stamp = Time(sec=ns//10**9, nanosec=ns % 10**9)
            result.px4_timestamp_sample = wire
            result.native_timestamp_sample = round(raw*1e6)
            result.quaternion_wxyz = q.tolist()
            result.angular_velocity_frd = omega.tolist()
            result.orientation_variance = variance.tolist()
            result.valid = True
            node.native_attitude_pub.publish(result)
        except (ValueError, TypeError, OverflowError):
            return
    native_qos = copy(qos_profile_sensor_data)
    native_qos.depth = 16
    node.native_odometry_sub = node.create_subscription(
        VehicleOdometry, '/fmu/out/vehicle_odometry', receive, native_qos)
    node.native_thrust_pub = node.create_publisher(String, '/navigation/native_thrust', 10)
    node.native_mode_pub = node.create_publisher(String, '/navigation/px4_control_mode', 10)
    node.native_reference_pub = node.create_publisher(
        String, '/navigation/native_local_reference', 10)

    def reference(message):
        if (getattr(node, 'navigation_frame_fault', '') or not message.z_global
                or not message.z_valid or not math.isfinite(message.ref_alt)):
            return
        wire = int(message.timestamp_sample)
        now = node._ros_seconds()
        mapped, raw, _ = node._map_px4_pose_stamp(wire*1e-6, now)
        if wire <= 0 or mapped is None or not 0 <= now-mapped <= .125:
            return
        node.native_reference_pub.publish(String(data=json.dumps(dict(
            stamp=mapped, native_timestamp=round(raw*1e6), px4_timestamp=wire,
            ref_alt=float(message.ref_alt), reference='base_link'))))

    node.native_reference_sub = node.create_subscription(
        VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', reference, native_qos)

    def relay(message, is_thrust):
        if getattr(node, 'navigation_frame_fault', ''):
            return
        wire = int(message.timestamp_sample if is_thrust else message.timestamp)
        if wire <= 0:
            return
        now = node._ros_seconds()
        mapped, raw, _ = node._map_px4_pose_stamp(wire*1e-6, now)
        if mapped is None or not 0 <= now-mapped <= .125:
            return
        packet = dict(stamp=mapped, native_timestamp=round(raw*1e6), px4_timestamp=wire)
        if is_thrust:
            if not all(math.isfinite(value) for value in message.xyz):
                return
            packet['xyz'] = [float(value) for value in message.xyz]
        else:
            packet['flags'] = {key: bool(getattr(message, 'flag_control_'+key+'_enabled'))
                               for key in ('offboard', 'rates', 'position', 'velocity',
                                           'attitude', 'altitude', 'climb_rate')}
        publisher = node.native_thrust_pub if is_thrust else node.native_mode_pub
        publisher.publish(String(data=json.dumps(packet)))
    node.native_thrust_sub = node.create_subscription(
        VehicleThrustSetpoint, '/fmu/out/vehicle_thrust_setpoint',
        lambda message: relay(message, True), qos_profile_sensor_data)
    node.native_control_mode_sub = node.create_subscription(
        VehicleControlMode, '/fmu/out/vehicle_control_mode',
        lambda message: relay(message, False), qos_profile_sensor_data)

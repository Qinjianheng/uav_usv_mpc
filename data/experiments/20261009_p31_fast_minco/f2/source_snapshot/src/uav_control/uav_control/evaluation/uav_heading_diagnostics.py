"""Acquisition-aligned UAV heading reference for evaluation only."""

from collections import deque
import math
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from .gazebo_entity_pose import GazeboEntityPoseTracker, gazebo_enu_to_ned
from .gazebo_entity_pose import TimedPoseHistory
from .pose_frame_diagnostics import (
    four_pose_counterfactuals, gazebo_model_rotation_ned_frd,
    interpolate_model_pose, quaternion_rotation, query_metadata,
)


_METADATA = ('query_ros_stamp', 'status', 'left_sim_stamp', 'right_sim_stamp',
             'left_ros_stamp', 'right_ros_stamp', 'fraction', 'interval')
UAV_HEADING_CSV_FIELDS = (
    'gazebo_uav_diagnostics_enabled', 'heading_diagnostics_status',
    'heading_reference_source', 'uav_position_reference_status',
    'px4_heading', 'reference_heading', 'delta_yaw',
    'predicted_lateral_error', 'actual_lateral_error',
    'heading_induced_error_x', 'heading_induced_error_y',
    'uav_reference_position_x', 'uav_reference_position_y',
    'uav_reference_position_z', 'uav_reference_attitude_w',
    'uav_reference_attitude_x', 'uav_reference_attitude_y',
    'uav_reference_attitude_z', 'attitude_counterfactual_status',
    'attitude_counterfactual_error_x', 'attitude_counterfactual_error_y',
    'attitude_counterfactual_error_z',
) + tuple(f'uav_pose_{name}' for name in _METADATA)


class GazeboUavPoseTracker(GazeboEntityPoseTracker):
    """
    Keep exact-model ENU/FLU poses; map only with causal clock brackets.

    Model position remains the Gazebo model origin. Its equivalence with PX4
    local position has not been established and is never assumed by heading
    diagnostics. Receipt time only limits the pending wait, never stamps poses.
    """

    def __init__(self, history_duration=1.0, maximum_reference_age=0.5,
                 clock_history_duration=2.0, maximum_system_clock_step=0.25,
                 pose_wait_timeout=0.15, model_name='x500_mono_cam_0'):
        super().__init__(history_duration, maximum_reference_age,
                         clock_history_duration, maximum_system_clock_step)
        if not str(model_name).strip():
            raise ValueError('UAV model name must be explicit')
        self.model_name = str(model_name)
        self.history = TimedPoseHistory(
            history_duration, interpolate_model_pose)
        self.lock = threading.RLock()
        self.pose_wait_timeout = float(pose_wait_timeout)
        if (not math.isfinite(self.pose_wait_timeout)
                or self.pose_wait_timeout < 0):
            raise ValueError(
                'pose wait timeout must be finite and nonnegative')
        self.pending_poses = deque(maxlen=256)
        self.pending_pose_timeout_count = 0

    def matches_model(self, name):
        """Accept the exact model, without prefixes or nested links."""
        return name == self.model_name

    def add_clock_anchor(self, sim_time, system_time, receipt_ros_time,
                         monotonic_time):
        with self.lock:
            before = self.reset_count
            accepted = super().add_clock_anchor(
                sim_time, system_time, receipt_ros_time, monotonic_time)
            if self.reset_count != before:
                self.pending_poses.clear()
                self.last_raw_stamp = self.last_mapped_stamp = math.nan
            elif accepted:
                pending = tuple(self.pending_poses)
                self.pending_poses.clear()
                for raw, value, receipt in pending:
                    if (receipt_ros_time - receipt
                            > self.pose_wait_timeout + 1e-9):
                        self.pending_pose_timeout_count += 1
                        self.last_status = 'UAV_POSE_CLOCK_WAIT_TIMEOUT'
                    else:
                        self._store_or_wait(
                            raw, value, receipt_ros_time, receipt)
            return accepted

    def _store_or_wait(self, sim_stamp, value, now_ros_time, receipt):
        mapped, status = self.clock_mapper.map_time(sim_stamp, now_ros_time)
        self.last_raw_stamp = sim_stamp
        self.last_status = status
        if mapped is None:
            if status in ('CLOCK_REFERENCE_UNAVAILABLE',
                          'CLOCK_REFERENCE_WARMING_UP',
                          'IMAGE_AFTER_CLOCK_REFERENCE'):
                self.pending_poses.append((sim_stamp, value, receipt))
            return False
        if not self.history.add(sim_stamp, mapped, value):
            self.last_status = 'UAV_POSE_OUT_OF_ORDER'
            return False
        self.last_mapped_stamp = mapped
        return True

    def add_pose(self, sim_stamp, gazebo_enu_position, orientation_wxyz,
                 now_ros_time):
        """Store quaternion and position with the native sample stamp."""
        try:
            values = tuple(float(x) for x in gazebo_enu_position)
            quaternion = tuple(float(x) for x in orientation_wxyz)
            raw, receipt = float(sim_stamp), float(now_ros_time)
            if len(values) != 3 or len(quaternion) != 4:
                raise ValueError('pose dimensions are invalid')
            if not all(math.isfinite(x)
                       for x in values + quaternion + (raw, receipt)):
                raise ValueError('pose must be finite')
            quaternion_rotation(quaternion)
            norm = math.sqrt(sum(x * x for x in quaternion))
            value = values + tuple(x / norm for x in quaternion)
        except (ValueError, TypeError):
            self.last_status = 'UAV_POSE_INVALID'
            return False
        with self.lock:
            return self._store_or_wait(raw, value, receipt, receipt)


def heading_diagnostics(observation, query, error_ned=None,
                        entity_center_ned=None):
    """
    Flatten a trusted UAV heading bracket without any online correction.

    The attitude-only counterfactual holds PX4 position and measured camera
    vector fixed. A full independent position counterfactual is deliberately
    withheld until model-origin to PX4-position frame equivalence is verified.
    """
    row = {name: math.nan for name in UAV_HEADING_CSV_FIELDS}
    row.update(
        gazebo_uav_diagnostics_enabled=query is not None,
        heading_diagnostics_status='DISABLED', heading_reference_source='none',
        uav_position_reference_status='MODEL_ORIGIN_FRAME_NOT_ESTABLISHED',
        attitude_counterfactual_status='NOT_EVALUATED',
        uav_pose_status='DISABLED',
    )
    if query is None:
        return row
    row.update(query_metadata('uav_pose', query))
    if not bool(observation.geometry_diagnostics_enabled):
        row['heading_diagnostics_status'] = 'GEOMETRY_DISABLED'
        return row
    if query.value is None or query.status not in ('EXACT', 'INTERPOLATED'):
        row['heading_diagnostics_status'] = 'REFERENCE_' + query.status
        return row
    if not bool(observation.valid):
        row['heading_diagnostics_status'] = 'OBSERVATION_INVALID'
        return row
    try:
        px4_q = tuple(getattr(observation, f'interpolated_attitude_{axis}')
                      for axis in ('w', 'x', 'y', 'z'))
        px4_rotation = quaternion_rotation(px4_q)
        reference_rotation = gazebo_model_rotation_ned_frd(query.value[3:])
        px4_heading = math.atan2(px4_rotation[1, 0], px4_rotation[0, 0])
        reference_heading = math.atan2(reference_rotation[1, 0],
                                       reference_rotation[0, 0])
        delta = math.atan2(math.sin(px4_heading - reference_heading),
                           math.cos(px4_heading - reference_heading))
        target_range = float(observation.target_range)
        if not math.isfinite(target_range) or target_range <= 0:
            raise ValueError('target range must be positive and finite')
        position = gazebo_enu_to_ned(query.value[:3])
        x, y, z, w = Rotation.from_matrix(reference_rotation).as_quat()
        reference_q = (w, x, y, z)
        row.update(
            heading_diagnostics_status='VALID',
            heading_reference_source='gazebo_uav_model_pose',
            px4_heading=px4_heading, reference_heading=reference_heading,
            delta_yaw=delta,
            predicted_lateral_error=target_range * math.sin(delta),
        )
        for axis, value in zip(('x', 'y', 'z'), position):
            row['uav_reference_position_' + axis] = value
        for axis, value in zip(('w', 'x', 'y', 'z'), reference_q):
            row['uav_reference_attitude_' + axis] = value
        rel_x = observation.position.x - observation.interpolated_uav_x
        rel_y = observation.position.y - observation.interpolated_uav_y
        cosine, sine = math.cos(delta), math.sin(delta)
        row['heading_induced_error_x'] = (
            rel_x - (cosine * rel_x + sine * rel_y))
        row['heading_induced_error_y'] = (
            rel_y - (-sine * rel_x + cosine * rel_y))
        if error_ned is not None and all(math.isfinite(x) for x in error_ned):
            row['actual_lateral_error'] = (
                -math.sin(reference_heading) * error_ned[0]
                + math.cos(reference_heading) * error_ned[1])
    except (ValueError, TypeError, AttributeError):
        row['heading_diagnostics_status'] = 'GEOMETRY_OR_REFERENCE_INVALID'
        return row
    if entity_center_ned is not None:
        try:
            camera = tuple(getattr(observation, f'center_camera_{a}')
                           for a in ('x', 'y', 'z'))
            px4_position = tuple(getattr(observation, f'interpolated_uav_{a}')
                                 for a in ('x', 'y', 'z'))
            translation = tuple(getattr(observation, f'camera_translation_{a}')
                                for a in ('x', 'y', 'z'))
            offset = float(observation.target_reference_z_offset)
            poses = four_pose_counterfactuals(
                camera, px4_position, px4_position, px4_q, reference_q,
                translation, observation.camera_pitch_down, offset)
            expected = (
                np.asarray(entity_center_ned) + np.array((0., 0., offset)))
            error = np.asarray(poses['D']) - expected
            if not np.all(np.isfinite(error)):
                raise ValueError('counterfactual must be finite')
            row['attitude_counterfactual_status'] = 'PX4_POSITION_HELD_FIXED'
            for axis, value in zip(('x', 'y', 'z'), error):
                row['attitude_counterfactual_error_' + axis] = float(value)
        except (ValueError, TypeError, AttributeError):
            row['attitude_counterfactual_status'] = 'INSUFFICIENT_GEOMETRY'
    return row

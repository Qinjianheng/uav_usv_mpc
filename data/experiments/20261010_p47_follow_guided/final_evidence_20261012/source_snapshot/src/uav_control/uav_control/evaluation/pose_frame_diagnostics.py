"""Independent full-attitude comparisons for Gazebo and PX4 evaluation."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from uav_control.perception.rgbd_target_localizer import (
    camera_target_to_local_ned, quaternion_slerp,
)


def interpolate_model_pose(left, right, fraction):
    """Interpolate Gazebo world position linearly and attitude on SO(3)."""
    position = tuple(
        a + fraction * (b - a) for a, b in zip(left[:3], right[:3])
    )
    return position + tuple(quaternion_slerp(
        left[3:], right[3:], fraction,
    ))


def query_metadata(prefix, query):
    """Flatten query-specific source times for one CSV observation row."""
    return {
        f'{prefix}_query_ros_stamp': query.query_ros_stamp,
        f'{prefix}_status': query.status,
        f'{prefix}_left_sim_stamp': query.left_sim_stamp,
        f'{prefix}_right_sim_stamp': query.right_sim_stamp,
        f'{prefix}_left_ros_stamp': query.left_ros_stamp,
        f'{prefix}_right_ros_stamp': query.right_ros_stamp,
        f'{prefix}_fraction': query.fraction,
        f'{prefix}_interval': query.interval,
    }


def interpolate_px4_attitude(left, right, fraction):
    return tuple(quaternion_slerp(left[:4], right[:4], fraction)) + tuple(
        left[4:]
    )


def interpolate_px4_position(left, right, fraction):
    position = tuple(
        a + fraction * (b - a) for a, b in zip(left[:3], right[:3])
    )
    heading_delta = math.atan2(
        math.sin(right[3] - left[3]), math.cos(right[3] - left[3]),
    )
    heading = math.atan2(
        math.sin(left[3] + fraction * heading_delta),
        math.cos(left[3] + fraction * heading_delta),
    )
    return position + (heading,) + tuple(left[4:])


def query_crosses_reset(query, counter_index):
    return bool(
        query.left_value is not None and query.right_value is not None
        and query.left_value[counter_index]
        != query.right_value[counter_index]
    )


def quaternion_rotation(quaternion):
    """Return the active rotation matrix for a Hamilton wxyz quaternion."""
    values = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(values)
    if values.shape != (4,) or not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError('orientation quaternion must be finite and nonzero')
    w, x, y, z = values / norm
    return np.array((
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)),
    ))


def gazebo_model_rotation_ned_frd(gazebo_quaternion):
    """Convert Gazebo model FLU->ENU orientation into PX4 FRD->NED."""
    enu_to_ned = np.array((
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
    ))
    frd_to_flu = np.diag((1.0, -1.0, -1.0))
    return enu_to_ned @ quaternion_rotation(gazebo_quaternion) @ frd_to_flu


def rotation_residual_rpy(actual, reference):
    """Return roll, pitch, yaw of actual @ reference.T in the NED axes."""
    delta = np.asarray(actual, dtype=float) @ np.asarray(reference).T
    pitch = math.asin(float(np.clip(-delta[2, 0], -1.0, 1.0)))
    return (
        math.atan2(delta[2, 1], delta[2, 2]),
        pitch,
        math.atan2(delta[1, 0], delta[0, 0]),
    )


def rotation_angle_between_quaternions(actual, reference):
    """Geodesic angle; invalid rejected-observation quaternions yield NaN."""
    try:
        delta = (
            quaternion_rotation(actual) @ quaternion_rotation(reference).T
        )
    except ValueError:
        return math.nan
    return math.acos(float(np.clip(
        (np.trace(delta) - 1.0) / 2.0, -1.0, 1.0,
    )))


def model_quaternion_from_residual(px4_quaternion, residual_rpy):
    """Recover model NED/FRD quaternion from logged full rotation residual."""
    residual = np.asarray(residual_rpy, dtype=float)
    if residual.shape != (3,) or not np.all(np.isfinite(residual)):
        raise ValueError('rotation residual must be a finite 3-vector')
    model_rotation = (
        Rotation.from_euler('xyz', residual).as_matrix()
        @ quaternion_rotation(px4_quaternion)
    )
    x, y, z, w = Rotation.from_matrix(model_rotation).as_quat()
    return (w, x, y, z)


def four_pose_counterfactuals(
    camera_vector, px4_position, model_position,
    px4_quaternion, model_quaternion,
    camera_translation, camera_pitch_down, target_reference_z_offset,
):
    """Run the online camera-to-NED function with four offline poses."""
    poses = {
        'A': (px4_position, px4_quaternion),
        'B': (model_position, model_quaternion),
        'C': (model_position, px4_quaternion),
        'D': (px4_position, model_quaternion),
    }
    return {
        key: camera_target_to_local_ned(
            camera_vector, position, quaternion, camera_translation,
            camera_pitch_down, target_reference_z_offset,
        )
        for key, (position, quaternion) in poses.items()
    }

"""Dedicated FOLLOW proposal conversion; no intercept epochs or control publication."""
from uav_control.guidance.follow_profile import profiled

from dataclasses import asdict
import math
import json

import numpy as np
from builtin_interfaces.msg import Time
from uav_usv_interfaces.msg import FollowTrajectory, FollowPlanAck

from uav_control.guidance.follow_contract import FollowCurve


def seconds(stamp):
    """Retain the original time epoch."""
    return stamp.sec+stamp.nanosec/1e9


def stamp(value):
    """Encode a finite nonnegative absolute time, including zero unqualified holding."""
    if not math.isfinite(value) or value < 0:
        raise ValueError('INVALID_STAMP')
    whole = int(value)
    ns = round((value-whole)*1e9)
    return Time(sec=whole+ns//1_000_000_000, nanosec=ns % 1_000_000_000)


EPOCH_FIELDS = {'navigation_stamp': 'navigation_stamp', 'attitude_stamp': 'attitude_stamp',
                'observation_stamp': 'observation_stamp', 'source_stamp': 'source_stamp',
                'input_until': 'input_valid_until', 'coverage_until': 'prediction_coverage_until',
                'start': 'execution_start_stamp', 'end': 'execution_end_stamp',
                'holding_until': 'holding_valid_until'}


def curve_from_message(message):
    """Reject missing/mismatched dimensions before reconstructing complete coefficients."""
    n = len(message.durations)
    if (not 0 < n <= 16 or len(message.xyz_coefficients) != 18*n
            or len(message.yaw_coefficients) != 4*n or not message.nominal_sampled_valid):
        raise ValueError('INVALID_COEFFICIENTS')
    return FollowCurve(int(message.plan_id), int(message.mission_id),
                       int(message.clock_generation), int(message.prediction_sequence_id),
                       int(message.parent_plan_id),
                       **{k: seconds(getattr(message, v)) for k, v in EPOCH_FIELDS.items()},
                       durations=tuple(message.durations),
                       xyz=tuple(map(tuple, np.asarray(message.xyz_coefficients).reshape(-1, 3))),
                       yaw=tuple(map(tuple, np.asarray(message.yaw_coefficients).reshape(-1, 4))),
                       frame=message.frame_id, constraint_version=message.constraint_version,
                       camera_version=message.camera_version,
                       holding_model=message.holding_model_id,
                       receiver_boot_id=message.receiver_boot_id,
                       planner_boot_id=message.planner_boot_id,
                       constraint_snapshot=message.constraint_snapshot,
                       constraint_fingerprint=message.constraint_fingerprint)


@profiled('message_conversion')
def proposal_from_event(event, now):
    """Publish nominal research coefficients with explicit missing holding/first bridge."""
    result, req = event['output'], event['request']
    if not req or not result.get('valid') or not result.get('metrics', {}).get(
            'full_projection_validated'):
        return None
    c, metrics = req['context'], result['metrics']
    durations = metrics['durations']
    start = c['execution_start_stamp']
    xyz = np.asarray(metrics['xyz_coefficients'])
    yaw = np.asarray(metrics['yaw_coefficients'])
    if xyz.shape != (len(durations), 6, 3) or yaw.shape != (4, len(durations)):
        raise ValueError('INVALID_COEFFICIENT_LAYOUT')
    msg = FollowTrajectory()
    msg.plan_id, msg.mission_id = c['cycle_id'], c['mission_id']
    msg.clock_generation = c['clock_generation']
    msg.prediction_sequence_id = c['prediction_sequence_id']
    values = dict(navigation_stamp=c['navigation_stamp'], attitude_stamp=c['attitude_stamp'],
                  observation_stamp=c['observation_stamp'],
                  source_stamp=c['prediction_source_stamp'],
                  input_until=min(c['navigation_stamp']+.125, c['observation_stamp']+.125,
                                  c['prediction_source_stamp']+.125, c['prediction_valid_until']),
                  coverage_until=c['prediction_source_stamp']+req['prediction_times'][-1],
                  start=start, end=start+sum(durations), holding_until=0.)
    if now >= values['input_until'] or now >= start:
        return None
    for name, field in EPOCH_FIELDS.items():
        setattr(msg, field, stamp(values[name]))
    msg.published_stamp = stamp(now)
    msg.frame_id, msg.constraint_version, msg.camera_version = (
        c['frame_id'], 'constraints-p4-v1', 'camera-p1-v1')
    msg.planner_type, msg.validation_state = 'progress_minco', 'VALIDATED_RESEARCH'
    msg.durations = list(durations)
    msg.xyz_coefficients = xyz.ravel().tolist()
    msg.yaw_coefficients = yaw[::-1].T.ravel().tolist()
    msg.constraint_snapshot = metrics.get('constraint_snapshot', '')
    msg.constraint_fingerprint = metrics.get('constraint_fingerprint', '')
    policy = json.loads(msg.constraint_snapshot)['mpc'] if msg.constraint_snapshot else {}
    msg.dynamic_limits = [policy.get('maximum_'+name, default) for name, default in (
        ('horizontal_speed', 6.2), ('vertical_speed', 4.), ('horizontal_acceleration', 3.),
        ('vertical_acceleration', 3.), ('horizontal_jerk', 6.), ('vertical_jerk', 4.))]
    msg.fov_margins = [metrics['minimum_horizontal_margin'], metrics['minimum_vertical_margin']]
    msg.nominal_sampled_valid = True
    msg.reason = 'HOLDING_AND_INITIAL_BRIDGE_UNQUALIFIED'
    return msg


def ack_message(ack):
    """Encode receiver outcome; no synthetic test ACCEPTED is emitted by the ROS path."""
    msg = FollowPlanAck()
    for name, value in asdict(ack).items():
        field = 'clock_generation' if name == 'generation' else name
        converted = (stamp(value) if name == 'stamp' else
                     list(value) if name == 'reasons' else value)
        setattr(msg, field, converted)
    msg.control_owner, msg.replaced = 'ORIGINAL_FOLLOW', False
    return msg

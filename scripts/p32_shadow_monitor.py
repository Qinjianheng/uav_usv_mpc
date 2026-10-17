#!/usr/bin/env python3
"""Read-only experiment recorder: ideal target, rendered pose and original FOLLOW commands.

Gazebo truth enters this separate offline evidence file only. This process has
no publisher, service client, planner reference or control acknowledgement.
"""
import argparse
import json
from pathlib import Path
import threading
import time


def encode_record(record):
    """Preserve PX4 scalar values and NaN fields in offline evidence JSON."""
    return json.dumps(record, allow_nan=True, default=lambda value: value.item())


def run(output):
    """Preserve native stamps and dual clocks; interpolate only in subsequent analysis."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import TrajectorySetpoint
    from uav_usv_interfaces.msg import (
        ControllerDiagnostic, TargetState, TargetObservation, TargetPrediction,
    )
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    rclpy.init()
    node = Node('p32_offline_evidence_monitor')
    stream = Path(output).open('x')
    lock, latest = threading.Lock(), {}

    def write(kind, data):
        with lock:
            if not stream.closed:
                stream.write(encode_record(dict(kind=kind, receipt=time.time(), **data))+'\n')

    def seconds(stamp, native=False):
        return stamp.sec+(stamp.nsec if native else stamp.nanosec)*1e-9

    def vector(v):
        return [v.x, v.y, v.z]

    def target(message, kind):
        write(kind, dict(stamp=seconds(message.stamp), source=seconds(message.source_stamp),
                         valid=message.valid, p=vector(message.position),
                         v=vector(message.velocity), a=vector(message.acceleration)))

    def reference(message):
        write('reference', dict(stamp=message.timestamp/1e6, p=list(message.position),
                                v=list(message.velocity), a=list(message.acceleration),
                                yaw=message.yaw, rate=message.yawspeed))

    def observation(message):
        write('observation', dict(stamp=seconds(message.stamp),
              received=seconds(message.received_stamp), processed=seconds(message.processed_stamp),
              published=seconds(message.published_stamp), valid=message.valid,
              reason=message.rejection_reason, rgb_receipt=message.rgb_receipt_stamp,
              depth_receipt=message.depth_receipt_stamp))

    def prediction(message):
        write('prediction', dict(stamp=seconds(message.source_stamp),
              observation=seconds(message.observation_stamp),
              generated=seconds(message.generated_stamp), compute=message.compute_time,
              sequence=message.sequence_id, valid=message.valid, reason=message.invalid_reason))

    def diagnostic(message):
        write('diagnostic', dict(stamp=seconds(message.stamp), mission=message.mission_id,
                                 plan=message.plan_id, status=message.status,
                                 p=vector(message.reference_position),
                                 v=vector(message.command_velocity),
                                 a=vector(message.command_acceleration),
                                 locked=message.target_locked, visible=message.target_visible,
                                 search=message.search_state, yaw_owner=message.yaw_owner))

    def clock(message):
        native = seconds(message.sim, True)
        if native-latest.get('clock', -1.) >= .025:
            latest['clock'] = native
            write('clock', dict(sim=native, system=seconds(message.system, True)))

    def poses(message):
        native = seconds(message.header.stamp, True)
        if native-latest.get('pose', -1.) < .025:
            return
        latest['pose'] = native
        for pose in message.pose:
            if pose.name == 'usv_target':
                write('rendered_target', dict(sim=native, p=(
                    pose.position.y, pose.position.x, -pose.position.z)))

    subscriptions = [
        node.create_subscription(TargetObservation, '/perception/front/target_observation',
                                 observation, qos_profile_sensor_data),
        node.create_subscription(TargetPrediction, '/planning/target_prediction',
                                 prediction, qos_profile_sensor_data),
        node.create_subscription(TargetState, '/target/state', lambda m: target(m, 'ideal'), 10),
        node.create_subscription(TargetState, '/tracking/target_state',
                                 lambda m: target(m, 'kf'), qos_profile_sensor_data),
        node.create_subscription(TrajectorySetpoint, '/control/reference', reference, 10),
        node.create_subscription(ControllerDiagnostic, '/control/diagnostic', diagnostic, 10),
    ]
    transport = GzNode()
    write('start', dict(evaluation_only=True, gazebo_subscriptions=[
        transport.subscribe(Clock, '/world/default/clock', clock),
        transport.subscribe(Pose_V, '/world/default/pose/info', poses)],
        ros_subscriptions=len(subscriptions)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        with lock:
            stream.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    arguments = parser.parse_args()
    run(arguments.output)

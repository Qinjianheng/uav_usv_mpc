#!/usr/bin/env python3
"""Evaluation-only P4 recorder; retains native pose/clock and continuous ROS authority evidence."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time

import numpy as np

from p32_shadow_monitor import encode_record


def run(output):
    """Read only; truth is never published or passed to the planner/control receiver."""
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import (
        qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy,
    )
    from uav_usv_interfaces.msg import (
        ControllerDiagnostic, UavState, TargetState, TargetObservation, TargetPrediction,
        FollowPlanAck, FollowTrajectory, FollowReceiverState, MissionState,
    )
    from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleStatus
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in handlers:
        signal.signal(sig, lambda *_: stopping.set())
    node = Node('p4_offline_evidence_monitor')
    stream = output.open('x')
    lock, latest = threading.Lock(), {}

    def write(kind, **data):
        with lock:
            if not stream.closed:
                stream.write(encode_record(dict(kind=kind, receipt=time.time(), **data))+'\n')
                if kind == 'graph':
                    stream.flush()

    def seconds(stamp, native=False):
        return stamp.sec+(stamp.nsec if native else stamp.nanosec)/1e9

    def xyz(value):
        return [float(getattr(value, k)) for k in 'xyz']

    def target(message, kind):
        write(kind, stamp=seconds(message.stamp), source=seconds(message.source_stamp),
              valid=message.valid, p=xyz(message.position), v=xyz(message.velocity),
              a=xyz(message.acceleration))

    def uav(message):
        write('uav', stamp=seconds(message.stamp), native=message.native_timestamp_sample,
              valid=message.valid, p=xyz(message.position), v=xyz(message.velocity),
              a=xyz(message.acceleration), yaw=message.heading)

    def reference(message):
        write('reference', stamp=message.timestamp/1e6, p=list(message.position),
              v=list(message.velocity), a=list(message.acceleration), yaw=message.yaw,
              rate=message.yawspeed)

    def diagnostic(message):
        write('diagnostic', stamp=seconds(message.stamp), mission=message.mission_id,
              plan=message.plan_id, status=message.status, locked=message.target_locked,
              visible=message.target_visible, search=message.search_state,
              yaw_owner=message.yaw_owner)

    def observation(message):
        write('observation', stamp=seconds(message.stamp),
              received=seconds(message.received_stamp),
              processed=seconds(message.processed_stamp),
              published=seconds(message.published_stamp),
              valid=message.valid, reason=message.rejection_reason)

    def prediction(message):
        write('prediction', stamp=seconds(message.source_stamp),
              observation=seconds(message.observation_stamp),
              generated=seconds(message.generated_stamp),
              compute=message.compute_time, sequence=message.sequence_id, valid=message.valid,
              reason=message.invalid_reason)

    def ack(message):
        write('follow_ack', stamp=seconds(message.stamp), plan=message.plan_id,
              mission=message.mission_id, generation=message.clock_generation,
              state=message.state, reasons=list(message.reasons), receiver=message.receiver,
              active=message.active_plan_id, pending=message.pending_plan_id,
              owner=message.control_owner, replaced=message.replaced,
              receiver_boot=message.receiver_boot_id, planner_boot=message.planner_boot_id)

    def graph():
        topics = ('/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode',
                  '/fmu/in/vehicle_command', '/control/follow_ack', '/planning/follow_trajectory',
                  '/control/follow_receiver_state')
        publishers = {t: [dict(node=e.node_name, namespace=e.node_namespace,
                               type=e.topic_type, gid=list(e.endpoint_gid))
                          for e in node.get_publishers_info_by_topic(t)] for t in topics}
        write('graph', publishers=publishers)
        snapshot = output.parent/'live_graph.tmp'
        snapshot.write_text(json.dumps(publishers, indent=2))
        snapshot.replace(output.parent/'live_graph.json')

    def clock(message):
        native = seconds(message.sim, True)
        if native-latest.get('clock', -1.) >= .025:
            latest['clock'] = native
            write('clock', sim=native, system=seconds(message.system, True))

    def poses(message):
        native = seconds(message.header.stamp, True)
        if native-latest.get('pose', -1.) < .025:
            return
        latest['pose'] = native
        for pose in message.pose:
            if pose.name == 'usv_target' or pose.name.startswith('x500_mono_cam_'):
                write('rendered_target' if pose.name == 'usv_target' else 'rendered_uav',
                      sim=native, name=pose.name,
                      p=[pose.position.y, pose.position.x, -pose.position.z],
                      q=[pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z])

    sensor = qos_profile_sensor_data
    subscriptions = [
        node.create_subscription(UavState, '/navigation/uav_state', uav, sensor),
        node.create_subscription(TargetObservation, '/perception/front/target_observation',
                                 observation, sensor),
        node.create_subscription(TargetPrediction, '/planning/target_prediction',
                                 prediction, sensor),
        node.create_subscription(TargetState, '/target/state', lambda m: target(m, 'ideal'), 10),
        node.create_subscription(TargetState, '/tracking/target_state', lambda m: target(m, 'kf'),
                                 sensor),
        node.create_subscription(TrajectorySetpoint, '/control/reference', reference, 10),
        node.create_subscription(ControllerDiagnostic, '/control/diagnostic', diagnostic, 10),
        node.create_subscription(FollowReceiverState, '/control/follow_receiver_state',
                                 lambda m: write(
                                     'receiver_epoch', stamp=seconds(m.stamp),
                                     boot=m.receiver_boot_id, generation=m.clock_generation,
                                     mission=m.mission_id, prediction=m.prediction_sequence_id,
                                     holding=m.holding_qualified, bridge=m.bridge_qualified),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE)),
        node.create_subscription(FollowPlanAck, '/control/follow_ack', ack, 10),
        node.create_subscription(FollowTrajectory, '/planning/follow_trajectory',
                                 lambda m: write(
                                     'proposal', plan=m.plan_id, stamp=seconds(m.published_stamp),
                                     receiver_boot=m.receiver_boot_id,
                                     planner_boot=m.planner_boot_id,
                                     generation=m.clock_generation,
                                     prediction=m.prediction_sequence_id,
                                     input_until=seconds(m.input_valid_until),
                                     start=seconds(m.execution_start_stamp),
                                     end=seconds(m.execution_end_stamp),
                                     holding=seconds(m.holding_valid_until)), 10),
        node.create_subscription(MissionState, '/mission/state', lambda m: write(
            'mission', state=m.state, name=m.state_name, mission=m.mission_id), 10),
        node.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', lambda m: write(
            'attitude', stamp=m.timestamp_sample/1e6, q=list(m.q)), sensor),
        node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', lambda m: write(
            'vehicle_status', nav_state=m.nav_state, armed=m.arming_state), sensor),
    ]
    timer = node.create_timer(1., graph)
    transport = GzNode()
    write('start', evaluation_only=True, gazebo_subscriptions=[
        transport.subscribe(Clock, '/world/default/clock', clock),
        transport.subscribe(Pose_V, '/world/default/pose/info', poses)],
        ros_subscriptions=len(subscriptions), timer_period=timer.timer_period_ns/1e9,
        numerical_library=np.__version__)
    try:
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except RuntimeError as error:
        if not stopping.is_set():
            raise
        write('requested_shutdown_exception', error=str(error))
    except KeyboardInterrupt:
        pass
    finally:
        transport.unsubscribe('/world/default/clock')
        transport.unsubscribe('/world/default/pose/info')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        with lock:
            stream.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    run(parser.parse_args().output)

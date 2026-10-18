#!/usr/bin/env python3
"""Evaluation-only P4 recorder; retains native pose/clock and continuous ROS authority evidence."""
import argparse
import json
import os
from pathlib import Path
import signal
import threading
import time

import numpy as np

from p32_shadow_monitor import encode_record


def execution_debug_kind(record):
    """Reception timing must not masquerade as the one control-cycle reference record."""
    return {'proposal_received': 'follow_receive',
            'execution_revoked': 'follow_revocation'}.get(record.get('event'), 'execution_debug')


def component_id(name):
    """Gazebo hash64, represented by the signed int64 key in SerializedEntityMap."""
    value = 0xcbf29ce484222325
    for byte in ('gz_sim_components.'+name).encode():
        value = ((value ^ byte)*0x100000001b3) & ((1 << 64)-1)
    return value if value < 1 << 63 else value-(1 << 64)


def state_model_poses(message):
    """Decode current top-level model Pose components, rather than initial Scene poses."""
    poses = {}
    for entity in message.state.entities.values():
        components = entity.components
        name = components.get(component_id('Name'))
        pose = components.get(component_id('Pose'))
        if name is None or pose is None:
            continue
        name = name.component.decode()
        if name == 'usv_target' or name.startswith('x500_mono_cam_'):
            values = [float(v) for v in pose.component.decode().split()]
            if len(values) >= 6 and np.isfinite(values).all():
                poses[name] = dict(p=[values[1], values[0], -values[2]],
                                   rpy=values[3:6])
    return poses


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
        InterceptTrajectory,
        InterceptResult,
    )
    from std_msgs.msg import String
    from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleStatus
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.serialized_map_pb2 import SerializedStepMap
    from gz.transport13 import Node as GzNode
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in handlers:
        signal.signal(sig, lambda *_: stopping.set())
    node = Node('p4_offline_evidence_monitor')
    stream = output.open('x')
    lock, latest = threading.Lock(), {}
    p6_record = os.environ.get('UAV_USV_P6_RECORD', '0') == '1'

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
        latest['navigation'] = dict(receipt=time.time(), valid=message.valid,
                                    p=xyz(message.position),
                                    v=xyz(message.velocity))
        write('uav', stamp=seconds(message.stamp), native=message.native_timestamp_sample,
              valid=message.valid, p=xyz(message.position), v=xyz(message.velocity),
              a=xyz(message.acceleration), yaw=message.heading)

    def reference(message):
        write('reference', stamp=message.timestamp/1e6, p=list(message.position),
              v=list(message.velocity), a=list(message.acceleration), yaw=message.yaw,
              rate=message.yawspeed)

    def diagnostic(message):
        latest['control'] = dict(receipt=time.time(), status=message.status,
                                 locked=message.target_locked,
                                 compute=message.callback_compute_time)
        write('diagnostic', stamp=seconds(message.stamp), mission=message.mission_id,
              plan=message.plan_id, status=message.status, compute=message.callback_compute_time,
              locked=message.target_locked,
              visible=message.target_visible, search=message.search_state,
              yaw_owner=message.yaw_owner)

    def observation(message):
        write('observation', stamp=seconds(message.stamp),
              received=seconds(message.received_stamp),
              processed=seconds(message.processed_stamp),
              published=seconds(message.published_stamp),
              valid=message.valid, reason=message.rejection_reason)
        if p6_record:
            write('p6_observation', stamp=seconds(message.stamp),
                  source=message.source, frame=message.frame_id,
                  valid=message.valid, reason=message.rejection_reason,
                  p=xyz(message.position), covariance=list(message.covariance),
                  received=seconds(message.received_stamp),
                  processed=seconds(message.processed_stamp),
                  published=seconds(message.published_stamp),
                  image_measurement_stamp=message.image_measurement_stamp,
                  image_clock_reset_count=message.image_clock_reset_count,
                  px4_clock_reset_count=message.px4_clock_reset_count,
                  image_clock_status=message.image_clock_status,
                  px4_clock_status=message.px4_clock_status)

    def prediction(message):
        write('prediction', stamp=seconds(message.source_stamp),
              observation=seconds(message.observation_stamp),
              generated=seconds(message.generated_stamp),
              compute=message.compute_time, sequence=message.sequence_id, valid=message.valid,
              reason=message.invalid_reason)
        if p6_record:
            prediction_samples(message, '/planning/target_prediction')

    def prediction_samples(message, topic):
        write('p6_prediction', topic=topic, mission=message.mission_id,
              sequence=message.sequence_id, source_stamp=seconds(message.source_stamp),
              observation_stamp=seconds(message.observation_stamp),
              generated_stamp=seconds(message.generated_stamp),
              valid_until=seconds(message.valid_until), model=message.model,
              source=message.source, frame=message.frame_id,
              valid=message.valid, reason=message.invalid_reason,
              compute=message.compute_time,
              times=[seconds(s.relative_time) for s in message.samples],
              p=[xyz(s.position) for s in message.samples],
              v=[xyz(s.velocity) for s in message.samples],
              a=[xyz(s.acceleration) for s in message.samples],
              covariance=[list(s.position_covariance) for s in message.samples])

    def ack(message):
        write('follow_ack', stamp=seconds(message.stamp), plan=message.plan_id,
              mission=message.mission_id, generation=message.clock_generation,
              state=message.state, reasons=list(message.reasons), receiver=message.receiver,
              active=message.active_plan_id, pending=message.pending_plan_id,
              owner=message.control_owner, replaced=message.replaced,
              receiver_compute_seconds=message.receiver_compute_seconds,
              receiver_boot=message.receiver_boot_id, planner_boot=message.planner_boot_id)

    def result(message):
        data = dict(stamp=seconds(message.stamp), success=message.success,
                    outcome=message.outcome, reason=message.reason,
                    radius=message.capture_radius, minimum_distance=message.minimum_distance,
                    elapsed=message.elapsed_time, relative_speed=message.relative_speed)
        latest['result'] = data
        write('capture_result', **data)
        snapshot = output.parent/'terminal_result.tmp'
        snapshot.write_text(json.dumps(data))
        snapshot.replace(output.parent/'terminal_result.json')

    def graph():
        health = output.parent/'live_health.tmp'
        health.write_text(json.dumps(latest))
        health.replace(output.parent/'live_health.json')
        topics = ('/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode',
                  '/fmu/in/vehicle_command', '/control/follow_ack', '/planning/follow_trajectory',
                  '/control/follow_receiver_state', '/planning/target_prediction')
        if p6_record:
            topics += ('/planning/target_prediction', '/planning/imm_target_prediction',
                       '/planning/imm_diagnostics')
        publishers = {t: [dict(node=e.node_name, namespace=e.node_namespace,
                               type=e.topic_type, gid=list(e.endpoint_gid))
                          for e in node.get_publishers_info_by_topic(t)] for t in topics}
        write('graph', publishers=publishers)
        snapshot = output.parent/'live_graph.tmp'
        snapshot.write_text(json.dumps(publishers, indent=2))
        snapshot.replace(output.parent/'live_graph.json')
        if latest.get('paused'):
            executed, state = transport.request('/world/default/state', Empty(),
                                                Empty, SerializedStepMap, 250)
            if executed and state is not None:
                for name, pose in state_model_poses(state).items():
                    write('paused_native_pose', name=name, sim=seconds(state.stats.sim_time, True),
                          **pose)

    def statistics(message):
        latest['paused'] = message.paused
        write('gazebo_stats', rtf=message.real_time_factor,
              paused=message.paused, sim=seconds(message.sim_time, True))

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
        node.create_subscription(InterceptResult, '/simulation/impact/result', result,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE)),
        node.create_subscription(InterceptTrajectory, '/planning/intercept_trajectory',
                                 lambda m: write('intercept_proposal', plan=m.plan_id,
                                                 stamp=seconds(m.published_stamp)), 10),
        node.create_subscription(String, '/control/p7_guidance',
                                 lambda m: write('p7_guidance', **json.loads(m.data)), 10),
        node.create_subscription(String, '/control/follow_execution_debug',
                                 lambda m: write(execution_debug_kind(json.loads(m.data)),
                                                 **json.loads(m.data)), 10),
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
                                     active=m.active_plan_id, pending=m.pending_plan_id,
                                     research_state=m.research_state,
                                     entry_allowed=m.entry_allowed,
                                     converged=m.follow_converged,
                                     lease_deadline=m.active_lease_deadline,
                                     recovery_until=m.recovery_until,
                                     entry_attempts=m.entry_attempts,
                                     valid_probe_count=m.valid_probe_count,
                                     last_fallback_reason=m.last_fallback_reason,
                                     entry_inhibition_reason=m.entry_inhibition_reason,
                                     holding=m.holding_qualified, bridge=m.bridge_qualified),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE)),
        node.create_subscription(FollowPlanAck, '/control/follow_ack', ack, 10),
        node.create_subscription(FollowTrajectory, '/planning/follow_trajectory',
                                 lambda m: write(
                                     'proposal', plan=m.plan_id, stamp=seconds(m.published_stamp),
                                     parent=m.parent_plan_id, durations=list(m.durations),
                                     xyz=list(m.xyz_coefficients), yaw=list(m.yaw_coefficients),
                                     source=seconds(m.source_stamp),
                                     observation=seconds(m.observation_stamp),
                                     navigation=seconds(m.navigation_stamp),
                                     attitude=seconds(m.attitude_stamp),
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
        node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', lambda m: (
            latest.update(vehicle=dict(receipt=time.time(), nav_state=m.nav_state,
                                       armed=m.arming_state)), write(
                'vehicle_status', nav_state=m.nav_state, armed=m.arming_state)), sensor),
    ]
    if p6_record:
        subscriptions.extend([
            node.create_subscription(TargetPrediction, '/planning/imm_target_prediction',
                                     lambda m: prediction_samples(
                                         m, '/planning/imm_target_prediction'), sensor),
            node.create_subscription(String, '/planning/imm_diagnostics',
                                     lambda m: write('p6_imm', **json.loads(m.data)), sensor),
            node.create_subscription(String, '/simulation/impact/motion_truth',
                                     lambda m: write('p6_motion_truth', **json.loads(m.data)), 10),
            node.create_subscription(String, '/simulation/impact/motion_mode',
                                     lambda m: write('p6_motion_mode', data=json.loads(m.data)),
                                     QoSProfile(
                                         depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                         reliability=ReliabilityPolicy.RELIABLE)),
        ])
    timer = node.create_timer(1., graph)
    transport = GzNode()
    write('start', evaluation_only=True, gazebo_subscriptions=[
        transport.subscribe(Clock, '/world/default/clock', clock),
        transport.subscribe(Pose_V, '/world/default/pose/info', poses),
        transport.subscribe(WorldStatistics, '/world/default/stats', statistics)],
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
        transport.unsubscribe('/world/default/stats')
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

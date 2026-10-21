#!/usr/bin/env python3
"""Task-owned, bounded physical calibration controller; never instantiate the formal Tracker."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (ActuatorMotors, OffboardControlMode, TrajectorySetpoint,
                          VehicleAcceleration, VehicleCommand, VehicleControlMode,
                          VehicleLandDetected,
                          VehicleRatesSetpoint, VehicleStatus)
from std_msgs.msg import Bool, String
from uav_usv_interfaces.msg import NativeAttitude, UavState

from uav_control.common.navigation_state import navigation_components
from uav_control.common.p76_body_geometry import (
    base_reference_flu, navigation_vertices, lowest_world_z,
)
from uav_control.common.p76_contract import load_contract
from uav_control.common.p76_native import NativePoseHistory, NativeReferenceHistory
from uav_control.common.p76_readiness import sha256, source_fingerprint
from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
from uav_control.control.p76_recovery import VelocityRecovery
from p76_calibration_analysis import analyze_rows, stage_verdict


def check_native_epoch(now, previous, states, poses, references):
    """Revoke the entire native acquisition epoch on a backward control clock jump."""
    if now < previous:
        states.clear()
        poses.clear()
        references.samples.clear()
        raise ValueError('P762_NATIVE_CLOCK_RESET')
    return now


def paired_native_state(states, poses, references, now):
    """Use the newest delivered complete native pair, never extrapolate a missing pose."""
    for state in reversed(states):
        if not 0 <= now-state[0] <= .125:
            continue
        try:
            return state, poses.at(state[0]), references.at(state[0])
        except ValueError:
            continue
    raise ValueError('P761_PAIRED_NATIVE_INPUT_UNAVAILABLE')


class Calibration(Node):
    """Velocity takeoff/settling and bounded Body-rate actions use one command owner."""

    def __init__(self, output, stage, prior):
        super().__init__('p761_calibration_controller')
        require_isolated_sitl()
        self.output, self.stage = output, stage
        self.contract = load_contract(os.environ['UAV_USV_P76_CONTRACT'])
        model_root = Path(os.environ['UAV_USV_P76_MODEL_ROOT'])
        model = json.loads((model_root/'p76_model_manifest.json').read_text())
        px4 = Path(self.contract['dynamics']['px4_model_root']).parents[3]
        self.identity = dict(contract_file=self.contract['_path'],
                             contract_sha256=sha256(self.contract['_path']),
                             detector_sha256=self.contract['detector']['model_sha256'],
                             sources=source_fingerprint(Path(os.environ['UAV_USV_WS'])),
                             px4=dict(binary_sha256=sha256(px4/'build/px4_sitl_default/bin/px4'),
                                      dds_sha256=sha256(px4/'src/modules/uxrce_dds_client/'
                                                        'dds_topics.yaml')), model=model)
        (output/'identity.json').write_text(json.dumps(self.identity, indent=2)+'\n')
        self.mass, self.gravity = model['total_mass'], model['gravity']
        self.points = navigation_vertices(model['collision_envelope'], model_root)
        self.base_height = base_reference_flu(model_root)[2, 3]
        self.force_samples, self.last_force_epoch = {}, None
        self.recovery_velocity = VelocityRecovery()
        self.rows, self.schedule, self.rate_history = [], [], deque(maxlen=2000)
        self.raw = (output/'calibration.jsonl').open('x')
        self.history, self.references = NativePoseHistory(), NativeReferenceHistory()
        self.state, self.status, self.control_mode = None, None, None
        self.landed, self.landing_seen = False, False
        self.native_states = deque(maxlen=32)
        self.thrust, self.hover, self.home = None, None, None
        self.phase, self.phase_wall, self.phase_ros = '', 0., 0.
        self.started = time.monotonic()
        self.last_tick, self.last_command, self.settled = self.started, 0., None
        self.last_ros_time = self.now()
        self.steps, self.step_index, self.rate_command = [], -1, np.zeros(3)
        self.done, self.completed, self.failure = False, False, ''
        self.landing_started, self.trial = None, None
        self.prior = None
        if stage == 'A2':
            if prior is None:
                raise ValueError('P761_A2_REQUIRES_A1')
            self.prior = json.loads((prior/'analysis.json').read_text())
            from p76_calibration_manifest import validate_stage_evidence
            prior_identity, _ = validate_stage_evidence(prior, self.prior)
            if not self.prior['A1_passed'] or prior_identity != self.identity:
                raise ValueError('P761_A1_NOT_CURRENT_OR_NOT_QUALIFIED')
        self.offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.velocity = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.rates = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', 10)
        self.commands = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.ready = self.create_publisher(Bool, '/simulation/impact/flight_ready', 10)
        self.subs = [self.create_subscription(UavState, '/navigation/uav_state', self.navigation,
                                              qos_profile_sensor_data),
                     self.create_subscription(NativeAttitude, '/navigation/native_attitude',
                                              self.pose, 16),
                     self.create_subscription(
                         String, '/navigation/native_thrust', self.collective, 16),
                     self.create_subscription(String, '/navigation/native_local_reference',
                                              self.reference, 16)]
        for name, cls in (('vehicle_status', VehicleStatus),
                          ('vehicle_control_mode', VehicleControlMode),
                          ('vehicle_land_detected', VehicleLandDetected),
                          ('vehicle_acceleration', VehicleAcceleration),
                          ('actuator_motors', ActuatorMotors)):
            version = getattr(cls, 'MESSAGE_VERSION', 0)
            topic = '/fmu/out/'+name+(f'_v{version}' if version else '')
            self.subs.append(
                self.create_subscription(
                    cls,
                    topic,
                    lambda message,
                    key=name: self.wire(
                        key,
                        message),
                    qos_profile_sensor_data))
        self.change('WAIT_NATIVE')
        self.timer = self.create_timer(.005, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def log(self, kind, **values):
        stamp = values.get('stamp', self.now())
        phase, begin = next(((p, t) for t, p in reversed(self.schedule) if t <= stamp),
                            ('BEFORE_CONTROLLER', stamp))
        row = dict(kind=kind, phase=phase, phase_elapsed=max(0., stamp-begin), **values)
        self.rows.append(row)
        self.raw.write(json.dumps(row, allow_nan=True)+'\n')

    def change(self, phase):
        self.phase, self.phase_wall, self.phase_ros = phase, time.monotonic(), self.now()
        self.schedule.append((self.phase_ros, phase))
        self.log('phase', stamp=self.phase_ros, target=phase)
        self.settled = None

    def navigation(self, message):
        try:
            stamp, p, v, a = navigation_components(message, self.now())
            self.state = (stamp, np.asarray(p), np.asarray(v), np.asarray(a))
            self.native_states.append(self.state)
            self.log('state', stamp=stamp, p=list(p), v=list(v), a=list(a), age=self.now()-stamp,
                     px4_timestamp_sample=message.px4_timestamp_sample,
                     native_timestamp_sample=message.native_timestamp_sample)
        except ValueError:
            return

    def pose(self, message):
        if not message.valid:
            return
        stamp = message.stamp.sec+message.stamp.nanosec*1e-9
        try:
            self.history.add(stamp, message.quaternion_wxyz, message.angular_velocity_frd,
                             message.orientation_variance)
            self.log('pose', stamp=stamp, q=list(message.quaternion_wxyz),
                     omega=list(message.angular_velocity_frd),
                     px4_timestamp_sample=message.px4_timestamp_sample,
                     native_timestamp_sample=message.native_timestamp_sample)
            command = next((c for t, c in reversed(self.rate_history) if t <= stamp), None)
            if command is not None:
                self.log('rate_sample', stamp=stamp, desired=command,
                         actual=list(message.angular_velocity_frd))
        except ValueError:
            return

    def collective(self, message):
        packet = json.loads(message.data)
        if 0 <= self.now()-packet['stamp'] <= .125:
            self.thrust = packet['stamp'], -float(packet['xyz'][2])
            self.log('executed_collective', **packet)

    def reference(self, message):
        packet = json.loads(message.data)
        self.references.add(packet['stamp'], packet['ref_alt'])
        self.log('reference', **packet)

    def wire(self, name, message):
        row = {key: getattr(message, key).tolist() if hasattr(getattr(message, key), 'tolist')
               else getattr(message, key) for key in message.get_fields_and_field_types()}
        self.log('wire', topic=name, values=row)
        if name == 'vehicle_status':
            self.status = message
        elif name == 'vehicle_land_detected':
            self.landed = bool(message.landed)
            self.landing_seen = self.landing_seen or (self.phase == 'LAND' and self.landed)
        elif name == 'vehicle_control_mode':
            self.control_mode = message
            self.log('mode', flags={key: bool(getattr(message, 'flag_control_'+key+'_enabled'))
                                    for key in ('offboard', 'rates', 'position', 'velocity',
                                                'attitude', 'altitude', 'climb_rate')})

    def command(self, command, first=0., second=0.):
        self.commands.publish(VehicleCommand(
            timestamp=round(self.now()*1e6), command=command, param1=float(first),
            param2=float(second), target_system=1, target_component=1,
            source_system=1, source_component=1, from_external=True))

    def emit_velocity(self, desired):
        stamp = round(self.now()*1e6)
        self.offboard.publish(OffboardControlMode(timestamp=stamp, velocity=True))
        self.velocity.publish(TrajectorySetpoint(timestamp=stamp, position=[math.nan]*3,
                                                 velocity=list(map(float, desired)),
                                                 acceleration=[math.nan]*3, yaw=float(self.yaw)))
        self.log('command', stamp=self.now(), interface='velocity', velocity=list(desired))

    def emit_body(self, pose, dt, thrust, axis=None, desired_pitch=0.):
        roll, pitch, yaw = Rotation.from_matrix(pose.rotation).as_euler('xyz')
        rates = np.array([-4*roll, 4*(desired_pitch-pitch),
                          2*math.atan2(math.sin(self.yaw-yaw), math.cos(self.yaw-yaw))])
        if axis is not None:
            rates[axis] = (1.5, 1.5, 1.2)[axis]
        rates = np.clip(rates, [-1.5, -1.5, -1.2], [1.5, 1.5, 1.2])
        delta = rates-self.rate_command
        rates = self.rate_command+delta*min(1., 3*dt/max(np.linalg.norm(delta), 1e-9))
        self.rate_command = rates
        stamp = round(self.now()*1e6)
        self.offboard.publish(OffboardControlMode(timestamp=stamp, body_rate=True))
        self.rates.publish(VehicleRatesSetpoint(timestamp=stamp, roll=float(rates[0]),
                                                pitch=float(rates[1]), yaw=float(rates[2]),
                                                thrust_body=[0., 0., -float(thrust)]))
        self.rate_history.append((self.now(), rates.tolist()))
        self.log('command', stamp=self.now(), interface='body_rate',
                 rates=rates.tolist(), thrust=thrust)

    def abort(self, reason):
        if self.failure:
            return
        self.failure = reason
        self.log('failure', reason=reason)
        self.change('LAND')
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.landing_started = time.monotonic()

    def make_steps(self):
        if self.stage == 'A1':
            for value in (.3, .4, .5, .6, .7, .85, 1.):
                self.steps += [('BODY_PREP', .65), (f'THRUST_{value:.3f}', .22), ('RESET', 3.)]
            for axis in range(3):
                self.steps += [('BODY_PREP', .65), (f'RATE_{axis}', .65), ('RESET', 3.)]
            self.steps += [('BODY_PREP', .65), ('MANEUVER', 1.7), ('RESET', 3.)]
        else:
            for value in (.5, 1., 2.):
                self.steps += [(f'DESCENT_{value}', 2.5), (f'BRAKE_{value}', 2.), ('RESET', 3.)]
            self.steps += [('FAULT_ACCEL', 1.2), ('BODY_PREP', .65), ('FAULT_RECOVERY', 3.)]

    def next_step(self):
        self.step_index += 1
        if self.step_index >= len(self.steps):
            self.completed = True
            self.change('LAND')
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_started = time.monotonic()
        else:
            self.change(self.steps[self.step_index][0])

    def tick(self):
        wall = time.monotonic()
        dt, self.last_tick = wall-self.last_tick, wall
        if self.done:
            return
        if wall-self.started > 150:
            self.abort('CALIBRATION_WATCHDOG')
        if self.phase == 'LAND':
            if (self.status is not None and self.status.arming_state == 1
                    and self.landing_seen and self.landed):
                self.done = True
            elif wall-self.landing_started > 20:
                self.failure = self.failure or 'LANDING_DISARM_TIMEOUT'
                self.log('failure', reason=self.failure)
                self.done = True
            return
        try:
            self.last_ros_time = check_native_epoch(
                self.now(), self.last_ros_time, self.native_states, self.history, self.references)
            if self.state is None or not 0 <= self.now()-self.state[0] <= .125:
                raise ValueError('NATIVE_NAVIGATION_STALE')
            paired, pose, reference = paired_native_state(
                self.native_states, self.history, self.references, self.now())
            stamp, position, velocity, acceleration = paired
            self.log('paired_state', stamp=stamp, receipt=self.now(), age=self.now()-stamp,
                     pose_stamp=pose.stamp, ref_alt=reference)
            sea = reference-self.contract['candidate']['sea_altitude_msl']
            if self.home is None:
                self.home = position.copy()
                self.yaw = float(Rotation.from_matrix(pose.rotation).as_euler('xyz')[2])
                self.home[2] = sea-8.-self.base_height
                self.change('PRESTREAM')
            desired = np.clip(1.5*(self.home-position), [-2., -2., -1.2], [2., 2., 1.2])
            elapsed = wall-self.phase_wall
            if self.status is None:
                self.emit_velocity([0., 0., 0.])
                return
            armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if not armed:
                self.emit_velocity([0., 0., 0.])
                if self.phase not in ('PRESTREAM', 'WAIT_NATIVE'):
                    raise ValueError('UNEXPECTED_DISARM')
                if elapsed > 1.2 and wall-self.last_command > 1.:
                    self.last_command = wall
                    if self.status.nav_state != 14:
                        self.command(176, 1., 6.)
                    elif self.status.pre_flight_checks_pass:
                        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.)
                return
            self.ready.publish(Bool(data=True))
            source_phase, source_begin = next(
                ((p, t) for t, p in reversed(self.schedule) if t <= stamp),
                ('BEFORE_CONTROLLER', stamp))
            if (source_phase.startswith('THRUST_') and stamp-source_begin >= .08
                    and stamp != self.last_force_epoch):
                self.last_force_epoch = stamp
                force = self.mass*float(-pose.rotation[:, 2]@(acceleration-[0., 0., self.gravity]))
                self.force_samples.setdefault(float(source_phase.split('_')[1]), []).append(force)
            if self.phase == 'PRESTREAM':
                self.change('TAKEOFF')
            if abs(position[2]-self.home[2]) > 4. and self.phase not in ('TAKEOFF',):
                raise ValueError('CALIBRATION_ALTITUDE_ENVELOPE')
            if (np.linalg.norm(velocity) > 7. or np.max(np.abs(pose.rotation[:, 2][:2])) > .7
                    or np.linalg.norm(position[:2]-self.home[:2]) > 15.):
                raise ValueError('CALIBRATION_MOTION_ENVELOPE')
            if self.phase not in ('TAKEOFF',) and sea-lowest_world_z(
                    self.points, position, pose.rotation) < 2.:
                raise ValueError('CALIBRATION_SEA_CLEARANCE')
            settled = np.linalg.norm(position-self.home) < .25 and np.linalg.norm(velocity) < .25
            if self.phase in ('TAKEOFF', 'HOVER', 'RESET'):
                self.emit_velocity(desired)
                self.settled = (self.settled or wall) if settled else None
                if self.phase == 'TAKEOFF' and self.settled and wall-self.settled > 1.:
                    self.change('HOVER')
                elif self.phase == 'HOVER' and elapsed > 2. and settled:
                    values = [r['xyz'][2] for r in self.rows if r['kind'] == 'executed_collective'
                              and r['phase'] == 'HOVER']
                    if len(values) < 20:
                        raise ValueError('NO_ACTUAL_HOVER_COLLECTIVE')
                    self.hover = -float(np.median(values))
                    self.make_steps()
                    self.next_step()
                elif self.phase == 'RESET' and self.settled and wall-self.settled > .7:
                    self.next_step()
                elif self.phase == 'RESET' and elapsed > 9.:
                    raise ValueError('FAILED_TO_SETTLE_BETWEEN_ACTIONS')
                return
            if self.phase.startswith('DESCENT_'):
                speed = float(self.phase.split('_')[1])
                desired[2] = speed
                self.emit_velocity(desired)
                if velocity[2] >= .95*speed or elapsed > 2.5:
                    self.trial = dict(start=self.now(), initial_speed=float(velocity[2]),
                                      response_seconds=None, decelerations=[], previous=paired,
                                      initial_z=float(position[2]))
                    self.next_step()
                return
            if self.phase == 'FAULT_ACCEL':
                desired[2] = .8
                self.emit_velocity(desired)
                if elapsed > 1.2:
                    self.next_step()
                return
            if self.phase == 'FAULT_RECOVERY':
                if not hasattr(self, 'fault_data'):
                    self.fault_data = dict(previous=paired, decelerations=[], response=None)
                previous = self.fault_data['previous']
                if stamp > previous[0] and velocity[2] > .2:
                    value = (previous[2][2]-velocity[2])/(stamp-previous[0])
                    if value > 0:
                        self.fault_data['decelerations'].append(float(value))
                        if self.fault_data['response'] is None:
                            self.fault_data['response'] = max(.001, stamp-self.phase_ros)
                    self.fault_data['previous'] = paired
                from uav_control.control.trajectory_tracker_node import command_to_setpoint
                recovery = self.recovery_velocity.setpoint(position, velocity, self.now(), 4.)
                native_us = round(self.now()*1e6)
                self.offboard.publish(OffboardControlMode(timestamp=native_us, velocity=True))
                self.velocity.publish(command_to_setpoint(
                    recovery, native_us, yaw=self.yaw, velocity_mode=True))
                self.log('command', stamp=self.now(), interface='velocity',
                         velocity=recovery.velocity.tolist(),
                         acceleration=recovery.acceleration.tolist())
                if elapsed > 2.5:
                    if (np.linalg.norm(velocity) > .25 or self.control_mode is None
                            or not self.control_mode.flag_control_velocity_enabled):
                        raise ValueError('FAULT_RECOVERY_FAILED')
                    self.log('fault_recovered', stamp=stamp,
                             initial_velocity=self.recovery_velocity.initial.tolist(),
                             response_seconds=self.fault_data['response'] or 0.,
                             reliable_deceleration=float(np.percentile(
                                 self.fault_data['decelerations'], 25))
                             if self.fault_data['decelerations'] else 0.)
                    self.next_step()
                return
            thrust = self.hover
            axis, desired_pitch = None, 0.
            if self.phase.startswith('THRUST_'):
                thrust = float(self.phase.split('_')[1])
            elif self.phase.startswith('RATE_'):
                axis = int(self.phase.split('_')[1])
            elif self.phase == 'MANEUVER' or self.phase.startswith('BRAKE_'):
                curve = (self.prior['measured_curve'] if self.prior else [
                    dict(normalized=u, force=float(np.median(values)))
                    for u, values in sorted(self.force_samples.items()) if len(values) >= 3])
                if len(curve) < 5:
                    raise ValueError('MEASURED_CURVE_REQUIRED_FOR_DYNAMICS')
                desired_pitch = -.5 if self.phase == 'MANEUVER' else 0.
                force = self.mass*(self.gravity/math.cos(desired_pitch)
                                   + (5. if self.phase.startswith('BRAKE_') else 0.))
                thrust = float(np.interp(force, [c['force'] for c in curve],
                                         [c['normalized'] for c in curve]))
            self.emit_body(pose, min(.02, max(.001, dt)), thrust, axis, desired_pitch)
            if self.phase.startswith('BRAKE_'):
                trial = self.trial
                previous = trial['previous']
                if stamp > previous[0]:
                    deceleration = (previous[2][2]-velocity[2])/(stamp-previous[0])
                    trial['decelerations'].append(float(deceleration))
                    trial['previous'] = paired
                if trial['response_seconds'] is None and velocity[2] < trial['initial_speed']-.05:
                    trial['response_seconds'] = max(.001, stamp-trial['start'])
                if velocity[2] <= .1 or elapsed > 2.:
                    positive = [v for v in trial['decelerations'] if v > 0]
                    self.log('brake_trial', initial_speed=trial['initial_speed'],
                             stopped=bool(velocity[2] <= .1),
                             response_seconds=trial['response_seconds'] or 0.,
                             maximum_downward_displacement=float(
                                 max(0., position[2]-trial['initial_z'])),
                             reliable_deceleration=float(np.percentile(positive, 10))
                             if positive else 0.)
                    self.next_step()
                return
            if elapsed >= self.steps[self.step_index][1]:
                self.next_step()
        except (ValueError, np.linalg.LinAlgError) as error:
            if self.home is not None and self.status is not None and self.status.arming_state == 2:
                self.abort(str(error))
            elif wall-self.started > 20:
                self.abort('STARTUP_'+str(error))


def main():
    """The parent harness owns simulator, authorization grant and wall-clock termination."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=('A1', 'A2'), required=True)
    parser.add_argument('--prior-a1', type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Calibration(args.output, args.stage, args.prior_a1)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.raw.flush()
        analysis = analyze_rows(node.rows, node.mass, node.gravity)
        (args.output/'analysis.json').write_text(json.dumps(analysis, indent=2)+'\n')
        (args.output/'run_result.json').write_text(json.dumps(dict(
            completed=node.completed and not node.failure, failure=node.failure,
            stage=args.stage, elapsed=time.monotonic()-node.started,
            scope='Actual isolated PX4/Gazebo calibration; not a target capture'), indent=2)+'\n')
        node.raw.close()
        node.destroy_node()
        rclpy.shutdown()
    outcome = stage_verdict(
        args.stage, dict(completed=node.completed, failure=node.failure), analysis, node.rows,
        node.identity['sources'] == source_fingerprint(Path(os.environ['UAV_USV_WS'])),
        True, node.prior is not None or args.stage == 'A1')
    return 0 if outcome['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

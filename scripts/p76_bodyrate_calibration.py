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
from uav_control.control.p763_calibration_control import (
    ARCHITECTURE, CalibrationBodyHold, load_bootstrap, pulse_admission,
)
from p76_calibration_analysis import (
    analyze_rows, stage_verdict, formal_controller_identity, make_formal_controller,
    formal_identity_digest,
)


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
    """Velocity takeoff, then exclusively Body-rate calibration until latched native LAND."""

    def __init__(self, output, stage, prior):
        super().__init__('p761_calibration_controller')
        require_isolated_sitl()
        self.output, self.stage = output, stage
        self.contract = load_contract(os.environ['UAV_USV_P76_CONTRACT'])
        model_root = Path(os.environ['UAV_USV_P76_MODEL_ROOT'])
        model = json.loads((model_root/'p76_model_manifest.json').read_text())
        px4 = Path(self.contract['dynamics']['px4_model_root']).parents[3]
        self.identity = dict(schema_version=2, architecture=ARCHITECTURE,
                             contract_file=self.contract['_path'],
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
        self.body_entered, self.body_hold, self.brake_origin = False, None, None
        self.land_requested = False
        self.model_maximum_thrust = model['maximum_thrust']
        bootstrap = (Path(os.environ['UAV_USV_WS'])
                     / 'data/experiments/20261021_p761_01/session_A1_03')
        self.bootstrap = load_bootstrap(bootstrap, self.identity)
        (output/'bootstrap_physical_map.json').write_text(json.dumps(dict(
            qualified=False, source=str(bootstrap), model=model, px4=self.identity['px4'],
            raw_sha256=sha256(bootstrap/'calibration.jsonl'),
            analysis_sha256=sha256(bootstrap/'analysis.json'),
            scope='Historical A1_03 physical bootstrap only; not inherited A qualification'),
            indent=2)+'\n')
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
        self.formal_identity, self.formal_outer = None, None
        self.formal_last_ros, self.formal_recovery_phase = None, None
        self.formal_seed = None
        if stage == 'A2':
            if prior is None:
                raise ValueError('P761_A2_REQUIRES_A1')
            self.prior = json.loads((prior/'analysis.json').read_text())
            from p76_calibration_manifest import validate_stage_evidence
            prior_identity, prior_evidence = validate_stage_evidence(prior, self.prior)
            if not self.prior['A1_passed'] or prior_identity != self.identity:
                raise ValueError('P761_A1_NOT_CURRENT_OR_NOT_QUALIFIED')
            braking_request = min(4., self.prior['maximum_effective_thrust']/self.mass
                                  - self.gravity)
            if braking_request <= 0:
                raise ValueError('P763_A1_BRAKING_CANDIDATE_UNAVAILABLE')
            self.formal_identity = formal_controller_identity(
                self.contract, self.mass, self.prior['maximum_effective_thrust'],
                self.prior['measured_curve'], self.points, braking_request, .24, prior_evidence)
            self.formal_identity_sha256 = formal_identity_digest(self.formal_identity)
            self.log('formal_controller_identity', identity=self.formal_identity,
                     identity_sha256=self.formal_identity_sha256)
            self.formal_outer, self.formal_recovery, self.formal_barrier = (
                make_formal_controller(self.formal_identity))
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
        values.setdefault('stamp', stamp)
        row = dict(kind=kind, phase=phase, phase_elapsed=max(0., stamp-begin), **values)
        self.rows.append(row)
        self.raw.write(json.dumps(row, allow_nan=True)+'\n')

    def change(self, phase):
        self.phase, self.phase_wall, self.phase_ros = phase, time.monotonic(), self.now()
        self.schedule.append((self.phase_ros, phase))
        self.log('phase', stamp=self.phase_ros, target=phase)
        self.brake_origin = None
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
        self.log('vehicle_command', stamp=self.now(), command=int(command),
                 param1=float(first), param2=float(second))
        self.commands.publish(VehicleCommand(
            timestamp=round(self.now()*1e6), command=command, param1=float(first),
            param2=float(second), target_system=1, target_component=1,
            source_system=1, source_component=1, from_external=True))

    def emit_velocity(self, desired):
        if self.body_entered:
            raise ValueError('P763_VELOCITY_AFTER_BODY_ENTRY')
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
        self.publish_body(rates, thrust)

    def publish_body(self, rates, thrust):
        self.body_entered = True
        self.rate_command = np.asarray(rates, float)
        stamp = round(self.now()*1e6)
        self.offboard.publish(OffboardControlMode(timestamp=stamp, body_rate=True))
        self.rates.publish(VehicleRatesSetpoint(timestamp=stamp, roll=float(rates[0]),
                                                pitch=float(rates[1]), yaw=float(rates[2]),
                                                thrust_body=[0., 0., -float(thrust)]))
        self.rate_history.append((self.now(), rates.tolist()))
        self.log('command', stamp=self.now(), interface='body_rate',
                 rates=rates.tolist(), thrust=float(thrust))

    def emit_hold(self, pose, position, velocity, dt, *, brake=False, desired_velocity=None):
        if self.stage == 'A2':
            return self.emit_formal(pose, position, velocity, brake=brake,
                                    desired_velocity=desired_velocity)
        if self.body_hold is None:
            self.body_hold = CalibrationBodyHold(self.mass, self.gravity, self.bootstrap)
            actual = self.thrust[1] if self.thrust is not None else self.hover
            self.body_hold.initialize(pose.omega, actual)
        if self.brake_origin is None:
            self.brake_origin = position.copy()
        target = self.brake_origin if brake else self.home
        if desired_velocity is not None:
            target = self.brake_origin+desired_velocity*(self.now()-self.phase_ros)
        self.body_hold.initialize(self.rate_command, self.thrust[1] if self.thrust else self.hover)
        result = self.body_hold.step(target, position, velocity, pose.rotation,
                                     min(.02, max(.001, dt)), self.yaw, brake=brake,
                                     desired_velocity=desired_velocity)
        self.publish_body(result.rates, result.thrust)
        self.log('body_control', stamp=self.now(), acceleration=result.acceleration.tolist(),
                 force_newtons=result.force_newtons,
                 explicit_high_pulse_override=result.explicit_high_pulse_override)

    def emit_formal(self, pose, position, velocity, *, brake=False, desired_velocity=None):
        """A2 exercises the identical pure limiter/tracker/recovery core used in normal safety."""
        if self.formal_outer is None:
            raise ValueError('P763_FORMAL_RECOVERY_A1_MAP_REQUIRED')
        now, native_stamp = self.now(), self.last_native[0]
        dt = .005 if self.formal_last_ros is None else now-self.formal_last_ros
        if not 0 < dt <= .125:
            raise ValueError('P763_FORMAL_RECOVERY_CONTROL_CLOCK')
        seed = None
        if self.formal_last_ros is None:
            if self.thrust is None or not 0 <= now-self.thrust[0] <= .125:
                raise ValueError('P763_FORMAL_RECOVERY_NATIVE_THRUST_REQUIRED')
            seed = dict(rates=pose.omega.tolist(), thrust=-self.thrust[1])
            self.formal_outer.initialize(seed['rates'], seed['thrust'])
        self.formal_last_ros = now
        self.formal_barrier.sea = self.current_sea
        if brake:
            if (self.control_mode is None or not self.control_mode.flag_control_rates_enabled
                    or self.control_mode.flag_control_velocity_enabled):
                raise ValueError('P763_FORMAL_RECOVERY_BODYRATE_UNCONFIRMED')
            if self.formal_recovery_phase != self.phase_ros:
                self.formal_recovery.reset()
                self.formal_recovery_phase = self.phase_ros
            output = self.formal_recovery.step(
                self.formal_outer, position, velocity, pose.rotation, pose.omega, now, dt,
                now-native_stamp, self.formal_barrier)
        else:
            if self.brake_origin is None:
                self.brake_origin = position.copy()
            target, vr = self.home, np.zeros(3)
            if desired_velocity is not None:
                vr = desired_velocity
                target = self.brake_origin+vr*max(0., native_stamp-self.phase_ros)
            reference = np.r_[target, vr, np.zeros(3), self.yaw, 0.]
            output = self.formal_outer.step(reference, position, velocity,
                                            pose.rotation, dt, pose.omega)
        self.publish_body(output['rates'], -output['thrust'])
        command_stamp = self.rows[-1]['stamp']
        self.log('formal_control', stamp=now,
                 controller_identity_sha256=self.formal_identity_sha256,
                 phase_begin=self.phase_ros, recovery=bool(brake), seed=seed,
                 inputs=dict(native_stamp=native_stamp, position=position.tolist(),
                             velocity=velocity.tolist(), rotation=pose.rotation.tolist(),
                             omega=pose.omega.tolist(), now=now, dt=dt, age=now-native_stamp,
                             sea=self.current_sea), command_stamp=command_stamp,
                 reference=output['reference'].tolist(),
                 acceleration=output['acceleration'].tolist())

    def land(self):
        if self.land_requested:
            return
        self.land_requested = True
        self.change('LAND')
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.landing_started = time.monotonic()

    def abort(self, reason):
        if self.failure:
            return
        self.failure = reason
        self.log('failure', reason=reason)
        last = getattr(self, 'last_native', None)
        if self.stage == 'A2' and self.body_entered:
            # A failed formal stopping guard relinquishes to native LAND.
            # The latched LAND is never converted into an independent A1 brake.
            pass
        elif self.body_entered and last and 0 <= self.now()-last[0] <= .125:
            self.emit_hold(last[3], last[1], last[2], .005, brake=True)
        elif self.body_entered:
            # State is unavailable: replace any retained maximum pulse with the
            # known hover request before handing over to native LAND protection.
            self.publish_body(self.rate_command, self.hover)
        self.land()

    def make_steps(self):
        if self.stage == 'A1':
            for value in (.3, .4, .5, .6, .7, .85, 1.):
                self.steps += [('BODY_PREP', .65), (f'THRUST_{value:.3f}', .22),
                               ('BODY_BRAKE', 3.), ('BODY_HOLD', 3.)]
            for axis in range(3):
                self.steps += [('BODY_PREP', .65), (f'RATE_{axis}', .65),
                               ('BODY_BRAKE', 3.), ('BODY_HOLD', 3.)]
            self.steps += [('BODY_PREP', .65), ('MANEUVER', 1.7),
                           ('BODY_BRAKE', 3.), ('BODY_HOLD', 3.)]
        else:
            for value in (.5, 1., 2.):
                self.steps += [(f'DESCENT_{value}', 2.5), (f'BRAKE_{value}', 2.),
                               ('BODY_BRAKE', 3.), ('BODY_HOLD', 3.)]
            self.steps.insert(0, ('BODY_PREP', .65))
            self.steps += [('FAULT_ACCEL', 1.2), ('FAULT_RECOVERY', 3.), ('BODY_HOLD', 3.)]

    def next_step(self):
        self.step_index += 1
        if self.step_index >= len(self.steps):
            self.completed = True
            self.land()
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
            elif wall-self.landing_started > 20 and not getattr(self, 'land_timeout', False):
                self.land_timeout = True
                self.failure = self.failure or 'LANDING_DISARM_TIMEOUT'
                self.log('failure', reason=self.failure)
                # The harness retains its bounded process watchdog; an airborne
                # timeout is failure and never safe completion. Keep observing.
            return
        try:
            self.last_ros_time = check_native_epoch(
                self.now(), self.last_ros_time, self.native_states, self.history, self.references)
            if self.state is None or not 0 <= self.now()-self.state[0] <= .125:
                raise ValueError('NATIVE_NAVIGATION_STALE')
            paired, pose, reference = paired_native_state(
                self.native_states, self.history, self.references, self.now())
            stamp, position, velocity, acceleration = paired
            self.last_native = (stamp, position.copy(), velocity.copy(), pose)
            self.log('paired_state', stamp=stamp, receipt=self.now(), age=self.now()-stamp,
                     pose_stamp=pose.stamp, ref_alt=reference)
            sea = reference-self.contract['candidate']['sea_altitude_msl']
            self.current_sea = sea
            if self.home is None:
                self.home = position.copy()
                self.yaw = float(Rotation.from_matrix(pose.rotation).as_euler('xyz')[2])
                self.home[2] = sea-8.-self.base_height
                self.change('PRESTREAM')
            desired = np.clip(1.5*(self.home-position), [-2., -2., -1.2], [2., 2., 1.2])
            elapsed = wall-self.phase_wall
            if self.status is None:
                if self.body_entered:
                    raise ValueError('P763_NATIVE_STATUS_UNAVAILABLE')
                self.emit_velocity([0., 0., 0.])
                return
            armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if not armed:
                if self.body_entered:
                    raise ValueError('UNEXPECTED_DISARM')
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
            if self.phase in ('TAKEOFF', 'HOVER'):
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
                return
            if self.phase in ('BODY_BRAKE', 'BODY_HOLD'):
                self.emit_hold(pose, position, velocity, dt, brake=self.phase == 'BODY_BRAKE')
                if self.phase == 'BODY_BRAKE':
                    stable = np.linalg.norm(velocity) < .25
                else:
                    executed = self.thrust is not None and 0 <= self.now()-self.thrust[0] <= .125
                    stable = settled and executed and abs(self.thrust[1]-self.hover) < .08
                self.settled = (self.settled or wall) if stable else None
                if self.settled and wall-self.settled >= .7:
                    self.next_step()
                elif elapsed > 9.:
                    raise ValueError('FAILED_TO_SETTLE_BETWEEN_ACTIONS')
                return
            if self.phase.startswith('DESCENT_') or self.phase == 'FAULT_ACCEL':
                speed = (float(self.phase.split('_')[1])
                         if self.phase.startswith('DESCENT_') else .8)
                desired = np.array([0., 0., speed])
                self.emit_hold(pose, position, velocity, dt, desired_velocity=desired)
                if (self.phase == 'FAULT_ACCEL' and elapsed >= 1.2 or
                        self.phase.startswith('DESCENT_') and (velocity[2] >= .95*speed
                                                               or elapsed >= 2.5)):
                    self.trial = dict(start=self.now(), initial_speed=float(velocity[2]),
                                      response_seconds=None, decelerations=[], previous=paired,
                                      initial_z=float(position[2]))
                    self.next_step()
                return
            if self.phase.startswith('BRAKE_') or self.phase == 'FAULT_RECOVERY':
                self.emit_hold(pose, position, velocity, dt, brake=True)
                trial, previous = self.trial, self.trial['previous']
                if stamp > previous[0]:
                    deceleration = (previous[2][2]-velocity[2])/(stamp-previous[0])
                    trial['decelerations'].append(float(deceleration))
                    trial['previous'] = paired
                if trial['response_seconds'] is None and velocity[2] < trial['initial_speed']-.05:
                    trial['response_seconds'] = max(.001, stamp-trial['start'])
                if np.linalg.norm(velocity) <= .1 and elapsed >= .15:
                    positive = [v for v in trial['decelerations'] if v > 0]
                    self.log('brake_trial' if self.phase.startswith('BRAKE_')
                             else 'bodyrate_recovered', initial_speed=trial['initial_speed'],
                             initial_velocity=[0., 0., trial['initial_speed']], stopped=True,
                             response_seconds=trial['response_seconds'] or 0.,
                             maximum_downward_displacement=float(
                                 max(0., position[2]-trial['initial_z'])),
                             reliable_deceleration=float(np.percentile(positive, 10))
                             if positive else 0.)
                    self.next_step()
                elif elapsed >= 3.:
                    raise ValueError('BODYRATE_RECOVERY_FAILED')
                return
            thrust = self.hover
            axis, desired_pitch = None, 0.
            if self.phase == 'BODY_PREP':
                if not self.body_entered:
                    if self.thrust is None or not 0 <= self.now()-self.thrust[0] <= .125:
                        raise ValueError('P763_BODY_ENTRY_THRUST_STALE')
                    self.rate_command = np.asarray(pose.omega, float).copy()
                self.emit_hold(pose, position, velocity, dt)
                if elapsed >= .65:
                    if np.linalg.norm(velocity) >= .25 or self.control_mode is None or not (
                            self.control_mode.flag_control_rates_enabled
                            and not self.control_mode.flag_control_velocity_enabled):
                        raise ValueError('P763_BODY_PREP_NOT_STABLE')
                    self.next_step()
                return
            if self.phase.startswith('THRUST_'):
                thrust = float(self.phase.split('_')[1])
                guarded = any(r['kind'] == 'pulse_guard'
                              and r.get('phase_begin') == self.phase_ros for r in self.rows)
                if thrust > self.hover and not guarded:
                    guard = pulse_admission(
                        velocity=velocity, acceleration=acceleration,
                        sample_age=self.now()-stamp, duration=.22, mass=self.mass,
                        maximum_thrust=self.model_maximum_thrust, gravity=self.gravity,
                        upward_room=position[2]-(self.home[2]-4.),
                        clearance=sea-lowest_world_z(self.points, position, pose.rotation),
                        upward_braking=self.gravity-self.bootstrap.force[0]/self.mass,
                        downward_braking=self.bootstrap.force[-1]/self.mass-self.gravity)
                    self.log('pulse_guard', phase_begin=self.phase_ros, **guard)
                    if not guard['admitted']:
                        raise ValueError('P763_HIGH_PULSE_UNSAFE_REFUSED')
            elif self.phase.startswith('RATE_'):
                axis = int(self.phase.split('_')[1])
            elif self.phase == 'MANEUVER':
                curve = (self.prior['measured_curve'] if self.prior else [
                    dict(normalized=u, force=float(np.median(values)))
                    for u, values in sorted(self.force_samples.items()) if len(values) >= 3])
                if len(curve) < 5:
                    raise ValueError('MEASURED_CURVE_REQUIRED_FOR_DYNAMICS')
                desired_pitch = -.5
                force = self.mass*self.gravity/math.cos(desired_pitch)
                thrust = float(np.interp(force, [c['force'] for c in curve],
                                         [c['normalized'] for c in curve]))
            if elapsed >= self.steps[self.step_index][1]:
                self.next_step()
                if self.phase == 'BODY_BRAKE':
                    self.emit_hold(pose, position, velocity, dt, brake=True)
                return
            self.emit_body(pose, min(.02, max(.001, dt)), thrust, axis, desired_pitch)
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
            schema_version=2, architecture=ARCHITECTURE,
            completed=node.completed and not node.failure, failure=node.failure,
            stage=args.stage, elapsed=time.monotonic()-node.started,
            scope='Actual isolated PX4/Gazebo calibration; not a target capture'), indent=2)+'\n')
        node.raw.close()
        node.destroy_node()
        rclpy.shutdown()
    outcome = stage_verdict(
        args.stage, dict(schema_version=2, architecture=ARCHITECTURE,
                         completed=node.completed, failure=node.failure), analysis, node.rows,
        node.identity['sources'] == source_fingerprint(Path(os.environ['UAV_USV_WS'])),
        True, node.prior is not None or args.stage == 'A1')
    return 0 if outcome['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

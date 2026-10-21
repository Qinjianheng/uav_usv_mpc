"""Sole Tracker's opt-in body-rate path; no second PX4 controller or truth subscription."""
import json
import math
import os
from collections import deque

import numpy as np
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint
from std_msgs.msg import String
from uav_usv_interfaces.msg import NativeAttitude, TargetImageState

from uav_control.common.p76_contract import load_contract, parameter_bindings
from uav_control.common.p76_body_geometry import collision_envelope, navigation_vertices
from uav_control.common.p76_native import NativePoseHistory, NativeReferenceHistory
from uav_control.common.p76_phase import phase_qos
from uav_control.common.p76_readiness import validate_calibration, validate_follow_qualification
from uav_control.perception.yolo_detection_contract import stamp_seconds
from .p76_bodyrate_safety import BodyrateSeaBarrier
from .p76_mode_coordinator import ModeCoordinator
from .p76_paper_controller import PaperController
from .p76_paper_thrust import CalibratedThrustMap
from .p76_minco_bodyrate_tracker import BodyrateTracker, WaypointReference, sample_reference
from .p76_recovery import BodyrateRecovery


class P76TrackerRuntime:
    """The 20Hz callback supervises admission; only this 200Hz runtime emits PX4 commands."""

    def __init__(self, node):
        self.node = node
        node.declare_parameter('p76_contract_file', '')
        self.contract = load_contract(node.get_parameter('p76_contract_file').value)
        for key, value in parameter_bindings(self.contract)['trajectory_tracker_node'].items():
            if not node.has_parameter(key):
                node.declare_parameter(key, value)
            if node.get_parameter(key).value != value:
                raise ValueError('P76_TRACKER_PARAMETER_MISMATCH:'+key)
        from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
        require_isolated_sitl()
        model_root = os.environ.get('UAV_USV_P76_MODEL_ROOT', '')
        workspace = os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv_mpc')
        calibration = validate_calibration(self.contract, model_root, workspace)
        self.workspace = workspace
        self.map = CalibratedThrustMap(calibration['force_newtons'], calibration['normalized'],
                                       qualified=True)
        barrier = BodyrateSeaBarrier(
            navigation_vertices(collision_envelope(model_root), model_root),
            self.contract['candidate']['minimum_body_clearance'], calibration['response_seconds'],
            calibration['braking_acceleration'], self.contract['candidate']['sea_surface_z'])
        self.controller = PaperController(self.contract, calibration['mass'],
                                          calibration['maximum_thrust'], self.map, barrier)
        if self.contract.get('architecture') != 'parallel_dkf_unified_bodyrate':
            raise ValueError('P763_ARCHITECTURE_REQUIRED')
        self.outer = BodyrateTracker(self.contract, calibration['mass'],
                                     calibration['maximum_thrust'], self.map)
        self.recovery = BodyrateRecovery(calibration['formal_recovery']['braking_request'],
                                         max(.24, calibration['response_seconds']))
        self.recovery_phase = None
        self.image_ack = None
        self.follow_revoked = False
        self.waypoint, self.waypoint_state = None, None
        self.mode_wait_started = None
        self.land_observation = {}
        self.mode = ModeCoordinator(self.contract['timing']['handover_timeout'])
        self.history = NativePoseHistory(self.contract['timing']['maximum_pose_bracket'])
        self.references = NativeReferenceHistory()
        self.image, self.spatial_ack, self.px4_mode = None, None, None
        self.last_tick, self.last_report, self.initialized = None, 0., False
        self.thrust_hold = None
        self.executed_thrust = None
        self.fatal_dispatched = False
        self._reset_initialization()
        self.mode_pub = node.create_publisher(String, '/control/p76_mode', phase_qos())
        self.diagnostic_pub = node.create_publisher(String, '/control/p76_paper', 10)
        self.rates_pub = node.create_publisher(VehicleRatesSetpoint,
                                               '/fmu/in/vehicle_rates_setpoint', 10)
        self.image_sub = node.create_subscription(TargetImageState, '/tracking/image_state',
                                                  self.receive_image, 10)
        self.pose_sub = node.create_subscription(NativeAttitude, '/navigation/native_attitude',
                                                 self.receive_pose, 16)
        self.ack_sub = node.create_subscription(String, '/tracking/p76_filter_ack', self.ack, 10)
        self.control_sub = node.create_subscription(String, '/navigation/px4_control_mode',
                                                    self.control_mode, 10)
        self.thrust_sub = node.create_subscription(String, '/navigation/native_thrust',
                                                   self.receive_thrust, 10)
        self.reference_sub = node.create_subscription(
            String, '/navigation/native_local_reference', self.receive_reference, 16)
        self._install_landing_observers()
        self.timer = node.create_timer(
            1/self.contract['paper']['paper_controller_target_rate_hz'], self.tick)
        self.publish_phase()

    def publish_phase(self):
        """Expose unique control ownership; mode epoch never changes on detector loss."""
        self.mode_pub.publish(String(data=json.dumps(dict(
            schema_version=2, phase=self.mode.phase, epoch=self.mode.epoch,
            guidance_owner=self.mode.guidance_owner, px4_interface='BODY_RATE',
            control_state=self.control_state,
            y_stamp=self.mode.y_stamp,
            mission_id=self.node.mission_id, reason=self.mode.reason))))

    def receive_image(self, message):
        """Accept only the active epoch, pinned model and fresh real image authority."""
        if message.mode_epoch == self.mode.epoch:
            self.image = message

    def receive_pose(self, message):
        """Use the independent physical attitude relay throughout both estimator phases."""
        if message.valid:
            try:
                self.history.add(stamp_seconds(message.stamp), message.quaternion_wxyz,
                                 message.angular_velocity_frd, message.orientation_variance)
            except ValueError:
                self.mode.fail('NATIVE_ATTITUDE_INVALID')

    def ack(self, message):
        """Track continuous estimator readiness and separate guidance authorization."""
        try:
            data = json.loads(message.data)
            if data.get('schema_version') == 2 and data['epoch'] == self.mode.epoch:
                if data['estimator'] == 'spatial':
                    self.spatial_ack = data
                elif data['estimator'] == 'image':
                    self.image_ack = data
        except (ValueError, KeyError, TypeError):
            return

    def control_mode(self, message):
        """Use causally recovered PX4 mode publication time, never callback receipt time."""
        try:
            packet = json.loads(message.data)
            stamp = float(packet['stamp'])
            flags = packet['flags']
            if (packet['native_timestamp'] > 0 and packet['px4_timestamp'] > 0
                    and 0 < stamp <= self.node._ros_seconds()
                    and self.node._ros_seconds()-stamp <= .125
                    and all(type(flags[key]) is bool for key in (
                        'offboard', 'rates', 'position', 'velocity', 'attitude',
                        'altitude', 'climb_rate'))):
                self.px4_mode = stamp, packet['flags']
        except (ValueError, KeyError, TypeError):
            return

    def receive_thrust(self, message):
        """Require real PX4 executed setpoint feedback; acceleration is not a substitute."""
        try:
            packet = json.loads(message.data)
            stamp, values = float(packet['stamp']), np.asarray(packet['xyz'])
            if (packet['native_timestamp'] > 0 and packet['px4_timestamp'] > 0
                    and 0 < stamp <= self.node._ros_seconds()
                    and self.node._ros_seconds()-stamp <= .125
                    and values.shape == (3,) and np.isfinite(values).all()
                    and -1 <= values[2] < 0 and np.max(np.abs(values[:2])) < 1e-6):
                self.executed_thrust = stamp, values.copy()
                self.executed_thrust_identity = dict(
                    native_timestamp=int(packet['native_timestamp']),
                    px4_timestamp=int(packet['px4_timestamp']))
        except (ValueError, KeyError, TypeError):
            return

    def receive_reference(self, message):
        """Use PX4's native local datum, independently of the target and truth evaluator."""
        try:
            packet = json.loads(message.data)
            stamp = float(packet['stamp'])
            if (packet['reference'] == 'base_link' and packet['px4_timestamp'] > 0
                    and packet['native_timestamp'] > 0
                    and 0 <= self.node._ros_seconds()-stamp <= .125):
                self.references.add(stamp, float(packet['ref_alt']))
        except (ValueError, KeyError, TypeError):
            return

    def accept_y(self, now):
        """Withdraw the MINCO lease; retain both estimators and the physical Body-rate mode."""
        if self.mode.phase != 'FOLLOW_ACTIVE':
            return
        qualified = os.environ.get('UAV_USV_P761_STAGE') != 'B'
        try:
            validate_follow_qualification(self.contract, self.workspace)
        except (ValueError, OSError):
            qualified = False
        self.node._revoke_nominal_follow('P763_Y_AUTHORITY_HANDOVER')
        self.node.pending_trajectory = self.node.tracker.active_trajectory = None
        self.node.terminal_execution = None
        self.follow_revoked = True
        self.mode.accept_y(now, qualified)
        self.publish_phase()

    def _image_valid(self, now):
        image = self.image
        return bool(image is not None and image.valid and image.mode_epoch == self.mode.epoch
                    and image.model_sha256 == self.contract['detector']['model_sha256']
                    and 0 <= now-stamp_seconds(image.stamp) <= .125
                    and 0 <= now-stamp_seconds(image.source_stamp) <= .125)

    def _paired_navigation(self, stamp):
        rows = self.node.follow_navigation_history
        for row in rows:
            if abs(row['stamp']-stamp) < 1e-9:
                return np.asarray(row['p']), np.asarray(row['v'])
        for left, right in zip(rows, list(rows)[1:]):
            if left['stamp'] < stamp < right['stamp'] and right['stamp']-left['stamp'] <= .05:
                weight = (stamp-left['stamp'])/(right['stamp']-left['stamp'])
                return tuple((1-weight)*np.asarray(left[k])+weight*np.asarray(right[k])
                             for k in ('p', 'v'))
        raise ValueError('P76_NATIVE_NAVIGATION_BRACKET')

    def _snapshot(self, now):
        for row in reversed(self.node.follow_navigation_history):
            stamp = row['stamp']
            if not 0 <= now-stamp <= .125:
                continue
            try:
                pose = self.history.at(stamp)
                sea = self.references.at(stamp)-self.contract['candidate']['sea_altitude_msl']
                return stamp, np.asarray(row['p']), np.asarray(row['v']), pose, sea
            except ValueError:
                continue
        raise ValueError('P763_COMPLETE_NATIVE_PAIR_UNAVAILABLE')

    def _emit_rates(self, rates, thrust, now):
        heartbeat = OffboardControlMode(timestamp=self.node._timestamp_us(), body_rate=True)
        self.node.offboard_pub.publish(heartbeat)
        message = VehicleRatesSetpoint(timestamp=self.node._timestamp_us())
        message.roll, message.pitch, message.yaw = map(float, rates)
        message.thrust_body = [0., 0., float(thrust)]
        self.rates_pub.publish(message)
        self.last_emitted = dict(stamp=now, rates=list(map(float, rates)), thrust=float(thrust))
        if not hasattr(self, 'output_history'):
            self.output_history = deque(maxlen=128)
        self.output_history.append(dict(self.last_emitted,
                                        reference=getattr(self, 'emission_reference', None)))

    def _bodyrate_confirmed(self, now):
        if self.px4_mode is None:
            return False
        stamp, flags = self.px4_mode
        return bool(0 <= now-stamp <= self.contract['timing']['mode_feedback_max_age']
                    and flags['offboard'] and flags['rates']
                    and not any(flags[k] for k in (
                        'velocity', 'position', 'attitude', 'altitude', 'climb_rate')))

    def _reset_initialization(self):
        """Begin with no qualified executed output, even if PX4 is armed."""
        self.control_state, self.previous_armed = 'DISARMED_PRESTREAM', False
        self.armed_at, self.initialization_count, self.native_arm_epoch = None, 0, 0
        self.initialization_reason, self.initialization_margin = '', None

    def _observe_armed_edge(self, now):
        """Repeated status messages cannot reseed; disarm explicitly revokes the old seed."""
        armed = bool(self.node.vehicle_armed)
        native_epoch = int(getattr(self, 'land_observation', {}).get('armed_time_native', 0))
        if (armed != self.previous_armed or armed and native_epoch > 0
                and native_epoch != getattr(self, 'native_arm_epoch', 0)):
            self.outer.reset()
            self.initialization_margin = None
            self.waypoint, self.waypoint_state = None, None
            self.previous_armed = armed
            self.armed_at = now if armed else None
            self.native_arm_epoch = native_epoch if armed else 0
            self.control_state = ('ARMED_WAIT_NATIVE_FEEDBACK' if armed
                                  else 'DISARMED_PRESTREAM')
            if not armed:
                self.fatal_dispatched = False
        return armed

    def _initialize_armed(self, now, dt):
        """Confirm all native inputs before granting one real first output to this arm edge."""
        maximum_wait = self.contract['timing']['mode_feedback_max_age']
        if now-self.armed_at > maximum_wait:
            raise ValueError('P764_ARMED_NATIVE_FEEDBACK_TIMEOUT:'+self.initialization_reason)
        if not self._bodyrate_confirmed(now):
            self.initialization_reason = 'P764_BODYRATE_MODE_UNCONFIRMED'
            # No native hold qualification exists in this architecture. Without fresh mode
            # confirmation even a no-output wait cannot prove who is controlling the motors.
            raise ValueError(self.initialization_reason)
        # A confirmed Body-rate mode already owns the actuator. Missing feedback cannot be
        # bridged by an armed zero packet, an inferred hover value, or a fabricated rate seed.
        native = self.land_observation
        arm_epoch = int(native.get('armed_time_native', 0))
        identity = getattr(self, 'executed_thrust_identity', {})
        if (self.executed_thrust is None or arm_epoch <= 0
                or identity.get('native_timestamp', 0) < arm_epoch
                or not 0 < self.executed_thrust[0] <= now
                or now-self.executed_thrust[0] > .125):
            raise ValueError('P764_EXECUTED_THRUST_SEED_UNAVAILABLE')
        if (native.get('disarmed', True) or native.get('status_native_wire', 0) <= 0
                or not 0 <= now-native.get('status_receipt', -math.inf) <= maximum_wait):
            raise ValueError('P764_NATIVE_ARMED_CONFIRMATION_UNAVAILABLE')
        stamp, position, velocity, pose, sea = self._snapshot(now)
        self.controller.barrier.sea = sea
        thrust = float(self.executed_thrust[1][2])
        self.initialization_margin = self.outer.hover_initialization_margin(
            position, velocity, pose.rotation, thrust, now-stamp, dt, self.controller.barrier)
        self.outer.initialize(pose.omega, thrust)
        self.initialization_count += 1
        self.initialization_reason = ''
        self.control_state = 'BODYRATE_READY'
        return dict(rates=pose.omega, thrust=thrust, acceleration=np.zeros(3))

    def _install_landing_observers(self):
        from px4_msgs.msg import VehicleLandDetected, VehicleStatus
        from rclpy.qos import qos_profile_sensor_data

        def status(message):
            self.land_observation.update(status_receipt=self.node._ros_seconds(),
                                         status_native_wire=int(message.timestamp),
                                         armed_time_native=int(message.armed_time),
                                         nav_state=int(message.nav_state),
                                         disarmed=message.arming_state == 1)

        def landed(message):
            self.land_observation.update(land_receipt=self.node._ros_seconds(),
                                         land_native_wire=int(message.timestamp),
                                         landed=bool(message.landed))
        self.land_subscriptions = []
        for name, cls, callback in (('vehicle_status', VehicleStatus, status),
                                    ('vehicle_land_detected', VehicleLandDetected, landed)):
            version = getattr(cls, 'MESSAGE_VERSION', 0)
            topic = '/fmu/out/'+name+(f'_v{version}' if version else '')
            self.land_subscriptions.append(self.node.create_subscription(
                cls, topic, callback, qos_profile_sensor_data))

    def _land(self, reason, now):
        """One latched native LAND; no alternating control interfaces or false completion timer."""
        self.mode.fail(reason)
        self.control_state = 'LAND_REQUESTED'
        self.node._revoke_nominal_follow(reason)
        self.follow_revoked = True
        if not self.fatal_dispatched and self.node.vehicle_armed:
            from px4_msgs.msg import VehicleCommand
            self.node._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.)
            self.fatal_dispatched, self.land_requested = True, now

    def _follow_output(self, now, dt, stamp, position, velocity, pose):
        state = self.node.mission_state_name
        if not self._bodyrate_confirmed(now):
            raise ValueError('P763_BODYRATE_MODE_UNCONFIRMED')
        if state == 'FOLLOW':
            context = self.node._follow_context()
            if not (context.following and context.visible and context.navigation_fresh
                    and context.target_fresh and context.prediction_fresh
                    and context.observation_fresh and not context.recovering):
                raise ValueError('P763_FOLLOW_CONTEXT_LOST')
            receiver = self.node.follow_receiver
            curve = receiver.active
            if curve is not None:
                if self.follow_revoked or curve.mission_id != self.node.mission_id:
                    raise ValueError('P763_FOLLOW_AUTHORITY_REVOKED')
                reference = sample_reference(curve, now, receiver.deadline)
                feedback = sample_reference(curve, stamp, receiver.deadline)
                self.emission_reference = dict(stamp=stamp, position=feedback[:3].tolist(),
                                               velocity=feedback[3:6].tolist(),
                                               acceleration=feedback[6:9].tolist())
                return self.outer.step(reference, position, velocity, pose.rotation,
                                       dt, pose.omega, feedback_reference=feedback), 'MINCO_FOLLOW'
        if self.waypoint is None or self.waypoint_state != state:
            goal = position.copy()
            if state not in ('INIT', 'GROUND_HOLD'):
                goal[2] = self.node.flight_guidance.flight_altitude
            duration = max(2., 2.5*float(np.linalg.norm(goal-position)))
            self.waypoint = WaypointReference(stamp, position, velocity, goal, duration,
                                              self.node.current_heading)
            self.waypoint_state = state
        reference = self.waypoint.sample(now)
        feedback = self.waypoint.sample(stamp)
        return self.outer.step(reference, position, velocity, pose.rotation,
                               dt, pose.omega, feedback_reference=feedback), (
                                   'HOLD' if state == 'FOLLOW' else 'SETUP')

    def _paper_output(self, now, dt):
        valid = self._image_valid(now)
        if self.mode.phase == 'PAPER_INTERCEPT_ACTIVE' and not valid:
            raise ValueError('P76_IMAGE_SOURCE_STALE')
        if not valid:
            if self.initialized:
                raise ValueError('P76_HANDOVER_IMAGE_SOURCE_STALE')
            return None
        stamp = stamp_seconds(self.image.pose_stamp)
        pose = self.history.at(stamp)
        position, velocity = self._paired_navigation(stamp)
        self.controller.barrier.sea = (
            self.references.at(stamp)-self.contract['candidate']['sea_altitude_msl'])
        if self.mode.phase == 'Y_HANDOVER':
            ack = self.image_ack
            image_ready = bool(ack and ack.get('schema_version') == 2 and ack['active']
                               and ack['ready'] and ack['control_authorized']
                               and ack['epoch'] == self.mode.epoch
                               and 0 <= now-ack['stamp'] <= .125)
            receiver = self.node.follow_receiver
            revoked = self.follow_revoked and receiver.active is None and receiver.pending is None
            ready = self.mode.ready(now, revoked, stamp_seconds(self.image.source_stamp),
                                    0 <= now-stamp <= .125, image_ready)
            if not ready:
                return None
            if self.executed_thrust is None or not 0 <= now-self.executed_thrust[0] <= .125:
                raise ValueError('P76_EXECUTED_THRUST_FEEDBACK_UNAVAILABLE')
            if not self.initialized:
                self.controller.initialize(self.image.state, pose.rotation, velocity, stamp,
                                           pose.omega)
                self.initialized = True
                self.mode.request_bodyrate(now)
            output = self.outer.limit_output(dict(rates=pose.omega,
                                                  thrust=float(self.executed_thrust[1][2]),
                                                  acceleration=np.zeros(3)), dt)
            if self._bodyrate_confirmed(now):
                self.mode.confirm(now, self.px4_mode[0], True,
                                  self.contract['timing']['mode_feedback_max_age'])
            return output
        if not self._bodyrate_confirmed(now):
            raise ValueError('P763_BODYRATE_MODE_LOST')
        output = self.controller.step(self.image.state, pose.rotation, position, velocity,
                                      stamp, dt, navigation_age=now-stamp)
        return self.outer.limit_output(output, dt)

    def _recovery(self, now, dt, snapshot=None):
        """Only calibrated target-independent body braking may retain external control."""
        if snapshot is None:
            snapshot = self._snapshot(now)
        stamp, position, velocity, pose, sea = snapshot
        self.controller.barrier.sea = sea
        if not self._bodyrate_confirmed(now):
            raise ValueError('P763_RECOVERY_BODYRATE_UNCONFIRMED')
        if getattr(self, 'recovery_phase', None) != self.mode.phase:
            self.recovery.reset()
            self.recovery_phase = self.mode.phase
        return self.recovery.step(self.outer, position, velocity, pose.rotation, pose.omega,
                                  now, dt, now-stamp, self.controller.barrier)

    def tick(self):
        """Sole physical outlet during ground prestream, X, Y and bounded recovery."""
        now = self.node._ros_seconds()
        if not hasattr(self, 'control_state'):
            self._reset_initialization()
        dt = (now-self.last_tick if self.last_tick is not None else
              1/self.contract['paper']['paper_controller_target_rate_hz'])
        self.last_tick = now
        self.emission_reference = None
        owner = 'NONE'
        armed = self._observe_armed_edge(now)
        if getattr(self.node, 'navigation_frame_fault', ''):
            self._land('P76_NAVIGATION_FRAME_FAULT', now)
        try:
            if not math.isfinite(now) or not 0 < dt <= .125:
                self.outer.reset()
                raise ValueError('P76_CONTROL_CLOCK_OR_DEADLINE')
            self.mode.timeout(now)
            if not self.fatal_dispatched and self.mode.phase != 'CAPTURED':
                if not armed:
                    # Native callbacks arrive asynchronously after the timer starts. Disarmed
                    # zero prestream grants no flight readiness and cannot latch a flight fault.
                    self._emit_rates(np.zeros(3), 0., now)
                    owner = 'DISARMED_PRESTREAM'
                elif self.control_state == 'ARMED_WAIT_NATIVE_FEEDBACK':
                    output = self._initialize_armed(now, dt)
                    owner = self.control_state
                    if output is not None:
                        self._emit_rates(output['rates'], output['thrust'], now)
                else:
                    if not self._bodyrate_confirmed(now):
                        raise ValueError('P763_BODYRATE_MODE_LOST')
                    snapshot = self._snapshot(now)
                    stamp, position, velocity, pose, sea = snapshot
                    self.controller.barrier.sea = sea
                    speed = float(np.linalg.norm(velocity))
                    if (not math.isfinite(speed)
                            or speed > self.contract['candidate']['maximum_horizontal_speed']):
                        reason = ('P764_NATIVE_SPEED_LIMIT_EXCEEDED' if math.isfinite(speed)
                                  else 'P764_NATIVE_SPEED_STATE_INVALID')
                        self.node._revoke_nominal_follow(reason)
                        self.follow_revoked = True
                        raise ValueError(reason)
                    if self.outer.limiter.thrust is None:
                        raise ValueError('P764_QUALIFIED_OUTPUT_SEED_REQUIRED')
                    if self.mode.phase == 'FOLLOW_ACTIVE':
                        output, owner = self._follow_output(
                            now, dt, stamp, position, velocity, pose)
                    elif self.mode.phase in ('Y_HANDOVER', 'PAPER_INTERCEPT_ACTIVE'):
                        output = self._paper_output(now, dt)
                        owner = ('PAPER' if self.mode.phase == 'PAPER_INTERCEPT_ACTIVE'
                                 else 'HANDOVER')
                        if output is None:
                            # Guidance has no owner while awaiting warm estimator labels. Bounded
                            # native braking replaces target feedback without a PX4 mode change.
                            output = self._recovery(now, dt, snapshot)
                    else:
                        output, owner = self._recovery(now, dt, snapshot), 'BODYRATE_RECOVERY'
                    safety = self.recovery.check_margin(
                        self.outer, position, velocity, pose.rotation, output['acceleration'],
                        now-stamp, dt, self.controller.barrier)
                    if safety != 'CLEAR':
                        raise ValueError('P76_SEA_'+safety)
                    self._emit_rates(output['rates'], output['thrust'], now)
                    self.control_state = ('SAFE_RECOVERY' if self.mode.phase == 'SAFE_RECOVERY'
                                          else 'FLIGHT_ACTIVE')
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            self.mode.fail(str(error))
            if armed and not self.fatal_dispatched:
                if (self.control_state == 'ARMED_WAIT_NATIVE_FEEDBACK'
                        or str(error) == 'P76_CONTROL_CLOCK_OR_DEADLINE'):
                    self._land(str(error), now)
                else:
                    self.control_state = 'SAFE_RECOVERY'
                    try:
                        output = self._recovery(now, min(.125, max(.001, dt)))
                        self._emit_rates(output['rates'], output['thrust'], now)
                        owner = 'BODYRATE_RECOVERY'
                    except (ValueError, TypeError, np.linalg.LinAlgError):
                        self._land(str(error), now)
        if now-self.last_report >= .05:
            self.last_report = now
            self.publish_phase()
            status = 'MINCO_FOLLOW' if owner == 'MINCO_FOLLOW' else self.mode.phase
            self.node._publish_diagnostic(now, None, status, 0.)
            self.diagnostic_pub.publish(String(data=json.dumps(dict(
                schema_version=2, stamp=now, phase=self.mode.phase,
                guidance_owner=owner, px4_interface='BODY_RATE', reason=self.mode.reason,
                control_state=self.control_state,
                initialization_count=self.initialization_count,
                initialization_reason=self.initialization_reason,
                initialization_margin=self.initialization_margin,
                source_stamp=stamp_seconds(self.image.source_stamp) if self.image else 0.,
                output=getattr(self, 'last_emitted', None), actual_dt=dt,
                output_history=list(getattr(self, 'output_history', ())),
                land=dict(self.land_observation,
                          requested=getattr(self, 'land_requested', None))))))
            if hasattr(self, 'output_history'):
                self.output_history.clear()

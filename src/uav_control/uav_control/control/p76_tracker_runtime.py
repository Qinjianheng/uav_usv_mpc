"""Sole Tracker's opt-in body-rate path; no second PX4 controller or truth subscription."""
import json
import os
from types import SimpleNamespace

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


class P76TrackerRuntime:
    """FOLLOW uses the existing 20Hz path; Y owns the exclusive target-200Hz timer."""

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
        self.mode = ModeCoordinator(self.contract['timing']['handover_timeout'])
        self.history = NativePoseHistory(self.contract['timing']['maximum_pose_bracket'])
        self.references = NativeReferenceHistory()
        self.image, self.spatial_ack, self.px4_mode = None, None, None
        self.last_tick, self.last_report, self.initialized = None, 0., False
        self.thrust_hold = None
        self.executed_thrust = None
        self.fatal_dispatched = False
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
        self.timer = node.create_timer(
            1/self.contract['paper']['paper_controller_target_rate_hz'], self.tick)
        self.publish_phase()

    def publish_phase(self):
        """Expose unique control ownership; mode epoch never changes on detector loss."""
        self.mode_pub.publish(String(data=json.dumps(dict(
            phase=self.mode.phase, epoch=self.mode.epoch, y_stamp=self.mode.y_stamp,
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
        """Track an actual spatial-node freeze acknowledgement in the current epoch."""
        try:
            data = json.loads(message.data)
            if data['epoch'] == self.mode.epoch and data['estimator'] == 'spatial':
                self.spatial_ack = data
        except (ValueError, KeyError, TypeError):
            return

    def control_mode(self, message):
        """Use causally recovered PX4 mode publication time, never callback receipt time."""
        try:
            packet = json.loads(message.data)
            stamp = float(packet['stamp'])
            flags = packet['flags']
            if (0 < stamp <= self.node._ros_seconds() and self.node._ros_seconds()-stamp <= .125
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
            if (0 < stamp <= self.node._ros_seconds() and self.node._ros_seconds()-stamp <= .125
                    and values.shape == (3,) and np.isfinite(values).all()
                    and -1 <= values[2] < 0 and np.max(np.abs(values[:2])) < 1e-6):
                self.executed_thrust = stamp, values.copy()
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
        """Revoke MINCO after the mission manager accepts Y and before freezing inputs."""
        qualified = True
        try:
            validate_follow_qualification(self.contract, self.workspace)
        except (ValueError, OSError):
            qualified = False
        self.node._revoke_nominal_follow('P76_Y_HANDOVER')
        self.node.pending_trajectory = self.node.tracker.active_trajectory = None
        self.node.terminal_execution = None
        self.mode.accept_y(now, qualified)
        self.publish_phase()

    def _image_valid(self, now):
        image = self.image
        return bool(image is not None and image.valid and image.mode_epoch == self.mode.epoch
                    and image.model_sha256 == self.contract['detector']['model_sha256']
                    and 0 <= now-stamp_seconds(image.stamp) <= .125
                    and 0 <= now-stamp_seconds(image.source_stamp) <= .125
                    and stamp_seconds(image.source_stamp) > self.mode.y_stamp)

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

    def _emit_rates(self, rates, thrust, now):
        heartbeat = OffboardControlMode(timestamp=self.node._timestamp_us(), body_rate=True)
        self.node.offboard_pub.publish(heartbeat)
        message = VehicleRatesSetpoint(timestamp=self.node._timestamp_us())
        message.roll, message.pitch, message.yaw = map(float, rates)
        message.thrust_body = [0., 0., float(thrust)]
        self.rates_pub.publish(message)
        self.last_emitted = dict(stamp=now, rates=list(map(float, rates)), thrust=float(thrust))

    def _recovery(self, now, dt):
        """Switch once to controlled velocity braking; retain heartbeat, never zero thrust."""
        state = self.node.latest_state
        if state is None or not 0 <= now-state.stamp <= .125:
            # Stopping heartbeat invokes PX4's configured offboard-loss failsafe.
            return
        from .trajectory_tracker_node import command_to_setpoint
        velocity = np.asarray(state.velocity)
        braking = self.controller.barrier.braking
        delta = min(1., braking*dt/max(np.linalg.norm(velocity), 1e-9))
        desired = velocity*(1-delta)
        command = SimpleNamespace(position=state.position, velocity=desired,
                                  acceleration=(desired-velocity)/dt)
        self.node._publish_offboard_mode(self.node._timestamp_us(), velocity_control=True)
        self.node.setpoint_pub.publish(command_to_setpoint(
            command, self.node._timestamp_us(), velocity_mode=True))

    def tick(self):
        """Only this callback emits setpoints outside FOLLOW; the old timer returns early."""
        if self.mode.phase == 'FOLLOW_ACTIVE':
            return
        now = self.node._ros_seconds()
        dt = 1/self.contract['paper']['paper_controller_target_rate_hz']
        if self.last_tick is not None:
            dt = now-self.last_tick
        self.last_tick = now
        if dt <= 0 or dt > .125:
            self.mode.fail('P76_CONTROL_CLOCK_OR_DEADLINE')
        if getattr(self.node, 'navigation_frame_fault', ''):
            self.mode.fail('P76_NAVIGATION_FRAME_FAULT')
            self.publish_phase()
            return
        self.mode.timeout(now)
        try:
            valid = self._image_valid(now)
            if (self.mode.phase == 'Y_HANDOVER' and self.initialized and not valid):
                # Once body-rate prestream begins, stale imagery must latch recovery.
                # Never silently resume velocity output while awaiting its PX4 ACK.
                raise ValueError('P76_HANDOVER_IMAGE_SOURCE_STALE')
            if self.mode.phase == 'PAPER_INTERCEPT_ACTIVE' and not valid:
                raise ValueError('P76_IMAGE_SOURCE_STALE')
            if self.mode.phase in ('Y_HANDOVER', 'PAPER_INTERCEPT_ACTIVE') and valid:
                stamp = stamp_seconds(self.image.pose_stamp)
                pose = self.history.at(stamp)
                position, velocity = self._paired_navigation(stamp)
                # The SDF collision envelope is now about base_link. The sea plane
                # must likewise use this PX4 local frame, not Gazebo's world Z=0.
                self.controller.barrier.sea = (
                    self.references.at(stamp)-self.contract['candidate']['sea_altitude_msl'])
                ack = self.spatial_ack
                inactive = bool(ack and not ack['active'] and 0 <= now-ack['stamp'] <= .125)
                ready = self.mode.ready(now, inactive, stamp_seconds(self.image.source_stamp),
                                        0 <= now-stamp <= .125)
                if ready and not self.initialized:
                    if (self.executed_thrust is None
                            or now-self.executed_thrust[0] > .125):
                        raise ValueError('P76_EXECUTED_THRUST_FEEDBACK_UNAVAILABLE')
                    limits = self.contract['candidate']
                    bound = np.array((limits['maximum_body_roll_pitch_rate'],)*2
                                     + (limits['maximum_body_yaw_rate'],))
                    if np.any(np.abs(pose.omega) > bound):
                        raise ValueError('P76_HANDOVER_RATE_OUTSIDE_CALIBRATION')
                    self.controller.initialize(self.image.state, pose.rotation, velocity,
                                               stamp, pose.omega)
                    self.thrust_hold = float(self.executed_thrust[1][2])
                    self.initialized = True
                if self.initialized and self.mode.phase == 'Y_HANDOVER':
                    self.mode.request_bodyrate(now)
                    self._emit_rates(pose.omega, self.thrust_hold, now)
                    if self.px4_mode:
                        received, flags = self.px4_mode
                        exclusive = (flags['offboard'] and flags['rates']
                                     and not any(flags[key] for key in (
                                         'velocity', 'position', 'attitude', 'altitude',
                                         'climb_rate')))
                        self.mode.confirm(now, received, exclusive)
                elif self.mode.phase == 'PAPER_INTERCEPT_ACTIVE':
                    output = self.controller.step(self.image.state, pose.rotation, position,
                                                  velocity, stamp, dt, navigation_age=now-stamp)
                    self._emit_rates(output['rates'], output['thrust'], now)
            elif self.mode.phase == 'Y_HANDOVER':
                self._recovery(now, max(.001, min(dt, .05)))
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            self.mode.fail(str(error))
            if 'UNRECOVERABLE' in str(error) and not self.fatal_dispatched:
                from px4_msgs.msg import VehicleCommand
                self.node._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.)
                self.fatal_dispatched = True
        if self.mode.phase in ('SAFE_RECOVERY', 'CAPTURED') and not self.fatal_dispatched:
            self._recovery(now, max(.001, min(dt, .05)))
        if now-self.last_report >= .05:
            self.last_report = now
            self.publish_phase()
            self.node.search_state = ('PAPER_ACTIVE' if self.mode.phase
                                      == 'PAPER_INTERCEPT_ACTIVE' else self.mode.phase)
            self.node._publish_diagnostic(now, None, self.mode.phase, 0.)
            self.diagnostic_pub.publish(String(data=json.dumps(dict(
                phase=self.mode.phase, epoch=self.mode.epoch, reason=self.mode.reason,
                source_stamp=stamp_seconds(self.image.source_stamp) if self.image else 0.,
                output=getattr(self, 'last_emitted', None), actual_dt=dt))))

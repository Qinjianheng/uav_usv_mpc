#!/usr/bin/env python3
"""Bounded disarmed-only proof of the real PX4 body-rate DDS interface; never arm."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (ActuatorMotors, OffboardControlMode, TrajectorySetpoint,
                          VehicleAcceleration, VehicleAngularVelocity, VehicleCommand,
                          VehicleControlMode, VehicleRatesSetpoint, VehicleStatus,
                          VehicleThrustSetpoint)
from std_msgs.msg import Bool


class DisarmedProbe(Node):
    """Emit one interface at a time and require PX4's own exclusive mode flags."""

    def __init__(self, output):
        super().__init__('p76_disarmed_px4_probe')
        self.output = Path(output)
        self.raw = (self.output/'wire.jsonl').open('x')
        self.started = time.monotonic()
        self.counts, self.status, self.mode = Counter(), None, None
        self.finished, self.result, self.body_ack, self.velocity_ack = False, None, False, False
        self.body_started, self.velocity_started = None, None
        self.offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.rates = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', 10)
        self.velocity = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.commands = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.ready = self.create_publisher(Bool, '/simulation/impact/flight_ready', 10)
        self.subs = []
        for name, message_type in (
            ('vehicle_status', VehicleStatus), ('vehicle_control_mode', VehicleControlMode),
            ('vehicle_angular_velocity', VehicleAngularVelocity),
            ('vehicle_thrust_setpoint', VehicleThrustSetpoint),
            ('vehicle_rates_setpoint', VehicleRatesSetpoint), ('actuator_motors', ActuatorMotors),
            ('vehicle_acceleration', VehicleAcceleration),
        ):
            version = getattr(message_type, 'MESSAGE_VERSION', 0)
            topic = '/fmu/out/'+name+(f'_v{version}' if version else '')
            self.subs.append(self.create_subscription(message_type, topic,
                                                      lambda msg, key=name: self.receive(key, msg),
                                                      qos_profile_sensor_data))
        self.timer = self.create_timer(.005, self.tick)

    def receive(self, name, message):
        """Preserve native wire timestamps and sample values without restamping acquisition."""
        self.counts[name] += 1
        row = dict(topic=name, receipt_monotonic=time.monotonic())
        for key in message.get_fields_and_field_types():
            value = getattr(message, key)
            row[key] = value.tolist() if hasattr(value, 'tolist') else value
        self.raw.write(json.dumps(row, allow_nan=True)+'\n')
        if name == 'vehicle_status':
            self.status = message
            if message.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
                self.finish(False, 'NOT_DISARMED_ABORT')
        elif name == 'vehicle_control_mode':
            self.mode = message
            if message.flag_armed:
                self.finish(False, 'ARMED_MODE_ABORT')

    def finish(self, success, reason):
        """A successful interface probe is explicitly not physical actuator qualification."""
        if self.finished:
            return
        self.finished = True
        self.raw.flush()
        self.result = dict(success=success, reason=reason,
                           scope='DISARMED_DDS_MODE_INTERFACE_ONLY',
                           flight_qualified=False, thrust_curve_qualified=False,
                           bodyrate_ack=self.body_ack, velocity_ack=self.velocity_ack,
                           messages=dict(self.counts), elapsed=time.monotonic()-self.started)
        (self.output/'result.json').write_text(json.dumps(self.result, indent=2)+'\n')

    def tick(self):
        """No arm or takeoff command exists in this diagnostic program."""
        if self.finished:
            return
        now = time.monotonic()
        if now-self.started > 35:
            self.finish(False, 'DISARMED_PROBE_TIMEOUT')
            return
        if self.status is None:
            return
        if self.status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            self.finish(False, 'NOT_DISARMED_ABORT')
            return
        stamp = self.get_clock().now().nanoseconds//1000
        if self.body_started is None:
            self.body_started = now
        returning = self.velocity_started is not None
        self.offboard.publish(OffboardControlMode(timestamp=stamp, body_rate=not returning,
                                                  velocity=returning))
        if returning:
            self.velocity.publish(TrajectorySetpoint(timestamp=stamp,
                                                     position=[math.nan]*3, velocity=[0.]*3,
                                                     acceleration=[math.nan]*3, yaw=math.nan))
        else:
            self.rates.publish(VehicleRatesSetpoint(timestamp=stamp, roll=0., pitch=0., yaw=0.,
                                                    thrust_body=[0., 0., -.4]))
        if now-self.body_started > 1.2 and self.status.nav_state != 14:
            # This is exclusively a mode command, never an arming command.
            if not hasattr(self, 'last_request') or now-self.last_request > 1.:
                self.commands.publish(VehicleCommand(
                    timestamp=stamp, command=176, param1=1., param2=6., target_system=1,
                    target_component=1, source_system=1, source_component=1, from_external=True))
                self.last_request = now
        mode = self.mode
        if mode is None or not mode.flag_control_offboard_enabled:
            return
        higher = any(getattr(mode, 'flag_control_'+key+'_enabled') for key in (
            'position', 'velocity', 'attitude', 'altitude', 'climb_rate'))
        if not returning and mode.flag_control_rates_enabled and not higher:
            self.body_ack = True
            required = ('vehicle_angular_velocity', 'vehicle_thrust_setpoint',
                        'vehicle_rates_setpoint', 'actuator_motors', 'vehicle_acceleration')
            if all(self.counts[name] >= 20 for name in required):
                self.velocity_started = now
        elif returning and mode.flag_control_velocity_enabled:
            self.velocity_ack = True
            self.ready.publish(Bool(data=True))
            if now-self.velocity_started > .5:
                self.finish(True, 'DISARMED_BODYRATE_AND_VELOCITY_ACK_VERIFIED')


def main():
    """Run only inside the task-owned isolated SITL environment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    from uav_control.guidance.follow_sitl_gate import require_isolated_sitl
    require_isolated_sitl()
    rclpy.init(args=[])
    node = DisarmedProbe(options.output)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if not node.finished:
            node.finish(False, 'PROBE_INTERRUPTED')
        node.raw.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Read-only P5 health at 20Hz; original sample epochs are retained for stop decisions."""
import argparse
import json
from pathlib import Path
import time


def attitude_record(message, receipt):
    """ROS fixed arrays contain NumPy float32 scalars; JSON requires native floats."""
    return dict(receipt=receipt, q=[float(value) for value in message.q])


def run(root):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleAttitude, VehicleStatus
    from uav_usv_interfaces.msg import ControllerDiagnostic, TargetPrediction, UavState
    rclpy.init()
    node = Node('p5_readonly_health')
    latest = {}

    def stamp(value):
        return value.sec+value.nanosec/1e9

    def xyz(value):
        return [float(getattr(value, axis)) for axis in 'xyz']

    def nav(m):
        latest['navigation'] = dict(receipt=time.time(), stamp=stamp(m.stamp),
                                    valid=m.valid, p=xyz(m.position), v=xyz(m.velocity))

    def prediction(m):
        latest['prediction'] = dict(receipt=time.time(), stamp=stamp(m.source_stamp),
                                    observation=stamp(m.observation_stamp), valid=m.valid)

    def control(m):
        latest['control'] = dict(receipt=time.time(), status=m.status,
                                 locked=m.target_locked, compute=m.callback_compute_time)

    subscriptions = [
        node.create_subscription(UavState, '/navigation/uav_state', nav, qos_profile_sensor_data),
        node.create_subscription(TargetPrediction, '/planning/target_prediction', prediction,
                                 qos_profile_sensor_data),
        node.create_subscription(ControllerDiagnostic, '/control/diagnostic', control, 10),
        node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', lambda m:
                                 latest.update(vehicle=dict(receipt=time.time(),
                                                            nav_state=m.nav_state,
                                                            armed=m.arming_state)),
                                 qos_profile_sensor_data),
        node.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', lambda m:
                                 latest.update(attitude=attitude_record(m, time.time())),
                                 qos_profile_sensor_data),
    ]

    def write():
        snapshot = dict(latest, written=time.time(),
                        ros_now=node.get_clock().now().nanoseconds/1e9)
        temporary = root/'p5_health.tmp'
        temporary.write_text(json.dumps(snapshot, allow_nan=False))
        temporary.replace(root/'p5_health.json')

    timer = node.create_timer(.05, write)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    run(parser.parse_args().root)

#!/usr/bin/env python3
"""Read-only P5 health at 20Hz; original sample epochs are retained for stop decisions."""
import argparse
import bisect
from collections import deque
import json
import math
from pathlib import Path
import time


def attitude_record(message, receipt):
    """ROS fixed arrays contain NumPy float32 scalars; JSON requires native floats."""
    return dict(receipt=receipt, q=[float(value) for value in message.q])


def follow_settling(navigation, tracking, now):
    """Read-only start gate at a common epoch; never extrapolate native navigation."""
    def valid(row):
        return row.get('valid') and all(len(row.get(k, ())) == 3 for k in ('p', 'v')) and all(
            math.isfinite(v) for k in ('p', 'v') for v in row[k])

    nav = sorted((row for row in navigation if valid(row)), key=lambda row: row['stamp'])
    stamps = [row['stamp'] for row in nav]
    for target in sorted(tracking, key=lambda row: row['stamp'], reverse=True):
        if not valid(target) or not all(
                0 <= now-target.get(k, 0.) <= .125 for k in ('stamp', 'source')):
            continue
        i = bisect.bisect_left(stamps, target['stamp'])
        if not 0 < i < len(nav):
            continue
        a, b = nav[i-1], nav[i]
        dt = b['stamp']-a['stamp']
        if not 0 < dt <= .05:
            continue
        f = (target['stamp']-a['stamp'])/dt
        p = [x+f*(y-x) for x, y in zip(a['p'], b['p'])]
        v = [x+f*(y-x) for x, y in zip(a['v'], b['v'])]
        speed = math.hypot(*target['v'][:2])
        direction = ([x/speed for x in target['v'][:2]] if speed > 1e-6 else [1., 0.])
        error = math.hypot(*(target['p'][i]-5.*direction[i]-p[i] for i in range(2)))
        relative = math.hypot(*(target['v'][i]-v[i] for i in range(2)))
        height = abs(p[2]+5.)
        return dict(settled=error <= 1. and relative <= .5 and height <= .1,
                    epoch=target['stamp'], source_stamp=target['source'],
                    navigation_bracket=[a['stamp'], b['stamp']], horizontal_error=error,
                    relative_speed=relative, height_error=height,
                    method='native bracket interpolation; no extrapolation')
    return dict(settled=False, reason='NO_FRESH_COMMON_EPOCH')


def run(root):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleAttitude, VehicleStatus
    from uav_usv_interfaces.msg import (
        ControllerDiagnostic, TargetPrediction, TargetState, UavState)
    rclpy.init()
    node = Node('p5_readonly_health')
    latest = {}
    navigation_history, tracking_history = deque(maxlen=128), deque(maxlen=64)

    def stamp(value):
        return value.sec+value.nanosec/1e9

    def xyz(value):
        return [float(getattr(value, axis)) for axis in 'xyz']

    def nav(m):
        latest['navigation'] = dict(receipt=time.time(), stamp=stamp(m.stamp),
                                    valid=m.valid, p=xyz(m.position), v=xyz(m.velocity))
        navigation_history.append(latest['navigation'])

    def tracking(m):
        tracking_history.append(dict(stamp=stamp(m.stamp), source=stamp(m.source_stamp),
                                     valid=m.valid, p=xyz(m.position), v=xyz(m.velocity)))

    def prediction(m):
        latest['prediction'] = dict(receipt=time.time(), stamp=stamp(m.source_stamp),
                                    observation=stamp(m.observation_stamp), valid=m.valid)

    def control(m):
        latest['control'] = dict(receipt=time.time(), status=m.status,
                                 locked=m.target_locked, compute=m.callback_compute_time)

    subscriptions = [
        node.create_subscription(UavState, '/navigation/uav_state', nav, qos_profile_sensor_data),
        node.create_subscription(TargetState, '/tracking/target_state', tracking,
                                 qos_profile_sensor_data),
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
        ros_now = node.get_clock().now().nanoseconds/1e9
        settling = follow_settling(navigation_history, tracking_history, ros_now)
        snapshot = dict(latest, written=time.time(), ros_now=ros_now, follow_settling=settling)
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

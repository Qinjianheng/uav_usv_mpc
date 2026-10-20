#!/usr/bin/env python3
"""Observe actual Gazebo/ROS YOLO acquisition ages without a flight controller."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uav_usv_interfaces.msg import TargetImageDetection


def observe(output, duration):
    """Record every received packet, unique publisher graph and actual end-to-end age."""
    output = Path(output)
    output.mkdir(exist_ok=False)
    rclpy.init()
    node = Node('p76_perception_probe')
    rows, diagnostics, graph = [], [], []

    def receive(message):
        now = node.get_clock().now().nanoseconds*1e-9
        source = message.stamp.sec+message.stamp.nanosec*1e-9
        completed = message.detection_stamp.sec+message.detection_stamp.nanosec*1e-9
        rows.append(dict(receipt=now, source_stamp=source, completed=completed,
                         age=now-source if source > 0 else None, valid=message.valid,
                         reason=message.rejection_reason, frame=message.frame_sequence,
                         center=list(message.center_uv), confidence=message.confidence,
                         model_sha256=message.model_sha256))
    subscription = node.create_subscription(TargetImageDetection, '/perception/target_detection',
                                            receive, 100)
    status_sub = node.create_subscription(String, '/diagnostics/p76_yolo',
                                          lambda m: diagnostics.append(json.loads(m.data)), 100)
    began = time.monotonic()
    try:
        while time.monotonic()-began < duration:
            rclpy.spin_once(node, timeout_sec=.1)
            if not graph or time.monotonic()-graph[-1]['monotonic'] > 1.:
                publishers = node.get_publishers_info_by_topic('/perception/target_detection')
                graph.append(dict(monotonic=time.monotonic(), publishers=[
                    dict(name=p.node_name, namespace=p.node_namespace) for p in publishers],
                    nodes=node.get_node_names(), target_state_publishers=[
                        p.node_name for p in node.get_publishers_info_by_topic(
                            '/tracking/target_state')], px4_command_publishers={
                                topic: [
                                    p.node_name for p in node.get_publishers_info_by_topic(topic)]
                                for topic in ('/fmu/in/vehicle_rates_setpoint',
                                              '/fmu/in/trajectory_setpoint')}))
    except KeyboardInterrupt:
        pass
    finally:
        def quantiles(values):
            return dict(median=float(np.median(values)), p95=float(np.quantile(values, .95)),
                        maximum=float(max(values))) if values else None
        ages = [r['age'] for r in rows if r['age'] is not None]
        latencies = [r['inference_seconds'] for r in diagnostics if r['inference_seconds'] > 0]
        summary = dict(kind='Actual live Gazebo RGB -> ROS bridge -> ONNX -> ROS subscriber',
                       qualified=False, flight_qualification=False, packets=len(rows),
                       valid=sum(r['valid'] for r in rows),
                       acquisition_age_seconds=quantiles(ages),
                       inference_seconds=quantiles(latencies),
                       maximum_unique_publishers=max(
                           (len(g['publishers']) for g in graph), default=0),
                       observed_packets_per_second=((len(rows)-1)/(
                           rows[-1]['receipt']-rows[0]['receipt']) if len(rows) > 1 else 0.),
                       valid_packet_age_seconds=quantiles([
                           r['age'] for r in rows if r['valid'] and r['age'] is not None]),
                       valid_but_stale_at_receipt=sum(
                           r['valid'] and r['age'] is not None and r['age'] > .125 for r in rows),
                       reasons={reason: sum(r['reason'] == reason for r in rows)
                                for reason in sorted(set(r['reason'] for r in rows))})
        for name, values in [('packets', rows), ('diagnostics', diagnostics), ('graph', graph)]:
            (output/(name+'.jsonl')).write_text(''.join(json.dumps(v)+'\n' for v in values))
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        node.destroy_subscription(subscription)
        node.destroy_subscription(status_sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--duration', type=float, default=60.)
    options = parser.parse_args()
    observe(options.output, min(90., options.duration))

#!/usr/bin/env python3
"""Synthetic DDS test: actual tracker assignments -> two membership filters, no hardware."""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import time

from amr_interfaces.msg import (DetectedObject, DetectedObjectArray, ObservationProvenance,
                                TrackedObservationArray)
from dynamic_filter.dynamic_filter_node import DynamicFilterNode
import numpy as np
from object_tracker.object_tracker_node import ObjectTrackerNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(domain_id=194)
    executor = SingleThreadedExecutor()
    tracker, driver = ObjectTrackerNode(), Node('membership_test_driver')
    nodes = [tracker, driver]
    outputs = {'front': [], 'rear': []}
    originals = {'front': [], 'rear': []}
    scans = {}
    linked = []
    driver.create_subscription(TrackedObservationArray, '/tracked_observations',
                               linked.append, 10)
    detections = driver.create_publisher(DetectedObjectArray, '/detections_tracking', 10)
    for side in outputs:
        node = DynamicFilterNode(
            cli_args=['--ros-args', '-r', f'__node:=membership_{side}'],
            use_global_arguments=False, parameter_overrides=[
                Parameter('mask_method', value='tracked_cluster'),
                Parameter('observation_sensor_id', value=f'/scan_{side}'),
                Parameter('scan_input_topic', value=f'/audit/input_{side}'),
                Parameter('scan_output_topic', value=f'/audit/output_{side}')])
        nodes.append(node)
        scans[side] = driver.create_publisher(LaserScan, f'/audit/input_{side}',
                                              qos_profile_sensor_data)
        driver.create_subscription(LaserScan, f'/audit/output_{side}', outputs[side].append, 10)
    for node in nodes:
        executor.add_node(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    try:
        until = time.monotonic() + 10.
        while (detections.get_subscription_count() < 1
               or any(pub.get_subscription_count() < 1 for pub in scans.values())
               or driver.count_subscribers('/tracked_observations') < 3):
            assert time.monotonic() < until, 'DDS discovery timeout'
            spin(.05)
        spin(.2)
        for sequence in range(5):
            stamp = driver.get_clock().now().to_msg()
            batch = DetectedObjectArray()
            batch.header.stamp = stamp
            batch.header.frame_id = 'odom'
            obj = DetectedObject()
            obj.header = deepcopy(batch.header)
            obj.source, obj.class_name, obj.class_id, obj.confidence = 2, 'person', 0, .99
            obj.position.x = 100.  # Far from scan returns: this cannot be radius masking.
            obj.position_covariance = [.01, 0., 0., .01]
            for side in outputs:
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, f'laser_{side}'
                scan.angle_min, scan.angle_max, scan.angle_increment = -.1, .1, .1
                scan.range_min, scan.range_max = .1, 10.
                scan.ranges, scan.intensities = [1., 2., 3.], [4., 5., 6.]
                obs = ObservationProvenance()
                obs.header = deepcopy(scan.header)
                obs.sensor_id = f'/scan_{side}'
                obs.observation_id = f'{side}-{sequence}'
                obs.beam_indices = [0, 2] if side == 'front' else [1]
                obj.observations.append(obs)
                originals[side].append(deepcopy(scan))
                scans[side].publish(scan)
            batch.objects = [obj]
            batch.observations = list(obj.observations)
            spin(.02)
            detections.publish(batch)
            spin(.15)
        spin(.6)
        assert len(linked) == 5, len(linked)
        for side in outputs:
            assert len(outputs[side]) == 5
            for i, (actual, before) in enumerate(zip(outputs[side], originals[side])):
                expected = ([True, False, True] if side == 'front' else [False, True, False])
                if i < 2:  # Tracker requires three observations to confirm a person.
                    expected = [False]*3
                assert np.isnan(actual.ranges).tolist() == expected, (side, i, actual.ranges)
                restored = deepcopy(actual)
                restored.ranges = before.ranges
                assert serialize_message(restored) == serialize_message(before)
        result = {'passed': True, 'domain_id': 194, 'tracker_updates': len(linked),
                  'link_counts': [len(msg.links) for msg in linked],
                  'front_scans': len(outputs['front']), 'rear_scans': len(outputs['rear']),
                  'scope': 'Synthetic fused observations via DDS -> real tracker -> exact '
                           'native beam links -> front/rear filters. No TF/radius matching. '
                           'Does not test camera detection or real sensor latency.'}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

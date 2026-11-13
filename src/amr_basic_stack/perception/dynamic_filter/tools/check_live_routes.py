#!/usr/bin/env python3
"""Launch real Cartographer OFF/ON in an isolated ROS domain and inspect endpoints."""

from copy import deepcopy
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from amr_interfaces.msg import ImmModeState, ImmTrackedObject, ImmTrackedObjectArray
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


def check_transport(node):
    """Send synthetic tracks/scans through both real filter processes and DDS."""
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    tracks = ImmTrackedObjectArray()
    tracks.header.frame_id = 'odom'
    tracks.header.stamp.sec = 10
    received = {}
    subscriptions, publishers, scans = [], [], []
    for side, x in (('front', 1.), ('rear', -1.)):
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = f'test_{side}_laser'
        transform.transform.rotation.w = 1. if side == 'front' else 0.
        transform.transform.rotation.z = 0. if side == 'front' else 1.
        transforms.append(transform)
        obj = ImmTrackedObject()
        obj.lifecycle_state = obj.LIFECYCLE_CONFIRMED
        obj.class_name = 'person'
        mode = ImmModeState()
        mode.probability = 1.
        mode.state = [x, 0., 0., 0., 0.]
        obj.modes = [mode]
        tracks.objects.append(obj)
        scan = LaserScan()
        scan.header.frame_id = transform.child_frame_id
        scan.header.stamp.sec = 10
        scan.angle_min, scan.angle_max, scan.angle_increment = -.1, .1, .1
        scan.range_min, scan.range_max = .1, 10.
        scan.time_increment, scan.scan_time = .01, .1
        scan.ranges, scan.intensities = [1., 2., 1.], [10., 20., 30.]
        scans.append(scan)
        publishers.append(node.create_publisher(LaserScan, f'/scan_{side}_carto', 10))
        subscriptions.append(node.create_subscription(
            LaserScan, f'/scan_{side}_static',
            lambda msg, key=side: received.update({key: msg}), qos_profile_sensor_data))
    track_pub = node.create_publisher(ImmTrackedObjectArray, '/tracked_object_modes', 10)
    broadcaster.sendTransform(transforms)
    deadline = time.monotonic() + 10.
    while time.monotonic() < deadline:
        track_pub.publish(tracks)
        for publisher, scan in zip(publishers, scans):
            publisher.publish(scan)
        rclpy.spin_once(node, timeout_sec=.1)
        if all(side in received and math.isnan(received[side].ranges[0])
               for side in ('front', 'rear')):
            break
    for side, original in zip(('front', 'rear'), scans):
        assert side in received, f'No {side} output'
        result = received[side]
        assert math.isnan(result.ranges[0]) and math.isnan(result.ranges[2])
        assert result.ranges[1] == 2.
        restored = deepcopy(result)
        restored.ranges = original.ranges
        assert serialize_message(restored) == serialize_message(original)
    for subscription in subscriptions:
        node.destroy_subscription(subscription)
    for publisher in publishers + [track_pub]:
        node.destroy_publisher(publisher)
    print('PASS: dual filter DDS transport, rotated rear sensor, NaN mask and metadata',
          flush=True)


def main():
    """Run after sourcing the overlay; never launch robot hardware or controllers."""
    os.environ['ROS_DOMAIN_ID'] = '198'
    root = Path(tempfile.mkdtemp(prefix='dynamic-filter-routes-'))
    os.environ['ROS_LOG_DIR'] = str(root / 'ros-logs')
    rclpy.init(domain_id=198)
    node = rclpy.create_node('dynamic_filter_route_check')
    try:
        for enabled in ('false', 'true'):
            with (root / f'{enabled}.log').open('w') as log:
                process = subprocess.Popen([
                    'ros2', 'launch', 'amr_perception_bringup', 'cartographer.launch.py',
                    'mode:=mapping', f'use_dynamic_filter:={enabled}', 'use_sim_time:=true',
                    'odom_topic:=/odometry/filtered',
                    f'maps_dir:={root / "maps"}',
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    deadline = time.monotonic() + 30.
                    last = {}
                    while time.monotonic() < deadline:
                        if process.poll() is not None:
                            raise RuntimeError(f'Launch exited early; see {log.name}')
                        rclpy.spin_once(node, timeout_sec=.1)
                        last = {
                            topic: [endpoint.node_name for endpoint in
                                    node.get_subscriptions_info_by_topic(topic)]
                            for topic in ('/scan_front_carto', '/scan_rear_carto',
                                          '/scan_front_static', '/scan_rear_static')}
                        good = True
                        for side in ('front', 'rear'):
                            source, target = f'/scan_{side}_carto', f'/scan_{side}_static'
                            if enabled == 'true':
                                good &= f'dynamic_filter_{side}' in last[source]
                                good &= 'cartographer_node' in last[target]
                                good &= 'cartographer_node' not in last[source]
                                publishers = node.get_publishers_info_by_topic(target)
                                good &= [p.node_name for p in publishers] == [
                                    f'dynamic_filter_{side}']
                            else:
                                good &= 'cartographer_node' in last[source]
                                good &= not last[target]
                                good &= not any(name.startswith('dynamic_filter_')
                                                and name != node.get_name()
                                                for name in node.get_node_names())
                        if good:
                            print(f'PASS use_dynamic_filter={enabled}: {last}', flush=True)
                            if enabled == 'true':
                                check_transport(node)
                            break
                    else:
                        raise RuntimeError(f'Graph mismatch: {last}; see {log.name}')
                finally:
                    if process.poll() is None:
                        # Launch forwards SIGINT to its children; avoid delivering it twice.
                        process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15.)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5.)
                # Wait for DDS discovery to remove the processes just stopped.
                deadline = time.monotonic() + 10.
                while time.monotonic() < deadline and 'cartographer_node' in node.get_node_names():
                    rclpy.spin_once(node, timeout_sec=.1)
        print(f'Live graph logs: {root}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Run the stack's Cartographer mapping profile and capture genuine map updates.

Use an isolated ROS domain; this runner never launches base or sensor drivers.
Input is a sensor-only bag made by prepare_slam_video_bag.py.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def static_transform(transforms, source, target):
    """Return source<-target from the recorded URDF transforms."""
    graph = {}
    for item in transforms:
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(item['rotation']).as_matrix()
        transform[:3, 3] = item['translation']
        graph.setdefault(item['parent'], []).append((item['child'], transform))
        graph.setdefault(item['child'], []).append((item['parent'], np.linalg.inv(transform)))
    queue = [(source, np.eye(4))]
    visited = set()
    while queue:
        frame, accumulated = queue.pop(0)
        if frame == target:
            return accumulated
        if frame in visited:
            continue
        visited.add(frame)
        for child, transform in graph.get(frame, []):
            if child not in visited:
                queue.append((child, accumulated @ transform))
    raise RuntimeError(f'No static TF from {source} to {target}')


class Capture(Node):
    def __init__(self, output, tracking_to_base):
        super().__init__('slam_video_capture')
        self.output = output
        self.tracking_to_base = tracking_to_base
        self.map_count = 0
        self.pose_count = 0
        self.latest_stamp = 0.0
        self.last_pose_stamp = -math.inf
        self.map_log = (output / 'maps' / 'index.jsonl').open('w')
        self.pose_file = (output / 'poses.csv').open('w', newline='')
        self.pose_writer = csv.writer(self.pose_file)
        self.pose_writer.writerow(['t', 'x', 'y', 'yaw'])
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(PoseStamped, '/tracked_pose', self.on_pose, 1000)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)

    def on_clock(self, message):
        self.latest_stamp = stamp_seconds(message.clock)

    def on_pose(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if stamp <= self.last_pose_stamp:
            return
        pose = message.pose
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w]).as_matrix()
        matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        base = matrix @ self.tracking_to_base
        yaw = math.atan2(base[1, 0], base[0, 0])
        self.pose_writer.writerow([stamp, base[0, 3], base[1, 3], yaw])
        self.pose_count += 1
        self.last_pose_stamp = stamp

    def on_map(self, message):
        if not message.info.width or not message.info.height:
            return
        stamp = stamp_seconds(message.header.stamp)
        if stamp < 1e9:
            stamp = self.latest_stamp
        if stamp < 1e9:
            return
        origin = message.info.origin
        rotation = Rotation.from_quat([origin.orientation.x, origin.orientation.y,
                                      origin.orientation.z, origin.orientation.w]).as_matrix()
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        filename = f'map_{self.map_count:06d}.npz'
        np.savez_compressed(
            self.output / 'maps' / filename,
            data=np.asarray(message.data, dtype=np.int8).reshape(
                message.info.height, message.info.width),
            resolution=message.info.resolution,
            origin=np.asarray([origin.position.x, origin.position.y, yaw]))
        self.map_log.write(json.dumps({'t': stamp, 'file': filename}) + '\n')
        self.map_log.flush()
        self.map_count += 1

    def close(self):
        self.map_log.close()
        self.pose_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config-dir', type=Path, required=True)
    parser.add_argument('--config', default='mapping_hq_2d.lua')
    parser.add_argument('--timeout', type=float, default=600)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'maps').mkdir(exist_ok=True)
    metadata_path = args.output / 'metadata.json'
    metadata = json.loads(metadata_path.read_text())
    tracking = metadata.get('tracking_frame', 'camera_imu_optical_frame')
    tracking_to_base = static_transform(metadata['static_transforms'], tracking,
                                       'base_footprint')
    state_path = args.output / 'slam.pbstream'
    if state_path.exists() or (args.output / 'poses.csv').exists():
        raise RuntimeError('Capture output already exists; choose a new output directory')
    rclpy.init()
    node = Capture(args.output, tracking_to_base)
    carto_log = (args.output / 'cartographer.log').open('w')
    grid_log = (args.output / 'occupancy_grid.log').open('w')
    processes = []
    started = time.monotonic()
    completed_at = None
    last_report = 0.0
    try:
        grid = subprocess.Popen([
            '/opt/ros/jazzy/lib/cartographer_ros/cartographer_occupancy_grid_node',
            '-resolution', '0.05', '-publish_period_sec', '0.25',
            '--ros-args', '-p', 'use_sim_time:=true'], stdout=grid_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(grid)
        # Give the map service client and collector time to discover the offline node.
        time.sleep(1.0)
        offline = subprocess.Popen([
            '/opt/ros/jazzy/lib/cartographer_ros/cartographer_offline_node',
            '-configuration_directory', str(args.config_dir),
            '-configuration_basenames', args.config,
            '-bag_filenames', str(args.bag),
            '-save_state_filename', str(state_path),
            '-keep_running=true', '--ros-args', '-p', 'use_sim_time:=true'], stdout=carto_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(offline)
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_report >= 10.0:
                print(json.dumps({'wall_sec': round(now - started, 1),
                                  'bag_elapsed_sec': round(node.latest_stamp -
                                      metadata['bag_start_sec'], 1) if node.latest_stamp else None,
                                  'maps': node.map_count, 'poses': node.pose_count}), flush=True)
                last_report = now
            if grid.poll() is not None:
                raise RuntimeError(f'Occupancy node exited {grid.returncode}; see occupancy_grid.log')
            if offline.poll() is not None:
                if offline.returncode:
                    raise RuntimeError(f'Cartographer exited {offline.returncode}; see cartographer.log')
                if completed_at is None:
                    completed_at = now
            if state_path.exists() and state_path.stat().st_size > 1024:
                if completed_at is None:
                    completed_at = now
            if completed_at is not None and now - completed_at >= 6.0:
                break
        else:
            raise TimeoutError('SLAM capture exceeded timeout')
        if node.map_count < 5 or node.pose_count < 50 or not state_path.exists():
            raise RuntimeError(f'Insufficient capture: {node.map_count} maps, {node.pose_count} poses')
        metadata.update({'config': args.config, 'map_count': node.map_count,
                         'pose_count': node.pose_count, 'wall_sec': time.monotonic() - started,
                         'tracking_frame': tracking, 'robot_frame': 'base_footprint',
                         'processing': 'fresh offline Cartographer mapping; no saved map loaded',
                         'ros_domain_id': os.environ.get('ROS_DOMAIN_ID')})
        metadata_path.write_text(json.dumps(metadata, indent=2))
        print(json.dumps({'complete': True, 'maps': node.map_count,
                          'poses': node.pose_count, 'state': str(state_path)}), flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        carto_log.close()
        grid_log.close()


if __name__ == '__main__':
    main()

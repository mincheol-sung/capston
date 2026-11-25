#!/usr/bin/env python3
"""Apply the same robot-centric sector to LaserScan and timed PointCloud2 inputs."""

import copy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
import tf2_ros

from .scan_merger import yaw_from_quat
from .timed_cloud import records, select_records


def angle_in_sector(angles, lo, hi):
    """Return a mask for a normal or wrap-around angular sector."""
    if abs(lo + math.pi) < 1e-6 and abs(hi - math.pi) < 1e-6:
        return np.ones_like(angles, dtype=bool)
    if lo <= hi:
        return (angles >= lo) & (angles <= hi)
    return (angles >= lo) | (angles <= hi)


def endpoint_bearings(px, py, transform):
    """Calculate endpoint directions in the target frame."""
    tx, ty, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return np.arctan2(ty + s * px + c * py, tx + c * px - s * py)


def filter_cloud(cloud, transform, sector):
    """Test endpoints in the target frame and emit records in the lidar frame."""
    points = records(cloud)
    px, py = points['x'].astype(np.float64), points['y'].astype(np.float64)
    keep = np.isfinite(px) & np.isfinite(py)
    keep &= angle_in_sector(endpoint_bearings(px, py, transform), *sector)
    return select_records(cloud, points, keep)


def filter_ranges(ranges, angles, transform, sector):
    """Return ranges with endpoints outside ``sector`` replaced by infinity."""
    values = np.asarray(ranges, dtype=np.float64)
    yaw = transform[2]

    # 무한 거리 빔도 섹터를 판정할 수 있도록 우선 빔 방향만 회전시킨다.
    bearings = np.arctan2(np.sin(angles + yaw), np.cos(angles + yaw))
    finite = np.isfinite(values) & (values >= 0.0)
    if finite.any():
        px = values[finite] * np.cos(angles[finite])
        py = values[finite] * np.sin(angles[finite])
        bearings[finite] = endpoint_bearings(px, py, transform)

    keep = angle_in_sector(bearings, *sector)
    filtered = values.copy()
    filtered[~keep] = math.inf
    return filtered, keep


def filter_scan(msg, transform, sector):
    """Copy and sector-mask a scan without changing any timing metadata."""
    count = len(msg.ranges)
    angles = msg.angle_min + np.arange(count) * msg.angle_increment
    ranges, keep = filter_ranges(msg.ranges, angles, transform, sector)

    out = copy.deepcopy(msg)
    out.ranges = ranges.tolist()
    if len(out.intensities) == count:
        intensities = np.asarray(out.intensities, dtype=np.float64).copy()
        intensities[~keep] = 0.0
        out.intensities = intensities.tolist()
    return out


class ScanSectorFilter(Node):
    """Mask one lidar to its assigned robot-centric sector."""

    def __init__(self):
        super().__init__('scan_sector_filter')
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_scan').value
        lo_deg = float(self.declare_parameter('sector_min_deg', -180.0).value)
        hi_deg = float(self.declare_parameter('sector_max_deg', 180.0).value)
        self.sector = math.radians(lo_deg), math.radians(hi_deg)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_cache = {}
        self.warned_frames = set()

        self.sub = self.create_subscription(
            LaserScan, 'scan_in', self._on_scan, qos_profile_sensor_data)
        # RELIABLE publisher is compatible with Cartographer's BEST_EFFORT
        # SensorDataQoS subscriber and with ordinary diagnostic subscribers.
        self.pub = self.create_publisher(LaserScan, 'scan_out', 10)
        self.create_subscription(PointCloud2, 'points_in', self._on_cloud, qos_profile_sensor_data)
        self.points_pub = self.create_publisher(PointCloud2, 'points_out', 10)
        self.get_logger().info(
            f'LaserScan / timed PointCloud2 섹터 필터: {lo_deg:.1f}° ~ {hi_deg:.1f}° '
            f'(기준 {self.target_frame})')

    def _get_tf(self, frame_id):
        if frame_id in self.tf_cache:
            return self.tf_cache[frame_id]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            if frame_id not in self.warned_frames:
                self.warned_frames.add(frame_id)
                self.get_logger().warning(
                    f'{self.target_frame} <- {frame_id} TF 대기 중: {exc}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        result = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        self.tf_cache[frame_id] = result
        return result

    def _on_cloud(self, msg):
        transform = self._get_tf(msg.header.frame_id)
        if transform is not None:
            self.points_pub.publish(filter_cloud(msg, transform, self.sector))

    def _on_scan(self, msg):
        transform = self._get_tf(msg.header.frame_id)
        if transform is None:
            return

        self.pub.publish(filter_scan(msg, transform, self.sector))


def main(args=None):
    rclpy.init(args=args)
    node = ScanSectorFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

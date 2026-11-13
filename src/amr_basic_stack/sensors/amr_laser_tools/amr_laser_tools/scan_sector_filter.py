#!/usr/bin/env python3
"""LaserScan의 원래 시간 정보를 보존하면서 담당 방위각 밖의 빔을 가린다.

두 회전식 라이다를 하나의 LaserScan으로 래스터화하면 서로 다른 측정 시각을
단일 ``time_increment``로 표현할 수 없다. Cartographer에는 이 노드가 만든
앞/뒤 LaserScan 두 개를 직접 전달한다. 메시지의 frame, stamp, scan_time,
time_increment와 빔 순서는 바꾸지 않고 담당 섹터 밖 range만 ``inf``로 만든다.
메타데이터 보존은 드라이버 timing 정확성의 검증을 뜻하지 않는다.
"""

import copy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros

from .scan_merger import yaw_from_quat


def angle_in_sector(angles, lo, hi):
    """Return a mask for a normal or wrap-around angular sector."""
    if abs(lo + math.pi) < 1e-6 and abs(hi - math.pi) < 1e-6:
        return np.ones_like(angles, dtype=bool)
    if lo <= hi:
        return (angles >= lo) & (angles <= hi)
    return (angles >= lo) | (angles <= hi)


def filter_ranges(ranges, angles, transform, sector):
    """Return ranges with endpoints outside ``sector`` replaced by infinity."""
    values = np.asarray(ranges, dtype=np.float64)
    tx, ty, yaw = transform

    # 무한 거리 빔도 섹터를 판정할 수 있도록 우선 빔 방향만 회전시킨다.
    bearings = np.arctan2(np.sin(angles + yaw), np.cos(angles + yaw))
    finite = np.isfinite(values) & (values >= 0.0)
    if finite.any():
        px = values[finite] * np.cos(angles[finite])
        py = values[finite] * np.sin(angles[finite])
        c, s = math.cos(yaw), math.sin(yaw)
        bx = tx + c * px - s * py
        by = ty + s * px + c * py
        bearings[finite] = np.arctan2(by, bx)

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
        self.get_logger().info(
            f'LaserScan timing 보존 섹터 필터: {lo_deg:.1f}° ~ {hi_deg:.1f}° '
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

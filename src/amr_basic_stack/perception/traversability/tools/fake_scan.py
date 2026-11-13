#!/usr/bin/env python3
"""해제(clearing) 검증용 가짜 LaserScan 발행기.

로봇 앞 일정 거리에 장애물이 있다가 사라지는 상황을 만든다. 장애물이 있으면
그 거리에서 빔이 멈추고, 없으면 max_range 까지 간다.

    python3 fake_scan.py --obstacle 2.0        # 앞 2 m 에 벽
    python3 fake_scan.py --obstacle 0          # 장애물 없음 (전부 관통)

TF (map->odom->base_footprint->laser_front) 도 같이 낸다.
"""
import argparse
import math
import sys
import threading

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from tf2_ros import TransformBroadcaster

BASE_LINK_Z = 0.04
LASER_XYZ = (0.086291, -0.020494, 0.177797)


class FakeScan(Node):
    """장애물 거리를 런타임에 바꿀 수 있는 스캔 발행기."""

    def __init__(self, rate, beams, max_range, obstacle):
        super().__init__('fake_scan')
        self.publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.beams = beams
        self.max_range = max_range
        self.obstacle = obstacle
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'fake_scan: {beams} beams, max {max_range} m, '
            f'obstacle at {obstacle if obstacle > 0 else "없음"}')

    def _send_tf(self, stamp):
        def make(parent, child, xyz):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])
            t.transform.rotation.w = 1.0
            return t
        self.tf.sendTransform([
            make('map', 'odom', (0, 0, 0)),
            make('odom', 'base_footprint', (0, 0, 0)),
            make('base_footprint', 'base_link', (0, 0, BASE_LINK_Z)),
            make('base_link', 'laser_front', LASER_XYZ),
        ])

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        self._send_tf(stamp)

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'laser_front'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / self.beams
        msg.range_min = 0.05
        msg.range_max = float(self.max_range)

        ranges = np.full(self.beams, self.max_range * 0.999, dtype=np.float64)
        if self.obstacle > 0.0:
            # 정면 ±15도 구간만 장애물에서 멈춘다
            angles = msg.angle_min + np.arange(self.beams) * msg.angle_increment
            front = np.abs(angles) < math.radians(15.0)
            ranges[front] = self.obstacle
        msg.ranges = ranges.tolist()
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--beams', type=int, default=720)
    parser.add_argument('--max-range', type=float, default=6.0)
    parser.add_argument('--obstacle', type=float, default=2.0,
                        help='정면 장애물 거리 [m]. 0 이면 장애물 없음')
    parser.add_argument('--remove-after', type=float, default=0.0,
                        help='N 초 뒤 장애물을 치운다. 0 이면 안 치운다')
    args = parser.parse_args()

    rclpy.init()
    node = FakeScan(args.rate, args.beams, args.max_range, args.obstacle)
    if args.remove_after > 0.0:
        def remove():
            node.obstacle = 0.0
            node.get_logger().info('장애물 제거됨')
        threading.Timer(args.remove_after, remove).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

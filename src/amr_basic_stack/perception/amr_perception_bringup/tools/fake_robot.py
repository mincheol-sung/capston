#!/usr/bin/env python3
"""AMCL 검증용 가짜 로봇 — 지도 위를 실제로 움직이며 스캔을 만든다.

지도는 /map 에서 받아, 로봇의 실제(ground truth) 위치에서 광선을 쏴
가장 가까운 점유 셀까지의 거리를 LaserScan 으로 낸다. 그래서 AMCL 이
수렴하면 그 위치가 ground truth 와 맞는지 확인할 수 있다.
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       qos_profile_sensor_data, ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BASE_LINK_Z = 0.04
LASER_XYZ = (0.086291, -0.020494, 0.177797)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FakeRobot(Node):
    def __init__(self, start, speed, beams, max_range):
        super().__init__('fake_robot')
        self.grid = None
        self.info = None
        self.x, self.y, self.yaw = start
        self.speed = speed
        self.beams = beams
        self.max_range = max_range
        self.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED)
        self.scan_pub = self.create_publisher(LaserScan, '/scan',
                                              qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 20)
        # 진짜 위치. AMCL 추정과 비교할 기준이다. 로봇이 움직이므로
        # 출발 위치와 비교하면 안 된다.
        self.truth_pub = self.create_publisher(PoseStamped, '/ground_truth', 10)
        self.tf = TransformBroadcaster(self)
        self.last = time.monotonic()
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'fake_robot start=({start[0]:.2f},{start[1]:.2f},'
                               f'{math.degrees(start[2]):.0f}deg) speed={speed}')

    def _on_map(self, msg):
        self.info = msg.info
        self.grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.get_logger().info(f'map {msg.info.width}x{msg.info.height} 수신')

    def _occupied(self, x, y):
        i = self.info
        cx = int((x - i.origin.position.x) / i.resolution)
        cy = int((y - i.origin.position.y) / i.resolution)
        if not (0 <= cx < i.width and 0 <= cy < i.height):
            return True
        return self.grid[cy, cx] >= 65

    def _raycast(self, angle):
        step = self.info.resolution * 0.5
        d = 0.0
        while d < self.max_range:
            d += step
            if self._occupied(self.x + d * math.cos(angle),
                              self.y + d * math.sin(angle)):
                return d
        return float('inf')

    def _tick(self):
        if self.grid is None:
            return
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        # 앞이 막히면 회전, 아니면 직진
        ahead = self._raycast(self.yaw)
        if ahead < 0.8:
            self.yaw += 1.2 * dt
        else:
            self.x += self.speed * dt * math.cos(self.yaw)
            self.y += self.speed * dt * math.sin(self.yaw)

        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quat(self.yaw)

        # odom -> base_footprint (EKF 자리). map->odom 은 없다(AMCL 이 추정).
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        b = TransformStamped()
        b.header.stamp = stamp
        b.header.frame_id = 'base_footprint'
        b.child_frame_id = 'base_link'
        b.transform.translation.z = BASE_LINK_Z
        b.transform.rotation.w = 1.0
        L = TransformStamped()
        L.header.stamp = stamp
        L.header.frame_id = 'base_link'
        L.child_frame_id = 'laser_front'
        L.transform.translation.x, L.transform.translation.y = LASER_XYZ[0], LASER_XYZ[1]
        L.transform.translation.z = LASER_XYZ[2]
        L.transform.rotation.w = 1.0
        self.tf.sendTransform([t, b, L])

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
        self.odom_pub.publish(o)

        gt = PoseStamped()
        gt.header.stamp = stamp
        gt.header.frame_id = 'map'
        gt.pose.position.x = self.x
        gt.pose.position.y = self.y
        gt.pose.orientation.z, gt.pose.orientation.w = q[2], q[3]
        self.truth_pub.publish(gt)

        s = LaserScan()
        s.header.stamp = stamp
        s.header.frame_id = 'laser_front'
        s.angle_min = -math.pi
        s.angle_max = math.pi
        s.angle_increment = 2 * math.pi / self.beams
        s.range_min = 0.15
        s.range_max = self.max_range
        s.ranges = [self._raycast(self.yaw + s.angle_min + k * s.angle_increment)
                    for k in range(self.beams)]
        self.scan_pub.publish(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--speed', type=float, default=0.35)
    ap.add_argument('--beams', type=int, default=180)
    ap.add_argument('--max-range', type=float, default=8.0)
    a = ap.parse_args()
    rclpy.init()
    node = FakeRobot((a.x, a.y, math.radians(a.yaw)), a.speed, a.beams, a.max_range)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/python3
"""bag 재생용: /imu/base_link (base_footprint 축) → /camera/imu (camera_imu_optical_frame 축).

Cartographer 는 /camera/imu 를 받는데(cartographer.launch.py imu_topic 기본), 토크 실험 bag 에는 회전된 /imu/base_link 만 있다.
imu_to_base_node 가 한 회전(base_footprint ← camera_imu_optical_frame)을 거꾸로 적용한다. 회전값은 실차 TF
(tf2_echo base_footprint camera_imu_optical_frame: RPY −90°, 0, −90°, 2026-11-14). URDF 가 바뀌면 고칠 것.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

QX, QY, QZ, QW = -0.5, 0.5, -0.5, 0.5
R = np.array([[1 - 2 * (QY * QY + QZ * QZ), 2 * (QX * QY - QZ * QW), 2 * (QX * QZ + QY * QW)],
              [2 * (QX * QY + QZ * QW), 1 - 2 * (QX * QX + QZ * QZ), 2 * (QY * QZ - QX * QW)],
              [2 * (QX * QZ - QY * QW), 2 * (QY * QZ + QX * QW), 1 - 2 * (QX * QX + QY * QY)]])   # v_base = R v_optical


class Relay(Node):
    def __init__(self):
        super().__init__("imu_relay_replay")
        self.pub = self.create_publisher(Imu, "/camera/imu", qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/base_link", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        a = R.T @ [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
        g = R.T @ [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
        o = Imu()
        o.header.stamp = m.header.stamp
        o.header.frame_id = "camera_imu_optical_frame"
        o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z = map(float, a)
        o.angular_velocity.x, o.angular_velocity.y, o.angular_velocity.z = map(float, g)
        o.orientation_covariance[0] = -1.0
        self.pub.publish(o)


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(Relay())

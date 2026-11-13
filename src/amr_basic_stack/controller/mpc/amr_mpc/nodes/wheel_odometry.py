#!/usr/bin/env python3
# wheel_odometry — /wheel_states 의 바퀴 각속도로 휠 오도메트리를 낸다 (선택 입력, 기본 꺼짐).
#   ros2 run amr_mpc wheel_odometry_node
# ---------------------------------------------------------------
#   /wheel_states   (amr_interfaces/WheelStates)  ak40_can_node (시뮬은 sim_plant_torque_node)
#      ↓  v = T ω (드라이버 역기구학의 역), SE(2) 적분  — amr_mpc/torque/wheel_odometry.py
#   /wheel_odom     (nav_msgs/Odometry)            frame odom → base_footprint. TF 는 내지 않는다
#
# 쓰는 곳: Cartographer 로컬 스캔 매칭의 초기 짐작 (localization_torque_odom_2d.lua, odom_topic:=/wheel_odom).
#   슬립 측정·MPC 에는 쓰지 않는다. 켜고 끄는 스위치는 config/torque_estimator.yaml 의 slam.use_wheel_odom.
#   주의: Cartographer 정합은 초기 짐작 쪽으로 끌리는 가중이 커서(translation_weight 10), 슬립 중에는 결과 포즈가
#   휠 오돔 쪽으로 일부 끌려 슬립이 작게 측정될 수 있다 (torque_mpc_todo.md).
# ---------------------------------------------------------------
import math
import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from amr_interfaces.msg import WheelStates
from amr_mpc.torque.model import TorquePlant
from amr_mpc.torque.wheel_odometry import WheelOdometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry_node")
        p = TorquePlant.from_yaml()
        self.declare_parameters("", [
            ("wheel_states_topic", "/wheel_states"),
            ("output_topic", "/wheel_odom"),
            ("odom_frame", "odom"),
            ("base_frame", "base_footprint"),
            ("cov_vxy", 0.01),            # [(m/s)²] Cartographer 는 공분산을 쓰지 않는다. 다른 소비자를 위한 값
            ("cov_wz", 0.04),
        ])
        g = lambda n: self.get_parameter(n).value
        self.odo = WheelOdometry(p.r, p.k)
        self.odom_frame, self.base_frame = str(g("odom_frame")), str(g("base_frame"))
        self.cov_v, self.cov_w = float(g("cov_vxy")), float(g("cov_wz"))
        self.pub = self.create_publisher(Odometry, str(g("output_topic")), 20)
        self.create_subscription(WheelStates, str(g("wheel_states_topic")), self.on_wheels, 20)
        self.get_logger().info(
            f"wheel_odometry: {g('wheel_states_topic')} → {g('output_topic')} (r {p.r} m, k {p.k:.4f} m). "
            "Cartographer 초기 짐작 전용 — 슬립 측정에는 쓰지 않는다")

    def on_wheels(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, th, vx, vy, wz = self.odo.update(t, list(m.velocity))
        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        o.twist.covariance[0] = o.twist.covariance[7] = self.cov_v
        o.twist.covariance[35] = self.cov_w
        self.pub.publish(o)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

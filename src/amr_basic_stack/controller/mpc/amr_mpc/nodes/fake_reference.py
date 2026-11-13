#!/usr/bin/env python3
# 가짜 플래너 — DRA-MPPI 와 같은 형식(PlanarReferenceArray, 10 Hz, dt 0.2 s × 20)으로 원궤적을 낸다.
# mpc_tracking_node 의 토픽 경로·보간·타임아웃을 플래너 없이 시험할 때 쓴다.
#   ros2 run amr_mpc fake_reference_node --ros-args -p radius:=1.0 -p speed:=0.3 -p duration:=25.0
# 원점은 첫 odom 포즈. duration 이 지나면 끝점 정지 기준을 계속 낸다 (stop_after 로 발행 중단 → 타임아웃 시험).
import math
import numpy as np
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from amr_interfaces.msg import PlanarReference, PlanarReferenceArray

from amr_mpc.core.reference import circle_reference


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FakeReferenceNode(Node):
    def __init__(self):
        super().__init__("fake_reference_node")
        self.declare_parameters("", [
            ("radius", 1.0), ("speed", 0.3), ("ramp_s", 1.0), ("duration", 0.0),
            ("rate_hz", 10.0), ("dt", 0.2), ("horizon_steps", 20),
            ("frame_id", "odom"), ("odom_topic", "/odometry/filtered"),
            ("reference_topic", "/dra_mppi/reference_trajectory"),
            ("stop_after", 0.0),                    # >0 이면 이 시각 이후 발행 중단 (MPC 타임아웃 시험)
        ])
        g = lambda n: self.get_parameter(n).value
        self.R, self.V, self.ramp, self.dur = float(g("radius")), float(g("speed")), float(g("ramp_s")), float(g("duration"))
        self.dt, self.T, self.frame = float(g("dt")), int(g("horizon_steps")), str(g("frame_id"))
        self.stop_after = float(g("stop_after"))
        self.origin = None; self.t0 = None
        self.pub = self.create_publisher(PlanarReferenceArray, str(g("reference_topic")), 5)
        self.create_subscription(Odometry, str(g("odom_topic")), self.cb_odom, 10)
        self.create_timer(1.0 / float(g("rate_hz")), self.on_tick)

    def cb_odom(self, m):
        if self.origin is None:
            p = m.pose.pose
            self.origin = np.array([p.position.x, p.position.y, yaw_of(p.orientation)])
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f"원점 {np.round(self.origin, 3)}  R={self.R} v={self.V} dt={self.dt}×{self.T}")

    def on_tick(self):
        if self.origin is None:
            return
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.t0
        if self.stop_after > 0 and t > self.stop_after:
            return
        msg = PlanarReferenceArray(); msg.header.stamp = now.to_msg(); msg.header.frame_id = self.frame; msg.dt = self.dt
        for i in range(self.T):
            Xr, ur = circle_reference(t + (i + 1) * self.dt, self.origin, self.R, self.V, self.ramp, self.dur)
            r = PlanarReference(); r.header = msg.header
            r.x, r.y, r.yaw = map(float, Xr); r.vx, r.vy, r.yaw_rate = map(float, ur)
            msg.references.append(r)
        self.pub.publish(msg)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C → KeyboardInterrupt 로 받아 조용히 정리
    node = FakeReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 정리 중 두 번째 Ctrl+C(launch 가 보냄) 무시
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

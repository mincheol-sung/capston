#!/usr/bin/env python3
# 시뮬 플랜트 — /cmd_vel 을 받아 식별한 구동계(FOPDT, model.py 의 TAU/K_DC/THETA)와 메카넘 기구학으로
# 적분하고 /odometry/filtered 를 50 Hz 로 낸다. 로봇 없이 mpc_tracking_node 를 통째로 돌려 보는 용도.
#   ros2 run amr_mpc sim_plant_node
# 구동계는 20 ms 격자: v[k+1] = a·v[k] + b·u[k−d],  a = e^{−0.02/τ}, b = K(1−a), d = θ/0.02 = 2 스텝.
import math
import numpy as np
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from amr_mpc.body.model import TAU, K_DC, THETA, f

DT = 0.02


class SimPlantNode(Node):
    def __init__(self):
        super().__init__("sim_plant_node")
        self.declare_parameters("", [("odom_topic", "/odometry/filtered"), ("frame_id", "odom"),
                                     ("x0", 0.0), ("y0", 0.0), ("yaw0", 0.0), ("cmd_timeout", 0.3)])
        g = lambda n: self.get_parameter(n).value
        self.X = np.array([g("x0"), g("y0"), g("yaw0")], float); self.v = np.zeros(3)
        self.a = np.array([math.exp(-DT / TAU[k]) for k in ("vx", "vy", "wz")])
        self.b = np.array([K_DC[k] for k in ("vx", "vy", "wz")]) * (1 - self.a)
        d = int(round(THETA / DT)); self.buf = [np.zeros(3) for _ in range(d + 1)]
        self.u = np.zeros(3); self.last_cmd = None; self.cmd_timeout = float(g("cmd_timeout"))
        self.frame = str(g("frame_id"))
        self.pub = self.create_publisher(Odometry, str(g("odom_topic")), 10)
        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd, 10)
        self.create_timer(DT, self.on_tick)
        self.get_logger().info(f"τ={TAU} K={K_DC} θ={THETA} → a={np.round(self.a,3)} d={d}")

    def cb_cmd(self, m):
        self.u = np.array([m.linear.x, m.linear.y, m.angular.z]); self.last_cmd = self.get_clock().now().nanoseconds * 1e-9

    def on_tick(self):
        now = self.get_clock().now()
        if self.last_cmd is None or now.nanoseconds * 1e-9 - self.last_cmd > self.cmd_timeout:
            self.u = np.zeros(3)
        self.buf.append(self.u.copy()); u_d = self.buf.pop(0)          # θ 만큼 지연된 명령
        self.v = self.a * self.v + self.b * u_d
        self.X = self.X + DT * f(self.X, self.v)                       # 기구학 (오일러)
        self.X[2] = math.atan2(math.sin(self.X[2]), math.cos(self.X[2]))
        o = Odometry(); o.header.stamp = now.to_msg(); o.header.frame_id = self.frame; o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.X[0]), float(self.X[1])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.X[2] / 2), math.cos(self.X[2] / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = map(float, self.v)
        self.pub.publish(o)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C → KeyboardInterrupt 로 받아 조용히 정리
    node = SimPlantNode()
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

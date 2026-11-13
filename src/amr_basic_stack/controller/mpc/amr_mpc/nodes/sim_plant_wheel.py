#!/usr/bin/env python3
# 구조 B 시뮬 플랜트 — /wheel_cmd 를 받아 바퀴별 FOPDT(plant_wheel.yaml) 로 바퀴속도를 만들고,
# 순기구학 v = T ω 로 포즈를 적분해 /odom 과 /measured_wheel_speeds 를 50 Hz 로 낸다.
#   ros2 run amr_mpc sim_plant_wheel_node
# 드라이버와 같은 격자(20 ms), d = θ/0.02 = 2 스텝.  slip 없음 (d = 0) — 실차와의 차이가 곧 d 다.
import math, signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from amr_mpc.wheel import model as wm
from amr_mpc.core.kinematics import f

DT = 0.02


class SimPlantWheelNode(Node):
    def __init__(self):
        super().__init__("sim_plant_wheel_node")
        self.declare_parameters("", [("odom_topic", "/odom"), ("frame_id", "odom"),
                                     ("x0", 0.0), ("y0", 0.0), ("yaw0", 0.0), ("cmd_timeout", 0.5)])
        g = lambda n: self.get_parameter(n).value
        self.X = np.array([g("x0"), g("y0"), g("yaw0")], float); self.w = np.zeros(4)
        self.a = np.exp(-DT / wm.TAU); self.b = wm.K_DC * (1 - self.a)
        d = int(round(wm.THETA / DT)); self.buf = [np.zeros(4) for _ in range(d + 1)]
        self.cmd = np.zeros(4); self.last_cmd = None; self.timeout = float(g("cmd_timeout")); self.frame = str(g("frame_id"))
        self.pub_odom = self.create_publisher(Odometry, str(g("odom_topic")), 10)
        self.pub_wheel = self.create_publisher(Float32MultiArray, "/measured_wheel_speeds", 10)
        self.pub_wheel_enc = self.create_publisher(Float32MultiArray, "/encoder_wheel_speeds", 10)   # C++ 노드용, 같은 값
        self.create_subscription(Float32MultiArray, "/wheel_cmd", self.cb_cmd, 10)
        self.create_timer(DT, self.on_tick)
        self.get_logger().info(f"바퀴 τ={np.round(wm.TAU*1000)} ms  K={np.round(wm.K_DC,3)}  θ={wm.THETA*1000:.0f} ms → a={np.round(self.a,3)} d={d}")

    def cb_cmd(self, m):
        if len(m.data) == 4:
            self.cmd = np.clip(np.array(m.data, float), -wm.W_MAX, wm.W_MAX); self.last_cmd = self.get_clock().now().nanoseconds * 1e-9

    def on_tick(self):
        now = self.get_clock().now()
        if self.last_cmd is None or now.nanoseconds * 1e-9 - self.last_cmd > self.timeout:
            self.cmd = np.zeros(4)
        self.buf.append(self.cmd.copy()); u_d = self.buf.pop(0)
        self.w = self.a * self.w + self.b * u_d
        v = wm.T_FWD @ self.w
        self.X = self.X + DT * f(self.X, v); self.X[2] = math.atan2(math.sin(self.X[2]), math.cos(self.X[2]))
        o = Odometry(); o.header.stamp = now.to_msg(); o.header.frame_id = self.frame; o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.X[0]), float(self.X[1])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.X[2] / 2), math.cos(self.X[2] / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = map(float, v)
        self.pub_odom.publish(o)
        self.pub_wheel.publish(Float32MultiArray(data=[float(x) for x in self.w]))
        self.pub_wheel_enc.publish(Float32MultiArray(data=[float(x) for x in self.w]))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SimPlantWheelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

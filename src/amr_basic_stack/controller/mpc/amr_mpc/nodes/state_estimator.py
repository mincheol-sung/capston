#!/usr/bin/env python3
# state_estimator — SLAM 포즈(약 10 Hz, 늦게 도착) + IMU(200 Hz) → 100 Hz 상태 추정값 (구조도 v3).
#   ros2 run amr_mpc state_estimator_node
# ---------------------------------------------------------------
#   /imu/base_link          (sensor_msgs/Imu)             몸체 좌표 가속도 a_x, a_y, 자이로 ω_z
#   /tracked_pose           (geometry_msgs/PoseStamped)   Cartographer tracking_frame(카메라 IMU 위치)의 map 포즈
#   /scan_matched_points2   (sensor_msgs/PointCloud2)     pose_source: after_scan_match 일 때만 — 새 스캔 매칭이 반영됐다는 신호
#      ↓  amr_mpc/torque/estimator.py (계산·근거는 그 파일 머리주석)
#   /state_estimator/odom   (nav_msgs/Odometry, 100 Hz)   로봇 중심(base_footprint): pose = map, twist = 몸체 → torque_mpc_node
#   /state_estimator/status (Float64MultiArray)           보정 때마다 [t, 잔차 x, y, ψ, b_ax, b_ay, b_g, 포즈 튐 수, 보정 수, 포즈 나이]
#
# 설정: config/torque_estimator.yaml (포즈 고르는 방식 pose_source, IMU 위치, 잡음 → 게인, 포즈 튐 문턱)
#
# 동작
#   · 기동하면 init_s 초 동안 정지 상태의 IMU 를 평균해 바이어스를 잡는다 (무장 전이라 로봇은 서 있다).
#     그동안은 출력이 없다 → torque_mpc_node 는 "상태 추정값 대기" 로 DAMPING.
#   · 출력은 IMU stamp 가 publish_period 만큼 지날 때마다 낸다. stamp = 그 IMU 표본의 시각.
#   · 포즈는 stamp 가 과거여도 된다 (스캔 시각). 그 시각의 추정값과 비교해 보정한다.
#   · 잔차가 문턱을 넘는 포즈(전역 최적화의 튐)는 버리고 센다. 연속으로 계속되면 그 포즈로 다시 초기화한다.
#   · SLAM 포즈가 max_dead_reckoning 초 넘게 반영되지 않으면 출력을 멈춘다 (IMU 적분만으로 흘러가는 것을 막는다, R3 리뷰).
# ---------------------------------------------------------------
import math
import signal

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64MultiArray

from amr_mpc.core import config
from amr_mpc.torque.estimator import PlanarStateEstimator


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__("state_estimator_node")
        cfg = dict(config.section("estimator", "torque_estimator.yaml"))
        self.declare_parameters("", [
            ("imu_topic", "/imu/base_link"),
            ("pose_topic", "/tracked_pose"),
            ("scan_matched_topic", "/scan_matched_points2"),
            ("output_topic", "/state_estimator/odom"),
            ("map_frame", "map"),
            ("base_frame", "base_footprint"),
            ("pose_source", str(cfg.get("pose_source", "stamped"))),     # stamped | after_scan_match
            ("pose_period", 0.1),                                        # [s] 게인 설계에 쓰는 스캔 매칭 주기
        ])
        g = lambda n: self.get_parameter(n).value
        self.pose_source = str(g("pose_source")).strip().lower()
        if self.pose_source not in ("stamped", "after_scan_match"):
            raise SystemExit(f"[state_estimator] pose_source={self.pose_source!r} — stamped | after_scan_match")
        self.est = PlanarStateEstimator(cfg, pose_period=float(g("pose_period")))
        self.period = float(cfg.get("publish_period", 0.01))
        self.map_frame, self.base_frame = str(g("map_frame")), str(g("base_frame"))
        self.last_pub = None
        self.last_pose_stamp = -1.0
        self.take_next_pose = False
        self.was_ready = False

        self.pub = self.create_publisher(Odometry, str(g("output_topic")), 20)
        self.pub_status = self.create_publisher(Float64MultiArray, "/state_estimator/status", 10)
        self.create_subscription(Imu, str(g("imu_topic")), self.on_imu, 50)
        self.create_subscription(PoseStamped, str(g("pose_topic")), self.on_pose, 50)
        if self.pose_source == "after_scan_match":
            self.create_subscription(PointCloud2, str(g("scan_matched_topic")), self.on_scan_matched, 10)
        e = self.est
        self.get_logger().info(
            f"state_estimator: {g('imu_topic')} + {g('pose_topic')} [{self.pose_source}] → {g('output_topic')} "
            f"({1/self.period:.0f} Hz)\n"
            f"  IMU·포즈 기준점 ({e.r[0]:.3f}, {e.r[1]:.3f}) m (base_footprint 기준)\n"
            f"  게인  K_p {e.K_xy[0]:.3f}  K_v {e.K_xy[1]:.2f} 1/s  K_b {e.K_xy[2]:.2f} 1/s²   "
            f"K_psi {e.K_yaw[0]:.3f}  K_bg {e.K_yaw[1]:.4f} 1/s\n"
            f"  정지 상태에서 {e.init_s:.1f} s 동안 바이어스를 잡는다 — 그동안 로봇을 움직이지 말 것")

    def on_scan_matched(self, _msg):
        self.take_next_pose = True

    def on_pose(self, m):
        t = stamp_s(m.header)
        if self.pose_source == "stamped":
            if t <= self.last_pose_stamp:              # 같은 스캔의 포즈가 다시 온 것
                return
        elif not (self.take_next_pose or not self.est.pose_ready):
            return
        self.take_next_pose = False
        self.last_pose_stamp = t
        p = m.pose
        r = self.est.pose(t, p.position.x, p.position.y, yaw_of(p.orientation))
        if r == "jump":
            e = self.est.last_residual
            self.get_logger().warning(
                f"SLAM 포즈 튐: 잔차 {math.hypot(e[0], e[1])*100:.1f} cm, {math.degrees(e[2]):.1f}° — 버림 "
                f"(누적 {self.est.jump_count})", throttle_duration_sec=1.0)
        elif r == "reset":
            self.get_logger().error("SLAM 포즈가 계속 다른 곳을 가리킨다 — 그 포즈로 다시 초기화했다")
        elif r == "old":
            self.get_logger().warning("너무 오래된 stamp 의 포즈 — 버림", throttle_duration_sec=2.0)
        if r in ("used", "jump", "reset") and self.est.t is not None:
            e, s = self.est.last_residual, Float64MultiArray()
            s.data = [float(self.est.t), float(e[0]), float(e[1]), float(e[2]), float(self.est.b_a[0]),
                      float(self.est.b_a[1]), float(self.est.b_g), float(self.est.jump_count),
                      float(self.est.n_corrections), float(self.est.t - t)]
            self.pub_status.publish(s)

    def on_imu(self, m):
        t = stamp_s(m.header)
        if not self.est.imu(t, m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z):
            return
        if not self.was_ready:
            self.was_ready = True
            self.get_logger().info(
                f"준비 끝. 바이어스: 가속도 ({self.est.b_a[0]:+.3f}, {self.est.b_a[1]:+.3f}) m/s², "
                f"자이로 {self.est.b_g:+.4f} rad/s — 출력 시작")
        if not self.est.ready_to_publish():       # SLAM 포즈 끊김 → 출력을 멈춰 MPC·fault_monitor 가 세우게 한다
            self.get_logger().error(
                f"SLAM 포즈가 {self.est.dead_reckoning_age():.2f} s 동안 반영되지 않음 (한계 {self.est.max_dead_reckoning} s) "
                f"— IMU 적분만으로는 위치가 흘러가므로 출력을 멈춘다", throttle_duration_sec=1.0)
            return
        if self.last_pub is not None and t - self.last_pub < self.period - 0.002:
            return
        self.last_pub = t
        _, x, y, psi, vx, vy, wz = self.est.state()
        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id = self.map_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(psi / 2), math.cos(psi / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        self.pub.publish(o)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = StateEstimatorNode()
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

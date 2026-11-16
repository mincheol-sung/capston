#!/usr/bin/env python3
# state_estimator — SLAM 포즈(약 10 Hz, 늦게 도착) + IMU(200 Hz) → 100 Hz 상태 추정값 (구조도 v3).
#   ros2 run amr_mpc state_estimator_node
# ---------------------------------------------------------------
#   /imu/base_link          (sensor_msgs/Imu, BEST_EFFORT) 몸체 좌표 가속도 a_x, a_y, 자이로 ω_z
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

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64MultiArray

from amr_mpc.core import config, node_main
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
        # IMU 는 BEST_EFFORT 로 받는다 (실차에서 IMU 가 안 붙던 문제, 2026-11-13). 실차 imu_to_base_node 가 RealSense 에 맞춰 BEST_EFFORT 로
        # 내는데, 숫자(50)만 넘기면 RELIABLE 구독이 돼 연결이 막힌다 — IMU 가 0 개 → 바이어스 초기화가 안 끝나 출력 없음.
        # BEST_EFFORT 구독은 RELIABLE 발행(시뮬)과도 붙는다.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, str(g("imu_topic")), self.on_imu, sensor_qos)
        self.create_subscription(PoseStamped, str(g("pose_topic")), self.on_pose, 50)
        if self.pose_source == "after_scan_match":
            self.create_subscription(PointCloud2, str(g("scan_matched_topic")), self.on_scan_matched, 10)
        # 입력 감시 (실차 점검, 2026-11-13). 시뮬은 모든 노드가 같은 시계·같은 설정이라 드러나지 않는 실패를 로그로 보인다.
        #   · IMU·포즈가 아예 안 오면 (토픽 이름, 배달 방식(QoS), TF 대기 등) 아무 출력 없이 조용히 멈춰 있게 된다.
        #   · IMU stamp 는 MPC 가 기준열(시스템 시각 stamp)을 고르는 시각이 된다 → 시스템 시각과 어긋나면 기준이 밀린다.
        #   · 포즈 stamp 가 IMU 시각보다 미래면 그 시각의 추정값을 못 찾아 최신 값과 비교한다 (조용히 틀림).
        self.declare_parameters("", [("clock_tolerance", 0.1), ("input_check_period", 2.0)])
        self.clock_tol = float(g("clock_tolerance"))
        self.n_imu = self.n_pose = 0
        self.t_start = self.now_s()
        self.create_timer(float(g("input_check_period")), self.check_inputs)
        self.imu_topic, self.pose_topic = str(g("imu_topic")), str(g("pose_topic"))
        e = self.est
        self.get_logger().info(
            f"state_estimator: {g('imu_topic')} + {g('pose_topic')} [{self.pose_source}] → {g('output_topic')} "
            f"({1/self.period:.0f} Hz)\n"
            f"  IMU·포즈 기준점 ({e.r[0]:.3f}, {e.r[1]:.3f}) m (base_footprint 기준)\n"
            f"  게인  K_p {e.K_xy[0]:.3f}  K_v {e.K_xy[1]:.2f} 1/s  K_b {e.K_xy[2]:.2f} 1/s²   "
            f"K_psi {e.K_yaw[0]:.3f}  K_bg {e.K_yaw[1]:.4f} 1/s\n"
            f"  정지 상태에서 {e.init_s:.1f} s 동안 바이어스를 잡는다 — 그동안 로봇을 움직이지 말 것")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_inputs(self):
        """입력이 안 들어와 출력이 없는 이유를 주기적으로 알린다 (준비가 끝나면 조용)."""
        if self.was_ready:
            return
        waited = self.now_s() - self.t_start
        if self.n_imu == 0:
            self.get_logger().error(
                f"{waited:.0f} s 동안 IMU 0 개 ({self.imu_topic}). 확인: ros2 topic info -v {self.imu_topic} "
                f"(발행자가 있는지, 배달 방식 QoS), imu_to_base_node 의 TF 대기 로그")
        elif not self.est.bias_ready:
            self.get_logger().warning(f"IMU {self.n_imu} 개 받음 — 정지 바이어스 아직 (로봇이 흔들리면 다시 모은다)")
        if self.n_pose == 0:
            self.get_logger().error(f"{waited:.0f} s 동안 SLAM 포즈 0 개 ({self.pose_topic}) — Cartographer 측위가 떠 있는지 확인")

    def on_scan_matched(self, _msg):
        self.take_next_pose = True

    def on_pose(self, m):
        t = stamp_s(m.header)
        self.n_pose += 1
        if self.est.t is not None and t - self.est.t > 0.05:
            self.get_logger().warning(
                f"SLAM 포즈 stamp 가 IMU 시각보다 {t - self.est.t:.3f} s 미래 — 두 센서의 시각 기준이 다르다 "
                f"(라이다·RealSense 시각 설정 확인)", throttle_duration_sec=2.0)
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
            self.get_logger().warning(
                f"너무 오래된 stamp 의 포즈 ({self.est.t - t:.3f} s 전 > {self.est.max_pose_age} s) — 버림. "
                f"계속되면 SLAM 지연이나 두 센서의 시각 기준 차이", throttle_duration_sec=2.0)
        if r in ("used", "jump", "reset") and self.est.t is not None:
            e, s = self.est.last_residual, Float64MultiArray()
            s.data = [float(self.est.t), float(e[0]), float(e[1]), float(e[2]), float(self.est.b_a[0]),
                      float(self.est.b_a[1]), float(self.est.b_g), float(self.est.jump_count),
                      float(self.est.n_corrections), float(self.est.t - t)]
            self.pub_status.publish(s)

    def on_imu(self, m):
        t = stamp_s(m.header)
        self.n_imu += 1
        lag = self.now_s() - t
        if abs(lag) > self.clock_tol:
            self.get_logger().warning(
                f"IMU stamp 가 시스템 시각과 {lag:+.3f} s 어긋남 (허용 {self.clock_tol} s) — MPC 가 기준열을 그만큼 "
                f"밀린 시각에서 고른다. RealSense global_time_enabled, 시각 동기 확인", throttle_duration_sec=2.0)
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
        node_main.spin(node)          # Ctrl+C·SIGTERM·SIGHUP 공통 처리, 신호가 메시지 변환 중에 떨어져도 정상 종료 (R9·R10 리뷰)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

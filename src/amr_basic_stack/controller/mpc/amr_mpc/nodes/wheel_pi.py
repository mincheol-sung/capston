#!/usr/bin/env python3
# wheel_pi_node — 비교군 A 의 바퀴별 PI 속도 루프 (6단계). 계산은 amr_mpc/baseline/wheel_pi.py.
#   ros2 run amr_mpc wheel_pi_node
# ---------------------------------------------------------------
#   /pure_pursuit/cmd_vel   (TwistStamped)        몸체 속도 명령 [v_x, v_y, ω_z]
#   /wheel_states           (WheelStates)         바퀴 각속도 ω_i (실차 ak40_can_node, 시뮬 플랜트 — 100 Hz)
#   /mpc/armed              (Bool, latched)       게이트 무장 여부
#      ↓  /wheel_states 가 올 때마다 한 번 (100 Hz. MPC 가 상태 추정값마다 한 번 푸는 것과 같은 방식)
#         ω_ref = T⁻¹ v  →  τ_i = K_p e_i + K_i ∫e_i  →  |τ| ≤ τ̄_B, |Δτ| ≤ Δτ̄_B
#   /wheel_pi/cmd_raw       (WheelTorqueCommand)  → torque_output_node → 게이트 → /wheel_torque_cmd → 플랜트
#                                                 (MPC 의 /torque_mpc/cmd_raw 자리. 뒤 경로는 MPC 와 똑같다)
#   /wheel_pi/status        (Float64MultiArray)   [t, ω_ref×4, ω×4, τ×4, 포화 바퀴 수]
#
# 속도 되먹임 출처 speed_source (2026-11-19, 사용자 결정 — MPC 와 위치·속도 정보 출처를 같게)
#   wheels    (기본) 엔코더 ω_i — 표준 퓨어 퍼슛 + 바퀴 PI
#   estimator 추정기 몸체 속도 [v_x, v_y, ω_z] (speed_topic, 기본 /state_estimator/odom) 를 T⁻¹ 로 바퀴 속도로 바꿔 ω_i 자리에.
#             MPC 와 같은 정보 (추정기 v_x 는 바퀴보다 약 0.09 s 늦다 — 2026-11-18 bag). 실행 시각은 그대로 /wheel_states 마다.
#             추정기 값이 wheel_timeout 보다 묵으면 DAMPING. PI 게인은 엔코더 기준으로 맞춘 값이라 진동하면 낮춘다.
#
# 공정성 (비교군 B 와 같게 — torque_mpc_todo.md "비교군 A")
#   같은 주기 100 Hz · 같은 출력 경로(torque_output 마찰 보상·안전 클램프, 게이트) ·
#   같은 토크 상한 τ̄ = τ_lim − τ_fric,max 와 변화율 Δτ̄ = τ̄·rate_fraction (torque_plant.yaml, torque_mpc.yaml 을 그대로 읽는다) ·
#   같은 정지 규칙(추종 중이 아니면 언제나 MODE_DAMPING, 그때 PI 리셋 — 토크 0 은 관성 주행이라 정지가 아니다)
#
# DAMPING + 리셋 조건
#   미무장 · 무장 뒤 새 속도 명령 전 · 속도 명령 끊김(cmd_timeout — 퓨어 퍼슛이 정지를 결정하면 명령을 끊는다) ·
#   /wheel_states 끊김(wheel_timeout, 감시 타이머가 낸다)
#
# 구동계 방향 보정 (2026-11-23, torque_plant.yaml drive — 기본 0): 실물 v_실제,xy = R(δ)·(T ω)_xy 이므로 ω = T⁻¹·R(−δ)·v.
#   δ = δ0 + c·a_c, a_c = |v|·ω_z (그 v 로). 근거 analysis/torque_mpc_real/2026-11-23_horizon_real 5·6 절. launch drive_yaw_deg·drive_slip_coef
# ---------------------------------------------------------------

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64MultiArray

from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from amr_mpc.baseline.wheel_pi import WheelPI, wheel_targets
from amr_mpc.core import config, node_main
from amr_mpc.torque.model import TorquePlant, drive_angle, drive_params, drive_Tinv


class WheelPINode(Node):
    def __init__(self):
        super().__init__("wheel_pi_node")
        ya = config.load("torque_baseline_a.yaml")["wheel_pi"]
        ym = config.load("torque_mpc.yaml")
        plant = TorquePlant.from_yaml()
        dyaw, dcoef, dac = drive_params(config.load("torque_plant.yaml"))
        self.declare_parameters("", [
            ("cmd_topic", "/pure_pursuit/cmd_vel"),
            ("wheel_states_topic", "/wheel_states"),
            ("armed_topic", "/mpc/armed"),
            ("output_topic", "/wheel_pi/cmd_raw"),
            ("kp", float(ya["kp"])),
            ("ki", float(ya["ki"])),
            ("tau_max_Nm", float(ya.get("tau_max_Nm", -1.0))),          # ≤ 0 이면 B 와 같게 (τ_lim − τ_fric,max)
            ("rate_fraction", float(ya.get("rate_fraction", -1.0))),     # ≤ 0 이면 B 와 같게 (torque_mpc.yaml)
            ("Ts", float(ym["tuning"]["Ts"])),
            ("cmd_timeout", float(ya.get("cmd_timeout", 0.05))),
            ("wheel_timeout", float(ym["safety"]["state_timeout"])),
            ("speed_source", str(ya.get("speed_source", "wheels"))),   # wheels | estimator (머리주석)
            ("speed_topic", "/state_estimator/odom"),
            # 2026-11-23 구동계 방향 보정 (torque_plant.yaml drive, 기본 0). ω_ref = T⁻¹·R(−δ)·v_cmd,  δ = δ0 + c·a_c (a_c = |v_cmd|·ω_cmd)
            ("drive_yaw_offset_deg", dyaw),
            ("drive_slip_coef_deg", dcoef),
            ("drive_slip_ac_max", dac),
        ])
        g = lambda n: self.get_parameter(n).value
        tau_max = float(g("tau_max_Nm"))
        if tau_max <= 0.0:
            tau_max = plant.tau_motor_limit()
        rf = float(g("rate_fraction"))
        if rf <= 0.0:
            rf = float(ym["torque_limit"]["rate_fraction"])
        self.Ts = float(g("Ts"))
        self.pi = WheelPI(float(g("kp")), float(g("ki")), tau_max, tau_max * rf / self.Ts, dt_max=3.0 * self.Ts)
        self.Tinv = plant.Tinv
        self.drive = (math.radians(float(g("drive_yaw_offset_deg"))), math.radians(float(g("drive_slip_coef_deg"))),
                      float(g("drive_slip_ac_max")))
        if self.drive[0] or self.drive[1]:
            self.get_logger().info(f"구동계 방향 보정: δ0 {math.degrees(self.drive[0]):+.2f}°, 선회 미끄럼 {math.degrees(self.drive[1]):+.2f} °/(m/s²) "
                                   f"× a_c (|a_c| ≤ {self.drive[2]:.1f}) — ω_ref = T⁻¹·R(−δ)·v")
        self.cmd_timeout, self.wheel_timeout = float(g("cmd_timeout")), float(g("wheel_timeout"))

        self.speed_source = str(g("speed_source")).strip().lower()
        if self.speed_source not in ("wheels", "estimator"):
            raise SystemExit(f"[wheel_pi] speed_source={self.speed_source!r} — wheels | estimator")
        self.v_est, self.est_rx = np.zeros(3), 0.0
        self.v_cmd, self.cmd_rx = np.zeros(3), 0.0
        self.last_stamp, self.wheel_rx = None, 0.0
        self.armed, self.armed_since, self.active = False, 0.0, False

        self.pub = self.create_publisher(WheelTorqueCommand, str(g("output_topic")), 10)
        self.pub_status = self.create_publisher(Float64MultiArray, "/wheel_pi/status", 10)
        self.create_subscription(TwistStamped, str(g("cmd_topic")), self.on_cmd, 10)
        self.create_subscription(WheelStates, str(g("wheel_states_topic")), self.on_wheels, 10)
        if self.speed_source == "estimator":
            self.create_subscription(Odometry, str(g("speed_topic")), self.on_est, 20)
        armed_topic = str(g("armed_topic"))
        if armed_topic:
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, armed_topic, self.on_armed, latched)
        else:
            self.armed = True
            self.get_logger().warning("armed_topic 이 비어 있다 — 무장 여부를 보지 않는다 (단독 시험 전용)")
        self.create_timer(0.05, self.on_watchdog)
        self.get_logger().info(
            f"바퀴별 PI (비교군 A). {g('cmd_topic')} + {g('wheel_states_topic')} → {g('output_topic')}\n"
            f"  K_p {self.pi.kp:g} N·m/(rad/s)  K_i {self.pi.ki:g} N·m/rad   상한 τ̄ {tau_max:.4f} N·m, "
            f"Δτ̄ {tau_max * rf:.4f} N·m/{self.Ts*1e3:.0f} ms (비교군 B 와 같다)   명령 끊김 {self.cmd_timeout:g} s\n"
            f"  속도 되먹임: " + ("엔코더 ω (표준)" if self.speed_source == "wheels" else
                               f"추정기 {g('speed_topic')} 의 몸체 속도 → T⁻¹ (MPC 와 같은 정보)"))

    def on_est(self, m):
        tw = m.twist.twist
        self.v_est = np.array([tw.linear.x, tw.linear.y, tw.angular.z], float)
        self.est_rx = self.now_s()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_armed(self, m):
        if bool(m.data) != self.armed:
            self.armed = bool(m.data)
            self.armed_since = self.now_s()
            self.get_logger().info("게이트 무장" if self.armed else "게이트 정지 — DAMPING, PI 리셋")

    def on_cmd(self, m):
        self.v_cmd = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.angular.z], float)
        self.cmd_rx = self.now_s()

    def Tinv_at(self, v):
        """몸체 속도 v 에서의 보정 T⁻¹ (drive 가 0 이면 공칭 그대로)."""
        return drive_Tinv(self.Tinv, drive_angle(*self.drive[:2], v, self.drive[2]))

    def damping(self, reason):
        out = WheelTorqueCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_footprint"
        out.mode = WheelTorqueCommand.MODE_DAMPING
        self.pub.publish(out)
        if self.active:
            self.get_logger().info(f"정지(DAMPING): {reason}")
        self.active = False
        self.pi.reset()

    def on_watchdog(self):
        if self.now_s() - self.wheel_rx > self.wheel_timeout:
            self.damping("/wheel_states 없음")
            self.last_stamp = None
            self.get_logger().warning("/wheel_states 대기/끊김", throttle_duration_sec=2.0)

    def on_wheels(self, m):
        now = self.now_s()
        self.wheel_rx = now
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = st if st > 0.0 else now
        dt = self.Ts if self.last_stamp is None else t - self.last_stamp
        self.last_stamp = t
        if not self.armed:
            return self.damping("게이트 미무장")
        if self.cmd_rx <= self.armed_since:
            return self.damping("무장 뒤 새 속도 명령 대기")
        if now - self.cmd_rx > self.cmd_timeout:
            return self.damping("속도 명령 끊김 (퓨어 퍼슛 정지)")
        if not self.active:
            self.active = True
            self.pi.reset()
            dt = self.Ts
            self.get_logger().info("PI 시작")

        if self.speed_source == "estimator":
            if now - self.est_rx > self.wheel_timeout:
                return self.damping("추정기 속도 끊김")
            omega = wheel_targets(self.Tinv_at(self.v_est), self.v_est)    # 추정기 몸체 속도 → 바퀴 속도 (T⁻¹ v, 보정 포함)
        else:
            omega = np.asarray(m.velocity, float)
        omega_ref = wheel_targets(self.Tinv_at(self.v_cmd), self.v_cmd)
        tau = self.pi.step(omega_ref, omega, dt)
        out = WheelTorqueCommand()
        out.header.stamp = m.header.stamp           # 계산에 쓴 바퀴 상태의 시각 (종단 지연 측정용, MPC 와 같은 규약)
        out.header.frame_id = "base_footprint"
        out.mode = WheelTorqueCommand.MODE_TORQUE
        out.torque = [float(x) for x in tau]
        self.pub.publish(out)
        s = Float64MultiArray()
        s.data = [now, *map(float, omega_ref), *map(float, omega), *map(float, tau), float(self.pi.saturated.sum())]
        self.pub_status.publish(s)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WheelPINode()
    try:
        node_main.spin(node)          # Ctrl+C·SIGTERM·SIGHUP 공통 처리, 신호가 메시지 변환 중에 떨어져도 정상 종료 (R9·R10 리뷰)
    finally:
        try:
            node.damping("종료")
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

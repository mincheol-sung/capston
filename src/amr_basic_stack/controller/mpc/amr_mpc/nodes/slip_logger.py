#!/usr/bin/env python3
# slip_logger — 바퀴별 슬립률을 계산해 기록한다 (구조도 v3, 로깅 전용 — 제어로 되먹이지 않는다).
#   ros2 run amr_mpc slip_logger_node
# ---------------------------------------------------------------
#   /wheel_states           (WheelStates)   바퀴가 실제로 도는 각속도 ω_i
#   /state_estimator/odom   (Odometry)      차체 속도 v̂ (라이다 + IMU 추정, 바퀴와 무관)
#   reference_topic         (Odometry)      비교용 참값 (시뮬 /sim/ground_truth, 비우면 안 씀)
#      ↓
#   /slip_logger/slip       (Float64MultiArray)  [t, λ_FL, λ_FR, λ_RL, λ_RR, ω_nom×4, (참값 λ×4)]
#                           응답이 없거나(fresh false) 에러인 바퀴의 λ 는 NaN (그 바퀴 속도는 오래된 값이다)
#   CSV (log_prefix) + 종료 때 요약
#
# 슬립률 (구조도 정의, torque_mpc_todo.md "결정됨"):
#   ω_nom = T⁻¹ v̂      미끄러지지 않았다면 바퀴가 돌았어야 할 각속도
#   λ_i  = (ω_i − ω_nom,i)·sgn(ω_nom,i) / max(|ω_i|, |ω_nom,i|, ω_ε)
#   λ > 0: 헛돎(구동 슬립, 바퀴가 더 빨리 돈다),  λ < 0: 끌림(제동 슬립).  범위 −2 … +2
#   (±1 = 한쪽이 멈춤, ±2 를 향하면 바퀴가 몸체와 반대로 돈다)
#   ω_ε 는 거의 멈춘 바퀴에서 0 으로 나누지 않게 하는 바닥값이다.
#
# 한계: v̂ 의 오차가 그대로 λ 오차가 된다. 4단계 시뮬 실측 수준 잡음에서 추정 속도 오차 약 5 cm/s RMS
#   → 0.5 m/s 에서 λ 약 ±0.1 이 바닥 잡음이다. 그래서 평균·RMS 와 함께 "|λ| > noise_floor 인 비율" 을 낸다.
# ---------------------------------------------------------------
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray

from amr_interfaces.msg import WheelStates
from amr_mpc.core import node_main
from amr_mpc.torque.model import TorquePlant

WHEELS = ("FL", "FR", "RL", "RR")


def slip_ratio(omega, omega_nom, omega_eps):
    """구조도 정의의 슬립률 (4,)."""
    omega, omega_nom = np.asarray(omega, float), np.asarray(omega_nom, float)
    den = np.maximum(np.maximum(np.abs(omega), np.abs(omega_nom)), omega_eps)
    sgn = np.where(omega_nom >= 0.0, 1.0, -1.0)
    return (omega - omega_nom) * sgn / den


class SlipLoggerNode(Node):
    def __init__(self):
        super().__init__("slip_logger_node")
        self.declare_parameters("", [
            ("wheel_states_topic", "/wheel_states"),
            ("body_topic", "/state_estimator/odom"),
            ("reference_topic", ""),          # 시뮬: /sim/ground_truth
            ("omega_eps", 2.5),               # [rad/s] = 0.1 m/s 바퀴 둘레 속도. 이보다 느린 바퀴의 슬립률은 작게 눌린다
            ("noise_floor", 0.1),             # 추정 속도 잡음으로 생기는 |λ| 수준 (요약 표시용)
            ("min_speed", 0.05),              # [m/s] 이보다 느릴 때는 요약 통계에서 뺀다
            ("log_prefix", ""),
        ])
        g = lambda n: self.get_parameter(n).value
        self.Tinv = TorquePlant.from_yaml().Tinv
        self.eps, self.floor, self.min_speed = float(g("omega_eps")), float(g("noise_floor")), float(g("min_speed"))
        self.v_body = None
        self.v_ref = None
        self.rows = []
        self.log_prefix = str(g("log_prefix"))
        self.pub = self.create_publisher(Float64MultiArray, "/slip_logger/slip", 20)
        self.create_subscription(Odometry, str(g("body_topic")), self.on_body, 20)
        if str(g("reference_topic")):
            self.create_subscription(Odometry, str(g("reference_topic")), self.on_ref, 20)
        self.create_subscription(WheelStates, str(g("wheel_states_topic")), self.on_wheels, 20)
        # 주기적으로 깨어나야 종료 신호(Ctrl-C)를 바로 받는다. 다른 노드가 먼저 꺼져 메시지가 끊기면
        # spin 이 대기에서 안 깨어나 launch 가 5 s 뒤 강제 종료했고, 그러면 요약·CSV 가 남지 않았다 (2026-11-11)
        self.create_timer(0.2, lambda: None)
        self.get_logger().info(
            f"slip_logger: 바퀴 {g('wheel_states_topic')} + 차체 {g('body_topic')}"
            f"{' + 참값 ' + str(g('reference_topic')) if str(g('reference_topic')) else ''} → /slip_logger/slip "
            f"(ω_ε {self.eps} rad/s, 잡음 바닥 |λ| {self.floor})")

    @staticmethod
    def _twist(m):
        t = m.twist.twist
        return np.array([t.linear.x, t.linear.y, t.angular.z])

    def on_body(self, m):
        self.v_body = self._twist(m)

    def on_ref(self, m):
        self.v_ref = self._twist(m)

    def on_wheels(self, m):
        if self.v_body is None:
            return
        w = np.array(m.velocity, float)
        w_nom = self.Tinv @ self.v_body
        lam = slip_ratio(w, w_nom, self.eps)
        # 실차: 응답이 없거나(fresh false) 에러인 바퀴의 velocity 는 마지막 값이 그대로 남아 있다 → 차체만 변하면 가짜 슬립.
        # 그 바퀴는 NaN 으로 둔다 (R9 리뷰 2026-11-14). 시뮬 토픽 모드는 늘 fresh, 에러 0 이라 안 보였다.
        bad = np.zeros(4, bool)
        if len(m.fresh) == 4:
            bad |= ~np.array(m.fresh, bool)
        if len(m.error) == 4:
            bad |= np.array(m.error) != 0
        lam[bad] = math.nan
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        data = [t, *lam, *w_nom]
        lam_ref = None
        if self.v_ref is not None:
            lam_ref = slip_ratio(w, self.Tinv @ self.v_ref, self.eps)
            lam_ref[bad] = math.nan
            data += list(lam_ref)
        out = Float64MultiArray()
        out.data = [float(x) for x in data]
        self.pub.publish(out)
        speed = math.hypot(self.v_body[0], self.v_body[1])
        self.rows.append((t, speed, *lam, *(lam_ref if lam_ref is not None else [math.nan] * 4)))

    def finish(self):
        if not self.rows:
            return
        R = np.array(self.rows)
        mv = R[:, 1] > self.min_speed
        lines = [f"[slip_logger] {len(R)}행, 움직인 구간 {mv.sum()}행 (차체 속도 > {self.min_speed} m/s)"]
        if mv.any():
            for i, wname in enumerate(WHEELS):
                lam = R[mv, 2 + i]
                n_bad = int(np.isnan(lam).sum())         # 응답 없음·에러로 뺀 행
                lam = lam[~np.isnan(lam)]
                if lam.size == 0:
                    lines.append(f"  {wname}: 쓸 수 있는 행 없음 (응답 없음·에러 {n_bad}행)")
                    continue
                txt = (f"  {wname}: λ 평균 {lam.mean():+.3f}  RMS {np.sqrt(np.mean(lam**2)):.3f}  최대 |λ| {np.abs(lam).max():.3f}"
                       f"  |λ|>{self.floor} 비율 {np.mean(np.abs(lam) > self.floor)*100:.1f} %"
                       + (f"  (응답 없음·에러로 뺀 행 {n_bad})" if n_bad else ""))
                ref = R[mv, 6 + i]
                if not np.all(np.isnan(ref)):
                    txt += f"   | 참값 λ RMS {np.sqrt(np.nanmean(ref**2)):.3f}  최대 {np.nanmax(np.abs(ref)):.3f}"
                lines.append(txt)
        print("\n".join(lines), flush=True)
        if self.log_prefix:
            path = f"{self.log_prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            hdr = "t,speed," + ",".join(f"lam_{w}" for w in WHEELS) + "," + ",".join(f"lam_ref_{w}" for w in WHEELS)
            np.savetxt(path, R, delimiter=",", header=hdr, comments="", fmt="%.6f")
            print(f"[slip_logger] 저장: {path}", flush=True)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SlipLoggerNode()
    try:
        node_main.spin(node)          # Ctrl+C·SIGTERM 이 메시지 변환 중에 떨어져도 정상 종료, SIGTERM 에도 요약·CSV 저장 (R9 리뷰)
    finally:
        node.finish()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

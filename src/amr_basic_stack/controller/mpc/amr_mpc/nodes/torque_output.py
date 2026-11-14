#!/usr/bin/env python3
# torque_output — MPC 순토크를 모터 명령 토크로 바꾼다 (구조도 v4).
#   ros2 run amr_mpc torque_output_node
# ---------------------------------------------------------------
#   /torque_mpc/cmd_raw     (WheelTorqueCommand)  MPC 순토크 τ* — 바닥에 전달하려는 토크
#   /wheel_states           (WheelStates)         바퀴 각속도 ω (마찰 보상용)
#      ↓  τ_cmd = τ* + τ_fric(ω),  |τ_cmd| ≤ motor.tau_rated_Nm (안전 클램프)
#   /torque_output/cmd_raw  (WheelTorqueCommand)  → 게이트 → /wheel_torque_cmd
#
# 구동계 마찰 보상  τ_fric(ω) = coulomb·tanh(ω/ω_s) + viscous·ω   (config/torque_plant.yaml friction 절)
#   감속기가 먹는 토크를 미리 더해, 바퀴에 실제로 τ* 가 전달되게 한다. MPC 의 외란 추정이 아니라
#   벤치에서 잰 고정 표로 하는 모터 보정이다.  2026-11-13: 벤치 식별 계수를 [FL, FR, RL, RR]별로 적용한다.
#   coulomb 항은 ω = 0 근처에서 부호가 튀지 않게 tanh 로 잇는다 (ω_s = friction_smooth_rad_s).
#   보상 계수 = friction 절 값 × compensation_scale (2026-11-14 기본 0.9 — 과보상 떨림을 피해 약간 덜 보상).
#   friction 절 값 자체는 실측 그대로라 시뮬 플랜트(실제 마찰)와 보상이 10 % 어긋난 채로 시험된다.
#
# 규칙
#   - 받는 즉시 내보낸다 (타이머 없음). header.stamp 는 바꾸지 않는다 — 종단 지연 측정용.
#   - MODE_DAMPING 은 그대로 통과시킨다.
#   - 변화율 제한은 여기 없다. MPC 제약이 한다 (v3 에서 이동).
#   - /wheel_states 가 끊기면 마찰 보상만 0 으로 두고 토크는 계속 낸다 (마찰 값이 0 이 아닐 때만 경고).
#   - 토크에 NaN·inf 가 있으면 DAMPING 으로 바꿔 낸다 (R6 리뷰 2026-11-13). 그대로 두면 inf 는 클램프에서
#     ±τ_lim(최대 토크)이 된다. CAN 다리(ak40 bridge.set_command)도 NaN·inf 를 DAMPING 으로 바꾸지만 여기서 먼저 막는다.
#     바퀴 속도에 NaN 이 있으면 그 바퀴 마찰 보상만 0 으로 둔다.
# ---------------------------------------------------------------

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from amr_mpc.torque.model import TorquePlant, wheel_coefficients
from amr_mpc.core import config, node_main


def friction_torque(omega, coulomb, viscous, smooth):
    """τ_fric(ω) [N·m] — 바퀴별 (4,)."""
    omega = np.asarray(omega, float)
    return wheel_coefficients(coulomb) * np.tanh(omega / smooth) + wheel_coefficients(viscous) * omega


def motor_command(tau_net, omega, coulomb, viscous, smooth, tau_limit):
    """순토크 + 마찰 보상을 안전 한계로 자른 모터 명령. 반환 (τ_cmd, 잘렸는가).
    τ_net 이 유한하다고 가정한다 (on_cmd 가 먼저 거른다). ω 의 NaN·inf 는 그 바퀴 보상만 0."""
    omega = np.asarray(omega, float)
    omega = np.where(np.isfinite(omega), omega, 0.0)
    raw = np.asarray(tau_net, float) + friction_torque(omega, coulomb, viscous, smooth)
    out = np.clip(raw, -tau_limit, tau_limit)
    return out, bool(np.any(out != raw))


def friction_reserve_warning(coulomb, viscous, fric_max, max_speed=0.0):
    """Check compensation reserve at the configured operating speed (not MIT range)."""
    coulomb, viscous = wheel_coefficients(coulomb), wheel_coefficients(viscous)
    required = float(np.max(coulomb + viscous * abs(max_speed)))
    if required > fric_max + 1e-12:
        return (f"마찰 보상 여유 {fric_max} N·m < 운용 속도 {max_speed:.3f} rad/s에서 "
                f"추정 최대 보상 {required:.6f} N·m. 토크 상한 근처에서는 보상이 잘릴 수 있음.")
    return None


class TorqueOutputNode(Node):
    def __init__(self):
        super().__init__("torque_output_node")
        p = TorquePlant.from_yaml()
        rated_rpm = float(config.load("torque_plant.yaml")["motor"]["rated_speed_rpm"])
        self.declare_parameters("", [
            ("input_topic", "/torque_mpc/cmd_raw"),
            ("output_topic", "/torque_output/cmd_raw"),
            ("wheel_states_topic", "/wheel_states"),
            ("coulomb_Nm", p.fric_coulomb.tolist()),
            ("viscous_Nms_per_rad", p.fric_viscous.tolist()),
            ("compensation_scale", float(config.load("torque_plant.yaml")["friction"].get("compensation_scale", 1.0))),
            ("friction_smooth_rad_s", 0.5),
            ("tau_limit_Nm", p.tau_rated),
            ("wheel_timeout", 0.3),
            ("max_wheel_rad_per_sec", rated_rpm * 2.0 * np.pi / 60.0),
        ])
        g = lambda n: self.get_parameter(n).value
        scale = float(g("compensation_scale"))
        if not 0.0 <= scale <= 1.5:
            raise ValueError(f"compensation_scale {scale} — 0~1.5 밖 (1 초과는 과보상)")
        self.coulomb = scale * wheel_coefficients(g("coulomb_Nm"), "coulomb_Nm")
        self.viscous = scale * wheel_coefficients(g("viscous_Nms_per_rad"), "viscous_Nms_per_rad")
        self.smooth, self.limit = max(1e-6, float(g("friction_smooth_rad_s"))), float(g("tau_limit_Nm"))
        self.wheel_timeout = float(g("wheel_timeout"))
        self.has_friction = bool(np.any(self.coulomb) or np.any(self.viscous))
        self.omega = np.zeros(4)
        self.omega_t = None

        self.pub = self.create_publisher(WheelTorqueCommand, str(g("output_topic")), 10)
        self.create_subscription(WheelStates, str(g("wheel_states_topic")), self.on_wheel, 10)
        self.create_subscription(WheelTorqueCommand, str(g("input_topic")), self.on_cmd, 10)
        self.get_logger().info(
            f"torque_output: {g('input_topic')} → {g('output_topic')}   마찰 보상 "
            f"{'없음(0)' if not self.has_friction else f'coulomb {self.coulomb} N·m, viscous {self.viscous} N·m·s/rad (실측 × {scale})'}"
            f"   안전 클램프 ±{self.limit} N·m")
        warn = friction_reserve_warning(self.coulomb, self.viscous, p.fric_max, float(g("max_wheel_rad_per_sec")))
        if warn:
            self.get_logger().warning(warn)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wheel(self, msg):
        self.omega = np.array(msg.velocity, float)
        self.omega_t = self.now_s()

    def on_cmd(self, msg):
        if msg.mode != WheelTorqueCommand.MODE_TORQUE:
            self.pub.publish(msg)                       # DAMPING 등은 그대로
            return
        if not np.all(np.isfinite(msg.torque)):
            self.get_logger().error(f"MPC 토크에 NaN·inf {list(msg.torque)} — DAMPING 으로 바꿔 낸다",
                                    throttle_duration_sec=1.0)
            out = WheelTorqueCommand()
            out.header = msg.header
            out.mode = WheelTorqueCommand.MODE_DAMPING
            self.pub.publish(out)
            return
        omega = self.omega
        if self.omega_t is None or self.now_s() - self.omega_t > self.wheel_timeout:
            omega = np.zeros(4)
            if self.has_friction:
                self.get_logger().warning("/wheel_states 끊김 — 마찰 보상 없이 낸다", throttle_duration_sec=2.0)
        tau, clipped = motor_command(msg.torque, omega, self.coulomb, self.viscous, self.smooth, self.limit)
        if clipped:
            self.get_logger().warning(f"안전 클램프 ±{self.limit} N·m 에 걸림", throttle_duration_sec=2.0)
        out = WheelTorqueCommand()
        out.header = msg.header                         # stamp 유지
        out.mode = WheelTorqueCommand.MODE_TORQUE
        out.torque = [float(v) for v in tau]
        self.pub.publish(out)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TorqueOutputNode()
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

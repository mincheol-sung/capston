#!/usr/bin/env python3
# torque_output — MPC 순토크를 모터 명령 토크로 바꾼다 (구조도 v3, 1판 식-22).
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
#
# 규칙
#   - 받는 즉시 내보낸다 (타이머 없음). header.stamp 는 바꾸지 않는다 — 종단 지연 측정용.
#   - MODE_DAMPING 은 그대로 통과시킨다.
#   - 변화율 제한은 여기 없다. MPC 제약이 한다 (v3 에서 이동).
#   - /wheel_states 가 끊기면 마찰 보상만 0 으로 두고 토크는 계속 낸다 (마찰 값이 0 이 아닐 때만 경고).
# ---------------------------------------------------------------
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from amr_mpc.torque.model import TorquePlant, wheel_coefficients
from amr_mpc.core import config


def friction_torque(omega, coulomb, viscous, smooth):
    """τ_fric(ω) [N·m] — 바퀴별 (4,)."""
    omega = np.asarray(omega, float)
    return wheel_coefficients(coulomb) * np.tanh(omega / smooth) + wheel_coefficients(viscous) * omega


def motor_command(tau_net, omega, coulomb, viscous, smooth, tau_limit):
    """순토크 + 마찰 보상을 안전 한계로 자른 모터 명령. 반환 (τ_cmd, 잘렸는가)."""
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
            ("friction_smooth_rad_s", 0.5),
            ("tau_limit_Nm", p.tau_rated),
            ("wheel_timeout", 0.3),
            ("max_wheel_rad_per_sec", rated_rpm * 2.0 * np.pi / 60.0),
        ])
        g = lambda n: self.get_parameter(n).value
        self.coulomb = wheel_coefficients(g("coulomb_Nm"), "coulomb_Nm")
        self.viscous = wheel_coefficients(g("viscous_Nms_per_rad"), "viscous_Nms_per_rad")
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
            f"{'없음(0)' if not self.has_friction else f'coulomb {self.coulomb} N·m, viscous {self.viscous} N·m·s/rad'}"
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

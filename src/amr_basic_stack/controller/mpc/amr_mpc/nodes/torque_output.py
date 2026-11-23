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
# 정지 마찰 보상 (2026-11-16, stiction_comp)
#   τ_static = F_s·exp(−(ω/ω_st)²)·tanh(ω_ref/ω_s)      F_s = friction.static_Nm, ω_st = friction.stribeck_rad_s
#   ω_ref = T⁻¹·[vx, vy, ω_z]_ref — 기준열(/dra_mppi/reference_trajectory, 몸체프레임 속도)의 지금 시각 값.
#   위 coulomb 항은 측정 ω 의 방향을 따르므로 바퀴가 서 있으면(ω = 0) 0 이 된다. 그래서 메카넘 역회전 지점에서
#   바퀴가 0 에 먼저 서면 모터 명령이 τ̄ 뿐이라 출발 토크를 못 넘고 붙어 있었다 (bag 2026-11-16/142534: FL·RR 1.78 s).
#   Stribeck 모델의 정지 수준을 "기준이 가라는 방향" 으로 더한다 (기준 속도 기반 friction feedforward,
#   Armstrong-Hélouvry·Dupont·Canudas de Wit 1994 Automatica 30(7)). 바퀴가 돌기 시작하면 exp 항이 사라져 위 식과 같고,
#   기준이 멈춰 있으면(ω_ref = 0, 정지·hold·무장 해제) tanh 항이 0 이라 서 있는 바퀴를 스스로 밀지 않는다.
#   기준열이 reference_timeout 넘게 안 오거나 지평을 다 썼으면 ω_ref = 0. 시뮬 플랜트에는 정지 마찰이 없어
#   launch 기본(stiction_comp:=auto)은 plant:=real 에서만 켠다.
#
# 규칙
#   - 받는 즉시 내보낸다 (타이머 없음). header.stamp 는 바꾸지 않는다 — 종단 지연 측정용.
#   - MODE_DAMPING 은 그대로 통과시킨다.
#   - 변화율 제한은 여기 없다. MPC 제약이 한다 (v3 에서 이동).
#   - /wheel_states 가 끊기면 마찰 보상만 0 으로 두고 토크는 계속 낸다 (마찰 값이 0 이 아닐 때만 경고).
#   - 토크에 NaN·inf 가 있으면 DAMPING 으로 바꿔 낸다 (R6 리뷰 2026-11-13). 그대로 두면 inf 는 클램프에서
#     ±τ_lim(최대 토크)이 된다. CAN 다리(ak40 bridge.set_command)도 NaN·inf 를 DAMPING 으로 바꾸지만 여기서 먼저 막는다.
#     바퀴 속도에 NaN 이 있으면 그 바퀴 마찰 보상만 0 으로 둔다.
#
# 바퀴별 슬립 가드 (2026-11-21, slip_guard — 기본 끔. 비교군 C 실험용)
#   /imu/base_link 자이로 ω_z 와 네 바퀴 ω 로 바퀴마다 leave-one-out 기구학 잔차 σ_i 를 내고, 한 바퀴가 자기 토크 방향으로
#   바닥보다 더 돌면 (가속 헛돎·제동 잠김) 그 바퀴 순토크에 곱하는 이득 g_i 를 줄인다 — 식·문턱 근거는 amr_mpc/torque/slip_guard.py.
#   마찰 보상 전 순토크 τ* 에 곱한다 (τ_cmd = g·τ* + τ_fric). MPC 는 이 줄임을 모른다 (MPC 바깥 층).
#   /torque_output/slip_guard (Float64MultiArray) [t, σ×4, g×4, 판정 바퀴 (−1 없음, 0 FL … 3 RR)]
#   자이로 바이어스는 네 바퀴가 서 있을 때 (|ω| < 0.05 rad/s) 지수 평균으로 잡는다. DAMPING 이 지나가면 g 를 1 로 되돌린다.
#
# 구동계 앞방향 보정 δ0 (2026-11-23, torque_plant.yaml drive.yaw_offset_deg, 기본 0): 정지 마찰 방향의 기준 바퀴속도와 슬립 가드의
#   T⁻¹ 을 T⁻¹·R(−δ0) 로 (amr_mpc/torque/model.py drive_Tinv). launch drive_yaw_deg
# ---------------------------------------------------------------

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

from amr_interfaces.msg import PlanarReferenceArray, WheelStates, WheelTorqueCommand
from amr_mpc.torque.slip_guard import SlipGuard
from amr_mpc.torque.model import TorquePlant, drive_params, drive_Tinv, wheel_coefficients
from amr_mpc.core import config, node_main


def friction_torque(omega, coulomb, viscous, smooth, omega_ref=None, static=0.0, stribeck=0.5):
    """τ_fric [N·m] — 바퀴별 (4,).
    운동 마찰 coulomb·tanh(ω/ω_s) + viscous·ω (측정 ω 방향) + 정지 마찰 static·exp(−(ω/ω_st)²)·tanh(ω_ref/ω_s)
    (기준 바퀴속도 ω_ref 방향, omega_ref 가 None 이면 없음)."""
    omega = np.asarray(omega, float)
    out = wheel_coefficients(coulomb) * np.tanh(omega / smooth) + wheel_coefficients(viscous) * omega
    if omega_ref is not None:
        st = wheel_coefficients(static, "static_Nm")
        if np.any(st):
            out = out + st * np.exp(-(omega / stribeck) ** 2) * np.tanh(np.asarray(omega_ref, float) / smooth)
    return out


def motor_command(tau_net, omega, coulomb, viscous, smooth, tau_limit, omega_ref=None, static=0.0, stribeck=0.5):
    """순토크 + 마찰 보상을 안전 한계로 자른 모터 명령. 반환 (τ_cmd, 잘렸는가).
    τ_net 이 유한하다고 가정한다 (on_cmd 가 먼저 거른다). ω·ω_ref 의 NaN·inf 는 그 바퀴 해당 항만 0."""
    omega = np.asarray(omega, float)
    omega = np.where(np.isfinite(omega), omega, 0.0)
    if omega_ref is not None:
        omega_ref = np.asarray(omega_ref, float)
        omega_ref = np.where(np.isfinite(omega_ref), omega_ref, 0.0)
    raw = np.asarray(tau_net, float) + friction_torque(omega, coulomb, viscous, smooth, omega_ref, static, stribeck)
    out = np.clip(raw, -tau_limit, tau_limit)
    return out, bool(np.any(out != raw))


def reference_wheel_speed(ref, now, Tinv, timeout):
    """기준열에서 지금 시각의 기준 바퀴속도 ω_ref = T⁻¹·[vx, vy, ω_z] (4,).
    ref = (stamp [s], dt [s], 몸체속도 행 (N, 3)) — references[i] 의 목표 시각은 stamp + (i+1)·dt (PlanarReferenceArray).
    행 사이는 선형 보간, 첫 목표 시각 전이면 첫 행. 기준열이 없거나, stamp 가 timeout 보다 오래됐거나,
    지평을 다 썼으면 0 (정지 마찰 보상을 끈다)."""
    if ref is None:
        return np.zeros(4)
    stamp, dt, rows = ref
    if len(rows) == 0 or dt <= 0.0 or now - stamp > timeout or now > stamp + len(rows) * dt:
        return np.zeros(4)
    u = (now - stamp) / dt - 1.0                       # 행 번호 (실수). 위 검사로 u ≤ N − 1
    if u <= 0.0 or len(rows) == 1:
        v = rows[0]
    else:
        i = min(int(u), len(rows) - 2)
        a = u - i
        v = (1.0 - a) * rows[i] + a * rows[i + 1]
    return np.asarray(Tinv, float) @ np.asarray(v, float)


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
        plant_yaml = config.load("torque_plant.yaml")
        rated_rpm = float(plant_yaml["motor"]["rated_speed_rpm"])
        fric_yaml = plant_yaml["friction"]
        self.declare_parameters("", [
            ("input_topic", "/torque_mpc/cmd_raw"),
            ("output_topic", "/torque_output/cmd_raw"),
            ("wheel_states_topic", "/wheel_states"),
            ("reference_topic", "/dra_mppi/reference_trajectory"),
            ("coulomb_Nm", p.fric_coulomb.tolist()),
            ("viscous_Nms_per_rad", p.fric_viscous.tolist()),
            ("compensation_scale", float(fric_yaml.get("compensation_scale", 1.0))),
            ("friction_smooth_rad_s", 0.5),
            ("stiction_comp", True),                  # 정지 마찰 보상 (launch 기본: plant:=real 에서만 켬)
            ("static_Nm", [float(v) for v in wheel_coefficients(fric_yaml.get("static_Nm", 0.0), "static_Nm")]),
            ("stribeck_rad_s", float(fric_yaml.get("stribeck_rad_s", 0.5))),
            ("reference_timeout", 0.5),               # [s] 기준열이 이보다 오래되면 ω_ref = 0
            ("tau_limit_Nm", p.tau_rated),
            ("wheel_timeout", 0.3),
            ("max_wheel_rad_per_sec", rated_rpm * 2.0 * np.pi / 60.0),
            ("slip_guard", False),                    # 바퀴별 슬립 가드 (2026-11-21). 아래는 그 설정 — amr_mpc/torque/slip_guard.py
            ("imu_topic", "/imu/base_link"),
            ("slip_threshold", 0.35),                 # σ 문턱 (bag 으로 고른 값. 2026-11-21 233706: 0.5 는 FL 헛돎을 0.15 s 늦게 잡아 0.35 로)
            ("slip_floor_rad_s", 5.0),                # σ 분모 하한 [rad/s] (0.2 m/s) — 저속 잡음
            ("slip_min_on_s", 0.05),                  # [s] 문턱을 이만큼 계속 넘어야 판정
            ("slip_gain_min", 0.4),                   # g 하한 — 토크를 0 까지 끊지 않는다
            ("slip_down_per_s", 8.0),                 # 판정 중 g 감소율 [1/s] (1 → 0.4 에 75 ms)
            ("slip_up_per_s", 2.0),                   # 판정이 풀리면 g 회복률 [1/s] (0.4 → 1 에 0.3 s)
            # 2026-11-23 구동계 앞방향 보정 δ0 (torque_plant.yaml drive, 기본 0) — 기준 바퀴속도 (정지 마찰 방향) 와 슬립 가드 T⁻¹ 에.
            #   선회 미끄럼 몫은 여기서 안 쓴다 (슬립 가드가 잡을 대상이 아니고, 정지 마찰 방향에는 영향이 작다)
            ("drive_yaw_offset_deg", drive_params(plant_yaml)[0]),
        ])
        g = lambda n: self.get_parameter(n).value
        scale = float(g("compensation_scale"))
        if not 0.0 <= scale <= 1.5:
            raise ValueError(f"compensation_scale {scale} — 0~1.5 밖 (1 초과는 과보상)")
        self.coulomb = scale * wheel_coefficients(g("coulomb_Nm"), "coulomb_Nm")
        self.viscous = scale * wheel_coefficients(g("viscous_Nms_per_rad"), "viscous_Nms_per_rad")
        self.static = scale * wheel_coefficients(g("static_Nm"), "static_Nm") if bool(g("stiction_comp")) else np.zeros(4)
        self.stribeck = float(g("stribeck_rad_s"))
        if self.stribeck <= 0.0:
            raise ValueError(f"stribeck_rad_s {self.stribeck} — 0 보다 커야 한다")
        self.ref_timeout = float(g("reference_timeout"))
        self.drive_yaw = math.radians(float(g("drive_yaw_offset_deg")))
        self.Tinv = drive_Tinv(p.Tinv, self.drive_yaw)          # 실제 몸체 속도 → 바퀴 ω (δ0 0 이면 공칭)
        self.smooth, self.limit = max(1e-6, float(g("friction_smooth_rad_s"))), float(g("tau_limit_Nm"))
        self.wheel_timeout = float(g("wheel_timeout"))
        self.has_friction = bool(np.any(self.coulomb) or np.any(self.viscous))
        self.has_static = bool(np.any(self.static))
        self.omega = np.zeros(4)
        self.omega_t = None
        self.ref = None                                 # (stamp, dt, 몸체속도 행 (N, 3))
        self.guard = None
        if bool(g("slip_guard")):
            self.guard = SlipGuard(self.Tinv, g("slip_threshold"), g("slip_floor_rad_s"), g("slip_min_on_s"), 3,
                                   g("slip_gain_min"), g("slip_down_per_s"), g("slip_up_per_s"))
            self.wz, self.wz_t, self.gyro_bias, self.bias_n = 0.0, None, 0.0, 0
            self.tau_net = np.zeros(4)
            self.pub_guard = self.create_publisher(Float64MultiArray, "/torque_output/slip_guard", 10)
            self.create_subscription(Imu, str(g("imu_topic")), self.on_imu, qos_profile_sensor_data)

        ref_topic = str(g("reference_topic"))
        self.pub = self.create_publisher(WheelTorqueCommand, str(g("output_topic")), 10)
        self.create_subscription(WheelStates, str(g("wheel_states_topic")), self.on_wheel, 10)
        if self.has_static:
            self.create_subscription(PlanarReferenceArray, ref_topic, self.on_reference, 10)
        self.create_subscription(WheelTorqueCommand, str(g("input_topic")), self.on_cmd, 10)
        static_txt = (f"{self.static} N·m (실측 × {scale}), ω_st {self.stribeck} rad/s, 방향 = {ref_topic} 의 ω_ref"
                      if self.has_static else "끔")
        self.get_logger().info(
            f"torque_output: {g('input_topic')} → {g('output_topic')}   마찰 보상 "
            f"{'없음(0)' if not self.has_friction else f'coulomb {self.coulomb} N·m, viscous {self.viscous} N·m·s/rad (실측 × {scale})'}"
            f"   정지 마찰 {static_txt}   안전 클램프 ±{self.limit} N·m")
        if self.drive_yaw:
            self.get_logger().info(f"구동계 앞방향 보정 δ0 {math.degrees(self.drive_yaw):+.2f}° — 기준 바퀴속도·슬립 가드 T⁻¹·R(−δ0)")
        if self.guard is not None:
            gd = self.guard
            self.get_logger().info(
                f"슬립 가드 켬: σ 문턱 {gd.th} (분모 하한 {gd.floor} rad/s, {gd.min_on*1e3:.0f} ms 지속), g ≥ {gd.g_min}, "
                f"감소 {gd.down}/s · 회복 {gd.up}/s, 자이로 {g('imu_topic')}")
        warn = friction_reserve_warning(self.coulomb, self.viscous, p.fric_max, float(g("max_wheel_rad_per_sec")))
        if warn:
            self.get_logger().warning(warn)
        if self.has_static and float(np.max(self.static)) > p.fric_max + 1e-12:
            self.get_logger().warning(f"정지 마찰 보상 {np.max(self.static):.4f} N·m > 마찰 여유 tau_fric_max {p.fric_max} N·m")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, msg):
        self.wz, self.wz_t = float(msg.angular_velocity.z), self.now_s()

    def on_wheel(self, msg):
        now = self.now_s()
        dt = now - self.omega_t if self.omega_t is not None else 0.0
        self.omega = np.array(msg.velocity, float)
        self.omega_t = now
        if self.guard is None or not np.all(np.isfinite(self.omega)):
            return
        if self.wz_t is None or now - self.wz_t > 0.1:      # 자이로가 없으면 감지하지 않고 g 를 되돌린다
            self.guard.reset()
            return
        if np.all(np.abs(self.omega) < 0.05):               # 서 있으면 자이로 바이어스를 지수 평균으로
            self.bias_n = min(self.bias_n + 1, 200)
            self.gyro_bias += (self.wz - self.gyro_bias) / self.bias_n
        gains = self.guard.update(self.omega, self.wz - self.gyro_bias, self.tau_net, dt)
        m = Float64MultiArray()
        m.data = [now, *self.guard.sigma.tolist(), *gains.tolist(), float(self.guard.flagged)]
        self.pub_guard.publish(m)

    def on_reference(self, msg):
        rows = np.array([[r.vx, r.vy, r.yaw_rate] for r in msg.references], float).reshape(-1, 3)
        self.ref = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, float(msg.dt), rows)

    def on_cmd(self, msg):
        if msg.mode != WheelTorqueCommand.MODE_TORQUE:
            if self.guard is not None:                  # 추종이 끊겼다 — 다음 무장은 g = 1 에서
                self.guard.reset()
                self.tau_net = np.zeros(4)
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
        wheel_ok = self.omega_t is not None and self.now_s() - self.omega_t <= self.wheel_timeout
        if not wheel_ok:
            omega = np.zeros(4)
            if self.has_friction or self.has_static:
                self.get_logger().warning("/wheel_states 끊김 — 마찰 보상 없이 낸다", throttle_duration_sec=2.0)
        # 정지 마찰 항은 ω ≈ 0 에서 최대라, 바퀴 속도를 모를 때(0 으로 둔 때) 켜면 도는 바퀴에도 더해진다 → 끈다
        omega_ref = (reference_wheel_speed(self.ref, self.now_s(), self.Tinv, self.ref_timeout)
                     if self.has_static and wheel_ok else None)
        tau_net = np.asarray(msg.torque, float)
        if self.guard is not None:
            self.tau_net = tau_net.copy()               # 감지의 토크 방향은 MPC 가 원한 값으로
            tau_net = self.guard.g * tau_net
        tau, clipped = motor_command(tau_net, omega, self.coulomb, self.viscous, self.smooth, self.limit,
                                     omega_ref, self.static, self.stribeck)
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

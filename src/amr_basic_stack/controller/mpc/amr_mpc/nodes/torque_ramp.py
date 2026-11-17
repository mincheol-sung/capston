#!/usr/bin/env python3
# torque_ramp — 바닥 토크 램프 시험 (2026-11-17). MPC 자리에 들어가 τ* 를 낸다.
#   ros2 launch amr_mpc torque_ramp.launch.py dir:=fwd mu_hat:=0.6       (실차 한 터미널, amrramp)
# ---------------------------------------------------------------
# 경로 (amrall use_mpc:=true 와 같고 torque_mpc_node 만 이 노드로 바뀐다)
#   torque_ramp_node ─/torque_mpc/cmd_raw (τ*)─▶ torque_output (+coulomb·viscous·static 보상, ±1.3 클램프)
#                    ─▶ 게이트('a' 무장) ─▶ /wheel_torque_cmd ─▶ ak40_can_node ─▶ /wheel_states (torque = 드라이버 상전류 추정)
#
# τ* = τ_g + J_w·T⁻¹·M_b⁻¹·(T⁻¹)ᵀ·τ_g          M_b = diag(m, m, I_z)  (몸체만)
#   τ_g = min(rate·t, tau_max)·pattern      바퀴–바닥 토크 (램프하는 것은 이것뿐)
#   pattern = pinv((T⁻¹)ᵀ)·e 를 최대 1 로   fwd: e = [1,0,0] → [1,1,1,1],  left: e = [0,1,0] → [−1,1,1,−1]
#   두 번째 항 = 바퀴 관성 feedforward. 무슬립 가정에서 몸체 가속 a = M_b⁻¹(T⁻¹)ᵀτ_g 를 내려면 바퀴 각가속 T⁻¹a 만큼
#   J_w 를 더 돌려야 한다. MPC 가 M = diag(m + m_w, …) (m_w = 4J_w/r²) 로 푸는 것과 같은 관성 몫이다:
#   (T⁻¹)ᵀτ* = (m + m_w)·a. 앞·옆은 τ* = (1 + m_w/m)·τ_g = 1.488·τ_g (m 4.56, m_w 2.225).
#   → τ_g > 1.3/1.488 = 0.874 N·m 부터는 torque_output 클램프 ±1.3 에 걸린다 (그 위는 명령도 1.3 에서 멈춤).
#   inertia_ff:=false 면 τ* = τ_g.
#
# 기울기 rate (rate:=0 이면 자동) — 램프가 tau_max 에 닿는 순간 이동 거리 = distance 가 되게 이분법으로 정한다.
#   무슬립:  a = 4τ_g/(r m)                          (관성 feedforward 덕에 m 만)
#   슬립:    a = μ̂·g/√2                              (SlipPlant 롤러 모델: 바퀴마다 롤러 수직 방향 μ̂·mg/4, 앞·옆 같음)
#   속도 상한 v ≤ ω_max·r                            (정격 370 rpm = 38.7 rad/s → 1.55 m/s, 이 위로는 모터가 토크를 못 냄)
#   a = min(무슬립, 슬립), v 가 상한이면 a = 0. 1 ms Euler.
#
# 무장('a') 할 때마다 한 판. 끝나는 조건: τ_g 가 tau_max 에 닿음 / 출발점에서 stop_distance 넘음 / 시간 초과 /
#   무장 해제(s·fault_monitor). 끝나면 cmd_raw 에 DAMPING, /mpc/arm false.
# 판마다 CSV (log_dir): 명령(τ_g, τ*, 모터 명령 /wheel_torque_cmd)과 피드백(/wheel_states torque·velocity), 추정기 상태.
# ---------------------------------------------------------------
import csv
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from amr_interfaces.msg import PlanarReference, PlanarReferenceArray, WheelStates, WheelTorqueCommand
from amr_mpc.core import config, node_main
from amr_mpc.torque.model import TorquePlant

DIRS = {"fwd": np.array([1.0, 0.0, 0.0]), "left": np.array([0.0, 1.0, 0.0])}


def ramp_pattern(p, e):
    """몸체 방향 e (3,) → 바퀴 토크 패턴 (4,), 최대 |·| = 1."""
    pat = np.linalg.pinv(p.Tinv.T) @ e
    return pat / np.max(np.abs(pat))


def inertia_gain(p):
    """τ* = (I + K)·τ_g 의 K = J_w·T⁻¹·M_b⁻¹·(T⁻¹)ᵀ (4×4)."""
    Mb_inv = np.diag([1.0 / p.mass, 1.0 / p.mass, 1.0 / p.inertia_z])
    return p.J_w * p.Tinv @ Mb_inv @ p.Tinv.T


def ramp_distance(rate, p, mu, tau_max, omega_max, tau_limit, inertia_ff=True, dt=1e-3):
    """램프 0 → tau_max (기울기 rate) 동안 이동 거리 [m] — 위 머리말의 1차원 모델."""
    m_w = 4.0 * p.J_w / p.r ** 2
    a_slip = mu * p.g / math.sqrt(2.0)
    v_max = omega_max * p.r
    T = tau_max / rate
    t = x = v = 0.0
    while t < T:
        tau_g = rate * t
        tau_s = min(tau_g * (1.0 + m_w / p.mass) if inertia_ff else tau_g, tau_limit)
        a_ns = 4.0 * tau_s / (p.r * (p.mass + m_w))
        a = 0.0 if v >= v_max else min(a_ns, a_slip)
        v = min(v + a * dt, v_max)
        x += v * dt
        t += dt
    return x


def rate_for_distance(dist, p, mu, tau_max, omega_max, tau_limit, inertia_ff=True):
    """ramp_distance(rate) = dist 인 rate [N·m/s] (거리는 rate 에 대해 감소). 로그 이분법."""
    lo, hi = 0.01, 20.0
    for _ in range(40):
        mid = math.sqrt(lo * hi)
        if ramp_distance(mid, p, mu, tau_max, omega_max, tau_limit, inertia_ff) > dist:
            lo = mid
        else:
            hi = mid
    return math.sqrt(lo * hi)


class TorqueRampNode(Node):
    def __init__(self):
        super().__init__("torque_ramp_node")
        p = TorquePlant.from_yaml()
        plant_yaml = config.load("torque_plant.yaml")
        rated = float(plant_yaml["motor"]["rated_speed_rpm"]) * 2.0 * math.pi / 60.0
        self.declare_parameters("", [
            ("direction", "fwd"),          # fwd | left
            ("tau_max", 1.0),              # 바퀴–바닥 토크 끝값 [N·m]
            ("rate", 0.0),                 # [N·m/s], 0 = distance·mu_hat 로 자동
            ("distance", 3.0),             # 자동 기울기의 목표 거리 [m]
            ("mu_hat", 0.6),               # 자동 기울기 계산에만 쓴다
            ("omega_max", rated),          # 자동 기울기 계산의 바퀴 속도 상한 [rad/s]
            ("stop_distance", 0.0),        # 출발점에서 이만큼이면 끊는다 [m], 0 = 1.15·distance
            ("inertia_ff", True),
            ("tau_limit", p.tau_rated),    # torque_output 클램프 (기울기 계산에만)
            ("rate_hz", 100.0),
            ("state_topic", "/state_estimator/odom"),
            ("log_dir", ""),               # 비우면 CSV 안 남김
        ])
        g = lambda n: self.get_parameter(n).value
        key = str(g("direction"))
        if key not in DIRS:
            raise ValueError(f"direction {key!r} — fwd 또는 left")
        self.key, self.e = key, DIRS[key]
        self.p = p
        self.pat = ramp_pattern(p, self.e)
        self.inertia_ff = bool(g("inertia_ff"))
        self.K = inertia_gain(p) if self.inertia_ff else np.zeros((4, 4))
        self.tau_max = float(g("tau_max"))
        if not 0.0 < self.tau_max <= 1.3:
            raise ValueError(f"tau_max {self.tau_max} — 0~1.3 N·m")
        self.distance = float(g("distance"))
        self.mu_hat = float(g("mu_hat"))
        self.rate = float(g("rate")) or rate_for_distance(self.distance, p, self.mu_hat, self.tau_max,
                                                           float(g("omega_max")), float(g("tau_limit")), self.inertia_ff)
        if not 0.01 <= self.rate <= 5.0:
            raise ValueError(f"rate {self.rate:.3f} N·m/s — 0.01~5 밖")
        self.T = self.tau_max / self.rate
        self.stop_dist = float(g("stop_distance")) or 1.15 * self.distance
        self.dt = 1.0 / float(g("rate_hz"))
        self.log_dir = os.path.expanduser(str(g("log_dir")))
        self.fk = np.linalg.pinv(p.Tinv)                  # 바퀴속도 → 몸체속도

        self.armed = False
        self.run = None                                   # 진행 중인 판 dict
        self.n_run = 0
        self.pose = None                                  # (x, y, yaw, vx_body, vy_body, wz, 받은 시각)
        self.ws = None                                    # 최근 WheelStates
        self.tau_motor = np.full(4, np.nan)               # 최근 /wheel_torque_cmd

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(WheelTorqueCommand, "/torque_mpc/cmd_raw", 10)
        self.pub_ref = self.create_publisher(PlanarReferenceArray, "/dra_mppi/reference_trajectory", 5)
        self.pub_arm = self.create_publisher(Bool, "/mpc/arm", 10)
        self.create_subscription(Bool, "/mpc/armed", self.on_armed, latched)
        self.create_subscription(Odometry, str(g("state_topic")), self.on_state, 10)
        self.create_subscription(WheelStates, "/wheel_states", self.on_wheels, 10)
        self.create_subscription(WheelTorqueCommand, "/wheel_torque_cmd", self.on_motor_cmd, 10)
        self.create_timer(self.dt, self.tick)

        ramp_only = ramp_distance(self.rate, p, self.mu_hat, self.tau_max, float(g("omega_max")),
                                  float(g("tau_limit")), self.inertia_ff)
        m_w = 4.0 * p.J_w / p.r ** 2
        self.get_logger().info(
            f"torque_ramp: {key} 패턴 {np.round(self.pat, 3)}, τ_g 0 → {self.tau_max} N·m, 기울기 {self.rate:.3f} N·m/s "
            f"({'지정' if float(g('rate')) else f'자동: μ̂ {self.mu_hat}, 목표 {self.distance} m'}) → {self.T:.2f} s, "
            f"모델 예상 이동 {ramp_only:.2f} m.  관성 ff {'켬 τ* = (I+K)τ_g, 앞·옆 ×%.3f' % (1 + m_w / p.mass) if self.inertia_ff else '끔'}"
            f".  끊는 거리 {self.stop_dist:.2f} m.  'a' = 한 판 시작")

    # ── 구독
    def on_armed(self, m):
        armed = bool(m.data)
        if armed and not self.armed:
            self.start()
        elif not armed and self.armed and self.run is not None:
            self.finish("무장 해제 (s·fault_monitor)", disarm=False)
        self.armed = armed

    def on_state(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tw = m.twist.twist
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z,
                     time.monotonic())

    def on_wheels(self, m):
        self.ws = m

    def on_motor_cmd(self, m):
        self.tau_motor = (np.array(m.torque, float) if m.mode == WheelTorqueCommand.MODE_TORQUE
                          else np.full(4, np.nan))

    # ── 판
    def start(self):
        if self.pose is None or time.monotonic() - self.pose[6] > 0.3:
            self.get_logger().error("추정기 상태(/state_estimator/odom)가 없다 — 시작 안 함, 무장 해제")
            self.pub_arm.publish(Bool(data=False))
            return
        self.n_run += 1
        f = w = None
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            path = os.path.join(self.log_dir, f"ramp_{self.key}_{time.strftime('%H%M%S')}_run{self.n_run}.csv")
            f = open(path, "w", newline="")
            w = csv.writer(f)
            w.writerow(["t", "phase", "tau_g", *[f"tau_star_{i}" for i in "FL FR RL RR".split()],
                        *[f"tau_motor_{i}" for i in "FL FR RL RR".split()], *[f"tau_fb_{i}" for i in "FL FR RL RR".split()],
                        *[f"omega_{i}" for i in "FL FR RL RR".split()], "x", "y", "yaw", "vx_body", "vy_body", "wz",
                        "dist", "v_wheel_dir", "v_est_dir"])
            self.get_logger().info(f"[run {self.n_run}] CSV {path}")
        self.run = {"t0": time.monotonic(), "p0": self.pose[:3], "f": f, "w": w, "tau_g": 0.0, "dist": 0.0,
                    "onset": None, "n_gap": 0, "rows": []}
        self.get_logger().info(f"[run {self.n_run}] 시작 — {self.key}, {self.rate:.3f} N·m/s")

    def finish(self, why, disarm=True):
        r = self.run
        self.run = None
        msg = WheelTorqueCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = WheelTorqueCommand.MODE_DAMPING
        self.pub.publish(msg)
        if disarm:
            self.pub_arm.publish(Bool(data=False))
        if r["f"]:
            r["f"].close()
        rows = np.array(r["rows"]) if r["rows"] else np.zeros((0, 6))
        txt = f"[run {self.n_run}] 끝: {why}. τ_g {r['tau_g']:.3f} N·m, 이동 {r['dist']:.2f} m"
        if r["onset"] is not None:
            tg = r["onset"]
            mu = 4.0 * math.sqrt(2.0) * tg / (self.p.r * self.p.mass * self.p.g)
            txt += (f".  바퀴–추정기 속도 차가 0.15 m/s 넘은 τ_g {tg:.3f} N·m → μ̂ (롤러 모델 √2·4τ_g/(r m g)) ≈ {mu:.3f}"
                    f" [참고 — 추정기 지연만큼 이르다]")
        if len(rows):
            ok = np.isfinite(rows[:, 1:5]).all(axis=1) & np.isfinite(rows[:, 5:9]).all(axis=1)
            if ok.any():
                d = rows[ok, 5:9] - rows[ok, 1:5]
                txt += (f"\n    피드백 − 모터 명령 [FL FR RL RR]: 평균 {np.round(d.mean(0), 3)}, "
                        f"최소 {np.round(d.min(0), 3)} N·m (|명령| 최대 {np.round(np.nanmax(np.abs(rows[:, 1:5]), 0), 3)})")
        self.get_logger().info(txt)

    def tick(self):
        r = self.run
        if r is None or not self.armed:
            return
        now = time.monotonic()
        t = now - r["t0"]
        tau_g = min(self.rate * t, self.tau_max)
        r["tau_g"] = tau_g
        if self.pose is not None:
            r["dist"] = math.hypot(self.pose[0] - r["p0"][0], self.pose[1] - r["p0"][1])
        if tau_g >= self.tau_max:
            return self.finish(f"τ_g {self.tau_max} N·m 도달 ({t:.2f} s)")
        if r["dist"] >= self.stop_dist:
            return self.finish(f"이동 {r['dist']:.2f} m ≥ {self.stop_dist:.2f} m")
        if self.pose is None or now - self.pose[6] > 0.3:
            return self.finish("추정기 상태 끊김")

        tg = tau_g * self.pat
        tau_star = tg + self.K @ tg
        stamp = self.get_clock().now().to_msg()
        msg = WheelTorqueCommand()
        msg.header.stamp = stamp
        msg.mode = WheelTorqueCommand.MODE_TORQUE
        msg.torque = [float(v) for v in tau_star]
        self.pub.publish(msg)

        # 정지 마찰 보상(torque_output)이 방향을 읽는 기준열 — 방향만 맞으면 된다 (0.3 m/s 면 ω_ref 7.5 rad/s, tanh ≈ 1)
        ref = PlanarReferenceArray()
        ref.header.stamp, ref.header.frame_id, ref.dt = stamp, "map", 0.1
        for _ in range(5):
            pr = PlanarReference()
            pr.header = ref.header
            pr.vx, pr.vy = 0.3 * float(self.e[0]), 0.3 * float(self.e[1])
            ref.references.append(pr)
        self.pub_ref.publish(ref)

        ws = self.ws
        fb = np.array(ws.torque, float) if ws is not None else np.full(4, np.nan)
        om = np.array(ws.velocity, float) if ws is not None else np.full(4, np.nan)
        v_wheel = float((self.fk @ om) @ self.e) if ws is not None else float("nan")
        v_est = float(np.array(self.pose[3:6]) @ self.e)
        if r["onset"] is None and np.isfinite(v_wheel):
            r["n_gap"] = r["n_gap"] + 1 if v_wheel - v_est > 0.15 else 0
            if r["n_gap"] >= 5:
                r["onset"] = max(0.0, tau_g - 4 * self.dt * self.rate)
        r["rows"].append([t, *self.tau_motor, *fb])
        if r["w"]:
            r["w"].writerow([f"{t:.4f}", "ramp", f"{tau_g:.5f}", *[f"{v:.5f}" for v in tau_star],
                             *[f"{v:.5f}" for v in self.tau_motor], *[f"{v:.5f}" for v in fb], *[f"{v:.4f}" for v in om],
                             *[f"{v:.4f}" for v in self.pose[:6]], f"{r['dist']:.4f}", f"{v_wheel:.4f}", f"{v_est:.4f}"])


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TorqueRampNode()
    try:
        node_main.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

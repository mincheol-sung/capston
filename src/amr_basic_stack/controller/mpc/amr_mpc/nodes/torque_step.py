#!/usr/bin/env python3
# torque_step — 바닥 토크 스텝 시험 (2026-11-17). torque_ramp 와 같은 자리(MPC 대신)에서 τ_g 를 처음부터 일정하게 준다.
#   ros2 launch amr_mpc torque_step.launch.py dir:=fwd tau:=0.3 distance:=2.0       (실차 한 터미널, amrstep)
# ---------------------------------------------------------------
# 경로: torque_step_node ─/torque_mpc/cmd_raw─▶ torque_output (마찰 보상, ±1.3 클램프) ─▶ 게이트 ─▶ CAN   (amrall 과 같음)
#
# τ* = (I + K)·τ_g,  τ_g = tau·pattern  ('a' 누른 순간부터 끝날 때까지 일정)
#   pattern·K 는 torque_ramp 와 같다 (fwd [1,1,1,1], left [−1,1,1,−1], 관성 ff 앞·옆 ×1.488).
#   → 모터 명령 ≈ 1.488·tau + 마찰 보상. tau > 0.874 N·m 면 1.3 클램프에 걸린다. inertia_ff:=false 면 τ* = tau.
# 끝나는 조건: 출발점에서 distance 이상 이동 (추정기 위치) / max_time 초과 / 추정기 끊김 / 무장 해제.
# 2026-11-25 제자리 회전 ccw·cw: 몸체 방향 e = [0, 0, ±1] → 패턴 [−1, +1, −1, +1]·(±1) (바퀴당 같은 크기).
#   끝나는 조건은 회전각 angle_deg (추정기 yaw 누적) — distance 는 회전 중 벗어남 한도 (안 주면 0.3 m).
#   정지 마찰 보상 방향용 기준은 yaw_rate 1.5 rad/s (바퀴 7.2 rad/s, 직진 0.3 m/s 의 7.5 와 비슷).
#   끝나면 cmd_raw 에 DAMPING, /mpc/arm false. 정지 거리(≈ v²/(2·μg/√2), 1.55 m/s·μ 0.6 이면 0.29 m)가 distance 뒤에 더해진다.
# 판마다 CSV (log_dir): torque_ramp 와 같은 열 (tau_g 는 일정).
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
from amr_mpc.core import node_main
from amr_mpc.nodes.torque_ramp import DIRS, inertia_gain, ramp_pattern

ROT_DIRS = {"ccw": np.array([0.0, 0.0, 1.0]), "cw": np.array([0.0, 0.0, -1.0])}   # 2026-11-25 제자리 회전
STEP_DIRS = {**DIRS, **ROT_DIRS}
ROT_DRIFT_M = 0.3            # 회전 판에서 distance 를 안 주면 쓰는 벗어남 한도 [m]
from amr_mpc.torque.model import TorquePlant

WHEELS = "FL FR RL RR".split()


class TorqueStepNode(Node):
    def __init__(self):
        super().__init__("torque_step_node")
        p = TorquePlant.from_yaml()
        self.declare_parameters("", [
            ("direction", "fwd"),          # fwd | back | left | right | ccw | cw
            ("tau", 0.0),                  # 바퀴–바닥 토크 τ_g [N·m] — 꼭 준다
            ("distance", 0.0),             # 끊는 이동 거리 [m] — 병진이면 꼭 준다 (회전이면 벗어남 한도, 0 = 0.3 m)
            ("angle_deg", 0.0),            # 회전 판: 끊는 회전각 [°] — ccw·cw 면 꼭 준다
            ("max_time", 5.0),             # [s]
            ("inertia_ff", True),
            ("rate_hz", 100.0),
            ("state_topic", "/state_estimator/odom"),
            ("log_dir", ""),
        ])
        g = lambda n: self.get_parameter(n).value
        self.key = str(g("direction"))
        if self.key not in STEP_DIRS:
            raise ValueError(f"direction {self.key!r} — fwd, back, left, right, ccw, cw")
        self.rot = self.key in ROT_DIRS
        self.tau = float(g("tau"))
        self.distance = float(g("distance"))
        self.angle = math.radians(float(g("angle_deg")))
        self.max_time = float(g("max_time"))
        if not 0.0 < self.tau <= 1.3:
            raise ValueError(f"tau {self.tau} — 0 < tau ≤ 1.3 N·m 로 준다 (tau:=)")
        if self.rot:
            if not 0.0 < self.angle <= 2.0 * math.pi:
                raise ValueError(f"angle_deg {g('angle_deg')} — 회전 판은 0 < angle_deg ≤ 360 로 준다 (angle:=)")
            if self.distance <= 0.0:
                self.distance = ROT_DRIFT_M
        if not 0.0 < self.distance <= 6.0:
            raise ValueError(f"distance {self.distance} — 0 < distance ≤ 6 m 로 준다 (distance:=)")
        self.e = STEP_DIRS[self.key]
        self.p = p
        tg = self.tau * ramp_pattern(p, self.e)
        K = inertia_gain(p) if bool(g("inertia_ff")) else np.zeros((4, 4))
        self.tau_g, self.tau_star = tg, tg + K @ tg
        self.dt = 1.0 / float(g("rate_hz"))
        self.log_dir = os.path.expanduser(str(g("log_dir")))
        self.fk = np.linalg.pinv(p.Tinv)

        self.armed, self.run, self.n_run = False, None, 0
        self.pose = self.ws = None
        self.tau_motor = np.full(4, np.nan)
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(WheelTorqueCommand, "/torque_mpc/cmd_raw", 10)
        self.pub_ref = self.create_publisher(PlanarReferenceArray, "/dra_mppi/reference_trajectory", 5)
        self.pub_arm = self.create_publisher(Bool, "/mpc/arm", 10)
        self.create_subscription(Bool, "/mpc/armed", self.on_armed, latched)
        self.create_subscription(Odometry, str(g("state_topic")), self.on_state, 10)
        self.create_subscription(WheelStates, "/wheel_states", lambda m: setattr(self, "ws", m), 10)
        self.create_subscription(WheelTorqueCommand, "/wheel_torque_cmd", self.on_motor_cmd, 10)
        self.create_timer(self.dt, self.tick)
        clip = " — 1.3 클램프에 걸림" if np.max(np.abs(self.tau_star)) > p.tau_rated else ""
        end = (f"끊는 회전각 {math.degrees(self.angle):g}° (벗어남 {self.distance} m 넘으면 끊음)" if self.rot
               else f"끊는 거리 {self.distance} m")
        self.get_logger().info(
            f"torque_step: {self.key}, τ_g {np.round(self.tau_g, 3)} N·m 일정 → τ* {np.round(self.tau_star, 3)}{clip} "
            f"(+ torque_output 마찰 보상).  {end}, 최대 {self.max_time} s.  'a' = 한 판 시작")

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

    def on_motor_cmd(self, m):
        self.tau_motor = (np.array(m.torque, float) if m.mode == WheelTorqueCommand.MODE_TORQUE
                          else np.full(4, np.nan))

    def start(self):
        if self.pose is None or time.monotonic() - self.pose[6] > 0.3:
            self.get_logger().error("추정기 상태(/state_estimator/odom)가 없다 — 시작 안 함, 무장 해제")
            self.pub_arm.publish(Bool(data=False))
            return
        self.n_run += 1
        f = w = None
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            path = os.path.join(self.log_dir, f"step_{self.key}_{self.tau:g}Nm_{time.strftime('%H%M%S')}_run{self.n_run}.csv")
            f = open(path, "w", newline="")
            w = csv.writer(f)
            w.writerow(["t", "phase", "tau_g", *[f"tau_star_{i}" for i in WHEELS], *[f"tau_motor_{i}" for i in WHEELS],
                        *[f"tau_fb_{i}" for i in WHEELS], *[f"omega_{i}" for i in WHEELS],
                        "x", "y", "yaw", "vx_body", "vy_body", "wz", "dist", "v_wheel_dir", "v_est_dir", "dyaw"])
            self.get_logger().info(f"[run {self.n_run}] CSV {path}")
        self.run = {"t0": time.monotonic(), "p0": self.pose[:3], "f": f, "w": w, "dist": 0.0, "rows": [], "vmax": 0.0,
                    "yaw_prev": self.pose[2], "dyaw": 0.0}
        self.get_logger().info(f"[run {self.n_run}] 시작 — {self.key}, τ_g {self.tau} N·m")

    def finish(self, why, disarm=True):
        r, self.run = self.run, None
        msg = WheelTorqueCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = WheelTorqueCommand.MODE_DAMPING
        self.pub.publish(msg)
        if disarm:
            self.pub_arm.publish(Bool(data=False))
        if r["f"]:
            r["f"].close()
        txt = (f"[run {self.n_run}] 끝: {why}. 회전 {math.degrees(r['dyaw']):+.1f}°, 벗어남 {r['dist']:.3f} m, 추정기 최대 회전 속도 {r['vmax']:.2f} rad/s"
               if self.rot else f"[run {self.n_run}] 끝: {why}. 이동 {r['dist']:.2f} m, 추정기 최대 속도 {r['vmax']:.2f} m/s")
        rows = np.array(r["rows"]) if r["rows"] else np.zeros((0, 13))
        ok = np.isfinite(rows[:, 1:9]).all(axis=1) if len(rows) else np.zeros(0, bool)
        if ok.any():
            R = rows[ok]
            d = R[:, 5:9] - R[:, 1:5]
            txt += (f"\n    피드백 − 모터 명령 [FL FR RL RR]: 평균 {np.round(d.mean(0), 3)}, 최소 {np.round(d.min(0), 3)} N·m"
                    f"   |ω| 최대 {np.round(np.nanmax(np.abs(R[:, 9:13]), 0), 1)} rad/s")
        self.get_logger().info(txt)

    def tick(self):
        r = self.run
        if r is None or not self.armed:
            return
        now = time.monotonic()
        t = now - r["t0"]
        if self.pose is not None:
            r["dist"] = math.hypot(self.pose[0] - r["p0"][0], self.pose[1] - r["p0"][1])
            dy = self.pose[2] - r["yaw_prev"]
            r["dyaw"] += math.atan2(math.sin(dy), math.cos(dy))           # 누적 회전 (±π 넘어감 처리)
            r["yaw_prev"] = self.pose[2]
        if self.rot and abs(r["dyaw"]) >= self.angle:
            return self.finish(f"회전 {math.degrees(r['dyaw']):+.1f}° ≥ {math.degrees(self.angle):g}° ({t:.2f} s)")
        if r["dist"] >= self.distance:
            return self.finish(f"{'벗어남' if self.rot else '이동'} {r['dist']:.2f} m ≥ {self.distance} m ({t:.2f} s)")
        if t >= self.max_time:
            return self.finish(f"시간 {self.max_time} s")
        if self.pose is None or now - self.pose[6] > 0.3:
            return self.finish("추정기 상태 끊김")

        stamp = self.get_clock().now().to_msg()
        msg = WheelTorqueCommand()
        msg.header.stamp = stamp
        msg.mode = WheelTorqueCommand.MODE_TORQUE
        msg.torque = [float(v) for v in self.tau_star]
        self.pub.publish(msg)
        ref = PlanarReferenceArray()                       # 정지 마찰 보상 방향용 (torque_ramp 와 같음)
        ref.header.stamp, ref.header.frame_id, ref.dt = stamp, "map", 0.1
        for _ in range(5):
            pr = PlanarReference()
            pr.header = ref.header
            pr.vx, pr.vy, pr.yaw_rate = 0.3 * float(self.e[0]), 0.3 * float(self.e[1]), 1.5 * float(self.e[2])
            ref.references.append(pr)
        self.pub_ref.publish(ref)

        ws = self.ws
        fb = np.array(ws.torque, float) if ws is not None else np.full(4, np.nan)
        om = np.array(ws.velocity, float) if ws is not None else np.full(4, np.nan)
        v_wheel = float((self.fk @ om) @ self.e) if ws is not None else float("nan")
        v_est = float(np.array(self.pose[3:6]) @ self.e)
        r["vmax"] = max(r["vmax"], v_est)
        r["rows"].append([t, *self.tau_motor, *fb, *om])
        if r["w"]:
            r["w"].writerow([f"{t:.4f}", "step", f"{self.tau:.5f}", *[f"{v:.5f}" for v in self.tau_star],
                             *[f"{v:.5f}" for v in self.tau_motor], *[f"{v:.5f}" for v in fb], *[f"{v:.4f}" for v in om],
                             *[f"{v:.4f}" for v in self.pose[:6]], f"{r['dist']:.4f}", f"{v_wheel:.4f}", f"{v_est:.4f}",
                             f"{r['dyaw']:.4f}"])


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TorqueStepNode()
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

#!/usr/bin/env python3
# 구조 B — 바퀴 각속도 입력 MPC 노드.  기존 mpc_tracking_node(몸체속도 출력)는 그대로 두고 병행한다.
# ---------------------------------------------------------------
#   /dra_mppi/reference_trajectory   PlanarReferenceArray (10 Hz, dt 0.2 × 20)
#   odom_topic (기본 /odom)           pose X=[x,y,yaw]   ← 상태 원천
#     ★ 실주행(autonomy.launch.py)에서는 /odometry/filtered 로 덮어쓴다. 선택이 아니라 필수다 —
#       플래너(dra_mppi.yaml:6 odometry_topic)가 /odometry/filtered 로 기준열을 만들므로,
#       여기서 /odom 을 쓰면 X_e = X_r − X 가 서로 다른 적분 경로의 두 포즈를 빼게 된다.
#       2026-09-09 실측 (bags/mpc_chain_20260909_151007, 416 s): 같은 odom frame 인데도
#       두 포즈가 위치 62~222 cm, 누적 yaw 1127° 벌어졌다 (메카넘 휠 오도메트리 yaw 대 IMU 융합).
#       그 차이가 그대로 추종오차로 들어간다.
#     ★ θ_d 재측정은 필요 없다 (예전 주석이 시켰지만 근거를 재보니 아니다).
#       plant_wheel.yaml 의 model.theta 는 **바퀴속도** 경로(ω_cmd → /measured_wheel_speeds)의
#       dead time 이고, 그 경로는 odom_topic 과 무관하다. 포즈 경로에는 지연 상태가 아예 없다
#       (wheel/model.py plant() 의 X_e 행은 A_k, B_k 기구학 야코비안뿐).
#       측정해도 EKF 가 포즈에 더하는 지연은 x 증분 20 ms, yaw 증분 0 ms 로 Ts=40 ms 의 0.5 스텝 —
#       정수 d 로 표현할 수 있는 크기가 아니다.
#     기본값을 /odom 으로 두는 것은 자기시험(wheel_circle.launch.py + sim_plant_wheel)용이다.
#   /measured_wheel_speeds           Float32MultiArray [FL, FR, RL, RR] rad/s  ← 드라이버 encoder
#      ↓ ReferenceHorizon 보간 → WheelMPC
#   /wheel_cmd                       Float32MultiArray [FL, FR, RL, RR] rad/s  → 드라이버 (T⁻¹ 안 거침)
#   /wheel_mpc/reference_horizon, /wheel_mpc/status
#
# 안전: reference / odom / wheel 끊김 → 0 출력 + 리셋.  /cmd_vel 발행자가 있으면 경고 (드라이버 명령 경쟁).
# hold:  플래너 정지 기준열(속도 0, 포즈 한 점) + 오차가 hold.enter 안 → 명령 0 + MPC 리셋 (HoldLatch).
#        exit 를 넘으면 추종 재개. 도착 후 잡음을 쫓는 stick-slip 을 막는다 (core/reference.py 주석).
# ---------------------------------------------------------------
import csv, math, signal, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from amr_interfaces.msg import PlanarReferenceArray

from amr_mpc.wheel import model as wm
from amr_mpc.wheel.mpc import WheelMPC
from amr_mpc.core.reference import ReferenceHorizon, HoldLatch, wrap

_S = wm.CFG["safety"]; _L = wm.CFG["limits"]; _H = wm.CFG["hold"]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sec(st):
    return st.sec + st.nanosec * 1e-9


class WheelTrackingNode(Node):
    def __init__(self):
        super().__init__("wheel_mpc_node")
        self.declare_parameters("", [
            ("reference_topic", "/dra_mppi/reference_trajectory"),
            ("odom_topic", "/odom"),
            ("wheel_topic", "/measured_wheel_speeds"),
            ("wheel_cmd_topic", "/wheel_cmd"),
            ("ref_timeout", float(_S["ref_timeout"])), ("odom_timeout", float(_S["odom_timeout"])),
            ("wheel_timeout", float(_S.get("wheel_timeout", 0.3))),
            ("load_budget", float(_L["load_budget"])),
            ("publish_debug", True), ("log_prefix", ""),
            ("hold_enter_xy", float(_H["enter_xy"])), ("hold_enter_yaw_deg", float(_H["enter_yaw_deg"])),
            ("hold_exit_xy", float(_H["exit_xy"])), ("hold_exit_yaw_deg", float(_H["exit_yaw_deg"])),
            ("hold_stopped_wheel", float(_H["stopped_wheel"])),
        ])
        g = lambda n: self.get_parameter(n).value
        self.mpc = WheelMPC(); self.Ts, self.Np = self.mpc.Ts, self.mpc.Np
        self.ref_timeout, self.odom_timeout, self.wheel_timeout = float(g("ref_timeout")), float(g("odom_timeout")), float(g("wheel_timeout"))
        self.budget = float(g("load_budget"))
        self.X = None; self.last_odom = None; self.odom_frame = None
        self.w = None; self.last_wheel = None
        self.ref = None; self.ref_rx = None; self.ref_frame = None
        self.w_prev = np.zeros(4); self.active = False; self.done = False
        self.latch = HoldLatch(g("hold_enter_xy"), math.radians(g("hold_enter_yaw_deg")),
                               g("hold_exit_xy"), math.radians(g("hold_exit_yaw_deg")))
        self.hold_stopped_wheel = float(g("hold_stopped_wheel"))
        self.rows = []; self.csv_path = f"{g('log_prefix')}_{time.strftime('%Y%m%d_%H%M%S')}.csv" if g("log_prefix") else None
        self.debug = bool(g("publish_debug"))
        self.pub = self.create_publisher(Float32MultiArray, str(g("wheel_cmd_topic")), 10)
        self.pub_path = self.create_publisher(Path, "/wheel_mpc/reference_horizon", 1)
        self.pub_status = self.create_publisher(Float64MultiArray, "/wheel_mpc/status", 10)
        self.create_subscription(Odometry, str(g("odom_topic")), self.cb_odom, 20)
        self.create_subscription(Float32MultiArray, str(g("wheel_topic")), self.cb_wheel, 20)
        self.create_subscription(PlanarReferenceArray, str(g("reference_topic")), self.cb_ref, 5)
        self.timer = self.create_timer(self.Ts, self.on_tick)
        self.create_timer(2.0, self.check_competition)
        self.get_logger().info(f"구조 B (바퀴 입력).  기준 {g('reference_topic')}  포즈 {g('odom_topic')}  바퀴 {g('wheel_topic')} → {g('wheel_cmd_topic')}"
                               f"   Ts {self.Ts*1000:.0f} ms  Np {self.Np}  Nc {self.mpc.Nc}  |ω| ≤ {wm.W_MAX:.0f} rad/s")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_odom(self, m):
        p = m.pose.pose
        self.X = np.array([p.position.x, p.position.y, yaw_of(p.orientation)]); self.last_odom = self.now(); self.odom_frame = m.header.frame_id

    def cb_wheel(self, m):
        if len(m.data) != 4:
            self.get_logger().warn(f"measured_wheel_speeds 길이 {len(m.data)} ≠ 4", throttle_duration_sec=2.0); return
        self.w = np.array(m.data, float); self.last_wheel = self.now()

    def cb_ref(self, m):
        if not m.references:
            return
        rows = [(r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate) for r in m.references]
        try:
            self.ref = ReferenceHorizon.from_rows(sec(m.header.stamp), m.dt, rows)
        except ValueError as e:
            self.get_logger().warn(f"기준열 무시: {e}", throttle_duration_sec=2.0); return
        self.ref_rx = self.now(); self.ref_frame = m.header.frame_id

    def check_competition(self):
        n = self.count_publishers("/cmd_vel")
        if n > 0 and self.active:
            self.get_logger().error(f"/cmd_vel 발행자 {n}개 — 드라이버가 두 명령을 받는다. wheel_cmd 와 경쟁", throttle_duration_sec=10.0)

    def stop(self, reason):
        self.pub.publish(Float32MultiArray(data=[0.0] * 4))
        if self.active:
            self.get_logger().info(f"정지: {reason}")
        self.active = False; self.w_prev = np.zeros(4); self.mpc.reset()

    def on_tick(self):
        now = self.now()
        if self.X is None or now - self.last_odom > self.odom_timeout:
            self.stop("odom 없음"); self.get_logger().warn("odom 대기/끊김", throttle_duration_sec=2.0); return
        if self.w is None or now - self.last_wheel > self.wheel_timeout:
            self.stop("wheel 없음"); self.get_logger().warn("measured_wheel_speeds 대기/끊김", throttle_duration_sec=2.0); return
        if self.ref is None:
            self.stop("기준 없음"); return
        if now - self.ref_rx > self.ref_timeout:
            self.stop("기준열 끊김"); self.get_logger().warn(f"기준열 {now-self.ref_rx:.2f} s 없음", throttle_duration_sec=2.0); return
        if not self.active:
            self.active = True; self.mpc.reset(); self.w_prev = np.zeros(4)
            self.get_logger().info(f"추종 시작. 기준열 {self.ref.T}×{self.ref.dt:.2f} s")
        hz = self.ref.horizon(now, self.Ts, self.Np); Xr, ur = hz[0]
        Xe = Xr - self.X; Xe[2] = wrap(Xe[2])
        was_idle = self.latch.idle
        if self.latch.update(self.ref.is_hold, Xe, stopped=bool(np.abs(self.w).max() < self.hold_stopped_wheel)):
            if not was_idle:
                self.get_logger().info(f"hold 진입: 오차 {np.hypot(Xe[0], Xe[1])*1000:.1f} mm, {math.degrees(Xe[2]):.2f}° — 명령 0, MPC 리셋")
                self.mpc.reset(); self.w_prev = np.zeros(4)
            self.pub.publish(Float32MultiArray(data=[0.0] * 4))
            if self.debug:
                self.pub_status.publish(Float64MultiArray(data=[now, *map(float, Xe), 0.0, 0.0, 0.0, 0.0, now - self.ref.t0]))
            return
        if was_idle:
            self.get_logger().info(f"hold 해제: 오차 {np.hypot(Xe[0], Xe[1])*1000:.1f} mm, {math.degrees(Xe[2]):.2f}° — 추종 재개")
        ref = [(Xr_i[2], ur_i[0], ur_i[1]) for Xr_i, ur_i in hz]
        t_c = time.perf_counter()
        wc = self.mpc.step(self.X, self.w, self.w_prev, ref, Xr, ur)
        ms = (time.perf_counter() - t_c) * 1000; self.w_prev = wc
        self.pub.publish(Float32MultiArray(data=[float(v) for v in wc]))
        w_r = wm.T_INV @ ur; load = np.abs(w_r).max() / wm.W_MAX
        if load > self.budget:
            self.get_logger().warn(f"기준 바퀴속도 {load*100:.0f} % > 예산 {self.budget*100:.0f} %", throttle_duration_sec=2.0)
        st = self.mpc.stats
        row = [round(now, 4), *self.X, *Xr, *Xe, *self.w, *wc, float(np.abs(wc).max() / wm.W_MAX),
               st["iters"], st["fallback"], round(ms, 2), round(now - self.ref.t0, 3)]
        if self.csv_path:
            self.rows.append(row)
        if self.debug:
            self.pub_status.publish(Float64MultiArray(data=[float(row[0]), *map(float, Xe), row[17], float(st["iters"]), float(st["fallback"]), ms, row[-1]]))
            path = Path(); path.header.frame_id = self.ref_frame or "odom"; path.header.stamp = self.get_clock().now().to_msg()
            for Xr_i, _ in hz:
                ps = PoseStamped(); ps.header = path.header
                ps.pose.position.x, ps.pose.position.y = float(Xr_i[0]), float(Xr_i[1])
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(Xr_i[2] / 2), math.cos(Xr_i[2] / 2)
                path.poses.append(ps)
            self.pub_path.publish(path)

    def finish(self):
        if self.done:
            return
        self.done = True
        try: self.pub.publish(Float32MultiArray(data=[0.0] * 4))
        except Exception: pass
        if not self.csv_path or not self.rows:
            return
        hdr = ["t", "x", "y", "th", "xr", "yr", "thr", "xe", "ye", "the",
               "w_fl", "w_fr", "w_rl", "w_rr", "wc_fl", "wc_fr", "wc_rl", "wc_rr", "load", "qp_iters", "fallback", "step_ms", "ref_age"]
        with open(self.csv_path, "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(hdr); w.writerows(self.rows)
        R = np.array(self.rows, float); e = np.hypot(R[:, 7], R[:, 8]); the = np.abs(R[:, 9])
        print(f"\n[INFO] 저장: {len(self.rows)}행 → {self.csv_path}")
        print(f"       위치오차 RMS {np.sqrt(np.mean(e**2))*100:.2f} cm  최대 {e.max()*100:.2f} cm  헤딩 최대 {np.degrees(the.max()):.1f}°")
        print(f"       바퀴 부하 평균 {R[:,18].mean()*100:.0f} %  최대 {R[:,18].max()*100:.0f} %   QP 반복 최대 {int(R[:,19].max())}  폴백 {int(R[:,20].sum())}  step 평균 {R[:,21].mean():.1f} / 최대 {R[:,21].max():.1f} ms")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WheelTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.finish()
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# WP4 — MPC 궤적추종 노드 (플래너 연결판)
# ---------------------------------------------------------------
#   /dra_mppi/reference_trajectory  (amr_interfaces/PlanarReferenceArray, 10 Hz, dt 0.2 s × 20)
#   /odometry/filtered              (nav_msgs/Odometry)  포즈 X=[x,y,yaw] + 몸체속도 v=[vx,vy,wz]
#      ↓ ReferenceHorizon 이 Ts=0.04 s × Np=20 (0.8 s) 로 보간
#   /cmd_vel                        (geometry_msgs/Twist)  ← MPC 첫 이동 u(k)
#   /mpc/reference_horizon          (nav_msgs/Path)  보간된 지평 (RViz 확인용)
#   /mpc/status                     (std_msgs/Float64MultiArray)
#                                    [t, xe, ye, the, load, qp_iters, fallback, step_ms, ref_age]
#
# 안전
#   · 기준열이 ref_timeout 보다 오래되거나 odom 이 odom_timeout 동안 없으면 매 틱 0 을 낸다.
#   · 다시 기준이 들어오면 MPC 내부(Δx_m, λ)와 u_prev 를 리셋하고 재시작 — 끊긴 사이의 Δx_m 점프 방지.
#   · 기준 부하 |vx|+|vy|+k|ω| 가 load_budget·V_LIMIT 를 넘으면 경고만 한다 (거부는 플래너 몫).
#     DRA-MPPI 도 같은 L1 다면체를 투영으로 건다 (dra_mppi.yaml limits.load_budget 을 이 값과
#     같게 두면 이 경고는 안 뜬다). 다른 기준 원천은 상자만 걸 수 있으니 경고는 유지한다.
#   · 플래너 정지 기준열(속도 0, 포즈 한 점) + 오차가 hold.enter 안이면 명령 0 + MPC 리셋 (HoldLatch).
#     exit 를 넘으면 추종 재개. 도착 후 잡음을 쫓는 stick-slip 을 막는다 (core/reference.py 주석).
#
# 프레임: 기준열의 header.frame_id 와 odom 의 header.frame_id 가 달라야 할 이유가 없다 (둘 다 odom).
#        다르면 한 번 경고한다 — 변환은 하지 않는다.
# ---------------------------------------------------------------
import csv, math, time
import numpy as np
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray
from amr_interfaces.msg import PlanarReferenceArray

from amr_mpc.body import mpc as mpc_mod
from amr_mpc.core import config as params
from amr_mpc.core.reference import ReferenceHorizon, HoldLatch, wrap
MPC, K_GEOM = mpc_mod.MPC, mpc_mod.K_GEOM


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_to_sec(st):
    return st.sec + st.nanosec * 1e-9


class MPCTrackingNode(Node):
    def __init__(self):
        super().__init__("mpc_tracking_node")
        self.declare_parameters("", [
            # 기본값은 mpc.py 의 단일 출처에서 온다 (config/mpc.yaml 이 덮어쓴다).
            ("Ts", mpc_mod.DEFAULT_TS), ("Np", mpc_mod.DEFAULT_NP), ("Nc", mpc_mod.DEFAULT_NC),
            ("q_pos", mpc_mod.DEFAULT_Q[0]), ("q_yaw", mpc_mod.DEFAULT_Q[2]),
            ("r_du", mpc_mod.DEFAULT_R[0]),
            ("max_pose_error", [float(v) for v in mpc_mod.DEFAULT_Y_MAX]),   # 출력 연성 한계 [m,m,rad]
            ("max_cmd_accel", list(mpc_mod.DEFAULT_ACCEL)),   # [m/s², m/s², rad/s²]. du_max = 이 값 × Ts.
                                                       # 0 이하인 축은 제약을 안 건다 (mpc.py du_max=inf).
            ("max_wheel_rad_per_sec", mpc_mod.W_MAX),  # base.yaml 과 같게. V_LIMIT = 0.04 × 이 값
            ("load_budget", mpc_mod.LOAD_BUDGET),      # 기준 부하 경고 문턱 (비율)
            ("reference_topic", "/dra_mppi/reference_trajectory"),
            ("odom_topic", "/odometry/filtered"),      # 플래너와 같은 원천이어야 오차 프레임이 맞는다
            ("cmd_vel_topic", "/cmd_vel"),
            ("ref_timeout", mpc_mod.REF_TIMEOUT),      # 기준열 header.stamp 가 이보다 오래되면 정지
            ("odom_timeout", mpc_mod.ODOM_TIMEOUT),
            ("publish_debug", True),
            ("log_prefix", ""),                        # 비면 CSV 저장 안 함. 예: mpc_track
            ("hold_enter_xy", float(params.section("hold")["enter_xy"])),
            ("hold_enter_yaw_deg", float(params.section("hold")["enter_yaw_deg"])),
            ("hold_exit_xy", float(params.section("hold")["exit_xy"])),
            ("hold_exit_yaw_deg", float(params.section("hold")["exit_yaw_deg"])),
            ("hold_stopped_v", float(params.section("hold")["stopped_v"])),
            ("hold_stopped_w_deg", float(params.section("hold")["stopped_w_deg"])),
        ])
        g = lambda n: self.get_parameter(n).value
        self.Ts, self.Np = float(g("Ts")), int(g("Np"))
        mpc_mod.set_wheel_limit(g("max_wheel_rad_per_sec"))
        self.V_LIMIT, self.budget = mpc_mod.V_LIMIT, float(g("load_budget"))
        acc = np.asarray(g("max_cmd_accel"), float)
        du_max = mpc_mod.du_max_from_accel(acc, self.Ts)           # Wang Ch.2 의 |Δu| ≤ du_max
        ymax = np.asarray(g("max_pose_error"), float)              # 출력(포즈오차) 연성 한계
        self.mpc = MPC(Ts=self.Ts, Np=self.Np, Nc=int(g("Nc")),
                       q=(g("q_pos"), g("q_pos"), g("q_yaw")), r=(g("r_du"), g("r_du"), g("r_du")),
                       du_max=(None if np.all(np.isinf(du_max)) else du_max),
                       y_max=(None if np.all(np.isinf(ymax)) else ymax))
        self.ref_timeout, self.odom_timeout = float(g("ref_timeout")), float(g("odom_timeout"))

        self.X = self.v = None; self.last_odom = None; self.odom_frame = None
        self.ref = None; self.ref_frame = None; self.ref_stamp = None; self.ref_rx = None
        self.u_prev = np.zeros(3); self.active = False; self.frame_warned = False
        self.latch = HoldLatch(g("hold_enter_xy"), math.radians(g("hold_enter_yaw_deg")),
                               g("hold_exit_xy"), math.radians(g("hold_exit_yaw_deg")))
        self.hold_stopped_v, self.hold_stopped_w = float(g("hold_stopped_v")), math.radians(g("hold_stopped_w_deg"))
        self.rows = []; self.csv_path = (f"{g('log_prefix')}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
                                         if g("log_prefix") else None)
        self.done = False

        self.cmd_topic = str(g("cmd_vel_topic"))
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.debug = bool(g("publish_debug"))
        self.pub_path = self.create_publisher(Path, "/mpc/reference_horizon", 1)
        self.pub_status = self.create_publisher(Float64MultiArray, "/mpc/status", 10)
        self.create_subscription(Odometry, str(g("odom_topic")), self.cb_odom, 20)
        self.create_subscription(PlanarReferenceArray, str(g("reference_topic")), self.cb_ref, 5)
        self.timer = self.create_timer(self.Ts, self.on_tick)
        self._owner_timer = self.create_timer(2.0, self.check_cmd_owner)   # 아래 참조
        self.get_logger().info(
            f"기준 {g('reference_topic')}  상태 {g('odom_topic')}  → {g('cmd_vel_topic')}   "
            f"Ts {self.Ts*1000:.0f} ms Np {self.Np} Nc {int(g('Nc'))}  V_LIMIT {self.V_LIMIT:.2f} m/s   "
            f"r_du {g('r_du')}  명령가속도 {'제한없음' if self.mpc.du_max is None else np.array2string(acc, precision=2)}"
            f"{'' if self.mpc.du_max is None else f' → |Δu| ≤ {np.array2string(du_max, precision=3)}'}")

    def check_cmd_owner(self):
        """cmd_vel 을 다른 노드가 같이 발행하면 명령이 섞여 로봇이 명령의 일부만 실행한다.
        2026-09-03: joy_teleop 이 조이스틱 조작이 없어도 0 을 20 Hz 로 쏘고 있었고, 25 Hz MPC 와
        섞이며 실측 이득이 0.97 → 0.83 으로 떨어졌다 (제자리회전 시험). 그 상태로 8자를
        돌리자 MPC 가 포화·발산했다. joy_teleop 은 이후 중립일 때 버스를 놓도록 고쳤지만,
        발행자로 등록돼 있는 한 조작하는 순간 다시 섞인다 — 그래서 개수만으로 경고한다.
        경고만 한다 — 소유권 강제는 keyboard_command_gate 몫."""
        n = self.count_publishers(self.cmd_topic)
        if n > 1:
            self.get_logger().error(
                f"{self.cmd_topic} 발행자가 {n} 개다 — MPC 말고 다른 노드가 이 토픽을 쥐고 있다 "
                f"(joy_teleop 등). 그쪽이 실제로 발행하는 순간 명령이 섞여 로봇이 명령의 일부만 "
                f"실행하고 MPC 는 포화한다. use_joystick:=false 로 띄우거나 "
                f"keyboard_command_gate 로 소유권을 강제해라.")
        else:
            self.destroy_timer(self._owner_timer)

    # ---------- 콜백 ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        self.X = np.array([p.position.x, p.position.y, yaw_of(p.orientation)])
        self.v = np.array([t.linear.x, t.linear.y, t.angular.z])
        self.last_odom = self.now(); self.odom_frame = m.header.frame_id

    def cb_ref(self, m):
        if not m.references:
            self.get_logger().warn("빈 기준열", throttle_duration_sec=2.0); return
        rows = [(r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate) for r in m.references]
        try:
            self.ref = ReferenceHorizon.from_rows(stamp_to_sec(m.header.stamp), m.dt, rows)
        except ValueError as e:
            self.get_logger().warn(f"기준열 무시: {e}", throttle_duration_sec=2.0); return
        self.ref_stamp, self.ref_rx, self.ref_frame = self.ref.t0, self.now(), m.header.frame_id
        if self.odom_frame and self.ref_frame != self.odom_frame and not self.frame_warned:
            self.get_logger().warn(f"프레임 불일치: 기준 '{self.ref_frame}' vs odom '{self.odom_frame}' — 변환 없이 사용")
            self.frame_warned = True

    # ---------- 주기 ----------
    def stop(self, reason=None):
        self.pub.publish(Twist())
        if self.active:
            self.get_logger().info(f"정지: {reason}")
        self.active = False; self.u_prev = np.zeros(3); self.mpc.reset()

    def on_tick(self):
        now = self.now()
        if self.X is None or now - self.last_odom > self.odom_timeout:
            self.stop("odom 없음"); self.get_logger().warn("odom 대기/끊김", throttle_duration_sec=2.0); return
        if self.ref is None:
            self.stop("기준 없음"); return
        ref_age = now - self.ref_stamp                     # 플래너 시각 기준 (클럭이 같다는 전제)
        if now - self.ref_rx > self.ref_timeout:           # 수신 시각 기준 — 클럭이 어긋나도 끊김은 잡힌다
            self.stop("기준열 끊김"); self.get_logger().warn(f"기준열 {now-self.ref_rx:.2f} s 동안 없음", throttle_duration_sec=2.0); return
        if ref_age < -self.ref.dt or ref_age > self.ref_timeout + self.ref.dt:
            self.get_logger().warn(f"기준열 stamp 와 노드 시각 차 {ref_age:+.2f} s — 클럭 확인", throttle_duration_sec=5.0)
        if not self.active:
            self.active = True; self.mpc.reset(); self.u_prev = np.zeros(3)
            self.get_logger().info(f"추종 시작. 기준열 {self.ref.T}×{self.ref.dt:.2f} s, 나이 {ref_age:.2f} s")

        hz = self.ref.horizon(now, self.Ts, self.Np)       # [(Xr_i, ur_i)] i=0..Np−1
        Xr, ur = hz[0]
        Xe = Xr - self.X; Xe[2] = wrap(Xe[2])
        was_idle = self.latch.idle
        stopped = np.hypot(self.v[0], self.v[1]) < self.hold_stopped_v and abs(self.v[2]) < self.hold_stopped_w
        if self.latch.update(self.ref.is_hold, Xe, stopped=bool(stopped)):
            if not was_idle:
                self.get_logger().info(f"hold 진입: 오차 {np.hypot(Xe[0], Xe[1])*1000:.1f} mm, {math.degrees(Xe[2]):.2f}° — 명령 0, MPC 리셋")
                self.mpc.reset(); self.u_prev = np.zeros(3)
            self.pub.publish(Twist())
            if self.debug:
                self.pub_status.publish(Float64MultiArray(data=[now, *map(float, Xe), 0.0, 0.0, 0.0, 0.0, ref_age]))
            return
        if was_idle:
            self.get_logger().info(f"hold 해제: 오차 {np.hypot(Xe[0], Xe[1])*1000:.1f} mm, {math.degrees(Xe[2]):.2f}° — 추종 재개")
        ref = [(Xr_i[2], ur_i[0], ur_i[1]) for Xr_i, ur_i in hz]
        t_c = time.perf_counter()
        u = self.mpc.step(self.X, self.v, self.u_prev, ref, Xr, ur)
        ms = (time.perf_counter() - t_c) * 1000
        self.u_prev = u
        tw = Twist(); tw.linear.x, tw.linear.y, tw.angular.z = (float(x) for x in u)
        self.pub.publish(tw)

        load_r = abs(ur[0]) + abs(ur[1]) + K_GEOM * abs(ur[2])
        if load_r > self.budget * self.V_LIMIT:
            self.get_logger().warn(f"기준 부하 {load_r/self.V_LIMIT*100:.0f} % > 예산 {self.budget*100:.0f} % — 보정 여유 부족",
                                   throttle_duration_sec=2.0)
        st = self.mpc.stats
        load = abs(u[0]) + abs(u[1]) + K_GEOM * abs(u[2])
        row = [round(now, 4), *self.X, *Xr, *Xe, *self.v, *u, load,
               st.get("iters", 0), st.get("fallback", 0), round(ms, 2), round(ref_age, 3)]
        if self.csv_path:
            self.rows.append(row)
        if self.debug:
            self.pub_status.publish(Float64MultiArray(data=[float(row[0]), *map(float, Xe), load,
                                                            float(row[17]), float(row[18]), ms, ref_age]))
            self.publish_path(hz)

    def publish_path(self, hz):
        path = Path(); path.header.frame_id = self.ref_frame or "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for Xr_i, _ in hz:
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(Xr_i[0]), float(Xr_i[1])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(Xr_i[2] / 2), math.cos(Xr_i[2] / 2)
            path.poses.append(ps)
        self.pub_path.publish(path)

    # ---------- 종료 ----------
    def finish(self):
        if self.done:
            return
        self.done = True
        try: self.pub.publish(Twist())
        except Exception as e: print(f"[WARN] 정지 실패: {e}")
        if not self.csv_path or not self.rows:
            return
        hdr = ["t", "x", "y", "th", "xr", "yr", "thr", "xe", "ye", "the", "vx", "vy", "wz",
               "ux", "uy", "uw", "load", "qp_iters", "fallback", "step_ms", "ref_age"]
        with open(self.csv_path, "w", newline="") as f:
            w = csv.writer(f); w.writerow(hdr); w.writerows(self.rows)
        R = np.array(self.rows, float); e = np.hypot(R[:, 7], R[:, 8]); the = np.abs(R[:, 9])
        load = R[:, 16] / self.V_LIMIT; ms = R[:, 19]
        print(f"\n[INFO] 저장: {len(self.rows)}행 → {self.csv_path}")
        print(f"       위치오차  RMS {np.sqrt(np.mean(e**2))*100:.2f} cm   최대 {e.max()*100:.2f} cm   헤딩 최대 {np.degrees(the.max()):.1f}°")
        print(f"       부하      평균 {load.mean()*100:.0f} %   최대 {load.max()*100:.0f} %   (V_LIMIT {self.V_LIMIT:.2f})")
        print(f"       QP        반복 최대 {int(R[:, 17].max())}   폴백 {int(R[:, 18].sum())} 스텝   step 평균 {ms.mean():.1f} / 최대 {ms.max():.1f} ms  (주기 {self.Ts*1000:.0f})")
        print(f"       기준 나이 평균 {R[:, 20].mean():.3f} s   최대 {R[:, 20].max():.3f} s")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MPCTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 정리 중 두 번째 Ctrl+C(launch 가 보냄) 무시
        node.finish()
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

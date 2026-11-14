#!/usr/bin/env python3
# 8자 기준궤적 발행 노드 — mpc_tracking_node 가 그대로 받아 추종한다.
# fake_reference_node(원궤적)의 8자판.  궤적 수식·검산은 figure8.py.
# ---------------------------------------------------------------
#   /odometry/filtered              (nav_msgs/Odometry)      첫 포즈를 8자의 교차점(원점)으로 잡는다
#      ↓
#   /dra_mppi/reference_trajectory  (PlanarReferenceArray, 10 Hz, dt 0.2 s × 20)  ← mpc_tracking_node 의 입력
#   /figure8/path                   (nav_msgs/Path, latched)  RViz 로 경로 전체 확인
#
# 진행
#   [정렬 회전 align_s] → [8자 laps 바퀴] → [역정렬 회전 align_s] → [기동 포즈 유지 hold_s] → 발행 중단
#
# wait_for_arm (토크 MPC 실험, 2026-11-11 추가 — 기본 false 라 기존 사용처는 그대로)
#   기준 시간은 게이트가 무장(/mpc/armed = true)된 동안만 흐른다.
#     · 기동 포즈(8자 교차점)는 처음 무장한 순간의 포즈로 잡는다. start_delay 도 그때부터 센다.
#     · 도중에 정지('s')하면 기준 시간도 멈춘다. 다시 무장하면 멈춘 자리에서 이어 간다.
#     · 무장할 때마다 기준 시간 배속을 0 → 1 로 resume_ramp_s 초에 걸쳐 올린다 (core/figure8.py ramp_elapsed).
#       순항 중에 멈췄다 재개하면 기준 속도가 0 → V 로 한 번에 뛰어 토크가 상한(1.3 N·m)까지 튀었다 (2026-11-11 시뮬).
#       발행하는 기준 속도도 그 배속을 곱한 값이다 (포즈와 속도가 서로 맞게).
#     · 다시 무장할 때는 멈춘 시각이 아니라, 멈춘 시각 앞뒤 resync_window_s 안에서 로봇의 지금 위치에 가장
#       가까운 경로 지점의 시각에서 이어 간다. 정지 명령 뒤 감쇠로 서는 동안 로봇이 더 미끄러져 가서(시뮬 0.5 m/s
#       에서 8.8 cm), 멈춘 기준점에서 재개하면 MPC 가 그 거리를 한 번에 메우려고 0.87 N·m 를 냈다 (2026-11-11).
#     · 기준열 첫 행을 "지금" 으로 둔다: stamp = 지금 − dt, 행 i 는 지금 + i·dt (규약 stamp + (i+1)·dt 그대로).
#       첫 행이 지금 + dt 면 받는 쪽(ReferenceHorizon)이 지금의 기준을 첫 행 속도로 거꾸로 외삽하는데, 배속이 오르는
#       중이면 서 있는 로봇에게 "이미 0.1 m/s" 를 요구해 재개 순간 0.4 N·m 가 났다.
#   이유: 게이트가 막힌 동안 기준만 앞서 가면, 서 있는 로봇과의 오차를 보고 MPC 토크가 상한까지 쌓였다가
#   무장하는 순간 한꺼번에 나간다 (2026-11-11 3단계 시뮬에서 발견).
#
# origin_mode (2026-11-11 추가, 기본 robot = 예전 동작)
#   robot  기동 포즈(처음 무장한 순간, wait_for_arm 이 아니면 첫 odom)가 8자 교차점이자 방향이다.
#   fixed  8자를 지도에 고정한다: 교차점 = (origin_x, origin_y), 방향 = origin_yaw_deg.  경로는 기동 전부터 RViz 에 보인다.
#          출발 때 로봇이 교차점에서 start_tolerance_* 안이면, 로봇 포즈 → 교차점의 진입 구간(스무스스텝, 평균
#          lead_speed·lead_yaw_rate 이하, 최소 start_delay)을 붙이고 8자를 시작한다. 밖이면 출발을 거부하고 기준을
#          내지 않는다 (MPC 는 기준이 없으니 DAMPING). 옮겨 놓고 다시 무장하면 다시 검사한다.
#   발행이 끊기면 mpc_tracking_node 가 ref_timeout(0.5 s) 뒤 /cmd_vel 0 을 내고 멈춘다.
#   yaw_mode=fixed 면 정렬 회전이 없다 (헤딩 고정 게걸음).
#
# 부하 예산
#   기준 부하 |vx|+|vy|+K_GEOM·|ω| 가 load_budget·V_LIMIT 를 넘으면 뜨지 않고 거부한다
#   (fake_reference_node 와 같은 규칙).  남는 35 % 가 MPC 의 보정 여유다.
#
# 예)  ros2 run amr_mpc figure8_reference_node --ros-args -p width:=1.4 -p height:=1.0 -p speed:=0.25
# ---------------------------------------------------------------
import math
import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from amr_interfaces.msg import PlanarReference, PlanarReferenceArray

from amr_mpc.core import node_main
from amr_mpc.body import mpc as mpc_mod
from amr_mpc.core.figure8 import Figure8, lead_in, lead_in_duration, ramp_elapsed, ramp_rate
K_GEOM = mpc_mod.K_GEOM


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Figure8ReferenceNode(Node):
    def __init__(self):
        super().__init__("figure8_reference_node")
        self.declare_parameters("", [
            # 8자 모양 — 경계상자가 정확히 width × height 이고 기동 포즈가 그 중심이다.
            # 방 2.0 m, 로봇 외접반경 0.18 m 이면 1.4 폭에서 좌우 여유 0.12 m.
            # height 는 곡률을 정한다 (잎사귀 꼭대기 κ ≈ 16H/W²) — 헤딩 오차와 부하가 여기서 나온다.
            # 폐루프 시뮬(초기오차 10 cm, v 0.25): H 1.0/0.8/0.6/0.4 → 헤딩오차 최대
            # 12.5/9.8/7.7/6.0 °, 부하 67/59/53/49 %.  위치오차는 1.3 cm 로 거의 안 변한다.
            # 기본 0.8 은 잎사귀가 0.7 × 0.8 로 가장 둥근 지점.
            ("width", 1.4), ("height", 0.8),
            ("lap_speeds", [0.0]),                    # 랩별 순항 속도 [m/s]. [0.0] 이면 안 씀 (speed·laps 하나로). 주면 laps = 목록 길이
            ("speed", 0.25),                          # 순항 접선속도 [m/s]
            ("accel", 0.4),                           # 호길이 축 가감속 [m/s²]
            ("laps", 1.0),                            # 바퀴 수. 1 바퀴마다 기동 포즈로 돌아온다
            ("yaw_mode", "tangent"),                  # tangent = 접선 추종 | fixed = 헤딩 고정(게걸음)
            ("align_s", 1.5),                         # tangent 모드 앞뒤 제자리 회전 시간 [s]
            ("start_delay", 3.0),                     # 첫 odom 뒤 이만큼 기동 포즈를 유지하다 출발 (실차에서 손 뺄 시간)
            ("abort_error", 0.35),                    # 로봇이 기준에서 이만큼 벌어지면 발행 중단 → MPC 타임아웃 정지.
                                                      # 2026-09-03 실차에서 오차 2.8 m 까지 달아났다. 0 이하면 끈다.
            ("hold_s", 5.0),                          # 복귀 후 제자리 유지 시간. <0 이면 무한 유지
            ("rate_hz", 10.0), ("dt", 0.2), ("horizon_steps", 20),
            ("frame_id", "odom"),
            ("odom_topic", "/odometry/filtered"),
            ("reference_topic", "/dra_mppi/reference_trajectory"),
            ("max_wheel_rad_per_sec", mpc_mod.W_MAX),  # base.yaml·mpc.yaml 과 같게. V_LIMIT = 0.04 × 이 값
            ("wait_for_arm", False),                  # true 면 무장된 동안만 기준 시간이 흐른다 (머리주석)
            ("armed_topic", "/mpc/armed"),            # keyboard_command_gate 가 내는 무장 상태 (latched)
            ("resume_ramp_s", -1.0),                  # 무장 뒤 배속 0→1 시간 [s]. 음수면 speed/accel (경로 가속과 같은 가속도)
            ("resync_window_s", 2.0),                 # 재무장 때 가장 가까운 경로 지점을 찾는 범위 (멈춘 시각 ±) [s]. 0 이면 안 찾음
            ("load_budget", mpc_mod.LOAD_BUDGET),
            ("origin_mode", "robot"),                 # robot | fixed (머리주석)
            ("origin_x", 0.0), ("origin_y", 0.0),     # fixed: 8자 교차점 [m] (frame_id 좌표)
            ("origin_yaw_deg", 0.0),                  # fixed: 8자 가로축 방향 [deg]
            ("start_tolerance_xy", 0.30),             # fixed: 출발점에서 이보다 멀면 출발 거부 [m]
            ("start_tolerance_yaw_deg", 30.0),        # fixed: [deg]
            ("lead_speed", 0.10),                     # fixed: 진입 구간 평균 속도 상한 [m/s]
            ("lead_yaw_rate", 0.5),                   # fixed: 진입 구간 평균 회전속도 상한 [rad/s]
        ])
        g = lambda n: self.get_parameter(n).value
        self.dt, self.T, self.frame = float(g("dt")), int(g("horizon_steps")), str(g("frame_id"))
        self.hold, self.delay = float(g("hold_s")), max(0.0, float(g("start_delay")))
        self.abort = float(g("abort_error"))

        mpc_mod.set_wheel_limit(g("max_wheel_rad_per_sec"))
        V_LIMIT, budget = mpc_mod.V_LIMIT, float(g("load_budget"))
        try:
            self.path = Figure8(width=g("width"), height=g("height"), speed=g("speed"),
                                accel=g("accel"), laps=g("laps"), yaw_mode=str(g("yaw_mode")),
                                align_s=g("align_s"),
                                lap_speeds=[v for v in g("lap_speeds") if v > 0.0] or None)
        except ValueError as e:
            raise SystemExit(f"[figure8] 파라미터 오류: {e}")

        run, spin = self.path.peak_load(K_GEOM)
        cap = budget * V_LIMIT
        if run > cap:                                 # 주행 부하는 speed 에 선형
            raise SystemExit(
                f"[figure8] 기준 부하 {run:.3f} > 예산 {cap:.3f} ({budget*100:.0f} % of {V_LIMIT:.2f}). "
                f"speed 를 {float(g('speed')) * cap / run:.3f} m/s 이하로 낮추거나 height 를 줄여라 "
                f"(최대곡률 {self.path.max_curvature():.2f} 1/m 이 ω=κv 를 키운다).")
        if spin > cap:                                # 정렬 회전 부하는 align_s 에 반비례
            raise SystemExit(f"[figure8] 정렬 회전 부하 {spin:.3f} > 예산 {cap:.3f}. "
                             f"align_s 를 {float(g('align_s')) * spin / cap:.2f} s 이상으로 늘려라.")

        self.origin = None; self.t0 = None; self.finished = False; self.pose = None
        self.wait_for_arm = bool(g("wait_for_arm"))
        self.armed = not self.wait_for_arm            # 기다리지 않으면 처음부터 흐른다 (기존 동작)
        self.t_run = 0.0                              # 무장된 동안 누적된 기준 시간 [s]
        self.t_last_tick = None
        self.rate = 0.0 if self.wait_for_arm else 1.0  # 기준 시간 배속 (기다리지 않으면 늘 1)
        rr = float(g("resume_ramp_s"))
        self.ramp_s = (self.path.V / self.path.A) if rr < 0.0 else rr
        self.resync_window = max(0.0, float(g("resync_window_s")))
        self.pose_full = None
        mode = str(g("origin_mode")).strip().lower()
        if mode not in ("robot", "fixed"):
            raise SystemExit(f"[figure8] origin_mode={mode!r} — robot | fixed")
        self.fixed = mode == "fixed"
        self.track_origin = np.array([float(g("origin_x")), float(g("origin_y")), math.radians(float(g("origin_yaw_deg")))])
        self.tol_xy, self.tol_yaw = float(g("start_tolerance_xy")), math.radians(float(g("start_tolerance_yaw_deg")))
        self.lead_speed, self.lead_yaw_rate = float(g("lead_speed")), float(g("lead_yaw_rate"))
        self.min_delay = self.delay
        self.lead_from = None                         # 진입 구간 시작 포즈 (fixed 출발 때)
        self.refused = False                          # fixed: 출발점에서 멀어 거부한 상태 (다음 무장 때 다시 검사)
        if self.wait_for_arm:
            arm_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, str(g("armed_topic")), self.cb_armed, arm_qos)
        self.pub = self.create_publisher(PlanarReferenceArray, str(g("reference_topic")), 5)
        self.pub_path = self.create_publisher(
            Path, "/figure8/path", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, str(g("odom_topic")), self.cb_odom, 10)
        self.create_timer(1.0 / float(g("rate_hz")), self.on_tick)
        if self.fixed:                                # 지도 고정: 기동 전부터 경로를 보여 준다
            self.publish_path(self.track_origin)

        p = self.path
        for k, V, vmax, t0 in p.lap_profile_summary():
            self.get_logger().info(f"  랩 {k}: 목표 {V:.2f} m/s, 실제 최고 {vmax:.2f} m/s, 시작 {t0:.1f} s")
        self.get_logger().info(
            f"8자 {p.W:.2f} × {p.H:.2f} m (Gerono 렘니스케이트)  yaw {p.yaw_mode}  {p.laps:g} 바퀴\n"
            f"  호길이 {p.L:.2f} m  순항 {p.V:.2f} m/s  가감속 {p.A:.2f} m/s²  "
            f"최대곡률 {p.max_curvature():.2f} 1/m (R_min {1/p.max_curvature():.3f} m)\n"
            f"  시간  대기 {self.delay:.1f} s + 정렬 {p.T_align:.1f} s ×2 + 주행 {p.T_path:.1f} s = {p.T_total:.1f} s"
            f"{'' if self.hold < 0 else f' + 유지 {self.hold:.1f} s'}\n"
            f"  부하  주행 {run/V_LIMIT*100:.0f} %  정렬 {spin/V_LIMIT*100:.0f} %  "
            f"(예산 {budget*100:.0f} %, V_LIMIT {V_LIMIT:.2f} m/s)  → 보정 여유 {(1-run/V_LIMIT)*100:.0f} %")

    # ---------- 콜백 ----------
    def cb_odom(self, m):
        p = m.pose.pose
        self.pose = np.array([p.position.x, p.position.y])
        self.pose_full = np.array([p.position.x, p.position.y, yaw_of(p.orientation)])
        if self.origin is None and self.armed and not self.refused:
            self.set_origin()

    def set_origin(self):
        if not self.fixed:
            self.origin = self.pose_full.copy()
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f"기동 포즈(8자 교차점) x {self.origin[0]:.3f}  y {self.origin[1]:.3f}  "
                                   f"yaw {math.degrees(self.origin[2]):.1f}°  — {self.delay:.1f} s 뒤 출발")
            self.publish_path()
            return
        # 지도 고정: 출발점과의 거리 검사 → 진입 구간
        o, p = self.track_origin, self.pose_full
        d = float(np.hypot(*(o[:2] - p[:2])))
        a = abs(math.atan2(math.sin(o[2] - p[2]), math.cos(o[2] - p[2])))
        if d > self.tol_xy or a > self.tol_yaw:
            self.refused = True
            self.get_logger().error(
                f"출발 거부: 로봇이 출발점(x {o[0]:.2f}, y {o[1]:.2f}, {math.degrees(o[2]):.0f}°)에서 "
                f"{d*100:.1f} cm, {math.degrees(a):.1f}° 떨어져 있다 (허용 {self.tol_xy*100:.0f} cm, "
                f"{math.degrees(self.tol_yaw):.0f}°). 출발점에 놓고 's' 뒤 다시 'a'.")
            return
        self.refused = False
        self.origin = o.copy()
        self.lead_from = p.copy()
        self.delay = lead_in_duration(p, o, self.lead_speed, self.lead_yaw_rate, self.min_delay)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"출발: 출발점에서 {d*100:.1f} cm, {math.degrees(a):.1f}° — 진입 {self.delay:.1f} s 뒤 8자 시작 "
            f"(교차점 x {o[0]:.3f}  y {o[1]:.3f}  {math.degrees(o[2]):.1f}°)")
        self.publish_path(o)

    def ref_at(self, t):
        """기준 시각 t 의 (Xr, ur). t < 0 은 진입 구간(fixed) 또는 기동 포즈 유지."""
        if t < 0.0 and self.lead_from is not None:
            return lead_in(t + self.delay, self.delay, self.lead_from, self.origin)
        return self.path.at(t, self.origin)

    def cb_armed(self, m):
        armed = bool(m.data)
        if armed == self.armed:
            return
        self.armed = armed
        self.rate = 0.0                               # 무장하면 0 배속에서 다시 올린다, 정지하면 0
        if armed and self.origin is None and self.pose_full is not None:
            self.refused = False                      # 다시 무장했으니 다시 검사한다
            self.set_origin()
        elif not self.finished and self.origin is not None:
            moved = ""
            if armed:
                moved = self.resync()
            self.get_logger().info(
                f"{'무장 — 기준 시간 재개' if armed else '정지 — 기준 시간 멈춤'} (t = {self.t_run - self.delay:.2f} s){moved}")

    def resync(self):
        """재무장: 멈춘 시각 ±resync_window 안에서 로봇 위치에 가장 가까운 경로 시각으로 옮긴다."""
        if self.resync_window <= 0.0 or self.pose is None:
            return ""
        t_stop = self.t_run - self.delay
        lo = max(-self.delay, t_stop - self.resync_window)
        hi = min(self.path.T_total, t_stop + self.resync_window)
        ts = np.arange(lo, hi + 1e-9, 0.005)
        d = [np.hypot(*(self.ref_at(t)[0][:2] - self.pose)) for t in ts]
        k = int(np.argmin(d))
        d_stop = float(np.hypot(*(self.ref_at(t_stop)[0][:2] - self.pose)))
        self.t_run = float(ts[k]) + self.delay
        return f" — 로봇 위치에 맞춰 {ts[k] - t_stop:+.2f} s 옮김 (거리 {d_stop*100:.1f} → {d[k]*100:.1f} cm)"

    def publish_hold(self, now, Xr):
        """기준 시간이 멈춘 동안: 멈춘 자리의 포즈에 속도 0 인 기준열."""
        msg = PlanarReferenceArray()
        msg.header.stamp = now.to_msg(); msg.header.frame_id = self.frame; msg.dt = self.dt
        for _ in range(self.T):
            r = PlanarReference(); r.header = msg.header
            r.x, r.y, r.yaw = map(float, Xr)
            msg.references.append(r)
        self.pub.publish(msg)

    def publish_path(self, origin=None):
        msg = Path(); msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path.path_xy(origin=self.origin if origin is None else origin):
            ps = PoseStamped(); ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def on_tick(self):
        if self.origin is None or self.finished:
            return
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        if self.t_last_tick is not None and self.armed:          # 무장된 동안만 기준 시간이 흐른다
            el = max(0.0, now_s - self.t_last_tick)
            self.t_run += ramp_elapsed(el, self.rate, self.ramp_s) if self.wait_for_arm else el
            if self.wait_for_arm:
                self.rate = ramp_rate(el, self.rate, self.ramp_s)
        self.t_last_tick = now_s
        t = self.t_run - self.delay                             # t<0 이면 at() 이 기동 포즈·속도 0 을 낸다
        if not self.armed:                                      # 멈춘 동안은 그 자리 정지 기준 (속도 0)
            Xr, _ = self.ref_at(t)
            self.publish_hold(now, Xr)
            return
        if 0.0 <= self.hold and t > self.path.T_total + self.hold:
            self.finished = True
            self.get_logger().info(f"8자 완주 + 유지 {self.hold:.1f} s 끝 — 발행 중단. "
                                   f"MPC 가 곧 타임아웃으로 정지한다.")
            return
        if self.abort > 0.0 and self.pose is not None:      # 발산 비상정지 — MPC 가 못 잡으면 여기서 끊는다
            err = float(np.hypot(*(self.pose - self.ref_at(t)[0][:2])))
            if err > self.abort:
                self.finished = True
                self.get_logger().error(
                    f"중단: 기준에서 {err*100:.1f} cm 벗어남 (한계 {self.abort*100:.1f} cm). 발행을 끊는다 — MPC 가 곧 정지한다.\n"
                    f"  체크: odom_topic 이 /odom 인가 (EKF 를 끼면 모델에 없는 지연이 붙는다),"
                    f" mpc/status 의 부하·QP 반복, speed 를 낮췄는가")
                return

        msg = PlanarReferenceArray()
        msg.header.frame_id = self.frame; msg.dt = self.dt
        if self.wait_for_arm:                          # 첫 행 = 지금 (머리주석)
            msg.header.stamp = (now - rclpy.duration.Duration(nanoseconds=int(self.dt * 1e9))).to_msg()
            first = 0
        else:                                          # 기존 동작 그대로
            msg.header.stamp = now.to_msg()
            first = 1
        for i in range(self.T):                        # references[i] 의 목표 시각 = stamp + (i+1)·dt
            tau = (i + first) * self.dt
            if self.wait_for_arm and self.rate < 1.0:  # 배속이 오르는 중: 기준 시간과 속도를 배속대로
                Xr, ur = self.ref_at(t + ramp_elapsed(tau, self.rate, self.ramp_s))
                ur = ur * ramp_rate(tau, self.rate, self.ramp_s)
            else:
                Xr, ur = self.ref_at(t + tau)
            r = PlanarReference(); r.header = msg.header
            r.x, r.y, r.yaw = map(float, Xr); r.vx, r.vy, r.yaw_rate = map(float, ur)
            msg.references.append(r)
        self.pub.publish(msg)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Figure8ReferenceNode()
    try:
        node_main.spin(node)          # Ctrl+C·SIGTERM·SIGHUP 공통 처리, 신호가 메시지 변환 중에 떨어져도 정상 종료 (R9·R10 리뷰)
    finally:
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# pure_pursuit_node — 비교군 A 의 경로 추종기 (6단계). 계산은 amr_mpc/baseline/pure_pursuit.py (근거·식은 거기 머리주석).
#   ros2 run amr_mpc pure_pursuit_node
# ---------------------------------------------------------------
#   /dra_mppi/reference_trajectory  (PlanarReferenceArray)  figure8_reference_node 의 지도 고정 8자 — MPC 와 같은 기준열
#   /state_estimator/odom           (Odometry)              MPC 와 같은 상태. pose = map, twist = 몸체
#   /mpc/armed                      (Bool, latched)         게이트 무장 여부 — MPC 와 같은 규칙
#      ↓  상태 메시지가 올 때마다 한 번 (추정기 100 Hz = MPC 와 같은 주기)
#   /pure_pursuit/cmd_vel           (TwistStamped)          몸체 속도 명령 (v_x, 0, ω_z) → wheel_pi_node
#                                                           header.stamp = 이 명령을 계산한 상태 추정값의 시각
#   /pure_pursuit/status            (Float64MultiArray)     [t, e_x, e_y, e_θ, v, ω, mode(0 추종/1 회전), L_d, κ, lead_s]
#                                                           앞 4개는 /torque_mpc/status 와 같은 꼴 (fault_monitor 가 본다)
#   /pure_pursuit/lookahead         (PointStamped)          RViz 용 룩어헤드 점
#
# 정지 규칙 — torque_mpc_node 와 같다 (같은 조건에서 같은 때 선다)
#   미무장 · 기준 없음 · 무장 뒤 새 기준열 전 · 기준열 끊김(ref_timeout) · hold 진입(정지 기준 + 오차 enter 안 + 멈춤)
#   이면 명령을 내지 않는다. wheel_pi_node 가 cmd_timeout 뒤 DAMPING + PI 리셋으로 세운다.
#   상태가 끊기면 이 노드가 깨어나지 않으니 역시 명령이 끊긴다 (같은 결과).
#   기준열과 상태의 frame_id 가 다르면 명령을 끊는다 (torque_mpc_node 와 같은 검사, R7 리뷰 2026-11-13).
#   safety·hold 문턱은 torque_mpc.yaml 을 그대로 읽는다 (A 와 B·C 의 정지 규칙이 갈라지지 않게 한 곳에서).
#
# controller:=all (2026-11-28, nodes/controller_select.py): variant_topic (/ctrl/active) 을 주면 룩어헤드 묶음을 그 값으로 고른다.
#   pp   = lookahead_time·min·max (launch 가 torque_baseline_a.yaml pure_pursuit + pp_lookahead_* 로 채운다)
#   adpp = adpp_lookahead_time·min·max (pure_pursuit_adaptive + pp_lookahead_*)
#   mpc  = 이 노드는 무장 신호를 못 받아 쉰다 (묶음은 그대로).
#   이 여섯 값은 정지 중에 바꿀 수 있다 (tools/pit.py 프리셋 s2 의 zigzag 0.1·0.2 m 처럼 경로마다 다른 값). 무장 중에는 거부.
# ---------------------------------------------------------------
import dataclasses
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64MultiArray, String

from amr_interfaces.msg import PlanarReferenceArray
from amr_mpc.baseline.pure_pursuit import PurePursuitConfig, pure_pursuit
from amr_mpc.core import config, node_main
from amr_mpc.core.ctrl_select import parse_active
from amr_mpc.core.reference import HoldLatch, ReferenceHorizon, wrap
from amr_mpc.torque.model import TorquePlant


LOOKAHEAD = ("lookahead_time", "lookahead_min", "lookahead_max")


def lookahead_sets(get):
    """parameter 읽기 함수 → {"pp": {lookahead_*}, "adpp": {lookahead_*}}."""
    return {"pp": {k: float(get(k)) for k in LOOKAHEAD}, "adpp": {k: float(get("adpp_" + k)) for k in LOOKAHEAD}}


def check_lookahead(la):
    if not (la["lookahead_time"] >= 0.0 and 0.0 < la["lookahead_min"] <= la["lookahead_max"]):
        raise ValueError(f"룩어헤드 {la} — time ≥ 0, 0 < min ≤ max")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")
        yb = config.load("torque_baseline_a.yaml")
        ya, yad = yb["pure_pursuit"], yb.get("pure_pursuit_adaptive", {})
        ym = config.load("torque_mpc.yaml")
        d = PurePursuitConfig()
        self.declare_parameters("", [
            ("reference_topic", "/dra_mppi/reference_trajectory"),
            ("state_topic", "/state_estimator/odom"),
            ("armed_topic", "/mpc/armed"),
            ("cmd_topic", "/pure_pursuit/cmd_vel"),
            ("state_timeout", float(ym["safety"]["state_timeout"])),
            ("ref_timeout", float(ym["safety"]["ref_timeout"])),
            ("hold_enter_xy", float(ym["hold"]["enter_xy"])),
            ("hold_enter_yaw_deg", float(ym["hold"]["enter_yaw_deg"])),
            ("hold_exit_xy", float(ym["hold"]["exit_xy"])),
            ("hold_exit_yaw_deg", float(ym["hold"]["exit_yaw_deg"])),
            ("hold_stopped_speed", float(ym["hold"]["stopped_speed"])),
            *[(k, float(ya.get(k, getattr(d, k)))) for k in PurePursuitConfig.__dataclass_fields__],
            ("variant_topic", ""),         # 2026-11-28 controller:=all 머리주석. 비우면 위 lookahead_* 하나
            *[("adpp_" + k, float(yad.get(k, ya.get(k, getattr(d, k))))) for k in LOOKAHEAD],
        ])
        g = lambda n: self.get_parameter(n).value
        self.cfg = PurePursuitConfig(**{k: float(g(k)) for k in PurePursuitConfig.__dataclass_fields__})
        self.ref_timeout = float(g("ref_timeout"))
        deg = math.pi / 180.0
        self.latch = HoldLatch(g("hold_enter_xy"), g("hold_enter_yaw_deg") * deg,
                               g("hold_exit_xy"), g("hold_exit_yaw_deg") * deg)
        self.stopped_speed = float(g("hold_stopped_speed"))
        self.k = TorquePlant.from_yaml().k
        self.ref, self.ref_rx, self.ref_frame = None, 0.0, "map"
        self.armed, self.armed_since, self.active = False, 0.0, False

        self.sets = lookahead_sets(g)
        for la in self.sets.values():
            check_lookahead(la)
        self.variant = None                        # variant_topic 을 안 쓰면 None (예전 그대로: lookahead_* 하나)
        if str(g("variant_topic")):
            latched_v = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, str(g("variant_topic")), self.on_variant, latched_v)
        self.add_on_set_parameters_callback(self.on_params)
        self.pub = self.create_publisher(TwistStamped, str(g("cmd_topic")), 10)
        self.pub_status = self.create_publisher(Float64MultiArray, "/pure_pursuit/status", 10)
        self.pub_la = self.create_publisher(PointStamped, "/pure_pursuit/lookahead", 1)
        self.create_subscription(PlanarReferenceArray, str(g("reference_topic")), self.on_ref, 5)
        self.create_subscription(Odometry, str(g("state_topic")), self.on_state, 20)
        armed_topic = str(g("armed_topic"))
        if armed_topic:
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, armed_topic, self.on_armed, latched)
        else:
            self.armed = True
            self.get_logger().warning("armed_topic 이 비어 있다 — 무장 여부를 보지 않는다 (단독 시험 전용)")
        # 할 일 없는 타이머 — 메시지가 끊겨도 spin 이 주기적으로 깨어나 종료 신호(Ctrl+C)를 바로 받게 한다 (slip_logger 와 같은 이유)
        self.create_timer(0.1, lambda: None)
        c = self.cfg
        self.get_logger().info(
            f"퓨어 퍼슛 (비교군 A). {g('reference_topic')} + {g('state_topic')} → {g('cmd_topic')}\n"
            f"  룩어헤드 L_d = clamp({c.lookahead_time:g} s × |v_ref|, {c.lookahead_min:g}, {c.lookahead_max:g}) m"
            f"   진행 보정 k_along {c.k_along:g} /s   회전 구간 |v_ref| < {c.rotate_speed:g} m/s (k_heading {c.k_heading:g})")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def apply_lookahead(self):
        la = self.sets[self.variant or "pp"]
        self.cfg = dataclasses.replace(self.cfg, **la)
        c = self.cfg
        self.get_logger().info(f"룩어헤드 ({self.variant or 'pp'}): L_d = clamp({c.lookahead_time:g} s × |v_ref|, "
                               f"{c.lookahead_min:g}, {c.lookahead_max:g}) m")

    def on_variant(self, m):
        try:
            v = parse_active(m.data)
        except ValueError as e:
            self.get_logger().error(f"제어기 선택 무시: {e}")
            return
        if v in ("pp", "adpp") and v != self.variant:
            self.variant = v
            self.apply_lookahead()

    def on_params(self, params):
        """룩어헤드 여섯 값만 정지 중에. 나머지는 기동 때 읽은 값이라 바꿔도 안 쓰인다 → 거부."""
        names = [p.name for p in params if p.name != "use_sim_time"]
        allowed = set(LOOKAHEAD) | {"adpp_" + k for k in LOOKAHEAD}
        bad = [n for n in names if n not in allowed]
        if bad:
            return SetParametersResult(successful=False, reason=f"{bad} 는 launch 때만 (실행 중: {sorted(allowed)})")
        if not names:
            return SetParametersResult(successful=True)
        if self.armed:
            return SetParametersResult(successful=False, reason="무장 중 — 정지한 뒤")
        sets = {k: dict(v) for k, v in self.sets.items()}
        for p in params:
            if p.name in allowed:
                fam, key = ("adpp", p.name[5:]) if p.name.startswith("adpp_") else ("pp", p.name)
                sets[fam][key] = float(p.value)
        try:
            for la in sets.values():
                check_lookahead(la)
        except ValueError as e:
            return SetParametersResult(successful=False, reason=str(e))
        self.sets = sets
        self.apply_lookahead()
        return SetParametersResult(successful=True)

    def on_armed(self, m):
        if bool(m.data) != self.armed:
            self.armed = bool(m.data)
            self.armed_since = self.now_s()
            self.get_logger().info("게이트 무장 — 추종 가능" if self.armed else "게이트 정지 — 명령 끊음")

    def on_ref(self, m):
        if not m.references:
            return
        rows = [(r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate) for r in m.references]
        try:
            self.ref = ReferenceHorizon.from_rows(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.dt, rows)
        except ValueError as e:
            self.get_logger().warning(f"기준열 무시: {e}", throttle_duration_sec=2.0)
            return
        self.ref_rx = self.now_s()
        self.ref_frame = m.header.frame_id or "map"

    def stop(self, reason):
        if self.active:
            self.get_logger().info(f"정지 (명령 끊음 → wheel_pi DAMPING): {reason}")
        self.active = False

    def on_state(self, m):
        now = self.now_s()
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = st if st > 0.0 else now
        if not self.armed:
            return self.stop("게이트 미무장")
        if self.ref is None:
            return self.stop("기준 없음")
        if self.ref_rx < self.armed_since:
            return self.stop("무장 뒤 새 기준열 대기")
        if now - self.ref_rx > self.ref_timeout:
            self.get_logger().warning(f"기준열 {now - self.ref_rx:.2f} s 없음", throttle_duration_sec=2.0)
            return self.stop("기준열 끊김")
        if m.header.frame_id and self.ref_frame != m.header.frame_id:    # torque_mpc_node 와 같은 검사 (R5·R7 리뷰)
            self.get_logger().error(f"기준열 좌표계 '{self.ref_frame}' ≠ 상태 좌표계 '{m.header.frame_id}' — 명령 끊음",
                                    throttle_duration_sec=2.0)
            return self.stop("기준·상태 좌표계 다름")

        p = m.pose.pose
        pose = np.array([p.position.x, p.position.y, yaw_of(p.orientation)])
        tw = m.twist.twist
        Xr, _ = self.ref.at(t)
        Xe = np.array([Xr[0] - pose[0], Xr[1] - pose[1], wrap(Xr[2] - pose[2])])
        stopped = math.hypot(tw.linear.x, tw.linear.y) < self.stopped_speed and \
            abs(tw.angular.z) * self.k < self.stopped_speed
        was_idle = self.latch.idle
        if self.latch.update(self.ref.is_hold, Xe, stopped):
            if not was_idle:
                self.get_logger().info(f"hold 진입: 오차 {math.hypot(Xe[0], Xe[1])*1e3:.1f} mm, "
                                       f"{math.degrees(Xe[2]):.2f}° — 명령 끊음 (DAMPING)")
            self.active = False
            self.publish_status(now, Xe, 0.0, 0.0, None)
            return
        if was_idle:
            self.get_logger().info(f"hold 해제: 오차 {math.hypot(Xe[0], Xe[1])*1e3:.1f} mm — 추종 재개")
        if not self.active:
            self.active = True
            self.get_logger().info(f"추종 시작. 기준열 {self.ref.T}×{self.ref.dt:.2f} s")

        v, w, info = pure_pursuit(self.cfg, self.ref.at, t, pose)
        cmd = TwistStamped()
        cmd.header.stamp = m.header.stamp            # 계산에 쓴 상태 추정값의 시각 (MPC 와 같다)
        cmd.header.frame_id = "base_footprint"
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.pub.publish(cmd)
        self.publish_status(now, Xe, v, w, info)

    def publish_status(self, now, Xe, v, w, info):
        s = Float64MultiArray()
        if info is None:
            s.data = [now, *map(float, Xe), 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]      # mode 2 = hold
        else:
            s.data = [now, *map(float, Xe), float(v), float(w), 0.0 if info["mode"] == "pursuit" else 1.0,
                      float(info["L"]), float(info["kappa"]), float(info["lead_s"])]
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = self.ref_frame
            pt.point.x, pt.point.y = float(info["target"][0]), float(info["target"][1])
            self.pub_la.publish(pt)
        self.pub_status.publish(s)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuitNode()
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

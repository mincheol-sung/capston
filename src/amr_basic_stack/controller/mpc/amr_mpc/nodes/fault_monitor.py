#!/usr/bin/env python3
# fault_monitor — 무장 중 위험한 상황을 보면 게이트를 내려(/mpc/arm = false) 로봇을 DAMPING 으로 세운다 (구조도 v3).
#   ros2 run amr_mpc fault_monitor_node
# ---------------------------------------------------------------
# 검사 (무장된 동안만. 한 번 걸리면 다시 'a' 로 무장할 때까지 기록이 남는다)
#   pose_jump      /state_estimator/status 의 SLAM 포즈 튐 누적 수가 늘었다 (전역 최적화·재측위로 포즈가 계단처럼 바뀜)
#   track_error    /torque_mpc/status 의 기준–상태 위치 오차가 max_track_error 를 넘었다
#                  (비교군 A 는 track_status_topic:=/pure_pursuit/status — 같은 꼴 [t, e_x, e_y, …])
#   motor_error    /wheel_states.error 에 0 이 아닌 값 (드라이버 에러)
#   wheel_timeout  /wheel_states 가 timeout 동안 안 왔다
#   state_timeout  /state_estimator/odom 이 timeout 동안 안 왔다
# 동작: /mpc/arm 에 false → 게이트가 DAMPING 으로 세운다 (MPC·기준궤적도 무장 해제를 보고 멈춘다).
#       /fault_monitor/fault (String, latched) 에 이유를 남기고 ERROR 로그.
# 문턱은 아래 파라미터 (launch 에서 덮어쓰기). figure8 의 abort_error(1.6 m)보다 track_error(1.5 m)를 먼저 건다.
# 텔레옵 모드 (2026-11-28, teleop_armed_topic — 비우면 안 본다): 게이트가 패드 텔레옵을 중계하는 동안 (/teleop/armed true)
#   motor_error · wheel_timeout 만 본다 (추종 오차·포즈 튐·추정기 끊김은 자율 주행 검사). 걸리면 똑같이 /mpc/arm false — 게이트가 둘 다 끈다.
# controller:=all (2026-11-28, nodes/controller_select.py): select_topic 을 주면 track_status_topic 대신 select_status 의 토픽을 다 받고
#   /ctrl/active 가 고른 제어기의 오차만 본다. max_track_error 는 실행 중 바꿀 수 있다 (정지 중만 — tools/pit.py 프리셋 s1·s2·s3).
# 2026-11-16 0.30 → 1.5 m (사용자 결정): C 가 한계 넘는 기준에서 미끄러지는 대신 늦어지는 모습을 끝까지 남긴다.
#   로봇이 경로에서 1.5 m 까지 벗어날 수 있다 — 두 사람 정지 절차로 지켜본다.
# ---------------------------------------------------------------
import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64MultiArray, String

from amr_interfaces.msg import WheelStates
from amr_mpc.core import node_main
from amr_mpc.core.ctrl_select import TopicSelect, parse_active


def check_faults(now, armed, cfg, st):
    """순수 판정 함수. st: 최근 관측 dict. 반환: 걸린 이유 목록 (빈 목록이면 정상)."""
    if not armed:
        return []
    out = []
    if cfg["trip_on_pose_jump"] and st.get("jump_count", 0) > st.get("jump_count_at_arm", 0):
        out.append(f"pose_jump: SLAM 포즈 튐 {st['jump_count'] - st['jump_count_at_arm']}회")
    if st.get("track_error") is not None and st["track_error"] > cfg["max_track_error"]:
        out.append(f"track_error: 기준과 {st['track_error']*100:.1f} cm (한계 {cfg['max_track_error']*100:.1f} cm)")
    if any(st.get("motor_error", [])):
        out.append(f"motor_error: 드라이버 에러 코드 {list(st['motor_error'])}")
    keys = [("wheel_rx", "wheel_timeout")]
    if cfg.get("check_state_timeout", True):        # 2026-11-23: 텔레옵(teleop.launch.py)은 추정기 없이 돈다
        keys.append(("state_rx", "state_timeout"))
    for key, name in keys:
        t = st.get(key)
        if t is None or now - t > cfg["timeout"]:
            out.append(f"{name}: {'한 번도 안 옴' if t is None else f'{now - t:.2f} s 동안 안 옴'}")
    return out


class FaultMonitorNode(Node):
    def __init__(self):
        super().__init__("fault_monitor_node")
        self.declare_parameters("", [
            ("arm_topic", "/mpc/arm"),
            ("armed_topic", "/mpc/armed"),
            ("max_track_error", 1.5),      # [m]
            ("timeout", 0.3),              # [s]
            ("trip_on_pose_jump", True),
            ("rate_hz", 20.0),
            # 추종 오차를 읽을 status 토픽 — [t, e_x, e_y, ...] 꼴. MPC = /torque_mpc/status, 비교군 A = /pure_pursuit/status
            ("track_status_topic", "/torque_mpc/status"),
            # false 면 /state_estimator/odom 끊김을 안 본다 — 추정기 없이 도는 텔레옵용 (2026-11-23). 모터 에러·바퀴 끊김은 그대로 본다
            ("check_state_timeout", True),
            ("teleop_armed_topic", ""),    # 2026-11-28 머리주석. 비우면 텔레옵 모드를 안 본다
            ("select_topic", ""),          # 2026-11-28 controller:=all 머리주석. 비우면 track_status_topic 하나
            ("select_status", ["mpc=/torque_mpc/status", "pp=/pure_pursuit/status", "adpp=/pure_pursuit/status"]),
        ])
        g = lambda n: self.get_parameter(n).value
        self.cfg = {"max_track_error": float(g("max_track_error")), "timeout": float(g("timeout")),
                    "trip_on_pose_jump": bool(g("trip_on_pose_jump")),
                    "check_state_timeout": bool(g("check_state_timeout"))}
        self.armed, self.teleop, self.st = False, False, {}
        self.teleop_cfg = {**self.cfg, "trip_on_pose_jump": False, "check_state_timeout": False}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_arm = self.create_publisher(Bool, str(g("arm_topic")), 10)
        self.pub_fault = self.create_publisher(String, "/fault_monitor/fault", latched)
        self.create_subscription(Bool, str(g("armed_topic")), self.on_armed, latched)
        if str(g("teleop_armed_topic")):
            self.create_subscription(Bool, str(g("teleop_armed_topic")), self.on_teleop, latched)
        self.create_subscription(Float64MultiArray, "/state_estimator/status", self.on_est_status, 10)
        self.sel = None
        if str(g("select_topic")):
            self.sel = TopicSelect(list(g("select_status")))
            for topic in self.sel.topics():
                self.create_subscription(Float64MultiArray, topic,
                                         lambda m, t=topic: self.on_mpc_status(m) if self.sel.passes(t) else None, 10)
            self.create_subscription(String, str(g("select_topic")), self.on_select, latched)
        else:
            self.create_subscription(Float64MultiArray, str(g("track_status_topic")), self.on_mpc_status, 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.create_subscription(WheelStates, "/wheel_states", self.on_wheels, 10)
        self.create_subscription(Odometry, "/state_estimator/odom", self.on_state, 10)
        self.create_timer(1.0 / float(g("rate_hz")), self.on_tick)
        self.get_logger().info(
            f"fault_monitor: 무장 중 검사 — 포즈 튐 {'켬' if self.cfg['trip_on_pose_jump'] else '끔'}, "
            f"추종 오차 > {self.cfg['max_track_error']*100:.0f} cm, 모터 에러, "
            f"{'바퀴·추정기' if self.cfg['check_state_timeout'] else '바퀴'} 끊김 > {self.cfg['timeout']} s → /mpc/arm false")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_armed(self, m):
        if bool(m.data) and not self.armed:                  # 무장하는 순간을 기준으로 센다
            self.st["jump_count_at_arm"] = self.st.get("jump_count", 0)
            self.st["track_error"] = None
        self.armed = bool(m.data)

    def on_select(self, m):
        try:
            topic = self.sel.select(parse_active(m.data))
        except ValueError as e:
            self.sel.active = None
            self.get_logger().error(f"제어기 선택 무시: {e}")
            return
        self.st["track_error"] = None
        self.get_logger().info(f"추종 오차: {topic} ({self.sel.active})")

    def on_params(self, params):
        """max_track_error 만 실행 중에 (정지 중). 나머지는 launch 때 읽은 값 그대로라 바꿔도 안 쓰인다 → 거부."""
        for p in params:
            if p.name == "use_sim_time":
                continue
            if p.name != "max_track_error":
                return SetParametersResult(successful=False, reason=f"{p.name} 는 launch 때만 (실행 중: max_track_error)")
            if self.armed:
                return SetParametersResult(successful=False, reason="무장 중 — 정지한 뒤")
            if not 0.05 <= float(p.value) <= 1.5:
                return SetParametersResult(successful=False, reason=f"max_track_error {p.value} — 0.05~1.5 m")
        for p in params:
            if p.name == "max_track_error":
                self.cfg["max_track_error"] = self.teleop_cfg["max_track_error"] = float(p.value)
                self.get_logger().info(f"추종 오차 한계 {float(p.value)*100:.0f} cm")
        return SetParametersResult(successful=True)

    def on_teleop(self, m):
        self.teleop = bool(m.data)

    def on_est_status(self, m):
        if len(m.data) > 7:
            self.st["jump_count"] = int(m.data[7])

    def on_mpc_status(self, m):
        if len(m.data) > 2:
            self.st["track_error"] = math.hypot(m.data[1], m.data[2])

    def on_wheels(self, m):
        self.st["wheel_rx"] = self.now()
        self.st["motor_error"] = list(m.error)

    def on_state(self, _m):
        self.st["state_rx"] = self.now()

    def on_tick(self):
        faults = check_faults(self.now(), self.armed, self.cfg, self.st)
        if not faults and self.teleop:                        # 텔레옵: 모터 에러·바퀴 끊김만 (추종 오차 기록은 빼고 본다)
            faults = check_faults(self.now(), True, self.teleop_cfg, {**self.st, "track_error": None})
        if not faults:
            return
        reason = " / ".join(faults)
        self.get_logger().error(f"정지: {reason}")
        self.pub_arm.publish(Bool(data=False))
        self.pub_fault.publish(String(data=reason))
        self.armed = self.teleop = False                      # 게이트의 /mpc/armed 가 돌아오기 전에 반복 발행하지 않게


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FaultMonitorNode()
    try:
        node_main.spin(node)          # Ctrl+C·SIGTERM 이 메시지 변환 중에 떨어져도 정상 종료 (R9 리뷰)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

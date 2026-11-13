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
# 문턱은 아래 파라미터 (launch 에서 덮어쓰기). figure8 의 abort_error(0.35 m)보다 track_error(0.30 m)를 먼저 건다.
# ---------------------------------------------------------------
import math
import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64MultiArray, String

from amr_interfaces.msg import WheelStates


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
    for key, name in (("wheel_rx", "wheel_timeout"), ("state_rx", "state_timeout")):
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
            ("max_track_error", 0.30),     # [m]
            ("timeout", 0.3),              # [s]
            ("trip_on_pose_jump", True),
            ("rate_hz", 20.0),
            # 추종 오차를 읽을 status 토픽 — [t, e_x, e_y, ...] 꼴. MPC = /torque_mpc/status, 비교군 A = /pure_pursuit/status
            ("track_status_topic", "/torque_mpc/status"),
        ])
        g = lambda n: self.get_parameter(n).value
        self.cfg = {"max_track_error": float(g("max_track_error")), "timeout": float(g("timeout")),
                    "trip_on_pose_jump": bool(g("trip_on_pose_jump"))}
        self.armed, self.st = False, {}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_arm = self.create_publisher(Bool, str(g("arm_topic")), 10)
        self.pub_fault = self.create_publisher(String, "/fault_monitor/fault", latched)
        self.create_subscription(Bool, str(g("armed_topic")), self.on_armed, latched)
        self.create_subscription(Float64MultiArray, "/state_estimator/status", self.on_est_status, 10)
        self.create_subscription(Float64MultiArray, str(g("track_status_topic")), self.on_mpc_status, 10)
        self.create_subscription(WheelStates, "/wheel_states", self.on_wheels, 10)
        self.create_subscription(Odometry, "/state_estimator/odom", self.on_state, 10)
        self.create_timer(1.0 / float(g("rate_hz")), self.on_tick)
        self.get_logger().info(
            f"fault_monitor: 무장 중 검사 — 포즈 튐 {'켬' if self.cfg['trip_on_pose_jump'] else '끔'}, "
            f"추종 오차 > {self.cfg['max_track_error']*100:.0f} cm, 모터 에러, 센서 끊김 > {self.cfg['timeout']} s → /mpc/arm false")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_armed(self, m):
        if bool(m.data) and not self.armed:                  # 무장하는 순간을 기준으로 센다
            self.st["jump_count_at_arm"] = self.st.get("jump_count", 0)
            self.st["track_error"] = None
        self.armed = bool(m.data)

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
        if not faults:
            return
        reason = " / ".join(faults)
        self.get_logger().error(f"정지: {reason}")
        self.pub_arm.publish(Bool(data=False))
        self.pub_fault.publish(String(data=reason))
        self.armed = False                                    # 게이트의 /mpc/armed 가 돌아오기 전에 반복 발행하지 않게


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FaultMonitorNode()
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

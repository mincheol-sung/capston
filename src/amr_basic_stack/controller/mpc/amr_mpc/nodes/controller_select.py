#!/usr/bin/env python3
# controller_select — controller:=all 에서 회차마다 제어기를 고른다 (2026-11-28 사용자 결정 "A launch 하나").
#   ros2 run amr_mpc controller_select_node --ros-args -p active:=pp
# ---------------------------------------------------------------
# controller:=all launch 는 세 제어기를 다 띄운다: torque_mpc_node (C) · pure_pursuit_node + wheel_pi_node (PP·ADPP 공용).
# 이 노드는 무장 신호만 나눈다 — 토크·상태 토픽은 중계하지 않는다 (torque_output · fault_monitor 가 /ctrl/active 를 보고
# 고른 제어기의 토픽만 받는다. 토크 경로에 노드 하나를 더 끼우지 않으려고).
#
#   /mpc/armed           (Bool, latched)   게이트 무장 여부 (keyboard_command_gate)
#      ↓  고른 제어기 쪽에만 그대로, 다른 쪽은 늘 false
#   /ctrl/mpc/armed      (Bool, latched)   torque_mpc_node 의 armed_topic
#   /ctrl/pp/armed       (Bool, latched)   pure_pursuit_node · wheel_pi_node 의 armed_topic
#   /ctrl/active         (String JSON, latched)  {"t", "active": mpc|pp|adpp, "family": mpc|pp, "armed", "event"}
#                        event = start (기동) · select (바꿈) · arm · disarm — bag 에 남아 회차마다 어느 제어기였는지 가른다
#
# 고르지 않은 제어기는 무장 신호가 false 라 미무장 때와 똑같이 쉰다: torque_mpc_node 는 풀지 않고 DAMPING + reset
#   (torque_mpc_node.cpp on_state), pure_pursuit 은 명령을 끊고, wheel_pi 는 DAMPING + PI 리셋.
# parameter active (mpc | pp | adpp) — rqt_reconfigure · tools/seq.py 가 바꾼다. 무장 중에는 거부 (회차 도중에 제어기가 바뀌지 않게).
#   pp 와 adpp 는 같은 pure_pursuit_node 다. 룩어헤드 묶음은 pure_pursuit_node 가 /ctrl/active 를 보고 고른다.
# ---------------------------------------------------------------
import json

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from amr_mpc.core import node_main
from amr_mpc.core.ctrl_select import family, route

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ControllerSelectNode(Node):
    def __init__(self):
        super().__init__("controller_select_node")
        self.declare_parameters("", [
            ("active", "mpc"),
            ("armed_topic", "/mpc/armed"),
            ("mpc_armed_topic", "/ctrl/mpc/armed"),
            ("pp_armed_topic", "/ctrl/pp/armed"),
            ("active_topic", "/ctrl/active"),
        ])
        g = lambda n: self.get_parameter(n).value
        self.active = str(g("active")).strip().lower()
        family(self.active)                                   # 틀린 값이면 기동 때 끝낸다
        self.armed = False
        self.pub = {"mpc": self.create_publisher(Bool, str(g("mpc_armed_topic")), LATCHED),
                    "pp": self.create_publisher(Bool, str(g("pp_armed_topic")), LATCHED)}
        self.pub_active = self.create_publisher(String, str(g("active_topic")), LATCHED)
        self.create_subscription(Bool, str(g("armed_topic")), self.on_armed, LATCHED)
        self.add_on_set_parameters_callback(self.on_params)
        self.create_timer(0.1, lambda: None)                  # Ctrl+C 를 바로 받게 (다른 노드와 같은 이유)
        self.forward()
        self.publish_active("start")
        self.get_logger().info(f"제어기 선택: {self.active} — {g('armed_topic')} → {g('mpc_armed_topic')} · {g('pp_armed_topic')}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def forward(self):
        for f, v in route(self.active, self.armed).items():
            self.pub[f].publish(Bool(data=v))

    def publish_active(self, event):
        st = {"t": self.now(), "active": self.active, "family": family(self.active), "armed": self.armed, "event": event}
        self.pub_active.publish(String(data=json.dumps(st)))

    def on_armed(self, m):
        armed = bool(m.data)
        if armed == self.armed:
            return
        self.armed = armed
        self.forward()
        self.publish_active("arm" if armed else "disarm")

    def on_params(self, params):
        for p in params:
            if p.name != "active":
                continue
            v = str(p.value).strip().lower()
            if self.armed:
                return SetParametersResult(successful=False, reason="무장 중 — 's' 로 정지한 뒤 바꾼다 (회차 도중 제어기 교체 금지)")
            try:
                family(v)
            except ValueError as e:
                return SetParametersResult(successful=False, reason=str(e))
            if v != self.active:
                self.active = v
                self.forward()
                self.publish_active("select")
                self.get_logger().info(f"제어기 선택: {v}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ControllerSelectNode()
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

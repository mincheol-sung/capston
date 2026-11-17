#!/usr/bin/env python3
# torque_viz — 실차 토크 실험을 RViz 로 보기 위한 표시 전용 노드 (제어에는 쓰지 않는다, 2026-11-16).
#   ros2 launch amr_mpc torque_viz.launch.py        (랩 PC 에서 RViz 와 같이 띄운다)
# ---------------------------------------------------------------
#   /state_estimator/odom (Odometry, pose = map 좌표)  →  TF map → base_footprint   (tf_rate Hz 로 솎아서)
#                                                     →  /torque_viz/trail (nav_msgs/Path) 지나간 경로
#   /mpc/armed (Bool, latched)                         →  false → true 가 될 때 지나간 경로를 비운다
#
# 왜 필요한가
#   · 실험 스택에는 움직이는 TF 가 없다 (2026-11-16 bag 의 /tf 메시지 0 개). Cartographer 는 publish_to_tf = false,
#     상태 추정기는 Odometry 만 낸다. 그래서 RViz 고정 프레임 map 에서 로봇 모델·라이다 스캔을 놓을 자리가 없다.
#     이 노드가 추정기 포즈로 map → base_footprint 를 내면 base_footprint → base_link → laser_* (URDF 고정 TF) 가 이어진다.
#   · 8자 기준 경로는 figure8_reference_node 가 /figure8/path 로 이미 낸다. 실제로 지나간 경로는 없어서 여기서 쌓는다.
#
# 주의
#   · base_footprint 의 부모를 하나 더 만들면 TF 가 깨진다. localization.launch.py (ekf_filter_node 의 odom → base_footprint)
#     와 같이 띄우지 말 것 — 토크 실험 안내(torque_experiment.launch.py)와 같은 규칙이다.
#   · 표시용이라 추정기 출력이 끊기면 TF 도 멈춘다 (마지막 자리에 남는다). 제어 판단에 쓰지 말 것.
# ---------------------------------------------------------------
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from amr_mpc.core import node_main


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TorqueVizNode(Node):
    def __init__(self):
        super().__init__("torque_viz_node")
        self.declare_parameters("", [
            ("odom_topic", "/state_estimator/odom"),
            ("armed_topic", "/mpc/armed"),
            ("trail_topic", "/torque_viz/trail"),
            ("publish_tf", True),          # false 면 지나간 경로만 (다른 노드가 map → base_footprint 를 낼 때)
            ("tf_rate", 30.0),             # [Hz] TF 를 내는 최대 빈도. 추정기는 100 Hz — 표시에는 30 이면 충분하다
            ("trail_min_step", 0.005),     # [m] 이만큼 움직여야 경로에 점을 더한다
            ("trail_min_yaw_deg", 2.0),    # [deg] 또는 이만큼 돌면
            ("trail_period", 0.2),         # [s] 지나간 경로 발행 주기
            ("trail_max_points", 20000),   # 넘으면 앞쪽부터 버린다 (0.5 cm 간격이면 100 m)
        ])
        g = lambda n: self.get_parameter(n).value
        self.publish_tf = bool(g("publish_tf"))
        self.tf_dt = 1.0 / float(g("tf_rate")) if float(g("tf_rate")) > 0 else 0.0
        self.min_step = float(g("trail_min_step"))
        self.min_yaw = math.radians(float(g("trail_min_yaw_deg")))
        self.max_points = int(g("trail_max_points"))

        self.tfb = TransformBroadcaster(self) if self.publish_tf else None
        self.pub_trail = self.create_publisher(Path, str(g("trail_topic")), 1)
        self.create_subscription(Odometry, str(g("odom_topic")), self.on_odom, 20)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, str(g("armed_topic")), self.on_armed, latched)
        self.create_timer(float(g("trail_period")), self.publish_trail)

        self.trail = Path()
        self.trail.header.frame_id = "map"
        self.last_tf_t = None
        self.last_pt = None             # (x, y, yaw) 경로에 마지막으로 더한 점
        self.armed = None
        self.dirty = False
        self.get_logger().info(
            f"torque_viz: {g('odom_topic')} → "
            + (f"TF map → base_footprint ({g('tf_rate'):.0f} Hz), " if self.publish_tf else "")
            + f"{g('trail_topic')} (무장할 때 비움)")

    def on_armed(self, m):
        if m.data and self.armed is False:
            self.trail.poses = []
            self.last_pt = None
            self.dirty = True
            self.get_logger().info("무장 — 지나간 경로를 비운다")
        self.armed = bool(m.data)

    def on_odom(self, m):
        frame = m.header.frame_id or "map"
        child = m.child_frame_id or "base_footprint"
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = stamp_s(m.header)
        if self.tfb is not None and (self.last_tf_t is None or t - self.last_tf_t >= self.tf_dt - 1e-4 or t < self.last_tf_t):
            self.last_tf_t = t
            tf = TransformStamped()
            tf.header.stamp = m.header.stamp
            tf.header.frame_id = frame
            tf.child_frame_id = child
            tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
            tf.transform.rotation = q
            self.tfb.sendTransform(tf)
        yaw = yaw_of(q)
        if self.last_pt is not None:
            x0, y0, yaw0 = self.last_pt
            dyaw = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
            if math.hypot(p.x - x0, p.y - y0) < self.min_step and dyaw < self.min_yaw:
                return
        ps = PoseStamped()
        ps.header = m.header
        ps.pose = m.pose.pose
        self.trail.header.frame_id = frame
        self.trail.poses.append(ps)
        if len(self.trail.poses) > self.max_points:
            del self.trail.poses[: len(self.trail.poses) - self.max_points]
        self.last_pt = (p.x, p.y, yaw)
        self.dirty = True

    def publish_trail(self):
        if not self.dirty:
            return
        self.dirty = False
        self.trail.header.stamp = self.get_clock().now().to_msg()
        self.pub_trail.publish(self.trail)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TorqueVizNode()
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

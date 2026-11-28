#!/usr/bin/env python3
# torque_viz — 실차 토크 실험을 RViz 로 보기 위한 표시 전용 노드 (제어에는 쓰지 않는다, 2026-11-16).
#   ros2 launch amr_mpc torque_viz.launch.py        (랩 PC 에서 RViz 와 같이 띄운다)
# ---------------------------------------------------------------
#   /state_estimator/odom (Odometry, pose = map 좌표)  →  TF map → base_footprint   (tf_rate Hz 로 솎아서)
#                                                     →  /torque_viz/trail (nav_msgs/Path) 지나간 경로 (estimator)
#   /tracked_pose (PoseStamped, 카메라 IMU 점)          →  /torque_viz/trail_slam   SLAM 포즈를 base_footprint 로 옮긴 경로 (2026-11-26)
#   /wheel_states + /imu/base_link ω_z + /tracked_pose →  /torque_viz/trail_smooth 바퀴·자이로 적분 + SLAM 저역 보정 (점프 없음, 2026-11-26)
#                                                         core/trail_fusion.py. estimator 경로의 좌우 흔들림은 SLAM 보정 점프라
#                                                         실제 운동이 아니다 — analysis/torque_mpc_real/2026-11-26_trail/RESULTS_KO.md
#   /mpc/armed (Bool, latched)                         →  false → true 가 될 때 지나간 경로를 모두 비운다
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
from amr_interfaces.msg import WheelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from amr_mpc.core import config, node_main
from amr_mpc.core.trail_fusion import SmoothTrail, mecanum_fwd, slam_to_base


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_of(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Trail:
    """무장 뒤 지나간 경로 하나 — min_step·min_yaw 만큼 움직일 때만 점을 더한다."""

    def __init__(self, node, topic, min_step, min_yaw, max_points):
        self.pub = node.create_publisher(Path, topic, 1)
        self.min_step, self.min_yaw, self.max_points = min_step, min_yaw, max_points
        self.path = Path()
        self.path.header.frame_id = "map"
        self.last = None
        self.dirty = False

    def clear(self):
        self.path.poses = []
        self.last = None
        self.dirty = True

    def add(self, header, x, y, yaw, pose=None):
        if self.last is not None:
            x0, y0, yaw0 = self.last
            dyaw = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
            if math.hypot(x - x0, y - y0) < self.min_step and dyaw < self.min_yaw:
                return
        ps = PoseStamped()
        ps.header = header
        if pose is not None:
            ps.pose = pose
        else:
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = quat_of(yaw)
        self.path.header.frame_id = header.frame_id or "map"
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_points:
            del self.path.poses[: len(self.path.poses) - self.max_points]
        self.last = (x, y, yaw)
        self.dirty = True

    def publish(self, stamp):
        if not self.dirty:
            return
        self.dirty = False
        self.path.header.stamp = stamp
        self.pub.publish(self.path)


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
            ("slam_topic", "/tracked_pose"),
            ("wheel_topic", "/wheel_states"),
            ("imu_topic", "/imu/base_link"),
            ("trail_slam_topic", "/torque_viz/trail_slam"),
            ("trail_smooth_topic", "/torque_viz/trail_smooth"),
            ("smooth_tau", 0.5),           # [s] 점프 없는 경로의 SLAM 보정 시상수 (bag 재생으로 고름, core/trail_fusion.py)
        ])
        g = lambda n: self.get_parameter(n).value
        self.publish_tf = bool(g("publish_tf"))
        self.tf_dt = 1.0 / float(g("tf_rate")) if float(g("tf_rate")) > 0 else 0.0
        self.min_step = float(g("trail_min_step"))
        self.min_yaw = math.radians(float(g("trail_min_yaw_deg")))
        self.max_points = int(g("trail_max_points"))

        self.tfb = TransformBroadcaster(self) if self.publish_tf else None
        mk = lambda topic: Trail(self, str(g(topic)), self.min_step, self.min_yaw, self.max_points)
        self.trail, self.trail_slam, self.trail_smooth = mk("trail_topic"), mk("trail_slam_topic"), mk("trail_smooth_topic")
        geo = config.section("geometry", "torque_plant.yaml")
        est = config.section("estimator", "torque_estimator.yaml")
        self.r_imu = (float(est["imu_offset_x"]), float(est["imu_offset_y"]))   # /tracked_pose 점 (base_footprint 기준)
        self.smooth = SmoothTrail(mecanum_fwd(geo["wheel_radius_m"], geo["wheel_base_m"], geo["robot_width_m"]),
                                  self.r_imu, tau=float(g("smooth_tau")))
        self.create_subscription(Odometry, str(g("odom_topic")), self.on_odom, 20)
        self.create_subscription(PoseStamped, str(g("slam_topic")), self.on_slam, 20)
        self.create_subscription(WheelStates, str(g("wheel_topic")), self.on_wheels, 50)
        self.create_subscription(Imu, str(g("imu_topic")), self.on_imu, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, str(g("armed_topic")), self.on_armed, latched)
        self.create_timer(float(g("trail_period")), self.publish_trail)

        self.last_tf_t = None
        self.armed = None
        self.get_logger().info(
            f"torque_viz: {g('odom_topic')} → "
            + (f"TF map → base_footprint ({g('tf_rate'):.0f} Hz), " if self.publish_tf else "")
            + f"{g('trail_topic')} · {g('trail_slam_topic')} · {g('trail_smooth_topic')} (무장할 때 비움), "
            + f"SLAM 점 → base_footprint ({self.r_imu[0]:.4f}, {self.r_imu[1]:.4f}) m, smooth τ {g('smooth_tau'):g} s")

    def on_armed(self, m):
        if m.data and self.armed is False:
            for tr in (self.trail, self.trail_slam, self.trail_smooth):
                tr.clear()
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
        self.trail.add(m.header, p.x, p.y, yaw_of(q), m.pose.pose)

    def on_slam(self, m):
        p, q = m.pose.position, m.pose.orientation
        t, yaw = stamp_s(m.header), yaw_of(q)
        self.smooth.on_slam(t, p.x, p.y, yaw)
        bx, by, bp = slam_to_base(p.x, p.y, yaw, *self.r_imu)
        self.trail_slam.add(m.header, bx, by, bp)

    def on_imu(self, m):
        self.smooth.on_gyro(m.angular_velocity.z)

    def on_wheels(self, m):
        if len(m.velocity) != 4:
            return
        pose = self.smooth.on_wheels(stamp_s(m.header), m.velocity)
        if pose is not None:
            m.header.frame_id = "map"
            self.trail_smooth.add(m.header, *pose)

    def publish_trail(self):
        now = self.get_clock().now().to_msg()
        for tr in (self.trail, self.trail_slam, self.trail_smooth):
            tr.publish(now)


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

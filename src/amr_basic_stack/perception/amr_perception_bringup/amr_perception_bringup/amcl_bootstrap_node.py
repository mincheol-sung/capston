#!/usr/bin/env python3
"""amcl_bootstrap_node — AMCL 전역 측위로 cartographer 초기 위치를 자동으로 잡는다.

RViz "2D Pose Estimate" 를 손으로 찍는 대신, AMCL 에 전역 측위를 걸고 파티클이
수렴하면 그 pose 를 /initialpose 로 내보낸다. 그 뒤는 기존 set_initial_pose_node
가 그대로 처리한다 — FinishTrajectory -> StartTrajectory(use_initial_pose) ->
ekf_carto set_pose.

    map_server /map ─┐
                     ├─► amcl (tf_broadcast: false) ─► /amcl_pose
    /scan ───────────┘                                    │
                                                          ▼
    /odometry/filtered ──► amcl_bootstrap_node (수렴 판정) ──► /initialpose
                                                                  │
                                                                  ▼
                                                    set_initial_pose_node
                                                    -> cartographer 궤적 재시작

★ AMCL 은 map -> odom TF 를 내면 안 된다 (amcl.yaml 의 tf_broadcast: false).
  그 TF 의 주인은 ekf_carto 이고, 둘이 같이 내면 TF 트리가 깨진다. AMCL 은
  /amcl_pose 만 내고 좌표 변환은 cartographer 계열이 계속 소유한다.

★ 로봇을 움직여야 한다. AMCL 은 update_min_d/update_min_a 를 넘는 이동이 있어야
  파티클을 갱신한다. 전역 측위를 건 뒤 조이스틱이나 텔레옵으로 몇 미터
  주행시키면 수렴한다. 제자리 회전만으로는 대칭 구간에서 엉뚱하게 붙는다.
"""

import math

from amr_perception_bringup.amcl_convergence import (
    ConvergenceTracker, MapMatcher)

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid, Odometry

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from std_srvs.srv import Empty

import tf2_ros


def _quaternion_yaw(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))


def _rotate(points, yaw):
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return np.column_stack((
        cos_yaw * points[:, 0] - sin_yaw * points[:, 1],
        sin_yaw * points[:, 0] + cos_yaw * points[:, 1],
    ))


class AmclBootstrapNode(Node):
    """AMCL 이 수렴하면 그 pose 를 초기 위치로 한 번 넘긴다."""

    def __init__(self):
        super().__init__('amcl_bootstrap_node')

        self.map_frame = self._declare('map_frame', 'map')
        amcl_pose_topic = self._declare('amcl_pose_topic', '/amcl_pose')
        odom_topic = self._declare('odometry_topic', '/odometry/filtered')
        self.initialpose_topic = self._declare(
            'initialpose_topic', '/initialpose')
        self.global_localization_service = self._declare(
            'global_localization_service', '/reinitialize_global_localization')

        # 전역 측위를 스스로 걸지 여부. false 면 사용자가 직접 서비스를 부른
        # 뒤 수렴만 지켜본다.
        self.trigger_global_localization = bool(
            self._declare('trigger_global_localization', True))
        self.trigger_delay = float(self._declare('trigger_delay', 3.0))

        # 한 번 넘기고 끝낼지. true 가 기본이다 — 주행 중에 초기 위치를 다시
        # 밀어 넣으면 cartographer 궤적이 끊기고 EKF 가 리셋된다.
        self.oneshot = bool(self._declare('oneshot', True))

        self.tracker = ConvergenceTracker(
            position_std=float(self._declare('position_std', 0.15)),
            yaw_std=math.radians(float(self._declare('yaw_std_degrees', 8.0))),
            required_samples=int(self._declare('required_samples', 5)),
            min_travel=float(self._declare('min_travel', 1.0)),
        )
        self.status_period = float(self._declare('status_period', 3.0))

        # ── 스캔-지도 정합 검증 ─────────────────────────────────────────
        # 공분산만 보면 AMCL 이 자신 있게 틀린 곳에 수렴한 것을 못 거른다.
        # 수렴한 pose 에 현재 스캔을 얹어 벽에 붙는 비율을 센다.
        self.verify_with_scan = bool(self._declare('verify_with_scan', True))
        self.min_match_score = float(self._declare('min_match_score', 0.55))
        self.match_distance = float(self._declare('match_distance', 0.20))
        self.min_match_points = int(self._declare('min_match_points', 40))
        self.base_frame = self._declare('base_frame', 'base_footprint')
        map_topic = self._declare('map_topic', '/map')
        scan_topic = self._declare('scan_topic', '/scan')
        # 검증에 실패하면 전역 측위를 다시 건다. 최대 몇 번까지.
        self.max_retries = int(self._declare('max_retries', 3))

        # set_initial_pose_node 의 구독자가 BEST_EFFORT 다. RViz 의
        # "2D Pose Estimate" 와 같은 QoS 로 맞춰야 받는다.
        initialpose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, initialpose_qos)

        # AMCL 은 /amcl_pose 를 TRANSIENT_LOCAL RELIABLE 로 낸다.
        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_pose_topic,
            self._on_amcl_pose, amcl_qos)
        self.create_subscription(Odometry, odom_topic, self._on_odometry, 20)

        if self.verify_with_scan:
            map_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                OccupancyGrid, map_topic, self._on_map, map_qos)
            self.create_subscription(
                LaserScan, scan_topic, self._on_scan,
                QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1))
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._matcher = None
        self._last_scan = None
        self._retries = 0

        self.global_localization_client = self.create_client(
            Empty, self.global_localization_service)

        self._last_position = None
        self._published = False
        self._last_status = 0.0
        self._triggered = False
        if self.trigger_global_localization:
            self.create_timer(self.trigger_delay, self._trigger_once)

        self.get_logger().info(
            f'amcl_bootstrap_node: {amcl_pose_topic} + {odom_topic} -> '
            f'{self.initialpose_topic}; 수렴 기준 '
            f'{self.tracker.position_std:.2f} m / '
            f'{math.degrees(self.tracker.yaw_std):.0f} deg, '
            f'연속 {self.tracker.required_samples} 회, '
            f'최소 이동 {self.tracker.min_travel:.1f} m'
        )
        self.get_logger().warning(
            '\n' + '=' * 62 + '\n'
            'AMCL 전역 측위로 초기 위치를 잡습니다.\n'
            '로봇을 몇 미터 주행시키세요 (제자리 회전만으로는 부족합니다).\n'
            + '=' * 62)

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _trigger_once(self):
        """전역 측위를 한 번 건다. 서비스가 아직 없으면 다음 주기에 다시."""
        if self._triggered:
            return
        if not self.global_localization_client.service_is_ready():
            self.get_logger().info(
                f'{self.global_localization_service} 대기 중...',
                throttle_duration_sec=5.0)
            return
        self.global_localization_client.call_async(Empty.Request())
        self._triggered = True
        self.get_logger().info(
            '전역 측위 요청 완료. 파티클이 자유공간 전체에 뿌려졌다.')

    def _on_map(self, message):
        """지도를 받아 점유 셀 거리장을 만든다. 한 번만 만들면 된다."""
        info = message.info
        try:
            self._matcher = MapMatcher(
                message.data, info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y,
                match_distance=self.match_distance)
        except ValueError as exc:
            self.get_logger().error(f'지도로 정합기를 못 만들었다: {exc}')
            return
        self.get_logger().info(
            f'정합 검증 준비됨 ({info.width}x{info.height} @ '
            f'{info.resolution:.3f} m/cell, 허용 {self.match_distance:.2f} m)')

    def _on_scan(self, message):
        self._last_scan = message

    def _scan_points_at(self, pose):
        """후보 pose 에 현재 스캔을 얹어 map 좌표 끝점을 만든다.

        TF 의 map->odom 은 지금 틀어져 있으므로 쓰지 않는다. base_footprint
        에서 라이다까지의 정적 변환만 TF 에서 받고, base->map 은 후보 pose 를
        그대로 쓴다. 그래야 "이 pose 가 맞나" 를 독립적으로 물을 수 있다.
        """
        scan = self._last_scan
        if scan is None or self._matcher is None:
            return None
        source = scan.header.frame_id or self.base_frame
        try:
            static = self.tf_buffer.lookup_transform(
                self.base_frame, source, Time(),
                timeout=Duration(seconds=0.2)).transform
        except Exception as exc:
            self.get_logger().warning(
                f'{self.base_frame} <- {source} TF 없음: {exc}',
                throttle_duration_sec=5.0)
            return None

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        maximum = float(scan.range_max) if scan.range_max > 0.0 else np.inf
        # 끝까지 안 맞고 돌아온 광선은 벽 위치를 말해주지 않으므로 뺀다.
        valid = (
            np.isfinite(ranges)
            & (ranges >= float(scan.range_min))
            & (ranges <= maximum * 0.99)
        )
        if not np.any(valid):
            return None
        angles = (float(scan.angle_min)
                  + np.arange(ranges.size) * float(scan.angle_increment))
        local = np.column_stack((ranges[valid] * np.cos(angles[valid]),
                                 ranges[valid] * np.sin(angles[valid])))

        laser_yaw = _quaternion_yaw(static.rotation)
        in_base = _rotate(local, laser_yaw) + np.array(
            [static.translation.x, static.translation.y])

        base_yaw = _quaternion_yaw(pose.orientation)
        return _rotate(in_base, base_yaw) + np.array(
            [pose.position.x, pose.position.y])

    def _verify(self, pose):
        """후보 pose 를 스캔-지도 정합으로 검증한다. (통과 여부, 설명)."""
        if not self.verify_with_scan:
            return True, '검증 꺼짐'
        if self._matcher is None:
            return False, '지도를 아직 못 받았다'
        points = self._scan_points_at(pose)
        if points is None:
            return False, '검증에 쓸 스캔이 없다'
        score, used = self._matcher.score(points)
        if used < self.min_match_points:
            return False, f'격자 안 표본이 {used} 개뿐 (최소 {self.min_match_points})'
        detail = f'정합 {score:.2f} (기준 {self.min_match_score:.2f}), 표본 {used}'
        self.get_logger().info(f'검증: {detail}')
        return score >= self.min_match_score, detail

    def _retry_global_localization(self, reason):
        self._retries += 1
        self.tracker.reset()
        if self._retries > self.max_retries:
            self.get_logger().error(
                f'{reason} — 재시도 {self.max_retries} 회를 모두 썼다. '
                'RViz 2D Pose Estimate 로 직접 찍어 주세요.')
            return
        self.get_logger().warning(
            f'{reason} — 전역 측위를 다시 건다 '
            f'({self._retries}/{self.max_retries}). 로봇을 계속 주행시키세요.')
        if self.global_localization_client.service_is_ready():
            self.global_localization_client.call_async(Empty.Request())

    def _on_odometry(self, message):
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self._last_position is not None:
            self.tracker.add_travel(
                math.hypot(current[0] - self._last_position[0],
                           current[1] - self._last_position[1]))
        self._last_position = current

    def _on_amcl_pose(self, message):
        if self._published and self.oneshot:
            return
        try:
            converged = self.tracker.update(message.pose.covariance)
        except ValueError as exc:
            self.get_logger().error(f'AMCL 공분산이 이상하다: {exc}')
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.status_period > 0.0 and now - self._last_status >= self.status_period:
            self._last_status = now
            self.get_logger().info(self.tracker.status())

        if not converged:
            return

        passed, detail = self._verify(message.pose.pose)
        if not passed:
            self._retry_global_localization(f'수렴했지만 검증 실패 — {detail}')
            return

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        # set_initial_pose_node 는 map 좌표계만 받는다. AMCL 이 이미 map
        # 기준으로 내지만, frame_id 를 여기서 명시해 실수를 막는다.
        out.header.frame_id = self.map_frame
        out.pose = message.pose
        self.initialpose_publisher.publish(out)
        self._published = True

        std_x, std_y, std_yaw = self.tracker.last_std
        position = message.pose.pose.position
        self.get_logger().warning(
            '\n' + '=' * 62 + '\n'
            f'AMCL 수렴. 초기 위치를 {self.initialpose_topic} 로 보냈다.\n'
            f'  x={position.x:.3f} y={position.y:.3f} m, '
            f'std {std_x:.3f}/{std_y:.3f} m, '
            f'{math.degrees(std_yaw):.1f} deg\n'
            f'  이동 거리 {self.tracker.travel:.2f} m\n'
            'cartographer 가 궤적을 재시작합니다.\n'
            + '=' * 62)


def main(args=None):
    rclpy.init(args=args)
    node = AmclBootstrapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

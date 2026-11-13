#!/usr/bin/env python3
"""traversability_node — 1단계(a) traversable map 생산자.

설계문서 §4/§7.1 의 "주행 가능성 판단 → Planning" 자리다.

입력:
  /map   (nav_msgs/OccupancyGrid)  기준 2D 점유 격자. nav2_map_server 가 낸다.
                                   격자 기하(width/height/resolution/origin)를
                                   여기서 받아 그대로 쓴다 = 전역 고정 범위.
  /scan  (sensor_msgs/LaserScan)   현재 관측. map frame 으로 TF 변환해 투영한다.
  /traversability/cloud_band       depth_preprocess_node 가 z-band 만 남긴 D455
                                   점군. height_frame 기준이라 map 으로 변환해
                                   같은 격자에 투영한다. 없으면 LiDAR 만 쓴다.

출력:
  /traversable_map      (amr_interfaces/TraversableMap)  Planning 계약
  /traversable_map_grid (nav_msgs/OccupancyGrid)         RViz 확인용(옵션)

z_min/z_max 는 depth_preprocess_node 와 반드시 같은 값이어야 한다. 여기서는
계약 필드로 싣고, 실제 밴드 필터는 전처리 노드가 건다 — 점군을 두 번 나르지
않기 위해서다. 바닥 지지·단차·낙차 판정은 아직 없다.
"""

import math
import time

from amr_interfaces.msg import TraversableMap

from builtin_interfaces.msg import Time as TimeMsg

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

import tf2_ros

from traversability import depth_projection
from traversability import encoding
from traversability import scan_projection


def quaternion_yaw(quaternion):
    """평면 회전만 남긴 yaw 를 뽑는다."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class TraversabilityNode(Node):
    """기준 맵과 LiDAR 관측을 3-state traversable map 으로 합친다."""

    def __init__(self):
        super().__init__('traversability_node')

        self.map_frame = self._declare('map_frame', 'map')
        self.base_frame = self._declare('base_frame', 'base_footprint')
        base_map_topic = self._declare('base_map_topic', '/map')
        scan_topic = self._declare('scan_topic', '/scan')
        self.output_topic = self._declare('output_topic', '/traversable_map')
        debug_topic = self._declare('debug_grid_topic', '/traversable_map_grid')

        self.publish_frequency = float(self._declare('publish_frequency', 2.0))
        self.memory_seconds = float(self._declare('memory_seconds', 5.0))
        self.publish_debug_grid = bool(self._declare('publish_debug_grid', True))

        # 1단계에서는 기준 맵의 free 를 통과 가능으로 인정한다. 그 free 는
        # 0.218 m 높이 LiDAR 평면에서 관측된 것이고, 1단계의 traversability
        # 정의가 바로 그 평면이기 때문이다. D455 가 들어와 정의가 넓어지는
        # 2단계에서 false 로 뒤집는다.
        self.trust_base_map_free = bool(
            self._declare('trust_base_map_free', True)
        )

        # z-band. base_footprint 기준, z 축은 수직 위쪽.
        # z_min: 카메라 -0.10 m 를 지면 기준으로 환산한 값(카메라 높이 0.1328 m).
        # z_max: 로봇 최고점(LiDAR 상단 ~0.258 m) + 여유 0.05 m.
        self.height_frame = self._declare('height_frame', 'base_footprint')
        self.z_min = float(self._declare('z_min', 0.033))
        self.z_max = float(self._declare('z_max', 0.31))

        # 로봇 형상 팽창은 Planning 이 자기 footprint 로 한다. 여기서는 안 한다.
        self.inflation_radius = float(self._declare('inflation_radius', 0.0))

        self.scan_min_range = float(self._declare('scan_min_range', 0.15))
        self.scan_max_range = float(self._declare('scan_max_range', 8.0))
        self.scan_hit_threshold = int(self._declare('scan_hit_threshold', 3))

        # 관측 기반 해제(clearing). 광선이 관통한 셀을 통과 가능으로 세운다.
        # 이것이 없으면 traversable map 은 장애물을 더할 수만 있고 뺄 수 없다.
        self.enable_clearing = bool(self._declare('enable_clearing', True))
        self.clear_hit_threshold = int(self._declare('clear_hit_threshold', 3))
        self.clear_ray_stride = max(1, int(self._declare('clear_ray_stride', 2)))
        self.clear_step_ratio = float(self._declare('clear_step_ratio', 0.5))
        self.clear_stop_margin = float(self._declare('clear_stop_margin', 1.0))
        # FREE 기억의 유효시간. 0 이하이면 만료시키지 않는다 — 광선이
        # 관통했다는 증거는 시간이 지나도 사라지지 않는다.
        self.free_memory_seconds = float(
            self._declare('free_memory_seconds', 0.0))
        # 재측위 처리. /initialpose 가 온다는 것은 "여기가 어디인지 다시
        # 선언했다" 는 뜻이고, 그 이전 관측은 전부 다른 좌표계의 것이다.
        # 비우지 않으면 free_memory_seconds: 0.0 (무한) 때문에 잘못된 자세로
        # 뚫은 FREE 셀이 영구히 남는다.
        #
        # 비우는 것만으로는 부족하다. /initialpose 를 받은 set_initial_pose_node
        # 가 FinishTrajectory -> StartTrajectory -> ekf_carto set_pose 를 도는 데
        # 실측 0.51 s 가 걸리고(2026-09-09 로그), 그동안 들어오는 스캔은 아직
        # 옛 map->odom 으로 투영된다. 그래서 그 시간 동안 누적을 멈춘다.
        self.initialpose_topic = self._declare(
            'initialpose_topic', '/initialpose')
        self.relocalize_settle_seconds = float(
            self._declare('relocalize_settle_seconds', 1.5))

        # 관측/기억 개수를 주기적으로 찍는다. 실차에서 "왜 안 지워지지" 를
        # 볼 때 쓴다. 0 이면 끈다.
        self.diagnostic_period = float(self._declare('diagnostic_period', 0.0))
        self._last_diagnostic = 0.0
        self.scan_window_seconds = float(
            self._declare('scan_window_seconds', 1.0)
        )
        self.tf_timeout = float(self._declare('tf_timeout', 0.20))
        # Cartographer 의 map TF 는 스캔 stamp 보다 늦을 수 있다. 그 stamp 로
        # 물으면 future extrapolation 으로 전부 거부되어 관측이 통째로 빈다.
        self.scan_use_latest_tf = bool(self._declare('scan_use_latest_tf', True))

        # D455 z-band 투영. depth_preprocess_node 가 밴드를 걸어 보내 준다.
        self.use_depth = bool(self._declare('use_depth', True))
        band_topic = self._declare('band_cloud_topic',
                                   '/traversability/cloud_band')
        # 점군은 스캔보다 느리고 무거우므로 히트 임계를 따로 둔다.
        self.depth_hit_threshold = int(self._declare('depth_hit_threshold', 2))
        self.depth_window_seconds = float(
            self._declare('depth_window_seconds', 1.0))
        # D455 관측 기반 해제. 카메라 원점에서 밴드 끝점까지 광선을 쏘아
        # 관통한 셀의 DEPTH 근거를 내린다. LiDAR 해제와 대칭이며, 밴드에
        # 아무 반사가 없는 방위에서는 광선이 생기지 않아 해제도 없다 — 밴드
        # 위쪽 구조물에 가려 밴드를 못 본 방위가 여기서 자동으로 걸러진다.
        self.depth_enable_clearing = bool(
            self._declare('depth_enable_clearing', True))
        self.depth_clear_hit_threshold = int(
            self._declare('depth_clear_hit_threshold', 2))
        self.depth_clear_ray_stride = max(
            1, int(self._declare('depth_clear_ray_stride', 4)))
        self.depth_clear_step_ratio = float(
            self._declare('depth_clear_step_ratio', 0.5))
        self.depth_clear_stop_margin = float(
            self._declare('depth_clear_stop_margin', 1.0))
        # 광선 원점. URDF: base_link -> camera_link xyz 0.11398 0 0.092772
        # 이라 base_footprint 원점과 x 로 0.114 m(약 2.3 셀) 떨어져 있다.
        # base_footprint 를 원점으로 쓰면 근거리 셀의 해제가 어긋난다.
        self.camera_frame = self._declare('camera_frame', 'camera_link')

        if self.publish_frequency <= 0.0:
            raise ValueError('publish_frequency must be positive')
        if self.memory_seconds < 0.0:
            raise ValueError('memory_seconds must not be negative')
        if self.scan_max_range <= self.scan_min_range:
            raise ValueError('scan range limits are invalid')
        if self.z_max <= self.z_min:
            raise ValueError('z_max must be greater than z_min')

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._info = None          # 기준 맵의 기하. 격자 크기의 단일 출처.
        self._base = None          # 3-state 로 분류한 기준 맵
        self._memory = None        # 관측 기억 (3-state)
        self._source = None        # 셀별 BLOCKED 근거 비트마스크 (uint8)
        self._observed_at = None   # 셀별 최종 관측 시각 [s, monotonic]
        self._scan_history = []    # [(수신시각, flat cell 인덱스)]
        self._depth_history = []   # [(수신시각, flat cell 인덱스)]
        self._clear_history = []   # [(수신시각, 광선이 관통한 flat 인덱스)]
        self._depth_clear_history = []  # [(수신시각, 카메라 광선이 관통한 셀)]
        self._oldest_observation = None
        self._latest_scan_stamp = TimeMsg()
        self._latest_depth_stamp = TimeMsg()
        # 이 시각(monotonic) 전까지는 관측을 쌓지 않는다. 재측위 정착용.
        self._accumulate_after = 0.0

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, base_map_topic, self._on_base_map, latched
        )
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        if self.use_depth:
            self.create_subscription(
                PointCloud2, band_topic, self._on_band_cloud, 1
            )
        # /initialpose 는 두 곳에서 온다 — RViz 의 2D Pose Estimate 와
        # amcl_bootstrap_node. 후자는 BEST_EFFORT 로 발행한다
        # (set_initial_pose_node 의 구독자에 맞춘 것). RELIABLE 구독자는
        # BEST_EFFORT 발행자와 매칭되지 않아 조용히 한 개도 못 받으므로
        # 여기도 BEST_EFFORT 여야 한다.
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self._on_initial_pose,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )
        self.publisher = self.create_publisher(
            TraversableMap, self.output_topic, latched
        )
        self.debug_publisher = (
            self.create_publisher(OccupancyGrid, debug_topic, latched)
            if self.publish_debug_grid
            else None
        )
        self.create_timer(1.0 / self.publish_frequency, self._publish)

        if not self.enable_clearing:
            clearing_status = 'off'
        elif self.free_memory_seconds > 0.0:
            clearing_status = f'on (free memory {self.free_memory_seconds:.0f} s)'
        else:
            clearing_status = 'on (free memory 무한)'
        self.get_logger().info(
            f'traversability_node: {base_map_topic} + {scan_topic} -> '
            f'{self.output_topic} at {self.publish_frequency:.1f} Hz; '
            f'z-band [{self.z_min:.3f}, {self.z_max:.3f}] m in '
            f'{self.height_frame}; memory {self.memory_seconds:.1f} s; '
            f'trust_base_map_free={self.trust_base_map_free}; '
            f'depth={"on (" + band_topic + ")" if self.use_depth else "off"}; '
            f'clearing={clearing_status}'
        )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _settling(self):
        """재측위 직후 정착 구간이면 True. 그동안은 관측을 버린다."""
        return time.monotonic() < self._accumulate_after

    def _on_base_map(self, message):
        """기준 맵을 받아 격자 기하와 3-state 분류를 갱신한다."""
        info = message.info
        shape = (info.height, info.width)
        if info.width == 0 or info.height == 0:
            self.get_logger().warning('Ignoring empty base map.')
            return
        origin_yaw = quaternion_yaw(info.origin.orientation)
        if abs(origin_yaw) > 1.0e-6:
            self.get_logger().warning(
                f'Base map origin yaw {origin_yaw:.4f} rad is not supported; '
                'treating the grid as axis-aligned.'
            )

        reshaped = np.asarray(message.data, dtype=np.int8).reshape(shape)
        self._base = encoding.classify_base_map(
            reshaped, self.trust_base_map_free
        )

        if self._info is None or (
            self._info.width != info.width
            or self._info.height != info.height
            or abs(self._info.resolution - info.resolution) > 1.0e-9
        ):
            # 기하가 바뀌면 기억을 이어 붙일 수 없다. 새로 시작한다.
            self._reset_memory(
                shape,
                f'base map geometry {info.width}x{info.height} @ '
                f'{info.resolution:.3f} m/cell, origin '
                f'({info.origin.position.x:.3f}, '
                f'{info.origin.position.y:.3f})'
            )
        self._info = info

    def _reset_memory(self, shape, reason):
        """누적 관측을 전부 버린다.

        shape 이 None 이면 격자 기하를 아직 못 받은 것이라 배열은 만들지 않고
        history 만 비운다 — 기하는 _on_base_map 이 단일 출처다.
        """
        if shape is not None:
            self._memory = np.full(shape, encoding.UNKNOWN, dtype=np.int8)
            self._source = np.zeros(shape, dtype=np.uint8)
            self._observed_at = np.full(shape, -math.inf, dtype=np.float64)
        self._scan_history.clear()
        self._depth_history.clear()
        self._clear_history.clear()
        self._depth_clear_history.clear()
        self._oldest_observation = None
        self._latest_scan_stamp = TimeMsg()
        self._latest_depth_stamp = TimeMsg()
        self.get_logger().info(f'observation memory reset — {reason}')

    def _on_initial_pose(self, message):
        """초기위치가 다시 선언되면 그 이전 관측은 전부 무효다.

        RViz 2D Pose Estimate 와 amcl_bootstrap_node 둘 다 이 토픽으로 온다.
        받는 쪽에서 궤적이 재시작되고 map->odom 이 점프하므로, 기억을 비우고
        relocalize_settle_seconds 동안 누적을 멈춘다. 멈추지 않으면 그 사이
        스캔이 옛 좌표로 다시 쌓인다.
        """
        del message  # pose 값은 안 쓴다. 사건이 일어났다는 것만 쓴다.
        shape = (
            None if self._info is None
            else (self._info.height, self._info.width)
        )
        self._reset_memory(shape, f'initial pose on {self.initialpose_topic}')
        self._accumulate_after = (
            time.monotonic() + self.relocalize_settle_seconds)
        self.get_logger().warning(
            f'초기위치 수신 — 기억을 비우고 '
            f'{self.relocalize_settle_seconds:.1f} s 동안 누적을 멈춘다.'
        )

    def _on_scan(self, message):
        if self._info is None or self._settling():
            return
        try:
            cells, cleared = self._scan_cells(message)
        except Exception as exc:
            self.get_logger().warning(
                f'Cannot transform LaserScan into {self.map_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return
        received_at = time.monotonic()
        self._scan_history.append((received_at, cells))
        if cleared.size:
            self._clear_history.append((received_at, cleared))
        self._prune_scan_history(received_at)
        self._latest_scan_stamp = message.header.stamp
        if self._oldest_observation is None:
            self._oldest_observation = self.get_clock().now()

    def _scan_cells(self, message):
        info = self._info
        minimum = max(self.scan_min_range, float(message.range_min))
        sensor_maximum = float(message.range_max)
        maximum = min(
            self.scan_max_range,
            sensor_maximum if sensor_maximum > 0.0 else self.scan_max_range,
        )
        points = scan_projection.laser_endpoints(
            message.ranges,
            float(message.angle_min),
            float(message.angle_increment),
            minimum,
            maximum,
        )
        empty = (np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64))
        if points.size == 0:
            return empty

        source_frame = message.header.frame_id or self.base_frame
        stamp_is_zero = (
            message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0
        )
        query_time = Time()
        if not self.scan_use_latest_tf and not stamp_is_zero:
            query_time = Time.from_msg(message.header.stamp)
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            source_frame,
            query_time,
            timeout=Duration(seconds=self.tf_timeout),
        ).transform

        world = scan_projection.transform_points(
            points,
            transform.translation.x,
            transform.translation.y,
            quaternion_yaw(transform.rotation),
        )
        hits = scan_projection.world_to_flat_cells(
            world,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
        )

        if not self.enable_clearing:
            return hits, np.empty(0, dtype=np.int64)

        # 광선이 관통한 셀 = 그 평면이 비어 있었다는 증거. 빔을 솎아도
        # 인접 빔이 거의 같은 셀을 지나므로 해제 품질이 크게 떨어지지 않는다.
        sensor_origin = (transform.translation.x, transform.translation.y)
        cleared = scan_projection.raycast_free_cells(
            sensor_origin,
            world[::self.clear_ray_stride],
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            step_ratio=self.clear_step_ratio,
            stop_margin=self.clear_stop_margin,
        )
        return hits, cleared

    def _on_band_cloud(self, message):
        """z-band 점군을 map frame 셀 인덱스로 바꿔 쌓는다.

        밴드 필터는 depth_preprocess_node 가 이미 걸었다. 여기서 하는 일은
        height_frame -> map 변환과 격자 투영뿐이다.
        """
        if self._info is None or self._settling():
            return
        source_frame = message.header.frame_id or self.base_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except Exception as exc:
            self.get_logger().warning(
                f'Cannot transform {source_frame} into {self.map_frame}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        points = point_cloud2.read_points_numpy(
            message, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        # A valid, transformed empty band is still a live sensor frame.
        self._latest_depth_stamp = message.header.stamp
        if points.shape[0] == 0:
            return

        rotation = depth_projection.quaternion_to_matrix(
            transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w)
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ], dtype=np.float64)
        in_map = depth_projection.transform_points(
            points, translation, rotation)

        info = self._info
        cells = depth_projection.to_flat_cells(
            in_map,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
        )
        received_at = time.monotonic()
        self._depth_history.append((received_at, cells))
        cleared = self._depth_cleared_cells(in_map, info)
        if cleared.size:
            self._depth_clear_history.append((received_at, cleared))
        self._prune_depth_history(received_at)
        if self._oldest_observation is None:
            self._oldest_observation = self.get_clock().now()

    def _depth_cleared_cells(self, in_map, info):
        """카메라 원점에서 밴드 끝점까지의 광선이 관통한 셀.

        LiDAR 와 같은 raycast_free_cells 를 쓴다. 끝점이 밴드 반사이므로 그
        광선은 해당 방위에서 밴드를 실제로 통과했다 — 더 가까운 곳에 밴드
        물체가 있었다면 끝점이 더 가까웠을 것이다. 반사가 없는 방위는 광선이
        없어 해제도 없으므로, 밴드보다 높은 구조물에 가려 밴드를 못 본 경우가
        자동으로 제외된다.

        같은 주기의 depth hit 은 update_memory_sourced 에서 이 해제를 이긴다.
        """
        empty = np.empty(0, dtype=np.int64)
        if not self.depth_enable_clearing or in_map.shape[0] == 0:
            return empty
        try:
            camera = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except Exception as exc:
            self.get_logger().warning(
                f'Cannot locate {self.camera_frame} in {self.map_frame}; '
                f'depth clearing skipped: {exc}',
                throttle_duration_sec=5.0,
            )
            return empty
        return scan_projection.raycast_free_cells(
            (camera.translation.x, camera.translation.y),
            in_map[::self.depth_clear_ray_stride, :2],
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            step_ratio=self.depth_clear_step_ratio,
            stop_margin=self.depth_clear_stop_margin,
        )

    def _prune_depth_history(self, now):
        oldest = now - self.depth_window_seconds
        self._depth_history = [
            entry for entry in self._depth_history if entry[0] >= oldest
        ]
        self._depth_clear_history = [
            entry for entry in self._depth_clear_history if entry[0] >= oldest
        ]

    def _prune_scan_history(self, now):
        oldest = now - self.scan_window_seconds
        self._scan_history = [
            entry for entry in self._scan_history if entry[0] >= oldest
        ]
        self._clear_history = [
            entry for entry in self._clear_history if entry[0] >= oldest
        ]
        if not self._scan_history:
            self._oldest_observation = None

    def _observation_layers(self, shape):
        """이번 주기의 관측을 센서·종류별 bool 마스크 네 개로 만든다.

        3-state 하나로 합치지 않는 이유는 근거를 잃지 않기 위해서다. 두 센서가
        서로 다른 높이를 보므로, 한쪽의 관통을 다른 쪽의 표시를 지우는 데 쓰면
        안 된다(설계문서 §6). 합치는 규칙은 encoding.update_memory_sourced 가
        갖는다 — 여기서는 세지 않고 모으기만 한다.

        반환: (lidar_hits, depth_hits, lidar_clear, depth_clear)
        """
        def mask(history, threshold):
            if not history:
                return np.zeros(shape, dtype=bool)
            return scan_projection.accumulated_hit_mask(
                [cells for _, cells in history], shape, threshold)

        return (
            mask(self._scan_history, self.scan_hit_threshold),
            mask(self._depth_history, self.depth_hit_threshold),
            mask(self._clear_history, self.clear_hit_threshold),
            mask(self._depth_clear_history, self.depth_clear_hit_threshold),
        )

    def _log_diagnostics(self, now, composed):
        """관측·기억·출력 개수를 찍는다. 해제가 안 될 때 어디서 막혔는지 본다."""
        if self.diagnostic_period <= 0.0:
            return
        if now - self._last_diagnostic < self.diagnostic_period:
            return
        self._last_diagnostic = now
        self.get_logger().info(
            f'scans={len(self._scan_history)} '
            f'clears={len(self._clear_history)} '
            f'depth={len(self._depth_history)} '
            f'depth_clears={len(self._depth_clear_history)} | '
            f'src lidar={int(((self._source & encoding.SOURCE_LIDAR) != 0).sum())} '
            f'depth={int(((self._source & encoding.SOURCE_DEPTH) != 0).sum())} | '
            f'memory blocked={int((self._memory == encoding.BLOCKED).sum())} '
            f'free={int((self._memory == encoding.FREE).sum())} | '
            f'out blocked={int((composed == encoding.BLOCKED).sum())} '
            f'free={int((composed == encoding.FREE).sum())} '
            f'unknown={int((composed == encoding.UNKNOWN).sum())}'
        )

    def _publish(self):
        if self._info is None or self._memory is None:
            self.get_logger().info(
                'Waiting for the base map before publishing.',
                throttle_duration_sec=5.0,
            )
            return

        now = time.monotonic()
        self._prune_scan_history(now)
        self._prune_depth_history(now)
        shape = (self._info.height, self._info.width)

        lidar_hits, depth_hits, lidar_clear, depth_clear = (
            self._observation_layers(shape))
        self._memory, self._source, self._observed_at = (
            encoding.update_memory_sourced(
                self._memory,
                self._source,
                self._observed_at,
                lidar_hits,
                depth_hits,
                lidar_clear,
                depth_clear,
                now,
                self.memory_seconds,
                self.free_memory_seconds,
            )
        )
        composed = encoding.compose(self._memory, self._base)
        fill = encoding.unknown_fill_flags(self._memory, self._base, composed)
        self._log_diagnostics(now, composed)

        stamp = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self.map_frame
        grid.info = self._info
        grid.data = composed.reshape(-1).tolist()

        message = TraversableMap()
        message.header = grid.header
        message.grid = grid
        message.height_frame = self.height_frame
        message.z_min = self.z_min
        message.z_max = self.z_max
        message.inflation_radius = self.inflation_radius
        message.unknown_fill = fill
        message.oldest_observation = (
            self._oldest_observation.to_msg()
            if self._oldest_observation is not None
            else TimeMsg()
        )
        stamps = [self._latest_scan_stamp]
        if self.use_depth:
            stamps.append(self._latest_depth_stamp)
        message.sensor_watermark = min(
            stamps, key=lambda value: value.sec * 1000000000 + value.nanosec)
        self.publisher.publish(message)

        if self.debug_publisher is not None:
            self.debug_publisher.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityNode()
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

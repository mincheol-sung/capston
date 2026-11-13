#!/usr/bin/env python3
"""depth_preprocess_node — D455 점군 공통 전처리 (설계문서 §3.2-3).

  "유효 깊이 범위와 관심 영역을 제한하고, 필요 시 점군을 다운샘플링한다.
   차체·로봇팔의 자기 관측을 제거하되 실제 외부 장애물을 지우지 않도록 한다."

입력
  /camera/depth/color/points  (sensor_msgs/PointCloud2, camera optical frame)

출력
  /traversability/cloud_filtered  전체 높이. output_frame(기본 odom) 기준.
                                  octomap_server 와 STVL 이 같은 입력을 받도록
                                  여기서 한 번만 만든다(§3.3 "공통 D455 전처리를
                                  유지하고 3D 갱신 모듈만 교체한다").
  /traversability/cloud_band      z-band 만. height_frame(기본 base_footprint) 기준.
                                  traversability_node 의 2D 투영과 RViz 밴드 확인용.
                                  **카메라 기준 band_max_range(기본 1.5 m) 안쪽만**
                                  남긴다 — 2D 격자는 점 하나로 셀을 세우는데
                                  깊이오차가 2.5 m 에서 이미 1 셀을 넘는다.
                                  3D 쪽(cloud_filtered)은 그 상한을 안 받는다.

z 축 규약: height_frame 원점 기준 **수직 위쪽**이다. camera optical 의 z(전방
깊이)가 아니다. 그래서 밴드 필터 전에 반드시 TF 변환을 거친다.
"""

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import tf2_ros

from traversability import depth_projection as dp


class DepthPreprocessNode(Node):
    """D455 점군을 3D 표현과 2D 투영이 공유할 형태로 다듬는다."""

    def __init__(self):
        super().__init__('depth_preprocess_node')

        cloud_topic = self._declare('cloud_topic', '/camera/depth/color/points')
        self.filtered_topic = self._declare(
            'filtered_topic', '/traversability/cloud_filtered')
        self.band_topic = self._declare(
            'band_topic', '/traversability/cloud_band')

        # 3D 누적은 연속적인 odom 로컬 윈도우에서 한다(설계문서 §3.2).
        # map 은 위치추정 보정 때 과거 자세가 소급 변경되어 누적이 어긋난다.
        self.output_frame = self._declare('output_frame', 'odom')
        # z 를 재는 기준. 지면이 원점이라 밴드 값을 그대로 읽을 수 있다.
        self.height_frame = self._declare('height_frame', 'base_footprint')

        self.z_min = float(self._declare('z_min', 0.033))
        self.z_max = float(self._declare('z_max', 0.31))
        self.min_range = float(self._declare('min_range', 0.30))
        self.max_range = float(self._declare('max_range', 6.0))
        # 2D 밴드에만 걸는 별도 상한. **카메라 원점 기준** 수평거리다.
        # 깊이오차가 거리의 2 % 라 2.5 m 에서 1 셀(0.05 m)을 넘고, 셀을 덮는
        # 픽셀도 1.5 m 197 px² -> 6 m 12 px² 로 16 배 줄어든다. 2D 격자는 점
        # 하나로 셀을 세우므로 원거리 점을 넣을 이유가 없다. 3D 쪽은 log-odds
        # 로 노이즈를 누적 판정하므로 max_range(6.0)를 그대로 쓴다.
        self.band_max_range = float(self._declare('band_max_range', 1.5))
        self.voxel_leaf = float(self._declare('voxel_leaf', 0.03))
        self.max_input_points = int(self._declare('max_input_points', 120000))

        # 차체 자기 관측 제거 상자. height_frame 기준 축정렬.
        # URDF 실측: base_link 충돌박스 0.28584 x 0.18841 x 0.16701, 지면 위
        # 0.0219~0.1889 m. LiDAR 마운트 0.2178 m. 여유 2 cm 를 두되 그 이상
        # 키우지 않는다 — 상자를 키우면 차체 앞의 실제 장애물까지 지운다.
        self.self_box_min = [float(v) for v in self._declare(
            'self_filter_box_min', [-0.20, -0.15, -0.05])]
        self.self_box_max = [float(v) for v in self._declare(
            'self_filter_box_max', [0.20, 0.15, 0.28])]

        self.tf_timeout = float(self._declare('tf_timeout', 0.20))
        # RealSense 의 stamp 와 TF 가 어긋나면 전 프레임이 버려진다. 최신 변환을
        # 쓰는 편이 이 짧은 수명의 관측에는 낫다(cartographer.launch 와 같은 이유).
        self.use_latest_tf = bool(self._declare('use_latest_tf', True))
        self.log_period = float(self._declare('log_period', 5.0))

        if self.z_max <= self.z_min:
            raise ValueError('z_max must be greater than z_min')
        if self.max_range <= self.min_range:
            raise ValueError('max_range must be greater than min_range')
        if self.band_max_range <= 0.0:
            raise ValueError('band_max_range must be positive')
        if len(self.self_box_min) != 3 or len(self.self_box_max) != 3:
            raise ValueError('self_filter_box_* must have three elements')

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        reliable = QoSProfile(depth=1)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        self.filtered_publisher = self.create_publisher(
            PointCloud2, self.filtered_topic, reliable)
        self.band_publisher = self.create_publisher(
            PointCloud2, self.band_topic, reliable)
        self.create_subscription(
            PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)

        self._frames = 0
        self._last_report = 0.0
        self.get_logger().info(
            f'depth_preprocess_node: {cloud_topic} -> {self.filtered_topic} '
            f'({self.output_frame}) + {self.band_topic} ({self.height_frame}); '
            f'z-band [{self.z_min:.3f}, {self.z_max:.3f}] m, '
            f'range {self.min_range:.2f}~{self.max_range:.2f} m, '
            f'band max {self.band_max_range:.2f} m from the camera, '
            f'voxel {self.voxel_leaf:.3f} m'
        )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _lookup(self, target, source, stamp):
        query = Time()
        if not self.use_latest_tf:
            query = Time.from_msg(stamp)
        return self.tf_buffer.lookup_transform(
            target, source, query, timeout=Duration(seconds=self.tf_timeout)
        ).transform

    @staticmethod
    def _as_arrays(transform):
        t = transform.translation
        r = transform.rotation
        return (
            np.array([t.x, t.y, t.z], dtype=np.float64),
            dp.quaternion_to_matrix(r.x, r.y, r.z, r.w),
        )

    def _on_cloud(self, message):
        source_frame = message.header.frame_id
        if not source_frame:
            self.get_logger().warning(
                'PointCloud2 without frame_id; dropping.',
                throttle_duration_sec=5.0)
            return
        try:
            to_height = self._as_arrays(
                self._lookup(self.height_frame, source_frame,
                             message.header.stamp))
            height_to_output = self._as_arrays(
                self._lookup(self.output_frame, self.height_frame,
                             message.header.stamp))
        except Exception as exc:
            self.get_logger().warning(
                f'TF unavailable ({source_frame} -> {self.height_frame} -> '
                f'{self.output_frame}): {exc}',
                throttle_duration_sec=5.0)
            return

        raw = point_cloud2.read_points_numpy(
            message, field_names=('x', 'y', 'z'), skip_nans=False)
        raw = np.asarray(raw, dtype=np.float64).reshape(-1, 3)
        if raw.shape[0] > self.max_input_points:
            # 결정적 균등 추출. 무작위를 쓰면 프레임마다 다른 점이 남아
            # octomap 의 광선 통과가 불안정해진다.
            step = int(np.ceil(raw.shape[0] / self.max_input_points))
            raw = raw[::step]

        points = dp.finite_points(raw)
        points = dp.transform_points(points, to_height[0], to_height[1])
        points = dp.drop_self_observations(
            points, self.self_box_min, self.self_box_max)
        points = dp.limit_range(points, self.min_range, self.max_range)
        points = dp.voxel_downsample(points, self.voxel_leaf)

        band = dp.select_height_band(points, self.z_min, self.z_max)
        # to_height[0] 은 height_frame <- 카메라 optical frame 변환의 평행이동,
        # 즉 height_frame 안의 카메라 위치다. 이걸 원점으로 재야 "카메라 기준
        # 전방 N m" 가 된다 — height_frame 원점은 지면 중심이라 x 로 0.114 m
        # 어긋난다.
        band_in = band.shape[0]
        band = dp.limit_range_from(band, to_height[0][:2], self.band_max_range)
        output = dp.transform_points(
            points, height_to_output[0], height_to_output[1])

        stamp = message.header.stamp
        self.filtered_publisher.publish(
            point_cloud2.create_cloud_xyz32(
                self._header(stamp, self.output_frame),
                output.astype(np.float32)))
        self.band_publisher.publish(
            point_cloud2.create_cloud_xyz32(
                self._header(stamp, self.height_frame),
                band.astype(np.float32)))

        self._frames += 1
        self._report(raw.shape[0], points.shape[0], band_in, band.shape[0])

    def _header(self, stamp, frame_id):
        header = PointCloud2().header
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def _report(self, raw_count, kept, band_in_count, band_count):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.log_period <= 0.0 or now - self._last_report < self.log_period:
            return
        self._last_report = now
        self.get_logger().info(
            f'{self._frames} frames; last: {raw_count} in -> {kept} filtered '
            f'-> {band_in_count} in z-band [{self.z_min:.3f}, {self.z_max:.3f}] '
            f'-> {band_count} within {self.band_max_range:.2f} m of the camera'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthPreprocessNode()
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

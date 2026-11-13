#!/usr/bin/env python3
"""ABD membership on native scans, then beam deskew before representation.

/scan remains a legacy raster for navigation. This node never consumes it.
Unknown driver timing is explicit, and never replaced with scan_time/N.
"""
from collections import Counter
import math
import time

from amr_interfaces.msg import DetectedObject, DetectedObjectArray, ObservationProvenance
from amr_laser_tools.scan_sector_filter import filter_ranges
import numpy as np
from object_tracker.core.pending import PendingMeasurements
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros

from .observation_time import beam_offsets, deskew, rigid, stamp_ns

SOURCE_LIDAR = 1  # DetectedObject.source: 1=LIDAR (라이다 단독 관측)

CLASS_ID_LIDAR = -1
CLASS_NAME_LIDAR = 'lidar_cluster'


# 클러스터 대표점(object representative position) 후보.
# 물체의 기하학적 중심이 아니라 "보이는 점들의 대표값"이라는 점에 주의.
# 새 후보를 추가하려면 여기에 한 줄 넣고 cluster_position_method 로 고르면 된다.
def _mean_centroid(pts):
    return pts.mean(axis=0)


def _median_point(pts):
    return np.median(pts, axis=0)


def _nearest_point(pts):
    return pts[int(np.argmin(np.linalg.norm(pts, axis=1)))]


_REPRESENTATIVE = {
    'mean_centroid': _mean_centroid,
    'median': _median_point,
    'nearest_point': _nearest_point,
}


class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_node')

        self.scan_topics = list(self.declare_parameter(
            'scan_topics', ['/scan_front', '/scan_rear']).value)
        self.timing_modes = list(self.declare_parameter(
            'timing_modes', ['unknown', 'unknown']).value)
        self.sector_min = list(self.declare_parameter(
            'sector_min_deg', [-90.0, 90.0]).value)
        self.sector_max = list(self.declare_parameter(
            'sector_max_deg', [90.0, -90.0]).value)
        if not (len(self.scan_topics) == len(self.timing_modes)
                == len(self.sector_min) == len(self.sector_max)):
            raise ValueError('scan topics, timing modes and sectors must have equal length')
        for mode in self.timing_modes:
            beam_offsets(2, 0.01, mode)
        self.fixed_frame = self.declare_parameter('fixed_frame', 'odom').value
        if self.fixed_frame != 'odom':
            raise ValueError('fixed_frame must be continuous odom')
        self.knot_interval = float(self.declare_parameter('tf_knot_interval', 0.01).value)
        self.max_scan_duration = float(self.declare_parameter('max_scan_duration', 0.5).value)
        self.max_age = float(self.declare_parameter('max_observation_age', 0.75).value)
        self.unknown_policy = self.declare_parameter('unknown_timing_policy', 'snapshot').value
        if self.unknown_policy not in ('snapshot', 'drop'):
            raise ValueError('unknown_timing_policy must be snapshot or drop')
        if min(self.knot_interval, self.max_scan_duration, self.max_age) <= 0:
            raise ValueError('time bounds must be positive')
        wait = float(self.declare_parameter('tf_wait_timeout', 0.25).value)
        capacity = int(self.declare_parameter('pending_capacity', 16).value)
        self.pending = [PendingMeasurements(wait, capacity) for _ in self.scan_topics]
        self.counts = Counter()
        if 'unknown' in self.timing_modes:
            self.get_logger().warning(
                'Native LiDAR timing is UNVERIFIED: snapshot/drop policy active; '
                'intra-scan deskew is not claimed. Audit the driver before changing timing_modes.')
        self.clusters_topic = self.declare_parameter(
            'clusters_topic', '/lidar_clusters').value
        # 카메라 검출(/detections)과 같은 프레임으로 맞춘다. 빈 문자열이면 변환 안 함.
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value
        if self.target_frame != 'base_link':
            raise ValueError('perception output must use base_link')

        # ABD 파라미터
        self.lambda_rad = math.radians(
            float(self.declare_parameter('lambda_deg', 20.0).value))
        self.sigma_r = float(self.declare_parameter('sigma_r', 0.03).value)
        self.sigma_mult = float(self.declare_parameter('sigma_mult', 3.0).value)
        # 인접 점 각도차가 이 배수(스캔 각분해능 기준)를 넘으면 무조건 끊음.
        # 유효점 사이에 무효점(inf/0)이 많이 낀 경우 = 실제로 떨어진 물체.
        self.max_gap_bins = float(
            self.declare_parameter('max_gap_bins', 3.0).value)

        # 클러스터 필터
        self.min_points = int(self.declare_parameter('min_points', 3).value)
        # 이보다 좁으면 노이즈/스펙클로 보고 버림. 근거리에서 min_points=3 은
        # 0.6m 기준 16mm 폭도 통과시키므로 폭 하한이 따로 필요하다.
        self.min_width = float(self.declare_parameter('min_width', 0.05).value)
        # 이보다 넓으면 벽/배경으로 보고 버림(객체가 아님)
        self.max_width = float(self.declare_parameter('max_width', 1.5).value)
        # 클러스터 대표점 산출 방식(_REPRESENTATIVE 참고). 기본은 기존과 동일한 평균.
        #   mean_centroid : 보이는 점들의 산술평균  (기존 동작)
        #   median        : 좌표별 median          (아웃라이어에 강함)
        #   nearest_point : 원점에 가장 가까운 점  (카메라가 보는 앞면과 의미가 가까움)
        # 폭(width) 계산은 방식과 무관하게 항상 mean centroid 기준이다.
        # 그래야 대표점을 바꿔도 min_width/max_width 필터가 같은 클러스터를 통과시킨다.
        self.pos_method = str(self.declare_parameter(
            'cluster_position_method', 'mean_centroid').value)
        if self.pos_method not in _REPRESENTATIVE:
            self.get_logger().warn(
                f"cluster_position_method='{self.pos_method}' 알 수 없음 "
                '-> mean_centroid 로 대체')
            self.pos_method = 'mean_centroid'
        self.min_range = float(self.declare_parameter('min_range', 0.0).value)
        self.max_range = float(self.declare_parameter('max_range', 10.0).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            DetectedObjectArray, self.clusters_topic, 10)
        self.subscriptions_native = [self.create_subscription(
            LaserScan, topic, lambda msg, i=i: self._on_scan(msg, i),
            qos_profile_sensor_data) for i, topic in enumerate(self.scan_topics)]
        self.create_timer(0.01, self._drain)
        self.create_timer(5.0, self._diagnostics)

        # rgbd_detection_node 와 같은 이유 — 캐시된 값과 param get 이 어긋나면
        # CSV 의 note 컬럼이 거짓말을 한다.
        self.add_on_set_parameters_callback(self._on_param_set)

        self.get_logger().info(
            f'lidar_cluster_node 시작: {self.scan_topics} -> {self.clusters_topic} '
            f'(frame={self.target_frame or "스캔 그대로"}, '
            f'λ={math.degrees(self.lambda_rad):.0f}°, σ_r={self.sigma_r}, '
            f'min_points={self.min_points}, '
            f'width={self.min_width}~{self.max_width}m, '
            f'position={self.pos_method})')

    # ------------------------------------------------------------------ #

    def _on_param_set(self, params):
        """대표점 방식의 런타임 변경을 검증 후 실제로 반영."""
        for p in params:
            if p.name != 'cluster_position_method':
                continue
            if p.value not in _REPRESENTATIVE:
                return SetParametersResult(
                    successful=False,
                    reason=f'cluster_position_method: {p.value} 는 허용값이 아님 '
                           f'({" | ".join(_REPRESENTATIVE)})')
            self.pos_method = str(p.value)
            self.get_logger().warn(
                f'실험 정의 파라미터 변경: {p.name} = {p.value}. '
                '측정 진행 중이었다면 그 run 은 폐기할 것.')
        return SetParametersResult(successful=True)

    def _on_scan(self, msg, sensor=0):
        stamp = stamp_ns(msg.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        if stamp <= 0 or stamp > now_ns + 50_000_000:
            self.counts['invalid_or_future'] += 1
            return
        if (now_ns-stamp)*1e-9 > self.max_age:
            self.counts['stale'] += 1
            return
        self.pending[sensor].add(stamp, msg, time.monotonic())
        self._drain()

    def _drain(self):
        for sensor, pending in enumerate(self.pending):
            pending.drain(time.monotonic(),
                          lambda msg: self._prepare(msg, sensor),
                          lambda msg, result: self._publish(result, msg, sensor))

    def _prepare(self, msg, sensor):
        if (self.get_clock().now().nanoseconds-stamp_ns(msg.header.stamp))*1e-9 > self.max_age:
            self.counts['stale'] += 1
            return (None, None, None, None, None)
        offsets = beam_offsets(len(msg.ranges), msg.time_increment,
                               self.timing_modes[sensor], self.max_scan_duration)
        if offsets is None and self.unknown_policy == 'drop':
            self.counts['timing_unknown_drop'] += 1
            return (None, None, None, None, None)
        start = stamp_ns(msg.header.stamp)
        reference = start + (int(round(offsets.max()*1e9)) if offsets is not None else 0)

        def lookup(offset):
            tf = self.tf_buffer.lookup_transform_full(
                self.target_frame, rclpy.time.Time(nanoseconds=reference),
                msg.header.frame_id,
                rclpy.time.Time(nanoseconds=start+int(round(offset*1e9))),
                self.fixed_frame)
            return rigid(tf)
        try:
            # Sector membership is determined on the native scan in robot axes.
            # Historical rigid mounting, never latest-TF fallback.
            mounting = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp))
            rotation, translation = rigid(mounting)
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            r = np.asarray(msg.ranges, dtype=float)
            indices = np.arange(len(r))
            angles = msg.angle_min + indices*msg.angle_increment
            r, _ = filter_ranges(r, angles, (translation[0], translation[1], yaw),
                                 (math.radians(self.sector_min[sensor]),
                                  math.radians(self.sector_max[sensor])))
            valid = (np.isfinite(r) & (r > 0)
                     & (r >= max(msg.range_min, self.min_range))
                     & (r <= min(msg.range_max, self.max_range)))
            indices, angles, r = indices[valid], angles[valid], r[valid]
            if len(indices) == 0:
                return [], reference, offsets, indices, None
            native = np.column_stack((r*np.cos(angles), r*np.sin(angles)))
            labels = self._segment(angles, r, native, msg.angle_increment)
            xyz = np.column_stack((native, np.zeros(len(native))))
            times = offsets[indices] if offsets is not None else np.zeros(len(indices))
            corrected = deskew(xyz, times, lookup, self.knot_interval)
            clusters = []
            for label in np.unique(labels):
                mask = labels == label
                members = corrected[mask]
                if len(members) < self.min_points:
                    continue
                width = 2*float(np.max(np.linalg.norm(
                    members[:, :2]-members[:, :2].mean(axis=0), axis=1)))
                if not self.min_width <= width <= self.max_width:
                    continue
                point = _REPRESENTATIVE[self.pos_method](members)
                clusters.append((point, width, indices[mask]))
            return clusters, reference, offsets, indices, None
        except tf2_ros.TransformException:
            self.counts['tf_wait'] += 1
            return None

    def _diagnostics(self):
        drops = [dict(p.dropped) for p in self.pending]
        self.get_logger().info(f'LiDAR timing counts={dict(self.counts)} queue_drops={drops}')

    def _segment(self, ang, rng, xy, angle_increment):
        """ABD 분할 -> 각 점의 클러스터 라벨(0..k-1) 반환."""
        n = ang.size
        if n == 1:
            return np.zeros(1, dtype=np.int64)

        # 인접 유효점 사이의 각도차/거리. 무효점을 건너뛴 경우 dphi 가 커진다.
        dphi = np.abs(np.diff(ang))
        dist = np.linalg.norm(np.diff(xy, axis=0), axis=1)

        # sin(λ - Δφ) 가 0 이하가 되면 식이 무의미 -> 무조건 끊음.
        gap_limit = self.max_gap_bins * abs(angle_increment)
        forced = (dphi >= self.lambda_rad) | (dphi > gap_limit)

        denom = np.sin(self.lambda_rad - dphi)
        with np.errstate(divide='ignore', invalid='ignore'):
            d_max = rng[:-1] * np.sin(dphi) / denom + self.sigma_mult * self.sigma_r
        d_max = np.where(np.isfinite(d_max), d_max, 0.0)

        brk = forced | (dist > d_max)
        labels = np.concatenate(([0], np.cumsum(brk)))

        # 360° wraparound: 마지막 점과 첫 점이 실제로 이어져 있으면 같은 클러스터.
        if labels[-1] != labels[0] and n >= 2:
            dphi_w = 2.0 * math.pi - abs(ang[-1] - ang[0])
            if 0.0 < dphi_w < min(self.lambda_rad, gap_limit):
                d_w = float(np.linalg.norm(xy[0] - xy[-1]))
                dmax_w = (rng[-1] * math.sin(dphi_w) / math.sin(self.lambda_rad - dphi_w)
                          + self.sigma_mult * self.sigma_r)
                if d_w <= dmax_w:
                    labels[labels == labels[-1]] = labels[0]
        return labels

    def _publish(self, result, msg, sensor):
        clusters, reference, offsets, _, _ = result
        if clusters is None:
            return
        verified = offsets is not None
        self.counts['deskewed' if verified else 'timing_unknown_snapshot'] += 1
        arr = DetectedObjectArray()
        arr.header.stamp = rclpy.time.Time(nanoseconds=reference).to_msg()
        arr.header.frame_id = self.target_frame
        record = ObservationProvenance()
        record.header = msg.header
        record.sensor_id = self.scan_topics[sensor]
        record.observation_id = (f'{record.sensor_id}|{msg.header.frame_id}|'
                                 f'{stamp_ns(msg.header.stamp)}')
        record.timing_status = 1 if verified else 0
        arr.observations = [record]
        for point, width, indices in clusters:
            obj = DetectedObject()
            obj.header = arr.header
            obj.class_id, obj.class_name = CLASS_ID_LIDAR, CLASS_NAME_LIDAR
            obj.confidence, obj.source = 1.0, SOURCE_LIDAR
            obj.bbox.size_x = width
            obj.position.x, obj.position.y, obj.position.z = map(float, point)
            obj.distance = math.hypot(point[0], point[1])
            obs = ObservationProvenance()
            obs.header = msg.header
            obs.sensor_id = self.scan_topics[sensor]
            obs.beam_indices = indices.astype(int).tolist()
            obs.beam_time_offsets = offsets[indices].tolist() if verified else []
            obs.timing_status = 1 if verified else 0
            obs.observation_id = (f'{obs.sensor_id}|{msg.header.frame_id}|'
                                  f'{stamp_ns(msg.header.stamp)}|'
                                  + ','.join(map(str, obs.beam_indices)))
            obj.observations = [obs]
            obj.ego_motion_compensated = verified
            arr.objects.append(obj)
        self.counts['published_clusters'] += len(arr.objects)
        self.counts['last_age_ms'] = round(
            (self.get_clock().now().nanoseconds-stamp_ns(msg.header.stamp))*1e-6, 2)
        self.counts['reference_ns'] = reference
        self.get_logger().debug(f'LiDAR record={record.observation_id} reference_ns={reference}')
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

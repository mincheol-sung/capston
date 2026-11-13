#!/usr/bin/env python3
"""dual_scan_merger — 앞/뒤 RPLiDAR S2 두 스캔을 단일 /scan 으로 병합.

slam_toolbox는 LaserScan 토픽 하나만 받으므로, /scan_front + /scan_rear 를
공통 프레임(base_link)에 투영해 하나의 360° LaserScan(/scan)으로 래스터화한다.

원리:
  - 각 입력 스캔의 (range, angle) 점을 자기 프레임의 (x, y)로 변환
  - URDF에서 나온 TF(base_link <- laser_front/rear)로 base_link 좌표로 회전·평행이동
    (front 라이다의 yaw=180° 뒤집힘도 TF가 자동 처리)
  - base_link 기준 (range, bearing)으로 다시 계산해 출력 빈(bin)에 최소거리로 채움
  - 두 라이다가 서로/차체를 보는 문제는 range_min 및 (옵션) 담당 섹터 마스킹으로 완화

파라미터:
  input_topics      : ['/scan_front', '/scan_rear']
  input_sectors_deg : 각 입력의 유효 방위각 [min,max] (base_link 기준, 도).
                      [-180,180]이면 전체. 상호간섭 제거 시 front=[-90,90], rear=[90,-90] 등.
  target_frame      : 'base_link'
  output_topic      : '/scan'
  angle_increment_deg, range_min, range_max, publish_rate_hz
  max_scan_age_s    : 수신 후 이 시간보다 오래된 입력은 병합에서 제외
  max_pair_skew_s   : 두 입력 stamp 차이가 이 값보다 크면 오래된 쪽을 제외
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros


def yaw_from_quat(x, y, z, w):
    """쿼터니언 -> yaw(z축 회전). 2D 투영용."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DualScanMerger(Node):
    def __init__(self):
        super().__init__('dual_scan_merger')

        self.input_topics = self.declare_parameter(
            'input_topics', ['/scan_front', '/scan_rear']).value
        # 각 입력의 담당 방위각 섹터(도). 기본은 전체 360°.
        sectors = self.declare_parameter(
            'input_sectors_deg', [-180.0, 180.0, -180.0, 180.0]).value
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value
        self.output_topic = self.declare_parameter('output_topic', '/scan').value
        self.angle_increment = math.radians(
            self.declare_parameter('angle_increment_deg', 0.5).value)
        self.range_min = self.declare_parameter('range_min', 0.15).value
        self.range_max = self.declare_parameter('range_max', 30.0).value
        rate = self.declare_parameter('publish_rate_hz', 12.0).value
        self.max_scan_age_s = float(
            self.declare_parameter('max_scan_age_s', 0.25).value)
        self.max_pair_skew_s = float(
            self.declare_parameter('max_pair_skew_s', 0.05).value)
        self.require_all_inputs = bool(
            self.declare_parameter('require_all_inputs', True).value)

        # 섹터 파라미터를 입력별 [min,max] 쌍으로 정리
        self.sectors = []
        for i in range(len(self.input_topics)):
            lo = math.radians(sectors[2 * i])
            hi = math.radians(sectors[2 * i + 1])
            self.sectors.append((lo, hi))

        # 출력 스캔 격자
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.n_bins = int(round((self.angle_max - self.angle_min) / self.angle_increment))

        # TF (정적이므로 최초 1회 조회 후 캐시)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_cache = {}  # frame_id -> (tx, ty, yaw)

        # 최신 스캔 버퍼. 메시지 stamp가 아니라 실제 수신 시각도 함께 저장한다.
        # 라이다 하나가 멈췄을 때 마지막 메시지를 영원히 재사용하면 유령 장애물이
        # 생기므로, _publish_merged 에서 수신 후 경과시간을 검사한다.
        self.latest = {topic: None for topic in self.input_topics}
        self.stale_topics = set()
        self.last_output_signature = None
        self.last_skew_warning_ns = 0
        for topic in self.input_topics:
            self.create_subscription(
                LaserScan, topic,
                lambda msg, t=topic: self._on_scan(t, msg),
                qos_profile_sensor_data)

        # /scan 은 RELIABLE로 발행 → slam_toolbox·ros2 topic hz 등 RELIABLE 구독자와 호환
        # (RELIABLE 발행은 BEST_EFFORT 구독자도 받으므로 모든 소비자와 호환됨).
        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._publish_merged)
        self.get_logger().info(
            f'dual_scan_merger: {self.input_topics} -> {self.output_topic} '
            f'({self.n_bins} bins @ {math.degrees(self.angle_increment):.2f} deg)')

    def _on_scan(self, topic, msg):
        self.latest[topic] = (msg, self.get_clock().now().nanoseconds)
        if topic in self.stale_topics:
            self.stale_topics.remove(topic)
            self.get_logger().info(f'{topic} scan 수신 복구')

    @staticmethod
    def _stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _fresh_inputs(self):
        """신선한 입력만 반환하고, 지나치게 어긋난 쌍에서는 오래된 쪽을 뺀다."""
        now_ns = self.get_clock().now().nanoseconds
        fresh = []
        for i, topic in enumerate(self.input_topics):
            entry = self.latest[topic]
            if entry is None:
                continue
            msg, received_ns = entry
            age_s = max(0.0, (now_ns - received_ns) * 1e-9)
            source_age = (now_ns - self._stamp_ns(msg)) * 1e-9
            age_s = max(age_s, source_age)
            if (self._stamp_ns(msg) <= 0 or source_age < -0.1
                    or (self.max_scan_age_s > 0.0 and age_s > self.max_scan_age_s)):
                if topic not in self.stale_topics:
                    self.stale_topics.add(topic)
                    self.get_logger().warning(
                        f'{topic} scan stale ({age_s:.3f} s > '
                        f'{self.max_scan_age_s:.3f} s) — 병합에서 제외')
                continue
            fresh.append((i, topic, msg))

        if len(fresh) == 2 and self.max_pair_skew_s > 0.0:
            stamp_a = self._stamp_ns(fresh[0][2])
            stamp_b = self._stamp_ns(fresh[1][2])
            skew_s = abs(stamp_a - stamp_b) * 1e-9
            if skew_s > self.max_pair_skew_s:
                older = 0 if stamp_a < stamp_b else 1
                dropped = fresh[older][1]
                fresh.pop(older)
                if now_ns - self.last_skew_warning_ns > 5_000_000_000:
                    self.last_skew_warning_ns = now_ns
                    self.get_logger().warning(
                        f'라이다 stamp 차이 {skew_s:.3f} s > '
                        f'{self.max_pair_skew_s:.3f} s — 오래된 {dropped} 제외')
        return fresh

    def _get_tf(self, frame_id):
        """base_link <- frame_id 의 (tx, ty, yaw). 캐시."""
        if frame_id in self.tf_cache:
            return self.tf_cache[frame_id]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        val = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        self.tf_cache[frame_id] = val
        return val

    def _scan_to_base_points(self, msg, sector):
        """LaserScan -> base_link 기준 (range, bearing) 배열. 섹터 마스킹 적용."""
        tf = self._get_tf(msg.header.frame_id)
        if tf is None:
            return None
        tx, ty, yaw = tf

        n = len(msg.ranges)
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        # 유효 범위 필터(inf/nan/범위밖 제거)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        if ranges.size == 0:
            return np.empty(0), np.empty(0)

        # 센서 프레임 (x,y) -> base_link (x,y)
        px = ranges * np.cos(angles)
        py = ranges * np.sin(angles)
        c, s = math.cos(yaw), math.sin(yaw)
        bx = tx + c * px - s * py
        by = ty + s * px + c * py

        br = np.hypot(bx, by)
        bb = np.arctan2(by, bx)

        # 담당 섹터 마스킹(상호간섭 제거용)
        lo, hi = sector
        if not (abs(lo + math.pi) < 1e-6 and abs(hi - math.pi) < 1e-6):
            if lo <= hi:
                keep = (bb >= lo) & (bb <= hi)
            else:  # 랩어라운드 섹터 (예: 90°~-90° = 후방)
                keep = (bb >= lo) | (bb <= hi)
            br, bb = br[keep], bb[keep]
        return br, bb

    def _publish_merged(self):
        merged = np.full(self.n_bins, np.inf, dtype=np.float64)
        fresh = self._fresh_inputs()
        if not fresh or (self.require_all_inputs and len(fresh) != len(self.input_topics)):
            return

        # 같은 입력 조합을 타이머가 반복 발행하지 않는다. 특히 한 센서가 멈췄을
        # 때 동일 stamp가 계속 나가 Cartographer/소비자가 새 scan으로 오인하는
        # 문제를 막는다.
        signature = tuple(
            (topic, self._stamp_ns(msg)) for _, topic, msg in fresh)
        if signature == self.last_output_signature:
            return
        transformed_any = False
        for i, _topic, msg in fresh:
            res = self._scan_to_base_points(msg, self.sectors[i])
            if res is None:
                continue  # TF 아직 없음
            transformed_any = True
            br, bb = res
            if br.size == 0:
                continue
            # 방위각 -> 빈 인덱스, 같은 빈은 최소거리 유지
            idx = np.floor((bb - self.angle_min) / self.angle_increment).astype(int)
            np.clip(idx, 0, self.n_bins - 1, out=idx)
            in_rng = (br >= self.range_min) & (br <= self.range_max)
            idx, br = idx[in_rng], br[in_rng]
            np.minimum.at(merged, idx, br)

        if not transformed_any:
            return
        self.last_output_signature = signature

        out = LaserScan()
        # 병합 LaserScan 하나로 두 비동기 회전식 라이다의 포인트별 시각을 정확히
        # 표현할 수는 없다. timestamp는 포함된 입력 중 가장 오래된 scan 시작
        # 시각으로 둔다. Cartographer는 이 병합본 대신 원래 timing을 보존하는
        # /scan_front_carto, /scan_rear_carto를 사용한다.
        #
        # 아래 time_increment=0, scan_time=0 은 "포인트별 시각을 모른다"는 표시이자
        # 동시에 복원 불가능 선언이다. 인지 lidar_cluster는 /scan_front와
        # /scan_rear를 직접 사용하고 원본 빔 인덱스와 timing 검증 상태를 보존한다.
        # 이 병합본에서 provenance나 빔 시각을 복원하지 않는다.
        # Consumers must age the oldest contributing capture, not the newest.
        oldest_msg = min((item[2] for item in fresh), key=self._stamp_ns)
        out.header.stamp = oldest_msg.header.stamp
        out.header.frame_id = self.target_frame
        out.angle_min = self.angle_min
        out.angle_max = self.angle_max
        out.angle_increment = self.angle_increment
        out.time_increment = 0.0
        out.scan_time = 0.0
        out.range_min = self.range_min
        out.range_max = self.range_max
        out.ranges = merged.tolist()  # inf = 측정 없음(유효)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DualScanMerger()
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

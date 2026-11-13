#!/usr/bin/env python3
"""object_tracker — 융합 검출을 트랙으로 잇는다.

    /detections_tracking (base_link) ──▶ TF ──▶ odom ──▶ MultiObjectTracker
                                                              │
                                        /tracks (odom) ◀──────┤
                         /tracked_object_modes (odom) ◀───────┤
                                        /tracks_markers ◀─────┘

이 노드는 추정과 지도 의존 출력 필터의 경계를 나눈다. 추정 로직은 전부
core/ 에 있고 ROS 를 모른다.

  1. TF 로 검출을 추적 프레임(odom)으로 옮긴다 — 공분산도 같이 회전시킨다.
  2. core.MultiObjectTracker 를 프레임마다 한 번 돌린다.
  3. 통합 결과는 기존 TrackedObjectArray로, CV/CT/STATIC posterior는 별도 internal
     ImmTrackedObjectArray로 낸다.
  4. 선택 옵션인 지도 벽 필터는 기본 OFF: 벽 근처 정지한 사람도 출력한다.

공분산 회전이 왜 필요한가:
  camera_lidar_fusion 이 채운 R 은 극좌표에서 만들어 회전시킨 이방성 행렬이다
  (C2 실측에서 반경/횡 오차 비가 4m 에서 3.5배). base_link 에서 odom 으로 옮길
  때 위치만 옮기고 R 을 그대로 두면 오차 타원의 장축이 엉뚱한 방향을 가리켜
  association 게이트와 Kalman gain 이 둘 다 틀어진다.

      p_odom = t + C p_base,    R_odom = C R_base C^T,    C = R(yaw)

  지면 주행 로봇이라 odom <- base_link 는 평면 변환이고 yaw 만 쓰면 된다.
  roll/pitch 가 크게 들어오면 경고를 낸다.

검출 시각의 TF 를 쓴다:
  최신 TF(Time())로 대신하면 로봇이 움직이는 동안 검출 지연만큼 위치가 밀린다.
  0.3 m/s 로 100ms 밀리면 3cm 인데, 이 시스템의 융합 위치 잡음이 4m 에서
  수 mm 수준이라 잡음보다 한 자릿수 큰 계통오차가 된다. TF 준비 전 입력은
  제한 큐에서 비동기로 기다린다. 큐가 만료돼야 프레임을 버린다.
"""
import copy
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rcl_interfaces.msg import SetParametersResult

import tf2_ros
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from amr_interfaces.msg import (
    DetectedObjectArray,
    ImmModeState,
    ImmTrackedObject,
    ImmTrackedObjectArray,
    TrackedObject,
    TrackedObjectArray,
    TrackedObservationArray,
)

from object_tracker.core.config import TrackerConfig
from object_tracker.core.grid_filter import (
    GridFilterConfig,
    OccupancyDistanceGrid,
    StaticTrackSuppressor,
)
from object_tracker.core.nis import NisAccumulator
from object_tracker.core.measurements import tracking_measurement
from object_tracker.core.pending import PendingMeasurements
from object_tracker.core.track import DYNAMIC, MOTION_NAME, STATIC, UNKNOWN
from object_tracker.core.tracker import MultiObjectTracker
from object_tracker.observation_links import observation_links

MOTION_COLOR = {
    UNKNOWN: (0.6, 0.6, 0.6, 0.8),
    STATIC: (0.2, 0.6, 1.0, 0.8),
    DYNAMIC: (1.0, 0.3, 0.1, 0.9),
}


def yaw_of(q):
    """쿼터니언 -> yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def oldest_capture_ns(msg):
    """Use original capture age even when the coordinate epoch was advanced."""
    stamps = [rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds]
    for record in [msg, *msg.objects]:
        for obs in getattr(record, 'observations', []):
            stamps.append(rclpy.time.Time.from_msg(obs.header.stamp).nanoseconds)
    return min(stamps)


class ObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('object_tracker_node')

        self.in_topic = self.declare_parameter(
            'detections_topic', '/detections_tracking').value
        self.out_topic = self.declare_parameter(
            'tracks_topic', '/tracks').value
        self.modes_topic = self.declare_parameter(
            'tracked_modes_topic', '/tracked_object_modes').value
        self.observations_topic = self.declare_parameter(
            'tracked_observations_topic', '/tracked_observations').value
        # 추적 프레임. odom 을 쓰는 근거는 core/tracker.py 독스트링 참고.
        self.track_frame = self.declare_parameter('track_frame', 'odom').value
        self.tf_timeout = float(
            self.declare_parameter('tf_timeout', 0.25).value)
        self.pending = PendingMeasurements(
            max_wait=self.tf_timeout,
            capacity=int(self.declare_parameter('tf_queue_size', 32).value))
        self.max_observation_age = float(self.declare_parameter('max_observation_age', 0.75).value)
        self._observation_ids = {}
        self._reported_drops = 0
        self.publish_markers = bool(
            self.declare_parameter('publish_markers', True).value)
        self.publish_tentative = bool(
            self.declare_parameter('publish_tentative', False).value)

        # 지도 필터는 추적기 운동모델과 무관한 ROS 경계 기능이다. 파라미터에는
        # grid_filter_ 접두사를 붙여 tracker 설정과 명확히 구분한다.
        self.grid_cfg = GridFilterConfig()
        for name, default in GridFilterConfig.field_defaults().items():
            value = self.declare_parameter(
                'grid_filter_' + name, default).value
            setattr(self.grid_cfg, name, type(default)(value))
        self.grid_cfg.validate()
        self.grid = OccupancyDistanceGrid()
        self.grid_suppressor = StaticTrackSuppressor(self.grid_cfg)
        self._last_map = None

        # 설계 파라미터는 core.config.TrackerConfig 가 단일 출처다.
        self.cfg = TrackerConfig()
        for name, default in TrackerConfig.field_defaults().items():
            value = self.declare_parameter(name, default).value
            setattr(self.cfg, name, type(default)(value))
        self.cfg.validate()

        # NIS 진단. sigma_a 를 실측할 때만 켠다 — 켜도 비용은 덧셈 몇 번이다.
        # 로그의 sigma_a 추정값은 한 번에 맞지 않는다. 그 값을 파라미터로 다시
        # 넣고 3~4회 반복해야 수렴한다 (core/nis.py 의 수렴 표).
        self.nis_period = float(
            self.declare_parameter('nis_report_period', 0.0).value)
        self.nis = (NisAccumulator(gate=self.cfg.gate_chi2)
                    if self.nis_period > 0.0 else None)
        self.tracker = MultiObjectTracker(self.cfg, nis=self.nis)
        if self.nis is not None:
            self.create_timer(self.nis_period, self._report_nis)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            TrackedObjectArray, self.out_topic, 10)
        self.modes_pub = self.create_publisher(
            ImmTrackedObjectArray, self.modes_topic, 10)
        self.observations_pub = self.create_publisher(
            TrackedObservationArray, self.observations_topic, 10)
        self.marker_pub = (
            self.create_publisher(MarkerArray, self.out_topic + '_markers', 10)
            if self.publish_markers else None)
        self.create_subscription(DetectedObjectArray, self.in_topic,
                                 self._on_detections, 10)
        self.create_timer(0.01, self._drain_detections)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.grid_cfg.map_topic, self._on_map, map_qos)

        self.add_on_set_parameters_callback(self._on_param_set)
        self._warned_empty_cov = False

        self.get_logger().info(
            f'object_tracker_node 시작: {self.in_topic} -> {self.out_topic} '
            f'(frame={self.track_frame}, sigma_a={self.cfg.sigma_a} m/s^1.5, '
            f'gate={self.cfg.gate_chi2:.2f}, '
            f'IMM STATIC/CV/CT, probability thresholds '
            f'{self.cfg.static_enter_probability:.2f}/'
            f'{self.cfg.dynamic_enter_probability:.2f}, '
            f'grid_filter={self.grid_cfg.enabled}, '
            f'map={self.grid_cfg.map_topic}, '
            f'wall={self.grid_cfg.enter_distance:.2f}/'
            f'{self.grid_cfg.exit_distance:.2f}m)')

    def _report_nis(self):
        """누적 NIS 를 로그로 낸다. 누적은 초기화하지 않는다 — 표본이 쌓일수록
        정밀해지므로(상대오차 ~ 1/sqrt(N)) 구간별이 아니라 누적이 맞다."""
        report = self.nis.report(self.cfg.sigma_a)
        report += (
            ' [IMM fused innovation; sigma_a 역산값은 단일 CV 보정에 '
            '직접 사용 금지]')
        self.get_logger().info(report)

    def _on_param_set(self, params):
        names = TrackerConfig.field_defaults()
        grid_names = GridFilterConfig.field_defaults()
        candidate = copy.copy(self.cfg)
        grid_candidate = copy.copy(self.grid_cfg)
        updates = {}
        grid_updates = {}
        for p in params:
            if p.name in names:
                value = type(names[p.name])(p.value)
                setattr(candidate, p.name, value)
                updates[p.name] = value
                continue
            if not p.name.startswith('grid_filter_'):
                continue
            name = p.name[len('grid_filter_'):]
            if name not in grid_names:
                continue
            value = type(grid_names[name])(p.value)
            if name == 'map_topic' and value != self.grid_cfg.map_topic:
                return SetParametersResult(
                    successful=False,
                    reason='grid_filter_map_topic 변경은 노드 재시작 필요')
            setattr(grid_candidate, name, value)
            grid_updates[name] = value
        try:
            candidate.validate()
            grid_candidate.validate()
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        # Existing Track instances hold this config object, so commit the
        # validated values in place instead of replacing self.cfg.
        for name, value in updates.items():
            setattr(self.cfg, name, value)
            self.get_logger().warn(f'파라미터 변경: {name} = {value}')
        for name, value in grid_updates.items():
            setattr(self.grid_cfg, name, value)
            self.get_logger().warn(
                f'파라미터 변경: grid_filter_{name} = {value}')
        if 'enabled' in grid_updates:
            self.grid_suppressor.reset()
        if 'occupied_threshold' in grid_updates and self._last_map is not None:
            self._update_grid(self._last_map)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #

    def _lookup(self, frame_id, stamp):
        """odom <- frame_id 의 (평면 평행이동, 2x2 회전). 없으면 None."""
        if not frame_id or rclpy.time.Time.from_msg(stamp).nanoseconds <= 0:
            return None
        if frame_id == self.track_frame:
            return np.zeros(2), np.eye(2)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.track_frame, frame_id, rclpy.time.Time.from_msg(stamp))
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        if abs(q.x) > 1e-3 or abs(q.y) > 1e-3:
            self.get_logger().warn(
                f'{frame_id} 에 roll/pitch 가 있다 (qx={q.x:.4f}, qy={q.y:.4f}). '
                '평면 변환 가정이 깨진다.', throttle_duration_sec=10.0)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y])
        return t, rot(yaw_of(q))

    def _measurement(self, obj, t, C):
        """검출 하나 -> (추적 프레임 z, 추적 프레임 R)."""
        p = C @ np.array([obj.position.x, obj.position.y]) + t
        R = np.array(obj.position_covariance, dtype=float).reshape(2, 2)
        if not np.any(R):
            # 전부 0 = 미기입. /detections_fused 는 항상 채우므로 정상 경로는
            # 여기 오지 않는다. 원본 /detections 를 물렸을 때 R 이 특이행렬이
            # 되어 게이트가 전부 inf 가 되는 것만 막는다.
            if not self._warned_empty_cov:
                self.get_logger().warn(
                    f'position_covariance 가 비어 있다. fallback sigma '
                    f'{self.cfg.fallback_sigma} m 로 대체한다. '
                    f'{self.in_topic} 가 fusion이 낸 토픽인지 확인할 것.')
                self._warned_empty_cov = True
            R = np.eye(2) * self.cfg.fallback_sigma ** 2
            return p, R
        return p, C @ R @ C.T

    def _on_map(self, msg):
        """latched /map을 벽까지의 거리장으로 바꾼다."""
        self._last_map = msg
        try:
            self._update_grid(msg)
        except ValueError as exc:
            self.get_logger().warn(f'grid filter map 무시: {exc}')

    def _update_grid(self, msg):
        info = msg.info
        self.grid.update(
            msg.data,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            yaw_of(info.origin.orientation),
            msg.header.frame_id or 'map',
            self.grid_cfg.occupied_threshold,
        )
        self.get_logger().info(
            f'grid filter map 준비: {info.width}x{info.height} @ '
            f'{info.resolution:.3f}m, frame={self.grid.frame_id}',
            throttle_duration_sec=5.0)

    def _filter_tracks(self, tracks, stamp):
        """지도 벽 근처의 STATIC 트랙만 출력 목록에서 제외한다.

        map/TF가 없거나 조회에 실패하면 전부 통과시킨다(fail-open). 추적기 내부
        상태는 건드리지 않으며 세 출력 토픽에 동일한 필터 결과를 사용한다.
        """
        if not self.grid_cfg.enabled or not self.grid.ready or not tracks:
            return tracks

        if self.grid.frame_id == self.track_frame:
            t, C = np.zeros(2), np.eye(2)
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.grid.frame_id,
                    self.track_frame,
                    rclpy.time.Time.from_msg(stamp))
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(
                    f'grid filter TF {self.grid.frame_id}<-'
                    f'{self.track_frame} 없음, 필터 우회: {exc}',
                    throttle_duration_sec=2.0)
                return tracks
            t = np.array([tf.transform.translation.x,
                          tf.transform.translation.y])
            C = rot(yaw_of(tf.transform.rotation))

        track_points = np.asarray(
            [track.kf.x[:2] for track in tracks], dtype=np.float64)
        map_points = track_points @ C.T + t
        distances = self.grid.distances(map_points)
        kept, suppressed = self.grid_suppressor.select(
            tracks, distances, STATIC)
        if suppressed:
            ids = ','.join(str(track.id) for track in suppressed)
            self.get_logger().info(
                f'grid filter: 벽 근처 STATIC {len(suppressed)}개 억제 '
                f'(id={ids})', throttle_duration_sec=2.0)
        return kept

    def _on_detections(self, msg):
        stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        now_ns = self.get_clock().now().nanoseconds
        capture_ns = oldest_capture_ns(msg)
        age = (now_ns-capture_ns)*1e-9
        if (capture_ns <= 0 or stamp_ns > now_ns + 50_000_000
                or age > self.max_observation_age or age < -0.05):
            self.pending.dropped['stale_or_future'] += 1
            return
        if not msg.header.frame_id:
            self.pending.dropped['missing_frame'] += 1
            return
        if any(obj.header != msg.header for obj in msg.objects):
            self.pending.dropped['epoch_mismatch'] += 1
            return
        cutoff = stamp_ns-int(self.max_observation_age*1e9)
        self._observation_ids = {key: t for key, t in self._observation_ids.items() if t >= cutoff}
        identities = [obs.observation_id for obj in msg.objects for obs in obj.observations
                      if obs.observation_id]
        if len(identities) != len(set(identities)) or any(
                identity in self._observation_ids for identity in identities):
            self.pending.dropped['duplicate_observation'] += 1
            return
        self._observation_ids.update({identity: stamp_ns for identity in identities})
        self.pending.add(stamp_ns, msg, time.monotonic())
        self._drain_detections()

    def _drain_detections(self):
        self.pending.drain(
            time.monotonic(),
            lambda msg: self._lookup(msg.header.frame_id, msg.header.stamp),
            self._process_detections)
        dropped = sum(self.pending.dropped.values())
        if dropped > self._reported_drops:
            self.get_logger().warning(
                f'Tracker input drops (cumulative): {dict(self.pending.dropped)}',
                throttle_duration_sec=2.0)
            self._reported_drops = dropped

    def _process_detections(self, msg, lookup):
        if hasattr(self, 'max_observation_age'):
            age = (self.get_clock().now().nanoseconds
                   - oldest_capture_ns(msg))*1e-9
            if age > self.max_observation_age:
                self.pending.dropped['stale_after_tf_wait'] += 1
                return
        t, C = lookup

        zs, Rs, dets = [], [], []
        for obj in msg.objects:
            measured = tracking_measurement(obj, self.cfg.fallback_sigma)
            if measured is None:
                continue
            position, covariance = measured
            z, R = C @ position + t, C @ covariance @ C.T
            if not np.all(np.isfinite(z)) or not np.all(np.isfinite(R)):
                continue
            zs.append(z)
            Rs.append(R)
            dets.append(obj)

        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        tracks = self.tracker.step(stamp, zs, Rs, dets)
        if self.tracker.last_superseded:
            replacements = ', '.join(
                f'{old}->{new}' for old, new in self.tracker.last_superseded)
            self.get_logger().info(
                f'person semantic owner superseded: {replacements}',
                throttle_duration_sec=1.0)
        if self.publish_tentative:
            tracks = self.tracker.tracks
        tracks = self._filter_tracks(tracks, msg.header.stamp)

        self.pub.publish(self._to_msg(tracks, msg.header.stamp))
        self.modes_pub.publish(self._to_modes_msg(tracks, msg.header.stamp))
        self.observations_pub.publish(observation_links(
            tracks, self.tracker.last_measurement_matches, dets, msg.header))
        if self.marker_pub is not None:
            self.marker_pub.publish(self._to_markers(tracks, msg.header.stamp))

    # ------------------------------------------------------------------ #

    def _to_msg(self, tracks, stamp):
        out = TrackedObjectArray()
        out.header.stamp = stamp
        out.header.frame_id = self.track_frame
        for trk in tracks:
            o = TrackedObject()
            o.header = out.header
            o.track_id = int(trk.id)
            o.class_id = int(trk.class_id)
            o.class_name = trk.class_name
            o.pose.pose.position.x = float(trk.kf.x[0])
            o.pose.pose.position.y = float(trk.kf.x[1])
            o.pose.pose.orientation.w = 1.0    # 물체 방위는 관측하지 않는다
            o.velocity.twist.linear.x = float(trk.kf.x[2])
            o.velocity.twist.linear.y = float(trk.kf.x[3])
            # 6x6 row-major 의 (x,y) 블록만 채운다. 나머지는 미관측이라 0.
            P_pp, P_vv = trk.kf.P_pp, trk.kf.P_vv
            for cov, blk in ((o.pose.covariance, P_pp),
                             (o.velocity.covariance, P_vv)):
                cov[0], cov[1] = float(blk[0, 0]), float(blk[0, 1])
                cov[6], cov[7] = float(blk[1, 0]), float(blk[1, 1])
            # 두 6x6 사이의 교차항. 등속 외삽 공분산에 필요하다 (msg 주석 참고).
            o.position_velocity_covariance = [
                float(v) for v in trk.kf.P_pv.flatten()]
            o.motion_state = int(trk.motion)
            o.is_dynamic = bool(trk.is_dynamic)
            o.age = float(trk.age)
            out.objects.append(o)
        return out

    def _to_modes_msg(self, tracks, stamp):
        out = ImmTrackedObjectArray()
        out.header.stamp = stamp
        out.header.frame_id = self.track_frame
        for trk in tracks:
            obj = ImmTrackedObject()
            obj.track_id = int(trk.id)
            obj.class_id = int(trk.class_id)
            obj.class_name = trk.class_name
            obj.confidence = float(trk.confidence)
            obj.radius = float(trk.radius)
            obj.lifecycle_state = int(trk.lifecycle)
            obj.motion_state = int(trk.motion)
            obj.measurement_age = max(0.0, trk.stamp - trk.last_update)
            obj.semantic_age = (max(0.0, trk.stamp - trk.last_semantic_update)
                                if trk.last_semantic_update is not None else -1.0)
            obj.last_measurement_source = int(trk.last_measurement_source)
            obj.mean_sojourn_cv = float(self.cfg.mean_sojourn_cv)
            obj.mean_sojourn_ct = float(self.cfg.mean_sojourn_ct)
            obj.mean_sojourn_static = float(self.cfg.mean_sojourn_static)
            obj.static_to_cv = float(self.cfg.static_to_cv)
            obj.cv_to_static = float(self.cfg.cv_to_static)
            obj.ct_to_static = float(self.cfg.ct_to_static)
            for index, (state, covariance, probability) in enumerate(
                    trk.estimator.mode_posteriors()):
                mode = ImmModeState()
                mode.model_type = int(index)
                mode.probability = float(probability)
                mode.state = [float(value) for value in state]
                mode.covariance = [
                    float(value) for value in covariance.flatten()]
                obj.modes.append(mode)
            out.objects.append(obj)
        return out

    def _to_markers(self, tracks, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for trk in tracks:
            x, y, vx, vy = trk.kf.x[:4]
            r, g, b, a = MOTION_COLOR[trk.motion]

            body = Marker()
            body.header.stamp = stamp
            body.header.frame_id = self.track_frame
            body.ns = 'track'
            body.id = int(trk.id)
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x, body.pose.position.y = float(x), float(y)
            body.pose.orientation.w = 1.0
            body.scale = Vector3(x=0.35, y=0.35, z=0.05)
            body.color = ColorRGBA(r=r, g=g, b=b, a=a)
            arr.markers.append(body)

            label = Marker()
            label.header = body.header
            label.ns = 'track_label'
            label.id = int(trk.id)
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y = float(x), float(y)
            label.pose.position.z = 0.4
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            label.text = (f'#{trk.id} {trk.class_name or "-"}\n'
                          f'{MOTION_NAME[trk.motion]} {trk.kf.speed:.2f} m/s\n'
                          f'P(CV/CT/S)={trk.estimator.mu[0]:.2f}/'
                          f'{trk.estimator.mu[1]:.2f}/{trk.estimator.mu[2]:.2f}')
            arr.markers.append(label)

            if trk.motion == DYNAMIC:
                vel = Marker()
                vel.header = body.header
                vel.ns = 'track_velocity'
                vel.id = int(trk.id)
                vel.type = Marker.ARROW
                vel.action = Marker.ADD
                vel.points = [Point(x=float(x), y=float(y), z=0.05),
                              Point(x=float(x + vx), y=float(y + vy), z=0.05)]
                vel.scale = Vector3(x=0.05, y=0.10, z=0.0)
                vel.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
                vel.pose.orientation.w = 1.0
                arr.markers.append(vel)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

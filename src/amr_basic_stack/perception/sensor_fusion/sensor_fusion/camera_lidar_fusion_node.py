#!/usr/bin/env python3
"""Bounded native-observation fusion at a common base_link epoch via odom.

Original captures and beam membership survive all outputs. Object velocity is
unknown: its time-dependent uncertainty enters both association and fusion.
"""
from collections import Counter
import copy
import math
import time

from amr_interfaces.msg import DetectedObject, DetectedObjectArray
import numpy as np
from object_tracker.core.association import assign
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
import tf2_ros

from .observation_time import (
    FusionQueue, observation_window, rigid, stamp_ns, temporal_variance)

SOURCE_CAMERA = 0
SOURCE_LIDAR = 1
SOURCE_FUSED = 2


def rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


class PolarNoiseModel:
    """극좌표에서 만든 뒤 회전시키는 측정 공분산 R(d).

    xy 대각행렬로 두면 안 된다. C2 실측에서 반경/횡 오차 비가 2m 에서 2.0배,
    4m 에서 3.5배라 오차 타원이 시선 방향으로 길쭉하다. 그 이방성이 사라지면
    "거리는 라이다, 방위는 카메라" 라는 가중치 분리가 나오지 않는다.

        R(d, theta) = J(theta) * diag(sigma_r^2, sigma_t^2) * J(theta)^T
    """

    def __init__(self, r_coef, r_exp, t_coef, t_exp, r_floor=0.0, r_break=None):
        self.r_coef, self.r_exp = r_coef, r_exp      # sigma_r = r_coef * d^r_exp  [m]
        self.t_coef, self.t_exp = t_coef, t_exp      # sigma_t = t_coef * d^t_exp  [m]
        # r_break 이상에서는 멱함수를 벗어나 평탄해진다 -> r_floor 로 고정.
        self.r_floor, self.r_break = r_floor, r_break

    def sigma_r(self, d):
        if self.r_break is not None and d >= self.r_break:
            return self.r_floor
        return self.r_coef * d ** self.r_exp

    def sigma_t(self, d):
        return self.t_coef * d ** self.t_exp

    def cov(self, x, y):
        d = math.hypot(x, y)
        if d < 1e-6:
            d = 1e-6
        J = rot(math.atan2(y, x))
        S = np.diag([self.sigma_r(d) ** 2, self.sigma_t(d) ** 2])
        return J @ S @ J.T


class RepresentativeOffset:
    """두 센서 대표점 정의 차이 delta(d). 잡음이 아니라 계통 차이다.

    카메라는 물체 앞면을, 라이다는 앞면 호의 centroid 를 잰다. 거기에 ABD 의
    D_max 가 거리에 비례해 커지며 배경을 흡수하는 성분이 더해진다(C2 에서
    클러스터 폭이 참폭 대비 1m +1%, 5m +52%).

    빼지 않으면 게이트가 자기 자신을 기각한다 — C2 실측에서 보정 없는 진짜 쌍의
    chi^2 가 1.1m 704, 2.1m 84, 4.1m 21 이었다(문턱 9.21).
    """

    def __init__(self, r_coef, r_exp, t_const):
        self.r_coef, self.r_exp, self.t_const = r_coef, r_exp, t_const

    def apply(self, x, y):
        """라이다 관측을 카메라 대표점 규약으로 옮긴다.

        카메라 규약(물체 앞면)으로 통일하는 이유: 충돌 회피에 필요한 것은
        가장 가까운 표면이지 centroid 가 아니다.
        """
        d = math.hypot(x, y)
        if d < 1e-6:
            return x, y
        th = math.atan2(y, x)
        d2 = d + self.r_coef * d ** self.r_exp        # delta_r 은 음수(카메라가 더 가깝다)
        th2 = th + self.t_const / d                   # delta_t 는 상수 [m]
        return d2 * math.cos(th2), d2 * math.sin(th2)


class CameraLidarFusionNode(Node):

    def __init__(self):
        super().__init__('camera_lidar_fusion_node')

        self.cam_topic = self.declare_parameter('camera_topic', '/detections').value
        self.lid_topic = self.declare_parameter('lidar_topic', '/lidar_clusters').value
        self.out_topic = self.declare_parameter('output_topic', '/detections_fused').value
        self.tracking_topic = self.declare_parameter(
            'tracking_output_topic', '/detections_tracking').value
        self.lidar_obstacles_topic = self.declare_parameter(
            'lidar_obstacles_topic', '/lidar_obstacles').value
        # Legacy/debug escape hatch. The safe default preserves the semantic-only
        # /detections_fused contract; tracking uses /detections_tracking.
        self.forward_unmatched_lidar = bool(self.declare_parameter(
            'forward_unmatched_lidar_to_fused', False).value)

        # Preserve the existing 67ms epoch tolerance; holdback is independent.
        self.slop = float(self.declare_parameter('sync_slop', 0.067).value)
        self.queue = int(self.declare_parameter('sync_queue', 10).value)

        # ── 카메라 R(d).  C2: sigma_r = 0.321mm*d^2.741 (R^2 0.991),
        #                        sigma_t = 0.560mm*d^1.450 (R^2 0.945)
        # 지수가 2 를 넘는 것은 소자 잡음이 아니라 roi_median_3d 추정량의 성질이다.
        # bbox 가 1/d 로 줄면 median 에 들어가는 픽셀이 1/d^2 로 줄어 표준오차가
        # d 에 비례해 커지고, 스테레오의 d^2 에 그 항이 곱해진다.
        self.cam_model = PolarNoiseModel(
            r_coef=float(self.declare_parameter('cam_sigma_r_coef', 0.321e-3).value),
            r_exp=float(self.declare_parameter('cam_sigma_r_exp', 2.741).value),
            t_coef=float(self.declare_parameter('cam_sigma_t_coef', 0.560e-3).value),
            t_exp=float(self.declare_parameter('cam_sigma_t_exp', 1.450).value))

        # ── 라이다 R(d).  C2: sigma_r = 0.331mm*d^0.354 (1.1~4.1m),
        #                        5m 이상은 1.2~1.9mm 로 평탄 -> 1.6mm 고정.
        # sigma_t 는 거리 멱함수로 맞지 않았다(R^2 음수). 실제로는 클러스터 폭의
        # 시간 표준편차의 절반이다 — sigma_t = 0.974 * (w_sd/2), r=0.923, n=42.
        # ABD 의 breakpoint 가 배경 점을 붙였다 뗐다 하며 centroid 를 밀어내는 것이지
        # 각분해능과는 무관하다. 즉 거리가 아니라 환경(배경까지의 간격 대 D_max)의
        # 함수다. 여기서는 C2 관측 중앙값 6.5mm 를 상수로 쓴다.
        # TODO: lidar_cluster_node 가 클러스터 폭을 함께 발행하고, 트랙 ID 가 생긴
        #       뒤 그 시간 변동으로 sigma_t 를 적응 추정하는 것이 물리적으로 맞다.
        self.lid_model = PolarNoiseModel(
            r_coef=float(self.declare_parameter('lid_sigma_r_coef', 0.331e-3).value),
            r_exp=float(self.declare_parameter('lid_sigma_r_exp', 0.354).value),
            t_coef=float(self.declare_parameter('lid_sigma_t_const', 6.5e-3).value),
            t_exp=0.0,
            r_floor=float(self.declare_parameter('lid_sigma_r_floor', 1.6e-3).value),
            r_break=float(self.declare_parameter('lid_sigma_r_break', 4.5).value))

        self.offset = RepresentativeOffset(
            r_coef=float(self.declare_parameter('delta_r_coef', -10.98e-3).value),
            r_exp=float(self.declare_parameter('delta_r_exp', 1.281).value),
            t_const=float(self.declare_parameter('delta_t_const', 13.0e-3).value))

        # delta 모델의 잔차. 라이다 공분산에 더한다. 이게 없으면 근거리에서
        # sigma_comb 가 0.5mm 수준이라 모델 오차 몇 mm 에도 chi^2 가 폭발한다.
        self.q_r = float(self.declare_parameter('delta_resid_r', 5.08e-3).value)
        self.q_t = float(self.declare_parameter('delta_resid_t', 6.95e-3).value)

        # Existing chi-square threshold; temporal uncertainty belongs in R.
        self.gate = float(self.declare_parameter('gate_chi2', 9.21).value)

        # ── 라이다 yaw 보정 (임시) ──
        # C2 에서 0도 조건만으로 두 센서 방위가 -38.61 +- 2.30 mrad (-2.212도)
        # 어긋나 있음을 확인했다. 거리 5.5배 범위에서 변동계수 6.0% 라 거리 무관한
        # 각도이고, 대표점 차이(mm 상수)도 파랄랙스(1/d)도 아니다.
        # theta 의 CV(12%)가 y 의 CV(43%)보다 작은 쪽이 라이다여서 라이다 yaw 로
        # 귀속했으나, 절대 방위 기준을 재지 않아 카메라 쪽일 가능성이 남아 있다.
        #
        # 올바른 해법은 URDF(laser_front 의 rpy_offset)를 고치는 것이다. 그래야
        # /scan 을 쓰는 SLAM 과 코스트맵도 같이 맞는다. URDF 를 고치면 이 값을
        # 반드시 0.0 으로 되돌릴 것 — 안 그러면 이중 보정이 된다.
        self.lidar_yaw = float(self.declare_parameter('lidar_yaw_correction', -0.043670).value)
        # laser_front 장착 위치(URDF). 회전 중심이 base_link 원점이 아니다.
        self.laser_xy = np.array([
            float(self.declare_parameter('laser_origin_x', 0.086291).value),
            float(self.declare_parameter('laser_origin_y', -0.020494).value)])

        self.pub = self.create_publisher(DetectedObjectArray, self.out_topic, 10)
        self.tracking_pub = self.create_publisher(
            DetectedObjectArray, self.tracking_topic, 10)
        self.lidar_obstacles_pub = self.create_publisher(
            DetectedObjectArray, self.lidar_obstacles_topic, 10)
        self.reference_frame = self.declare_parameter('reference_frame', 'base_link').value
        self.fixed_frame = self.declare_parameter('fixed_frame', 'odom').value
        if self.fixed_frame != 'odom' or self.reference_frame != 'base_link':
            raise ValueError('calibration requires base_link reference and continuous odom')
        self.max_association_delta = float(self.declare_parameter(
            'max_association_time_delta', 0.167).value)
        self.sigma_v = float(self.declare_parameter('object_velocity_sigma', 1.0).value)
        self.unknown_sigma = float(self.declare_parameter('unknown_timing_sigma', 0.15).value)
        self.rear_sigma = float(self.declare_parameter('uncalibrated_lidar_sigma', 0.05).value)
        self.calibrated_sensor = self.declare_parameter(
            'calibrated_sensor_id', '/scan_front').value
        if min(self.sigma_v, self.unknown_sigma, self.rear_sigma) <= 0:
            raise ValueError('uncertainty parameters must be positive')
        self.pending = FusionQueue(
            wait=float(self.declare_parameter('partner_wait', 0.10).value),
            tf_wait=float(self.declare_parameter('tf_wait_timeout', 0.25).value),
            capacity=self.queue, slop=self.slop,
            max_age=float(self.declare_parameter('max_observation_age', 0.75).value))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.counts = Counter()
        self.last_ids = []
        self.create_subscription(DetectedObjectArray, self.cam_topic,
                                 lambda msg: self._enqueue(msg, True), 10)
        self.create_subscription(DetectedObjectArray, self.lid_topic,
                                 lambda msg: self._enqueue(msg, False), 10)
        self.create_timer(0.01, self._drain)
        self.create_timer(5.0, self._diagnostics)

        self.add_on_set_parameters_callback(self._on_param_set)

        self.get_logger().info(
            f'camera_lidar_fusion_node 시작: {self.cam_topic} + {self.lid_topic} '
            f'-> semantic {self.out_topic}, tracking {self.tracking_topic}, '
            f'unmatched LiDAR '
            f'{self.lidar_obstacles_topic} (slop={self.slop*1000:.0f}ms, '
            f'gate={self.gate}, forward_unmatched={self.forward_unmatched_lidar})')
        if abs(self.lidar_yaw) > 1e-9:
            self.get_logger().warn(
                f'라이다 yaw 보정 {math.degrees(self.lidar_yaw):+.3f}도 적용 중. '
                'URDF(laser_front rpy_offset)를 고치면 이 파라미터를 0.0 으로 '
                '되돌릴 것 — 안 그러면 이중 보정이 된다.')

    def _on_param_set(self, params):
        for p in params:
            if p.name == 'gate_chi2':
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False,
                                               reason='gate_chi2 는 양수')
                self.gate = float(p.value)
            elif p.name == 'lidar_yaw_correction':
                self.lidar_yaw = float(p.value)
            elif p.name == 'forward_unmatched_lidar_to_fused':
                self.forward_unmatched_lidar = bool(p.value)
            else:
                continue
            self.get_logger().warn(f'파라미터 변경: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #

    def _enqueue(self, msg, camera):
        stream = 'camera' if camera else (
            msg.observations[0].sensor_id if msg.observations else 'lidar_unknown')
        self.pending.add(stream, msg, time.monotonic(), self.get_clock().now().nanoseconds)
        self._drain()

    def _drain(self):
        self.pending.drain(time.monotonic(), self.get_clock().now().nanoseconds,
                           self._consume)

    def _diagnostics(self):
        self.get_logger().info(
            f'Fusion counts={dict(self.counts)} queue={dict(self.pending.counts)} '
            f'last_ids={self.last_ids[:3]}')

    def _lidar_xy(self, obj):
        """C2 correction in native base_link axes, FRONT ONLY, before alignment."""
        p = np.array([obj.position.x, obj.position.y])
        if not any(o.sensor_id == self.calibrated_sensor for o in obj.observations):
            return p
        p = self.laser_xy + rot(self.lidar_yaw) @ (p - self.laser_xy)
        return np.array(self.offset.apply(*p))

    def _align(self, msg, reference, camera):
        out = copy.deepcopy(msg)
        if stamp_ns(msg.header.stamp) <= 0 or reference < stamp_ns(msg.header.stamp):
            raise ValueError('alignment requires a positive source epoch and causal reference')
        if msg.header.frame_id != self.reference_frame:
            raise ValueError('inputs must already use base_link axes')
        if any(o.header != msg.header for o in msg.objects):
            raise ValueError('object and array coordinate epochs disagree')
        tf = self.tf_buffer.lookup_transform_full(
            self.reference_frame, rclpy.time.Time(nanoseconds=reference),
            msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp),
            self.fixed_frame)
        rotation, translation = rigid(tf)
        out.header.stamp = rclpy.time.Time(nanoseconds=reference).to_msg()
        for obj in out.objects:
            p = np.array([obj.position.x, obj.position.y, obj.position.z])
            covariance = np.array(obj.position_covariance).reshape(2, 2)
            if not np.any(covariance):
                if camera:
                    covariance = self.cam_model.cov(*p[:2])
                elif any(o.sensor_id == self.calibrated_sensor for o in obj.observations):
                    covariance = self.lid_model.cov(*p[:2])
                else:
                    covariance = np.eye(2)*self.rear_sigma**2
            if not camera:
                # Propagate the nonlinear representative offset with its Jacobian.
                corrected = self._lidar_xy(obj)
                jacobian = np.empty((2, 2))
                for axis, attr in enumerate(('x', 'y')):
                    shifted = copy.deepcopy(obj)
                    setattr(shifted.position, attr, getattr(shifted.position, attr)+1e-5)
                    jacobian[:, axis] = (self._lidar_xy(shifted)-corrected)/1e-5
                covariance = jacobian @ covariance @ jacobian.T
                if any(o.sensor_id == self.calibrated_sensor for o in obj.observations):
                    covariance += self._gate_cov(*corrected)
                p[:2] = corrected
            p = rotation @ p + translation
            c = rotation[:2, :2]
            covariance = c @ covariance @ c.T
            covariance += np.eye(2)*temporal_variance(
                obj, reference, self.sigma_v, self.unknown_sigma)
            covariance = (covariance+covariance.T)*0.5
            if not np.isfinite(p).all() or not np.isfinite(covariance).all():
                raise ValueError('non-finite observation')
            if np.linalg.eigvalsh(covariance).min() < 0:
                raise ValueError('invalid covariance')
            obj.position.x, obj.position.y, obj.position.z = map(float, p)
            obj.position_covariance = covariance.ravel().tolist()
            obj.header = out.header
        return out

    def _consume(self, batch, reference):
        camera, lidar = DetectedObjectArray(), DetectedObjectArray()
        camera.header.frame_id = lidar.header.frame_id = self.reference_frame
        camera.header.stamp = lidar.header.stamp = rclpy.time.Time(nanoseconds=reference).to_msg()
        try:
            for entry in batch:
                is_camera = entry.stream == 'camera'
                aligned = self._align(entry.message, reference, is_camera)
                target = camera if is_camera else lidar
                target.objects.extend(aligned.objects)
                target.observations.extend(aligned.observations)
        except tf2_ros.TransformException:
            return False
        except ValueError:
            self.counts['invalid_contract'] += len(batch)
            return True
        self.counts['reference_ns'] = reference
        self.counts['age_ms'] = round((self.get_clock().now().nanoseconds-reference)*1e-6, 2)
        self.counts['processing_delay_ms'] = round(
            (time.monotonic()-min(e.received for e in batch))*1000, 2)
        cam_times = [stamp_ns(o.header.stamp) for o in camera.observations]
        lid_times = [stamp_ns(o.header.stamp) for o in lidar.observations]
        if cam_times and lid_times:
            self.counts['capture_delta_ms'] = round(
                max(abs(cam-lid) for cam in cam_times for lid in lid_times)*1e-6, 2)
        windows_c = [observation_window(o) for o in camera.objects]
        windows_l = [observation_window(o) for o in lidar.objects]
        if windows_c and windows_l and all(windows_c+windows_l):
            self.counts['beam_capture_delta_ms_max'] = round(1000*max(
                max(abs(cam[0]-lid[1]), abs(cam[1]-lid[0]))
                for cam in windows_c for lid in windows_l), 2)
        self.last_ids = [o.observation_id for o in camera.observations+lidar.observations]
        self._on_pair(camera, lidar)
        return True

    def _gate_cov(self, x, y):
        """C2 representative residual, included in both association and fusion R."""
        J = rot(math.atan2(y, x))
        return J @ np.diag([self.q_r ** 2, self.q_t ** 2]) @ J.T

    def _on_pair(self, cam_msg, lid_msg):
        if (not cam_msg.header.frame_id
                or cam_msg.header.frame_id != lid_msg.header.frame_id
                or cam_msg.header.stamp != lid_msg.header.stamp):
            self.get_logger().warning(
                'Fusion requires a common frame AND coordinate epoch: '
                f'camera={cam_msg.header.frame_id!r}, lidar={lid_msg.header.frame_id!r}',
                throttle_duration_sec=2.0)
            return
        cam = list(cam_msg.objects)
        lid = list(lid_msg.objects)

        cam_p = [np.array([o.position.x, o.position.y]) for o in cam]
        lid_p = [np.array([o.position.x, o.position.y]) for o in lid]
        cam_R = [np.array(o.position_covariance).reshape(2, 2)
                 if any(o.position_covariance) else self.cam_model.cov(*p)
                 for o, p in zip(cam, cam_p)]
        lid_R = [np.array(o.position_covariance).reshape(2, 2)
                 if any(o.position_covariance) else self.lid_model.cov(*p)
                 for o, p in zip(lid, lid_p)]

        chi2 = np.full((len(cam), len(lid)), np.inf)
        for i, (pc, Rc) in enumerate(zip(cam_p, cam_R)):
            for j, (pl, Rl) in enumerate(zip(lid_p, lid_R)):
                cw, lw = observation_window(cam[i]), observation_window(lid[j])
                if (cw and lw and max(abs(cw[0]-lw[1]), abs(cw[1]-lw[0]))
                        > self.max_association_delta):
                    self.counts['association_time_rejected'] += 1
                    continue
                d = pc - pl
                S = Rc + Rl
                try:
                    chi2[i, j] = float(d @ np.linalg.solve(S, d))
                except np.linalg.LinAlgError:
                    continue

        matched, _, _ = assign(chi2, self.gate)

        out = DetectedObjectArray()
        out.header.stamp = cam_msg.header.stamp
        out.header.frame_id = cam_msg.header.frame_id

        lidar_obstacles = DetectedObjectArray()
        lidar_obstacles.header = lid_msg.header

        out.observations = list(cam_msg.observations) + list(lid_msg.observations)
        lidar_obstacles.observations = list(lid_msg.observations)
        used_lid = set(matched.values())
        if hasattr(self, 'counts'):
            self.counts['matched'] += len(matched)
            self.counts['camera_only'] += len(cam)-len(matched)
            self.counts['lidar_only'] += len(lid)-len(used_lid)
        for i, o in enumerate(cam):
            if i in matched:
                j = matched[i]
                out.objects.append(self._fuse(o, lid[j], cam_p[i], lid_p[j],
                                              cam_R[i], lid_R[j]))
            else:
                # 카메라 단독 — 라이다 스캔면(지면 21.8cm) 위의 물체나 가림.
                out.objects.append(self._passthrough(o, cam_p[i], cam_R[i],
                                                     SOURCE_CAMERA))
        for j, o in enumerate(lid):
            if j not in used_lid:
                obstacle = self._passthrough(
                    o, lid_p[j], lid_R[j], SOURCE_LIDAR)
                lidar_obstacles.objects.append(obstacle)
                if self.forward_unmatched_lidar:
                    out.objects.append(obstacle)
        tracking = DetectedObjectArray()
        tracking.header = out.header
        tracking.observations = list(out.observations)
        tracking.objects = list(out.objects)
        if not self.forward_unmatched_lidar:
            tracking.objects.extend(lidar_obstacles.objects)
        self.pub.publish(out)
        self.tracking_pub.publish(tracking)
        self.lidar_obstacles_pub.publish(lidar_obstacles)

    def _fuse(self, cam_obj, lid_obj, pc, pl, Rc, Rl):
        """공분산 가중 평균. R 이 이방성이라 방향별로 가중치가 갈린다.

        C2 기준 4m 에서 카메라 가중치가 반경 0.14%, 방위 70.7% 다. 즉 "거리는
        라이다, 방위는 카메라" 가 if 문이 아니라 이 식에서 저절로 나온다.
        """
        Ic, Il = np.linalg.inv(Rc), np.linalg.inv(Rl)
        P = np.linalg.inv(Ic + Il)
        z = P @ (Ic @ pc + Il @ pl)

        o = DetectedObject()
        o.header = cam_obj.header
        o.class_id = cam_obj.class_id
        o.class_name = cam_obj.class_name
        o.confidence = cam_obj.confidence
        o.bbox = cam_obj.bbox
        o.position.x, o.position.y = float(z[0]), float(z[1])
        # z 는 스캔 평면 높이(라이다)가 아니라 카메라가 본 높이를 유지한다.
        o.position.z = cam_obj.position.z
        o.distance = float(math.hypot(z[0], z[1]))
        o.source = SOURCE_FUSED
        o.observations = list(cam_obj.observations) + list(lid_obj.observations)
        o.ego_motion_compensated = lid_obj.ego_motion_compensated
        o.position_covariance = [float(v) for v in P.flatten()]
        return o

    def _passthrough(self, src, p, R, source):
        o = DetectedObject()
        o.header = src.header
        o.class_id = src.class_id
        o.class_name = src.class_name
        o.confidence = src.confidence
        o.bbox = src.bbox
        o.position.x, o.position.y = float(p[0]), float(p[1])
        o.position.z = src.position.z
        o.distance = float(math.hypot(p[0], p[1]))
        o.source = source
        o.observations = list(src.observations)
        o.ego_motion_compensated = src.ego_motion_compensated
        o.position_covariance = [float(v) for v in R.flatten()]
        return o


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

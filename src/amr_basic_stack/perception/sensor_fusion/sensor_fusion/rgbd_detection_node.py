#!/usr/bin/env python3
"""rgbd_detection — RGB-D 검출 -> base_link 기준 객체 대표 위치.

파이프라인:
  color + aligned_depth 시간동기화(message_filters)
    -> YOLO(OpenVINO) 2D 검출            [YoloOpenVINODetector]
    -> bbox 중앙 ROI 의 유효 depth 픽셀 전부를 camera_info(K)로 3D 역투영
       -> TF로 target_frame(base_link) 변환 -> 좌표별 median
       = object representative position (visible-point representative position)
       (camera_position_method=center_patch 이면 기존 bbox 중심 1점 방식)
    -> DetectedObjectArray(/detections) 발행

라이다와의 관계:
  이 노드는 라이다를 읽지 않는다. /detections 는 순수 카메라 관측이다.
  라이다 정보는 lidar_cluster_node 의 /lidar_clusters 로 따로 나가며, 두 센서를
  합치는 일은 검출-검출 data association(게이팅 + 배정) 단계의 몫이다.
  이전의 거리별 선형 가중 융합(near/far blend)은 근거가 될 σ_lid(d) 가 없는
  임의 휴리스틱이라 제거했다.

전제:
  realsense align_depth.enable:=true (depth가 color에 픽셀정렬).
  robot_state_publisher가 base_link<-camera optical TF 발행.

대표점의 의미(중요):
  카메라도 라이다도 물체의 **보이는 표면점**만 관측한다. 여기서 만드는 위치는
  물체의 기하학적 중심이 아니라 object representative position 이다.
  (= visible-point representative position). 라이다 클러스터도 마찬가지이므로
  두 센서의 bias 에는 "서로 다른 표면을 본다"는 성분이 남는다 — 센서 잡음이 아니다.

C단계(σ 실측)와의 관계:
  measure_c.py 가 재는 σ_C(d) 는 D455 depth 소자의 intrinsic noise 가 아니라
  RGB-D -> YOLO detection -> depth ROI -> 3D representative position -> TF
  전체 파이프라인의 변동, 즉 object-level measurement uncertainty 다.
"""
import copy
import math
from pathlib import Path
import time

from amr_interfaces.msg import (
    DetectedObject, DetectedObjectArray, ObservationProvenance,
    SemanticObservation, SemanticObservationArray,
)
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PointStamped
import message_filters
import numpy as np
from object_tracker.core.pending import PendingMeasurements
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_fusion.semantic_geometry import decode_mask, depth_surface, surface_statistics
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
import tf2_ros

from .observation_time import stamp_ns


SOURCE_CAMERA_ONLY = 0   # DetectedObject.source: 0=CAMERA — 이 노드는 카메라 위치만 낸다


class YoloOpenVINODetector:
    """YOLO 검출기 - OpenVINO IR(.xml/.bin) 직접 추론.

    NUC(Intel Core 5, NVIDIA 없음)에서 실행하려면 모델을 OpenVINO로 export:
      python3 scripts/export_yolo_openvino.py   (-> yolo11n_openvino_model/)
      그리고 파라미터 yolo_model_path 를 그 폴더로 지정.

    OpenVINO 런타임이나 모델 폴더가 없으면 안전하게 STUB(빈 검출)로 동작.
    detect(bgr) -> [{class_id, class_name, confidence, cx, cy, w, h}] (픽셀).
    """

    def __init__(self, node: Node, model_path, conf, imgsz, class_filter,
                 device='AUTO', iou=0.45):
        self.node = node
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.iou = float(iou)
        self.class_filter = set(int(c) for c in class_filter) if class_filter else None
        self.compiled = None
        self.input_layer = None
        self.output_layer = None
        self.input_h = self.imgsz
        self.input_w = self.imgsz
        self.names = {}
        self.mask_classes = {56, 60}
        self.max_masks = 8

        try:
            import openvino as ov
        except ImportError:
            node.get_logger().warn(
                'openvino 미설치 -> YOLO STUB(빈 검출). '
                'NUC에서 OpenVINO 런타임 설치 후 사용.')
            return

        xml_path = self._resolve_model_xml(model_path)
        if xml_path is None:
            node.get_logger().warn(
                f'YOLO OpenVINO 모델 없음({model_path}) -> STUB. '
                'scripts/export_yolo_openvino.py 로 생성하세요.')
            return

        try:
            core = ov.Core()
            model = core.read_model(str(xml_path))
            shape = list(model.inputs[0].shape)
            self._set_input_size_from_shape(shape)
            self.compiled = core.compile_model(model, device)
            self.input_layer = self.compiled.input(0)
            self.output_layer = self.compiled.output(0)
            self.names = self._load_names(xml_path)
            node.get_logger().info(
                f'YOLO(OpenVINO) 로드 완료: {xml_path} '
                f'({device}, input={self.input_w}x{self.input_h})')
        except Exception as e:
            if device != 'CPU':
                self._compile_cpu_fallback(ov, xml_path, e)
            else:
                node.get_logger().error(f'YOLO OpenVINO 로드 실패 -> STUB: {e}')
                self.compiled = None

    def _compile_cpu_fallback(self, ov, xml_path, first_error):
        try:
            core = ov.Core()
            model = core.read_model(str(xml_path))
            shape = list(model.inputs[0].shape)
            self._set_input_size_from_shape(shape)
            self.compiled = core.compile_model(model, 'CPU')
            self.input_layer = self.compiled.input(0)
            self.output_layer = self.compiled.output(0)
            self.names = self._load_names(xml_path)
            self.node.get_logger().warn(
                f'OpenVINO {first_error} -> CPU로 재시도 성공: {xml_path}')
        except Exception as e:
            self.node.get_logger().error(
                f'YOLO OpenVINO 로드 실패 -> STUB: {first_error}; CPU fallback도 실패: {e}')
            self.compiled = None

    def _set_input_size_from_shape(self, shape):
        if len(shape) != 4:
            return
        h_dim, w_dim = shape[2], shape[3]
        if not getattr(h_dim, 'is_static', True) or not getattr(w_dim, 'is_static', True):
            return
        try:
            self.input_h = int(h_dim)
            self.input_w = int(w_dim)
        except TypeError:
            pass

    @staticmethod
    def _resolve_model_xml(model_path):
        if not model_path:
            return None
        path = Path(str(model_path)).expanduser()
        if path.is_file() and path.suffix == '.xml':
            return path
        if path.is_dir():
            preferred = path / 'yolo11n.xml'
            if preferred.exists():
                return preferred
            xmls = sorted(path.glob('*.xml'))
            if xmls:
                return xmls[0]
        return None

    def _load_names(self, xml_path):
        metadata = xml_path.with_name('metadata.yaml')
        if not metadata.exists():
            return {}
        try:
            import yaml
            data = yaml.safe_load(metadata.read_text()) or {}
            names = data.get('names', {}) or {}
            items = names.items() if isinstance(names, dict) else enumerate(names)
            return {int(k): str(v) for k, v in items}
        except Exception as e:
            self.node.get_logger().warn(f'YOLO metadata 읽기 실패({metadata}): {e}')
            return {}

    def detect(self, bgr):
        if self.compiled is None:
            return []

        blob, scale, pad_x, pad_y = self._preprocess(bgr)
        outputs = self.compiled([blob])
        raw = next((np.asarray(value) for value in outputs.values()
                    if np.asarray(value).ndim == 3), outputs[self.output_layer])
        proto = next((np.asarray(value) for value in outputs.values()
                      if np.asarray(value).ndim == 4), None)
        mask_dim = int(proto.shape[1]) if proto is not None else 0
        feature_count = 4 + len(self.names) + mask_dim if self.names else None
        pred = np.asarray(raw)
        if pred.ndim == 3:
            pred = pred[0]
        if feature_count and pred.ndim == 2 and pred.shape[0] == feature_count:
            pred = pred.T
        else:
            pred = self._normalize_output(raw)
        if pred is None or pred.shape[1] < 5:
            return []
        if feature_count and pred.shape[1] != feature_count:
            raise ValueError('Unsupported YOLO export; export raw outputs with nms=False')

        boxes = pred[:, :4]
        scores = pred[:, 4:pred.shape[1] - mask_dim]
        if scores.shape[1] < 1:
            raise ValueError('Invalid YOLO class/mask output dimensions')
        coefficients = pred[:, -mask_dim:] if mask_dim else None
        class_ids = np.argmax(scores, axis=1)
        confs = scores[np.arange(scores.shape[0]), class_ids]

        keep = (confs >= self.conf) & np.isfinite(confs) & np.isfinite(boxes).all(axis=1)
        if self.class_filter is not None:
            keep &= np.isin(class_ids, list(self.class_filter))
        if not np.any(keep):
            return []

        boxes = boxes[keep]
        confs = confs[keep]
        class_ids = class_ids[keep]
        if coefficients is not None:
            coefficients = coefficients[keep]
        boxes_xyxy = self._xywh_to_xyxy_original(boxes, scale, pad_x, pad_y, bgr.shape)
        keep_idx = self._classwise_nms(boxes_xyxy, confs, class_ids)

        dets = []
        masks_decoded = 0
        for i in keep_idx:
            x1, y1, x2, y2 = boxes_xyxy[i]
            w = x2 - x1
            h = y2 - y1
            cls = int(class_ids[i])
            dets.append({
                'class_id': cls,
                'class_name': str(self.names.get(cls, cls)),
                'confidence': float(confs[i]),
                'cx': float(x1 + w / 2.0),
                'cy': float(y1 + h / 2.0),
                'w': float(w),
                'h': float(h),
            })
            if proto is not None and cls in self.mask_classes and masks_decoded < self.max_masks:
                content_w = int(round(bgr.shape[1] * scale))
                content_h = int(round(bgr.shape[0] * scale))
                dets[-1]['mask'] = decode_mask(
                    coefficients[i], proto, (self.input_h, self.input_w),
                    (pad_x, pad_y, content_w, content_h), bgr.shape, boxes_xyxy[i])
                masks_decoded += 1
        return dets

    def _preprocess(self, bgr):
        h, w = bgr.shape[:2]
        scale = min(self.input_w / w, self.input_h / h)
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))
        resized = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        canvas = np.full((self.input_h, self.input_w, 3), 114, dtype=np.uint8)
        pad_x = (self.input_w - new_w) // 2
        pad_y = (self.input_h - new_h) // 2
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        blob = rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        return blob, scale, pad_x, pad_y

    @staticmethod
    def _normalize_output(raw):
        pred = np.asarray(raw)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim != 2:
            return None
        # YOLOv8/11 OpenVINO export without NMS is usually [84, 8400].
        if pred.shape[0] < pred.shape[1] and pred.shape[0] <= 256:
            pred = pred.T
        return pred

    @staticmethod
    def _xywh_to_xyxy_original(boxes, scale, pad_x, pad_y, image_shape):
        h, w = image_shape[:2]
        x, y, bw, bh = boxes.T
        x1 = (x - bw / 2.0 - pad_x) / scale
        y1 = (y - bh / 2.0 - pad_y) / scale
        x2 = (x + bw / 2.0 - pad_x) / scale
        y2 = (y + bh / 2.0 - pad_y) / scale
        out = np.stack((x1, y1, x2, y2), axis=1)
        out[:, [0, 2]] = np.clip(out[:, [0, 2]], 0, w - 1)
        out[:, [1, 3]] = np.clip(out[:, [1, 3]], 0, h - 1)
        return out

    def _classwise_nms(self, boxes, scores, class_ids):
        keep = []
        for cls in np.unique(class_ids):
            idxs = np.where(class_ids == cls)[0]
            keep.extend(idxs[self._nms(boxes[idxs], scores[idxs])].tolist())
        keep.sort(key=lambda i: float(scores[i]), reverse=True)
        return keep

    def _nms(self, boxes, scores):
        if boxes.size == 0:
            return np.empty((0,), dtype=np.int64)
        x1, y1, x2, y2 = boxes.T
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            if order.size == 1:
                break
            rest = order[1:]
            xx1 = np.maximum(x1[i], x1[rest])
            yy1 = np.maximum(y1[i], y1[rest])
            xx2 = np.minimum(x2[i], x2[rest])
            yy2 = np.minimum(y2[i], y2[rest])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[i] + areas[rest] - inter
            iou = np.divide(inter, union, out=np.zeros_like(inter), where=union > 0)
            order = rest[iou <= self.iou]
        return np.asarray(keep, dtype=np.int64)

class RgbdDetectionNode(Node):
    def __init__(self):
        super().__init__('rgbd_detection_node')

        self.color_topic = self.declare_parameter(
            'color_topic', '/camera/color/image_raw').value
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/camera/aligned_depth_to_color/image_raw').value
        self.info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/color/camera_info').value
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/detections').value
        self.depth_scale = self.declare_parameter('depth_scale', 0.001).value  # mm->m
        self.sync_slop = self.declare_parameter('sync_slop', 0.05).value
        self.sample_win = int(self.declare_parameter('depth_sample_window', 5).value)
        # 객체 대표 위치 산출 방식.
        #   roi_median_3d : bbox 중앙 ROI 의 유효 depth 픽셀을 전부 역투영 -> base_link
        #                   변환 후 좌표별 median (라이다 클러스터 대표점과 의미가 맞음)
        #   center_patch  : 기존 방식 — bbox 중심 (2*win+1)^2 창의 depth median 1점
        # 비교 실험을 위해 옛 방식을 남겨둔다. 기본은 roi_median_3d.
        self.cam_pos_method = str(self.declare_parameter(
            'camera_position_method', 'roi_median_3d').value)
        if self.cam_pos_method not in ('roi_median_3d', 'center_patch'):
            self.get_logger().warn(
                f"camera_position_method='{self.cam_pos_method}' 알 수 없음 "
                '-> roi_median_3d 로 대체')
            self.cam_pos_method = 'roi_median_3d'
        # bbox 전체를 쓰면 테두리에서 배경 depth 가 섞인다. 중앙 일부만 사용.
        # 0.5 -> 가로/세로 절반 (면적 기준 1/4)
        self.depth_roi_ratio = float(self.declare_parameter(
            'depth_roi_ratio', 0.5).value)
        # ROI 안 유효 depth 픽셀이 이보다 적으면 대표 위치가 불안정 -> 측정 자체를 버린다.
        # (거리를 틀리게 내보내는 것보다 안 내보내는 편이 낫다)
        self.min_valid_px = int(self.declare_parameter(
            'min_valid_depth_pixels', 20).value)
        # 검증용: YOLO 없이 영상 중앙에 가짜 박스 1개 주입(파이프라인 테스트)
        self.debug_center_box = bool(
            self.declare_parameter('debug_center_box', False).value)
        # YOLO(OpenVINO)
        yolo_model = self.declare_parameter('yolo_model_path', 'yolo11n_openvino_model').value
        yolo_conf = self.declare_parameter('yolo_conf', 0.4).value
        yolo_imgsz = int(self.declare_parameter('yolo_imgsz', 640).value)
        yolo_iou = float(self.declare_parameter('yolo_iou', 0.45).value)
        openvino_device = self.declare_parameter('openvino_device', 'AUTO').value
        yolo_classes = list(self.declare_parameter('yolo_classes', []).value)

        self.bridge = CvBridge()
        self.detector = YoloOpenVINODetector(
            self, yolo_model, yolo_conf, yolo_imgsz, yolo_classes,
            openvino_device, yolo_iou)
        self.K = None  # 카메라 내부행렬 (3x3)

        self.pending = PendingMeasurements(
            max_wait=float(self.declare_parameter('tf_wait_timeout', 0.25).value),
            capacity=int(self.declare_parameter('pending_capacity', 8).value))
        self.max_age = float(self.declare_parameter('max_observation_age', 0.75).value)
        self.rgbd_max_delta = float(self.declare_parameter('rgbd_max_delta', 0.025).value)
        self.timing_mode = self.declare_parameter('timing_mode', 'unknown').value
        self.unknown_policy = self.declare_parameter('unknown_timing_policy', 'snapshot').value
        if self.timing_mode not in ('unknown', 'capture'):
            raise ValueError('timing_mode must be unknown or capture')
        if self.unknown_policy not in ('snapshot', 'drop'):
            raise ValueError('unknown_timing_policy must be snapshot or drop')
        if self.timing_mode == 'unknown':
            self.get_logger().warning(
                'RGB-D exposure timing is UNVERIFIED; shared image stamps may be '
                'frameset stamps. Applying unknown_timing_policy=' + self.unknown_policy)
        self.create_timer(0.01, self._drain_pairs)
        self.create_timer(5.0, lambda: self.get_logger().info(
            f'RGB-D queue drops={dict(self.pending.dropped)}'))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)

        color_sub = message_filters.Subscriber(
            self, Image, self.color_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=self.sync_slop)
        self.sync.registerCallback(self._on_pair)

        self.pub = self.create_publisher(DetectedObjectArray, self.detections_topic, 10)
        self.semantic_enabled = bool(self.declare_parameter('publish_semantic', False).value)
        self.semantic_classes = set(self.declare_parameter('semantic_classes', [56, 60]).value)
        self.semantic_max_points = int(self.declare_parameter('semantic_max_points', 512).value)
        self.semantic_max_objects = int(self.declare_parameter('semantic_max_objects', 8).value)
        self.detector.mask_classes = self.semantic_classes if self.semantic_enabled else set()
        self.detector.max_masks = self.semantic_max_objects
        self.inference_hz = float(self.declare_parameter('inference_hz', 0.0).value)
        self.last_inference = -float('inf')
        self.semantic_pub = self.create_publisher(
            SemanticObservationArray, '/semantic/observations', qos_profile_sensor_data)
        self.debug_image_pub = self.create_publisher(
            Image, '/semantic/debug_image', qos_profile_sensor_data)
        self.semantic_debug = bool(self.declare_parameter('semantic_debug_image', False).value)
        self.info_shape = None
        self.info_frame = None
        # 실험 정의 파라미터는 __init__ 에서 self 에 캐시된다. 콜백이 없으면
        # `ros2 param set` 이 성공했다고 응답하는데 노드 동작은 안 바뀌고,
        # `ros2 param get` 은 바뀐 값을 돌려준다 -> CSV 의 note 컬럼이 거짓말을 한다.
        # 측정 캠페인에서 이건 조용한 데이터 오염이므로 콜백으로 실제 반영한다.
        self.add_on_set_parameters_callback(self._on_param_set)

        self.get_logger().info(
            f'rgbd_detection_node 시작: color={self.color_topic}, '
            f'depth={self.depth_topic} -> {self.detections_topic} '
            f'(position={self.cam_pos_method}, roi_ratio={self.depth_roi_ratio}, '
            f'min_valid_px={self.min_valid_px})')

    def _on_param_set(self, params):
        """실험 정의 파라미터의 런타임 변경을 검증 후 실제로 반영."""
        for p in params:
            if p.name == 'publish_semantic':
                self.semantic_enabled = bool(p.value)
                self.detector.mask_classes = (
                    self.semantic_classes if self.semantic_enabled else set())
            elif p.name == 'semantic_debug_image':
                self.semantic_debug = bool(p.value)
            elif p.name == 'inference_hz':
                value = float(p.value)
                if not math.isfinite(value) or value < 0:
                    return SetParametersResult(
                        successful=False, reason='inference_hz must be >= 0')
                self.inference_hz = value
            elif p.name in ('semantic_classes', 'semantic_max_points', 'semantic_max_objects'):
                return SetParametersResult(
                    successful=False, reason=f'{p.name} requires node restart')
            elif p.name == 'camera_position_method':
                if p.value not in ('roi_median_3d', 'center_patch'):
                    return SetParametersResult(
                        successful=False,
                        reason=f'camera_position_method: {p.value} 는 허용값이 아님 '
                               '(roi_median_3d | center_patch)')
                self.cam_pos_method = str(p.value)
            elif p.name == 'depth_roi_ratio':
                if not 0.0 < float(p.value) <= 1.0:
                    return SetParametersResult(
                        successful=False, reason='depth_roi_ratio 는 (0, 1] 범위')
                self.depth_roi_ratio = float(p.value)
            elif p.name == 'min_valid_depth_pixels':
                if int(p.value) < 1:
                    return SetParametersResult(
                        successful=False, reason='min_valid_depth_pixels 는 1 이상')
                self.min_valid_px = int(p.value)
            else:
                continue
            # 측정 중에 바뀌면 그 run 의 표본이 두 설정에 걸쳐 섞인다. 흔적을 남긴다.
            self.get_logger().warn(
                f'실험 정의 파라미터 변경: {p.name} = {p.value}. '
                '측정 진행 중이었다면 그 run 은 폐기할 것.')
        return SetParametersResult(successful=True)

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.info_shape = (msg.height, msg.width)
        self.info_frame = msg.header.frame_id

    def _on_pair(self, color_msg: Image, depth_msg: Image):
        stamp = stamp_ns(depth_msg.header.stamp)
        color_stamp = stamp_ns(color_msg.header.stamp)
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        delta = abs(stamp-color_stamp)*1e-9
        if (min(stamp, color_stamp) <= 0 or age > self.max_age or age < -0.05
                or delta > self.rgbd_max_delta):
            self.pending.dropped['age_or_rgbd_skew'] += 1
            return
        # Approximate synchronization and spatial depth alignment do not move
        # a color ROI to another capture epoch. No temporal reprojection here.
        if (stamp != color_stamp or not depth_msg.header.frame_id
                or color_msg.header.frame_id != depth_msg.header.frame_id):
            self.pending.dropped['rgbd_unaligned'] += 1
            return
        if self.timing_mode == 'unknown' and self.unknown_policy == 'drop':
            self.pending.dropped['timing_unknown_drop'] += 1
            return
        self.pending.add(stamp, (color_msg, depth_msg), time.monotonic())
        self._drain_pairs()

    def _drain_pairs(self):
        self.pending.drain(time.monotonic(),
                           lambda pair: self._lookup_tf(pair[1].header),
                           lambda pair, _tf: self._process_pair(*pair))

    def _process_pair(self, color_msg: Image, depth_msg: Image):
        if ((self.get_clock().now().nanoseconds-stamp_ns(depth_msg.header.stamp))*1e-9
                > self.max_age):
            self.pending.dropped['stale'] += 1
            return
        if self.K is None:
            self.get_logger().warn('camera_info 대기 중...', throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if self.inference_hz > 0 and now - self.last_inference < 1.0 / self.inference_hz:
            return
        self.last_inference = now

        bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg)  # 16UC1(mm) 등 원본 인코딩
        if (depth.shape != bgr.shape[:2] or self.info_shape != depth.shape
                or self.info_frame != depth_msg.header.frame_id
                or depth_msg.encoding not in ('16UC1', '32FC1')
                or not np.isfinite(self.K).all() or self.K[0, 0] <= 0 or self.K[1, 1] <= 0):
            self.get_logger().warn('Invalid aligned depth/calibration/encoding',
                                   throttle_duration_sec=5.0)
            return
        # Existing representative-position helpers expect depth_scale units.
        legacy_depth = depth if depth_msg.encoding == '16UC1' else depth / self.depth_scale

        dets = self.detector.detect(bgr)
        if self.debug_center_box and not dets:
            h, w = bgr.shape[:2]
            dets = [{'class_id': -1, 'class_name': 'debug', 'confidence': 1.0,
                     'cx': w / 2.0, 'cy': h / 2.0, 'w': 40.0, 'h': 40.0}]

        out = DetectedObjectArray()
        # 위치는 target_frame(base_link) 기준이므로 헤더도 그 프레임으로.
        out.header.stamp = depth_msg.header.stamp
        out.header.frame_id = self.target_frame

        records = []
        for topic, header in ((self.depth_topic, depth_msg.header),
                              (self.color_topic, color_msg.header)):
            record = ObservationProvenance()
            record.header, record.sensor_id = header, topic
            record.observation_id = f'{topic}|{header.frame_id}|{stamp_ns(header.stamp)}'
            # RealSense framesets can publish a shared stamp for distinct
            # exposures; equality alone does not verify capture timing.
            record.timing_status = 1 if self.timing_mode == 'capture' else 0
            records.append(record)
        out.observations = records
        for index, d in enumerate(dets):
            obj = self._build_object(d, legacy_depth, depth_msg.header)
            if obj is not None:
                obj.observations = copy.deepcopy(records)
                for obs in obj.observations:
                    obs.observation_id += f'|{index}'
                out.objects.append(obj)

        self.pub.publish(out)
        if self.semantic_enabled:
            self._publish_semantic(dets, depth, depth_msg, bgr)

    def _publish_semantic(self, dets, depth, depth_msg, bgr):
        """Fan out the same inference using capture-time optical geometry."""
        out = SemanticObservationArray()
        out.header = depth_msg.header
        debug = bgr.copy() if self.semantic_debug else None
        for detection_index, detection in enumerate(dets):
            if detection['class_id'] not in self.semantic_classes:
                continue
            if len(out.observations) >= self.semantic_max_objects:
                break
            surface = depth_surface(
                detection, depth, depth_msg.encoding, self.K,
                max_points=self.semantic_max_points, min_pixels=self.min_valid_px,
                roi_ratio=self.depth_roi_ratio, integer_scale=self.depth_scale)
            if surface is None:
                continue
            points, ratio, method = surface
            position, center, extent, cov = surface_statistics(points)
            obs = SemanticObservation()
            obs.header = depth_msg.header
            obs.detection_index = detection_index
            obs.class_id = int(detection['class_id'])
            obs.class_name = str(detection['class_name'])
            obs.confidence = float(detection['confidence'])
            for message, values in ((obs.position, position), (obs.box_center, center),
                                    (obs.extent, extent)):
                message.x, message.y, message.z = map(float, values)
            obs.points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
            obs.covariance = cov.ravel().tolist()
            obs.valid_depth_ratio = ratio
            obs.support_method = method
            out.observations.append(obs)
            if debug is not None:
                if method == 0:
                    mask = detection['mask']
                    debug[mask] = (debug[mask] * 0.5 + np.array([0, 100, 0])).astype(np.uint8)
                x, y = int(detection['cx']), int(detection['cy'])
                cv2.putText(debug, f"{obs.class_name} {'mask' if method == 0 else 'ROI'}",
                            (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.semantic_pub.publish(out)
        if debug is not None:
            image = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            image.header = depth_msg.header
            self.debug_image_pub.publish(image)

    # ------------------------------------------------------------------ #
    # 객체 대표 위치 (object representative position)
    #
    # 카메라는 물체의 보이는 표면만 관측하므로 여기서 나오는 위치는 물체의
    # 기하학적 중심이 아니다. visible-point representative position 이다.
    # 라이다 클러스터 대표점도 같은 성질이라, 두 센서를 같은 의미의 양으로
    # 맞추기 위해 "여러 표면점 -> 대표값" 구조를 카메라에도 적용한다.
    # ------------------------------------------------------------------ #

    def _camera_position(self, d, depth, header):
        """검출 하나 -> target_frame 기준 (x, y, z). 못 만들면 None."""
        if self.cam_pos_method == 'center_patch':
            return self._pos_center_patch(d, depth, header)
        return self._pos_roi_median_3d(d, depth, header)

    def _pos_center_patch(self, d, depth, header):
        """기존 방식 — bbox 중심 창의 depth median 1점을 역투영."""
        z = self._sample_depth(depth, d['cx'], d['cy'])
        if z is None:
            return None  # 유효 depth 없음(0.6m 이내 등)
        x, y, zc = self._deproject(d['cx'], d['cy'], z)
        pt = self._transform_point(x, y, zc, header)
        if pt is None:
            return None
        return pt

    def _pos_roi_median_3d(self, d, depth, header):
        """bbox 중앙 ROI 의 유효 depth 픽셀을 전부 3D 로 올린 뒤 좌표별 median.

        median 을 쓰는 이유: ROI 에 배경(뒷벽)이 조금 섞여도 대표값이 끌려가지
        않는다. 평균은 배경 픽셀 하나가 수 m 를 끌어올린다.
        base_link 로 변환한 **뒤** median 을 잡는다(요구 순서). 좌표별 median 은
        회전에 대해 불변이 아니므로 변환 전/후 결과가 미세하게 다르다.
        """
        h, w = depth.shape[:2]
        u0, v0, u1, v1 = self._roi_bounds(d, w, h)
        if u1 <= u0 or v1 <= v0:
            return None

        roi = depth[v0:v1, u0:u1]
        n_roi = int(roi.size)
        mask = np.isfinite(roi) & (roi > 0)
        n_valid = int(np.count_nonzero(mask))
        # 참고 지표 — DetectedObject.msg 를 바꾸지 않기 위해 로그로만 남긴다.
        ratio = (n_valid / n_roi) if n_roi else 0.0
        if n_valid < self.min_valid_px:
            self.get_logger().debug(
                f"{d.get('class_name', '?')}: ROI 유효 depth {n_valid}/{n_roi} "
                f'(ratio={ratio:.2f}) < {self.min_valid_px} -> 측정 버림')
            return None
        self.get_logger().debug(
            f"{d.get('class_name', '?')}: valid_depth_ratio={ratio:.2f} "
            f'({n_valid}/{n_roi})')

        vs, us = np.nonzero(mask)
        zs = roi[mask].astype(np.float64) * self.depth_scale
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx0, cy0 = self.K[0, 2], self.K[1, 2]
        xs = (us + u0 - cx0) * zs / fx
        ys = (vs + v0 - cy0) * zs / fy
        pts = np.stack((xs, ys, zs), axis=1)          # camera optical frame

        pts_t = self._transform_points(pts, header)   # -> target_frame
        if pts_t is None:
            return None
        m = np.median(pts_t, axis=0)
        return float(m[0]), float(m[1]), float(m[2])

    def _roi_bounds(self, d, w, h):
        """bbox 중앙 ROI 의 픽셀 범위 [u0,u1) x [v0,v1). 이미지 밖은 잘라낸다."""
        hw = max(0.5, 0.5 * float(d['w']) * self.depth_roi_ratio)
        hh = max(0.5, 0.5 * float(d['h']) * self.depth_roi_ratio)
        u0 = int(max(0, math.floor(d['cx'] - hw)))
        u1 = int(min(w, math.floor(d['cx'] + hw) + 1))
        v0 = int(max(0, math.floor(d['cy'] - hh)))
        v1 = int(min(h, math.floor(d['cy'] + hh) + 1))
        return u0, v0, u1, v1

    def _sample_depth(self, depth, cx, cy):
        """bbox 중심 주변 창의 유효 depth 중앙값(m). 유효값 없으면 None."""
        h, w = depth.shape[:2]
        u, v = int(round(cx)), int(round(cy))
        if not (0 <= u < w and 0 <= v < h):
            return None
        r = self.sample_win
        patch = depth[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1].astype(np.float64)
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            return None
        return float(np.median(patch)) * self.depth_scale

    def _deproject(self, u, v, z):
        """픽셀(u,v)+깊이 z -> optical frame 3D (x우, y하, z전방)."""
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx0, cy0 = self.K[0, 2], self.K[1, 2]
        x = (u - cx0) * z / fx
        y = (v - cy0) * z / fy
        return x, y, z

    def _lookup_tf(self, header):
        if not header.frame_id or stamp_ns(header.stamp) <= 0:
            return None
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, header.frame_id, rclpy.time.Time.from_msg(header.stamp))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn(
                f'TF {self.target_frame}<-{header.frame_id} 없음',
                throttle_duration_sec=5.0)
            return None

    def _transform_point(self, x, y, z, header):
        """점 1개 변환(기존 center_patch 경로 — 동작을 그대로 유지)."""
        tf = self._lookup_tf(header)
        if tf is None:
            return None
        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        p = do_transform_point(pt, tf).point
        return p.x, p.y, p.z

    def _transform_points(self, pts, header):
        """(N,3) 점군을 한 번에 변환. do_transform_point 를 N 번 부르면 느리다."""
        tf = self._lookup_tf(header)
        if tf is None:
            return None
        R = self._quat_to_R(tf.transform.rotation)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z], dtype=np.float64)
        return pts @ R.T + t

    @staticmethod
    def _quat_to_R(q):
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-12:
            return np.eye(3)
        x, y, z, w = x / n, y / n, z / n, w / n
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)

    def _build_object(self, d, depth, header):
        pos = self._camera_position(d, depth, header)
        if pos is None:
            return None

        obj = DetectedObject()
        obj.header.stamp = header.stamp          # 카메라 stamp 유지
        obj.header.frame_id = self.target_frame  # 위치는 base_link 기준
        obj.class_id = int(d['class_id'])
        obj.class_name = str(d.get('class_name', ''))
        obj.confidence = float(d.get('confidence', 0.0))
        obj.bbox.center.position.x = float(d['cx'])
        obj.bbox.center.position.y = float(d['cy'])
        obj.bbox.size_x = float(d['w'])
        obj.bbox.size_y = float(d['h'])

        px, py, pz = pos
        obj.position.x, obj.position.y, obj.position.z = px, py, pz
        obj.distance = float(math.hypot(px, py))  # base_link 기준 수평거리(m)
        obj.source = SOURCE_CAMERA_ONLY
        return obj


def main(args=None):
    rclpy.init(args=args)
    node = RgbdDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

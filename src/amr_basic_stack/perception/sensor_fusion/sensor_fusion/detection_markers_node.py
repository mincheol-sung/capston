#!/usr/bin/env python3
"""detection_markers — 검출 결과를 RViz 마커로 시각화.

DetectedObjectArray 를 구독해 visualization_msgs/MarkerArray
(/detection_markers)로 발행한다.
RViz에서 Add -> MarkerArray -> Topic=/detection_markers 로 3D 검출 결과를 본다.

입력은 semantic-only /detections_fused 다(object_tracker 와 같은 토픽):
  마커는 "트래커가 왜 저기에 트랙을 만들었는가"를 눈으로 확인하는 도구다.
  따라서 트래커 입력과 같은 것을 봐야 한다. 카메라 raw 인 /detections 를 보면
  라이다 단독 관측(source=1)이 화면에서 통째로 사라진다 — 카메라 미검출 거리
  (C2: 6.05m 92.8% -> 7.10m 0%), ROI 유효 depth 부족으로 버려진 검출,
  미학습 클래스, 게이트 기각된 클러스터가 전부 여기 해당한다. 트래커는 그것들을
  정상 추적 중인데 근거가 화면에 없으니 유령 트랙으로 오진하게 된다.
  카메라 단독 관측만 보려면 detections_topic 을 /detections 로 바꾼다.

색상(DetectedObject.source 별):
  0 CAMERA 파랑   카메라 단독 — 라이다 스캔면(지면 0.178m) 위 물체나 가림
  1 LIDAR  초록   라이다 단독 — 카메라 미검출 거리/미학습 클래스. class_name 빈칸
  2 FUSED  주황   두 관측 융합 — 공분산 가중 평균
각 물체 = 구(위치) + 텍스트(클래스/거리).
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from amr_interfaces.msg import DetectedObjectArray

COLORS = {
    0: (0.20, 0.55, 0.95),   # CAMERA - 파랑
    1: (0.30, 0.80, 0.35),   # LIDAR - 초록
    2: (0.95, 0.60, 0.15),   # FUSED - 주황
}


class DetectionMarkers(Node):
    def __init__(self):
        super().__init__('detection_markers')
        self.sphere_size = self.declare_parameter('sphere_size', 0.3).value
        self.lifetime = self.declare_parameter('marker_lifetime', 0.5).value
        self.in_topic = self.declare_parameter(
            'detections_topic', '/detections_fused').value
        self.sub = self.create_subscription(
            DetectedObjectArray, self.in_topic, self._on_det, 10)
        self.pub = self.create_publisher(MarkerArray, '/detection_markers', 10)
        self.get_logger().info(
            f'detection_markers 시작: {self.in_topic} -> /detection_markers')

    def _on_det(self, msg: DetectedObjectArray):
        arr = MarkerArray()
        # 이전 마커 전부 삭제(잔상 방지)
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, obj in enumerate(msg.objects):
            r, g, b = COLORS.get(obj.source, (0.7, 0.7, 0.7))

            # 위치 구
            m = Marker()
            m.header = msg.header
            m.ns = 'objects'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = obj.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = float(self.sphere_size)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.85
            m.lifetime = rclpy.duration.Duration(seconds=self.lifetime).to_msg()
            arr.markers.append(m)

            # 라벨 텍스트
            t = Marker()
            t.header = msg.header
            t.ns = 'labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = obj.position.x
            t.pose.position.y = obj.position.y
            t.pose.position.z = obj.position.z + 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.2
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
            name = obj.class_name if obj.class_name else str(obj.class_id)
            t.text = f'{name} {obj.distance:.1f}m'
            t.lifetime = rclpy.duration.Duration(seconds=self.lifetime).to_msg()
            arr.markers.append(t)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionMarkers()
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

#!/usr/bin/env python3
"""IMU 를 로봇 body 프레임으로 회전시켜 재발행한다.

왜 필요한가
-----------
D455 가 내는 /camera/imu 의 frame_id 는 camera_imu_optical_frame 이고, optical
규약(x=우, y=하, z=전방)이라 로봇 base_link 축과 90도씩 어긋나 있다.

robot_localization 의 `imu0_config` 배열은 **센서 자신의 프레임 기준**으로
해석된 뒤 base_link 로 회전된다. 그래서 optical 프레임 IMU 에 vyaw(index 11)를
켜면, 그건 "센서 z축 회전" = 로봇의 전방축 회전으로 해석되어 vroll(index 9)로
옮겨가고, two_d_mode 가 vroll 을 0 으로 강제하면서 자이로 yaw 가 영원히
융합되지 않는다. 실제 디버그 출력:

    Original update vector:                    ... index 11 = true
    After applying transform to base_footprint ... index  9 = true

증상은 "정지 중인데 odom->base_footprint 가 계속 회전"으로 나타난다. yaw 가
관측 불가능해져 분산이 발산하고, /odom 의 x,y 보정이 yaw 로 새어 들어간다.

해결
----
각속도·선가속도를 target_frame 축으로 회전시켜 frame_id=target_frame 으로
다시 낸다. 그러면 robot_localization 쪽 변환이 항등이 되어 vyaw(index 11)가
글자 그대로 로봇의 yaw 를 뜻하게 되고, 카메라 장착 자세가 바뀌어도 안전하다.

각속도는 강체 위 모든 점에서 동일하므로 위치 오프셋(레버암)은 무시해도 된다.
회전만 적용한다. (선가속도는 회전 중 구심가속도 항이 생기지만, 이 스택은
가속도를 융합하지 않으므로 참고용으로만 회전시켜 싣는다.)

자이로 바이어스 보정
------------------
ekf.yaml 은 yaw 의 절대 기준이 없다 — odom0 에서 yaw·vyaw 를 빼고 imu0 의 vyaw
하나만 융합한다. 그래서 yaw 는 순수 적분기가 되고 자이로 바이어스가 무한정 쌓인다.

2026-09-03 실측: 로봇을 완전히 정지시킨 채 90 초.
    /odom               yaw 드리프트  0.00 °   (바퀴가 안 도니 당연)
    /odometry/filtered  yaw 드리프트 -8.24 °  = -5.50 °/min  (= -0.0016 rad/s)
10 분이면 55 ° 다. 이 상태로는 localization 이 성립하지 않는다.

그래서 기동 직후 bias_calib_s 동안의 각속도 평균을 바이어스로 잡아 빼준다.
같은 날 같은 로봇에서 보정 후 다시 90 초:
    /odometry/filtered  yaw 드리프트 -0.53 °  = -0.35 °/min   (16 배 개선)
    (이때 /odom 도 0.59 °/min 움직였다 = 로봇이 완전 정지는 아니었으므로 실제 잔차는 더 작다)
노드가 잡은 바이어스는 -0.00168 rad/s = -5.78 °/min 으로, 위 실측 드리프트와 맞는다.
로봇이 정지해 있다는 전제이므로, 표본 표준편차가 bias_max_std 를 넘으면
"움직이는 중"으로 보고 창을 버리고 다시 모은다.

※ 바이어스는 온도에 따라 변한다. 이 보정은 기동 시점 한 번이다. 장시간 운용에서
  다시 새면 재기동하거나, 정지 구간을 감지해 갱신하는 쪽으로 확장해야 한다.
※ 이걸로도 yaw 가 관측 가능해지는 것은 아니다 — 바이어스를 뺀 적분기일 뿐이다.
  진짜 절대 기준은 SLAM(map 정합)뿐이고, 이 EKF 는 world_frame: odom 이라 그걸 안 받는다.

※ cartographer 는 이 토픽을 쓰지 않는다. cartographer 의 tracking_frame 은
  IMU 프레임 그 자체라 원본 /camera/imu 를 그대로 받아야 한다.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from sensor_msgs.msg import Imu


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ImuToBaseNode(Node):

    def __init__(self):
        super().__init__('imu_to_base_node')

        self.declare_parameter('target_frame', 'base_footprint')
        # 기동 후 이 시간[s] 동안 정지로 보고 자이로 바이어스를 추정한다. 0 이하면 보정을 끈다.
        self.declare_parameter('bias_calib_s', 3.0)
        # 창 안 각속도의 표준편차가 이 값[rad/s]을 넘으면 움직이는 중 -> 창을 버리고 다시 모은다.
        self.declare_parameter('bias_max_std', 0.02)
        self.target_frame = self.get_parameter('target_frame').value
        self.calib_s = float(self.get_parameter('bias_calib_s').value)
        self.max_std = float(self.get_parameter('bias_max_std').value)

        self.bias = np.zeros(3)
        self._samples = []
        self._win_t0 = None
        self._calibrated = self.calib_s <= 0.0
        self._retries = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._rot = None          # 정적 TF 라 한 번만 조회
        self._warned = False

        # RealSense 드라이버는 BEST_EFFORT 로 낸다.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self.sub = self.create_subscription(Imu, 'imu_in', self.cb, qos)
        self.pub = self.create_publisher(Imu, 'imu_out', qos)

        self.get_logger().info(f'IMU 를 {self.target_frame} 축으로 회전시켜 재발행')

    def _rotation(self, source_frame):
        if self._rot is not None:
            return self._rot
        if source_frame == self.target_frame:
            self._rot = np.eye(3)
            return self._rot
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            if not self._warned:
                self.get_logger().warn(
                    f'{self.target_frame} <- {source_frame} TF 대기 중: {exc}')
                self._warned = True
            return None
        q = t.transform.rotation
        self._rot = quat_to_matrix((q.x, q.y, q.z, q.w))
        self.get_logger().info(f'{source_frame} -> {self.target_frame} 회전 확보')
        return self._rot

    def _collect(self, w):
        """정지 전제 창에서 각속도를 모아 바이어스를 잡는다. 움직였으면 창을 버린다."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._win_t0 is None:
            self._win_t0 = now
        self._samples.append(w)
        if now - self._win_t0 < self.calib_s:
            return
        S = np.asarray(self._samples)
        std = S.std(axis=0)
        self._samples = []
        self._win_t0 = None
        if std.max() > self.max_std:
            self._retries += 1
            self.get_logger().warn(
                f'자이로 바이어스 보정 보류 — 움직이는 중으로 보인다 '
                f'(std {np.array2string(std, precision=4)} > {self.max_std}). '
                f'로봇을 정지시켜라. 재시도 {self._retries}')
            return
        self.bias = S.mean(axis=0)
        self._calibrated = True
        self.get_logger().info(
            f'자이로 바이어스 보정 완료 ({len(S)} 표본, {self.calib_s:.1f} s): '
            f'{np.array2string(self.bias, precision=5)} rad/s   '
            f'yaw {math.degrees(self.bias[2]) * 60:+.2f} °/min 제거')

    def cb(self, msg: Imu):
        R = self._rotation(msg.header.frame_id)
        if R is None:
            return

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        w = R @ np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        if not self._calibrated:
            self._collect(w)
        w = w - self.bias
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = w

        a = R @ np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = a

        # 공분산도 같은 회전으로 옮긴다: C' = R C R^T
        for src, dst in ((msg.angular_velocity_covariance, out.angular_velocity_covariance),
                         (msg.linear_acceleration_covariance, out.linear_acceleration_covariance)):
            c = np.array(src).reshape(3, 3)
            dst[:] = (R @ c @ R.T).flatten().tolist()

        # D455 IMU 는 절대 자세를 주지 않는다. 규약대로 -1 로 표시해 둔다.
        out.orientation_covariance[0] = -1.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToBaseNode()
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

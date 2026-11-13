#!/usr/bin/env python3
"""cartographer 의 /tracked_pose 를 robot_localization 이 먹을 Odometry 로 변환.

cartographer 는 publish_to_tf=false 로 두고 TF 대신 /tracked_pose 만 낸다.
그 pose 를 EKF 에 넣으려면 두 가지를 해줘야 한다.

1) 프레임 변환
   /tracked_pose 는 map -> tracking_frame 의 pose 다(node.cpp 의 tracking_to_map).
   우리 tracking_frame 은 camera_imu_optical_frame 이라 base_link 와
   위치가 약 10cm 어긋나 있고 optical 규약이라 90도 회전까지 되어 있다.
   그대로 child_frame_id='base_link' 로 넘기면 자세가 통째로 90도 틀어진다.
   그래서 URDF 의 정적 TF 로 map -> base_link 까지 합성해서 낸다.
   (레퍼런스 레이싱 스택은 IMU 가 base_link 와 거의 일치해서 이 단계가 없다.)

2) 공분산 주입
   cartographer 는 pose 공분산을 주지 않는다. 전부 0 인 채로 EKF 에 들어가면
   특이행렬이 되어 필터가 NaN 으로 발산한다. 파라미터로 직접 넣는다.

파라미터
  tracking_frame  cartographer lua 의 tracking_frame 과 반드시 일치시킬 것
  target_frame    출력 pose 의 기준 프레임(= Odometry.child_frame_id)
  cov_x/y/yaw     2D 로 실제로 쓰는 축의 분산. 값이 작을수록 EKF 가 더 믿는다.
  cov_unused      2D 에서 안 쓰는 축(z, roll, pitch)의 분산. 크게 넣어 무시시킨다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


class PoseToOdomNode(Node):

    def __init__(self):
        super().__init__('pose_to_odom_node')

        self.declare_parameter('tracking_frame', 'camera_imu_optical_frame')
        self.declare_parameter('target_frame', 'base_link')
        # 기본값은 cartographer 2D 스캔매칭의 통상 정밀도 수준(수 cm / 수십 mrad).
        # 실제 환경에서 pose 가 튀면 키우고, EKF 가 cartographer 를 안 따라가면 줄인다.
        self.declare_parameter('cov_x', 0.01)      # m^2
        self.declare_parameter('cov_y', 0.01)      # m^2
        self.declare_parameter('cov_yaw', 0.01)    # rad^2
        self.declare_parameter('cov_unused', 1e6)

        self.tracking_frame = self.get_parameter('tracking_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.cov_x = float(self.get_parameter('cov_x').value)
        self.cov_y = float(self.get_parameter('cov_y').value)
        self.cov_yaw = float(self.get_parameter('cov_yaw').value)
        self.cov_unused = float(self.get_parameter('cov_unused').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # tracking_frame -> target_frame 은 URDF 정적 TF 라 한 번만 조회하면 된다.
        self._tracking_to_target = None
        self._warned = False

        # cartographer 는 /tracked_pose 를 기본 QoS(RELIABLE)로 낸다.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub = self.create_subscription(PoseStamped, 'pose_in', self.cb, qos)
        self.pub = self.create_publisher(Odometry, 'odom_out', 10)

        self.get_logger().info(
            f'{self.tracking_frame} -> {self.target_frame} 로 변환해 발행. '
            f'cov(x,y,yaw)=({self.cov_x}, {self.cov_y}, {self.cov_yaw})')

    def _lookup_tracking_to_target(self):
        """target_frame 의 원점이 tracking_frame 에서 어디에 있는지(= T_tracking_target)."""
        if self._tracking_to_target is not None:
            return self._tracking_to_target
        if self.tracking_frame == self.target_frame:
            self._tracking_to_target = TransformStamped()   # identity
            self._tracking_to_target.transform.rotation.w = 1.0
            return self._tracking_to_target
        try:
            self._tracking_to_target = self.tf_buffer.lookup_transform(
                self.tracking_frame, self.target_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            if not self._warned:
                self.get_logger().warn(
                    f'{self.tracking_frame} -> {self.target_frame} TF 대기 중: {exc}')
                self._warned = True
            return None
        return self._tracking_to_target

    def cb(self, msg: PoseStamped):
        tracking_to_target = self._lookup_tracking_to_target()
        if tracking_to_target is None:
            return

        # msg.pose 는 T_map_tracking. 이걸 TransformStamped 로 감싸서
        # tracking_frame 안의 target_frame pose 에 적용하면 T_map_target 이 된다.
        map_to_tracking = TransformStamped()
        map_to_tracking.header = msg.header
        map_to_tracking.transform.translation.x = msg.pose.position.x
        map_to_tracking.transform.translation.y = msg.pose.position.y
        map_to_tracking.transform.translation.z = msg.pose.position.z
        map_to_tracking.transform.rotation = msg.pose.orientation

        target_pose_in_tracking = Pose()
        target_pose_in_tracking.position.x = tracking_to_target.transform.translation.x
        target_pose_in_tracking.position.y = tracking_to_target.transform.translation.y
        target_pose_in_tracking.position.z = tracking_to_target.transform.translation.z
        target_pose_in_tracking.orientation = tracking_to_target.transform.rotation

        odom = Odometry()
        odom.header = msg.header                 # frame_id = map
        odom.child_frame_id = self.target_frame
        odom.pose.pose = do_transform_pose(target_pose_in_tracking, map_to_tracking)

        # 대각 공분산: [x, y, z, roll, pitch, yaw]
        odom.pose.covariance[0] = self.cov_x
        odom.pose.covariance[7] = self.cov_y
        odom.pose.covariance[14] = self.cov_unused    # z     (2D 미사용)
        odom.pose.covariance[21] = self.cov_unused    # roll  (2D 미사용)
        odom.pose.covariance[28] = self.cov_unused    # pitch (2D 미사용)
        odom.pose.covariance[35] = self.cov_yaw

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomNode()
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

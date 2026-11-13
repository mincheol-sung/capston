#!/usr/bin/env python3
"""하드웨어 없이 배관을 검증하는 가짜 D455 점군 발행기.

실제 D455 가 camera_depth_optical_frame 에서 내는 것과 같은 규약으로 만든다:
  optical 좌표 — x 오른쪽, y 아래쪽, z 전방(깊이).

만드는 장면 (모두 base_footprint 기준으로 설명):
  바닥       z = 0            앞 0.4~4.0 m       -> z-band 밖. 걸러져야 한다.
  상자       z = 0.05~0.25 m  앞 1.5 m           -> z-band 안. 장애물.
  낮은 물체  z = 0.04~0.09 m  앞 1.0 m, 좌 0.5 m -> LiDAR 평면(0.218 m)이 놓치는 것
  상부 돌출  z = 0.9~1.1 m    앞 2.5 m           -> 밴드 위. 3D 에는 남고 2D 엔 없다

사용:
    ros2 run --prefix "python3" ...  는 필요 없다. 그냥 실행하면 된다.
    python3 tools/fake_depth_cloud.py

필요한 TF 는 스스로 발행한다(odom -> base_footprint -> camera_depth_optical_frame).
URDF 실측값을 그대로 쓴다 — 카메라는 지면 위 0.132772 m, 앞으로 0.11398 m.
"""
import argparse
import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster

# URDF amr_basic.urdf 실측
BASE_LINK_Z = 0.04
CAMERA_XYZ = np.array([0.11398, 0.0, 0.092772])
CAMERA_HEIGHT = BASE_LINK_Z + CAMERA_XYZ[2]          # 0.132772 m

# base_footprint -> optical 회전. camera_depth_optical_joint 의
# rpy = (-pi/2, 0, -pi/2) 이며, base(x 전방, z 위) -> optical(z 전방, y 아래).
BASE_TO_OPTICAL = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


def _slab(x_range, y_range, z_range, step):
    """축정렬 상자 표면을 점으로 채운다 (base_footprint 기준)."""
    xs = np.arange(x_range[0], x_range[1] + 1e-9, step)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, step)
    zs = np.arange(z_range[0], z_range[1] + 1e-9, step)
    grid = np.array(np.meshgrid(xs, ys, zs, indexing='ij')).reshape(3, -1).T
    return grid


def build_scene(step=0.02):
    """base_footprint 기준 장면을 만든다."""
    parts = {
        'floor': _slab((0.4, 4.0), (-1.2, 1.2), (0.0, 0.0), step * 2),
        'box': _slab((1.45, 1.60), (-0.25, 0.25), (0.05, 0.25), step),
        'low_object': _slab((0.95, 1.10), (0.40, 0.60), (0.04, 0.09), step),
        'overhang': _slab((2.45, 2.60), (-0.4, 0.4), (0.90, 1.10), step),
    }
    return parts


def to_optical(points_in_base):
    """base_footprint 좌표를 camera_depth_optical_frame 좌표로."""
    relative = points_in_base - np.array([CAMERA_XYZ[0], 0.0, CAMERA_HEIGHT])
    return relative @ BASE_TO_OPTICAL.T


class FakeDepthCloud(Node):
    def __init__(self, rate, publish_tf, step):
        super().__init__('fake_depth_cloud')
        self.publisher = self.create_publisher(
            PointCloud2, '/camera/depth/color/points', qos_profile_sensor_data)
        self.publish_tf = publish_tf
        self.broadcaster = TransformBroadcaster(self) if publish_tf else None

        parts = build_scene(step)
        scene = np.vstack(list(parts.values()))
        self.optical = to_optical(scene).astype(np.float32)
        counts = ', '.join(f'{k} {len(v)}' for k, v in parts.items())
        self.get_logger().info(
            f'fake D455: {len(scene)} points ({counts}) at {rate:.1f} Hz'
            + ('; publishing TF' if publish_tf else '; TF from elsewhere'))
        self.create_timer(1.0 / rate, self._tick)

    def _send_tf(self, stamp):
        def make(parent, child, xyz, rotation):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])
            t.transform.rotation.x = float(rotation[0])
            t.transform.rotation.y = float(rotation[1])
            t.transform.rotation.z = float(rotation[2])
            t.transform.rotation.w = float(rotation[3])
            return t

        half = math.sqrt(0.5)
        self.broadcaster.sendTransform([
            make('map', 'odom', (0, 0, 0), (0, 0, 0, 1)),
            make('odom', 'base_footprint', (0, 0, 0), (0, 0, 0, 1)),
            make('base_footprint', 'base_link', (0, 0, BASE_LINK_Z),
                 (0, 0, 0, 1)),
            make('base_link', 'camera_link', CAMERA_XYZ, (0, 0, 0, 1)),
            # rpy (-pi/2, 0, -pi/2) 를 quaternion 으로: (-0.5, 0.5, -0.5, 0.5)
            make('camera_link', 'camera_depth_optical_frame', (0, 0, 0),
                 (-0.5, 0.5, -0.5, 0.5)),
            make('base_link', 'laser_front', (0.086291, -0.020494, 0.177797),
                 (0, 0, 0, 1)),
        ])
        del half

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        if self.broadcaster is not None:
            self._send_tf(stamp)
        header = PointCloud2().header
        header.stamp = stamp
        header.frame_id = 'camera_depth_optical_frame'
        self.publisher.publish(
            point_cloud2.create_cloud_xyz32(header, self.optical))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument('--step', type=float, default=0.02,
                        help='장면 점 간격 [m]. 작을수록 점이 많아진다')
    parser.add_argument('--no-tf', action='store_true',
                        help='TF 는 실제 로봇/URDF 가 낸다고 가정')
    args = parser.parse_args()

    rclpy.init()
    node = FakeDepthCloud(args.rate, not args.no_tf, args.step)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""Apply an optional dynamic LaserScan mask to its corresponding timed cloud."""

from message_filters import Subscriber, TimeSynchronizer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

from .timed_cloud import records, select_records


def apply_mask(cloud, scan):
    """Use original LaserScan bin indices, including gaps in the sector cloud."""
    if cloud.header != scan.header:
        raise ValueError('Cloud and beam mask must come from the same scan')
    points = records(cloud)
    if points.size and points['index'].max() >= len(scan.ranges):
        raise ValueError('Beam mask does not cover cloud indices')
    ranges = np.asarray(scan.ranges)[points['index']]
    keep = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    return select_records(cloud, points, keep)


class MaskedTimedCloud(Node):
    """Match masks to the corresponding stream by exact acquisition stamp."""

    def __init__(self):
        super().__init__('masked_timed_cloud')
        points = Subscriber(self, PointCloud2, 'points_in', qos_profile=qos_profile_sensor_data)
        mask = Subscriber(self, LaserScan, 'scan_mask', qos_profile=qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'points_out', 10)
        self.sync = TimeSynchronizer([points, mask], 20)
        self.sync.registerCallback(self._publish)

    def _publish(self, cloud, scan):
        try:
            self.pub.publish(apply_mask(cloud, scan))
        except (KeyError, IndexError, ValueError) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = MaskedTimedCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

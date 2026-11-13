from unittest.mock import Mock

from amr_laser_tools.scan_merger import DualScanMerger
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan


def scan_at(stamp_ns):
    msg = LaserScan()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    return msg


def test_stale_input_is_removed_and_large_skew_drops_older_scan():
    rclpy.init()
    node = DualScanMerger()
    try:
        now_ns = node.get_clock().now().nanoseconds
        front, rear = node.input_topics

        node.latest[front] = (scan_at(now_ns), now_ns - 1_000_000_000)
        node.latest[rear] = (scan_at(now_ns), now_ns)
        fresh = node._fresh_inputs()
        assert [item[1] for item in fresh] == [rear]

        node.latest[front] = (scan_at(now_ns - 100_000_000), now_ns)
        fresh = node._fresh_inputs()
        assert [item[1] for item in fresh] == [rear]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_partial_scan_cannot_refresh_downstream_freshness():
    rclpy.init()
    node = DualScanMerger()
    try:
        node.pub = Mock()
        now_ns = node.get_clock().now().nanoseconds
        node.latest[node.input_topics[0]] = (scan_at(now_ns), now_ns)
        node._publish_merged()
        node.pub.publish.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_replayed_scan_is_stale_even_when_recently_received():
    rclpy.init()
    node = DualScanMerger()
    try:
        now_ns = node.get_clock().now().nanoseconds
        node.latest[node.input_topics[0]] = (scan_at(now_ns - 1_000_000_000), now_ns)
        assert node._fresh_inputs() == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_complete_scan_keeps_oldest_capture_stamp():
    rclpy.init()
    node = DualScanMerger()
    try:
        node.pub = Mock()
        node._scan_to_base_points = Mock(return_value=(np.empty(0), np.empty(0)))
        now_ns = node.get_clock().now().nanoseconds
        oldest = now_ns - 10_000_000
        for topic, stamp in zip(node.input_topics, (oldest, now_ns)):
            node.latest[topic] = (scan_at(stamp), now_ns)
        node._publish_merged()
        message = node.pub.publish.call_args.args[0]
        assert node._stamp_ns(message) == oldest
    finally:
        node.destroy_node()
        rclpy.shutdown()

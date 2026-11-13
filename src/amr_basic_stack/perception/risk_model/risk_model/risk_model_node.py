#!/usr/bin/env python3
"""risk_model — STUB 노드 (연결성 검증용).

입력: /predictions (MultiModalObjectPredictionArray),
      /map (nav_msgs/OccupancyGrid)
출력: /risk (amr_interfaces/RiskLayeredModel)

TODO(구현): nav2 costmap layer 형태의 위험도 레이어 생성.
"""
import rclpy
from amr_interfaces.msg import (
    MultiModalObjectPredictionArray,
    RiskLayeredModel,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class RiskModelNode(Node):
    """Connectivity stub that accepts the multi-modal prediction contract."""

    def __init__(self):
        super().__init__('risk_model_node')
        self.create_subscription(
            MultiModalObjectPredictionArray, '/predictions', self._on_pred, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.pub = self.create_publisher(RiskLayeredModel, '/risk', 10)
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info('risk_model_node (STUB) started')

    def _on_pred(self, msg):
        pass

    def _on_map(self, msg):
        pass

    def _tick(self):
        out = RiskLayeredModel()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        self.pub.publish(out)


def main(args=None):
    """Run the ROS 2 risk-model connectivity stub."""
    rclpy.init(args=args)
    node = RiskModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

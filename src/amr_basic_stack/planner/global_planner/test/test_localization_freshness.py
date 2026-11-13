"""Global planning cannot substitute an old initial pose for stale live TF."""

import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from global_planner.rviz_theta_star_node import RvizThetaStarNode
import pytest
from rclpy.time import Time
from tf2_ros import Buffer


@pytest.mark.parametrize('stamp, accepted', [(1, False), (10, True), (11, False)])
def test_start_uses_only_recent_localization(stamp, accepted):
    buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
    transform.header.stamp = TimeMsg(sec=stamp)
    transform.transform.rotation.w = 1.
    buffer.set_transform(transform, 'unit-test')
    n = NS(initial_pose=(99., 99.), initial_pose_received_at=time.monotonic() - 30.,
           initial_pose_grace_period=2., tf_buffer=buffer, tf_timeout=.2,
           transform_max_age=.5, map_frame='map', base_frame='base_footprint',
           get_clock=lambda: NS(now=lambda: Time(seconds=10.)), get_logger=lambda: Mock())
    if accepted:
        assert RvizThetaStarNode._planning_start(n) == ((0., 0.), 'TF')
    else:
        with pytest.raises(RuntimeError, match='unavailable'):
            RvizThetaStarNode._planning_start(n)

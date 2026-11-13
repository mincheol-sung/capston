"""Fresh publication cannot hide missing captures or stale localization."""

from types import SimpleNamespace as NS

from builtin_interfaces.msg import Time as TimeMsg
from dra_mppi.dra_mppi_node import DraMppiNode
from dra_mppi.dra_mppi_node import TraversabilityUnavailable
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from rclpy.time import Time
from test_guide_node import map_message
from test_guide_node import node
from tf2_ros import Buffer


@pytest.mark.parametrize('capture', [0., 1., 10.2])
def test_new_grid_with_missing_stale_or_future_capture_is_rejected(capture):
    n = node(static_cost_enabled=True)
    try:
        m = map_message()
        m.sensor_watermark = Time(seconds=capture).to_msg()
        DraMppiNode._on_traversable_map(n, m)
        with pytest.raises(TraversabilityUnavailable):
            DraMppiNode._traversability_for_control(n)
    finally:
        n.controller.close()


def test_recent_capture_is_accepted():
    n = node(static_cost_enabled=True)
    try:
        m = map_message()
        m.sensor_watermark = Time(seconds=9.8).to_msg()
        DraMppiNode._on_traversable_map(n, m)
        assert DraMppiNode._traversability_for_control(n)[0] is not None
    finally:
        n.controller.close()


@pytest.mark.parametrize('terminal', [False, True])
def test_stale_sensor_stops_both_cruise_and_terminal_alignment(terminal):
    n = node(static_cost_enabled=True)
    try:
        m = map_message()
        m.sensor_watermark = TimeMsg(sec=1)
        DraMppiNode._on_traversable_map(n, m)
        if terminal:
            n._path_goal_in_planning_frame = lambda: np.array([.05, 0., .5])
        DraMppiNode._control_cycle(n)
        n._publish_stop.assert_called_once()
        n._publish_reference.assert_not_called()
    finally:
        n.controller.close()


@pytest.mark.parametrize('stamp, accepted', [(1, False), (10, True), (11, False)])
def test_cached_tf_requires_recent_source_stamp(stamp, accepted):
    buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
    transform.header.stamp = TimeMsg(sec=stamp)
    transform.transform.rotation.w = 1.
    buffer.set_transform(transform, 'unit-test')
    n = NS(tf_buffer=buffer, transform_max_age=.5,
           get_clock=lambda: NS(now=lambda: Time(seconds=10.)))
    if accepted:
        assert DraMppiNode._lookup_2d(n, 'map', 'odom') == (0., 0., 0.)
    else:
        with pytest.raises(ValueError, match='stale'):
            DraMppiNode._lookup_2d(n, 'map', 'odom')


@pytest.mark.parametrize('stamp', [0., 1., 10.2])
def test_fresh_reception_cannot_hide_stale_odometry_source(stamp):
    from nav_msgs.msg import Odometry
    n = node()
    previous = n._last_odom_monotonic
    m = Odometry()
    m.header.stamp = Time(seconds=stamp).to_msg()
    m.pose.pose.orientation.w = 1.
    try:
        DraMppiNode._on_odometry(n, m)
        assert n._last_odom_monotonic == previous
    finally:
        n.controller.close()


@pytest.mark.parametrize('stamp, steps, reason', [(10.2, 2, 'future'), (9.9, 5, 'horizon')])
def test_prediction_source_time_and_horizon_fail_closed(stamp, steps, reason):
    import time
    from amr_interfaces.msg import MultiModalObjectPredictionArray
    from rclpy.clock import ClockType
    from dra_mppi.dra_mppi_node import PredictionChainUnavailable
    from test_prediction_filter import _filter, _quality_prediction

    message = MultiModalObjectPredictionArray(objects=[_quality_prediction()])
    message.header.stamp = Time(seconds=stamp).to_msg()
    n = NS(
        _prediction_message=_filter(message), _last_prediction_monotonic=time.monotonic(),
        prediction_timeout=.5, prediction_fail_safe=True, planning_frame='odom',
        prediction_class_names=['person'], prediction_max_semantic_age=1.,
        prediction_max_measurement_age=.5, prediction_require_confirmed=True,
        get_clock=lambda: NS(now=lambda: Time(seconds=10., clock_type=ClockType.ROS_TIME)),
        _lookup_2d=lambda *args: (0., 0., 0.),
        controller=NS(config=NS(horizon_steps=steps, dt=.1,
                               dynamic_risk=NS(obstacle_radius=.25))))
    with pytest.raises(PredictionChainUnavailable, match=reason):
        DraMppiNode._aligned_predictions(n)

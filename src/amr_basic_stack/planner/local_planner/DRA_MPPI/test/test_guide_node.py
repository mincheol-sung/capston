"""ROS adapter checks without starting a live robot graph."""

import time
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

from amr_interfaces.msg import TraversableMap
from builtin_interfaces.msg import Time
from dra_mppi.dra_mppi_node import DraMppiNode
from dra_mppi.dynamic_risk import DynamicRiskConfig, PredictionSet
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiConfig, MppiController
from dra_mppi.path_handler import GoalCompletionLatch
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.parameter import Parameter
import numpy as np
import pytest


def node(static_cost_enabled=False):
    c = MppiController(MppiConfig(
        strategy='guide_conditioned', dt=.2, horizon_steps=10,
        guide_num_rollouts=50, desired_speed=.75,
        control_min=(-.375, -.5625, -3.), control_max=(.78, .5625, 3.),
        acceleration_limit=(2.25, 2.25, 6.75), heading_weight=3.,
        static_cost_enabled=static_cost_enabled,
        static_cost_robot_radius=.2,
        dynamic_risk=DynamicRiskConfig(mode='off', robot_radius=.2)))
    clock = SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: Time(sec=10)))
    n = SimpleNamespace(
        controller=c, _distance_field=None, _traversable_stamp_ns=0,
        _traversable_sensor_stamp_ns=0,
        _traversable_received_at=0., traversable_timeout=1.,
        traversable_unknown_is_obstacle=True, planning_frame='odom',
        get_clock=lambda: clock, get_logger=lambda: Mock(),
        _lookup_2d=Mock(return_value=(0., 0., 0.)),
        _state=np.zeros(3), _current_control=np.zeros(3),
        _goal_completion=GoalCompletionLatch(), _goal_path_pending=False,
        _last_nonempty_path_goal=None,
        _path_message=object(), _last_odom_monotonic=time.monotonic(), odom_timeout=.25,
        _publish_stop=Mock(), _clear_rollout_markers=Mock(),
        rollout_publisher=SimpleNamespace(publish=Mock()),
        _publish_reference=Mock(), _publish_reference_trajectory=Mock(),
        _publish_selected_path=Mock(), _publish_guide_mc_diagnostics=Mock(),
        _aligning_goal_heading=False,
        _goal_settle_count=0,
        _clear_local_visualization=Mock(), goal_tolerance=.15, goal_yaw_tolerance=.087,
        goal_heading_max_yaw_rate=.6, goal_heading_gain=1.5,
        goal_terminal_exit_tolerance=.25,
        goal_terminal_completion_tolerance=.08,
        goal_terminal_max_linear_speed=.10,
        goal_terminal_linear_gain=1.0,
        goal_terminal_settle_linear_speed=.03,
        goal_terminal_settle_yaw_rate=.05,
        goal_terminal_settle_cycles=3,
        path_lookahead=2., max_path_points=80, visualization_rollouts=20,
        controller_period=.2, _last_status_log=time.monotonic(),
        _path_in_planning_frame=lambda: np.array([[0., 0.], [2., 0.]]),
        _path_goal_in_planning_frame=lambda: np.array([2., 0., 0.]),
        _aligned_predictions=lambda: None,
        observation_target_timeout=.5, _observation_target_input=None,
    )
    n._path_goal_identity = DraMppiNode._path_goal_identity
    n._same_path_goal = DraMppiNode._same_path_goal
    n._traversability_for_control = lambda: DraMppiNode._traversability_for_control(n)
    n._observation_target_for_control = lambda: DraMppiNode._observation_target_for_control(n)
    n._publish_rollouts = lambda result, stamp: DraMppiNode._publish_rollouts(n, result, stamp)
    return n


def map_message():
    m = TraversableMap()
    m.header.frame_id = m.grid.header.frame_id = 'map'
    m.header.stamp = m.grid.header.stamp = Time(sec=10)
    m.sensor_watermark = Time(sec=10)
    m.grid.info.width = m.grid.info.height = 120
    m.grid.info.resolution = .05
    m.grid.info.origin.position.x = m.grid.info.origin.position.y = -3.
    m.grid.info.origin.orientation.w = 1.
    m.grid.data = [0] * 14400
    return m


@pytest.mark.parametrize('terminal', [False, True])
def test_prediction_budget_overflow_publishes_hold_before_motion(terminal):
    n = node()
    n.controller.config.dynamic_risk = replace(
        n.controller.config.dynamic_risk, mode='monte_carlo', max_obstacles=20)
    if terminal:
        n._state = np.array([1.95, 0., 0.])
    pred = PredictionSet(np.zeros((21, 1, 10, 2)),
        np.broadcast_to(np.eye(2)*.1, (21, 1, 10, 2, 2)), .2*np.arange(1, 11))
    n._aligned_predictions = lambda: pred
    n.controller.compute = Mock(side_effect=AssertionError('budget must be checked first'))
    try:
        DraMppiNode._control_cycle(n)
        assert n._cycle_status == 'prediction_budget_hold'
        assert '21 > 20' in n._cycle_error
        n._publish_stop.assert_called_once()
        n._publish_reference.assert_not_called()
        n._publish_reference_trajectory.assert_not_called()
        n.controller.compute.assert_not_called()
        assert len(pred.means) == 21
    finally:
        n.controller.close()


def test_terminal_alignment_uses_final_integrator():
    from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability
    n = node()
    n.controller.config.dynamic_risk = replace(
        n.controller.config.dynamic_risk, mode='monte_carlo', final_mode='gaussian_disk')
    final = GaussianDiskCollisionProbability()
    final.evaluate = Mock(wraps=final.evaluate)
    n.controller._final_risk_estimator = final
    n._state = np.array([1.95, 0., 0.])
    pred = PredictionSet(np.full((1, 1, 10, 2), 10.),
        np.broadcast_to(np.eye(2)*.1, (1, 1, 10, 2, 2)), .2*np.arange(1, 11))
    n._aligned_predictions = lambda: pred
    try:
        DraMppiNode._control_cycle(n)
        final.evaluate.assert_called_once()
        n._publish_reference.assert_called_once()
    finally:
        n.controller.close()


def path_message(goal_x, goal_y, goal_yaw=0., start=(0., 0.)):
    message = Path()
    message.header.frame_id = 'map'
    for x, y, yaw in ((*start, 0.), (goal_x, goal_y, goal_yaw)):
        pose = PoseStamped()
        message.poses.append(pose)
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = np.sin(yaw / 2.)
        pose.pose.orientation.w = np.cos(yaw / 2.)
    return message


def test_guide_mc_diagnostics_publish_complete_guide_time_matrices():
    areas = np.arange(50, dtype=float).reshape(5, 10) / 10.
    counts = np.arange(128, 178, dtype=np.int32).reshape(5, 10)
    area_publisher = SimpleNamespace(publish=Mock())
    count_publisher = SimpleNamespace(publish=Mock())
    adapter = SimpleNamespace(
        guide_mc_region_areas_publisher=area_publisher,
        guide_mc_sample_counts_publisher=count_publisher,
        _guide_mc_layout=DraMppiNode._guide_mc_layout,
    )
    result = SimpleNamespace(
        guide_mc_region_areas=areas,
        guide_mc_sample_counts=counts,
    )

    DraMppiNode._publish_guide_mc_diagnostics(adapter, result)

    area_message = area_publisher.publish.call_args.args[0]
    count_message = count_publisher.publish.call_args.args[0]
    assert [(d.label, d.size, d.stride) for d in area_message.layout.dim] == [
        ('guide', 5, 50), ('timestep', 10, 10)]
    assert np.allclose(area_message.data, areas.ravel())
    assert list(count_message.data) == counts.ravel().tolist()


@pytest.mark.parametrize('fault', [
    'missing', 'receipt_stale', 'header_stale', 'future', 'tf', 'nan_tf',
])
def test_map_faults_do_not_block_control(fault):
    n = node()
    m = map_message()
    DraMppiNode._on_traversable_map(n, m)
    if fault == 'missing':
        n._distance_field = None
    elif fault == 'receipt_stale':
        n._traversable_received_at -= 2.
    elif fault == 'header_stale':
        n._traversable_stamp_ns -= 2_000_000_000
    elif fault == 'future':
        n._traversable_stamp_ns += 2_000_000_000
    elif fault == 'tf':
        n._lookup_2d.side_effect = RuntimeError('no TF')
    else:
        n._lookup_2d.return_value = (0., 0., float('nan'))
    n.controller._guide_optimizer.initialized[:] = True
    DraMppiNode._control_cycle(n)
    n._publish_stop.assert_not_called()
    n._publish_reference.assert_called_once()
    assert n.controller._guide_optimizer.initialized.all()
    n._lookup_2d.assert_not_called()


def test_control_without_map_publishes_guides_and_selected_path():
    n = node()
    DraMppiNode._control_cycle(n)
    n._publish_stop.assert_not_called()
    n._publish_reference.assert_called_once()
    assert n._publish_reference.call_args.args[1][0] > 0.
    markers = n.rollout_publisher.publish.call_args.args[0].markers
    assert len([m for m in markers if m.ns == 'guide_paths']) == 5
    rollout_namespaces = {
        m.ns for m in markers if m.ns.startswith('guide_rollouts_')
    }
    assert rollout_namespaces == {
        'guide_rollouts_guide_+0',
        'guide_rollouts_guide_+45',
        'guide_rollouts_guide_+90',
        'guide_rollouts_guide_-45',
        'guide_rollouts_guide_-90',
    }
    assert len([m for m in markers if m.ns == 'selected_path']) == 1
    assert not any(m.ns in {
        'guide_status', 'mode_trajectories', 'mode_statistics'
    } for m in markers)
    n._lookup_2d.assert_not_called()


def test_enabled_static_cost_requires_a_fresh_map():
    n = node(static_cost_enabled=True)

    DraMppiNode._control_cycle(n)

    n._publish_stop.assert_called_once()
    n._publish_reference.assert_not_called()


def test_runtime_static_switch_updates_controller_and_keeps_dynamic_config():
    n = node()
    risk = n.controller.config.dynamic_risk
    for enabled in (True, False):
        params = [Parameter('static_cost.enabled', value=enabled)]
        assert DraMppiNode._validate_runtime_parameters(n, params).successful
        DraMppiNode._apply_runtime_parameters(n, params)
        assert n.controller.config.static_cost_enabled is enabled
        assert n.controller.config.dynamic_risk is risk
    params = [Parameter('static_cost.enabled', value='false')]
    assert not DraMppiNode._validate_runtime_parameters(n, params).successful


@pytest.mark.parametrize('fault', ['receipt', 'sensor', 'header', 'inflation'])
def test_enabled_static_switch_rejects_stale_or_inflated_map(fault):
    n = node(static_cost_enabled=True)
    m = map_message()
    if fault == 'inflation':
        m.inflation_radius = .2
    DraMppiNode._on_traversable_map(n, m)
    if fault == 'receipt':
        n._traversable_received_at -= 2.
    elif fault == 'sensor':
        n._traversable_sensor_stamp_ns -= 2_000_000_000
    elif fault == 'header':
        n._traversable_stamp_ns -= 2_000_000_000
    DraMppiNode._control_cycle(n)
    n._publish_stop.assert_called_once()
    n._publish_reference.assert_not_called()
    n.controller.close()


def test_enabled_static_cost_uses_fresh_map_and_transform():
    n = node(static_cost_enabled=True)
    DraMppiNode._on_traversable_map(n, map_message())

    DraMppiNode._control_cycle(n)

    n._publish_stop.assert_not_called()
    n._publish_reference.assert_called_once()
    n._lookup_2d.assert_called_once_with('map', 'odom')


@pytest.mark.parametrize('fault', ['dimensions', 'resolution', 'frames'])
def test_bad_map_invalidates_previous_field(fault):
    n, m = node(), map_message()
    DraMppiNode._on_traversable_map(n, m)
    assert n._distance_field is not None
    if fault == 'dimensions':
        m.grid.data = [0]
    elif fault == 'resolution':
        m.grid.info.resolution = 0.
    else:
        m.grid.header.frame_id = 'wrong_frame'
    DraMppiNode._on_traversable_map(n, m)
    assert n._distance_field is None


def test_unknown_current_footprint_does_not_block_terminal_rotation():
    n, m = node(), map_message()
    m.grid.data = [-1] * 14400
    DraMppiNode._on_traversable_map(n, m)
    n._path_goal_in_planning_frame = lambda: np.array([0., 0., 1.])
    DraMppiNode._control_cycle(n)
    n._publish_stop.assert_not_called()
    n._publish_reference.assert_called_once()


def test_terminal_alignment_and_completed_goal_hold_still_work():
    n = node()
    DraMppiNode._on_traversable_map(n, map_message())
    n._path_in_planning_frame = lambda: np.array([[-1., 0.], [0., 0.]])
    n._path_goal_in_planning_frame = lambda: np.array([0., 0., 1.])
    DraMppiNode._control_cycle(n)
    assert n._aligning_goal_heading
    ref = n._publish_reference_trajectory.call_args.args[0]
    assert np.all(ref[:, :2] == 0.)
    assert ref[0, 5] > 0.
    n._path_goal_in_planning_frame = lambda: np.zeros(3)
    for _ in range(n.goal_terminal_settle_cycles - 1):
        DraMppiNode._control_cycle(n)
        assert not n._goal_completion.completed
    DraMppiNode._control_cycle(n)
    assert n._goal_completion.completed
    n._publish_stop.assert_called_once()
    assert np.array_equal(n._goal_completion.hold_state, np.zeros(3))


def test_terminal_mode_keeps_correcting_inside_exit_hysteresis():
    n = node()
    DraMppiNode._on_traversable_map(n, map_message())
    n._state = np.array([.14, 0., 0.])
    n._path_in_planning_frame = lambda: np.array([[-1., 0.], [0., 0.]])
    n._path_goal_in_planning_frame = lambda: np.zeros(3)

    DraMppiNode._control_cycle(n)
    assert n._aligning_goal_heading
    first = n._publish_reference_trajectory.call_args.args[0]
    assert first[0, 0] < n._state[0]
    assert first[0, 3] < 0.

    n._state = np.array([.20, 0., 0.])
    DraMppiNode._control_cycle(n)
    assert n._aligning_goal_heading

    n._state = np.array([.26, 0., 0.])
    DraMppiNode._control_cycle(n)
    assert not n._aligning_goal_heading


def test_same_goal_path_refresh_does_not_cancel_terminal_state():
    n = node()
    n._aligning_goal_heading = True
    n._goal_settle_count = 2
    message = path_message(2., 3., .4)

    DraMppiNode._on_path(n, message)

    assert n._aligning_goal_heading
    assert n._goal_settle_count == 2


def test_same_goal_replan_does_not_release_completion_latch():
    n = node()
    DraMppiNode._on_path(n, path_message(2., 3., .4, start=(0., 0.)))
    n._goal_completion.completed = True
    n._goal_completion.hold_state = np.array([2., 3., .4])

    DraMppiNode._on_path(n, path_message(2., 3., .4, start=(1., 1.)))

    assert n._goal_completion.completed
    assert np.array_equal(n._goal_completion.hold_state, [2., 3., .4])


@pytest.mark.parametrize('replacement', [
    path_message(2.1, 3., .4),
    path_message(2., 3., -.4),
])
def test_replacement_path_endpoint_releases_completion_latch(replacement):
    n = node()
    DraMppiNode._on_path(n, path_message(2., 3., .4))
    n._goal_completion.completed = True
    n._goal_completion.hold_state = np.array([2., 3., .4])

    # The global planner publishes an empty path before a new planned path.
    DraMppiNode._on_path(n, Path())
    assert n._goal_completion.completed

    DraMppiNode._on_path(n, replacement)

    assert not n._goal_completion.completed
    assert n._goal_completion.hold_state is None


def test_fully_blocked_static_map_does_not_disable_rviz_guides():
    n = node()
    f = DistanceField.from_occupancy(np.full((120, 120), 100),
                                     GridMetadata(120, 120, .05, -3., -3., 0.))
    r = n.controller.compute(np.zeros(3), np.array([[0., 0.], [2., 0.]]),
                             distance_field=f)
    DraMppiNode._publish_rollouts(n, r, Time(sec=10))
    assert r.safety_status == 'ok'
    assert r.guide_active_mask.all()
    assert n.rollout_publisher.publish.call_count == 1
    markers = n.rollout_publisher.publish.call_args.args[0].markers
    assert len([m for m in markers if m.ns == 'guide_paths']) == 5
    assert any(m.ns.startswith('guide_rollouts_') for m in markers)
    assert len([m for m in markers if m.ns == 'selected_path']) == 1


def test_observation_target_toggle_freshness_and_control_forwarding():
    n = node()
    message = PointStamped()
    message.header.frame_id = 'odom'
    message.header.stamp = Time(sec=10)
    message.point.y = 2.
    enabled = Parameter('observation_cost.enabled', value=True)
    try:
        DraMppiNode._on_observation_target(n, message)
        assert n._observation_target_for_control() is None
        assert DraMppiNode._validate_runtime_parameters(n, [enabled]).successful
        assert not DraMppiNode._validate_runtime_parameters(n, [
            Parameter('observation_cost.enabled', value='true')]).successful
        DraMppiNode._apply_runtime_parameters(n, [enabled])
        assert n._observation_target_for_control() is None  # fresh selection required
        DraMppiNode._on_observation_target(n, message)
        np.testing.assert_equal(n._observation_target_for_control(), [0., 2.])
        n.controller.compute = Mock(wraps=n.controller.compute)
        DraMppiNode._control_cycle(n)
        np.testing.assert_equal(n.controller.compute.call_args.kwargs['observation_target'], [0., 2.])
        assert n._cycle_status == 'normal'
        xy, stamp, received = n._observation_target_input
        n._observation_target_input = (xy, stamp, received-1.)
        assert n._observation_target_for_control() is None
        DraMppiNode._on_observation_target(n, message)
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11_000_000_000))
        assert n._observation_target_for_control() is None
        DraMppiNode._apply_runtime_parameters(n, [Parameter('observation_cost.enabled', value=False)])
        assert not n.controller.config.observation_cost_enabled
        assert n.controller._observation_target is None
    finally:
        n.controller.close()


@pytest.mark.parametrize('frame,sec,x', [('map', 10, 1.), ('odom', 0, 1.),
    ('odom', 9, 1.), ('odom', 11, 1.), ('odom', 10, float('nan'))])
def test_observation_target_invalid_input_clears_selection(frame, sec, x):
    n = node()
    n.controller.config.observation_cost_enabled = True
    message = PointStamped()
    message.header.frame_id = frame
    message.header.stamp = Time(sec=sec)
    message.point.x = x
    n._observation_target_input = (np.ones(2), 10_000_000_000, time.monotonic())
    try:
        DraMppiNode._on_observation_target(n, message)
        assert n._observation_target_for_control() is None
    finally:
        n.controller.close()

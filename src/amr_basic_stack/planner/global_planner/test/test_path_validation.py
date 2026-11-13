"""Tests for global path validation helpers."""

import time
from types import SimpleNamespace

from global_planner.rviz_theta_star_node import (
    nearest_free_cell,
    remaining_path_has_collision,
    RvizThetaStarNode,
)
from global_planner.theta_star.costmap import Costmap

import numpy as np


class GridMap:
    """Minimal world-to-grid adapter for collision tests."""

    def world_to_grid(self, point):
        """Map test world coordinates directly to integer grid cells."""
        return int(point[0]), int(point[1])


def test_blocked_endpoint_snaps_to_nearest_free_cell():
    raw = np.zeros((9, 9), dtype=np.uint8)
    raw[3:6, 3:6] = 1
    costmap = Costmap(raw, robot_size=1)

    resolved = nearest_free_cell((4, 4), costmap, maximum_radius_cells=3)

    assert resolved is not None
    assert not costmap.is_collision(*resolved)
    assert np.isclose(np.linalg.norm(np.asarray(resolved) - [4, 4]), 2.0)


def test_endpoint_snap_is_bounded_and_keeps_free_cells_unchanged():
    raw = np.ones((7, 7), dtype=np.uint8)
    raw[0, 0] = 0
    costmap = Costmap(raw, robot_size=1)

    assert nearest_free_cell((0, 0), costmap, 2) == (0, 0)
    assert nearest_free_cell((3, 3), costmap, 2) is None


def test_remaining_path_checks_cells_between_waypoints():
    """An obstacle between sparse waypoints must still block the path."""
    raw = np.zeros((8, 8), dtype=np.uint8)
    raw[2, 3] = 1
    costmap = Costmap(raw, robot_size=1)
    path = [(1.0, 2.0), (6.0, 2.0)]

    assert remaining_path_has_collision(path, (1.0, 2.0), GridMap(), costmap)


def test_remaining_path_ignores_unpublished_connector_from_robot():
    """An off-path robot pose must not create an imaginary blocked segment."""
    raw = np.zeros((8, 8), dtype=np.uint8)
    raw[3, 2] = 1
    costmap = Costmap(raw, robot_size=1)
    path = [(2.0, 2.0), (6.0, 2.0)]

    assert not remaining_path_has_collision(
        path,
        (2.0, 4.0),
        GridMap(),
        costmap,
    )


class PlanningCostmapFixture:
    """Minimal receiver for testing RvizThetaStarNode planning-map policy."""

    def __init__(self, static_costmap, traversable):
        self.costmap = static_costmap
        self.traversable = traversable

    def _fresh_traversable_layer(self, now, verified=False):
        return self.traversable


def test_fresh_traversable_map_replaces_saved_map():
    raw = np.zeros((5, 5), dtype=np.uint8)
    raw[2, 2] = 1
    static_costmap = Costmap(raw, robot_size=1)
    fixture = PlanningCostmapFixture(
        static_costmap,
        np.zeros((5, 5), dtype=bool),
    )

    planning = RvizThetaStarNode._planning_costmap(fixture)

    assert static_costmap.is_collision(2, 2)
    assert not planning.is_collision(2, 2)


def test_replanning_requires_a_fresh_traversable_map():
    static_costmap = Costmap(np.zeros((5, 5), dtype=np.uint8), robot_size=1)
    fixture = PlanningCostmapFixture(static_costmap, None)

    assert RvizThetaStarNode._planning_costmap(fixture) is static_costmap
    assert RvizThetaStarNode._planning_costmap(
        fixture,
        require_fresh_traversable=True,
    ) is None


class VerifiedPolicyFixture:
    """Receiver carrying both traversable layers, as built from one map message."""

    _fresh_traversable_layer = RvizThetaStarNode._fresh_traversable_layer
    _planning_costmap = RvizThetaStarNode._planning_costmap
    _consume_verified_goal = RvizThetaStarNode._consume_verified_goal

    def __init__(self):
        self.costmap = Costmap(np.zeros((5, 5), dtype=np.uint8), robot_size=1)
        self.traversable_map_timeout = 2.0
        self.verified_goal_ttl = 10.0
        self._traversable_received_at = time.monotonic()
        # Unknown cell (2, 2) is passable for RViz goals but blocked when verified.
        self._traversable_mask = np.zeros((5, 5), dtype=bool)
        self._verified_mask = np.zeros((5, 5), dtype=bool)
        self._verified_mask[2, 2] = True
        self._verified_goal = None


def test_verified_goal_plans_and_replans_with_unknown_blocked():
    fixture = VerifiedPolicyFixture()
    assert not fixture._planning_costmap().is_collision(2, 2)
    assert fixture._planning_costmap(require_fresh_traversable=True,
                                     verified=True).is_collision(2, 2)


def test_verified_goal_never_falls_back_to_static_map():
    fixture = VerifiedPolicyFixture()
    fixture._traversable_received_at = time.monotonic() - 10.0
    assert fixture._planning_costmap() is fixture.costmap
    assert fixture._planning_costmap(verified=True) is None


def test_only_the_validated_endpoint_is_verified_once():
    fixture = VerifiedPolicyFixture()
    fixture._verified_goal = (time.monotonic(), (1.0, 2.0))
    assert not fixture._consume_verified_goal((1.0, 2.1))
    fixture._verified_goal = (time.monotonic(), (1.0, 2.0))
    assert fixture._consume_verified_goal((1.0, 2.0))
    assert not fixture._consume_verified_goal((1.0, 2.0))
    fixture._verified_goal = (time.monotonic() - 11.0, (1.0, 2.0))
    assert not fixture._consume_verified_goal((1.0, 2.0))


def test_initial_pose_logs_only_after_successful_path_clear():
    events = []
    logger = SimpleNamespace(info=lambda msg: events.append(('info', msg)),
                             error=lambda msg: events.append(('error', msg)))
    fixture = SimpleNamespace(
        _point_in_map=lambda x, y, frame: (x, y), map_frame='map',
        get_logger=lambda: logger, current_goal=(3, 4), current_path=[(3, 4)],
        current_goal_verified=True, _verified_goal=(3, 4), _replan_pending=True,
        _publish_empty_path=lambda: events.append(('clear', None)))
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0))))
    RvizThetaStarNode._on_initial_pose(fixture, message)
    assert fixture.current_goal is None and fixture.current_path == []
    assert not fixture.current_goal_verified and fixture._verified_goal is None
    assert not fixture._replan_pending
    assert [event for event, _ in events] == ['clear', 'info']
    assert '(1.000, 2.000)' in events[-1][1]

    def fail_transform(*args):
        raise RuntimeError('missing transform')

    events.clear()
    fixture._point_in_map = fail_transform
    RvizThetaStarNode._on_initial_pose(fixture, message)
    assert fixture.initial_pose is None
    assert [event for event, _ in events] == ['error']

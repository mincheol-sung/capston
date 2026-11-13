import math

from dra_mppi.path_handler import (
    build_heading_alignment_reference,
    build_terminal_pose_reference,
    GoalCompletionLatch,
    heading_error,
)
import numpy as np
import pytest


def test_goal_completion_stays_latched_after_robot_leaves_tolerance():
    latch = GoalCompletionLatch()
    path = np.array([[0.0, 0.0], [1.0, 0.0]])

    assert not latch.complete_if_reached(path, [0.80, 0.0, 0.1], 0.15)
    assert latch.complete_if_reached(path, [0.90, 0.0, 0.2], 0.15)
    assert latch.completed
    assert np.array_equal(latch.hold_state, [0.90, 0.0, 0.2])

    assert not latch.complete_if_reached(path, [1.30, 0.0, 0.4], 0.15)
    assert latch.completed
    assert np.array_equal(latch.hold_state, [0.90, 0.0, 0.2])


def test_new_goal_resets_goal_completion_latch():
    latch = GoalCompletionLatch()
    old_path = np.array([[0.0, 0.0], [1.0, 0.0]])
    new_path = np.array([[1.0, 0.0], [2.0, 0.0]])

    assert latch.complete_if_reached(old_path, [1.0, 0.0, 0.0], 0.15)
    latch.reset()

    assert not latch.completed
    assert latch.hold_state is None
    assert not latch.complete_if_reached(new_path, [1.0, 0.0, 0.0], 0.15)


def test_goal_completion_requires_terminal_heading_when_requested():
    latch = GoalCompletionLatch()
    path = np.array([[0.0, 0.0], [1.0, 0.0]])

    assert not latch.complete_if_reached(
        path,
        [1.0, 0.0, math.radians(30.0)],
        0.15,
        goal_yaw=math.radians(90.0),
        yaw_tolerance=math.radians(5.0),
    )
    assert not latch.completed
    assert latch.complete_if_reached(
        path,
        [1.0, 0.0, math.radians(86.0)],
        0.15,
        goal_yaw=math.radians(90.0),
        yaw_tolerance=math.radians(5.0),
    )


def test_heading_error_uses_shortest_direction_across_pi():
    error = heading_error(math.radians(-179.0), math.radians(179.0))
    assert error == pytest.approx(math.radians(2.0))


def test_heading_alignment_is_in_place_bounded_and_reaches_goal():
    sequence = build_heading_alignment_reference(
        [1.0, 2.0, 0.0],
        goal_yaw=math.pi / 2.0,
        dt=0.2,
        horizon_steps=20,
        max_yaw_rate=0.6,
        yaw_acceleration_limit=2.0,
        gain=1.5,
    )

    assert sequence.shape == (20, 6)
    assert np.allclose(sequence[:, :2], [1.0, 2.0])
    assert np.allclose(sequence[:, 3:5], 0.0)
    assert np.max(np.abs(sequence[:, 5])) <= 0.6 + 1e-12
    rate_steps = np.diff(np.concatenate(([0.0], sequence[:, 5])))
    assert np.max(np.abs(rate_steps)) <= 2.0 * 0.2 + 1e-12
    assert abs(
        heading_error(math.pi / 2.0, sequence[-1, 2])
    ) < math.radians(5.0)


def test_heading_alignment_takes_short_path_across_pi():
    sequence = build_heading_alignment_reference(
        [0.0, 0.0, math.radians(179.0)],
        goal_yaw=math.radians(-179.0),
        dt=0.2,
        horizon_steps=5,
        max_yaw_rate=0.6,
        yaw_acceleration_limit=2.0,
        gain=1.5,
    )

    assert sequence[0, 5] > 0.0
    assert abs(
        heading_error(math.radians(-179.0), sequence[-1, 2])
    ) < math.radians(0.5)


def test_terminal_pose_reference_corrects_xy_and_yaw_with_rate_limits():
    sequence = build_terminal_pose_reference(
        current_state=[0.14, 0.0, 0.0],
        goal_state=[0.0, 0.0, math.pi / 2.0],
        current_control=[0.30, 0.0, 0.0],
        dt=0.2,
        horizon_steps=20,
        max_linear_speed=0.10,
        linear_gain=1.0,
        max_yaw_rate=0.60,
        yaw_gain=1.5,
        acceleration_limit=[2.25, 2.25, 6.75],
    )

    assert sequence.shape == (20, 6)
    assert np.linalg.norm(sequence[-1, :2]) < 0.02
    assert abs(heading_error(math.pi / 2.0, sequence[-1, 2])) < math.radians(5.0)
    assert sequence[0, 0] < 0.14
    assert np.max(np.linalg.norm(sequence[:, 3:5], axis=1)) <= 0.10 + 1e-12
    assert np.max(np.abs(sequence[:, 5])) <= 0.60 + 1e-12

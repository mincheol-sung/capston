"""Regression tests for the 2026-10-20 shared cost and safety contract."""

import numpy as np
import pytest

from dra_mppi.common_cost import common_costs, project_path
from dra_mppi.dynamic_risk import (
    DynamicRiskConfig, MonteCarloJointCollisionProbability, PredictionSet,
)
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiConfig, MppiController


def config(**kwargs):
    return MppiConfig(horizon_steps=4, num_rollouts=20, guide_num_rollouts=20,
                      dynamic_risk=DynamicRiskConfig(mode='off'), **kwargs)


def test_crossing_projection_stays_on_branch_and_permits_reverse():
    path = np.array([[-2., -2.], [2., 2.], [-2., 2.], [2., -2.]])
    positions = np.array([[[-.1, -.1], [0., 0.], [.1, .1], [0., 0.], [-.1, -.1]]])
    _, s, _, _, _ = project_path(positions, path, .2, .3)
    assert np.all(s < np.sqrt(32.))
    assert s[0, 2] > s[0, 1]
    assert s[0, 4] < s[0, 3]


def test_heading_is_path_direction_not_line_of_sight_and_wraps():
    cfg = config(heading_weight=1.)
    state = np.array([0., 2., -np.pi+.01])
    path = np.array([[1., 0.], [0., 0.], [0., 0.], [-2., 0.]])
    controls = np.zeros((1, 4, 3))
    trajectory = rollout(state, controls, cfg.dt)
    costs = common_costs(cfg, state, path, controls, trajectory, np.zeros(3))
    assert costs[1][0] == pytest.approx(.01**2)
    _, _, heading, _, _ = project_path(
        np.array([[[-2., 1.], [-3., 1.]]]), path, .2, .5)
    assert np.allclose(np.abs(heading), np.pi)
    single = project_path(np.zeros((1, 2, 2)), np.zeros((3, 2)), .2, .5)
    assert np.all(np.isfinite(single[2]))


def test_first_input_rate_uses_measured_body_twist():
    cfg = config(input_delta_weight=(1., 0., 0.))
    controls = np.zeros((1, 4, 3))
    path = np.array([[0., 0.], [3., 0.]])
    states = rollout(np.zeros(3), controls, cfg.dt)
    cost = common_costs(cfg, np.zeros(3), path, controls, states, [.2, 0., 0.])
    assert cost[3][0] == pytest.approx(.2**2/4)


def test_far_goal_has_exact_squared_distance_and_no_terminal_reward():
    cfg = config(tracking_path_weight=2., progress_weight=0.)
    controls = np.zeros((1, 4, 3))
    state = np.array([0., 1., 0.])
    states = rollout(state, controls, cfg.dt)
    path = np.array([[0., 0.], [100., 0.]])
    assert common_costs(cfg, state, path, controls, states, np.zeros(3))[0][0] == 2.


def test_goal_speed_reference_decelerates_to_zero():
    cfg = config()
    controls = np.zeros((1, 4, 3))
    state = np.zeros(3)
    path = np.array([[-1., 0.], [0., 0.]])
    assert common_costs(cfg, state, path, controls, rollout(state, controls, cfg.dt),
                        np.zeros(3))[2][0] == 0.


def test_common_cost_identical_for_baseline_guide_and_batch_subset():
    path = np.array([[0., 0.], [3., 0.]])
    controls = np.random.default_rng(2).normal(0., .1, (5, 4, 3))
    poses = rollout(np.zeros(3), controls, .1)
    baseline = MppiController(config())
    guide = MppiController(config(strategy='guide_conditioned'))
    a, _ = baseline._evaluate_rollouts(np.zeros(3), path, controls, poses, None)
    b, _ = guide._evaluate_rollouts(np.zeros(3), path, controls, poses, None)
    one, _ = guide._evaluate_rollouts(np.zeros(3), path, controls[2:3], poses[2:3], None)
    assert np.allclose(a.total, b.total)
    assert one.total[0] == pytest.approx(b.total[2])
    guide.close()


@pytest.mark.parametrize('enabled', [False, True])
def test_static_switch_masks_collision_and_all_invalid_hold(enabled):
    c = MppiController(config(strategy='guide_conditioned', static_cost_enabled=enabled))
    field = DistanceField.from_occupancy(
        np.full((60, 60), 100), GridMetadata(60, 60, .1, -3., -3., 0.))
    result = c.compute(np.zeros(3), np.array([[0., 0.], [2., 0.]]),
                       distance_field=field)
    if enabled:
        assert np.all(np.isinf(result.rollout_costs))
        assert result.safety_status == 'no_safe_candidate_hold'
        assert result.selected_mode == 'stop'
        assert result.final_static_collisions[-1]
    else:
        assert np.all(result.cost_terms.static == 0.)
        assert result.safety_status == 'ok'
    c.close()


def test_swept_static_collision_and_map_boundary():
    c = MppiController(config(static_cost_enabled=True, static_cost_robot_radius=.1))
    grid = np.zeros((60, 60))
    grid[:, 35] = 100
    field = DistanceField.from_occupancy(grid, GridMetadata(60, 60, .1, -3., -3., 0.))
    poses = np.array([[[1., 0., 0.]], [[-2.95, 0., 0.]]])
    cost, collision = c._static_obstacle_cost(np.zeros(3), poses, field, None)
    assert collision.all() and np.isinf(cost).all()


def test_average_of_two_safe_rollouts_is_rejected_after_rerollout(monkeypatch):
    cfg = MppiConfig(
        strategy='guide_conditioned', horizon_steps=6, guide_num_rollouts=4,
        guide_angles_deg=(0.,), dt=.2, acceleration_limit=(100.,)*3,
        control_min=(-1., -1., -3.), control_max=(1., 1., 3.), speed_limit_l1=2.,
        static_cost_enabled=True, static_cost_robot_radius=.03,
        dynamic_risk=DynamicRiskConfig(mode='off'))
    c = MppiController(cfg)
    left = np.array([[.6, .6, 0.]]*3+[[.6, -.6, 0.]]*3)
    right = left.copy()
    right[:, 1] *= -1
    monkeypatch.setattr(c._guide_optimizer, 'anchors', lambda *_: left[None].copy())
    c._guide_optimizer.noise[0][:] = right-left
    monkeypatch.setattr(c, '_importance_weights', lambda cost, temp: (
        np.ones(4, bool), 0., 0., np.ones(4)/4., 4.))
    grid = np.zeros((200, 200))
    grid[99:101, 117:123] = 100
    field = DistanceField.from_occupancy(grid, GridMetadata(200, 200, .02, -2., -2., 0.))
    result = c.compute(np.zeros(3), np.array([[0., 0.], [2., 0.]]), distance_field=field)
    assert np.isfinite(result.cost_terms.static).all()
    assert result.final_static_collisions[0]
    assert result.selected_mode == 'stop'
    c.close()


def test_shared_rectangle_normalization_and_empty_circle_is_not_safe(monkeypatch):
    pred = PredictionSet(np.zeros((1, 1, 2)), np.eye(2)[None, None], np.array([.1]))
    estimator = MonteCarloJointCollisionProbability(num_samples=4)
    monkeypatch.setattr(estimator, '_draw_samples', lambda lo, hi, count:
                        np.array([[0., 0.], [.9, .9], [-.9, .9], [.9, -.9]]))
    monkeypatch.setattr(estimator, '_mixture_densities', lambda *args:
                        np.full((1, 4), .1))
    p = estimator.evaluate(np.zeros((1, 1, 2)), pred, 1.)
    assert p[0, 0] == pytest.approx(4./4.*.1)  # one hit, area(B)=4
    monkeypatch.setattr(estimator, '_draw_samples', lambda lo, hi, count:
                        np.full((4, 2), .9))
    assert estimator.evaluate(np.zeros((1, 1, 2)), pred, 1.)[0, 0] == 1.
    assert estimator.last_diagnostics.empty_circle_count == 1


@pytest.mark.parametrize('cov', [np.diag([-1., 1.]), np.array([[1., 2.], [0., 1.]])])
def test_invalid_covariance_is_rejected(cov):
    with pytest.raises(ValueError, match='covariance'):
        PredictionSet(np.zeros((1, 1, 2)), cov[None, None], np.array([.1]))


def test_all_strategies_share_joint_acceleration_and_wheel_constraints():
    c = MppiController(config(acceleration_limit=(.3, .5, .7), speed_limit_l1=.3))
    raw = np.random.default_rng(2).normal(size=(100, 4, 3))
    current = np.array([.1, -.1, .1])
    controls = c._constrain_controls(raw, current)
    previous = np.broadcast_to(current, (100, 1, 3))
    assert np.all(np.abs(np.diff(np.concatenate((previous, controls), axis=1), axis=1))
                  <= np.array(c.config.acceleration_limit)*c.config.dt+1e-12)
    load = np.abs(controls[..., :2]).sum(axis=-1)+c.config.yaw_lever*np.abs(controls[..., 2])
    assert np.all(load <= .3+1e-12)

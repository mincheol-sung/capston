import pytest
import numpy as np

import dra_mppi.mppi_controller as mppi_controller_module
from dra_mppi.dynamic_risk import (
    DynamicRiskConfig,
    RiskEvaluation,
)
from dra_mppi.mecanum_model import rollout
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import (
    MppiConfig,
    MppiController,
)


def test_controller_returns_updated_sequence_and_paper_cost_terms():
    config = MppiConfig(
        dt=0.1,
        horizon_steps=15,
        num_rollouts=256,
        seed=4,
        dynamic_risk=DynamicRiskConfig(mode='off'),
    )
    controller = MppiController(config)
    path = np.column_stack((np.linspace(0.0, 1.2, 25), np.zeros(25)))

    result = controller.compute(
        state=np.array([0.0, 0.0, 0.0]),
        reference_path=path,
        current_control=np.zeros(3),
    )

    assert result.reference_state.shape == (3,)
    assert result.control.shape == (3,)
    assert result.control_sequence.shape == (15, 3)
    assert result.reference_sequence.shape == (15, 6)
    assert result.trajectory.shape == (15, 3)
    assert result.sampled_controls.shape == (256, 15, 3)
    assert result.sampled_rollouts.shape == (40, 15, 3)
    assert result.sampled_rollout_indices.shape == (40,)
    assert result.rollout_costs.shape == (256,)
    assert result.weights.shape == (256,)
    assert result.collision_probabilities.shape == (256, 15)
    assert np.allclose(result.sampled_controls[1], 0.0)
    assert np.allclose(result.reference_sequence[:, :3], result.trajectory)
    assert np.allclose(result.reference_sequence[:, 3:], result.control_sequence)
    assert np.allclose(
        result.sampled_rollouts,
        rollout(
            np.zeros(3),
            result.sampled_controls[result.sampled_rollout_indices],
            config.dt,
        ),
    )
    assert np.allclose(result.cost_terms.total, result.rollout_costs)
    assert np.isclose(np.sum(result.weights), 1.0)
    assert 1.0 <= result.effective_sample_size <= 256.0
    assert result.temperature > 0.0
    assert result.next_temperature > 0.0
    assert result.weight_normalizer >= 1.0
    assert result.minimum_cost <= result.maximum_cost
    assert 0.0 <= result.minimum_weight <= result.maximum_weight <= 1.0
    assert np.all(np.isfinite(result.reference_state))
    assert np.all(np.isfinite(result.control))
    assert result.control[0] > 0.0
    assert result.reference_state[0] > 0.0
    assert result.compute_time_ms > 0.0


def test_stopping_control_decelerates_from_measured_velocity():
    config = MppiConfig(
        dt=0.2,
        horizon_steps=8,
        num_rollouts=16,
        control_min=(-1.0, -1.0, -3.0),
        control_max=(1.0, 1.0, 3.0),
        acceleration_limit=(0.5, 0.25, 1.0),
        speed_limit_l1=10.0,
        dynamic_risk=DynamicRiskConfig(mode="off"),
    )
    controller = MppiController(config)
    current = np.array([0.6, -0.3, 1.2])

    stopping = controller._make_stopping_control(current)
    step_delta = np.asarray(config.acceleration_limit) * config.dt
    steps = np.arange(1, config.horizon_steps + 1)[:, None]
    expected = (
        np.sign(current)
        * np.maximum(np.abs(current) - steps * step_delta, 0.0)
    )

    assert np.allclose(stopping, expected)
    assert np.all(np.abs(np.diff(np.vstack((current, stopping)), axis=0))
                  <= step_delta + 1e-12)
    assert np.all(stopping[-2:] == 0.0)

    trajectory = rollout(np.zeros(3), stopping, config.dt)
    assert np.linalg.norm(trajectory[-1, :2]) > 0.0
    assert np.allclose(trajectory[-1], trajectory[-2])

    result = controller.compute(
        np.zeros(3),
        np.array([[0.0, 0.0], [2.0, 0.0]]),
        current,
    )
    assert np.allclose(result.sampled_controls[1], stopping)


def test_acceleration_limit_applies_to_first_command():
    config = MppiConfig(
        dt=0.1,
        horizon_steps=8,
        num_rollouts=64,
        acceleration_limit=(0.5, 0.4, 1.0),
        dynamic_risk=DynamicRiskConfig(mode='off'),
    )
    controller = MppiController(config)
    path = np.array([[0.0, 0.0], [2.0, 0.0]])
    result = controller.compute([0.0, 0.0, 0.0], path, np.zeros(3))

    assert abs(result.control[0]) <= 0.05 + 1e-9
    assert abs(result.control[1]) <= 0.04 + 1e-9
    assert abs(result.control[2]) <= 0.10 + 1e-9


def test_reference_beta_update_targets_official_eta_interval():
    path = np.array([[0.0, 0.0], [2.0, 0.0]])
    common = dict(
        horizon_steps=8,
        num_rollouts=64,
        dynamic_risk=DynamicRiskConfig(mode='off'),
    )

    collapsed = MppiController(MppiConfig(temperature=1e-3, **common)).compute(
        np.zeros(3), path, np.zeros(3)
    )
    assert collapsed.weight_normalizer < 5.0
    assert np.isclose(
        collapsed.next_temperature,
        collapsed.temperature * 1.2,
    )

    flat = MppiController(MppiConfig(temperature=1e9, **common)).compute(
        np.zeros(3), path, np.zeros(3)
    )
    assert flat.weight_normalizer > 10.0
    assert np.isclose(flat.next_temperature, flat.temperature * 0.9)


def test_fixed_temperature_mode_remains_available():
    controller = MppiController(
        MppiConfig(
            temperature=2.5,
            adaptive_temperature=False,
            horizon_steps=8,
            num_rollouts=64,
            dynamic_risk=DynamicRiskConfig(mode='off'),
        )
    )
    result = controller.compute(
        np.zeros(3),
        np.array([[0.0, 0.0], [2.0, 0.0]]),
        np.zeros(3),
    )

    assert result.temperature == 2.5
    assert result.next_temperature == 2.5


def test_nonfinite_rollout_gets_zero_weight_without_poisoning_result(monkeypatch):
    rollout_count = 8
    horizon_steps = 3

    def risk_with_one_nonfinite_cost(trajectories, *_args, **_kwargs):
        costs = np.zeros(len(trajectories))
        costs[0] = np.inf
        return RiskEvaluation(
            cost=costs,
            collision_probability=np.zeros((len(trajectories), horizon_steps)),
        )

    monkeypatch.setattr(
        mppi_controller_module,
        'dynamic_risk_cost',
        risk_with_one_nonfinite_cost,
    )
    controller = MppiController(
        MppiConfig(
            horizon_steps=horizon_steps,
            num_rollouts=rollout_count,
            dynamic_risk=DynamicRiskConfig(mode='off'),
        )
    )
    result = controller.compute(
        np.zeros(3),
        np.array([[0.0, 0.0], [2.0, 0.0]]),
        np.zeros(3),
    )

    assert result.weights[0] == 0.0
    assert np.isfinite(result.cost)


def test_standard_result_reports_recomputed_representative_cost():
    controller = MppiController(
        MppiConfig(
            horizon_steps=5,
            num_rollouts=16,
            dynamic_risk=DynamicRiskConfig(mode='off'),
        )
    )
    path = np.column_stack((np.linspace(0.0, 2.0, 20), np.zeros(20)))
    result = controller.compute(np.zeros(3), path, np.zeros(3))
    terms, _ = controller._evaluate_rollouts(
        np.zeros(3), path, result.control_sequence[None],
        result.trajectory[None], None)
    assert np.isclose(result.cost, terms.total[0])


def _path_guide_controller(horizon_steps=20, num_rollouts=32):
    return MppiController(
        MppiConfig(
            dt=0.2,
            horizon_steps=horizon_steps,
            num_rollouts=num_rollouts,
            desired_speed=0.75,
            control_min=(-0.375, -0.5625, -3.0),
            control_max=(0.78, 0.5625, 3.0),
            acceleration_limit=(2.25, 2.25, 6.75),
            yaw_lever=0.19835,
            speed_limit_l1=0.78,
            dynamic_risk=DynamicRiskConfig(mode='off'),
        )
    )


def test_candidate_two_is_a_constrained_forward_path_guide():
    controller = _path_guide_controller()
    path = np.array([[0.0, 0.0], [4.0, 0.0]])

    result = controller.compute(np.zeros(3), path, np.zeros(3))
    guide = result.sampled_controls[2]
    previous = np.vstack((np.zeros((1, 3)), guide[:-1]))
    max_delta = np.asarray(controller.config.acceleration_limit) * (
        controller.config.dt
    )

    assert guide[0, 0] > 0.0
    assert np.allclose(guide[:, 1:], 0.0)
    assert np.all(np.abs(guide - previous) <= max_delta + 1e-9)
    assert np.all(wheel_load(guide, controller.config) <= 0.78 + 1e-9)


def test_path_guide_uses_holonomic_strafe_and_aligns_yaw():
    controller = _path_guide_controller()
    path = np.array([[0.0, 0.0], [0.0, 4.0]])

    guide = controller._make_path_guide(np.zeros(3), path, np.zeros(3))

    assert guide[0, 1] > 0.0
    assert guide[0, 2] > 0.0
    assert np.any(guide[:, 0] > 0.0)
    assert np.all(wheel_load(guide, controller.config) <= 0.78 + 1e-9)


def test_full_guide_path_continues_beyond_short_tracking_window():
    controller = _path_guide_controller(horizon_steps=8)
    local_path = np.array([[0.0, 0.0], [0.2, 0.0]])
    global_guide = np.array([[0.0, 0.0], [3.0, 0.0]])

    result = controller.compute(
        np.zeros(3),
        local_path,
        np.zeros(3),
        guide_path=global_guide,
    )

    assert result.sampled_controls[2, -1, 0] > 0.0


def test_path_guide_brakes_to_stop_at_global_goal():
    controller = _path_guide_controller()
    path = np.array([[0.0, 0.0], [0.3, 0.0]])

    guide = controller._make_path_guide(np.zeros(3), path, np.zeros(3))

    assert np.any(guide[:, 0] > 0.0)
    assert np.allclose(guide[-5:], 0.0)


def test_path_guide_receives_the_same_risk_penalty_as_other_candidates(
    monkeypatch,
):
    rollout_count = 16
    horizon_steps = 5

    def risk_rejecting_guide(trajectories, *_args, **_kwargs):
        costs = np.zeros(len(trajectories))
        if len(costs) > 2:
            costs[2] = 1e9
        return RiskEvaluation(
            cost=costs,
            collision_probability=np.zeros((len(trajectories), horizon_steps)),
        )

    monkeypatch.setattr(
        mppi_controller_module,
        'dynamic_risk_cost',
        risk_rejecting_guide,
    )
    controller = _path_guide_controller(horizon_steps, rollout_count)
    path = np.array([[0.0, 0.0], [2.0, 0.0]])

    result = controller.compute(np.zeros(3), path, np.zeros(3))

    assert result.weights[2] == 0.0


def wheel_load(controls, config):
    """|vx| + |vy| + k|yaw_rate|, the largest mecanum wheel speed times r."""
    return (
        np.abs(controls[..., 0])
        + np.abs(controls[..., 1])
        + config.yaw_lever * np.abs(controls[..., 2])
    )


def test_every_control_stays_inside_the_wheel_speed_polytope():
    # The box corner alone is 175 % of the L1 budget, so an axis-only clip
    # would let candidates through that the base driver has to scale down.
    config = MppiConfig(
        dt=0.2,
        horizon_steps=10,
        num_rollouts=128,
        noise_std=(0.6, 0.6, 2.0),
        control_min=(-0.375, -0.5625, -3.0),
        control_max=(0.78, 0.5625, 3.0),
        acceleration_limit=(2.25, 2.25, 6.75),
        yaw_lever=0.19835,
        speed_limit_l1=0.78,
        dynamic_risk=DynamicRiskConfig(mode='off'),
    )
    assert wheel_load(np.array(config.control_max), config) > config.speed_limit_l1
    controller = MppiController(config)
    path = np.column_stack((np.linspace(0.0, 4.0, 20), np.linspace(0.0, 3.0, 20)))

    control = np.zeros(3)
    for _ in range(5):
        result = controller.compute([0.0, 0.0, 0.0], path, control)
        control = result.control
        assert np.all(
            wheel_load(result.sampled_controls, config)
            <= config.speed_limit_l1 + 1e-9
        )
        assert np.all(
            wheel_load(result.control_sequence, config)
            <= config.speed_limit_l1 + 1e-9
        )
        # Projection only shrinks toward zero, so the box still holds.
        assert np.all(result.control_sequence >= np.array(config.control_min) - 1e-9)
        assert np.all(result.control_sequence <= np.array(config.control_max) + 1e-9)


def test_static_cost_toggle_and_hard_collision_penalty():
    metadata = GridMetadata(80, 80, 0.05, -2.0, -2.0, 0.0)
    occupancy = np.zeros((80, 80), dtype=np.int16)
    # A vertical obstacle centred at x=0.5.
    occupancy[:, 50] = 100
    field = DistanceField.from_occupancy(occupancy.ravel(), metadata)
    trajectories = np.array([
        [[0.1, 0.0, 0.0], [0.5, 0.0, 0.0]],
        [[0.0, 0.6, 0.0], [0.0, 1.0, 0.0]],
    ])

    disabled = MppiController(MppiConfig(
        static_cost_enabled=False,
        dynamic_risk=DynamicRiskConfig(mode='off'),
    ))
    cost, collision = disabled._static_obstacle_cost(
        np.zeros(3), trajectories, None, None)
    assert np.all(cost == 0.0)
    assert not collision.any()

    enabled = MppiController(MppiConfig(
        static_cost_enabled=True,
        static_cost_robot_radius=0.10,
        static_cost_soft_weight=2.0,
        static_cost_decay=0.10,
        dynamic_risk=DynamicRiskConfig(mode='off'),
    ))
    cost, collision = enabled._static_obstacle_cost(
        np.zeros(3), trajectories, field, None)

    assert collision.tolist() == [True, False]
    assert cost[0] >= 100000.0
    assert cost[1] == 0.0  # outside the hinge's d_safe band


def test_path_heading_uses_distance_lookahead_across_a_corner():
    controller = MppiController(
        MppiConfig(
            horizon_steps=2,
            num_rollouts=8,
            heading_lookahead=0.20,
            dynamic_risk=DynamicRiskConfig(mode='off'),
        )
    )
    path = np.array([[0.0, 0.0], [0.10, 0.0], [0.10, 0.40]])

    headings = controller._path_headings(path)

    # From the first point, 0.20 m along the path is (0.10, 0.10), so the
    # smoothed chord heading is 45 degrees rather than the first segment's 0.
    assert np.isclose(headings[0], np.pi / 4.0)
    assert np.isclose(headings[1], np.pi / 2.0)
    assert np.isclose(headings[-1], np.pi / 2.0)


def test_path_geometry_projects_onto_continuous_segments():
    controller = MppiController(
        MppiConfig(
            horizon_steps=3,
            num_rollouts=8,
            dynamic_risk=DynamicRiskConfig(mode='off'),
        )
    )
    path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    trajectories = np.array([[
        [0.5, 0.0, 0.0],
        [1.0, 0.5, np.pi / 2.0],
        [0.5, 0.2, 0.0],
    ]])

    squared, indices, closest, headings = controller._nearest_path_geometry(
        trajectories,
        path,
    )

    assert np.allclose(squared, [[0.0, 0.0, 0.04]])
    assert np.array_equal(indices, [[0, 1, 0]])
    assert np.allclose(closest, [[[0.5, 0.0], [1.0, 0.5], [0.5, 0.0]]])
    assert np.allclose(headings, [[0.0, np.pi / 2.0, 0.0]])


def test_polyline_projection_ignores_repeated_zero_length_segments():
    controller = MppiController(
        MppiConfig(
            horizon_steps=2,
            num_rollouts=8,
            dynamic_risk=DynamicRiskConfig(mode='off'),
        )
    )
    path = np.array([
        [-1.0, 0.0],
        [-1.0, 0.0],
        [1.0, 0.0],
        [1.0, 1.0],
    ])
    trajectories = np.array([[
        [-1.0, 0.0, 0.0],
        [0.0, 0.5, 0.0],
    ]])

    nearest_squared, nearest_indices, closest, _ = (
        controller._nearest_path_geometry(
            trajectories,
            path,
        )
    )

    assert np.array_equal(nearest_indices, [[1, 1]])
    assert np.allclose(nearest_squared, [[0.0, 0.25]])
    assert np.allclose(closest, [[[-1.0, 0.0], [0.0, 0.0]]])


def test_removed_strategy_is_rejected():
    with pytest.raises(ValueError, match='unknown MPPI strategy'):
        MppiController(MppiConfig(strategy='mode_conditioned'))


def test_guide_budget_does_not_depend_on_standard_sampler_settings():
    controller = MppiController(MppiConfig(
        strategy='guide_conditioned', horizon_steps=4,
        guide_num_rollouts=20, num_rollouts=0, noise_std=(-1., -1., -1.),
        dynamic_risk=DynamicRiskConfig(mode='off')))
    try:
        result = controller.compute(np.zeros(3), np.array([[0., 0.], [2., 0.]]))
        assert result.sampled_controls.shape == (20, 4, 3)
        assert np.isfinite(result.control).all()
    finally:
        controller.close()

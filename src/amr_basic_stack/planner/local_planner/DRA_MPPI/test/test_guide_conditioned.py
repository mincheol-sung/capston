"""Behavioral checks for the robot-relative guide ensemble."""

from dataclasses import replace

import numpy as np
import pytest

from dra_mppi.dynamic_risk import DynamicRiskConfig, PredictionSet, RiskEvaluation
from dra_mppi.geometry import transform_states, transform_points
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiConfig, MppiController, MppiCostTerms


def controller(**overrides):
    values = dict(
        strategy='guide_conditioned', dt=0.2, horizon_steps=10,
        num_rollouts=321, guide_num_rollouts=500, desired_speed=0.75,
        control_min=(-0.375, -0.5625, -3.0), control_max=(0.78, 0.5625, 3.0),
        acceleration_limit=(2.25, 2.25, 6.75), speed_limit_l1=0.78,
        heading_weight=3.0, dynamic_risk=DynamicRiskConfig(
            mode='off', robot_radius=0.2),
    )
    values.update(overrides)
    return MppiController(MppiConfig(**values))


def field(rectangles=(), unknown=False, transform=(0, 0, 0)):
    resolution = 0.025
    grid = np.zeros((240, 240), dtype=np.int16)
    centers = -3 + (np.arange(240) + 0.5) * resolution
    xx, yy = np.meshgrid(centers, centers)
    for x0, x1, y0, y1 in rectangles:
        grid[(xx >= x0) & (xx <= x1) & (yy >= y0) & (yy <= y1)] = -1 if unknown else 100
    origin = transform_points(np.array([-3., -3.]), transform)
    return DistanceField.from_occupancy(
        grid, GridMetadata(240, 240, resolution, *origin, transform[2]))


PATH = np.column_stack((np.linspace(0., 2., 21), np.zeros(21)))


def compute(c, distance_field=None, **kwargs):
    return c.compute(np.zeros(3), PATH, np.zeros(3),
                     distance_field=field() if distance_field is None else distance_field,
                     **kwargs)


def test_open_path_prefers_forward_and_weights_stay_inside_each_guide():
    c = controller()
    for _ in range(5):
        r = compute(c)
        assert r.selected_mode == 'guide_+0'
        assert r.guide_active_mask.all()
        assert r.guide_rollout_counts.tolist() == [100] * 5
        assert r.sampled_controls.shape == (500, 10, 3)
        assert r.mode_control_sequences.shape == (6, 10, 3)
        for g in range(5):
            mask = r.control_mode_ids == g
            assert r.weights[mask].sum() == pytest.approx(1.)
            average = np.sum(r.weights[mask, None, None] * r.sampled_controls[mask], axis=0)
            assert np.allclose(r.mode_control_sequences[g], average)
        index = r.candidate_names.index(r.selected_mode)
        assert np.array_equal(r.control_sequence, r.mode_control_sequences[index])
        assert np.array_equal(r.trajectory, r.mode_trajectories[index])
        assert r.cost == r.mode_final_costs[index]
        assert np.all(r.mode_control_sequences[-1] == 0.)
        assert np.allclose(r.cost_terms.total, r.rollout_costs)


def test_stop_candidate_brakes_from_current_velocity():
    c = controller(acceleration_limit=(0.5, 0.5, 1.0))
    current = np.array([0.3, -0.1, 0.5])
    r = c.compute(
        np.zeros(3),
        PATH,
        current,
        distance_field=field(),
    )
    stopping = c._make_stopping_control(current)

    assert np.allclose(r.mode_control_sequences[-1], stopping)
    assert np.allclose(
        r.mode_trajectories[-1],
        rollout(np.zeros(3), stopping, c.config.dt),
    )
    assert np.linalg.norm(r.mode_trajectories[-1, -1, :2]) > 0.0


def test_anchor_angles_lengths_and_pure_strafe_from_rest():
    c = controller()
    anchors = c._guide_optimizer.anchors(np.zeros(3), PATH, np.zeros(3))
    trajectories = rollout(np.zeros(3), anchors, c.config.dt)
    angles = np.rad2deg(np.arctan2(trajectories[:, -1, 1], trajectories[:, -1, 0]))
    assert np.allclose(angles, [0, 45, 90, -45, -90])
    assert np.allclose(trajectories[..., 2], 0.)
    lengths = np.linalg.norm(trajectories[:, -1, :2], axis=1)
    assert lengths[0] == pytest.approx(1.44)
    assert lengths[2] == pytest.approx(1.1025)
    assert lengths[2] < 0.5625 * 2.
    assert np.linalg.norm(anchors[1, -1, :2]) == pytest.approx(0.78 / np.sqrt(2))
    assert np.allclose(anchors[2, :, 0], 0.)
    moving = c._guide_optimizer.anchors(np.zeros(3), PATH, np.array([0., .5, 0.]))
    assert rollout(np.zeros(3), moving[2], .2)[-1, 1] > trajectories[2, -1, 1]


def test_robot_relative_rays_with_off_path_heading_are_frame_equivariant():
    c = controller(guide_noise_std=(0., 0., 0.))
    state = np.array([0., -.5, np.pi / 3])
    anchors = c._guide_optimizer.anchors(state, PATH, np.zeros(3))
    paths = rollout(state, anchors, .2)
    # Body yaw rotates toward the path while lateral translation remains available.
    assert np.all(np.abs(paths[:, -1, 2]) < abs(state[2]))
    shift = (1.1, -.7, .4)
    other = c._guide_optimizer.anchors(
        transform_states(state, shift), transform_points(PATH, shift), np.zeros(3))
    assert np.allclose(anchors, other)
    # Without yaw disagreement, entry yaw rotates the complete five-ray fan.
    rotated_path = transform_points(PATH, (0., 0., np.pi / 2))
    same = c._guide_optimizer.anchors(np.array([0., 0., np.pi / 2]), rotated_path, np.zeros(3))
    rays = rollout(np.array([0., 0., np.pi / 2]), same, .2)[:, -1, :2]
    expected = np.deg2rad(np.array([90., 135., 180., 45., 0.]))
    assert np.allclose(rays / np.linalg.norm(rays, axis=1)[:, None],
                       np.column_stack((np.cos(expected), np.sin(expected))))


@pytest.mark.parametrize('active_count', [1, 2, 3, 4, 5])
def test_budget_reallocation_keeps_500_and_rotates_remainders(active_count):
    optimizer = controller()._guide_optimizer
    active = np.arange(5) < active_count
    allocations = np.array([optimizer.allocate(active) for _ in range(active_count)])
    assert np.all(allocations.sum(axis=1) == 500)
    assert np.all(allocations[:, ~active] == 0)
    assert np.ptp(allocations[:, active], axis=1).max() <= 1
    assert np.ptp(allocations[:, active].sum(axis=0)) == 0


def test_static_map_does_not_prune_guides_or_rollouts():
    obstacles = field([(0.55, .75, -.8, .8)])
    r = compute(controller(), obstacles)
    assert r.guide_active_mask.all()
    assert r.guide_rollout_counts.tolist() == [100] * 5
    assert not r.final_static_collisions.any()
    assert r.guide_rejection_reasons == ('',) * 5
    sampled = rollout(np.zeros(3), r.sampled_controls, .2)
    blocked = obstacles.trajectory_collisions(sampled[..., :2], [0., 0.], .2, .02)
    assert np.any(blocked)
    assert np.all(np.isfinite(r.rollout_costs[blocked]))


def test_enabled_static_cost_penalizes_rollouts_without_pruning_guides():
    obstacles = field([(0.55, .75, -.8, .8)])
    c = controller(
        static_cost_enabled=True,
        static_cost_robot_radius=.2,
        static_cost_soft_weight=8.,
        static_cost_decay=.1,
    )

    r = compute(c, obstacles)
    sampled = rollout(np.zeros(3), r.sampled_controls, .2)
    clearance = obstacles.trajectory_clearances(
        sampled[..., :2], np.zeros(2))
    blocked = np.any(clearance <= .2, axis=1)

    assert r.guide_active_mask.all()
    assert np.any(blocked)
    assert np.all(r.cost_terms.static[blocked] >= 100000.)
    selected = r.candidate_names.index(r.selected_mode)
    assert r.selected_static_cost == r.mode_static_costs[selected]
    assert not r.final_static_collisions[
        selected
    ]


def test_fully_blocked_static_map_is_ignored():
    obstacles = field([(.55, .75, -2., 2.), (-2., 2., .55, .75), (-2., 2., -.75, -.55)])
    r = compute(controller(), obstacles)
    assert r.guide_active_mask.all()
    assert r.sampled_controls.shape == (500, 10, 3)
    assert r.guide_rollout_counts.tolist() == [100] * 5
    assert r.selected_mode != 'stop'
    assert r.primary_risk_time_ms == 0.
    assert np.isfinite(r.cost)


def test_static_map_does_not_reject_weighted_representative(monkeypatch):
    c = controller(guide_num_rollouts=4, guide_angles_deg=(45.,),
                   horizon_steps=6, acceleration_limit=(100., 100., 100.),
                   control_min=(-1., -1., -3.), control_max=(1., 1., 3.),
                   speed_limit_l1=1.5,
                   dynamic_risk=DynamicRiskConfig(mode='off', robot_radius=.03))
    left = np.array([[.6, .6, 0.]] * 3 + [[.6, -.6, 0.]] * 3)
    right = left.copy()
    right[:, 1] *= -1
    opt = c._guide_optimizer
    monkeypatch.setattr(opt, 'anchors', lambda *_: left[None].copy())
    opt.noise[0][:] = right - left
    def equal_cost(state, path, controls, trajectories, predictions, **kwargs):
        n, horizon = trajectories.shape[:2]
        zero = np.zeros(n)
        return MppiCostTerms(*(zero.copy() for _ in range(5))), RiskEvaluation(
            zero.copy(), np.zeros((n, horizon)))
    monkeypatch.setattr(c, '_evaluate_rollouts', equal_cost)
    r = compute(c, field([(.35, .45, -.025, .025)]))
    assert r.guide_active_mask[0]
    assert np.all(np.isfinite(r.rollout_costs))
    assert np.allclose(r.weights, .25)
    assert not r.final_static_collisions.any()
    assert r.guide_rejection_reasons[0] == ''
    assert r.selected_mode == 'guide_+45'


@pytest.mark.parametrize('stop_safe', [True, False])
def test_final_dynamic_risk_filter_and_no_safe_candidate_hold(monkeypatch, stop_safe):
    import dra_mppi.mppi_controller as module
    def risk(positions, *args, **kwargs):
        k, h = positions.shape[:2]
        cp = np.zeros((k, h))
        if k <= 6:
            cp[:] = .1
            if stop_safe:
                cp[-1] = 0.
        return RiskEvaluation(np.zeros(k), cp)
    monkeypatch.setattr(module, 'dynamic_risk_cost', risk)
    r = compute(controller())
    assert r.selected_mode == 'stop'
    assert r.safety_status == ('ok' if stop_safe else 'no_safe_candidate_hold')
    assert np.all(r.control_sequence == 0.)
    assert np.all(np.isinf(r.mode_selection_costs[:-1]))


def test_stop_is_fallback_while_moving_guides_keep_penalty_and_hysteresis(monkeypatch):
    c = controller()
    final_costs = np.array([1., 10., 10., 10., 10., 0.])
    block_moving = [False]

    def evaluate(state, path, controls, trajectories, predictions, **kwargs):
        n, horizon = trajectories.shape[:2]
        tracking = final_costs.copy() if n == 6 else np.zeros(n)
        zero = np.zeros(n)
        collision_probability = np.zeros((n, horizon))
        if n == 6 and block_moving[0]:
            collision_probability[:-1] = .1
        return MppiCostTerms(
            tracking,
            zero.copy(),
            zero.copy(),
            zero.copy(),
            zero.copy(),
        ), RiskEvaluation(zero.copy(), collision_probability)

    monkeypatch.setattr(c, '_evaluate_rollouts', evaluate)

    # Stop is cheaper, but any safe moving guide takes precedence.
    result = compute(c)
    assert result.selected_mode == 'guide_+0'
    assert np.isinf(result.mode_selection_costs[-1])

    # Switching between moving guides still requires clearing the configured
    # switch penalty and hysteresis margin.
    final_costs[:] = [1., .8, 10., 10., 10., 0.]
    assert compute(c).selected_mode == 'guide_+0'
    final_costs[:] = [1., .6, 10., 10., 10., 0.]
    assert compute(c).selected_mode == 'guide_+45'

    # Stop is selected only after every moving guide becomes unsafe.
    block_moving[0] = True
    assert compute(c).selected_mode == 'stop'
    assert c._guide_optimizer.previous == 5

    # A safe moving guide restarts immediately; Stop does not compete by cost
    # or retain motion through transition hysteresis.
    block_moving[0] = False
    result = compute(c)
    assert result.selected_mode == 'guide_+45'
    assert np.isinf(result.mode_selection_costs[-1])


def test_three_mode_ten_person_predictions_use_local_guides_and_common_final_batch(
        monkeypatch):
    cfg = DynamicRiskConfig(robot_radius=.2, adaptive_halton_mc=True,
                            mc_sample_density=124., mc_samples_min=512)
    c = controller(dynamic_risk=cfg)
    means = np.zeros((10, 3, 10, 2))
    means[..., 0] = np.linspace(1., 2., 10)[:, None, None]
    means[..., 1] = np.array([-.3, 0., .3])[None, :, None]
    predictions = PredictionSet(means, np.broadcast_to(np.eye(2) * .04,
        (10, 3, 10, 2, 2)).copy(), np.arange(1, 11) * .2,
        mode_weights=np.tile([.5, .3, .2], (10, 1)))
    sizes = []
    estimators = [
        *c._guide_optimizer._guide_risk_estimators,
        c._guide_optimizer._final_risk_estimator,
    ]
    for estimator in estimators:
        original = estimator.evaluate
        def record(positions, *args, _original=original, **kwargs):
            sizes.append(len(positions))
            return _original(positions, *args, **kwargs)
        monkeypatch.setattr(estimator, 'evaluate', record)
    r = compute(c, dynamic_predictions=predictions)
    assert sorted(sizes) == [6, 100, 100, 100, 100, 100]
    assert r.collision_probabilities.shape == (500, 10)
    assert np.max(r.collision_probabilities) > .05
    assert r.primary_risk_time_ms > 0
    assert r.final_risk_time_ms > 0
    assert r.guide_mc_region_areas.shape == (5, 10)
    assert r.guide_mc_sample_counts.shape == (5, 10)
    assert np.all(r.guide_mc_sample_counts >= 128)
    assert np.all(r.guide_mc_sample_counts <= 1024)
    assert c.risk_diagnostics.total_samples == int(
        np.sum(r.guide_mc_sample_counts))
    assert np.all(r.final_mc_sample_counts >= 256)


def test_guide_mc_budget_matches_each_full_rollout_domain_without_predictions():
    c = controller(dynamic_risk=DynamicRiskConfig(
        mode='off', robot_radius=.2, obstacle_radius=.25,
        adaptive_halton_mc=True, guide_mc_sample_density=124.,
        guide_mc_samples_min=128, guide_mc_samples_max=1024))
    r = compute(c)
    trajectories = rollout(np.zeros(3), r.sampled_controls, c.config.dt)
    expected_areas = np.zeros((5, 10))
    for guide in range(5):
        centers = trajectories[r.control_mode_ids == guide, :, :2]
        expected_areas[guide] = np.prod(
            np.max(centers, axis=0) - np.min(centers, axis=0) + .9,
            axis=1,
        )
    expected_samples = np.clip(
        np.ceil(124. * expected_areas), 128, 1024).astype(int)
    assert np.allclose(r.guide_mc_region_areas, expected_areas)
    assert np.array_equal(r.guide_mc_sample_counts, expected_samples)
    # Geometry diagnostics remain available even when no obstacle invokes MC.
    assert c.risk_diagnostics.total_samples == 0


def test_parallel_guide_risk_matches_single_worker_deterministically():
    base = DynamicRiskConfig(
        robot_radius=.2,
        adaptive_halton_mc=True,
        guide_mc_sample_density=124.,
        guide_mc_samples_min=128,
        guide_mc_samples_max=1024,
        final_mc_sample_density=124.,
        final_mc_samples_min=256,
    )
    means = np.zeros((2, 2, 10, 2))
    means[0, :, :, 0] = 1.0
    means[1, :, :, 1] = .7
    predictions = PredictionSet(
        means,
        np.broadcast_to(np.eye(2) * .04, (2, 2, 10, 2, 2)).copy(),
        np.arange(1, 11) * .2,
        mode_weights=np.tile([.6, .4], (2, 1)),
    )
    sequential = compute(controller(dynamic_risk=replace(
        base, guide_parallel_workers=1)), dynamic_predictions=predictions)
    parallel = compute(controller(dynamic_risk=replace(
        base, guide_parallel_workers=2)), dynamic_predictions=predictions)

    assert np.array_equal(
        sequential.guide_mc_sample_counts,
        parallel.guide_mc_sample_counts,
    )
    assert np.allclose(
        sequential.collision_probabilities,
        parallel.collision_probabilities,
    )
    assert np.allclose(sequential.rollout_costs, parallel.rollout_costs)
    assert sequential.selected_mode == parallel.selected_mode


def test_feasible_current_retains_box_wheel_and_acceleration_bounds():
    c = controller()
    rng = np.random.default_rng(18)
    current = np.array([.4, .1, .5])
    controls = c._guide_optimizer.constrain(rng.normal(size=(500, 10, 3)) * 3, current)
    previous = np.concatenate((np.broadcast_to(current, (500, 1, 3)), controls[:, :-1]), axis=1)
    assert np.all(np.abs(controls - previous) <= np.array(c.config.acceleration_limit) * .2 + 1e-12)
    assert np.all(controls >= c._control_min - 1e-12)
    assert np.all(controls <= c._control_max + 1e-12)
    wheel = np.abs(controls[..., 0]) + np.abs(controls[..., 1]) + c.config.yaw_lever * np.abs(controls[..., 2])
    assert np.all(wheel <= .78 + 1e-12)


def test_static_map_does_not_reset_nominals():
    c = controller()
    compute(c)
    opt = c._guide_optimizer
    assert opt.initialized.all()
    compute(c, field([(.55, .75, -.8, .8)]))
    assert opt.initialized.all()
    c.reset()
    assert not opt.initialized.any()
    assert np.all(opt.nominals == 0.)
    assert np.all(opt.temperatures == c.config.temperature)
    assert opt.previous is None


def test_all_strategies_work_without_static_map():
    for strategy in ('guide_conditioned', 'standard'):
        r = controller(strategy=strategy).compute(np.zeros(3), PATH)
        assert r.strategy == strategy
        expected = 500 if strategy == 'guide_conditioned' else 321
        assert r.sampled_controls.shape[0] == expected


def test_static_maps_and_transforms_do_not_affect_result():
    base = field([(.55, .75, -.8, .8)])
    shift = (3., -1., .9)
    rotated = field([(.55, .75, -.8, .8)], transform=shift)
    a = compute(controller(), base)
    b = compute(controller(), rotated, source_to_grid=shift)
    assert a.guide_active_mask.all() and b.guide_active_mask.all()
    assert np.allclose(a.rollout_costs, b.rollout_costs)
    assert not a.final_static_collisions.any() and not b.final_static_collisions.any()
    assert a.selected_mode == b.selected_mode


@pytest.mark.parametrize('unknown', [False, True])
def test_swept_collision_checks_thin_obstacle_between_horizon_points(unknown):
    obstacles = field([(.475, .525, -.3, .3)], unknown=unknown)
    assert obstacles.query(np.array([[0., 0.], [1., 0.]])).min() > .2
    assert obstacles.trajectory_collisions(np.array([[[1., 0.]]]), [0., 0.], .1)[0]


def test_swept_collision_includes_initial_footprint_and_map_boundary():
    obstacles = field()
    queries = np.array([[[2.9, 0.]], [[0., 0.]]])
    assert obstacles.trajectory_collisions(queries, [0., 0.], .2).tolist() == [True, False]
    assert obstacles.trajectory_collisions(np.array([[[0., 0.]]]), [3.1, 0.], .2)[0]


def test_forward_anchor_slows_at_true_goal_without_forcing_rejoin():
    c = controller(guide_noise_std=(0., 0., 0.))
    near_goal = np.array([[0., 0.], [.16, 0.]])
    r = c.compute(np.zeros(3), near_goal, distance_field=field())
    assert r.selected_mode == 'guide_+0'
    assert r.control[0] > 0.
    assert r.guide_anchors[0, -1, 0] == pytest.approx(.16)
    assert r.mode_control_sequences[0, -1, 0] == pytest.approx(0.)
    # A short tracking window must not act like the real global endpoint.
    r = c.compute(np.zeros(3), near_goal, guide_path=PATH, distance_field=field())
    assert r.guide_anchors[0, -1, 0] > 1.


@pytest.mark.parametrize('workers', [1, 2, 5])
def test_disk_integrator_has_identical_parallel_results_and_final_check(workers):
    means = np.zeros((5, 2, 10, 2))
    means[..., 0] = np.linspace(.8, 2.5, 5)[:, None, None]
    means[..., 1] = np.array([.4, -.5])[None, :, None]
    covariance = np.broadcast_to(np.array([[.05, .025], [.025, .025]]), (5, 2, 10, 2, 2))
    prediction = PredictionSet(means, covariance, .2*np.arange(1, 11))
    results = []
    for worker_count in (1, workers):
        c = controller(dynamic_risk=DynamicRiskConfig(mode='gaussian_disk', guide_parallel_workers=worker_count))
        try:
            result = compute(c, dynamic_predictions=prediction)
            assert c.risk_diagnostics.quadrature_evaluations > 0
            assert result.final_risk_diagnostics.quadrature_evaluations > 0
            assert c.risk_diagnostics.total_samples == 0
            assert result.final_risk_diagnostics.unresolved_integrals == 0
            results.append(result)
        finally:
            c.close()
    np.testing.assert_array_equal(results[0].collision_probabilities, results[1].collision_probabilities)
    np.testing.assert_array_equal(results[0].mode_collision_probabilities, results[1].mode_collision_probabilities)
    np.testing.assert_array_equal(results[0].control_sequence, results[1].control_sequence)

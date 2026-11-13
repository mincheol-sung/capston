"""Analytical objective, execution boundary, deterministic worker regressions."""
from dataclasses import replace
import threading
import time

import numpy as np
import pytest

from dra_mppi.common_cost import common_costs
from dra_mppi.dynamic_risk import DynamicRiskConfig, PredictionSet
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiConfig, MppiController


def test_exact_cost_components_and_pre_input_heading():
    cfg = MppiConfig(dt=.2, horizon_steps=2, tracking_path_weight=2.,
                     progress_weight=3., heading_weight=4., desired_speed=.5,
                     speed_weight=5., input_delta_weight=(7., 8., 9.))
    state = np.array([0., 1., np.pi/2])
    u = np.array([[[0., -.5, 1.], [0., -.5, 0.]]])
    x = rollout(state, u, cfg.dt)
    costs = common_costs(cfg, state, np.array([[0., 0.], [5., 0.]]), u, x,
                         np.array([0., -.25, 0.]), return_components=True)
    raw, weighted = costs[-2:]
    assert raw['path_error'][0] == pytest.approx(np.mean(x[0, :, 1]**2))
    assert raw['progress'][0] == pytest.approx((x[0, -1, 0]-.2)**2)
    assert raw['heading'][0] == pytest.approx(np.mean(x[0, :, 2]**2))
    # First body-y projects along the path using psi_0, not psi_1.
    assert raw['speed'][0] == pytest.approx((.5*np.cos(.2)-.5)**2/2)
    assert raw['delta_y'][0] == pytest.approx(.25**2/2)
    assert raw['delta_omega'][0] == 1.
    assert sum(value[0] for value in weighted.values()) == pytest.approx(sum(v[0] for v in costs[:4]))


def test_squared_heading_boundary_and_no_body_vy_penalty():
    cfg = MppiConfig(horizon_steps=2, dt=.2, desired_speed=.4)
    path = np.array([[0., 0.], [5., 0.]])
    body_x = np.array([[[.4, 0., 0.]]*2])
    body_y = np.array([[[0., -.4, 0.]]*2])
    a = common_costs(cfg, np.zeros(3), path, body_x, rollout(np.zeros(3), body_x, .2), body_x[0, 0])
    state = np.array([0., 0., np.pi/2])
    b = common_costs(cfg, state, path, body_y, rollout(state, body_y, .2), body_y[0, 0])
    assert a[2][0] == b[2][0] == 0.
    for yaw in (-np.pi, np.pi):
        x = np.broadcast_to([0., 0., yaw], (1, 2, 3))
        assert common_costs(cfg, x[0, 0], path, body_x*0, x, np.zeros(3))[1][0] == pytest.approx(np.pi**2*cfg.heading_weight)


def test_progress_tracks_fixed_reference_instead_of_rewarding_overspeed():
    cfg = MppiConfig(horizon_steps=4, dt=.1, desired_speed=.2)
    state = np.array([1., 0., 0.])
    path = np.array([[0., 0.], [5., 0.]])
    controls = np.zeros((4, 4, 3))
    controls[..., 0] = np.array([-.2, 0., .2, .4])[:, None]
    states = rollout(state, controls, cfg.dt)
    raw = common_costs(cfg, state, path, controls, states, np.zeros(3),
                       return_components=True)[-2]
    np.testing.assert_allclose(raw['progress'], [.16**2, .08**2, 0., .08**2], atol=1e-15)


@pytest.mark.parametrize('goal_x', [0., .01])
def test_progress_reference_stops_at_path_end(goal_x):
    cfg = MppiConfig(horizon_steps=4, dt=.1, desired_speed=.2)
    state = np.zeros(3)
    path = np.array([[-1., 0.], [goal_x, 0.]])
    controls = np.zeros((2, 4, 3))
    controls[1, 0, 0] = goal_x/cfg.dt
    raw = common_costs(cfg, state, path, controls, rollout(state, controls, cfg.dt),
                       np.zeros(3), return_components=True)[-2]
    np.testing.assert_allclose(raw['progress'], [goal_x**2, 0.], atol=1e-15)


def test_constant_yaw_rate_has_no_effort_penalty_but_changes_are_penalized():
    cfg = MppiConfig(horizon_steps=4, heading_weight=0., input_delta_weight=(0., 0., 2.))
    state = np.zeros(3)
    path = np.array([[0., 0.], [5., 0.]])
    controls = np.zeros((1, 4, 3))
    controls[..., 2] = .3
    states = rollout(state, controls, cfg.dt)
    steady = common_costs(cfg, state, path, controls, states, [0., 0., .3],
                          return_components=True)
    start = common_costs(cfg, state, path, controls, states, np.zeros(3))
    assert 'rotation' not in steady[-2] and 'rotation' not in steady[-1]
    assert steady[3][0] == 0.
    assert start[3][0] == pytest.approx(2.*.3**2/4)
    stopped = common_costs(cfg, state, path, controls*0.,
                          rollout(state, controls*0., cfg.dt), np.zeros(3))
    np.testing.assert_allclose(steady[:4], stopped)


def test_workers_same_samples_cost_representatives_and_selection():
    cfg = MppiConfig(dt=.2, horizon_steps=10, strategy='guide_conditioned',
                     guide_num_rollouts=50, guide_timeout=5.,
                     dynamic_risk=DynamicRiskConfig(adaptive_halton_mc=True))
    t = np.arange(1, 11)*.2
    pred = PredictionSet(np.broadcast_to([.8, .5], (2, 2, 10, 2)),
                         np.broadcast_to(np.eye(2)*.1, (2, 2, 10, 2, 2)), t)
    controllers = [MppiController(replace(cfg, dynamic_risk=replace(
        cfg.dynamic_risk, guide_parallel_workers=w))) for w in (1, 2, 5)]
    try:
        for _ in range(3):
            results = [c.compute(np.zeros(3), np.array([[0., 0.], [3., 0.]]),
                                 dynamic_predictions=pred) for c in controllers]
            for r in results[1:]:
                for key in ('sampled_controls', 'rollout_costs', 'weights',
                            'mode_control_sequences', 'mode_collision_probabilities'):
                    np.testing.assert_allclose(getattr(r, key), getattr(results[0], key), atol=1e-13, rtol=1e-13)
                assert r.selected_mode == results[0].selected_mode
    finally:
        for c in controllers:
            c.close()


@pytest.mark.parametrize('fault', ['exception', 'timeout'])
def test_failed_workers_are_drained_and_next_cycle_can_run(monkeypatch, fault):
    c = MppiController(MppiConfig(strategy='guide_conditioned', guide_num_rollouts=20,
                                  horizon_steps=3, guide_timeout=.03))
    opt = c._guide_optimizer
    original = opt._group
    running = set()
    lock = threading.Lock()

    def failing(g, *args, **kwargs):
        with lock:
            running.add(g)
        try:
            if fault == 'exception' and g == 0:
                raise ValueError('injected')
            time.sleep(.05)
            return original(g, *args, **kwargs)
        finally:
            with lock:
                running.remove(g)
    monkeypatch.setattr(opt, '_group', failing)
    with pytest.raises((TimeoutError, ValueError)):
        c.compute(np.zeros(3), np.array([[0., 0.], [2., 0.]]))
    assert not running
    assert not opt.initialized.any()
    monkeypatch.setattr(opt, '_group', original)
    c.config.guide_timeout = 2.
    assert np.isfinite(c.compute(np.zeros(3), np.array([[0., 0.], [2., 0.]])).control).all()
    c.close()


def test_reentry_is_rejected_and_inputs_remain_owned_by_caller():
    c = MppiController(MppiConfig(strategy='guide_conditioned', guide_num_rollouts=20))
    state, current, path = np.zeros(3), np.zeros(3), np.array([[0., 0.], [2., 0.]])
    c._compute_lock.acquire()
    with pytest.raises(RuntimeError, match='already running'):
        c.compute(state, path)
    c._compute_lock.release()
    c.compute(state, path, current)
    assert state.flags.writeable and current.flags.writeable and path.flags.writeable
    c.close()


def test_cap_insufficiency_is_not_safety_and_predictions_are_snapshots():
    from dra_mppi.dynamic_risk import MonteCarloJointCollisionProbability
    means = np.zeros((1, 1, 1, 2))
    pred = PredictionSet(means, np.eye(2)[None, None, None]*.1, [.2])
    means[:] = 100.
    assert pred.means[0, 0, 0, 0] == 0.
    assert not pred.means.flags.writeable
    estimator = MonteCarloJointCollisionProbability(
        adaptive_halton=True, sample_density=4096., min_samples=4,
        max_samples=4, reject_underresolved=True)
    assert estimator.evaluate(np.zeros((1, 1, 2)), pred, .45)[0, 0] == 1.
    assert estimator.last_diagnostics.underresolved_steps == 1


@pytest.mark.parametrize('field', ['hard_threshold', 'minimum_sigma', 'soft_weight', 'mc_sample_density'])
def test_nonfinite_risk_configuration_rejected(field):
    with pytest.raises(ValueError, match='finite'):
        DynamicRiskConfig(**{field: float('nan')})


def test_static_hinge_at_states_and_swept_hard_check_are_separate():
    from dra_mppi.grid_costmap import DistanceField, GridMetadata
    grid = np.zeros((100, 100))
    grid[:, 50] = 100
    field = DistanceField.from_occupancy(grid, GridMetadata(100, 100, .02, -1., -1., 0.))
    c = MppiController(MppiConfig(static_cost_enabled=True, static_cost_robot_radius=.05))
    # Both endpoints are clear, but the whole segment passes through a wall.
    cost, collision = c._static_obstacle_cost(
        np.array([-.5, 0., 0.]), np.array([[[.5, 0., 0.]]]), field, None)
    assert collision[0] and np.isinf(cost[0])
    c.close()


def test_standard_keeps_primary_and_final_mc_work_distinct():
    cfg = MppiConfig(strategy='standard', dt=.2, horizon_steps=3, num_rollouts=20,
                     dynamic_risk=DynamicRiskConfig(
                         robot_radius=.2, obstacle_radius=.25, adaptive_halton_mc=True,
                         mc_sample_density=512., mc_samples_min=16, mc_samples_max=4096))
    c = MppiController(cfg)
    pred = PredictionSet(np.broadcast_to([2., .7], (1, 1, 3, 2)),
                         np.broadcast_to(np.eye(2)*.1, (1, 1, 3, 2, 2)), [.2, .4, .6])
    try:
        r = c.compute(np.zeros(3), np.array([[0., 0.], [3., 0.]]), dynamic_predictions=pred)
        xy = rollout(np.zeros(3), r.sampled_controls, cfg.dt)[..., :2]
        expected = np.prod(np.ptp(xy, axis=0)+.9, axis=1)
        np.testing.assert_allclose(c.risk_diagnostics.region_areas, expected)
        assert not np.allclose(r.final_mc_region_areas, expected)
        assert c.risk_diagnostics.total_samples == sum(np.ceil(512.*expected).astype(int))
        assert len(r.final_mc_sample_counts) == 3
    finally:
        c.close()

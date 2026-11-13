"""Analytical FOV objective and integration with every rollout evaluation path."""
from dataclasses import replace

import numpy as np
import pytest

from dra_mppi.dynamic_risk import DynamicRiskConfig
from dra_mppi.mppi_controller import MppiConfig, MppiController
from dra_mppi.observation_cost import observation_cost, prepare_target


def config(**kwargs):
    return MppiConfig(observation_cost_enabled=True, observation_cost_weight=2.,
                      observation_half_fov=np.pi/6, **kwargs)


def test_fov_dead_zone_squared_angle_wrap_and_coincident_target():
    cfg = config()
    x = np.zeros((4, 2, 3))
    x[:, :, 2] = np.array([0., np.pi/6, np.pi/2, -np.pi])[:, None]
    raw, weighted = observation_cost(cfg, x, [1., 0.])
    np.testing.assert_allclose(raw, [0., 0., (np.pi/3)**2, (5*np.pi/6)**2], atol=1e-15)
    np.testing.assert_allclose(weighted, 2*raw)
    x[..., 2] = np.pi-.1
    assert np.all(observation_cost(cfg, x, [-1., -.1])[0] == 0.)
    assert np.all(observation_cost(cfg, x, [0., 0.])[0] == 0.)


def test_camera_extrinsics_and_target_trajectory():
    cfg = config(observation_camera_offset=(1., 0.), observation_camera_yaw=np.pi/2)
    x = np.array([[[0., 0., np.pi/2], [0., 0., 0.]]])
    target = prepare_target([[-1., 1.], [1., 1.]], 2)
    np.testing.assert_allclose(observation_cost(cfg, x, target)[0], 0., atol=1e-15)
    angle = .7
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    shifted = x.copy()
    shifted[..., :2] = x[..., :2] @ rotation.T + [5., -2.]
    shifted[..., 2] += angle
    np.testing.assert_allclose(observation_cost(cfg, shifted, target @ rotation.T + [5., -2.])[0],
                               observation_cost(cfg, x, target)[0], atol=1e-15)


def test_target_is_owned_immutable_and_validated():
    target = np.array([1., 2.])
    saved = prepare_target(target, 3)
    target[:] = 0.
    np.testing.assert_equal(saved, [1., 2.])
    assert not saved.flags.writeable
    for invalid in ([np.nan, 1.], [1., 2., 3.], np.zeros((2, 2))):
        with pytest.raises(ValueError, match='observation_target'):
            prepare_target(invalid, 3)


@pytest.mark.parametrize('strategy,evaluation', [('standard', 'split'),
    ('guide_conditioned', 'split'), ('guide_conditioned', 'batch')])
def test_rollout_and_final_costs_include_observation_and_clear_missing_target(strategy, evaluation):
    cfg = config(strategy=strategy, guide_evaluation=evaluation, num_rollouts=30,
                 guide_num_rollouts=30, horizon_steps=5, guide_timeout=5.,
                 dynamic_risk=DynamicRiskConfig(mode='off'))
    c = MppiController(cfg)
    state, path = np.zeros(3), np.array([[0., 0.], [3., 0.]])
    try:
        result = c.compute(state, path, observation_target=[0., 2.])
        assert np.all(result.cost_terms.observation > 0.)
        np.testing.assert_allclose(result.cost_terms.total,
                                   sum(result.cost_terms.weighted.values()))
        assert result.selected_cost_weighted['observation'] > 0.
        assert result.cost == pytest.approx(sum(result.selected_cost_weighted.values()))
        absent = c.compute(state, path)
        assert np.all(absent.cost_terms.observation == 0.)
        assert c._observation_target is None
    finally:
        c.close()
    # OFF ignores even a supplied target and is identical to ON without a target.
    a, b = MppiController(replace(cfg, observation_cost_enabled=False)), MppiController(cfg)
    try:
        off = a.compute(state, path, observation_target=[0., 2.])
        missing = b.compute(state, path)
        np.testing.assert_array_equal(off.control, missing.control)
        np.testing.assert_array_equal(off.rollout_costs, missing.rollout_costs)
    finally:
        a.close()
        b.close()


@pytest.mark.parametrize('parameter,value', [('observation_cost_weight', -1.),
    ('observation_half_fov', 0.), ('observation_half_fov', np.inf),
    ('observation_camera_offset', [np.nan, 0.]), ('observation_camera_yaw', np.nan),
    ('observation_cost_enabled', 'true')])
def test_invalid_observation_configuration(parameter, value):
    with pytest.raises(ValueError):
        MppiController(replace(MppiConfig(), **{parameter: value}))

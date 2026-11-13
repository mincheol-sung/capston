"""Primary MC, final disk verification and bounded prediction input contract."""
from unittest.mock import Mock

import numpy as np
import pytest

from dra_mppi.dynamic_risk import (
    DynamicRiskConfig, PredictionBudgetExceeded, PredictionSet,
    MonteCarloJointCollisionProbability, dynamic_risk_cost,
)
from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability
from dra_mppi.mppi_controller import MppiConfig, MppiController


def predictions(count, horizon=3):
    return PredictionSet(
        np.broadcast_to([.8, .4], (count, 1, horizon, 2)),
        np.broadcast_to([[.05, .025], [.025, .025]], (count, 1, horizon, 2, 2)),
        .1*np.arange(1, horizon+1))


@pytest.mark.parametrize('strategy', ['standard', 'guide_conditioned'])
def test_hybrid_uses_mc_for_primary_and_disk_for_every_final_candidate(strategy):
    risk = DynamicRiskConfig(final_mode='gaussian_disk', max_obstacles=20)
    c = MppiController(MppiConfig(strategy=strategy, horizon_steps=3,
        num_rollouts=20, guide_num_rollouts=20, guide_timeout=5., dynamic_risk=risk))
    final = (c._final_risk_estimator if strategy == 'standard'
             else c._guide_optimizer._final_risk_estimator)
    primary = ([c._risk_estimator] if strategy == 'standard'
               else c._guide_optimizer._guide_risk_estimators)
    assert isinstance(final, GaussianDiskCollisionProbability)
    for estimator in primary:
        assert type(estimator) is MonteCarloJointCollisionProbability
    final.evaluate = Mock(wraps=final.evaluate)
    pred = predictions(1)
    try:
        result = c.compute(np.zeros(3), [[0., 0.], [3., 0.]], dynamic_predictions=pred)
        assert all(e.last_diagnostics.total_samples > 0 for e in primary)
        assert result.final_risk_diagnostics.sampling_mode == 'gaussian_disk'
        assert result.final_risk_diagnostics.quadrature_evaluations > 0
        assert np.all(result.final_mc_sample_counts == 0)
        final.evaluate.assert_called_once()
        positions = final.evaluate.call_args.args[0]
        assert len(positions) == (2 if strategy == 'standard' else 6)
        expected = GaussianDiskCollisionProbability(minimum_sigma=risk.minimum_sigma).evaluate(
            result.trajectory[None, :, :2], pred, risk.robot_radius+risk.obstacle_radius)
        assert result.max_collision_probability == pytest.approx(expected.max(), abs=1e-12)
    finally:
        c.close()


@pytest.mark.parametrize('strategy', ['standard', 'guide_conditioned'])
def test_twenty_objects_allowed_twenty_one_rejected_before_sampling(strategy):
    c = MppiController(MppiConfig(strategy=strategy, horizon_steps=3,
        num_rollouts=20, guide_num_rollouts=20, guide_timeout=5.,
        dynamic_risk=DynamicRiskConfig(final_mode='gaussian_disk', max_obstacles=20)))
    try:
        result = c.compute(np.zeros(3), [[0., 0.], [3., 0.]], dynamic_predictions=predictions(20))
        assert np.all(np.isfinite(result.control))
        c._evaluate_rollouts = Mock(side_effect=AssertionError('must reject before evaluation'))
        with pytest.raises(PredictionBudgetExceeded, match='21 > 20'):
            c.compute(np.zeros(3), [[0., 0.], [3., 0.]], dynamic_predictions=predictions(21))
        c._evaluate_rollouts.assert_not_called()
        assert not c._compute_lock.locked()
        assert np.all(c._nominal == 0.)
        if strategy == 'guide_conditioned':
            assert not c._guide_optimizer.initialized.any()
    finally:
        c.close()


def test_risk_boundary_enforces_budget_without_truncating_and_off_still_disables_risk():
    pred = predictions(21)
    estimator = Mock()
    with pytest.raises(PredictionBudgetExceeded):
        dynamic_risk_cost(np.zeros((1, 3, 2)), pred,
                          DynamicRiskConfig(max_obstacles=20), estimator=estimator)
    estimator.evaluate.assert_not_called()
    assert len(pred.means) == 21
    result = dynamic_risk_cost(np.zeros((1, 3, 2)), pred,
        DynamicRiskConfig(mode='off', final_mode='gaussian_disk', max_obstacles=20))
    assert np.all(result.collision_probability == 0.)


@pytest.mark.parametrize('value', [-1, 1.5, True])
def test_invalid_obstacle_budget_rejected(value):
    with pytest.raises(ValueError, match='max_obstacles'):
        DynamicRiskConfig(max_obstacles=value)


def test_invalid_final_mode_rejected():
    with pytest.raises(ValueError, match='final_mode'):
        DynamicRiskConfig(final_mode='off')

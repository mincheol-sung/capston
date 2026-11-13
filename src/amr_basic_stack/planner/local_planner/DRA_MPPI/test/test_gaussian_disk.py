"""Independent numerical and model checks for direct disk integration."""
from dataclasses import replace
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.stats import ncx2

from dra_mppi.dynamic_risk import DynamicRiskConfig, PredictionSet, dynamic_risk_cost
from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability


def reference():
    spec = importlib.util.spec_from_file_location('disk_reference', Path(__file__).parents[1]/'tools/validate_mc_accuracy.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.disk_reference


def pred(means, covariance, weights=None):
    return PredictionSet(np.asarray(means)[:, :, None], np.asarray(covariance)[:, :, None], np.array([.2]), weights)


@pytest.mark.parametrize('sigma', [.08, .2, .6])
def test_isotropic_matches_cdf_at_and_around_threshold(sigma):
    points = np.stack((np.linspace(0, 3, 301), np.zeros(301)), axis=-1)[:, None]
    p = pred([[[0., 0.]]], np.eye(2)[None, None]*sigma**2)
    actual = GaussianDiskCollisionProbability().evaluate(points, p, .47)[:, 0]
    expected = ncx2.cdf(.47**2/(sigma**2+1e-12), 2, points[:, 0, 0]**2/(sigma**2+1e-12))
    np.testing.assert_allclose(actual, expected, atol=1.01e-10)


def test_rotated_mixtures_match_independent_polar_integral():
    rng = np.random.default_rng(927)
    ref = reference()
    solver = GaussianDiskCollisionProbability()
    for _ in range(32):
        theta = rng.uniform(-np.pi, np.pi)
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        covariance = np.broadcast_to(rot@np.diag([.08**2, .6**2])@rot.T, (2, 2, 2, 2)).copy()
        means = rng.uniform(-1.2, 1.2, (2, 2, 2))
        weights = np.array([[.7, .3], [.2, .8]])
        expected = ref(means, covariance, weights, .47, 160)
        actual = solver.evaluate(np.zeros((1, 1, 2)), pred(means, covariance, weights), .47)[0, 0]
        assert abs(actual-expected) < 3e-7
        assert actual >= expected-2e-10
        assert solver.last_diagnostics.unresolved_integrals == 0


def test_far_anisotropic_tail_is_upper_bound_and_candidate_independent():
    p = pred([[[3., 2.]]], [[[[.04, .015], [.015, .02]]]])
    solver = GaussianDiskCollisionProbability()
    alone = solver.evaluate(np.zeros((1, 1, 2)), p, .47)[0, 0]
    together = solver.evaluate(np.array([[[0., 0.]], [[100., -100.]]]), p, .47)[0, 0]
    assert alone == together
    assert solver.last_diagnostics.quadrature_evaluations == 0
    assert 0 <= alone <= 1e-10


def test_factory_radius_weights_and_hard_threshold():
    means = np.array([[[[0., 0.]], [[4., 0.]]]])
    covariance = np.broadcast_to(np.eye(2)*.02, (1, 2, 1, 2, 2))
    p = PredictionSet(means, covariance, np.array([.2]), np.array([[[1.], [0.]]]), np.array([.3]))
    cfg = DynamicRiskConfig(mode='gaussian_disk', robot_radius=.22)
    result = dynamic_risk_cost(np.zeros((1, 1, 2)), p, cfg)
    assert result.collision_probability[0, 0] > .99
    assert result.cost[0] > cfg.hard_penalty
    off = dynamic_risk_cost(np.zeros((1, 1, 2)), p, replace(cfg, mode='off'))
    assert off.cost[0] == 0.


def test_unresolved_integral_fails_closed(monkeypatch):
    solver = GaussianDiskCollisionProbability()
    monkeypatch.setattr(solver, '_integrate', lambda mu, precision, radius, order: np.full(len(mu), float(order % 3)/10))
    p = pred([[[0., 0.]]], [[[[.04, .01], [.01, .02]]]])
    assert solver.evaluate(np.zeros((1, 1, 2)), p, .47)[0, 0] == 1.
    assert solver.last_diagnostics.unresolved_integrals == 1

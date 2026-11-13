"""CV/CT IMM interaction, TPM, and fused-state tests."""
import numpy as np

from object_tracker.core.config import TrackerConfig
from object_tracker.core.imm import (
    ImmEstimator,
    moment_match,
    transition_matrix,
)


def test_mean_sojourn_transition_is_stochastic_and_composes():
    first = transition_matrix(0.07, 3.0, 1.0)
    second = transition_matrix(0.13, 3.0, 1.0)
    combined = transition_matrix(0.20, 3.0, 1.0)
    assert np.all(first >= 0.0)
    assert np.allclose(np.sum(first, axis=1), 1.0)
    assert np.allclose(first @ second, combined, atol=1e-14)


def test_moment_match_includes_between_mode_spread():
    states = np.array([
        [0.0, 0.0, 1.0, 0.0, 0.0],
        [2.0, 0.0, 1.0, 0.0, 0.5],
    ])
    covariances = np.repeat(np.eye(5)[None, ...] * 0.1, 2, axis=0)
    mean, covariance = moment_match(states, covariances, [0.5, 0.5])
    assert mean[0] == 1.0
    assert covariance[0, 0] == 1.1


def test_imm_probabilities_and_covariance_remain_valid():
    cfg = TrackerConfig()
    estimator = ImmEstimator([0.0, 0.0], np.eye(2) * 0.01 ** 2, cfg)
    rng = np.random.default_rng(5)
    position = np.zeros(2)
    for _ in range(100):
        position += np.array([0.08, 0.0])
        estimator.predict(0.1)
        estimator.update(
            position + rng.normal(0.0, 0.01, 2),
            np.eye(2) * 0.01 ** 2,
        )
        assert np.isclose(np.sum(estimator.mu), 1.0)
        assert np.all(estimator.mu >= cfg.minimum_mode_probability * 0.99)
        assert np.allclose(estimator.P, estimator.P.T, atol=1e-12)
        assert np.all(np.linalg.eigvalsh(estimator.P) > 0.0)


def test_measurement_likelihood_favors_the_matching_mode():
    cfg = TrackerConfig(minimum_mode_probability=0.0)
    states = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0, 0.0, 0.0],
    ])
    covariances = np.repeat(np.eye(5)[None, ...] * 0.01, 3, axis=0)
    estimator = ImmEstimator.from_components(
        states, covariances, [0.4, 0.2, 0.4], cfg)
    estimator.update([0.0, 0.0], np.eye(2) * 0.01)
    assert estimator.mu[0] > 0.99


def test_unmatched_prediction_uses_actual_dt_tpm():
    cfg = TrackerConfig(mean_sojourn_cv=3.0, mean_sojourn_ct=1.0)
    estimator = ImmEstimator([0.0, 0.0], np.eye(2) * 0.01, cfg)
    before = estimator.mu.copy()
    estimator.predict(0.2)
    assert np.allclose(estimator.mu, before @ transition_matrix(0.2, 3.0, 1.0, 3.0))

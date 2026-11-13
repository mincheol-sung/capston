"""Measurement-free IMM future propagation tests."""
from motion_predictor.core.predictor import ImmMotionPredictor
import numpy as np
from object_tracker.core.config import TrackerConfig


def posterior(omega=0.8):
    states = np.array([
        [0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, omega],
        [0.0, 0.0, 0.0, 0.0, 0.0],
    ])
    covariances = np.repeat(np.eye(5)[None, ...] * 0.01, 3, axis=0)
    return states, covariances, np.array([0.6, 0.3, 0.1])


def test_fixed_weight_prediction_keeps_two_distinct_hypotheses():
    predictor = ImmMotionPredictor(
        TrackerConfig(), prediction_dt=0.2, horizon=1.0,
        propagate_mode_probabilities=False)
    times, means, covariances, weights = predictor.predict(
        *posterior(), mean_sojourn_cv=3.0, mean_sojourn_ct=1.0)
    assert np.allclose(times, [0.2, 0.4, 0.6, 0.8, 1.0])
    assert means.shape == (3, 5, 2)
    assert covariances.shape == (3, 5, 2, 2)
    assert np.allclose(weights[0], 0.6)
    assert np.allclose(weights[1], 0.3)
    assert abs(means[0, -1, 1]) < 1e-12
    assert means[1, -1, 1] > 0.2
    assert np.allclose(means[2], 0.0)
    assert np.allclose(weights[2], 0.1)


def test_tpm_prediction_updates_weights_at_each_timestep():
    predictor = ImmMotionPredictor(
        TrackerConfig(), prediction_dt=0.2, horizon=1.0,
        propagate_mode_probabilities=True)
    _, _, _, weights = predictor.predict(
        *posterior(), mean_sojourn_cv=3.0, mean_sojourn_ct=1.0)
    assert np.allclose(np.sum(weights, axis=0), 1.0)
    assert not np.allclose(weights[:, 0], weights[:, -1])

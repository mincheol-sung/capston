"""Shared-state CV and CT motion-model tests."""
import numpy as np

from object_tracker.core.motion_models import (
    CtEkfModel,
    ct_transition_and_jacobian,
)


def test_ct_zero_turn_is_continuous_with_cv():
    state = np.array([1.0, -2.0, 0.8, -0.3, 0.0])
    predicted, _ = ct_transition_and_jacobian(state, 0.2)
    assert np.allclose(predicted[:2], state[:2] + 0.2 * state[2:4])
    assert np.allclose(predicted[2:4], state[2:4])


def test_ct_jacobian_matches_finite_difference_near_and_away_from_zero():
    for omega in (0.0, 1.0e-5, 0.7):
        state = np.array([0.2, -0.4, 1.1, 0.3, omega])
        _, analytic = ct_transition_and_jacobian(state, 0.1)
        numeric = np.empty((5, 5))
        epsilon = 1.0e-6
        for column in range(5):
            delta = np.zeros(5)
            delta[column] = epsilon
            plus, _ = ct_transition_and_jacobian(state + delta, 0.1)
            minus, _ = ct_transition_and_jacobian(state - delta, 0.1)
            numeric[:, column] = (plus - minus) / (2.0 * epsilon)
        assert np.allclose(analytic, numeric, rtol=2e-5, atol=2e-7)


def test_ct_ekf_covariance_remains_symmetric_positive():
    model = CtEkfModel(
        np.array([0.0, 0.0, 1.0, 0.0, 0.8]),
        np.eye(5) * 0.1,
    )
    for _ in range(100):
        model.predict(0.1, sigma_a=0.7, sigma_omega=0.5)
        model.update(model.x[:2], np.eye(2) * 0.01 ** 2)
        assert np.allclose(model.P, model.P.T, atol=1e-12)
        assert np.all(np.linalg.eigvalsh(model.P) > 0.0)

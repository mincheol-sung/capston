"""CV/CT/STATIC interacting multiple-model estimator."""
import math

import numpy as np
from scipy.linalg import expm

from object_tracker.core.motion_models import (
    CtEkfModel,
    CvKalmanModel,
    H5,
    StaticKalmanModel,
)


MODE_CV = 0
MODE_CT = 1
MODE_STATIC = 2
MODE_COUNT = 3


def transition_matrix(dt, mean_sojourn_cv, mean_sojourn_ct,
                      mean_sojourn_static=None, static_to_cv=0.9,
                      cv_to_static=0.5, ct_to_static=0.2):
    """CTMC transition, rows=source, columns=destination (CV, CT, STATIC).

    The optional two-mode form serves historical diagnostics only. Tracker
    and predictor supply static dwell and exit branching fractions explicitly.
    """
    if not np.all(np.isfinite([dt, mean_sojourn_cv, mean_sojourn_ct])) or dt < 0.0:
        raise ValueError('dt must be non-negative')
    if mean_sojourn_cv <= 0.0 or mean_sojourn_ct <= 0.0:
        raise ValueError('mean sojourn times must be positive')
    if mean_sojourn_static is not None:
        if mean_sojourn_static <= 0 or not np.isfinite(mean_sojourn_static):
            raise ValueError('static sojourn time must be finite and positive')
        branches = np.array([static_to_cv, cv_to_static, ct_to_static])
        if not np.all(np.isfinite(branches)) or np.any((branches < 0) | (branches > 1)):
            raise ValueError('transition branching fractions must be in [0,1]')
        rates = np.array([
            [-1., 1.-cv_to_static, cv_to_static],
            [1.-ct_to_static, -1., ct_to_static],
            [static_to_cv, 1.-static_to_cv, -1.],
        ]) / np.array([mean_sojourn_cv, mean_sojourn_ct,
                       mean_sojourn_static])[:, None]
        transition = np.maximum(expm(rates * dt), 0.0)
        return transition / transition.sum(axis=1, keepdims=True)
    # Two-state utility retained for historical CV/CT calibration tools.
    rate_cv = 1.0 / float(mean_sojourn_cv)
    rate_ct = 1.0 / float(mean_sojourn_ct)
    rate_sum = rate_cv + rate_ct
    decay = math.exp(-rate_sum * dt)
    switch_mass = -math.expm1(-rate_sum * dt)
    return np.array([
        [
            rate_ct / rate_sum + rate_cv / rate_sum * decay,
            rate_cv / rate_sum * switch_mass,
        ],
        [
            rate_ct / rate_sum * switch_mass,
            rate_cv / rate_sum + rate_ct / rate_sum * decay,
        ],
    ])


def moment_match(states, covariances, weights):
    """Collapse Gaussian components while retaining between-mode spread."""
    weights = np.asarray(weights, dtype=float)
    weights = weights / np.sum(weights)
    mean = np.sum(weights[:, None] * states, axis=0)
    covariance = np.zeros_like(covariances[0])
    for weight, state, component_covariance in zip(
            weights, states, covariances):
        delta = state - mean
        covariance += weight * (
            component_covariance + np.outer(delta, delta))
    return mean, 0.5 * (covariance + covariance.T)


def gaussian_log_likelihood(residual, covariance):
    """Stable log N(residual; 0, covariance), or -inf if singular."""
    sign, log_determinant = np.linalg.slogdet(covariance)
    if sign <= 0.0 or not np.isfinite(log_determinant):
        return float('-inf')
    try:
        mahalanobis = float(residual @ np.linalg.solve(covariance, residual))
    except np.linalg.LinAlgError:
        return float('-inf')
    normalizer = 2.0 * math.log(2.0 * math.pi)
    return -0.5 * (mahalanobis + log_determinant + normalizer)


class ImmEstimator:
    """CV-KF + CT-EKF + STATIC-KF with a moment-matched output."""

    def __init__(self, measurement, covariance, config):
        config.validate()
        state = np.zeros(5, dtype=float)
        state[:2] = np.asarray(measurement, dtype=float)
        initial_covariance = np.zeros((5, 5), dtype=float)
        initial_covariance[:2, :2] = np.asarray(covariance, dtype=float)
        initial_covariance[2:4, 2:4] = np.eye(2) * config.sigma_v0 ** 2
        initial_covariance[4, 4] = config.sigma_omega0 ** 2
        probability_cv = float(config.initial_probability_cv)
        self.models = [
            CvKalmanModel(state, initial_covariance),
            CtEkfModel(state, initial_covariance),
            StaticKalmanModel(state, initial_covariance),
        ]
        probability_static = float(config.initial_probability_static)
        self.mu = np.array([probability_cv, 1.0-probability_cv-probability_static,
                            probability_static])
        self.config = config
        self.last_transition = np.eye(MODE_COUNT)
        self.mode_innovations = ()
        self._refresh_fused()

    @classmethod
    def from_components(cls, states, covariances, probabilities, config):
        """Reconstruct an estimator from a serialized IMM posterior."""
        config.validate()
        states = np.asarray(states, dtype=float)
        covariances = np.asarray(covariances, dtype=float)
        if (states.shape != (MODE_COUNT, 5) or covariances.shape != (MODE_COUNT, 5, 5)
                or not np.all(np.isfinite(states)) or not np.all(np.isfinite(covariances))):
            raise ValueError('expected finite CV/CT/STATIC state (3,5), covariance (3,5,5)')
        instance = cls.__new__(cls)
        instance.models = [
            CvKalmanModel(states[MODE_CV], covariances[MODE_CV]),
            CtEkfModel(states[MODE_CT], covariances[MODE_CT]),
            StaticKalmanModel(states[MODE_STATIC], covariances[MODE_STATIC]),
        ]
        instance.mu = np.asarray(probabilities, dtype=float).copy()
        valid_probabilities = all((
            instance.mu.shape == (MODE_COUNT,),
            np.all(np.isfinite(instance.mu)),
            np.all(instance.mu >= 0.0),
            np.sum(instance.mu) > 0.0,
        ))
        if not valid_probabilities:
            raise ValueError(
                'IMM probabilities must be three non-negative finite values '
                'with a positive total')
        instance.mu = instance.mu / np.sum(instance.mu)
        instance.config = config
        instance.last_transition = np.eye(MODE_COUNT)
        instance.mode_innovations = ()
        instance._refresh_fused()
        return instance

    def _refresh_fused(self):
        states = np.asarray([model.x for model in self.models])
        covariances = np.asarray([model.P for model in self.models])
        self.x, self.P = moment_match(states, covariances, self.mu)

    def _mix(self, transition, mode_prior):
        mixed_states = []
        mixed_covariances = []
        states = np.asarray([model.x for model in self.models])
        covariances = np.asarray([model.P for model in self.models])
        for destination in range(MODE_COUNT):
            # A valid zero branching fraction can leave a zero-weight mode
            # unreachable. Its conditional mixing distribution is undefined;
            # preserve the component until it receives probability again.
            if mode_prior[destination] <= 0.0:
                mixed_states.append(states[destination].copy())
                mixed_covariances.append(covariances[destination].copy())
                continue
            conditional = self.mu * transition[:, destination]
            conditional /= mode_prior[destination]
            mean, covariance = moment_match(states, covariances, conditional)
            mixed_states.append(mean)
            mixed_covariances.append(covariance)
        for model, state, covariance in zip(
                self.models, mixed_states, mixed_covariances):
            model.x = state
            model.P = covariance

    def predict(self, dt):
        if dt <= 0.0:
            return
        cfg = self.config
        transition = transition_matrix(
            dt, cfg.mean_sojourn_cv, cfg.mean_sojourn_ct, cfg.mean_sojourn_static,
            cfg.static_to_cv, cfg.cv_to_static, cfg.ct_to_static)
        mode_prior = self.mu @ transition
        self._mix(transition, mode_prior)
        self.models[MODE_CV].predict(
            dt, cfg.sigma_a, cfg.sigma_omega_cv, cfg.cv_omega_decay_time)
        self.models[MODE_CT].predict(
            dt, cfg.sigma_a_ct, cfg.sigma_omega_ct)
        self.models[MODE_STATIC].predict(
            dt, cfg.sigma_position_static, cfg.static_velocity_decay_time)
        self.mu = mode_prior
        self.last_transition = transition
        self._refresh_fused()

    def innovation(self, measurement, covariance):
        """Innovation of the moment-matched predicted Gaussian."""
        residual = np.asarray(measurement, dtype=float) - H5 @ self.x
        innovation_covariance = H5 @ self.P @ H5.T + covariance
        return residual, innovation_covariance

    def update(self, measurement, covariance):
        """Update every mode and return the moment-matched innovation.

        The per-mode innovations are kept in ``mode_innovations`` because
        mode-conditioned NIS is what identifies ``sigma_a`` and
        ``sigma_a_ct`` separately.  The fused innovation is a single scalar
        stream and can only identify one scale parameter; see
        ``core/nis.py`` for the estimator built on top of this.
        """
        fused_innovation = self.innovation(measurement, covariance)
        log_posterior = np.empty(MODE_COUNT)
        mode_innovations = []
        for index, model in enumerate(self.models):
            residual, innovation_covariance = model.update(
                measurement, covariance)
            mode_innovations.append((residual, innovation_covariance))
            prior_log_probability = math.log(
                max(self.mu[index], np.finfo(float).tiny))
            log_posterior[index] = prior_log_probability
            log_posterior[index] += gaussian_log_likelihood(
                residual, innovation_covariance)
        maximum = np.max(log_posterior)
        if not np.isfinite(maximum):
            posterior = self.mu.copy()
        else:
            posterior = np.exp(log_posterior - maximum)
            posterior /= np.sum(posterior)
        floor = float(self.config.minimum_mode_probability)
        if floor > 0.0:
            posterior = np.maximum(posterior, floor)
            posterior /= np.sum(posterior)
        self.mode_innovations = tuple(mode_innovations)
        self.mu = posterior
        self._refresh_fused()
        return fused_innovation

    def mode_posteriors(self):
        return tuple(
            (model.x.copy(), model.P.copy(), float(probability))
            for model, probability in zip(self.models, self.mu)
        )

    @property
    def position(self):
        return self.x[:2]

    @property
    def velocity(self):
        return self.x[2:4]

    @property
    def P_pp(self):
        return self.P[:2, :2]

    @property
    def P_pv(self):
        return self.P[:2, 2:4]

    @property
    def P_vv(self):
        return self.P[2:4, 2:4]

    @property
    def speed(self):
        return float(np.linalg.norm(self.velocity))

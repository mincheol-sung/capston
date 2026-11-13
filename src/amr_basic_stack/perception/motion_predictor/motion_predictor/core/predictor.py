"""Measurement-free CV/CT/STATIC prediction from one IMM posterior."""
import copy
import math

import numpy as np

from object_tracker.core.imm import ImmEstimator, MODE_COUNT, MODE_CT, MODE_CV, MODE_STATIC
from object_tracker.core.motion_models import CtEkfModel, CvKalmanModel, StaticKalmanModel


class ImmMotionPredictor:
    """Propagate three IMM modes without performing another filter update."""

    def __init__(self, model_config, prediction_dt=0.2, horizon=4.5,
                 propagate_mode_probabilities=False):
        self.model_config = model_config
        self.prediction_dt = float(prediction_dt)
        self.horizon = float(horizon)
        self.propagate_mode_probabilities = bool(propagate_mode_probabilities)
        if self.prediction_dt <= 0.0 or self.horizon <= 0.0:
            raise ValueError('prediction_dt and horizon must be positive')

    @property
    def steps(self):
        return int(math.ceil(self.horizon / self.prediction_dt - 1e-12))

    def predict(self, states, covariances, probabilities,
                mean_sojourn_cv, mean_sojourn_ct, mean_sojourn_static=None,
                static_to_cv=None, cv_to_static=None, ct_to_static=None):
        states = np.asarray(states, dtype=float)
        covariances = np.asarray(covariances, dtype=float)
        probabilities = np.asarray(probabilities, dtype=float)
        if states.shape != (MODE_COUNT, 5):
            raise ValueError('states must have shape (3,5) ordered CV,CT,STATIC')
        if covariances.shape != (MODE_COUNT, 5, 5):
            raise ValueError('covariances must have shape (3,5,5)')
        if not np.all(np.isfinite(states)) or not np.all(np.isfinite(covariances)):
            raise ValueError('IMM components must be finite')
        if (not np.allclose(covariances, covariances.transpose(0, 2, 1))
                or np.linalg.eigvalsh(covariances).min() < -1e-10):
            raise ValueError('IMM covariance must be symmetric positive semidefinite')
        valid_probabilities = all((
            probabilities.shape == (MODE_COUNT,),
            np.all(np.isfinite(probabilities)),
            np.all(probabilities >= 0.0),
            np.sum(probabilities) > 0.0,
        ))
        if not valid_probabilities:
            raise ValueError(
                'probabilities must be three non-negative finite values '
                'with a positive total')
        probabilities = probabilities / np.sum(probabilities)

        cfg = copy.copy(self.model_config)
        cfg.mean_sojourn_cv = float(mean_sojourn_cv)
        cfg.mean_sojourn_ct = float(mean_sojourn_ct)
        for name, value in (('mean_sojourn_static', mean_sojourn_static),
                            ('static_to_cv', static_to_cv),
                            ('cv_to_static', cv_to_static),
                            ('ct_to_static', ct_to_static)):
            if value is not None:
                setattr(cfg, name, float(value))
        cfg.validate()

        count = self.steps
        means = np.empty((MODE_COUNT, count, 2), dtype=float)
        position_covariances = np.empty(
            (MODE_COUNT, count, 2, 2), dtype=float)
        weights = np.empty((MODE_COUNT, count), dtype=float)

        if self.propagate_mode_probabilities:
            estimator = ImmEstimator.from_components(
                states, covariances, probabilities, cfg)
            for step in range(count):
                estimator.predict(self.prediction_dt)
                for mode, (state, covariance, probability) in enumerate(
                        estimator.mode_posteriors()):
                    means[mode, step] = state[:2]
                    position_covariances[mode, step] = covariance[:2, :2]
                    weights[mode, step] = probability
        else:
            models = [
                CvKalmanModel(states[MODE_CV], covariances[MODE_CV]),
                CtEkfModel(states[MODE_CT], covariances[MODE_CT]),
                StaticKalmanModel(states[MODE_STATIC], covariances[MODE_STATIC]),
            ]
            for step in range(count):
                models[MODE_CV].predict(
                    self.prediction_dt,
                    cfg.sigma_a,
                    cfg.sigma_omega_cv,
                    cfg.cv_omega_decay_time,
                )
                models[MODE_CT].predict(
                    self.prediction_dt,
                    cfg.sigma_a_ct,
                    cfg.sigma_omega_ct,
                )
                models[MODE_STATIC].predict(
                    self.prediction_dt, cfg.sigma_position_static,
                    cfg.static_velocity_decay_time)
                for mode, model in enumerate(models):
                    means[mode, step] = model.x[:2]
                    position_covariances[mode, step] = model.P[:2, :2]
                    weights[mode, step] = probabilities[mode]

        times = np.arange(1, count + 1, dtype=float) * self.prediction_dt
        return times, means, position_covariances, weights

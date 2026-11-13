"""Common-state CV, coordinated-turn and stationary tracker IMM models.

All models use ``[px, py, vx, vy, omega]`` in the odom frame.  Sharing the
state space keeps IMM interaction lossless; the CV model simply ignores turn
rate in its position dynamics and exponentially damps it toward zero.
"""
import math

import numpy as np


H5 = np.array([
    [1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0, 0.0],
])
I5 = np.eye(5)


def _symmetrize(covariance):
    return 0.5 * (covariance + covariance.T)


def cwna_process_noise(dt, sigma_a):
    """Return 5-D Cartesian CWNA noise; omega noise is added by each model."""
    q = float(sigma_a) ** 2
    d3 = dt ** 3 / 3.0
    d2 = dt ** 2 / 2.0
    result = np.zeros((5, 5), dtype=float)
    result[0, 0] = result[1, 1] = q * d3
    result[2, 2] = result[3, 3] = q * dt
    result[0, 2] = result[2, 0] = q * d2
    result[1, 3] = result[3, 1] = q * d2
    return result


def cv_transition(dt, omega_decay_time):
    """Linear nearly-CV transition in the shared five-dimensional state."""
    transition = np.eye(5)
    transition[0, 2] = dt
    transition[1, 3] = dt
    if omega_decay_time <= 0.0:
        transition[4, 4] = 0.0
    else:
        transition[4, 4] = math.exp(-dt / omega_decay_time)
    return transition


def ct_transition_and_jacobian(state, dt, small_angle=1e-4):
    """Coordinated-turn transition and analytic EKF Jacobian.

    Taylor forms around zero turn rate avoid the removable divisions by
    ``omega`` and make the CT transition continuous with CV.
    """
    px, py, vx, vy, omega = np.asarray(state, dtype=float)
    angle = omega * dt
    sine = math.sin(angle)
    cosine = math.cos(angle)

    if abs(angle) < small_angle:
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        dt5 = dt4 * dt
        omega2 = omega * omega
        omega3 = omega2 * omega
        omega4 = omega2 * omega2
        a = dt - omega2 * dt3 / 6.0 + omega4 * dt5 / 120.0
        b = omega * dt2 / 2.0 - omega3 * dt4 / 24.0
        da = -omega * dt3 / 3.0 + omega3 * dt5 / 30.0
        db = dt2 / 2.0 - omega2 * dt4 / 8.0
    else:
        inv_omega = 1.0 / omega
        inv_omega2 = inv_omega * inv_omega
        a = sine * inv_omega
        b = (1.0 - cosine) * inv_omega
        da = (dt * cosine * omega - sine) * inv_omega2
        db = (dt * sine * omega - (1.0 - cosine)) * inv_omega2

    predicted = np.array([
        px + a * vx - b * vy,
        py + b * vx + a * vy,
        cosine * vx - sine * vy,
        sine * vx + cosine * vy,
        omega,
    ])

    jacobian = np.eye(5)
    jacobian[0, 2] = a
    jacobian[0, 3] = -b
    jacobian[0, 4] = da * vx - db * vy
    jacobian[1, 2] = b
    jacobian[1, 3] = a
    jacobian[1, 4] = db * vx + da * vy
    jacobian[2, 2] = cosine
    jacobian[2, 3] = -sine
    jacobian[2, 4] = -dt * sine * vx - dt * cosine * vy
    jacobian[3, 2] = sine
    jacobian[3, 3] = cosine
    jacobian[3, 4] = dt * cosine * vx - dt * sine * vy
    return predicted, jacobian


class GaussianMotionModel:
    """Shared measurement-update implementation for one IMM mode."""

    def __init__(self, state, covariance):
        self.x = np.asarray(state, dtype=float).copy()
        self.P = np.asarray(covariance, dtype=float).copy()
        if self.x.shape != (5,) or self.P.shape != (5, 5):
            raise ValueError(
                'motion model requires state (5,) and covariance (5,5)')

    def innovation(self, measurement, covariance):
        residual = np.asarray(measurement, dtype=float) - H5 @ self.x
        innovation_covariance = H5 @ self.P @ H5.T + covariance
        return residual, innovation_covariance

    def update(self, measurement, covariance):
        measurement_covariance = np.asarray(covariance, dtype=float)
        residual, innovation_covariance = self.innovation(
            measurement, measurement_covariance)
        gain = self.P @ H5.T @ np.linalg.inv(innovation_covariance)
        self.x = self.x + gain @ residual
        joseph = I5 - gain @ H5
        posterior = joseph @ self.P @ joseph.T
        posterior += gain @ measurement_covariance @ gain.T
        self.P = _symmetrize(posterior)
        return residual, innovation_covariance

    def copy(self):
        return type(self)(self.x.copy(), self.P.copy())


class StaticKalmanModel(GaussianMotionModel):
    """Constant position with damped latent velocity and centroid random walk."""

    def predict(self, dt, sigma_position, decay_time):
        decay = math.exp(-dt / decay_time)
        transition = np.diag([1.0, 1.0, decay, decay, decay])
        noise = np.diag([sigma_position ** 2 * dt] * 2
                        + [1e-6 * (1.0 - decay ** 2)] * 3)
        self.x = transition @ self.x
        self.P = _symmetrize(transition @ self.P @ transition.T + noise)


class CvKalmanModel(GaussianMotionModel):
    """Linear nearly-constant-velocity mode in the common IMM state."""

    def predict(self, dt, sigma_a, sigma_omega, omega_decay_time):
        transition = cv_transition(dt, omega_decay_time)
        noise = cwna_process_noise(dt, sigma_a)
        noise[4, 4] = float(sigma_omega) ** 2 * dt
        self.x = transition @ self.x
        self.P = _symmetrize(transition @ self.P @ transition.T + noise)


class CtEkfModel(GaussianMotionModel):
    """Coordinated-turn EKF mode with a linear position measurement."""

    def predict(self, dt, sigma_a, sigma_omega):
        predicted, jacobian = ct_transition_and_jacobian(self.x, dt)
        noise = cwna_process_noise(dt, sigma_a)
        noise[4, 4] = float(sigma_omega) ** 2 * dt
        self.x = predicted
        self.P = _symmetrize(jacobian @ self.P @ jacobian.T + noise)

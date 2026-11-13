"""Planar holonomic kinematic model used by the MPPI rollout."""

import numpy as np

from .geometry import wrap_angle


def rollout(initial_state, controls, dt):
    """Roll out body-frame [vx, vy, yaw_rate] controls.

    ``controls`` may be shaped ``(N, 3)`` or ``(K, N, 3)``. The returned
    array has the same leading dimensions and contains states after each
    control has been applied.
    """
    control_values = np.asarray(controls, dtype=np.float64)
    if control_values.ndim not in (2, 3) or control_values.shape[-1] != 3:
        raise ValueError('controls must have shape (N, 3) or (K, N, 3)')
    if dt <= 0.0:
        raise ValueError('dt must be positive')

    was_single = control_values.ndim == 2
    if was_single:
        control_values = control_values[None, ...]

    batch_size, horizon, _ = control_values.shape
    state = np.broadcast_to(
        np.asarray(initial_state, dtype=np.float64),
        (batch_size, 3),
    ).copy()
    states = np.empty((batch_size, horizon, 3), dtype=np.float64)

    for step in range(horizon):
        vx = control_values[:, step, 0]
        vy = control_values[:, step, 1]
        yaw_rate = control_values[:, step, 2]
        cos_yaw = np.cos(state[:, 2])
        sin_yaw = np.sin(state[:, 2])
        state[:, 0] += (vx * cos_yaw - vy * sin_yaw) * dt
        state[:, 1] += (vx * sin_yaw + vy * cos_yaw) * dt
        state[:, 2] = wrap_angle(state[:, 2] + yaw_rate * dt)
        states[:, step] = state

    return states[0] if was_single else states

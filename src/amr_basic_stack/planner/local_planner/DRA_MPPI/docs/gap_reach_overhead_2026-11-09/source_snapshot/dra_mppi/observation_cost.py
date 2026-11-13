"""Optional squared horizontal-FOV residual for one externally selected target.

This is a geometric visibility preference, not an information-gain estimate or
an occlusion/detection guarantee. Coordinates share the rollout planning frame.
"""
import numpy as np

from .geometry import wrap_angle


def prepare_target(target, horizon):
    """Own an immutable fixed XY point or x_1..x_N target position sequence."""
    if target is None:
        return None
    value = np.array(target, dtype=float, copy=True)
    if value.shape not in ((2,), (horizon, 2)) or not np.all(np.isfinite(value)):
        raise ValueError('observation_target must be finite (2,) or (horizon, 2)')
    value.setflags(write=False)
    return value


def observation_cost(cfg, trajectories, target):
    """Return raw mean squared FOV violation [rad^2] and its weighted cost."""
    raw = np.zeros(len(trajectories), dtype=float)
    if not cfg.observation_cost_enabled or target is None:
        return raw, raw.copy()
    yaw = trajectories[..., 2]
    c, s = np.cos(yaw), np.sin(yaw)
    ox, oy = cfg.observation_camera_offset
    camera = trajectories[..., :2] + np.stack((c*ox-s*oy, s*ox+c*oy), axis=-1)
    relative = np.asarray(target)-camera
    bearing = np.arctan2(relative[..., 1], relative[..., 0])
    error = wrap_angle(bearing-yaw-cfg.observation_camera_yaw)
    residual = np.maximum(0., np.abs(error)-cfg.observation_half_fov)
    # Bearing is undefined at a coincident point; do not invent a heading.
    residual[np.linalg.norm(relative, axis=-1) < 1e-9] = 0.
    raw = np.mean(residual**2, axis=1)
    return raw, cfg.observation_cost_weight*raw

"""Small 2-D geometry helpers shared by the planner and ROS adapter."""

import numpy as np


def wrap_angle(angle):
    """Wrap a scalar or array of angles to [-pi, pi)."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def quaternion_to_yaw(x, y, z, w):
    """Return planar yaw from a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(sin_yaw, cos_yaw))


def transform_points(points, transform):
    """Apply (tx, ty, yaw) to an array whose final dimension is x/y."""
    values = np.asarray(points, dtype=np.float64)
    tx, ty, yaw = transform
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    x = cos_yaw * values[..., 0] - sin_yaw * values[..., 1] + tx
    y = sin_yaw * values[..., 0] + cos_yaw * values[..., 1] + ty
    return np.stack((x, y), axis=-1)


def rotate_covariances(covariances, yaw):
    """Rotate one or more 2x2 covariance matrices by yaw."""
    values = np.asarray(covariances, dtype=np.float64)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    rotation = np.array(
        [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]],
        dtype=np.float64,
    )
    return rotation @ values @ rotation.T


def transform_states(states, transform):
    """Transform planar [x, y, yaw] states into another frame."""
    values = np.asarray(states, dtype=np.float64).copy()
    values[..., :2] = transform_points(values[..., :2], transform)
    values[..., 2] = wrap_angle(values[..., 2] + transform[2])
    return values

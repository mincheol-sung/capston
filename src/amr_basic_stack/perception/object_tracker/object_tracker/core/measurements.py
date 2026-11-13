"""Input contracts for semantic-only analysis and unified object tracking."""
import numpy as np


def _measurement(obj, fallback_sigma, allowed_sources):
    """Return valid XY and SPD covariance for an allowed source, or None.

    Negative off-diagonal covariance is valid; negative eigenvalues are not.
    An all-zero covariance is the established missing-value sentinel.
    """
    if int(obj.source) not in allowed_sources:
        return None
    if not np.isfinite(obj.confidence) or not 0 <= obj.confidence <= 1:
        return None
    position = np.array([obj.position.x, obj.position.y], dtype=float)
    covariance = np.asarray(obj.position_covariance, dtype=float)
    if (not np.all(np.isfinite(position)) or covariance.size != 4
            or not np.all(np.isfinite(covariance))):
        return None
    covariance = covariance.reshape(2, 2)
    if not np.any(covariance):
        covariance = np.eye(2) * fallback_sigma ** 2
    if not np.allclose(covariance, covariance.T, atol=1e-10, rtol=1e-5):
        return None
    covariance = .5 * (covariance + covariance.T)
    if np.linalg.eigvalsh(covariance).min() <= 0:
        return None
    return position, covariance


def semantic_measurement(obj, fallback_sigma):
    """Validate camera or camera/LiDAR fused measurements only."""
    return _measurement(obj, fallback_sigma, (0, 2))


def tracking_measurement(obj, fallback_sigma):
    """Validate camera, LiDAR-only, or fused measurements for tracking."""
    return _measurement(obj, fallback_sigma, (0, 1, 2))

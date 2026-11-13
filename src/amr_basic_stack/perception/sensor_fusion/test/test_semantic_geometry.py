"""Depth units, finite support, mask/letterbox, and bounded surface geometry."""

import numpy as np

from sensor_fusion.semantic_geometry import decode_mask
from sensor_fusion.semantic_geometry import depth_surface
from sensor_fusion.semantic_geometry import surface_statistics


def test_mask_letterbox_and_crop():
    # 8x4 original is letterboxed into 8x8, leaving two rows above/below.
    proto = np.ones((1, 1, 8, 8), dtype=np.float32)
    mask = decode_mask([1], proto, (8, 8), (0, 2, 8, 4), (4, 8, 3), [2, 1, 5, 2])
    assert mask.shape == (4, 8)
    assert mask[1, 3] and not mask[0, 3] and not mask[1, 0]
    assert not decode_mask([-1], proto, (8, 8), (0, 2, 8, 4), (4, 8, 3), [0, 0, 7, 3]).any()


def test_depth_unit_fallback_and_invalid_values():
    d = {'cx': 5.0, 'cy': 5.0, 'w': 8.0, 'h': 8.0}
    k = [[100, 0, 5], [0, 100, 5], [0, 0, 1]]
    mm = np.full((10, 10), 2000, dtype=np.uint16)
    meters = mm.astype(float) * 0.001
    a = depth_surface(d, mm, '16UC1', k, max_points=5)
    b = depth_surface(d, meters, '32FC1', k, max_points=5)
    assert np.allclose(a[0], b[0]) and a[2] == 1 and len(a[0]) == 5
    d['mask'] = np.ones((10, 10), dtype=bool)
    meters[:3] = np.nan
    meters[3] = np.inf
    result = depth_surface(d, meters, '32FC1', k, max_points=5)
    assert result[2] == 0 and result[1] == 0.6 and np.isfinite(result[0]).all()
    assert depth_surface(d, meters, 'mono16', k) is None
    assert depth_surface(d, meters, '32FC1', np.zeros((3, 3))) is None
    position, center, extent, covariance = surface_statistics(result[0])
    assert position[2] == 2 and center[2] == 2
    assert np.all(extent > 0) and np.all(np.linalg.eigvalsh(covariance) > 0)

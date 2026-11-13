"""Bounded mask/depth geometry, independent of ROS and the inference runtime."""

import cv2

import numpy as np


def decode_mask(coefficients, prototypes, input_shape, padding, image_shape, box):
    """Decode one YOLO mask in original pixels, removing letterbox padding."""
    proto = np.asarray(prototypes, dtype=np.float32)
    if proto.ndim == 4:
        proto = proto[0]
    logits = np.asarray(coefficients) @ proto.reshape(proto.shape[0], -1)
    logits = logits.reshape(proto.shape[1:])
    ih, iw = input_shape
    px, py, content_w, content_h = padding
    logits = cv2.resize(logits, (iw, ih), interpolation=cv2.INTER_LINEAR)
    logits = logits[py:py + content_h, px:px + content_w]
    h, w = image_shape[:2]
    logits = cv2.resize(logits, (w, h), interpolation=cv2.INTER_LINEAR)
    support = np.zeros((h, w), dtype=bool)
    x1, y1, x2, y2 = box
    support[
        max(0, int(np.floor(y1))):min(h, int(np.ceil(y2)) + 1),
        max(0, int(np.floor(x1))):min(w, int(np.ceil(x2)) + 1),
    ] = True
    return (logits > 0.0) & support


def depth_surface(
    detection,
    depth,
    encoding,
    intrinsics,
    max_points=512,
    min_pixels=20,
    roi_ratio=0.5,
    min_depth=0.3,
    max_depth=6.0,
    integer_scale=0.001,
):
    """Return sampled optical points and valid ratio; no complete-volume claim."""
    if encoding not in ('16UC1', '32FC1') or depth.ndim != 2:
        return None
    k = np.asarray(intrinsics).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        return None
    h, w = depth.shape
    support = detection.get('mask')
    if support is not None:
        support = np.asarray(support, dtype=bool)
        if support.shape != depth.shape:
            return None
        method = 0
    else:
        support = np.zeros_like(depth, dtype=bool)
        cx, cy = detection['cx'], detection['cy']
        hw = max(0.5, detection['w'] * roi_ratio / 2)
        hh = max(0.5, detection['h'] * roi_ratio / 2)
        x0, x1 = max(0, int(np.floor(cx - hw))), min(w, int(cx + hw) + 1)
        y0, y1 = max(0, int(np.floor(cy - hh))), min(h, int(cy + hh) + 1)
        support[y0:y1, x0:x1] = True
        method = 1
    z = depth.astype(np.float64) * (integer_scale if encoding == '16UC1' else 1.0)
    valid = support & np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    if np.any(valid):
        median = float(np.median(z[valid]))
        mad = float(np.median(np.abs(z[valid] - median)))
        # Reject isolated background depths, retaining broad/slanted surfaces.
        valid &= np.abs(z - median) <= max(0.15, 4.0 * mad)
    count = int(valid.sum())
    if count < min_pixels or max_points < 1:
        return None
    ratio = count / max(1, int(support.sum()))
    v, u = np.nonzero(valid)
    # Uniform deterministic sampling bounds allocations, wire size, and TF work.
    indices = np.linspace(0, count - 1, min(max_points, count), dtype=int)
    v, u = v[indices], u[indices]
    zs = z[v, u]
    pts = np.column_stack(((u - k[0, 2]) * zs / k[0, 0], (v - k[1, 2]) * zs / k[1, 1], zs))
    return pts, float(ratio), method


def surface_statistics(points, noise_floor=0.03):
    """Compute representative point, observed box and conservative position covariance."""
    pts = np.asarray(points, dtype=float)
    position = np.median(pts, axis=0)
    lo, hi = np.quantile(pts, [0.05, 0.95], axis=0)
    # Surface spread is retained, not divided by n as if all pixels were
    # independent observations of an unbiased object centroid.
    cov = np.cov(pts.T) if len(pts) > 1 else np.zeros((3, 3))
    cov += np.eye(3) * noise_floor**2
    return position, (lo + hi) / 2, np.maximum(hi - lo, noise_floor), cov

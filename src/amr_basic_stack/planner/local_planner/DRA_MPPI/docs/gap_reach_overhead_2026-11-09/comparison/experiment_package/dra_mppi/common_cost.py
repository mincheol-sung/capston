"""Shared SI-unit objective for both MPPI strategies."""

import numpy as np

from .geometry import wrap_angle


def project_path(positions, path, lookahead, window, initial_progress=None):
    """Project a batch of time sequences with a bidirectional arc-length window.

    Duplicate vertices are removed. The first unseeded point uses the nearest
    segment (earliest segment breaks ties); subsequent points search within
    max(window, twice the travelled distance) of the preceding projection.
    Reverse motion can decrease progress. A singleton path holds zero heading.
    """
    points = np.asarray(path, dtype=float)[:, :2]
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9]
    points = points[keep]
    xy = np.asarray(positions, dtype=float)
    k, n = xy.shape[:2]
    if len(points) == 1:
        closest = np.broadcast_to(points[0], xy.shape).copy()
        return closest, np.zeros((k, n)), np.zeros((k, n)), np.zeros((k, n)), 0.
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    tangent = segments / lengths[:, None]
    arc = np.r_[0., np.cumsum(lengths)]

    def at(s):
        idx = np.clip(np.searchsorted(arc, s, side='right') - 1, 0, len(lengths)-1)
        return points[idx] + (s - arc[idx])[..., None] * tangent[idx], idx

    closest = np.empty_like(xy)
    progress = np.empty((k, n))
    headings = np.empty((k, n))
    tangents = np.empty((k, n))
    previous = None if initial_progress is None else np.broadcast_to(initial_progress, (k,))
    rows = np.arange(k)
    for t in range(n):
        lower = np.broadcast_to(arc[:-1], (k, len(lengths))).copy()
        upper = np.broadcast_to(arc[1:], lower.shape).copy()
        if previous is not None:
            travel = 0. if t == 0 else 2*np.linalg.norm(xy[:, t]-xy[:, t-1], axis=1)
            reach = np.maximum(window, travel)
            lower = np.maximum(lower, (previous-reach)[:, None])
            upper = np.minimum(upper, (previous+reach)[:, None])
        relative = xy[:, t, None] - points[:-1]
        s = arc[:-1] + np.sum(relative*tangent, axis=-1)
        s = np.minimum(np.maximum(s, lower), upper)
        projections = points[:-1] + (s-arc[:-1])[..., None]*tangent
        squared = np.sum((xy[:, t, None]-projections)**2, axis=-1)
        squared[lower > upper] = np.inf
        idx = np.argmin(squared, axis=1)
        previous = s[rows, idx]
        progress[:, t] = previous
        closest[:, t] = projections[rows, idx]
        ahead, _ = at(np.minimum(previous+lookahead, arc[-1]))
        direction = ahead-closest[:, t]
        zero = np.linalg.norm(direction, axis=-1) < 1e-9
        direction[zero] = tangent[idx[zero]]
        headings[:, t] = np.arctan2(direction[:, 1], direction[:, 0])
        tangents[:, t] = np.arctan2(tangent[idx, 1], tangent[idx, 0])
    return closest, progress, headings, tangents, float(arc[-1])


def common_costs(cfg, state, path, controls, trajectories, current, progress=None,
                 return_components=False):
    """Squared-error objective, with raw SI-unit diagnostics.

    Future state terms use x_1..x_N. Motion uses x_0..x_(N-1).
    v_ref is fixed from the cycle's initial distance and nominal progress clock;
    candidates cannot lower their target by choosing a different trajectory.
    Progress tracks the integral of that common speed reference, capped at the
    path end. All terms are quadratic in their residuals, not globally in the
    controls (path projection, angle wrapping and rollout remain nonlinear).
    The separate near-goal terminal term is retained and explicitly reported.
    """
    initial = np.broadcast_to(state, (len(controls), 1, 3))
    states = np.concatenate((initial, trajectories), axis=1)
    closest, s, heading, _, length = project_path(
        states[..., :2], path, cfg.heading_lookahead, cfg.projection_window, progress)
    raw = {}
    raw['path_error'] = np.mean(np.sum(
        (trajectories[..., :2]-closest[:, 1:])**2, axis=-1), axis=1)
    near_goal = (length-s[:, 0] <= cfg.goal_activation_distance) & (
        np.linalg.norm(state[:2]-path[-1]) <= cfg.goal_activation_distance)
    raw['terminal'] = near_goal*np.sum(
        (trajectories[:, -1, :2]-path[-1])**2, axis=-1)
    raw['heading'] = np.mean(wrap_angle(
        trajectories[..., 2]-heading[:, 1:])**2, axis=1)
    angle = states[:, :-1, 2]-heading[:, :-1]
    parallel = controls[..., 0]*np.cos(angle)-controls[..., 1]*np.sin(angle)
    remaining = np.maximum(length-s[:, 0], np.linalg.norm(state[:2]-path[-1]))
    remaining = np.maximum(0., remaining[:, None]-
                           cfg.desired_speed*cfg.dt*np.arange(controls.shape[1]))
    v_ref = np.minimum(cfg.desired_speed, np.sqrt(
        2*min(cfg.acceleration_limit[:2])*remaining))
    v_ref = np.minimum(v_ref, remaining/cfg.dt)
    target_progress = np.minimum(length, s[:, 0]+cfg.dt*np.sum(v_ref, axis=1))
    raw['progress'] = (s[:, -1]-target_progress)**2
    raw['speed'] = np.mean((parallel-v_ref)**2, axis=1)
    previous = np.broadcast_to(current, (len(controls), 1, 3))
    delta = np.diff(np.concatenate((previous, controls), axis=1), axis=1)
    delta_squared = np.mean(delta**2, axis=1)
    for i, axis in enumerate(('x', 'y', 'omega')):
        raw['delta_'+axis] = delta_squared[:, i]
    gains = dict(path_error=cfg.tracking_path_weight, progress=cfg.progress_weight,
                 terminal=cfg.tracking_goal_weight, heading=cfg.heading_weight,
                 speed=cfg.speed_weight)
    gains.update(zip(('delta_x', 'delta_y', 'delta_omega'), cfg.input_delta_weight))
    weighted = {name: value*gains[name] for name, value in raw.items()}
    costs = (weighted['path_error']+weighted['progress']+weighted['terminal'],
             weighted['heading'], weighted['speed'],
             weighted['delta_x']+weighted['delta_y']+weighted['delta_omega'])
    return (*costs, raw, weighted) if return_components else costs

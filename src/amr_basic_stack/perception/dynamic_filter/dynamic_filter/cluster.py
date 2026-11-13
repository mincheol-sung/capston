"""Scan-local components associated with causal IMM positions, without radius fallback."""

from collections import defaultdict
import math

import numpy as np


def segment_scan(scan, values, valid, config):
    """Keep original beam indices; split at invalid beams and adaptive spatial gaps."""
    step = abs(scan.angle_increment)
    if not 0 < step < math.pi:
        return []
    indices = np.flatnonzero(valid)
    if not len(indices):
        return []
    angles = scan.angle_min + indices * scan.angle_increment
    native = values[indices, None] * np.column_stack((np.cos(angles), np.sin(angles)))

    def connected(a, b):
        limit = min(config.cluster_max_gap, config.cluster_base_gap
                    + config.cluster_gap_scale * min(values[indices[a]], values[indices[b]])
                    * step)
        return np.linalg.norm(native[a] - native[b]) <= limit

    limits = np.minimum(config.cluster_max_gap, config.cluster_base_gap
                        + config.cluster_gap_scale
                        * np.minimum(values[indices[:-1]], values[indices[1:]]) * step)
    breaks = np.flatnonzero((np.diff(indices) != 1)
                            | (np.linalg.norm(np.diff(native, axis=0), axis=1) > limits)) + 1
    groups = list(np.split(indices, breaks))
    # Only a full revolution can join across the array seam. Missing edge beams
    # and partial-FOV scans never become artificial neighbours.
    span = step * (len(values) - 1)
    full_circle = abs(span - 2*math.pi) <= 1.01*step
    if (len(groups) > 1 and full_circle and indices[0] == 0
            and indices[-1] == len(values)-1 and connected(-1, 0)):
        groups = [np.concatenate((groups[-1], groups[0]))] + groups[1:-1]
    return groups


def cluster_mask(scan, values, valid, points, centres, active, target_active, config, stats):
    """
    Match whole components; ambiguity/oversized groups pass through unchanged.

    Distances use beam-time odom points minus CV-extrapolated track positions.
    This is geometric reassociation, not an upstream observation-to-track ID
    guarantee. No covariance model or multi-frame shape history is claimed.
    """
    removed = np.zeros(len(values), dtype=bool)
    groups = segment_scan(scan, values, valid, config)
    stats['clusters_total'] = len(groups)
    if not groups:
        stats['cluster_invalid_geometry'] += 1
        return removed
    proposals = defaultdict(list)
    for members in groups:
        if len(members) < config.cluster_min_points:
            stats['clusters_small'] += 1
            continue
        if np.linalg.norm(np.ptp(points[members], axis=0)) > config.cluster_max_span:
            stats['clusters_oversized'] += 1
            continue
        relative = points[None, members, :] - centres[:, members, :]
        scores = np.linalg.norm(relative.mean(axis=1), axis=1)
        # Rank all temporally valid competitors, including non-person/static
        # tracks. Extent/policy gates are applied only after ambiguity checking.
        candidates = np.flatnonzero(active[:, members].all(axis=1)
                                    & (scores < config.cluster_match_distance))
        if not len(candidates):
            stats['clusters_unmatched'] += 1
            continue
        order = candidates[np.argsort(scores[candidates], kind='stable')]
        winner = order[0]
        if (len(order) > 1
                and scores[order[1]] - scores[winner] < config.cluster_ambiguity_margin):
            stats['clusters_ambiguous'] += 1
            continue
        if not target_active[winner, members].all():
            stats['clusters_non_target_or_expired'] += 1
            continue
        if np.max(np.linalg.norm(relative[winner], axis=1)) >= config.cluster_max_point_distance:
            stats['clusters_extent_rejected'] += 1
            continue
        proposals[int(winner)].append(members)
    for winner, components in proposals.items():
        # A person may produce two leg components. Do not pick arbitrary
        # fragments when clutter produces too many plausible components.
        if len(components) > config.cluster_max_per_track:
            stats['clusters_track_limit_rejected'] += len(components)
            continue
        members = np.concatenate(components)
        relative = points[members] - centres[winner, members]
        if np.linalg.norm(np.ptp(relative, axis=0)) > config.cluster_max_span:
            stats['clusters_group_extent_rejected'] += len(components)
            continue
        removed[members] = True
        stats['clusters_matched'] += len(components)
        stats['used_tracks'] += 1
    return removed

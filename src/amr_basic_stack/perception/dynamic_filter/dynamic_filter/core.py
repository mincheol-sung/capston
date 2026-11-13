"""ROS-independent, causal radius or cluster masking (planar odom distance)."""

from bisect import bisect_right
from collections import Counter
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np

from .cluster import cluster_mask


@dataclass(frozen=True)
class FilterConfig:
    """Initial engineering limits; durations are seconds, radius is metres."""

    filter_radius: float = 0.35
    filter_policy: str = 'confirmed_person'
    mask_method: str = 'radius'
    # Only verified scan contracts permit geometry-based removal. Membership
    # masking uses original indices and does not depend on this parameter.
    scan_timing_mode: str = 'unknown'
    observation_wait_timeout: float = 0.30
    max_observation_lag: float = 0.75
    max_observation_links: int = 2000
    cluster_min_points: int = 3
    cluster_base_gap: float = 0.05
    cluster_gap_scale: float = 2.0
    cluster_max_gap: float = 0.15
    cluster_max_span: float = 0.80
    cluster_match_distance: float = 0.45
    cluster_max_point_distance: float = 0.60
    cluster_ambiguity_margin: float = 0.10
    cluster_max_per_track: int = 2
    max_extrapolation: float = 0.30
    max_measurement_age: float = 0.30
    person_label_timeout: float = 1.0
    history_duration: float = 2.0
    history_size: int = 100
    pending_queue_size: int = 5
    tf_wait_timeout: float = 0.15
    tf_cache_duration: float = 10.0
    retry_period: float = 0.01
    diagnostic_period: float = 1.0
    max_scan_beams: int = 10000
    max_tracks: int = 200

    def __post_init__(self):
        if self.filter_policy not in ('confirmed_person', 'dynamic_only'):
            raise ValueError('filter_policy must be confirmed_person or dynamic_only')
        if self.mask_method not in ('radius', 'cluster', 'tracked_cluster'):
            raise ValueError('mask_method must be radius, cluster or tracked_cluster')
        if self.scan_timing_mode not in ('unknown', 'ros', 'reversed'):
            raise ValueError('scan_timing_mode must be unknown, ros or reversed')
        for name in ('filter_radius', 'max_extrapolation', 'max_measurement_age',
                     'person_label_timeout', 'history_duration', 'tf_wait_timeout',
                     'tf_cache_duration', 'retry_period', 'diagnostic_period',
                     'cluster_base_gap', 'cluster_gap_scale', 'cluster_max_gap',
                     'cluster_max_span', 'cluster_match_distance',
                     'cluster_max_point_distance', 'cluster_ambiguity_margin',
                     'observation_wait_timeout', 'max_observation_lag'):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name, upper in (('history_size', 10000), ('pending_queue_size', 1000),
                            ('max_scan_beams', 100000), ('max_tracks', 10000),
                            ('cluster_min_points', 10000), ('cluster_max_per_track', 10),
                            ('max_observation_links', 100000)):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= upper:
                raise ValueError(f'{name} must be an integer in [1, {upper}]')
        if self.cluster_base_gap > self.cluster_max_gap:
            raise ValueError('cluster_base_gap must not exceed cluster_max_gap')
        if self.cluster_match_distance > self.cluster_max_point_distance:
            raise ValueError('cluster_match_distance must not exceed cluster_max_point_distance')
        if self.history_duration < self.max_extrapolation:
            raise ValueError('history_duration must cover max_extrapolation')
        if self.tf_cache_duration < self.history_duration + self.tf_wait_timeout:
            raise ValueError('tf_cache_duration must cover history_duration + tf_wait_timeout')
        if self.retry_period > self.tf_wait_timeout:
            raise ValueError('retry_period must not exceed tf_wait_timeout')
        if self.retry_period > self.observation_wait_timeout:
            raise ValueError('retry_period must not exceed observation_wait_timeout')
        if self.history_duration < self.observation_wait_timeout:
            raise ValueError('history_duration must cover observation_wait_timeout')


def stamp_ns(stamp):
    """Keep epoch comparisons exact; zero is a real epoch, never latest TF."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class TrackHistory:
    """Bounded snapshots, including empty arrays; never fall back past a clear."""

    def __init__(self, config):
        self.config = config
        self._entries = []

    def clear(self):
        self._entries.clear()

    def add(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        stamps = [entry[0] for entry in self._entries]
        index = bisect_right(stamps, stamp)
        if index and stamps[index - 1] == stamp:
            self._entries[index - 1] = (stamp, msg)
        else:
            self._entries.insert(index, (stamp, msg))
        cutoff = self._entries[-1][0] - round(self.config.history_duration * 1e9)
        self._entries = [entry for entry in self._entries if entry[0] >= cutoff]
        self._entries = self._entries[-self.config.history_size:]

    def select(self, scan_ns):
        if not self._entries:
            return None, 'tracks_missing'
        index = bisect_right([entry[0] for entry in self._entries], scan_ns) - 1
        if index < 0:
            return None, 'no_causal_tracks'
        return self._entries[index][1], ''


class TransformUnavailable(Exception):
    """Adapter could not obtain a finite transform at the requested beam epoch."""


def rigid_matrix(translation, quaternion):
    """Full 3-D sensor transform; only its odom XY projection is masked."""
    q = np.asarray(quaternion, dtype=float)
    t = np.asarray(translation, dtype=float)
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(t)) or np.linalg.norm(q) < 1e-12:
        raise TransformUnavailable('invalid transform')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), t[0]],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w), t[1]],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y), t[2]],
    ])


def eligible_tracks(snapshot, config, confirmed, dynamic, diagnostics, *, apply_policy=True):
    """Use published labels/state constants, not a guessed detector class ID."""
    tracks = []
    for obj in snapshot.objects:
        if obj.lifecycle_state != confirmed:
            diagnostics['lifecycle_excluded'] += 1
            continue
        if apply_policy and config.filter_policy == 'confirmed_person':
            if obj.class_name.strip().lower() != 'person':
                continue
            if not math.isfinite(obj.semantic_age) or obj.semantic_age < 0:
                diagnostics['semantic_expired'] += 1
                continue
        elif apply_policy and obj.motion_state != dynamic:
            continue
        if not math.isfinite(obj.measurement_age) or obj.measurement_age < 0:
            diagnostics['invalid_tracks'] += 1
            continue
        probabilities = np.asarray([mode.probability for mode in obj.modes], dtype=float)
        states = np.asarray([mode.state for mode in obj.modes], dtype=float)
        if (not len(probabilities) or states.shape != (len(probabilities), 5)
                or not np.all(np.isfinite(states))
                or not np.all(np.isfinite(probabilities)) or np.any(probabilities < 0)
                or not math.isfinite(float(probabilities.sum())) or probabilities.sum() <= 0):
            diagnostics['invalid_tracks'] += 1
            continue
        state = (probabilities / probabilities.sum()) @ states
        tracks.append((obj, state))
    return tracks


def mask_scan(scan, snapshot, config, transform_at, *, confirmed, dynamic,
              missing_reason='tracks_missing'):
    """
    Return a new scan and diagnostics; a TF failure rolls back the whole mask.

    transform_at(frame, nanoseconds) must be nonblocking and return a 3x4
    odom<-sensor matrix. The weighted IMM mean receives only CV extrapolation.
    This is not IMM mode prediction, covariance propagation or shape estimation.
    """
    if config.mask_method == 'tracked_cluster':
        raise ValueError('tracked_cluster requires observation links, not IMM radius fallback')
    out = deepcopy(scan)
    stats = Counter(valid_points=0, removed_points=0, used_tracks=0,
                    zero_time_increment=int(scan.time_increment == 0))

    def finish(reason):
        stats['reason'] = reason
        return out, dict(stats)

    if len(scan.ranges) > config.max_scan_beams:
        return finish('scan_limit')
    values = np.asarray(scan.ranges, dtype=float)
    valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
    stats['valid_points'] = int(valid.sum())
    if not len(values):
        return finish('empty_scan')
    if (not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_increment,
                                           scan.time_increment, scan.range_min,
                                           scan.range_max))
            or scan.time_increment < 0 or scan.range_min < 0 or scan.range_max < scan.range_min):
        return finish('invalid_scan_metadata')
    if not valid.any():
        return finish('no_valid_points')
    if config.scan_timing_mode == 'unknown':
        return finish('scan_timing_unknown')
    if snapshot is None:
        return finish(missing_reason)
    if snapshot.header.frame_id != 'odom':
        return finish('unsupported_track_frame')
    if len(snapshot.objects) > config.max_tracks:
        return finish('track_limit')
    if not snapshot.objects:
        return finish('empty_tracks')
    start = stamp_ns(scan.header.stamp)
    epoch = stamp_ns(snapshot.header.stamp)
    if epoch > start:
        return finish('no_causal_tracks')
    tracks = eligible_tracks(snapshot, config, confirmed, dynamic, stats)
    if not tracks:
        return finish('no_eligible_tracks')
    targets = {id(obj) for obj, _ in tracks}
    if config.mask_method == 'cluster':
        # Non-target confirmed tracks must compete too: a nearby chair/unknown
        # must not silently become a person's cluster after policy filtering.
        tracks = eligible_tracks(snapshot, config, confirmed, dynamic, Counter(),
                                 apply_policy=False)
    offsets = np.arange(len(values)) * scan.time_increment
    if config.scan_timing_mode == 'reversed':
        offsets = offsets[::-1]
    dt = (start - epoch) * 1e-9 + offsets
    allowed_time = dt <= config.max_extrapolation
    stats['extrapolation_limit_beams'] = int(np.count_nonzero(valid & ~allowed_time))
    centres, active, target_active = [], [], []
    for obj, state in tracks:
        recent = obj.measurement_age + dt <= config.max_measurement_age
        stats['measurement_expired_beams'] += int(np.count_nonzero(valid & ~recent))
        policy_valid = np.full(len(values), id(obj) in targets, dtype=bool)
        if config.filter_policy == 'confirmed_person' and id(obj) in targets:
            semantic = obj.semantic_age + dt <= config.person_label_timeout
            stats['semantic_expired_beams'] += int(np.count_nonzero(valid & ~semantic))
            policy_valid &= semantic
            if config.mask_method == 'radius':
                recent &= semantic
        usable = valid & allowed_time & recent
        if usable.any():
            centres.append(state[:2] + dt[:, None] * state[2:4])
            active.append(usable)
            target_active.append(usable & policy_valid)
    if not active:
        return finish('tracks_expired')
    active = np.asarray(active)
    centres = np.asarray(centres)
    target_active = np.asarray(target_active)
    if config.mask_method == 'cluster' and not target_active.any():
        return finish('tracks_expired')
    # Cluster extent must include the entire observed component, including its
    # expired tail. Never crop a wall into a small removable fragment by age.
    indices = np.flatnonzero(valid if config.mask_method == 'cluster' else active.any(axis=0))
    points = np.zeros((len(values), 2))
    try:
        # Endpoint checks avoid thousands of partial lookups while TF catches up.
        endpoints = {int(i): transform_at(scan.header.frame_id,
                     start + round(offsets[i] * 1e9))
                     for i in (indices[0], indices[-1])}
        shared = endpoints[int(indices[0])] if scan.time_increment == 0 else None
        for i in indices:
            matrix = shared if shared is not None else endpoints.get(int(i))
            if matrix is None:
                matrix = transform_at(scan.header.frame_id,
                                      start + round(offsets[i] * 1e9))
            angle = scan.angle_min + i * scan.angle_increment
            point = matrix @ np.array([values[i]*math.cos(angle), values[i]*math.sin(angle), 0, 1])
            if not np.all(np.isfinite(point)):
                raise TransformUnavailable('nonfinite point')
            points[i] = point[:2]
    except TransformUnavailable:
        stats['tf_failures'] = 1
        return finish('tf_unavailable')
    if config.mask_method == 'cluster':
        removed = cluster_mask(scan, values, valid, points, centres, active,
                               target_active, config, stats)
    else:
        distances2 = np.sum((centres - points[None, :, :])**2, axis=2)
        removed = np.any(active & (distances2 < config.filter_radius**2), axis=0)
    values[removed] = math.nan
    out.ranges = values.tolist()
    stats['removed_points'] = int(removed.sum())
    if config.mask_method == 'radius':
        stats['used_tracks'] = len(active)
    empty_reason = 'no_cluster_match' if config.mask_method == 'cluster' else 'no_points_in_radius'
    return finish('filtered' if removed.any() else empty_reason)

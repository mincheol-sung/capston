"""Exact original-scan membership masking using actual tracker assignments."""

from collections import Counter, deque
from copy import deepcopy
import math

import numpy as np

from .core import stamp_ns


class ObservationHistory:
    """Bounded incremental events with receipt times for strict steady deadlines."""

    def __init__(self, config):
        self.config = config
        self.entries = deque(maxlen=config.max_observation_links)

    def clear(self):
        self.entries.clear()

    def add(self, message, received):
        if len(message.links) > self.config.max_observation_links:
            return False
        # Refuse oversized memberships before retaining the message payload.
        if any(len(link.observation.beam_indices) > self.config.max_scan_beams
               or len(link.observation.beam_time_offsets) > self.config.max_scan_beams
               for link in message.links):
            return False
        self.prune(received)
        epoch = stamp_ns(message.header.stamp)
        self.entries.extend((epoch, link, received) for link in message.links)
        return True

    def prune(self, now):
        cutoff = now - self.config.history_duration
        while self.entries and self.entries[0][2] < cutoff:
            self.entries.popleft()

    def select(self, scan, sensor_id, deadline):
        """Only exact sensor/frame/capture epoch, received before this scan's deadline."""
        key = (sensor_id, scan.header.frame_id, stamp_ns(scan.header.stamp))
        return [(epoch, link) for epoch, link, received in self.entries
                if received <= deadline and (
                    link.observation.sensor_id, link.observation.header.frame_id,
                    stamp_ns(link.observation.header.stamp)) == key]


def mask_observations(scan, records, sensor_id, config, *, confirmed, dynamic):
    """Mask consumed memberships, preserving all metadata and unrelated ranges."""
    out = deepcopy(scan)
    stats = Counter(valid_points=0, removed_points=0, used_tracks=0,
                    zero_time_increment=int(scan.time_increment == 0),
                    observation_links=len(records))

    def finish(reason):
        stats['reason'] = reason
        return out, dict(stats)

    if len(scan.ranges) > config.max_scan_beams:
        return finish('scan_limit')
    if not scan.ranges:
        return finish('empty_scan')
    if (not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_increment,
                                           scan.time_increment, scan.range_min, scan.range_max))
            or scan.time_increment < 0 or scan.range_min < 0 or scan.range_max < scan.range_min):
        return finish('invalid_scan_metadata')
    values = np.asarray(scan.ranges, dtype=float)
    valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
    stats['valid_points'] = int(valid.sum())
    if not valid.any():
        return finish('no_valid_points')
    start = stamp_ns(scan.header.stamp)
    if start <= 0 or not scan.header.frame_id or not sensor_id:
        return finish('invalid_observation_identity')
    if not records:
        return finish('observations_missing')
    if len(records) > config.max_observation_links:
        return finish('observation_limit')
    accepted, owners, blocked = [], {}, set()
    for epoch, link in records:
        obs = link.observation
        if (obs.sensor_id != sensor_id or obs.header.frame_id != scan.header.frame_id
                or stamp_ns(obs.header.stamp) != start):
            stats['observation_identity_rejected'] += 1
            continue
        indices = list(obs.beam_indices)
        if (not obs.observation_id or not indices or len(indices) > config.max_scan_beams
                or len(indices) != len(set(indices))
                or any(i < 0 or i >= len(values) for i in indices)):
            stats['observation_membership_rejected'] += 1
            continue
        # Any competing ownership (including non-target or expired labels)
        # vetoes the whole component rather than choosing an arbitrary owner.
        for i in indices:
            owners.setdefault(i, set()).add(link.track_id)
        lag = (epoch - start)*1e-9
        eligible = (0 <= lag <= config.max_observation_lag
                    and link.lifecycle_state == confirmed)
        if config.filter_policy == 'confirmed_person':
            eligible &= (link.class_name.strip().lower() == 'person'
                         and math.isfinite(link.semantic_age)
                         and 0 <= link.semantic_age <= config.person_label_timeout)
        else:
            eligible &= link.motion_state == dynamic
        offsets = list(obs.beam_time_offsets)
        if offsets:
            # The upstream driver may report reversed chronological order.
            # Membership is index based; do not invent index*time_increment.
            offsets_array = np.asarray(offsets)
            duration = (len(values)-1)*scan.time_increment
            if (len(offsets) != len(indices) or not np.isfinite(offsets).all()
                    or np.any(offsets_array < 0)
                    or np.any(offsets_array > min(duration, lag) + 1e-7)):
                eligible = False
                stats['observation_timing_rejected'] += 1
        elif getattr(obs, 'timing_status', 0) == 1:
            eligible = False
            stats['observation_timing_rejected'] += 1
        if not eligible:
            blocked.update(indices)
            stats['observation_policy_or_age_rejected'] += 1
        else:
            accepted.append((link.track_id, indices))
    blocked.update(i for i, ids in owners.items() if len(ids) > 1)
    removed = np.zeros(len(values), dtype=bool)
    used = set()
    for track_id, indices in accepted:
        if any(i in blocked for i in indices):
            stats['observation_conflicts'] += 1
            continue
        selected = np.asarray(indices, dtype=int)
        selected = selected[valid[selected]]
        if len(selected):
            removed[selected] = True
            used.add(track_id)
            stats['observations_masked'] += 1
    values[removed] = math.nan
    out.ranges = values.tolist()
    stats['removed_points'] = int(removed.sum())
    stats['used_tracks'] = len(used)
    return finish('filtered' if removed.any() else 'no_observation_match')

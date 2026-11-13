"""Sensor time contracts and batched, nonblocking rigid-body deskew."""
from collections import Counter
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def oldest_capture_ns(message):
    """Age the original capture even if coordinates were moved to a later epoch."""
    stamps = [stamp_ns(message.header.stamp)]
    for record in [message, *message.objects]:
        stamps.extend(stamp_ns(obs.header.stamp) for obs in record.observations)
    return min(stamps)


def beam_offsets(count, increment, mode, max_duration=0.5):
    """
    Only explicitly verified driver contracts may supply beam time.

    ros: stamp is measurement of ranges[0], indices are chronological.
    reversed: stamp is earliest measurement, ranges[-1] is earliest.
    unknown: including the bundled sorted/angle-compensated RPLIDAR driver.
    """
    if mode not in ('ros', 'reversed', 'unknown'):
        raise ValueError('timing mode must be ros, reversed or unknown')
    if (mode == 'unknown' or count < 1 or not np.isfinite(increment)
            or increment <= 0 or (count - 1) * increment > max_duration):
        return None
    indices = np.arange(count, dtype=float)
    if mode == 'reversed':
        indices = indices[::-1]
    return indices * increment


def rigid(tf):
    """Convert a transform to rotation matrix and translation (metres)."""
    q, t = tf.transform.rotation, tf.transform.translation
    return (Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
            np.array([t.x, t.y, t.z]))


def deskew(points, times, lookup, knot_interval=0.01):
    """
    Interpolate SE(3) knots; lookup never blocks and may raise for retry.

    All transforms map source(t) to the SAME target reference epoch. SLERP
    rotates full 3D geometry; linear translation includes the sensor lever arm.
    Query count is bounded by scan duration, not beam count. No extrapolation.
    """
    times = np.asarray(times, dtype=float)
    if len(points) == 0:
        return np.empty((0, 3))
    lo, hi = float(times.min()), float(times.max())
    if hi == lo:
        rotation, translation = lookup(lo)
        return points @ rotation.T + translation
    knots = np.linspace(lo, hi, max(2, int(np.ceil((hi-lo)/knot_interval)) + 1))
    transforms = [lookup(float(t)) for t in knots]
    rotations = Rotation.from_matrix(np.stack([r for r, _ in transforms]))
    translations = np.stack([t for _, t in transforms])
    rotated = Slerp(knots, rotations)(times).apply(points)
    return rotated + np.column_stack([
        np.interp(times, knots, translations[:, axis]) for axis in range(3)])


def observation_window(obj):
    times = []
    for obs in obj.observations:
        start = stamp_ns(obs.header.stamp)*1e-9
        times.extend(start+offset for offset in (obs.beam_time_offsets or [0.0]))
    return (min(times), max(times)) if times else None


def temporal_variance(obj, reference_ns, sigma_v, unknown_sigma):
    """
    Unknown object velocity: zero mean, isotropic sigma_v in m/s.

    Use the most distant constituent measurement, including intra-scan time.
    Unknown beam timing carries an explicit extra variance, never invented time.
    """
    horizon = 0.0
    unknown = not obj.observations
    for obs in obj.observations:
        delta = (reference_ns - stamp_ns(obs.header.stamp)) * 1e-9
        if obs.timing_status != 1:
            unknown = True
        offsets = obs.beam_time_offsets or [0.0]
        horizon = max(horizon, max(abs(delta - offset) for offset in offsets))
    return (sigma_v * horizon)**2 + (unknown_sigma**2 if unknown else 0.0)


@dataclass
class Entry:
    stamp: int
    stream: str
    message: object
    received: float


class FusionQueue:
    """
    Bounded reorder/partner wait with a committed epoch watermark.

    Each stream/stamp is consumed at most once. Anything at/before a committed
    epoch is discarded, including late camera recovery. A batch has at most one
    array per stream (camera/front/rear); all are aligned to its newest epoch.
    """

    def __init__(self, wait=0.10, tf_wait=0.25, capacity=32, slop=0.067,
                 max_age=0.75):
        if min(wait, tf_wait, slop, max_age) <= 0 or capacity < 1:
            raise ValueError('queue bounds must be positive')
        self.wait, self.tf_wait, self.capacity = wait, tf_wait, capacity
        self.slop_ns, self.max_age = int(slop*1e9), max_age
        self.entries = []
        self.watermark = -1
        self.counts = Counter()

    def add(self, stream, message, now, ros_ns):
        stamp = stamp_ns(message.header.stamp)
        capture = oldest_capture_ns(message)
        if capture <= 0 or stamp > ros_ns + 50_000_000:
            self.counts['invalid_or_future'] += 1
            return
        if stamp <= self.watermark:
            self.counts['late_or_duplicate'] += 1
            return
        if any(e.stamp == stamp and e.stream == stream for e in self.entries):
            self.counts['duplicate'] += 1
            return
        if (ros_ns-capture)*1e-9 > self.max_age:
            self.counts['stale'] += 1
            return
        if self.entries and stamp < max(e.stamp for e in self.entries):
            self.counts['reordered'] += 1
        self.entries.append(Entry(stamp, stream, message, now))
        self.entries.sort(key=lambda e: (e.stamp, e.stream))
        if len(self.entries) > self.capacity:
            self.entries.pop(0)
            self.counts['overflow'] += 1

    def drain(self, now, ros_ns, consume):
        while self.entries:
            first = self.entries[0]
            if (ros_ns-oldest_capture_ns(first.message))*1e-9 > self.max_age:
                self.entries.pop(0)
                self.counts['stale'] += 1
                continue
            if now-first.received < self.wait:
                return
            batch, streams = [], set()
            for entry in self.entries:
                if entry.stamp-first.stamp > self.slop_ns:
                    break
                if entry.stream not in streams:
                    batch.append(entry)
                    streams.add(entry.stream)
                else:
                    # Do not leap over an unconsumed epoch of the same sensor.
                    break
            reference = max(e.stamp for e in batch)
            expired = [e for e in batch
                       if (ros_ns-oldest_capture_ns(e.message))*1e-9 > self.max_age]
            if expired:
                expired_ids = {id(e) for e in expired}
                self.entries = [e for e in self.entries if id(e) not in expired_ids]
                self.counts['stale'] += len(expired)
                continue
            if not consume(batch, reference):
                self.counts['tf_wait'] += 1
                if now-first.received < self.wait+self.tf_wait:
                    return
                self.counts['tf_timeout'] += len(batch)
            self.watermark = reference
            ids = {id(e) for e in batch}
            self.entries = [e for e in self.entries if id(e) not in ids]
            remaining = [e for e in self.entries if e.stamp > self.watermark]
            self.counts['late_or_duplicate'] += len(self.entries)-len(remaining)
            self.entries = remaining

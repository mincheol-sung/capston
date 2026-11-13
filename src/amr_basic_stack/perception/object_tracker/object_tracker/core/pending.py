"""Bounded timestamp-ordered input queue with nonblocking transform retries."""
import heapq
from collections import Counter


class PendingMeasurements:
    """Wait using monotonic deadlines; never rewind the measurement timeline."""

    def __init__(self, max_wait=0.25, capacity=32):
        """Configure the queue's monotonic deadline and memory bound."""
        if max_wait <= 0 or capacity < 1:
            raise ValueError('max_wait and capacity must be positive')
        self.max_wait = max_wait
        self.capacity = capacity
        self.last_stamp = None
        self.dropped = Counter()
        self._heap = []
        self._stamps = set()

    def add(self, stamp, message, now):
        """Enqueue an input unless its timestamp was already processed/queued."""
        too_old = self.last_stamp is not None and stamp <= self.last_stamp
        if too_old or stamp in self._stamps:
            self.dropped['non_increasing'] += 1
            return
        heapq.heappush(self._heap, (stamp, now, message))
        self._stamps.add(stamp)
        if len(self._heap) > self.capacity:
            old, _, _ = heapq.heappop(self._heap)
            self._stamps.remove(old)
            self.dropped['overflow'] += 1

    def drain(self, now, lookup, consume):
        """Run callbacks for ready oldest inputs, without blocking TF reception."""
        while self._heap:
            stamp, received, message = self._heap[0]
            if now - received > self.max_wait:
                heapq.heappop(self._heap)
                self._stamps.remove(stamp)
                self.dropped['tf_expired'] += 1
                continue
            transform = lookup(message)
            if transform is None:
                return
            heapq.heappop(self._heap)
            self._stamps.remove(stamp)
            self.last_stamp = stamp
            consume(message, transform)

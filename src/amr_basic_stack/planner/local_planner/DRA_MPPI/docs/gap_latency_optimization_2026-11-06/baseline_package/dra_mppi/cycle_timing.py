"""Optional bounded asynchronous JSONL recorder; no per-cycle console logging."""

import json
import math
from pathlib import Path
import queue
import threading


class CycleRecorder:
    def __init__(self, filename, capacity=32768):
        self.queue = queue.Queue(maxsize=capacity)
        self.dropped = 0
        self.written = 0
        self.error = None
        self.stream = Path(filename).open('w', buffering=1024*1024)
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def record(self, row):
        try:
            self.queue.put_nowait(dict(row))
        except queue.Full:
            self.dropped += 1

    def _write(self):
        try:
            while True:
                row = self.queue.get()
                if row is None:
                    break
                self.stream.write(json.dumps(_finite_json(row), allow_nan=False)+'\n')
                self.written += 1
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        if self.worker.is_alive():
            self.queue.put(None)
            self.worker.join(timeout=10.)
        if not self.worker.is_alive():
            self.stream.write(json.dumps({
                'recorder_summary': True, 'written': self.written,
                'dropped': self.dropped, 'error': self.error})+'\n')
            self.stream.close()


def _finite_json(value):
    """Retain invalid safety costs as null, never lose entire timing records."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    return value

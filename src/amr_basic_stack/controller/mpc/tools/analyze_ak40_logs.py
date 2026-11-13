#!/usr/bin/env python3
"""Read-only bench-log analysis. Print reproducible JSON; never opens CAN."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def analyze(root):
    runs = []
    all_delays = []
    for path in sorted(root.glob('*/samples.csv')):
        meta_path = path.with_name('metadata.json')
        meta = json.loads(meta_path.read_text())
        rows = list(csv.DictReader(path.open()))
        received = [r for r in rows if r['rx_ok'] == '1']
        delays = [float(r['response_ms']) for r in received]
        gaps = np.diff([float(r['rx_time_s']) for r in received]) * 1000
        all_delays.extend(delays)
        runs.append({
            'directory': path.parent.name,
            'samples_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'metadata_sha256': hashlib.sha256(meta_path.read_bytes()).hexdigest(),
            'motor_ids': meta['motor_ids'], 'simulation': meta['simulation'],
            'status': meta['status'], 'reason': meta.get('reason'),
            'cleanup_errors': meta.get('cleanup_errors', []),
            'rows': len(rows), 'received': len(received),
            'missing_rows': len(rows) - len(received),
            'response_ms': dict(zip(('p50', 'p99', 'p99_9', 'max'),
                                   map(float, np.percentile(delays, [50, 99, 99.9, 100])))),
            'response_over_4ms': sum(x > 4 for x in delays),
            'max_rx_gap_ms': float(max(gaps)) if len(gaps) else None,
            'nonzero_error_rows': sum(int(r['error']) != 0 for r in received),
            'payload_lengths': sorted({len(bytes.fromhex(r['payload_hex'])) for r in received}),
            'max_abs_command_Nm': max(abs(float(r['torque_encoded_nm'])) for r in received),
            'max_temperature_C': max(float(r['temperature_c']) for r in received),
        })
    if not runs:
        raise ValueError(f'No bench samples.csv found under {root}')
    return {
        'source': str(root.resolve()),
        'scope': 'Single-motor bench runs, including aborted runs and superseded RR runs. Not four-motor latency validation.',
        'runs': runs,
        'total_rows': sum(r['rows'] for r in runs),
        'missing_rows': sum(r['missing_rows'] for r in runs),
        'max_response_ms': max(all_delays),
        'response_over_4ms': sum(x > 4 for x in all_delays),
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log_root', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.log_root), ensure_ascii=False, indent=2, allow_nan=False))

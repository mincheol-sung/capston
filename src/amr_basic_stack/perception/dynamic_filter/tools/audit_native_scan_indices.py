#!/usr/bin/env python3
"""Check original beam indices against Cartographer scans in a recorded bag."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py as bag_api
from sensor_msgs.msg import LaserScan


def main():
    """Compare every paired scan without interpreting intensities as beam identities."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reader = bag_api.SequentialReader()
    reader.open(bag_api.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                bag_api.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(bag_api.StorageFilter(topics=[
        '/scan_front', '/scan_rear', '/scan_front_carto', '/scan_rear_carto']))
    pending = {'native': {}, 'carto': {}}
    counts = Counter(paired_scans=0, geometry_mismatch_scans=0,
                     finite_range_mismatches=0, identical_finite_ranges=0,
                     intensity_mismatch_scans=0)
    geometry = ('angle_min', 'angle_max', 'angle_increment', 'time_increment',
                'scan_time', 'range_min', 'range_max')
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        scan = deserialize_message(raw, LaserScan)
        key = (topic.removesuffix('_carto'), scan.header.frame_id,
               scan.header.stamp.sec, scan.header.stamp.nanosec)
        kind = 'carto' if topic.endswith('_carto') else 'native'
        assert key not in pending[kind], 'Duplicate source scan identity'
        pending[kind][key] = scan
        if key not in pending['native'] or key not in pending['carto']:
            continue
        native, carto = (pending[k].pop(key) for k in ('native', 'carto'))
        counts['paired_scans'] += 1
        mismatch = (len(native.ranges) != len(carto.ranges)
                    or any(getattr(native, k) != getattr(carto, k) for k in geometry))
        counts['geometry_mismatch_scans'] += int(mismatch)
        if mismatch:
            continue
        original, processed = np.asarray(native.ranges), np.asarray(carto.ranges)
        finite = np.isfinite(processed)
        counts['finite_range_mismatches'] += int(np.count_nonzero(
            original[finite] != processed[finite]))
        counts['identical_finite_ranges'] += int(np.count_nonzero(
            original[finite] == processed[finite]))
        counts['intensity_mismatch_scans'] += int(not np.array_equal(
            native.intensities, carto.intensities, equal_nan=True))
    result = {'bag': str(args.bag), 'checks': dict(counts),
              'unpaired_records': {k: len(v) for k, v in pending.items()},
              'note': 'Native/cartographer intensities may differ. Membership requires '
                      'matching sensor/frame/stamp, layout and retained finite ranges.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    assert not pending['carto'], 'Some Cartographer scans have no native scan'
    assert counts['geometry_mismatch_scans'] == 0
    assert counts['finite_range_mismatches'] == 0
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

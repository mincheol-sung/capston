#!/usr/bin/env python3
"""
Read-only native scan/TF audit; never publishes topics or changes a bag.

Reports the difference between leaving a native point unchanged and expressing
it 50ms later using recorded odom. This is a motion correction magnitude, NOT
an accuracy score: bundled RPLIDAR beam times and object ground truth are unknown.
"""
import argparse
from collections import Counter
import json

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from sensor_fusion.observation_time import beam_offsets, rigid, stamp_ns
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


def audit(path, seconds):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path), ConverterOptions('', ''))
    topics = reader.get_all_topics_and_types()
    reader.set_filter(StorageFilter(topics=['/scan_front', '/scan_rear', '/tf', '/tf_static']))
    buffer = Buffer(cache_time=Duration(seconds=seconds+5.))
    counts, scans, first = Counter(), [], None
    while reader.has_next():
        topic, data, recorded = reader.read_next()
        if first is None:
            first = recorded
        if recorded-first > (seconds+1.)*1e9:
            break
        if topic.startswith('/tf'):
            for transform in deserialize_message(data, TFMessage).transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(transform, 'bag')
                else:
                    buffer.set_transform(transform, 'bag')
        elif recorded-first <= seconds*1e9:
            scans.append((topic, deserialize_message(data, LaserScan)))
    displacements = []
    for topic, scan in scans:
        counts[topic] += 1
        counts['timing_unknown'] += beam_offsets(
            len(scan.ranges), scan.time_increment, 'unknown') is None
        counts['positive_increment'] += scan.time_increment > 0
        ranges = np.asarray(scan.ranges)
        indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= scan.range_min)
                                 & (ranges <= min(scan.range_max, 10.)))
        if not len(indices):
            continue
        # Bounded representative sample; original arrays are untouched.
        indices = indices[::max(1, len(indices)//50)]
        angles = scan.angle_min+indices*scan.angle_increment
        points = np.column_stack((ranges[indices]*np.cos(angles),
                                  ranges[indices]*np.sin(angles), np.zeros(len(indices))))
        source_ns = stamp_ns(scan.header.stamp)
        try:
            r0, t0 = rigid(buffer.lookup_transform(
                'base_link', scan.header.frame_id, rclpy.time.Time(nanoseconds=source_ns)))
            r1, t1 = rigid(buffer.lookup_transform_full(
                'base_link', rclpy.time.Time(nanoseconds=source_ns+50_000_000),
                scan.header.frame_id, rclpy.time.Time(nanoseconds=source_ns), 'odom'))
        except TransformException:
            counts['tf_missing'] += 1
            continue
        counts['tf_available'] += 1
        displacements.extend(np.linalg.norm(points@r1.T+t1-(points@r0.T+t0), axis=1))
    return {
        'bag': path, 'window_seconds': seconds, 'counts': dict(counts),
        'image_topics': [t.name for t in topics if t.type in
                         ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')],
        'epoch_shift_50ms_displacement_m_p50_p95_max': (
            np.percentile(displacements, [50, 95, 100]).tolist() if displacements else []),
        'note': 'Magnitude only; unknown beam timing and no moving-object ground truth.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--seconds', type=float, default=10.)
    args = parser.parse_args()
    print(json.dumps(audit(args.bag, args.seconds), indent=2))


if __name__ == '__main__':
    main()

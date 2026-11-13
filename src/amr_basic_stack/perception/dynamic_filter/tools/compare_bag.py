#!/usr/bin/env python3
"""
Build matched OFF/ON Cartographer bags using the real filter callbacks.

Recorded receipt times drive a deterministic 10ms timer and bounded TF wait.
No robot topics are published. Original inputs are never changed. This checks
algorithmic replay, not DDS scheduling or real-time CPU deadline compliance.
"""

import argparse
from collections import Counter
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

from amr_interfaces.msg import ImmTrackedObject, ImmTrackedObjectArray, TrackedObject
from dynamic_filter.core import eligible_tracks, stamp_ns, TransformUnavailable
import dynamic_filter.dynamic_filter_node as node_module
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py as bag_api
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import yaml

SLAM_TOPICS = {
    '/scan_front_carto': '/scan_1', '/scan_rear_carto': '/scan_2',
    '/camera/imu': '/imu', '/odom': '/odom', '/tf_static': '/tf_static',
}


def key(scan):
    return scan.header.frame_id, stamp_ns(scan.header.stamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--visual-start', type=float, default=120.)
    parser.add_argument('--visual-end', type=float, default=140.)
    parser.add_argument('--mask-method', choices=('radius', 'cluster', 'tracked_cluster'),
                        default=None,
                        help='Override mask_method; otherwise YAML or radius default')
    parser.add_argument('--filter-config', type=Path,
                        help='Wildcard ROS parameter YAML, as used by Cartographer launch')
    parser.add_argument('--front-sensor-id', default='/scan_front')
    parser.add_argument('--rear-sensor-id', default='/scan_rear')
    parser.add_argument('--observations-topic', default='/tracked_observations')
    parser.add_argument('--time-origin-ns', type=int,
                        help='Original bag epoch for plots when using a derived input bag')
    args = parser.parse_args()
    parameters = {}
    document = {}
    if args.filter_config:
        document = yaml.safe_load(args.filter_config.read_text())
        parameters.update(document['/**']['ros__parameters'])
    if args.mask_method:
        parameters['mask_method'] = args.mask_method
    tracked = parameters.get('mask_method') == 'tracked_cluster'
    reader = bag_api.SequentialReader()
    reader.open(bag_api.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                bag_api.ConverterOptions('cdr', 'cdr'))
    metadata = reader.get_metadata()
    epoch_ns = (args.time_origin_ns if args.time_origin_ns is not None
                else metadata.starting_time.nanoseconds)
    available = {entry.name: entry for entry in reader.get_all_topics_and_types()}
    selected = list(SLAM_TOPICS) + ['/tracked_object_modes', '/tf']
    if tracked:
        selected.append(args.observations_topic)
        if args.observations_topic not in available:
            raise ValueError('tracked_cluster requires recorded /tracked_observations links; '
                             'old IMM-only bags cannot reconstruct actual associations')
        from amr_interfaces.msg import TrackedObservationArray
    if set(selected) - available.keys():
        raise ValueError('Bag is missing required scan, posterior or TF inputs')
    reader.set_filter(bag_api.StorageFilter(topics=selected))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'frames').mkdir()
    rclpy.init(domain_id=197)
    # Use current source implementation, including queue overflow and TF timeout.
    instances = []
    for side in ('front', 'rear'):
        settings = {**parameters, **document.get(f'dynamic_filter_{side}', {}).get(
            'ros__parameters', {})}
        if tracked:
            settings['observation_sensor_id'] = getattr(args, f'{side}_sensor_id')
            settings['tracked_observations_topic'] = args.observations_topic
        overrides = [Parameter(name, value=value) for name, value in settings.items()]
        instances.append(node_module.DynamicFilterNode(parameter_overrides=overrides))
    front, rear = instances
    nodes = {'/scan_front_carto': front, '/scan_rear_carto': rear}
    virtual_ns = epoch_ns
    node_module.time = SimpleNamespace(monotonic=lambda: (virtual_ns - epoch_ns) * 1e-9)
    raw_scans, filtered, receipt_times = {}, {}, {}
    scan_rows, visual_rows, records = [], [], []
    totals = {topic: Counter() for topic in nodes}
    frame_count = 0
    original_hash, filtered_hash = hashlib.sha256(), hashlib.sha256()

    def capture(topic, node, output, stats):
        nonlocal frame_count
        identity = key(output)
        original = raw_scans[identity]
        restored = deserialize_message(serialize_message(output), LaserScan)
        restored.ranges = original.ranges
        assert serialize_message(restored) == serialize_message(original), 'metadata changed'
        before = np.asarray(original.ranges)
        after = np.asarray(output.ranges)
        removed = np.isfinite(before) & np.isnan(after)
        valid = (np.isfinite(before) & (before >= original.range_min)
                 & (before <= original.range_max))
        assert np.array_equal(before[~removed], after[~removed], equal_nan=True)
        filtered[identity] = serialize_message(output)
        t = (identity[1] - epoch_ns) * 1e-9
        row = {'topic': topic, 't': t, 'delay_s': (virtual_ns - receipt_times[identity]) * 1e-9,
               'valid': int(valid.sum()), 'removed': int(removed.sum()), **stats}
        scan_rows.append(row)
        total = totals[topic]
        total['scans'] += 1
        total['valid_points'] += row['valid']
        total['removed_points'] += row['removed']
        total['scans_with_removed_points'] += int(removed.any())
        total['reason_' + stats['reason']] += 1
        if args.visual_start <= t <= args.visual_end:
            indices = np.flatnonzero(valid)
            points, keep_indices = [], []
            for index in indices:
                ns = identity[1] + round(int(index) * original.time_increment * 1e9)
                try:
                    matrix = node._transform_at(original.header.frame_id, ns)
                except TransformUnavailable:
                    continue
                angle = original.angle_min + int(index) * original.angle_increment
                p = matrix @ [before[index]*np.cos(angle), before[index]*np.sin(angle), 0., 1.]
                points.append(p[:2])
                keep_indices.append(index)
            snapshot, _ = node.history.select(identity[1])
            centres, track_ids = [], []
            if snapshot is not None and snapshot.header.frame_id == 'odom':
                eligible = eligible_tracks(
                    snapshot, node.config, ImmTrackedObject.LIFECYCLE_CONFIRMED,
                    TrackedObject.MOTION_DYNAMIC, Counter())
                for obj, state in eligible:
                    dt = (identity[1] - stamp_ns(snapshot.header.stamp))*1e-9
                    if (dt <= node.config.max_extrapolation
                            and obj.measurement_age + dt <= node.config.max_measurement_age
                            and obj.semantic_age + dt <= node.config.person_label_timeout):
                        centres.append(state[:2] + state[2:4]*dt)
                        track_ids.append(obj.track_id)
            filename = f'frame_{frame_count:05d}.npz'
            np.savez_compressed(args.output / 'frames' / filename,
                                points=np.asarray(points).reshape(-1, 2),
                                removed=removed[keep_indices],
                                centres=np.asarray(centres).reshape(-1, 2),
                                track_ids=np.asarray(track_ids), t=t)
            visual_rows.append({'file': filename, 'topic': topic, **row})
            frame_count += 1
        del raw_scans[identity]

    for topic, node in nodes.items():
        node._publish = lambda out, stats, topic=topic, node=node: capture(topic, node, out, stats)
    next_tick = epoch_ns
    tick_ns = round(front.config.retry_period * 1e9)
    last_progress = time.monotonic()
    counters = Counter()
    try:
        while reader.has_next():
            topic, raw, receipt_ns = reader.read_next()
            # Inputs received at a tick are available on the following tick.
            while next_tick < receipt_ns:
                virtual_ns = next_tick
                for node in nodes.values():
                    node._drain()
                next_tick += tick_ns
            virtual_ns = receipt_ns
            counters[topic] += 1
            if topic in SLAM_TOPICS:
                records.append((topic, raw, receipt_ns))
                original_hash.update(raw)
            if topic in ('/tf', '/tf_static'):
                message = deserialize_message(raw, TFMessage)
                for transform in message.transforms:
                    # map->odom is intentionally irrelevant to odom masking and excluded.
                    if transform.header.frame_id == 'map':
                        continue
                    for node in nodes.values():
                        if topic == '/tf_static':
                            node.tf_buffer.set_transform_static(transform, 'recorded')
                        else:
                            node.tf_buffer.set_transform(transform, 'recorded')
            elif topic == '/tracked_object_modes':
                message = deserialize_message(raw, ImmTrackedObjectArray)
                for node in nodes.values():
                    node._on_tracks(message)
            elif tracked and topic == args.observations_topic:
                message = deserialize_message(raw, TrackedObservationArray)
                for node in nodes.values():
                    node._on_observations(message)
            elif topic in nodes:
                message = deserialize_message(raw, LaserScan)
                identity = key(message)
                if identity in receipt_times:
                    raise ValueError('Duplicate sensor frame/stamp; use receipt sequence keys')
                raw_scans[identity] = message
                receipt_times[identity] = receipt_ns
                nodes[topic]._on_scan(message)
            if time.monotonic() - last_progress > 10.:
                print(json.dumps({'bag_seconds': round((receipt_ns-epoch_ns)*1e-9, 1),
                                  'completed_scans': len(scan_rows),
                                  'removed_points': sum(x['removed_points']
                                                        for x in totals.values())}),
                      flush=True)
                last_progress = time.monotonic()
        for _ in range(100):
            virtual_ns = next_tick
            next_tick += tick_ns
            for node in nodes.values():
                node._drain()
            if not any(node.pending for node in nodes.values()):
                break
        assert not raw_scans and len(filtered) == sum(counters[t] for t in nodes)
        for variant in ('off', 'on'):
            writer = bag_api.SequentialWriter()
            writer.open(bag_api.StorageOptions(uri=str(args.output / variant), storage_id='mcap'),
                        bag_api.ConverterOptions('cdr', 'cdr'))
            for index, (source, target) in enumerate(SLAM_TOPICS.items()):
                item = available[source]
                writer.create_topic(bag_api.TopicMetadata(
                    id=index, name=target, type=item.type, serialization_format='cdr',
                    offered_qos_profiles=item.offered_qos_profiles))
            for topic, raw, stamp in records:
                if variant == 'on' and topic in nodes:
                    raw = filtered[key(deserialize_message(raw, LaserScan))]
                writer.write(SLAM_TOPICS[topic], raw, stamp)
                if variant == 'on':
                    filtered_hash.update(raw)
            del writer
        fields = sorted(set().union(*(row.keys() for row in scan_rows)))
        with (args.output / 'scan_metrics.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(scan_rows)
        (args.output / 'frames.json').write_text(json.dumps(visual_rows, indent=2))
        report = {'input': str(args.bag), 'start_ns': epoch_ns,
                  'duration_s': metadata.duration.nanoseconds * 1e-9,
                  'config': asdict(front.config), 'input_topic_counts': dict(counters),
                  'totals': {topic: dict(total) for topic, total in totals.items()},
                  'source_slam_sha256': original_hash.hexdigest(),
                  'filtered_slam_sha256': filtered_hash.hexdigest(),
                  'visual_interval_s': [args.visual_start, args.visual_end],
                  'method': 'Recorded arrival order, real DynamicFilterNode callbacks, virtual '
                            'steady 10ms timer, bounded queues, per-beam recorded TF. '
                            'Both SLAM bags keep identical receipt/header times, odom, IMU '
                            'and static TF. Only masked ranges differ. Recorded IMM is reused.'}
        (args.output / 'filter_summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
    finally:
        for node in nodes.values():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

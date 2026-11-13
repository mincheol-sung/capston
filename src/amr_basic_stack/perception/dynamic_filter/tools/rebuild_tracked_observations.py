#!/usr/bin/env python3
"""
Replay current clustering/fusion/tracking on recorded native scans and camera detections.

This generates NEW associations, not historical tracker assignments. The legacy
decoder must be generated from the bag's recorded DetectedObjectArray schema.
Only recorded camera detections are reused; image inference is not rerun.
"""

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

from amr_interfaces.msg import DetectedObject, DetectedObjectArray, ObservationProvenance
from bag_legacy_detection.msg import DetectedObjectArray as LegacyDetections
import object_tracker.object_tracker_node as tracker_module
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
import rosbag2_py as bag_api
import sensor_fusion.camera_lidar_fusion_node as fusion_module
import sensor_fusion.lidar_cluster_node as cluster_module
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

STACK = Path(__file__).resolve().parents[3]
SLAM = ['/scan_front_carto', '/scan_rear_carto', '/camera/imu', '/odom', '/tf_static']


def migrate_camera(raw):
    """Roundtrip-check legacy CDR, copy every recorded field, add explicit provenance."""
    old = deserialize_message(raw, LegacyDetections)
    assert serialize_message(old) == raw, 'Legacy decoder does not reproduce recorded CDR'
    out = DetectedObjectArray()
    out.header = deepcopy(old.header)
    assert out.header.frame_id == 'base_link'
    for index, obj in enumerate(old.objects):
        assert obj.header == old.header and obj.source == 0
        new = DetectedObject()
        for field in obj.get_fields_and_field_types():
            setattr(new, field, deepcopy(getattr(obj, field)))
        provenance = ObservationProvenance()
        provenance.header = deepcopy(obj.header)
        provenance.sensor_id = '/recorded_camera_detections'
        stamp = obj.header.stamp.sec*10**9 + obj.header.stamp.nanosec
        provenance.observation_id = f'recorded-camera|{stamp}|{index}'
        new.observations = [provenance]
        out.objects.append(new)
    # Empty frames still retain the recorded camera epoch in the fusion queue.
    record = ObservationProvenance()
    record.header = deepcopy(old.header)
    record.sensor_id = '/recorded_camera_detections'
    record.observation_id = f'camera-frame|{old.header.stamp.sec}|{old.header.stamp.nanosec}'
    out.observations = [record]
    return out


def main():
    """Regenerate and audit current tracking observations for one recorded bag."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-seconds', type=float, default=None)
    args = parser.parse_args()
    reader = bag_api.SequentialReader()
    reader.open(bag_api.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                bag_api.ConverterOptions('cdr', 'cdr'))
    metadata = reader.get_metadata()
    epoch = metadata.starting_time.nanoseconds
    available = {t.name: t for t in reader.get_all_topics_and_types()}
    required = SLAM + ['/tf', '/scan_front', '/scan_rear', '/detections']
    assert not set(required) - available.keys()
    reader.set_filter(bag_api.StorageFilter(topics=required))
    args.output.mkdir(parents=True, exist_ok=False)
    writer = bag_api.SequentialWriter()
    writer.open(bag_api.StorageOptions(uri=str(args.output / 'input'), storage_id='mcap'),
                bag_api.ConverterOptions('cdr', 'cdr'))
    for index, topic in enumerate(SLAM + ['/tf']):
        item = available[topic]
        writer.create_topic(bag_api.TopicMetadata(
            id=index, name=topic, type=item.type, serialization_format='cdr',
            offered_qos_profiles=item.offered_qos_profiles))
    for index, (name, kind) in enumerate([
        ('/tracked_object_modes', 'ImmTrackedObjectArray'),
        ('/tracked_observations', 'TrackedObservationArray'),
    ], start=10):
        writer.create_topic(bag_api.TopicMetadata(
            id=index, name=name, type=f'amr_interfaces/msg/{kind}', serialization_format='cdr'))
    timing = STACK / 'perception/amr_perception_bringup/config/observation_timing.yaml'
    tracking = STACK / 'perception/object_tracker/config/tracker.yaml'
    rclpy.init(domain_id=192, args=[
        '--ros-args', '--params-file', str(tracking),
        '--params-file', str(timing), '-p', 'use_sim_time:=true'])
    cluster = cluster_module.LidarClusterNode()
    fusion = fusion_module.CameraLidarFusionNode()
    tracker = tracker_module.ObjectTrackerNode()
    nodes = [cluster, fusion, tracker]
    virtual_ns = epoch
    counts, sources, class_counts, link_lags = Counter(), Counter(), Counter(), []
    originals, carto = {}, {}
    original_hash = hashlib.sha256()
    for module in (cluster_module, fusion_module, tracker_module):
        module.time = SimpleNamespace(monotonic=lambda: (virtual_ns-epoch)*1e-9)

    def capture_modes(msg):
        writer.write('/tracked_object_modes', serialize_message(msg), virtual_ns)
        counts['posterior_frames'] += 1
        for obj in msg.objects:
            class_counts[obj.class_name or 'unknown'] += 1
            if obj.lifecycle_state == 1 and obj.class_name == 'person':
                counts['confirmed_person_samples'] += 1

    def capture_links(msg):
        writer.write('/tracked_observations', serialize_message(msg), virtual_ns)
        counts['link_frames'] += 1
        counts['links'] += len(msg.links)
        for link in msg.links:
            stamp = link.observation.header.stamp
            source_ns = stamp.sec*10**9 + stamp.nanosec
            link_lags.append((virtual_ns-source_ns)*1e-9)
            if link.lifecycle_state == 1 and link.class_name == 'person':
                counts['person_links'] += 1
                counts['person_link_beams'] += len(link.observation.beam_indices)

    def capture_cluster(msg):
        counts['cluster_frames'] += 1
        counts['clusters'] += len(msg.objects)
        fusion._enqueue(msg, False)

    def capture_tracking(msg):
        sources.update(str(obj.source) for obj in msg.objects)
        tracker._on_detections(msg)

    cluster.pub.publish = capture_cluster
    fusion.tracking_pub.publish = capture_tracking
    fusion.pub.publish = lambda _: None
    fusion.lidar_obstacles_pub.publish = lambda _: None
    tracker.pub.publish = lambda _: None
    tracker.modes_pub.publish = capture_modes
    tracker.observations_pub.publish = capture_links
    tracker.marker_pub = None

    def set_time(ns):
        nonlocal virtual_ns
        virtual_ns = ns
        for node in nodes:
            node.get_clock().set_ros_time_override(Time(nanoseconds=ns))
            assert node.get_clock().now().nanoseconds == ns

    def tick(ns):
        set_time(ns)
        cluster._drain()
        fusion._drain()
        tracker._drain_detections()

    next_tick = epoch
    last_progress = time.monotonic()
    try:
        while reader.has_next():
            topic, raw, receipt = reader.read_next()
            if args.max_seconds is not None and receipt-epoch > args.max_seconds*1e9:
                break
            while next_tick < receipt:
                tick(next_tick)
                next_tick += 10_000_000
            set_time(receipt)
            counts['input_' + topic] += 1
            if topic in SLAM + ['/tf']:
                writer.write(topic, raw, receipt)
                if topic in SLAM:
                    original_hash.update(raw)
            if topic in ('/tf', '/tf_static'):
                for transform in deserialize_message(raw, TFMessage).transforms:
                    if transform.header.frame_id == 'map':
                        continue
                    for node in nodes:
                        if topic == '/tf_static':
                            node.tf_buffer.set_transform_static(transform, 'recorded')
                        else:
                            node.tf_buffer.set_transform(transform, 'recorded')
            elif topic == '/detections':
                camera = migrate_camera(raw)
                counts['camera_frames_roundtrip_checked'] += 1
                fusion._enqueue(camera, True)
            elif topic.startswith('/scan_'):
                msg = deserialize_message(raw, LaserScan)
                stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                identity = (msg.header.frame_id, stamp)
                # Audit index-preserving preprocessing before trusting native IDs.
                geometry = (len(msg.ranges), msg.angle_min, msg.angle_max, msg.angle_increment,
                            msg.time_increment, msg.scan_time, msg.range_min, msg.range_max)
                if topic.endswith('_carto'):
                    carto[identity] = geometry
                else:
                    originals[identity] = geometry
                    cluster._on_scan(msg, 0 if topic == '/scan_front' else 1)
            if time.monotonic() - last_progress > 15.:
                print(json.dumps({'bag_seconds': (receipt-epoch)*1e-9,
                                  'posterior_frames': counts['posterior_frames'],
                                  'person_links': counts['person_links']}), flush=True)
                last_progress = time.monotonic()
        for _ in range(100):
            tick(next_tick)
            next_tick += 10_000_000
        paired = set(carto) & set(originals)
        assert all(carto[k] == originals[k] for k in paired), 'Native/carto beam layout changed'
        assert len(paired) == len(carto), 'A Cartographer scan lacks its native source'
        parameters = {}
        for node in nodes:
            parameters[node.get_name()] = {
                name: value.value for name, value in node._parameters.items()}
        import numpy as np
        report = {
            'original_bag': str(args.bag), 'original_epoch_ns': epoch,
            'max_seconds': args.max_seconds, 'counts': dict(counts),
            'source_slam_sha256': original_hash.hexdigest(),
            'native_carto_geometry_matches': len(paired),
            'tracking_detection_sources_0camera_1lidar_2fused': dict(sources),
            'posterior_classes': dict(class_counts),
            'link_receipt_minus_native_stamp_quantiles_s': dict(zip(
                ('p50', 'p95', 'max'), map(float, np.quantile(link_lags, [.5, .95, 1.]))))
            if link_lags else {},
            'cluster_counts': dict(cluster.counts),
            'cluster_drops': [dict(p.dropped) for p in cluster.pending],
            'fusion_counts': dict(fusion.counts), 'fusion_queue': dict(fusion.pending.counts),
            'tracker_drops': dict(tracker.pending.dropped), 'node_parameters': parameters,
            'method': 'Current native clustering -> fusion -> tracker callbacks, recorded receipt '
                      'order, 10ms virtual timers; event receipts use current virtual time.',
            'limitations': [
                'NEW associations/posteriors; these are not original recorded tracker decisions.',
                'Camera detections reused from old bag, with recorded header as only available '
                'capture epoch. RGB/depth inference and original image sync cannot be repeated.',
                'Native LiDAR beam timing remains unknown; configured snapshot policy retained.',
                'CPU computation consumes no virtual time. Not a real-time/DDS latency test.',
                'No ground-truth point labels or independently measured robot pose.'],
        }
        (args.output / 'rebuild_summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != 'node_parameters'}, indent=2))
    finally:
        writer = None
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

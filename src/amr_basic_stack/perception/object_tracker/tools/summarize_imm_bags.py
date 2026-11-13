"""Summarize sensor evidence for provisional IMM defaults (not ground truth).

Run with the ROS workspace sourced; this reads only the unchanged detections
and odometry schemas, so old two-mode bags remain usable after the upgrade.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml


def summarize(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    selected = [t for t in ('/detections_fused', '/odometry/filtered') if t in topics]
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    types = {t: get_message(topics[t]) for t in selected}
    stamps, sources, classes, sigma, speeds = [], Counter(), Counter(), [], []
    positions, odometry, frames = [], [], []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == '/odometry/filtered':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            odometry.append((stamp, p.x, p.y, yaw))
            continue
        stamps.append(stamp)
        frames.append(msg)
        for obj in msg.objects:
            sources[str(obj.source)] += 1
            if obj.source not in (0, 2):
                continue
            classes[obj.class_name] += 1
            R = np.asarray(obj.position_covariance).reshape(2, 2)
            if np.all(np.isfinite(R)):
                sigma.append(float(np.sqrt(max(0, np.linalg.eigvalsh(R).max()))))
            positions.append((stamp, obj.class_name, obj.position.x, obj.position.y))
    # Same-class adjacent samples, odom compensated; not verified identities.
    # Quantiles are apparent motion and include association/depth errors.
    jitter, previous = [], {}
    if odometry:
        odom = np.array(sorted(odometry))
        odom[:, 3] = np.unwrap(odom[:, 3])
        for stamp, label, x, y in positions:
            if not odom[0, 0] <= stamp <= odom[-1, 0]:
                continue
            ox, oy, yaw = [np.interp(stamp, odom[:, 0], odom[:, j]) for j in (1, 2, 3)]
            p = np.array([ox+np.cos(yaw)*x-np.sin(yaw)*y,
                          oy+np.sin(yaw)*x+np.cos(yaw)*y])
            if label in previous:
                last, old = previous[label]
                dt = stamp-last
                if .05 <= dt <= .3:
                    distance = float(np.linalg.norm(p-old))
                    if distance < .3:
                        speeds.append(distance/dt)
                        if label != 'person':
                            jitter.append(distance/np.sqrt(2*dt))
            previous[label] = stamp, p
    def quantiles(values):
        return dict(zip(('p10', 'p50', 'p90', 'p99'),
                        map(float, np.quantile(values, [.1, .5, .9, .99])))) if len(values) else {}
    intervals = np.diff(sorted(set(stamps)))
    from object_tracker.core.config import TrackerConfig
    from object_tracker.core.measurements import semantic_measurement
    from object_tracker.core.tracker import MultiObjectTracker
    from object_tracker.core.track import CONFIRMED, LOST, MOTION_NAME
    cfg = TrackerConfig()
    tracker = MultiObjectTracker(cfg)
    replay_states, skipped = Counter(), 0
    person_recoveries = person_superseded = 0
    for frame in frames:
        stamp = frame.header.stamp.sec + frame.header.stamp.nanosec*1e-9
        if frame.header.frame_id == 'odom':
            translation, rotation = np.zeros(2), np.eye(2)
        elif (frame.header.frame_id == 'base_link' and odometry
              and odom[0, 0] <= stamp <= odom[-1, 0]):
            x, y, yaw = [np.interp(stamp, odom[:, 0], odom[:, j]) for j in (1, 2, 3)]
            translation = np.array([x, y])
            rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                                 [np.sin(yaw), np.cos(yaw)]])
        else:
            skipped += 1
            continue
        zs, Rs, detections = [], [], []
        for obj in frame.objects:
            measured = semantic_measurement(obj, cfg.fallback_sigma)
            if measured is None:
                continue
            p, R = measured
            zs.append(rotation @ p + translation)
            Rs.append(rotation @ R @ rotation.T)
            detections.append(obj)
        lost_people = {
            track.id for track in tracker.tracks
            if track.lifecycle == LOST and track.class_name == 'person'
        }
        published = tracker.step(stamp, zs, Rs, detections)
        person_recoveries += sum(
            track.id in lost_people and track.lifecycle == CONFIRMED
            for track in tracker.tracks
        )
        person_superseded += len(tracker.last_superseded)
        for track in published:
            replay_states[f'{track.class_name or "unconfirmed_class"}/{track.lifecycle}/{MOTION_NAME[track.motion]}'] += 1
    return {'bag': str(path), 'frames': len(stamps), 'sources': dict(sources),
            'classes': dict(classes), 'interval_s': quantiles(intervals),
            'measurement_sigma_m': quantiles(sigma),
            'apparent_speed_m_s': quantiles(speeds),
            'nonperson_apparent_diffusion_m_sqrt_s': quantiles(jitter),
            'replay_track_count': tracker.next_id,
            'replay_person_recoveries': person_recoveries,
            'replay_person_superseded': person_superseded,
            'replay_track_samples': dict(replay_states),
            'replay_skipped_frames_no_transform': skipped,
            'replay_config': cfg.field_defaults(),
            'note': 'No ground truth; nonperson motion includes centroid/association noise.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='*', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    bags = args.bags
    if not bags:
        candidates = []
        for metadata in Path('bags').glob('*/metadata.yaml'):
            data = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
            candidates.append((data['starting_time']['nanoseconds_since_epoch'], metadata.parent))
        bags = [p for _, p in sorted(candidates, reverse=True)[:3]]
    result = json.dumps([summarize(p) for p in bags], indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result + '\n')
    print(result)

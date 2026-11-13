#!/usr/bin/env python3
"""Read-only observation evidence audit using each bag's embedded ROS schemas.

No replay publications, selector, or counterfactual camera observations.
semantic_age resets are tracker updates, not proof of a new camera exposure.
Run with ROS Jazzy sourced and rosbags installed in an isolated environment.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import rosbag2_py as bag
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

DETECTIONS = ('/detections', '/lidar_clusters', '/detections_fused', '/detections_tracking')
SELECTED = (*DETECTIONS, '/tracked_object_modes', '/camera/color/camera_info',
            '/tf_static', '/odometry/filtered', '/global_path')


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def audit(path):
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(path), storage_id='mcap'), bag.ConverterOptions('', ''))
    metadata = reader.get_metadata()
    epoch = metadata.starting_time.nanoseconds * 1e-9
    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    selected = set(SELECTED) & available.keys()
    types = get_typestore(Stores.EMPTY)
    definitions = {}
    for d in reader.get_all_message_definitions():
        if d.topic_type in {available[t] for t in selected}:
            if d.encoding != 'ros2msg':
                raise ValueError(f'Unsupported schema encoding: {d.encoding}')
            types.register(get_types_from_msg(d.encoded_message_definition, d.topic_type))
            definitions[d.topic_type] = d.encoded_message_definition
    reader.set_filter(bag.StorageFilter(topics=sorted(selected)))
    counts, frames = Counter(), defaultdict(Counter)
    detection = {t: {'objects': 0, 'sources': Counter(), 'classes': Counter(),
                     'nonempty_messages': 0, 'unique_header_stamps': set()} for t in DETECTIONS if t in selected}
    tr_counts, tr_sources, tr_classes, lifecycle = Counter(), Counter(), Counter(), Counter()
    all_ids, missing_ids, lidar_ids, candidate_ids = set(), set(), set(), set()
    previous, first_seen, missing_seen, events, refreshed = {}, {}, {}, [], []
    camera, transforms = None, []
    candidate_frames = 0
    for_topic_stamps = defaultdict(list)
    while reader.has_next():
        topic, raw, receipt = reader.read_next()
        msg = types.deserialize_cdr(raw, available[topic])
        counts[topic] += 1
        if hasattr(msg, 'header'):
            frames[topic][msg.header.frame_id] += 1
            for_topic_stamps[topic].append(stamp(msg.header))
        if topic in detection:
            d = detection[topic]
            d['objects'] += len(msg.objects)
            d['nonempty_messages'] += bool(msg.objects)
            d['unique_header_stamps'].add(stamp(msg.header))
            for o in msg.objects:
                d['sources'][str(o.source)] += 1
                d['classes'][o.class_name] += 1
        elif topic == '/tracked_object_modes':
            t = stamp(msg.header)
            candidates = 0
            for o in msg.objects:
                tid = int(o.track_id)
                all_ids.add(tid)
                first_seen.setdefault(tid, t)
                tr_counts['object_samples'] += 1
                tr_sources[str(o.last_measurement_source)] += 1
                tr_classes[o.class_name] += 1
                lifecycle[str(o.lifecycle_state)] += 1
                if o.semantic_age < 0:
                    missing_ids.add(tid)
                    missing_seen.setdefault(tid, t)
                    tr_counts['never_semantic_samples'] += 1
                if o.last_measurement_source == 1:
                    lidar_ids.add(tid)
                    tr_counts['last_source_lidar_samples'] += 1
                # Diagnostic eligibility ONLY; no path relevance/observability selection.
                if (o.lifecycle_state == 1 and 0 <= o.measurement_age <= .5
                        and o.last_measurement_source == 1
                        and (o.semantic_age < 0 or o.semantic_age > 1.)):
                    candidates += 1
                    candidate_ids.add(tid)
                    tr_counts['preliminary_lidar_information_need_samples'] += 1
                old = previous.get(tid)
                if old is not None and 0 < t-old['t'] <= .5:
                    if old['semantic_age'] < 0 <= o.semantic_age:
                        events.append(dict(track_id=tid, time_s=t-epoch,
                            first_unclassified_s=missing_seen[tid]-epoch,
                            elapsed_s=t-missing_seen[tid], class_name=o.class_name,
                            last_source=int(o.last_measurement_source), semantic_age=float(o.semantic_age)))
                    if old['semantic_age'] > 1. and 0 <= o.semantic_age <= .15:
                        refreshed.append(dict(track_id=tid, time_s=t-epoch,
                            previous_age=old['semantic_age'], class_name=o.class_name,
                            last_source=int(o.last_measurement_source)))
                previous[tid] = dict(t=t, semantic_age=float(o.semantic_age))
            candidate_frames += candidates > 0
        elif topic == '/camera/color/camera_info' and camera is None:
            fx, cx = float(msg.k[0]), float(msg.k[2])
            camera = dict(width=int(msg.width), height=int(msg.height), fx=fx, cx=cx,
                frame=msg.header.frame_id,
                horizontal_fov_deg=math.degrees(math.atan(cx/fx)+math.atan((msg.width-cx)/fx)),
                note='Pinhole estimate from recorded CameraInfo, not effective detection FOV')
        elif topic == '/tf_static':
            for tf in msg.transforms:
                if 'camera' in tf.child_frame_id and ('base' in tf.header.frame_id or tf.child_frame_id == 'camera_color_frame'):
                    p, q = tf.transform.translation, tf.transform.rotation
                    transforms.append(dict(parent=tf.header.frame_id, child=tf.child_frame_id,
                        xyz=[p.x,p.y,p.z], xyzw=[q.x,q.y,q.z,q.w]))
    expected = {x.topic_metadata.name: x.message_count for x in metadata.topics_with_message_count}
    for topic in selected:
        assert counts[topic] == expected[topic], (topic, counts[topic], expected[topic])
    for d in detection.values():
        d['unique_header_stamps'] = len(d['unique_header_stamps'])
    result = dict(bag=str(path), duration_s=metadata.duration.nanoseconds/1e9,
        selected_message_counts=counts, frame_counts=frames, detections=detection,
        image_topics=[t for t,v in available.items() if v in ('sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage')],
        observation_target_topics=[t for t in available if 'observation_target' in t],
        embedded_detection_schema_has_provenance='ObservationProvenance' in definitions.get('amr_interfaces/msg/DetectedObjectArray',''),
        tracks=dict(counts=tr_counts, sources=tr_sources, classes=tr_classes, lifecycle=lifecycle,
            unique_ids=len(all_ids), never_semantic_ids=len(missing_ids), last_source_lidar_ids=len(lidar_ids),
            preliminary_lidar_information_need_ids=len(candidate_ids),
            preliminary_lidar_information_need_frames=candidate_frames,
            never_semantic_to_semantic_events=events, stale_to_fresh_tracker_events=refreshed),
        camera=camera, camera_static_transforms=transforms,
        limitations=['Track IDs are recorded associations, not ground-truth identity.',
          'semantic_age reset is not proof of a distinct camera exposure; legacy fusion can reuse detections.',
          'Preliminary information-need gate excludes path relevance, turn feasibility, and occlusion.',
          'No counterfactual heading or new camera views can be tested from fixed recorded observations.',
          'No track-ID epoch reset inferred; events only compare adjacent samples within 0.5 s.'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for path in args.bags:
        r = audit(path)
        (args.output / (path.name+'.json')).write_text(json.dumps(r, indent=2, ensure_ascii=False)+'\n')
        t = r['tracks']
        print(json.dumps(dict(bag=path.name, duration_s=r['duration_s'], tracks=t['unique_ids'],
            never_semantic=t['never_semantic_ids'], lidar=t['last_source_lidar_ids'],
            preliminary_candidates=t['preliminary_lidar_information_need_ids'],
            initial_confirmations=len(t['never_semantic_to_semantic_events']),
            stale_refreshes=len(t['stale_to_fresh_tracker_events']),
            images=r['image_topics'], provenance=r['embedded_detection_schema_has_provenance']),ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Copy only standard SLAM inputs into a new, remapped offline MCAP bag.

Run after sourcing /opt/ros/jazzy/setup.bash. No ROS node is started.
The input bag and any existing destination are never modified.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


TOPICS = {
    "/scan_front_carto": ("/scan_1", "sensor_msgs/msg/LaserScan"),
    "/scan_rear_carto": ("/scan_2", "sensor_msgs/msg/LaserScan"),
    "/camera/imu": ("/imu", "sensor_msgs/msg/Imu"),
    "/odom": ("/odom", "nav_msgs/msg/Odometry"),
    "/tf_static": ("/tf_static", "tf2_msgs/msg/TFMessage"),
}


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Existing source bag directory")
    parser.add_argument("output", type=Path, help="New destination bag directory")
    parser.add_argument("--metadata-json", required=True, type=Path)
    parser.add_argument("--max-duration", type=float, default=None,
                        help="Copy at most this many seconds from source bag start")
    args = parser.parse_args()
    if args.max_duration is not None and (
        not math.isfinite(args.max_duration) or args.max_duration <= 0
    ):
        parser.error("--max-duration must be finite and greater than zero")
    args.input = args.input.expanduser().resolve()
    args.output = args.output.expanduser().resolve()
    args.metadata_json = args.metadata_json.expanduser().resolve()
    if not args.input.is_dir():
        parser.error(f"Source bag directory does not exist: {args.input}")
    if args.output.exists():
        parser.error(f"Destination already exists: {args.output}")
    if args.metadata_json.exists():
        parser.error(f"Metadata destination already exists: {args.metadata_json}")
    if args.output.is_relative_to(args.input):
        parser.error("Output must be outside the source bag directory")
    if args.metadata_json.is_relative_to(args.input):
        parser.error("Metadata must be outside the source bag directory")
    if args.metadata_json == args.output:
        parser.error("Metadata destination must differ from output bag directory")
    return args


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main() -> int:
    args = _args()
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu, LaserScan
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        raise RuntimeError(
            "ROS Python modules are unavailable; source /opt/ros/jazzy/setup.bash"
        ) from exc

    message_types = {
        "sensor_msgs/msg/LaserScan": LaserScan,
        "sensor_msgs/msg/Imu": Imu,
        "nav_msgs/msg/Odometry": Odometry,
        "tf2_msgs/msg/TFMessage": TFMessage,
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.input), storage_id=""),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    source_metadata = reader.get_metadata()
    available = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    source_counts = {
        entry.topic_metadata.name: int(entry.message_count)
        for entry in source_metadata.topics_with_message_count
    }
    for name, (_, expected_type) in TOPICS.items():
        if name not in available or source_counts.get(name, 0) == 0:
            raise ValueError(f"Required source topic is missing or empty: {name}")
        if available[name].type != expected_type:
            raise ValueError(
                f"Unexpected type for {name}: {available[name].type}; "
                f"expected {expected_type}"
            )

    source_start_sec = source_metadata.starting_time.nanoseconds / 1_000_000_000
    source_duration_sec = source_metadata.duration.nanoseconds / 1_000_000_000
    cutoff_ns = None
    if args.max_duration is not None:
        cutoff_ns = round((source_start_sec + args.max_duration) * 1_000_000_000)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.output), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for topic_id, (name, (destination, _)) in enumerate(TOPICS.items()):
        original = available[name]
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=topic_id,
            name=destination,
            type=original.type,
            serialization_format=original.serialization_format,
            offered_qos_profiles=original.offered_qos_profiles,
        ))

    counts = {destination: 0 for destination, _ in TOPICS.values()}
    header_stamps = {}
    static_transforms = {}
    first_record_ns = None
    last_record_ns = None
    try:
        while reader.has_next():
            source_topic, data, record_ns = reader.read_next()
            if cutoff_ns is not None and record_ns > cutoff_ns:
                break
            destination, type_name = TOPICS[source_topic]
            message = deserialize_message(data, message_types[type_name])
            if source_topic == "/tf_static":
                for transform in message.transforms:
                    parent = transform.header.frame_id
                    child = transform.child_frame_id
                    t = transform.transform.translation
                    q = transform.transform.rotation
                    static_transforms[(parent, child)] = {
                        "parent": parent,
                        "child": child,
                        "translation": [t.x, t.y, t.z],
                        "rotation": [q.x, q.y, q.z, q.w],
                    }
            else:
                stamp_ns = _stamp_ns(message.header.stamp)
                if destination not in header_stamps:
                    header_stamps[destination] = {
                        "first_ns": stamp_ns,
                        "first_sec": stamp_ns / 1_000_000_000,
                    }
                header_stamps[destination]["last_ns"] = stamp_ns
                header_stamps[destination]["last_sec"] = stamp_ns / 1_000_000_000
            writer.write(destination, data, record_ns)
            counts[destination] += 1
            if first_record_ns is None:
                first_record_ns = record_ns
            last_record_ns = record_ns
    finally:
        # SequentialWriter finalizes metadata on destruction in Jazzy.
        del writer

    if any(count == 0 for count in counts.values()):
        empty = [name for name, count in counts.items() if count == 0]
        raise ValueError(f"Selected interval produced empty required topics: {empty}")

    transforms = [static_transforms[key] for key in sorted(static_transforms)]
    frames = sorted({frame for item in transforms
                     for frame in (item["parent"], item["child"])})
    metadata = {
        "bag_name": args.input.name,
        "duration_sec": (last_record_ns - first_record_ns) / 1_000_000_000,
        "config": "mapping_hq_2d.lua",
        "bag_start_sec": source_start_sec,
        "output": str(args.output),
        "topic_counts": counts,
        "total_message_count": sum(counts.values()),
        "first_record_ns": first_record_ns,
        "last_record_ns": last_record_ns,
        "header_stamps": header_stamps,
        "static_frame_count": len(frames),
        "static_frames": frames,
        "static_transform_count": len(transforms),
        "static_transforms": transforms,
        "max_duration_sec": args.max_duration,
        "topic_remap": {name: destination
                        for name, (destination, _) in TOPICS.items()},
        "source": {
            "path": str(args.input),
            "storage_identifier": source_metadata.storage_identifier,
            "bag_start_sec": source_start_sec,
            "duration_sec": source_duration_sec,
            "total_message_count": int(source_metadata.message_count),
            "topic_counts": source_counts,
        },
    }
    args.metadata_json.parent.mkdir(parents=True, exist_ok=True)
    with args.metadata_json.open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps({
        "output": str(args.output),
        "metadata_json": str(args.metadata_json),
        "duration_sec": metadata["duration_sec"],
        "topic_counts": counts,
        "static_transform_count": len(transforms),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"prepare_slam_video_bag: {exc}", file=sys.stderr)
        sys.exit(1)

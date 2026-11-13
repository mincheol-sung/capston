#!/usr/bin/env python3
"""Measure graph deserialization, adapter conversion, validation and reprojection."""

import argparse
import importlib
import json
from pathlib import Path
import platform
import sys
import time
from types import SimpleNamespace

from bench_save import make_memory
from bench_save import peak_rss_bytes
from bench_save import summary

import numpy as np

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message


def main():
    """Use generated ROS messages without creating a DDS context or calling services."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nodes', nargs='+', type=int, default=[1000, 10000, 20000, 20001])
    parser.add_argument('--objects', nargs='+', type=int, default=[0, 20, 200])
    parser.add_argument('--anchors', type=int, default=16)
    parser.add_argument('--points', type=int, default=512)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--repeat', type=int, default=10)
    parser.add_argument('--max-total-ms', type=float, default=0.0,
                        help='Fail if any measured complete processing cycle exceeds this budget.')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (min(args.nodes) < max(2, args.anchors) or min(args.objects) < 0
            or args.anchors < 3 or args.points < 3 or args.repeat < 1 or args.warmup < 0):
        parser.error('Invalid sample sizes or repeat count')
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    graph_module = importlib.import_module('semantic_slam.pose_graph')
    adapter = importlib.import_module('semantic_slam.cartographer_adapter').CartographerAdapter
    response_type = importlib.import_module('cartographer_ros_msgs.srv').TrajectoryQuery.Response
    pose_type = importlib.import_module('geometry_msgs.msg').PoseStamped
    results = []
    for count in args.nodes:
        response = response_type()
        poses = []
        for index in range(count):
            pose = pose_type()
            pose.header.frame_id = 'map'
            pose.header.stamp.sec = 1 + index // 10
            pose.header.stamp.nanosec = (index % 10) * 100000000
            pose.pose.orientation.w = 1.0
            pose.pose.position.x = 0.1  # Every round reprojects from identity to this pose.
            poses.append(pose)
        response.trajectory = poses
        serialized = serialize_message(response)
        for objects in args.objects:
            samples = {key: [] for key in ('deserialize', 'convert', 'replace', 'reproject')}
            totals = []
            rejected = False
            for iteration in range(args.warmup + args.repeat):
                memory = make_memory(objects, args.anchors, args.points, args.seed)
                graph = graph_module.PoseGraph('benchmark-graph')
                receiver = SimpleNamespace(
                    p={'graph_max_nodes': 20000, 'map_frame': 'map'},
                    staged={}, current=0, _next=lambda: None)
                started = time.perf_counter()
                decoded = deserialize_message(serialized, response_type)
                deserialized = time.perf_counter()
                try:
                    adapter._trajectory(receiver, decoded)
                except ValueError as exc:
                    if count <= 20000 or 'capacity' not in str(exc):
                        raise
                    rejected = True
                converted = time.perf_counter()
                if not rejected:
                    graph.replace(0, receiver.staged[0])
                replaced = time.perf_counter()
                if not rejected:
                    memory.reproject(graph)
                finished = time.perf_counter()
                if not rejected and objects:
                    assert memory.revision == 1
                    assert all(record.support_valid for record in memory.objects.values())
                if iteration >= args.warmup:
                    samples['deserialize'].append(deserialized - started)
                    samples['convert'].append(converted - deserialized)
                    if not rejected:
                        samples['replace'].append(replaced - converted)
                        samples['reproject'].append(finished - replaced)
                        totals.append(finished - started)
            results.append({'nodes': count, 'objects': objects, 'capacity_rejected': rejected,
                            'serialized_bytes': len(serialized),
                            'stages': {k: summary(v) for k, v in samples.items() if v},
                            'total': summary(totals) if totals else None,
                            'process_peak_rss_bytes': peak_rss_bytes()})
    report = {'platform': platform.platform(), 'python': platform.python_version(),
              'numpy': np.__version__, 'anchors': args.anchors, 'points': args.points,
              'seed': args.seed, 'warmup': args.warmup, 'repeat': args.repeat,
              'results': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('graph benchmark: ' + '; '.join(
        f"{r['nodes']} nodes/{r['objects']} objects: "
        + ('capacity rejected' if r['capacity_rejected'] else ', '.join(
            f"{k} {v['median_ms']:.1f} ms" for k, v in r['stages'].items()))
        for r in results) + f'; JSON: {args.output}')
    if args.max_total_ms > 0 and any(
            r['total'] and r['total']['max_ms'] > args.max_total_ms for r in results):
        raise SystemExit('Complete graph-processing cycle exceeded the requested budget')


if __name__ == '__main__':
    main()

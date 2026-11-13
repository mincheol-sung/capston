#!/usr/bin/env python3
"""Benchmark dirty saves and reloads with deterministic, bounded graph evidence."""

import argparse
import importlib
import json
from pathlib import Path
import platform
import resource
import sys
import time

import numpy as np


def make_memory(count, anchors=16, points=512, seed=42, path=''):
    """Create the same valid input for old and new persistence implementations."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    module = importlib.import_module('semantic_slam.object_map')
    graph = importlib.import_module('semantic_slam.pose_graph')
    memory = module.ObjectMap(
        'benchmark-map', path, graph_id='benchmark-graph', max_objects=max(1, count),
        max_anchors=max(3, anchors), max_points=points)
    rng = np.random.default_rng(seed)
    covariance = (np.eye(3) * 0.01).ravel().tolist()
    pose = np.column_stack((np.eye(3), np.zeros(3))).ravel().tolist()
    for index in range(count):
        evidence = []
        for anchor_index in range(anchors):
            stamp = 1 + anchor_index // 10 + (anchor_index % 10) * 100000000 * 1e-9
            local = rng.uniform(-0.5, 0.5, (points, 3)) + [index * 2.0, 0, 1]
            evidence.append({
                'graph_id': memory.graph_id, 'trajectory_id': 0,
                'left': stamp, 'right': stamp, 'weight': 0.0, 'stamp': stamp,
                'points': (graph.FrozenPoints(local) if hasattr(graph, 'FrozenPoints')
                           else local.tolist()),
                'covariance': covariance[:], 'pose': pose[:],
            })
        position, center, extent = memory._geometry(local)
        record = module.ObjectRecord(
            f'object-{index:04d}', 56, 'chair', position, center, extent,
            local.tolist(), covariance[:], stamp, count=anchors, status=1,
            alias=f'chair-{index}', anchors=evidence,
            history=[[stamp, *position]])
        memory.objects[record.object_id] = record
    memory.dirty = True
    return memory


def summary(samples):
    """Keep raw samples alongside robust descriptive statistics in milliseconds."""
    values = np.asarray(samples) * 1000
    return {'samples_ms': values.tolist(), 'median_ms': float(np.median(values)),
            'p95_ms': float(np.percentile(values, 95)), 'max_ms': float(values.max())}


def peak_rss_bytes():
    """Report process high-water RSS, not current RSS or an isolated operation peak."""
    value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(value if sys.platform == 'darwin' else value * 1024)


def main():
    """Measure complete saves including fsync; exclude synthetic input construction."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--objects', nargs='+', type=int, default=[20, 200])
    parser.add_argument('--anchors', type=int, default=16)
    parser.add_argument('--points', type=int, default=512)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--repeat', type=int, default=10)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (min(args.objects) < 1 or args.anchors < 3 or args.points < 3
            or args.repeat < 1 or args.warmup < 0):
        parser.error('Positive capacities/repeats and nonnegative warmup are required')
    # Exclusive creation prevents accidentally reusing a user's DB or prior run.
    args.work_dir.mkdir(parents=True, exist_ok=False)
    results = []
    for count in args.objects:
        memory = make_memory(count, args.anchors, args.points, args.seed)
        memory.path = args.work_dir / f'objects-{count}.json'
        saves, loads = [], []
        for iteration in range(args.warmup + args.repeat):
            memory.dirty = True  # Every sample must serialize the full dirty map.
            started = time.perf_counter()
            memory.save()
            saved = time.perf_counter()
            loaded = type(memory)(
                memory.map_id, str(memory.path), graph_id=memory.graph_id,
                max_objects=count, max_points=args.points, max_anchors=args.anchors)
            finished = time.perf_counter()
            assert set(loaded.objects) == set(memory.objects)
            assert loaded.revision == memory.revision
            for object_id, record in memory.objects.items():
                restored = loaded.objects[object_id]
                assert restored.anchors == record.anchors
                assert restored.alias == record.alias and restored.last_seen == record.last_seen
            if iteration >= args.warmup:
                saves.append(saved - started)
                loads.append(finished - saved)
            del loaded
        results.append({'objects': count, 'anchors': args.anchors, 'points': args.points,
                        'save': summary(saves), 'load': summary(loads),
                        'file_bytes': memory.path.stat().st_size,
                        'process_peak_rss_bytes': peak_rss_bytes()})
    report = {'platform': platform.platform(), 'python': platform.python_version(),
              'numpy': np.__version__, 'seed': args.seed, 'warmup': args.warmup,
              'repeat': args.repeat, 'results': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print('save benchmark: ' + '; '.join(
        f"{r['objects']} objects: save {r['save']['median_ms']:.1f} ms, "
        f"load {r['load']['median_ms']:.1f} ms, {r['file_bytes'] / 1e6:.2f} MB"
        for r in results) + f'; JSON: {args.output}')


if __name__ == '__main__':
    main()

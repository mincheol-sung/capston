#!/usr/bin/env python3
"""Exercise installed graph processing and persistence without DDS or motor I/O."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import logging
import multiprocessing
from pathlib import Path
import time
from types import SimpleNamespace

from bench_save import make_memory
from bench_save import peak_rss_bytes
from bench_save import summary
from cartographer_ros_msgs.srv import TrajectoryQuery
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rclpy.time import Time
from semantic_slam.cartographer_adapter import CartographerAdapter
from semantic_slam.object_map import ObjectMap
from semantic_slam.semantic_slam_node import SemanticSlamNode


def rss_bytes():
    """Read current resident memory; high-water memory is recorded separately."""
    for line in Path('/proc/self/status').read_text().splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1]) * 1024
    raise RuntimeError('Current RSS is unavailable')


def evidence_digest(memory):
    """Hash raw sensor points independently of the optimized output geometry."""
    raw = [[anchor['points'] for anchor in record.anchors]
           for record in memory.objects.values()]
    return hashlib.sha256(json.dumps(raw, separators=(',', ':')).encode()).hexdigest()


def verify_database(path, map_id, graph_id, objects, points, anchors, aliases, digest):
    """Check atomic DB snapshots in a separate process, outside mapper timing."""
    restored = ObjectMap(map_id, path, graph_id=graph_id, max_objects=objects,
                         max_points=points, max_anchors=anchors)
    assert {key: record.alias for key, record in restored.objects.items()} == aliases
    assert evidence_digest(restored) == digest
    return peak_rss_bytes()


def run(args):
    """Run full response decode/adapter commit with a fixed 200-object capacity."""
    args.work_dir.mkdir(parents=True, exist_ok=False)
    memory = make_memory(args.objects, args.anchors, args.points)
    memory.path = args.work_dir / 'objects.json'
    identities = set(memory.objects)
    aliases = {key: record.alias for key, record in memory.objects.items()}
    raw_digest = evidence_digest(memory)
    expected_positions = np.array([
        np.median([np.median(anchor['points'], axis=0)
                   for anchor in record.anchors[-8:]], axis=0)
        for record in memory.objects.values()])
    response = TrajectoryQuery.Response()
    poses = []
    for index in range(args.nodes):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 1 + index // 10
        pose.header.stamp.nanosec = index % 10 * 100000000
        pose.pose.orientation.w = 1.
        poses.append(pose)
    response.trajectory = poses
    latest_ns = (poses[-1].header.stamp.sec * 1000000000
                 + poses[-1].header.stamp.nanosec)
    logger = logging.getLogger('semantic-soak')
    node = SimpleNamespace(
        create_client=lambda *_: None, get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=latest_ns)))
    adapter = CartographerAdapter(node, memory, {
        'cartographer_graph_id': memory.graph_id, 'graph_max_interpolation_gap': 1.,
        'graph_max_nodes': args.nodes, 'cartographer_states_service': '/unused/states',
        'cartographer_query_service': '/unused/query', 'graph_poll_period': 1.,
        'graph_service_timeout': 3., 'graph_max_trajectories': 8,
        'graph_freshness': 3., 'graph_node_timeout': 5., 'map_frame': 'map',
    })
    adapter.active = 0
    outputs = {}

    def publisher(name):
        def publish(message):
            outputs[name] = ({obj.object_id for obj in message.objects}
                             if name == 'objects' else len(message.markers))
            serialize_message(message)
        return SimpleNamespace(publish=publish)

    mapper = SimpleNamespace(
        memory=memory, carto=adapter, p={'map_frame': 'map'}, get_clock=node.get_clock,
        pub=publisher('objects'), markers=publisher('markers'))
    samples, graph_samples, publish_samples, save_samples, rss_samples = [], [], [], [], []
    rejected, saves, reloads, missed, overruns = 0, 0, 0, 0, 0
    worker_peak, verification = 0, None
    started = time.monotonic()
    next_cycle = started
    next_save, next_verify, next_report = started + 10., started + 60., started + 60.
    cycles = 0
    with (ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
          as pool, (args.work_dir / 'cycles.jsonl').open('w') as log):
        while time.monotonic() - started < args.duration:
            time.sleep(max(0., next_cycle - time.monotonic()))
            lateness = time.monotonic() - next_cycle
            if lateness > args.period:
                missed += 1
            translation = np.array([.1 + .001 * (cycles % 2), .001 * (cycles % 3), 0.])
            for pose in poses:
                pose.pose.position.x, pose.pose.position.y = translation[:2]
            # The response fixture is the external Cartographer's work. Measure
            # the complete semantic processing interval, including all decoding.
            serialized = serialize_message(response)
            inject = cycles > 0 and cycles % 300 == 0
            if inject:
                poses[0].pose.orientation.w = 0.
                serialized = serialize_message(response)
                poses[0].pose.orientation.w = 1.
            adapter.current, adapter.staged, adapter.queue = 0, {}, []
            previous_graph = adapter.graph
            processing_started = time.perf_counter()
            decoded = deserialize_message(serialized, TrajectoryQuery.Response)
            try:
                adapter._trajectory(decoded)
                if inject:
                    raise AssertionError('Malformed graph was accepted')
            except ValueError:
                if not inject:
                    raise
                adapter._fail('injected malformed quaternion')
                assert adapter.graph is previous_graph and not adapter.ready
                rejected += 1
            graph_samples.append(time.perf_counter() - processing_started)
            publish_started = time.perf_counter()
            SemanticSlamNode._publish(mapper)
            publish_samples.append(time.perf_counter() - publish_started)
            now = time.monotonic()
            saved_ms = 0.
            if now >= next_save and not inject:
                save_started = time.perf_counter()
                memory.save()
                save_samples.append(time.perf_counter() - save_started)
                saved_ms = save_samples[-1] * 1000
                saves += 1
                next_save = now + 10.
            elapsed = time.perf_counter() - processing_started
            samples.append(elapsed)
            if elapsed * 1000 > args.max_total_ms:
                overruns += 1
                print(f'Budget exceeded at cycle {cycles}: {elapsed * 1000:.3f}ms > '
                      f'{args.max_total_ms:.3f}ms', flush=True)
            assert set(memory.objects) == identities
            assert outputs['objects'] == identities
            assert outputs['markers'] == args.objects * 2 + 1
            assert {key: record.alias for key, record in memory.objects.items()} == aliases
            if not inject:
                assert adapter.ready and all(r.support_valid for r in memory.objects.values())
                actual = np.array([r.position for r in memory.objects.values()])
                assert np.allclose(actual, expected_positions + translation, atol=1e-9, rtol=0)
            now = time.monotonic()
            if verification is not None and verification.done():
                worker_peak = max(worker_peak, verification.result())
                verification = None
                reloads += 1
            if now >= next_verify and saves and verification is None:
                verification = pool.submit(
                    verify_database, str(memory.path), memory.map_id, memory.graph_id,
                    args.objects, args.points, args.anchors, aliases, raw_digest)
                next_verify = now + 60.
            resident = rss_bytes()
            assert resident <= args.max_rss_mb * 1024**2, 'RSS budget exceeded'
            rss_samples.append((now - started, resident))
            log.write(json.dumps({'cycle': cycles, 'elapsed_s': now - started,
                                  'total_ms': elapsed * 1000, 'rss_bytes': resident,
                                  'graph_ms': graph_samples[-1] * 1000,
                                  'publish_ms': publish_samples[-1] * 1000,
                                  'save_ms': saved_ms,
                                  'budget_exceeded': elapsed * 1000 > args.max_total_ms,
                                  'injected_rejection': inject, 'lateness_ms': lateness * 1000})
                      + '\n')
            log.flush()
            cycles += 1
            next_cycle += args.period
            if now >= next_report:
                print(f'{now-started:.0f}/{args.duration:.0f}s: {cycles} cycles, '
                      f'max {max(samples)*1000:.1f}ms, RSS {resident/1024**2:.0f}MiB, '
                      f'saves {saves}, reloads {reloads}, expected rejections {rejected}, '
                      f'budget overruns {overruns}',
                      flush=True)
                next_report = now + 60.
        if verification is not None:
            worker_peak = max(worker_peak, verification.result(timeout=30.))
            reloads += 1
    assert evidence_digest(memory) == raw_digest
    memory.save()
    # Compare post-warm-up first five minutes with the last five minutes. This
    # budget detects sustained growth; it is not a hardware safety threshold.
    initial = [value for elapsed, value in rss_samples if 120 <= elapsed <= 420]
    final = [value for elapsed, value in rss_samples if elapsed >= args.duration - 300]
    growth = float(np.median(final) - np.median(initial)) if initial and final else 0.
    assert growth <= args.max_growth_mb * 1024**2, 'Sustained RSS growth exceeded budget'
    return {'duration_s': time.monotonic() - started, 'cycles': cycles,
            'objects': args.objects, 'nodes': args.nodes, 'anchors': args.anchors,
            'points': args.points, 'total': summary(samples),
            'graph': summary(graph_samples), 'publish': summary(publish_samples),
            'save': summary(save_samples) if save_samples else None,
            'saves': saves + 1, 'reloads': reloads, 'expected_rejections': rejected,
            'budget_overruns': overruns, 'passed': overruns == 0,
            'missed_cycle_deadlines': missed, 'rss_growth_bytes': growth,
            'max_rss_bytes': max(value for _, value in rss_samples),
            'process_peak_rss_bytes': peak_rss_bytes(), 'raw_evidence_unchanged': True,
            'verification_worker_peak_rss_bytes': worker_peak,
            'motor_io': False, 'dds_context_created': False,
            'timing_scope': 'decode, adapter commit, reprojection, output message/CDR '
                            'generation, and synchronous save when due'}


def main():
    """Leave a failure report rather than silently truncating a soak."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=1800.)
    parser.add_argument('--period', type=float, default=1.)
    parser.add_argument('--objects', type=int, default=200)
    parser.add_argument('--nodes', type=int, default=20000)
    parser.add_argument('--anchors', type=int, default=16)
    parser.add_argument('--points', type=int, default=512)
    parser.add_argument('--max-total-ms', type=float, default=1000.)
    parser.add_argument('--max-rss-mb', type=float, default=1536.)
    parser.add_argument('--max-growth-mb', type=float, default=128.)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (args.duration <= 0 or args.period <= 0 or args.objects < 1
            or args.nodes < args.anchors or args.anchors < 3 or args.points < 3):
        parser.error('Invalid soak capacities or timing')
    try:
        report = run(args)
    except Exception as exc:
        report = {'passed': False, 'error': f'{type(exc).__name__}: {exc}'}
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        raise
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    if not report['passed']:
        raise SystemExit('Completed soak failed the processing-time budget')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Summarize three measured filter replays with common regenerated tracking inputs."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import matplotlib
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py as bag_api
from sensor_msgs.msg import LaserScan

matplotlib.use('Agg')
METHODS = ('radius', 'cluster', 'tracked_cluster')
COLORS = ('#ce6945', '#298577', '#7466a9')


def open_scans(path):
    """Read the original-order scans from a prepared SLAM input bag."""
    reader = bag_api.SequentialReader()
    reader.open(bag_api.StorageOptions(uri=str(path), storage_id='mcap'),
                bag_api.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(bag_api.StorageFilter(topics=['/scan_1', '/scan_2']))
    return reader


def main():
    """Check common inputs, compare individual beams and plot measured differences."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--example-time', type=float, default=135.327)
    parser.add_argument('--reference', type=Path,
                        help='Previous two-method result folder: verify its exact scene and view')
    args = parser.parse_args()
    root = args.root
    rebuilt = json.loads((root / 'rebuilt/rebuild_summary.json').read_text())
    reports, rows, frames, summary = {}, {}, {}, {}
    for method in METHODS:
        folder = root / method
        report = json.loads((folder / 'filter_summary.json').read_text())
        reports[method] = report
        assert report['source_slam_sha256'] == rebuilt['source_slam_sha256']
        assert report['start_ns'] == rebuilt['original_epoch_ns']
        with (folder / 'scan_metrics.csv').open() as stream:
            rows[method] = list(csv.DictReader(stream))
        frames[method] = {(f['topic'], f['t']): f for f in
                          json.loads((folder / 'frames.json').read_text())}
        counts = Counter()
        for row in rows[method]:
            counts['scans'] += 1
            counts['valid_points'] += int(row['valid'])
            counts['removed_points'] += int(row['removed'])
            counts['changed_scans'] += int(int(row['removed']) > 0)
            counts['reason_' + row['reason']] += 1
            counts.update({k: int(v) for k, v in row.items()
                           if v and (k.startswith('observation') or k.startswith('clusters_'))})
        delays = [float(row['delay_s']) for row in rows[method]]
        summary[method] = {**counts,
                           'removed_percent': 100*counts['removed_points']/counts['valid_points'],
                           'virtual_delay_p50_s': float(np.median(delays)),
                           'virtual_delay_p95_s': float(np.percentile(delays, 95))}
    # Scan-by-scan, beam-by-beam comparison over the entire bag, not just screenshots.
    readers = {'raw': open_scans(root / 'radius/off')}
    readers.update({m: open_scans(root / m / 'on') for m in METHODS})
    masks = Counter()
    scans = 0
    while readers['raw'].has_next():
        messages = {}
        identity = None
        for method, reader in readers.items():
            assert reader.has_next()
            topic, data, receipt = reader.read_next()
            msg = deserialize_message(data, LaserScan)
            current = (topic, receipt, msg.header.frame_id,
                       msg.header.stamp.sec, msg.header.stamp.nanosec)
            if identity is None:
                identity = current
            assert current == identity
            messages[method] = np.asarray(msg.ranges)
        raw = messages['raw']
        valid = np.isfinite(raw)
        bitmask = np.zeros(len(raw), dtype=np.uint8)
        for bit, method in enumerate(METHODS):
            changed = valid & np.isnan(messages[method])
            bitmask[changed] |= 1 << bit
            np.testing.assert_equal(messages[method][~changed], raw[~changed])
        masks.update({str(int(code)): int(count) for code, count in
                      zip(*np.unique(bitmask[bitmask > 0], return_counts=True))})
        scans += 1
    assert all(not reader.has_next() for reader in readers.values())
    assert scans == summary['radius']['scans']
    del readers
    for bit, method in enumerate(METHODS):
        assert sum(v for k, v in masks.items() if int(k) & (1 << bit)) == summary[
            method]['removed_points']

    windows = []
    for begin, end in ((0., 20.), (120., 140.), (200., 220.), (280., 300.)):
        window = {'start_s': begin, 'end_s': end}
        for method in METHODS:
            selected = [r for r in rows[method] if begin <= float(r['t']) < end]
            window[method] = {'scans': len(selected),
                              'removed': sum(int(r['removed']) for r in selected)}
        windows.append(window)
    result = {'source_bag': rebuilt['original_bag'], 'common_regenerated_tracking': True,
              'original_epoch_ns': rebuilt['original_epoch_ns'],
              'source_slam_sha256': rebuilt['source_slam_sha256'], 'methods': summary,
              'whole_bag_removal_intersection': dict(masks),
              'mask_code': {'1': 'radius only', '2': 'cluster only',
                            '3': 'radius + cluster', '4': 'tracked_cluster only',
                            '5': 'radius + tracked_cluster', '6': 'cluster + tracked_cluster',
                            '7': 'all three'}, 'windows': windows,
              'limitations': rebuilt['limitations'] + [
                  'Input-filter comparison only; no Cartographer rerun or accuracy measurement.',
                  'Removal membership differences do not identify people or walls.']}
    (root / 'three_methods.json').write_text(json.dumps(result, indent=2))
    table = [{'method': m, **{k: summary[m][k] for k in (
        'scans', 'valid_points', 'removed_points', 'removed_percent', 'changed_scans',
        'virtual_delay_p50_s', 'virtual_delay_p95_s')}} for m in METHODS]
    with (root / 'three_methods.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    heights = [summary[m]['removed_percent'] for m in METHODS]
    bars = axes[0].bar(METHODS, heights, color=COLORS, width=.55)
    labels = [f'{summary[m]["removed_points"]:,}\n{h:.3f}%'
              for m, h in zip(METHODS, heights)]
    axes[0].bar_label(bars, labels=labels, padding=4)
    axes[0].set(ylabel='Removed / valid returns [%]', title='Whole bag removal count',
                ylim=(0, max(heights)*1.3))
    duration = max(float(row['t']) for row in rows['radius'])
    bins = np.arange(0., np.ceil(duration/5.)*5. + 1., 5.)
    for method, color in zip(METHODS, COLORS):
        t = [float(r['t']) for r in rows[method]]
        removed = np.histogram(t, bins, weights=[int(r['removed']) for r in rows[method]])[0]
        valid = np.histogram(t, bins, weights=[int(r['valid']) for r in rows[method]])[0]
        axes[1].stairs(removed/np.maximum(valid, 1)*100, bins, baseline=None,
                       color=color, label=method)
    axes[1].set(xlabel='Original bag time [s]', ylabel='Removed / valid returns [%]',
                title='5-second intervals')
    axes[1].legend()
    for ax in axes:
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
    bag_name = Path(rebuilt['original_bag']).name
    fig.suptitle(f'{bag_name} | same sensor bag + common regenerated tracking')
    fig.text(.5, .015, 'Counts are not accuracy. New clustering/fusion/tracking; '
             'recorded camera detections reused. No ground truth.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .07, 1, .94))
    fig.savefig(root / 'three_methods.png', dpi=170)
    plt.close(fig)

    identities = []
    reference = None
    if args.reference:
        reference = json.loads((args.reference / 'method_comparison.json').read_text())
        assert reference['source_slam_sha256'] == rebuilt['source_slam_sha256']
    for topic in ('/scan_front_carto', '/scan_rear_carto'):
        candidates = [k for k in frames['radius'] if k[0] == topic]
        target = reference['example_times_s'][topic] if reference else args.example_time
        selected = min(candidates, key=lambda k: abs(k[1]-target))
        if reference:
            assert selected[1] == target, 'The requested reference scan is missing'
        identities.append(selected)
    views = {}
    for method in METHODS:
        data = [np.load(root / method / 'frames' / frames[method][k]['file']) for k in identities]
        views[method] = {k: np.concatenate([d[k] for d in data])
                         for k in ('points', 'removed', 'centres')}
        for d in data:
            d.close()
    points = views['radius']['points']
    centres = views['radius']['centres']
    focus = centres if len(centres) else points
    lower, upper = focus.min(axis=0)-.8, focus.max(axis=0)+.8
    example = {'times_s': dict(identities), 'removed_points': {
        m: int(views[m]['removed'].sum()) for m in METHODS}}
    if reference:
        previous = {}
        for method in ('radius', 'cluster'):
            folder = args.reference / method
            index = {(f['topic'], f['t']): f for f in
                     json.loads((folder / 'frames.json').read_text())}
            data = [np.load(folder / 'frames' / index[k]['file']) for k in identities]
            previous[method] = {k: np.concatenate([d[k] for d in data])
                                for k in ('points', 'removed', 'centres')}
            for d in data:
                d.close()
            np.testing.assert_array_equal(previous[method]['points'], points)
        reference_centres = previous['radius']['centres']
        focus = reference_centres if len(reference_centres) else points
        lower, upper = focus.min(axis=0)-1., focus.max(axis=0)+1.
        example['reference'] = {'folder': str(args.reference),
                                'raw_points_exactly_equal': True,
                                'removed_points': {m: int(previous[m]['removed'].sum())
                                                   for m in previous},
                                'note': 'Previous image used historical IMM. Current comparison '
                                        'uses common regenerated IMM and actual assignments.'}
    example['view_bounds'] = {'lower': lower.tolist(), 'upper': upper.tolist()}
    (root / 'example_comparison.json').write_text(json.dumps(example, indent=2))
    fig, axes = plt.subplots(1, 4, figsize=(19, 6), sharex=True, sharey=True)
    for ax, method in zip(axes, ('raw', *METHODS)):
        ax.scatter(points[:, 0], points[:, 1], s=5, c='#586776')
        if method != 'raw':
            np.testing.assert_array_equal(views[method]['points'], points)
            mask = views[method]['removed']
            ax.scatter(points[mask, 0], points[mask, 1], s=11, c='#cf4141',
                       label=f'{int(mask.sum())} removed')
            ax.legend(fontsize=8, loc='upper right')
        if method == 'radius':
            for centre in centres:
                ax.add_patch(Circle(centre, .35, fill=False, color='#287c96', linestyle='--'))
        if method in ('radius', 'cluster') and len(centres):
            ax.scatter(centres[:, 0], centres[:, 1], marker='+', color='#287c96', s=45)
        ax.set(xlim=(lower[0], upper[0]), ylim=(lower[1], upper[1]),
               title=method, aspect='equal', xlabel='odom x [m]')
        ax.grid(alpha=.2)
    axes[0].set_ylabel('odom y [m]')
    fig.suptitle(f'{bag_name} | front {identities[0][1]:.3f}s / rear '
                 f'{identities[1][1]:.3f}s | common regenerated tracking')
    fig.text(.5, .025, 'Red = removed. No human/wall labels. Exact membership removal '
             'does not prove that the upstream association was correct.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .1, 1, .94))
    fig.savefig(root / 'three_methods_example.png', dpi=170)
    plt.close(fig)
    print(json.dumps(table, indent=2))


if __name__ == '__main__':
    main()

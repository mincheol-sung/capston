#!/usr/bin/env python3
"""Compare radius/cluster replay artifacts without treating point counts as accuracy."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import matplotlib
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('Agg')


def load_run(root):
    """Read the measured outputs of compare_bag.py."""
    report = json.loads((root / 'filter_summary.json').read_text())
    with (root / 'scan_metrics.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    frames = json.loads((root / 'frames.json').read_text())
    return report, rows, {(f['topic'], f['t']): f for f in frames}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--radius', type=Path, required=True)
    parser.add_argument('--cluster', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--time', type=float, default=135.327,
                        help='Fixed example time in seconds, not a best-improvement selection')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    runs = {method: load_run(root) for method, root in
            (('radius', args.radius), ('cluster', args.cluster))}
    reports = [run[0] for run in runs.values()]
    assert reports[0]['source_slam_sha256'] == reports[1]['source_slam_sha256']
    assert reports[0]['start_ns'] == reports[1]['start_ns']
    assert reports[0]['input_topic_counts'] == reports[1]['input_topic_counts']
    for key, value in reports[0]['config'].items():
        if key != 'mask_method' and not key.startswith('cluster_'):
            assert reports[1]['config'][key] == value, f'Unmatched comparison setting: {key}'
    assert reports[0]['config'].get('mask_method', 'radius') == 'radius'
    assert reports[1]['config']['mask_method'] == 'cluster'
    identities = [{(r['topic'], r['t']): r['valid'] for r in run[1]} for run in runs.values()]
    assert identities[0] == identities[1]
    summary = {'source_slam_sha256': reports[0]['source_slam_sha256'],
               'input': reports[0]['input'], 'methods': {},
               'limitation': 'Counts and mask differences only; no ground-truth labels, '
                             'localization accuracy, submap accuracy or real-time latency claim.'}
    for method, (report, rows, _) in runs.items():
        counts = Counter()
        for data in report['totals'].values():
            counts.update(data)
        cluster_counts = Counter()
        for row in rows:
            cluster_counts.update({k: int(v) for k, v in row.items()
                                   if k.startswith('clusters_') and v})
        summary['methods'][method] = {**counts, **cluster_counts}

    # Exact per-beam agreement is evaluated for saved visualization frames only.
    ra, cl = runs['radius'][2], runs['cluster'][2]
    assert ra.keys() == cl.keys()
    agreement = Counter()
    for identity in ra:
        with np.load(args.radius / 'frames' / ra[identity]['file']) as a, \
                np.load(args.cluster / 'frames' / cl[identity]['file']) as b:
            np.testing.assert_array_equal(a['points'], b['points'])
            x, y = a['removed'], b['removed']
            agreement['both_removed'] += int(np.count_nonzero(x & y))
            agreement['radius_only_removed'] += int(np.count_nonzero(x & ~y))
            agreement['cluster_only_removed'] += int(np.count_nonzero(~x & y))
            agreement['both_kept'] += int(np.count_nonzero(~x & ~y))
    summary['saved_interval_agreement'] = dict(agreement)
    summary['saved_interval_s'] = reports[0]['visual_interval_s']

    selected = []
    for topic in ('/scan_front_carto', '/scan_rear_carto'):
        identities = [key for key in ra if key[0] == topic]
        selected.append(min(identities, key=lambda key: abs(key[1] - args.time)))
    frames = {}
    for method, root in (('radius', args.radius), ('cluster', args.cluster)):
        items = [np.load(root / 'frames' / runs[method][2][key]['file']) for key in selected]
        frames[method] = {name: np.concatenate([item[name] for item in items])
                          for name in ('points', 'removed', 'centres')}
        for item in items:
            item.close()
    points = frames['radius']['points']
    centres = frames['radius']['centres']
    extent = centres if len(centres) else points
    lower, upper = extent.min(axis=0)-1., extent.max(axis=0)+1.
    fig, axes = plt.subplots(1, 3, figsize=(14, 5.8), sharex=True, sharey=True)
    for ax, method in zip(axes, ('raw', 'radius', 'cluster')):
        ax.scatter(points[:, 0], points[:, 1], s=6, c='#687583', label='Observed returns')
        if method != 'raw':
            removed = frames[method]['removed']
            ax.scatter(points[removed, 0], points[removed, 1], s=13,
                       c='#d64545', label=f'Removed: {int(removed.sum())}')
            for centre in centres:
                ax.plot(*centre, '+', color='#126b91', markersize=9)
                if method == 'radius':
                    ax.add_patch(Circle(centre, reports[0]['config']['filter_radius'],
                                        fill=False, linestyle='--', edgecolor='#126b91'))
        ax.set(xlim=(lower[0], upper[0]), ylim=(lower[1], upper[1]),
               aspect='equal', title=method.upper(), xlabel='odom x [m]')
        ax.grid(alpha=.2)
        ax.legend(loc='upper right', fontsize=8)
    axes[0].set_ylabel('odom y [m]')
    fig.suptitle(f'Recorded LiDAR | front {selected[0][1]:.3f}s, rear {selected[1][1]:.3f}s')
    fig.text(.5, .02, 'Red: removed returns. No human/wall ground-truth labels. '
             'Fewer removed points does not establish better filtering.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .10, 1, .95))
    fig.savefig(args.output / 'methods_example.png', dpi=170)
    plt.close(fig)
    summary['example_times_s'] = dict(selected)

    fig, ax = plt.subplots(figsize=(12, 4.5))
    bins = np.arange(0, reports[0]['duration_s'] + 5., 5.)
    for method, (_, rows, _) in runs.items():
        times = [float(r['t']) for r in rows]
        removed = np.histogram(times, bins, weights=[int(r['removed']) for r in rows])[0]
        valid = np.histogram(times, bins, weights=[int(r['valid']) for r in rows])[0]
        ax.step(bins[:-1], 100*removed/np.maximum(valid, 1), where='post', label=method)
    ax.set(xlabel='Bag time [s]', ylabel='Removed / valid returns [%]',
           title='Identical recorded input | removal rate, not accuracy')
    ax.grid(alpha=.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output / 'methods_timeline.png', dpi=170)
    plt.close(fig)
    (args.output / 'method_comparison.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

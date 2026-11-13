#!/usr/bin/env python3
"""Compare stored graph consistency; none of these metrics is ground-truth error."""

import argparse
import csv
import json
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation


def rigid(pose):
    t, q = pose['translation'], pose['rotation']
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q[k] for k in ('x', 'y', 'z', 'w')]).as_matrix()
    result[:3, 3] = [t[k] for k in ('x', 'y', 'z')]
    return result


def invert(pose):
    result = np.eye(4)
    result[:3, :3] = pose[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ pose[:3, 3]
    return result


def stats(values):
    values = np.asarray(values)
    if not len(values):
        return {'count': 0}
    return {'count': len(values), 'mean': float(values.mean()),
            'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
            'rms': float(np.sqrt(np.mean(values**2)))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    root = args.input
    start_ns = json.loads((root / 'filter_summary.json').read_text())['start_ns']
    graphs, rows, summary, score_rows = {}, [], {}, []
    score_pattern = re.compile(
        r'Node \((\d+), (\d+)\).*?submap \((\d+), (\d+)\).*?score ([0-9.]+)%')
    for variant in ('off', 'on'):
        graph = json.loads((root / f'mapping/{variant}_pose_graph.json').read_text())
        nodes, submaps = {}, {}
        for trajectory in graph['trajectory']:
            tid = trajectory['trajectory_id']
            for node in trajectory['node']:
                # Cartographer UTS timestamps: 100ns ticks since 0001-01-01.
                ns = (int(node['timestamp']) - 62135596800*10_000_000)*100
                nodes[(tid, node['node_index'])] = {
                    'ns': ns, 't': (ns-start_ns)*1e-9, 'pose': rigid(node['pose'])}
            for submap in trajectory['submap']:
                submaps[(tid, submap['submap_index'])] = rigid(submap['pose'])
        for constraint in graph['constraint']:
            sid, nid = constraint['submap_id'], constraint['node_id']
            skey, nkey = (sid['trajectory_id'], sid['submap_index']), (
                nid['trajectory_id'], nid['node_index'])
            node = nodes[nkey]
            predicted = invert(submaps[skey]) @ node['pose']
            measured = rigid(constraint['relative_pose'])
            # Stored graph constraints and node poses both include gravity alignment.
            # A full rigid rotation residual has the same magnitude as the planar
            # yaw residual after cancelling the shared gravity rotation.
            translation = np.linalg.norm(predicted[:3, 3] - measured[:3, 3])
            rotation = Rotation.from_matrix(
                measured[:3, :3].T @ predicted[:3, :3]).magnitude()
            rows.append({'variant': variant, 'tag': constraint['tag'],
                         't': node['t'], 'node_index': nkey[1], 'submap_index': skey[1],
                         'translation_m': translation, 'rotation_deg': np.degrees(rotation)})
        scores = score_pattern.findall((root / f'mapping/{variant}.log').read_text())
        for tid, nid, sid, subid, score in scores:
            node = nodes[(int(tid), int(nid))]
            score_rows.append({'variant': variant, 't': node['t'], 'node_index': int(nid),
                               'submap_index': int(subid), 'score_percent': float(score)})
        summary[variant] = {'nodes': len(nodes), 'submaps': len(submaps), 'intervals': {}}
        for label, low, high in (('whole_bag', -np.inf, np.inf), ('120_140s', 120., 140.)):
            entry = {}
            for tag in ('INTRA_SUBMAP', 'INTER_SUBMAP'):
                selected = [r for r in rows if r['variant'] == variant
                            and r['tag'] == tag and low <= r['t'] < high]
                entry[tag] = {key: stats([r[key] for r in selected])
                              for key in ('translation_m', 'rotation_deg')}
            entry['accepted_scan_match_score_percent'] = stats([
                r['score_percent'] for r in score_rows
                if r['variant'] == variant and low <= r['t'] < high])
            summary[variant]['intervals'][label] = entry
        graphs[variant] = {node['ns']: node for node in nodes.values()}
    common = sorted(graphs['off'].keys() & graphs['on'].keys())
    anchors = {v: invert(graphs[v][common[0]]['pose']) for v in ('off', 'on')}
    differences = []
    for ns in common:
        off = anchors['off'] @ graphs['off'][ns]['pose']
        on = anchors['on'] @ graphs['on'][ns]['pose']
        differences.append({'t': graphs['off'][ns]['t'],
                            'position_difference_m': np.linalg.norm(off[:3, 3]-on[:3, 3]),
                            'rotation_difference_deg': np.degrees(Rotation.from_matrix(
                                off[:3, :3].T @ on[:3, :3]).magnitude())})
    summary['trajectory_difference_not_accuracy'] = {
        'common_exact_timestamps': len(common),
        'alignment': 'Each trajectory rigidly anchored at its first common node; no scale fitting',
        **{key: stats([r[key] for r in differences])
           for key in ('position_difference_m', 'rotation_difference_deg')}}
    summary['limitations'] = [
        'No independent ground truth in this bag. This is mapping, not fixed-map localization.',
        'Graph residuals measure agreement with estimated constraints, not true geometric error.',
        'Accepted scores exclude failed attempts and OFF/ON use different node/submap sets.',
        'Scores and graph residuals are in-sample and influenced by optimization/selection.',
        'A smaller OFF/ON trajectory difference does not imply either trajectory is more accurate.'
    ]
    for filename, data in (('constraint_residuals.csv', rows), ('accepted_scores.csv', score_rows),
                           ('trajectory_differences.csv', differences)):
        with (root / filename).open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)
    (root / 'pose_graph_metrics.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

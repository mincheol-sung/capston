"""Replay cached dual-LiDAR snapshots through the production gap strategy.

Source ROS/workspace first. Reads recorded data only; publishes no commands.
This verifies geometry and static MPPI integration, not ROS or dynamic timing.
"""
from dataclasses import asdict, fields, replace
from pathlib import Path
import argparse
import json
import time

import numpy as np
import yaml

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.gap_geometry import GapConfig, partition
from dra_mppi.gap_inputs import GapScan, directional_depth, fuse_scans
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiController


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'docs/gap_multiscene_2026-11-04'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'docs/gap_strategy_2026-11-04/validation.json')
    args = parser.parse_args()
    parameters = yaml.safe_load((ROOT / 'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    gap = GapConfig(**{f.name: parameters['gap.' + f.name] for f in fields(GapConfig)
                      if 'gap.' + f.name in parameters})
    cfg = replace(config_from_yaml(), strategy='gap', gap=gap)
    references = json.loads((DATA / 'disparity_partition_10_scenes/results.json').read_text())
    audits = {r['scene']: r for line in (DATA / 'scene_audit.jsonl').read_text().splitlines()
              if (r := json.loads(line))['accepted']}
    rows = []
    for reference in references:
        scene = reference['scene']
        with np.load(DATA / f'disparity_partition_10_scenes/data/{scene}.npz') as loaded:
            data = dict(loaded)
        field = DistanceField.from_occupancy(data['grid'], GridMetadata(200, 200, .04, -4., -4., 0.))
        scans = tuple(GapScan(*(data[f'scan_{i}_{key}'] for key in ('ranges', 'valid', 'origins', 'hits')))
                      for i in range(2))
        tick = time.perf_counter()
        fused, _, local = fuse_scans(scans, np.zeros(3), field, None, gap, cfg.static_cost_robot_radius)
        _, strict, hard = directional_depth(fused, gap, cfg.static_cost_robot_radius)
        arrays = dict(strict_depth=strict, hard_depth=hard)
        for i, scan in enumerate(local):
            arrays.update({f'scan_{i}_{k}': scan[k] for k in ('ranges', 'valid', 'hits')})
        actual = partition(arrays, asdict(gap))
        geometry_ms = (time.perf_counter() - tick) * 1000
        same_regions = [r['bins'] for r in actual['regions']] == [r['bins'] for r in reference['regions']]
        # Re-use the recorded nominal, rather than confounding this comparison
        # with the different first-cycle path-guide initialization.
        with np.load(DATA / f'midpoint_value_v1/{scene}_seeds.npz') as seed:
            nominal = seed['nominal_controls'].copy()
        c = MppiController(cfg)
        try:
            c._guide_optimizer._warm_nominal = nominal
            tick = time.perf_counter()
            result = c.compute(np.zeros(3), data['full_path'], np.array(audits[scene]['current']),
                               distance_field=field, gap_scans=scans)
            compute_ms = (time.perf_counter() - tick) * 1000
            assert result.strategy == 'gap'
            assert result.sampled_controls.shape == (gap.total_rollouts, cfg.horizon_steps, 3)
            assert sum(result.guide_rollout_counts) == gap.total_rollouts
            rows.append(dict(scene=scene, reference_regions=reference['region_count'],
                             regions=actual['region_count'], exact_region_bins_match=same_regions,
                             max_strict_depth_difference_m=float(np.max(abs(strict-data['strict_depth']))),
                             geometry_ms=geometry_ms, compute_ms=compute_ms,
                             selected_mode=result.selected_mode, safety_status=result.safety_status,
                             **result.gap_diagnostics))
        finally:
            c.close()
    report = dict(scope='10 recorded dual-LiDAR snapshots, cached deskewed hits and fused upstream map; '
                        'production partition, midpoint gates, fixed 500 sampling and final static check. '
                        'No dynamic predictions supplied; excludes ROS, TF, publication and sensor deskew time. '
                        'Not a closed-loop or full 5 Hz verification.',
                  configuration=asdict(gap), rows=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(scenes=len(rows), exact_partitions=sum(r['exact_region_bins_match'] for r in rows),
                          regions=[r['regions'] for r in rows],
                          selected_guides=[len(r['selected']) for r in rows],
                          compute_ms=[round(r['compute_ms'], 2) for r in rows], output=str(args.output))))
    assert all(r['exact_region_bins_match'] for r in rows)


if __name__ == '__main__':
    main()

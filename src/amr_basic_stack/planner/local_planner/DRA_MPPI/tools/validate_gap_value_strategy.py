"""Replay recorded LiDAR/map/predictions through the production gap strategy.

No ROS node, publications or robot commands. Repeated frozen snapshots exercise
cache reuse and deadlines; they are not closed-loop or full callback timing.
"""
from dataclasses import asdict, fields, replace
from pathlib import Path
import argparse
import json
import os
import platform
import time

import numpy as np
import yaml

from benchmark_guide_conditioned import config_from_yaml, summary
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.gap_geometry import GapConfig
from dra_mppi.gap_inputs import GapScan
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiController


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/gap_value_optimization_2026-11-05'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=12)
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'docs/gap_value_implementation_2026-11-05')
    parser.add_argument('--optimization-budget-ms', type=float,
                        help='Diagnostic override only; omitted uses production YAML')
    parser.add_argument('--cycle-timeout-ms', type=float,
                        help='Diagnostic override only; omitted uses production YAML')
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error('iterations must be positive')
    p = yaml.safe_load((ROOT / 'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    gap = GapConfig(**{f.name: p['gap.' + f.name] for f in fields(GapConfig) if 'gap.' + f.name in p})
    if args.optimization_budget_ms is not None:
        gap=replace(gap,optimization_timeout=args.optimization_budget_ms/1000)
    if args.cycle_timeout_ms is not None:
        gap=replace(gap,timeout=args.cycle_timeout_ms/1000)
    cfg = replace(config_from_yaml(), strategy='gap', gap=gap)
    sources = json.loads((SOURCE / 'raw_sources.json').read_text())
    rows = []
    for source in sources:
        scene = source['scene']
        with np.load(SOURCE / 'raw' / (scene + '.npz')) as arrays:
            data = dict(arrays)
        field = DistanceField.from_occupancy(data['grid'], GridMetadata(200, 200, .04, -4., -4., 0.))
        scans = tuple(GapScan(*(data[f'scan_{i}_{key}'] for key in ('ranges', 'valid', 'origins', 'hits')))
                      for i in range(2))
        predictions = None
        if 'prediction_means' in data:
            predictions = PredictionSet(data['prediction_means'], data['prediction_covariances'],
                data['prediction_times'], mode_weights=data.get('prediction_mode_weights'),
                obstacle_radii=data.get('prediction_obstacle_radii'))
        controller = MppiController(cfg)
        try:
            for trial in range(args.iterations):
                # Fix the nominal to isolate proposal/cache changes at the same
                # recorded state. Production itself shifts its actual selection.
                controller._guide_optimizer._warm_nominal = data['nominal'].copy()
                row = dict(scene=scene, trial=trial, cold=trial == 0,
                           predictions=source['predictions'])
                tick = time.perf_counter()
                try:
                    result = controller.compute(np.zeros(3), data['path'].copy(), data['current'].copy(),
                        predictions, visualization_rollouts=32, distance_field=field, gap_scans=scans)
                    row['compute_ms'] = (time.perf_counter() - tick) * 1000
                    counts = np.asarray(result.guide_rollout_counts)
                    assert counts.sum() == gap.total_rollouts
                    assert counts[0] >= gap.nominal_min_rollouts
                    assert np.all(counts[1:] >= gap.guide_min_rollouts)
                    assert result.sampled_controls.shape == (gap.total_rollouts, cfg.horizon_steps, 3)
                    assert result.gap_diagnostics['target_policy']=='wheel_then_lidar'
                    for q in result.gap_diagnostics['candidates']:
                        if q['wheel_distance_m'] is not None:
                            assert q['actual_target_distance_m']==min(q['wheel_distance_m'],q['depth_m'])
                        if q['seed_source']=='optimized':
                            assert q['allocation_cost_source']=='optimized_actual'
                            assert q['wheel_value_cost'] is None
                            assert q['wheel_value_status']=='not_computed'
                            assert q['cost']==q['actual_value_cost']
                            assert q['target_distance_m']==q['actual_target_distance_m']
                            assert not q['actual_value_reused_from_wheel']
                        elif q['seed_source'] in ('reused','fallback'):
                            assert q['allocation_cost_source']==q['seed_source']+'_seed'
                    expected = np.vstack((result.control_sequence[1:], result.control_sequence[-1]))
                    selected_index = result.candidate_names.index(result.selected_mode)
                    selected_collision = bool(result.final_static_collisions[selected_index])
                    if result.safety_status == 'ok':
                        np.testing.assert_allclose(controller._guide_optimizer._warm_nominal, expected)
                        assert not selected_collision
                        assert result.max_collision_probability <= cfg.dynamic_risk.hard_threshold
                    row.update(status=result.safety_status, selected_mode=result.selected_mode,
                               cost=float(result.cost) if np.isfinite(result.cost) else None,
                               selected_static_collision=selected_collision,
                               selected_collision_probability=float(result.max_collision_probability),
                               diagnostics=result.gap_diagnostics)
                except TimeoutError as exc:
                    row.update(compute_ms=(time.perf_counter() - tick) * 1000,
                               status='timeout_hold', error=str(exc))
                rows.append(row)
        finally:
            controller.close()
        recent = rows[-args.iterations:]
        print(scene, 'mean_ms', round(np.mean([r['compute_ms'] for r in recent]), 2),
              'ok', sum(r['status'] == 'ok' for r in recent), '/', len(recent), flush=True)
    ms = np.array([r['compute_ms'] for r in rows])
    completed = [r for r in rows if 'diagnostics' in r]
    candidates = [q for r in completed for q in r['diagnostics']['candidates']]
    totals = dict(cycles=len(rows), scenes=len(sources),
        dynamic_scenes=sum(s['predictions'] > 0 for s in sources),
        successful=sum(r['status'] == 'ok' for r in rows),
        timeout_holds=sum(r['status'] == 'timeout_hold' for r in rows),
        over_180_ms=int(np.sum(ms > 180)), over_200_ms=int(np.sum(ms > 200)),
        compute_ms=summary(ms),
        cold_ms=summary([r['compute_ms'] for r in rows if r['cold']]),
        proposal_ms=summary([r['diagnostics']['proposal_ms'] for r in completed]) if completed else None,
        optimized_candidates=sum(q['seed_source'] == 'optimized' for q in candidates),
        reused_candidates=sum(q['seed_source'] == 'reused' for q in candidates),
        fallback_candidates=sum(q['seed_source'] == 'fallback' for q in candidates),
        selected_optimized=sum(q['seed_source'] == 'optimized' and q['reason'] == 'selected' for q in candidates),
        selected_reused=sum(q['seed_source'] == 'reused' and q['reason'] == 'selected' for q in candidates),
        selected_fallback=sum(q['seed_source'] == 'fallback' and q['reason'] == 'selected' for q in candidates),
        wheel_reach_completed=sum(q['wheel_distance_m'] is not None for q in candidates),
        wheel_value_completed=sum(q['wheel_value_cost'] is not None for q in candidates),
        actual_value_completed=sum(q['actual_value_cost'] is not None for q in candidates),
        lidar_shortened_optimized=sum(q['seed_source']=='optimized' and q['lidar_limited'] for q in candidates),
        actual_value_reused_from_wheel=sum(q['actual_value_reused_from_wheel'] for q in candidates))
    report = dict(scope='Production gap implementation on 16 original-bag snapshots (6 with recorded dynamic predictions). '
        'Frozen snapshot repetitions, recorded nominal restored each cycle, persistent caches/workers; '
        'not chronological closed-loop replay. Includes geometry, budgeted seed solving, all 500 MPPI rollouts, '
        'primary/final risk and static checks. Excludes ROS/TF/deskew/prediction alignment/publication and perception load. '
        'Timeouts are counted rather than silently discarded. Does not establish full-stack 5 Hz.',
        environment=dict(python=platform.python_version(), platform=platform.platform(),
                         openblas_threads=os.getenv('OPENBLAS_NUM_THREADS'), omp_threads=os.getenv('OMP_NUM_THREADS')),
        production_budgets=args.optimization_budget_ms is None and args.cycle_timeout_ms is None,
        configuration=asdict(cfg), sources=sources, summary=totals, rows=rows)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'validation.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(totals, indent=2), flush=True)


if __name__ == '__main__':
    main()

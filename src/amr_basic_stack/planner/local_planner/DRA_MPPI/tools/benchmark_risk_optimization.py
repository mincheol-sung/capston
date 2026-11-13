#!/usr/bin/env python3
"""Matched-candidate, short risk integration ablation; excludes ROS scheduling."""
import argparse
import csv
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiController


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profiles', nargs='+', default=['mc_legacy', 'mc_high', 'mc_high_final_disk', 'gaussian_disk'])
    parser.add_argument('--rollouts', nargs='+', type=int, default=[100, 500])
    parser.add_argument('--iterations', type=int, default=8)
    parser.add_argument('--workers', nargs='+', type=int, default=[2])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base = config_from_yaml()
    t = base.dt*np.arange(1, base.horizon_steps+1)
    means = np.zeros((10, 2, len(t), 2))
    means[..., 0] = np.linspace(.8, 3., 10)[:, None, None]+.1*t
    means[:, 0, :, 1] = np.linspace(-1.4, 1.4, 10)[:, None]+.1*t
    means[:, 1, :, 1] = means[:, 0, :, 1]+.2
    path = np.array([[0., 0.], [3., 0.]])
    grid = np.zeros((200, 200)); grid[:2] = 100
    field = DistanceField.from_occupancy(grid, GridMetadata(200, 200, .05, -5., -5., 0.))
    rows = []
    geometry_hash = {}
    for covariance in ('isotropic', 'anisotropic'):
        matrix = np.eye(2)*.04 if covariance == 'isotropic' else np.array([[.05, .025], [.025, .025]])
        pred = PredictionSet(means, np.broadcast_to(matrix, (10, 2, len(t), 2, 2)), t, np.tile([.6, .4], (10, 1)))
        for k in args.rollouts:
            for workers in args.workers:
                for profile in args.profiles:
                    risk = replace(base.dynamic_risk, mode='monte_carlo', final_mode='inherit',
                                   guide_parallel_workers=workers)
                    if profile == 'mc_legacy':
                        risk = replace(risk, mc_sample_density=124., mc_samples_min=512, mc_samples_max=4096,
                                       guide_mc_sample_density=124., guide_mc_samples_min=128, guide_mc_samples_max=1024,
                                       final_mc_sample_density=124., final_mc_samples_min=256, final_mc_samples_max=4096)
                    elif profile == 'gaussian_disk':
                        risk = replace(risk, mode='gaussian_disk')
                    elif profile not in ('mc_high', 'mc_high_final_disk'):
                        raise ValueError(profile)
                    cfg = replace(base, guide_num_rollouts=k, guide_timeout=10., dynamic_risk=risk)
                    c = MppiController(cfg)
                    if profile == 'mc_high_final_disk':
                        from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability
                        c._guide_optimizer._final_risk_estimator = GaussianDiskCollisionProbability(minimum_sigma=risk.minimum_sigma)
                    try:
                        for iteration in range(args.iterations+2):
                            c.reset()
                            start = time.perf_counter()
                            r = c.compute(np.zeros(3), path, np.zeros(3), pred,
                                          visualization_rollouts=0, distance_field=field)
                            elapsed = (time.perf_counter()-start)*1000
                            digest = hashlib.sha256(r.sampled_controls.tobytes()).hexdigest()
                            key = (covariance, k)
                            if key in geometry_hash:
                                assert digest == geometry_hash[key], 'different candidate budget/geometry'
                            geometry_hash[key] = digest
                            if iteration >= 2:
                                rows.append(dict(covariance=covariance, K=k, profile=profile, workers=workers,
                                    iteration=iteration-2, total_ms=elapsed, primary_wall_ms=r.stage_ms['primary_wall'],
                                    final_ms=c.representative_time_ms, primary_mc_samples=c.risk_diagnostics.total_samples,
                                    final_mc_samples=int(np.sum(r.final_mc_sample_counts)),
                                    selected=r.selected_mode, safety_status=r.safety_status, candidate_sha256=digest))
                    finally:
                        c.close()
                    values = [r['total_ms'] for r in rows if r['covariance']==covariance and r['K']==k and r['profile']==profile and r['workers']==workers]
                    print(covariance, k, workers, profile, 'mean/p99', round(float(np.mean(values)), 2), round(float(np.percentile(values, 99)), 2), flush=True)
    with (args.output/'kernel_raw.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (args.output/'manifest.json').write_text(json.dumps(dict(args={**vars(args), 'output':str(args.output)}, scope='kernel only; fixed identical candidates; fresh process recommended for controlled comparisons',
        source_sha256={str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__).parents[1]/'dra_mppi').glob('*.py')}), indent=2))


if __name__ == '__main__':
    main()

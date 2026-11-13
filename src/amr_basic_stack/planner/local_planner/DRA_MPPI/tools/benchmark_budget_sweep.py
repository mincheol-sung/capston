#!/usr/bin/env python3
"""Exploratory kernel sweep; not ROS p99 or a thermal-controlled comparison."""
import argparse
from dataclasses import replace
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
    parser.add_argument('--iterations', type=int, default=5)
    args = parser.parse_args()
    base = config_from_yaml()
    path = np.array([[0., 0.], [3., 0.]])
    h = base.horizon_steps
    t = base.dt*np.arange(1, h+1)
    means = np.zeros((10, 2, h, 2))
    means[..., 0] = np.linspace(.8, 3., 10)[:, None, None]
    means[..., 1] = np.linspace(-1.5, 1.5, 10)[:, None, None]
    pred = PredictionSet(means, np.broadcast_to(np.eye(2)*.04, (10, 2, h, 2, 2)), t,
                         np.tile([.6, .4], (10, 1)))
    field = DistanceField.from_occupancy(np.zeros((200, 200)),
                                        GridMetadata(200, 200, .05, -5., -5., 0.))
    rows = []
    for k in (100, 250, 500):
        for density in (2048., 4096.):
            for backend, workers in (('batch', 1), ('split', 1), ('split', 2), ('split', 5)):
                options = {'guide_parallel_workers': workers}
                for prefix in ('mc', 'guide_mc', 'final_mc'):
                    options.update({prefix+'_sample_density': density,
                                    prefix+'_samples_min': 2048,
                                    prefix+'_samples_max': 131072})
                risk = replace(base.dynamic_risk, **options)
                cfg = replace(base, guide_num_rollouts=k, guide_evaluation=backend,
                              guide_timeout=10., dynamic_risk=risk)
                c, durations = MppiController(cfg), []
                try:
                    for cycle in range(args.iterations+1):
                        c.reset()
                        start = time.perf_counter()
                        c.compute(np.zeros(3), path, np.zeros(3), pred,
                                  visualization_rollouts=0, distance_field=field)
                        if cycle:
                            durations.append((time.perf_counter()-start)*1000)
                    row = dict(K=k, density=density, backend=backend, workers=workers,
                               mean_ms=float(np.mean(durations)), max_ms=max(durations),
                               samples=c.risk_diagnostics.total_samples, durations_ms=durations)
                    print(json.dumps(row), flush=True)
                    rows.append(row)
                finally:
                    c.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()

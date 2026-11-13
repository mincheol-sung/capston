#!/usr/bin/env python3
"""Offline timing of the actual guide strategy, including static collision checks.

Run from any directory with NumPy/SciPy/PyYAML available. No ROS graph or robot
commands are used. Synthetic predictions are deliberately 10 people x 3 modes,
independent of the ROS adapter's moving/stationary mode filtering.
"""

import argparse
from dataclasses import fields
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np
import scipy
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dra_mppi.dynamic_risk import DynamicRiskConfig, PredictionSet
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiConfig, MppiController


def config_from_yaml():
    p = yaml.safe_load((ROOT / 'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    risk = {f.name: p['dynamic_risk.' + f.name] for f in fields(DynamicRiskConfig)
            if 'dynamic_risk.' + f.name in p}
    risk['robot_radius'] = p['robot_radius']
    values = {}
    for f in fields(MppiConfig):
        for prefix in ('mppi.', 'cost.', 'limits.'):
            if prefix + f.name in p:
                values[f.name] = p[prefix + f.name]
    values.update(
        acceleration_limit=p['limits.acceleration'],
        speed_limit_l1=p['limits.wheel_radius'] * p['limits.max_wheel_rad_per_sec'] * p['limits.load_budget'],
        static_cost_enabled=p['static_cost.enabled'],
        static_cost_soft_weight=p['static_cost.soft_weight'],
        static_cost_decay=p['static_cost.decay'],
        static_cost_robot_radius=p['robot_radius'],
        observation_cost_enabled=p.get('observation_cost.enabled', False),
        observation_cost_weight=p.get('observation_cost.weight', 3.0),
        observation_half_fov=np.deg2rad(p.get('observation_cost.half_fov_deg', 35.0)),
        observation_camera_offset=tuple(p.get('observation_cost.camera_offset', [0., 0.])),
        observation_camera_yaw=np.deg2rad(p.get('observation_cost.camera_yaw_deg', 0.0)),
        dynamic_risk=DynamicRiskConfig(**risk), strategy='guide_conditioned')
    return MppiConfig(**values)


def summary(values):
    return {key: float(fn(values)) for key, fn in (
        ('mean', np.mean), ('p50', np.median),
        ('p95', lambda v: np.percentile(v, 95)),
        ('p99', lambda v: np.percentile(v, 99)), ('max', np.max))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=200)
    parser.add_argument('--warmup', type=int, default=20)
    args = parser.parse_args()
    if args.iterations < 1 or args.warmup < 0:
        parser.error('positive iterations and nonnegative warmup required')
    cfg = config_from_yaml()
    c = MppiController(cfg)
    h = cfg.horizon_steps
    times = cfg.dt * np.arange(1, h + 1)
    start = np.column_stack((np.linspace(.65, 2.45, 10), np.linspace(-1.35, 1.38, 10)))
    velocities = np.array([[0., .22], [-.18, .05], [.16, -.12]])
    means = start[:, None, None, :] + velocities[None, :, None, :] * times[None, None, :, None]
    cov = np.broadcast_to(np.eye(2) * .02, (10, 3, h, 2, 2)).copy()
    predictions = PredictionSet(means, cov, times, mode_weights=np.tile([.5, .3, .2], (10, 1)))
    grid = np.zeros((200, 200), dtype=int)
    grid[:3, :3] = 100  # Force a real EDT build while leaving the fan unobstructed.
    tick = time.perf_counter()
    field = DistanceField.from_occupancy(grid, GridMetadata(200, 200, .05, -5., -5., 0.))
    map_build_ms = (time.perf_counter() - tick) * 1000
    path = np.column_stack((np.linspace(0., 2., 21), np.zeros(21)))
    data = []
    for cycle in range(args.warmup + args.iterations):
        tick = time.perf_counter()
        result = c.compute(np.zeros(3), path, np.zeros(3), predictions,
                           visualization_rollouts=32, distance_field=field)
        total = (time.perf_counter() - tick) * 1000
        assert result.guide_active_mask.all()
        assert result.sampled_controls.shape == (cfg.guide_num_rollouts, h, 3)
        if cycle >= args.warmup:
            data.append([total, result.primary_risk_time_ms,
                         result.final_risk_time_ms,
                         c.risk_diagnostics.total_samples,
                         np.sum(result.final_mc_sample_counts)])
    data = np.asarray(data)
    print(json.dumps({
        'environment': {'python': platform.python_version(), 'numpy': np.__version__,
                        'scipy': scipy.__version__, 'platform': platform.platform()},
        'configuration': {'people': 10, 'modes_per_person': 3, 'guides': len(cfg.guide_angles_deg),
                          'rollouts': cfg.guide_num_rollouts, 'horizon': h, 'dt': cfg.dt,
                          'path_points': len(path), 'grid_shape': [200, 200],
                          'warmup': args.warmup, 'iterations': args.iterations},
        'scope': 'Actual controller incl. guides, constraints, swept static checks, '
                 'global-path costs, parallel guide-local MC risk, per-guide update '
                 'and common final recost; '
                 'excludes ROS callbacks/TF/publication and perception inference.',
        'map_build_once_ms': map_build_ms,
        'total_ms': summary(data[:, 0]), 'primary_risk_ms': summary(data[:, 1]),
        'final_risk_ms': summary(data[:, 2]),
        'guide_mc_samples_per_cycle': summary(data[:, 3]),
        'final_mc_samples_per_cycle': summary(data[:, 4]),
    }, indent=2))


if __name__ == '__main__':
    main()

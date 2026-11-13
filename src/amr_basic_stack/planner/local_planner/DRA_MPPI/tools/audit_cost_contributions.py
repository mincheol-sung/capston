#!/usr/bin/env python3
"""Save raw/weighted terms for every guide and rechecked representative.

Also probes synthetic measured-twist perturbations; no estimator calibration,
MPC, ROS timing, or physical safety claim is made by this diagnostic.
"""
import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.common_cost import common_costs
from dra_mppi.cycle_timing import _finite_json
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiController


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cfg = replace(config_from_yaml(), guide_timeout=10.)
    grid = np.zeros((200, 200))
    grid[92:108, 120:123] = 100
    field = DistanceField.from_occupancy(grid, GridMetadata(200, 200, .05, -5., -5., 0.))
    path = np.array([[0., 0.], [3., 0.]])
    t = np.arange(1, cfg.horizon_steps+1)*cfg.dt
    means = np.zeros((2, 2, cfg.horizon_steps, 2))
    means[..., 0] = np.array([1., 2.])[:, None, None]
    means[:, 0, :, 1] = -.8+.2*t
    means[:, 1, :, 1] = .8-.2*t
    pred = PredictionSet(means, np.broadcast_to(np.eye(2)*.04, (2, 2, cfg.horizon_steps, 2, 2)),
                         t, np.tile([.6, .4], (2, 1)))
    rows, details = [], []
    for scenario, state in (('straight', np.zeros(3)),
                            ('offpath_yaw90', np.array([0., .5, np.pi/2])),
                            ('near_goal', np.array([2.8, 0., .3]))):
        c = MppiController(cfg)
        try:
            r = c.compute(state, path, dynamic_predictions=pred, distance_field=field)
            finite = np.isfinite(r.rollout_costs)
            np.testing.assert_allclose(sum(r.cost_terms.weighted.values())[finite],
                                       r.rollout_costs[finite], rtol=1e-12, atol=1e-9)
            for g, name in enumerate(r.candidate_names[:-1]):
                group = r.control_mode_ids == g
                valid = group & finite
                for term in r.cost_terms.raw:
                    row = dict(scenario=scenario, scope='primary_mean', guide=name,
                               term=term, raw=float(np.mean(r.cost_terms.raw[term][group])),
                               weighted=float(np.mean(r.cost_terms.weighted[term][valid]))
                               if term in r.cost_terms.weighted and np.any(valid) else None,
                               count=int(np.sum(group)), valid_count=int(np.sum(valid)))
                    rows.append(row)
            # Non-risk terms are deterministic, while dynamic terms below use
            # exactly the probabilities stored during the actual final recheck.
            _, _, _, _, raw, weighted = common_costs(
                cfg, state, path, r.mode_control_sequences, r.mode_trajectories,
                np.zeros(3), c._projection_progress, return_components=True)
            cp = r.mode_collision_probabilities
            raw.update(dynamic_probability=np.mean(cp, axis=1),
                       dynamic_exceedances=np.sum(cp > cfg.dynamic_risk.hard_threshold, axis=1))
            weighted.update(dynamic_probability=cfg.dynamic_risk.soft_weight*raw['dynamic_probability'],
                            dynamic_exceedances=cfg.dynamic_risk.hard_penalty*raw['dynamic_exceedances'],
                            static=r.mode_static_costs)
            clearance = field.state_clearances(r.mode_trajectories[..., :2])-cfg.static_cost_robot_radius
            raw['static'] = np.mean(np.maximum(0., 1-clearance/cfg.static_cost_decay)**2, axis=1)
            raw['static_collision'] = r.final_static_collisions.astype(float)
            for g, name in enumerate(r.candidate_names):
                evaluated = g == len(r.candidate_names)-1 or r.guide_rejection_reasons[g] != 'all_samples_invalid'
                if not evaluated:
                    continue
                total = sum(values[g] for values in weighted.values())
                if np.isfinite(r.mode_final_costs[g]):
                    np.testing.assert_allclose(total, r.mode_final_costs[g], rtol=1e-12, atol=1e-8)
                for term, value in raw.items():
                    rows.append(dict(scenario=scenario, scope='representative', guide=name,
                                     term=term, raw=float(value[g]),
                                     weighted=float(weighted[term][g]) if term in weighted else None,
                                     count=1, valid_count=int(np.isfinite(r.mode_final_costs[g]))))
            details.append(dict(scenario=scenario, selected=r.selected_mode,
                                safety_status=r.safety_status, final_costs=r.mode_final_costs.tolist(),
                                reasons=r.guide_rejection_reasons))
        finally:
            c.close()
    with (args.output/'cost_components.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(_finite_json(rows))
    (args.output/'cost_components.json').write_text(json.dumps(_finite_json(dict(
        rows=rows, scenarios=details, scope='offline raw/weighted decomposition, not latency')), indent=2))
    rng = np.random.default_rng(20261022)
    sensitivity = []
    cfg = replace(cfg, static_cost_enabled=False, dynamic_risk=replace(cfg.dynamic_risk, mode='off'))
    for sigma in (0., .01, .03, .1):
        changes, rates, selected = [], [], []
        c = MppiController(cfg)
        try:
            reference = c.compute(np.zeros(3), path, np.zeros(3)).control.copy()
            for _ in range(30):
                c.reset()
                current = rng.normal(size=3)*np.array([sigma, sigma, 3*sigma])
                r = c.compute(np.zeros(3), path, current)
                changes.append(float(np.linalg.norm(r.control[:2]-reference[:2])))
                rates.append(float(abs(r.control[2]-reference[2])))
                selected.append(r.selected_mode)
        finally:
            c.close()
        sensitivity.append(dict(translation_sigma_mps=sigma, rotation_sigma_radps=3*sigma,
            trials=30, zero_noise_relative_translation_change_mean_mps=float(np.mean(changes)),
            zero_noise_relative_rotation_change_mean_radps=float(np.mean(rates)),
            selected_counts={name: selected.count(name) for name in sorted(set(selected))}))
    (args.output/'twist_noise_sensitivity.json').write_text(json.dumps(dict(
        scope='synthetic perturbations around zero measured body twist; not measured estimator noise',
        seed=20261022, samples=sensitivity), indent=2))
    print('raw/weighted sums verified; wrote cost_components and synthetic twist sensitivity')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Compare common/per-guide domains on identical actual rollout candidates.

Independent oracle: noncentral chi-square CDF, unequal obstacle radii and
three distinct isotropic covariance widths, two modes per obstacle.
"""
import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np
from scipy.stats import ncx2

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.dynamic_risk import MonteCarloJointCollisionProbability, PredictionSet
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiController


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, default=8)
    parser.add_argument('--seed-start', type=int, default=200)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cfg = replace(config_from_yaml(), static_cost_enabled=False, guide_timeout=10.)
    cfg.dynamic_risk = replace(cfg.dynamic_risk, mode='off')
    rows, work = [], []
    for yaw in (0., -70., 180.):
        state = np.array([0., .2, np.deg2rad(yaw)])
        c = MppiController(cfg)
        r = c.compute(state, np.array([[0., 0.], [3., 0.]]))
        positions = rollout(state, r.sampled_controls, cfg.dt)[..., :2]
        c.close()
        h = cfg.horizon_steps
        t = np.arange(1, h+1)*cfg.dt
        sigma = np.array([.08, .2, .6])
        radii = .2+np.array([.1, .25, .4])
        means = np.zeros((3, 2, h, 2))
        means[..., 0] = np.array([.8, 1.3, 1.8])[:, None, None]
        means[:, 0, :, 1] = -.6+.2*t
        means[:, 1, :, 1] = .7-.2*t
        cov = np.broadcast_to(np.eye(2), (3, 2, h, 2, 2)).copy()*sigma[:, None, None, None, None]**2
        weights = np.tile([.6, .4], (3, 1))
        pred = PredictionSet(means, cov, t, weights, radii-.2)
        survival = np.ones(positions.shape[:2])
        for o in range(3):
            marginal = np.zeros_like(survival)
            for mode in range(2):
                noncentrality = np.sum((positions-means[o, mode])**2, axis=-1)/sigma[o]**2
                marginal += weights[o, mode]*ncx2.cdf((radii[o]/sigma[o])**2, 2, noncentrality)
            survival *= 1-marginal
        reference = 1-survival
        for seed in range(args.seed_start, args.seed_start+args.seeds):
            for backend in ('batch', 'split'):
                groups = [np.arange(len(positions))] if backend == 'batch' else [np.flatnonzero(r.control_mode_ids == g) for g in range(5)]
                estimate = np.zeros_like(reference)
                samples = comparisons = empty = clipped = underresolved = 0
                started = time.perf_counter()
                for g, indices in enumerate(groups):
                    e = MonteCarloJointCollisionProbability(
                        num_samples=4096, adaptive_halton=True, sample_density=4096.,
                        min_samples=4096, max_samples=131072, seed=seed+1009+g,
                        reject_underresolved=True)
                    estimate[indices] = e.evaluate(positions[indices], pred, radii)
                    d = e.last_diagnostics
                    samples += d.total_samples
                    comparisons += d.candidate_sample_comparisons
                    empty += d.empty_circle_count
                    clipped += d.clipped_marginal_count
                    underresolved += d.underresolved_steps
                work.append(dict(yaw=yaw, seed=seed, backend=backend, samples=samples,
                    candidate_sample_comparisons=comparisons, empty=empty, clipped=clipped,
                    underresolved=underresolved, ms=(time.perf_counter()-started)*1000))
                for k, step in np.ndindex(reference.shape):
                    exact, value = float(reference[k, step]), float(estimate[k, step])
                    rows.append(dict(yaw=yaw, seed=seed, backend=backend, candidate=k, step=step,
                        reference=exact, estimate=value, error=value-exact,
                        false_safe=value <= .05 < exact,
                        false_safe_margin=exact >= .06 and value <= .05,
                        false_unsafe=exact <= .05 < value))
    with (args.output/'domains_raw.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = []
    for yaw in (0., -70., 180.):
        for backend in ('batch', 'split'):
            group = [row for row in rows if row['yaw'] == yaw and row['backend'] == backend]
            error = np.abs([row['error'] for row in group])
            report.append(dict(yaw=yaw, backend=backend, count=len(group), mae=float(np.mean(error)),
                absolute_p95=float(np.percentile(error, 95)), absolute_max=float(max(error)),
                unsafe_denominator=sum(r['reference'] > .05 for r in group),
                unsafe_margin_denominator=sum(r['reference'] >= .06 for r in group),
                false_safe=sum(r['false_safe'] for r in group),
                false_safe_margin=sum(r['false_safe_margin'] for r in group),
                false_unsafe=sum(r['false_unsafe'] for r in group)))
    (args.output/'domains_summary.json').write_text(json.dumps(dict(
        scope='fixed actual candidates; independent ncx2 reference; no closed-loop safety claim',
        seed_start=args.seed_start, seeds=args.seeds, summaries=report, work=work), indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

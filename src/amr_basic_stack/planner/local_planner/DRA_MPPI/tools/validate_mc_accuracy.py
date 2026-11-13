#!/usr/bin/env python3
"""Layer A only: online rectangle MC versus independent disk integration."""

import argparse
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.special import roots_legendre
from scipy.stats import ncx2
from scipy.optimize import brentq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dra_mppi.dynamic_risk import PredictionSet, MonteCarloJointCollisionProbability


def disk_reference(means, covariance, weights, radius, order=80):
    """Product Gauss-Legendre radial and periodic angular quadrature."""
    roots, quadrature_weights = roots_legendre(order)
    r = radius*(roots+1)/2
    theta = 2*np.pi*np.arange(4*order)/(4*order)
    points = np.stack((r[:, None]*np.cos(theta), r[:, None]*np.sin(theta)), axis=-1)
    mass = []
    for obstacle in range(len(means)):
        density = np.zeros(points.shape[:-1])
        for mode in range(means.shape[1]):
            sigma = covariance[obstacle, mode]
            delta = points-means[obstacle, mode]
            squared = np.einsum('...i,ij,...j->...', delta, np.linalg.inv(sigma), delta)
            density += weights[obstacle, mode]*np.exp(-squared/2)/(2*np.pi*np.sqrt(np.linalg.det(sigma)))
        mass.append(np.sum(quadrature_weights*r*np.mean(density, axis=1))*radius*np.pi)
    return float(1-np.prod(1-np.array(mass)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, default=12)
    parser.add_argument('--halton', action='store_true')
    parser.add_argument('--seed-start', type=int, default=0)
    parser.add_argument('--spans', type=float, nargs='+', default=[0., .5, 1., 2.])
    parser.add_argument('--counts', type=int, nargs='+', default=[128, 512, 2048])
    parser.add_argument('--densities', type=float, nargs='+')
    parser.add_argument('--min-samples', type=int, default=1)
    parser.add_argument('--max-samples', type=int, default=131072)

    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    radius, epsilon = .45, .05
    cases = []
    for sigma in (.08, .2, .6):
        near = brentq(lambda d: ncx2.cdf((radius/sigma)**2, 2, (d/sigma)**2)-epsilon, 0., 4.)
        for distance in (0., near-.01, near+.01, 2.):
            means = np.array([[[distance, 0.]]])
            cov = np.eye(2)[None, None]*sigma**2
            exact = float(ncx2.cdf((radius/sigma)**2, 2, (distance/sigma)**2))
            cases.append((f'isotropic_s{sigma}_d{distance:.4f}', means, cov,
                          np.ones((1, 1)), exact, 0.))
    for obstacles in (1, 5, 10, 20):
        means = np.zeros((obstacles, 2, 2))
        means[..., 0] = np.linspace(.7, 2.5, obstacles)[:, None]
        means[:, 1, 1] = .4
        cov = np.broadcast_to(np.array([[.09, .02], [.02, .02]]),
                              (obstacles, 2, 2, 2)).copy()
        weights = np.tile([.7, .3], (obstacles, 1))
        low = disk_reference(means, cov, weights, radius, 80)
        high = disk_reference(means, cov, weights, radius, 160)
        cases.append((f'anisotropic_mixture_o{obstacles}', means, cov, weights,
                      high, abs(high-low)))
    rows = []
    for name, means, cov, weights, reference, reference_error in cases:
        pred = PredictionSet(means[:, :, None], cov[:, :, None], np.array([.2]), weights)
        for span in args.spans:
            # Extra candidate centers enlarge B while keeping target C fixed.
            positions = np.array([[[0., 0.]], [[span, span]], [[-span, -span]]])
            for budget in (args.densities or args.counts):
                requested = int(np.ceil(budget*(2*span+2*radius)**2)) if args.densities else int(budget)
                count = int(np.clip(requested, args.min_samples, args.max_samples))
                for seed in range(args.seed_start, args.seed_start+args.seeds):
                    estimator = MonteCarloJointCollisionProbability(
                        num_samples=count, minimum_sigma=.08, seed=seed,
                        adaptive_halton=args.halton, min_samples=count, max_samples=count,
                        sample_density=budget if args.densities else 2048.,
                        reject_underresolved=bool(args.densities))
                    tick = time.perf_counter()
                    probability = estimator.evaluate(positions, pred, radius)[0, 0]
                    elapsed = (time.perf_counter()-tick)*1000
                    diag = estimator.last_diagnostics
                    rows.append({
                        'case': name, 'span': span, 'samples': count, 'budget': budget, 'seed': seed,
                        'reference': reference, 'reference_error_estimate': reference_error,
                        'estimate': float(probability), 'error': float(probability-reference),
                        'false_safe': bool(probability <= epsilon < reference),
                        'false_unsafe': bool(reference <= epsilon < probability),
                        'false_safe_margin': bool(reference >= epsilon+.01 and probability <= epsilon),
                        'ms': elapsed, 'empty_circles': diag.empty_circle_count,
                        'clipped_marginals': diag.clipped_marginal_count,
                    })
    with (args.output/'mc_raw.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summaries = []
    for span in args.spans:
        for budget in (args.densities or args.counts):
            group = [r for r in rows if r['budget'] == budget and r['span'] == span]
            errors = np.array([r['error'] for r in group])
            summaries.append({
                'span': span, 'budget': budget, 'samples': group[0]['samples'], 'evaluations': len(group),
                'absolute_p95': float(np.percentile(abs(errors), 95)),
                'absolute_max': float(np.max(abs(errors))),
                'false_safe_count': sum(r['false_safe'] for r in group),
                'unsafe_denominator': sum(r['reference'] > epsilon for r in group),
                'false_safe_margin_count': sum(r['false_safe_margin'] for r in group),
                'unsafe_margin_denominator': sum(r['reference'] >= epsilon+.01 for r in group),
                'false_unsafe_count': sum(r['false_unsafe'] for r in group),
                'safe_denominator': sum(r['reference'] <= epsilon for r in group),
                'mae': float(np.mean(abs(errors))), 'rmse': float(np.sqrt(np.mean(errors**2))),
                'bias': float(np.mean(errors)),
                'false_safe_rate': sum(r['false_safe'] for r in group)/max(1, sum(r['reference'] > epsilon for r in group)),
                'false_unsafe_rate': sum(r['false_unsafe'] for r in group)/max(1, sum(r['reference'] <= epsilon for r in group)),
                'mean_repeat_std': float(np.mean([
                    np.std([r['estimate'] for r in group if r['case'] == case[0]])
                    for case in cases])),
                'p99_ms': float(np.percentile([r['ms'] for r in group], 99)),
                'empty_circles': sum(r['empty_circles'] for r in group),
                'clipped_marginals': sum(r['clipped_marginals'] for r in group),
            })
    report = {'layer': 'A; synthetic exact distributions; shared-rectangle quadrature',
              'sampling': 'scrambled Halton (fixed budget)' if args.halton else 'random uniform',
              'epsilon_per_step': epsilon, 'seeds': args.seeds, 'seed_start': args.seed_start,
              'budgets_are_density': bool(args.densities), 'acceptance_protocol': '../protocol.md',
              'reference': 'isotropic ncx2 CDF; anisotropic disk quadrature 80 vs 160 radial nodes',
              'max_reference_refinement_difference': max(c[-1] for c in cases),
              'covariance_floor': .08, 'summaries': summaries,
              'calibration': 'not measured; no paired predictions/future ground truth supplied',
              'closed_loop': 'not evaluated by this tool'}
    (args.output/'mc_summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

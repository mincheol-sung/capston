#!/usr/bin/env python3
"""Direct conditional disk integration versus independent polar integration."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability
from validate_mc_accuracy import disk_reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(80117)
    rows = []
    solver = GaussianDiskCollisionProbability()
    radius = .47
    for case in range(64):
        o = (1, 2, 5, 20)[case % 4]
        m = 2
        angle = rng.uniform(-np.pi, np.pi, (o, m))
        rotation = np.stack((np.stack((np.cos(angle), -np.sin(angle)), -1),
                             np.stack((np.sin(angle), np.cos(angle)), -1)), -2)
        sigma = rng.uniform(.08, .6, (o, m, 2))
        if case % 8 == 0:
            sigma[..., 1] = sigma[..., 0]
        covariance = np.einsum('...ik,...k,...jk->...ij', rotation, sigma**2, rotation)
        covariance += np.eye(2)*1e-12
        means = rng.uniform(-.6, .6, (o, m, 2))
        weights = rng.dirichlet([1., 1.], o)
        # Independently locate a joint CP threshold, then probe either side.
        def shifted(distance):
            value = means.copy()
            value[..., 0] += distance
            return value
        distance = brentq(lambda d: disk_reference(shifted(d), covariance, weights, radius, 64)-.05, 0., 5.)
        for delta in (-.03, -.003, -.00003, .00003, .003, .03):
            mu = shifted(distance+delta)
            low = disk_reference(mu, covariance, weights, radius, 80)
            expected = disk_reference(mu, covariance, weights, radius, 160)
            prediction = PredictionSet(mu[:, :, None], covariance[:, :, None], np.array([.2]), weights)
            actual = solver.evaluate(np.zeros((1, 1, 2)), prediction, radius)[0, 0]
            diag = solver.last_diagnostics
            rows.append(dict(case=case, obstacles=o, delta=delta, reference=expected,
                estimate=actual, error=actual-expected, reference_refinement=abs(expected-low),
                false_safe=bool(actual <= .05 < expected), false_unsafe=bool(expected <= .05 < actual),
                unresolved=diag.unresolved_integrals, analytic=diag.analytic_evaluations,
                quadrature_evaluations=diag.quadrature_evaluations))
    with (args.output/'accuracy_raw.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    error = np.array([r['error'] for r in rows])
    summary = dict(evaluations=len(rows), absolute_max=float(np.max(abs(error))),
        absolute_p95=float(np.percentile(abs(error), 95)), mae=float(np.mean(abs(error))),
        reference_refinement_max=max(r['reference_refinement'] for r in rows),
        false_safe=sum(r['false_safe'] for r in rows), unsafe_denominator=sum(r['reference'] > .05 for r in rows),
        false_unsafe=sum(r['false_unsafe'] for r in rows), unresolved=sum(r['unresolved'] for r in rows),
        seed=80117, reference='Independent polar Gauss-Legendre 80/160; regularized covariance',
        limitation='Synthetic distributions only; refinement is not a certified error bound or prediction calibration.')
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

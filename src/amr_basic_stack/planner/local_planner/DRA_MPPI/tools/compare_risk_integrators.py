#!/usr/bin/env python3
"""Reproducible MC 1x/8x vs disk assessment; never changes runtime YAML.

Accuracy uses independent polar quadrature and identical query geometries.
Timing interleaves methods, freezes primary candidates, and excludes ROS.
"""
import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np
import scipy
from scipy.optimize import brentq

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiController
from validate_current_mc import estimator
from validate_mc_accuracy import disk_reference


def write_csv(path, rows):
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def risk_config(base, method):
    # Comparators must not inherit production's hybrid setting or 20-object guard.
    risk = replace(base.dynamic_risk, mode='monte_carlo', final_mode='inherit', max_obstacles=0)
    if method == 'disk':
        return replace(risk, mode='gaussian_disk')
    if method == 'mc1':
        return replace(risk, mc_sample_density=124., mc_samples_min=512, mc_samples_max=4096,
                       guide_mc_sample_density=124., guide_mc_samples_min=128, guide_mc_samples_max=1024,
                       final_mc_sample_density=124., final_mc_samples_min=256, final_mc_samples_max=4096)
    return risk


def regularize(covariance, minimum_sigma):
    values, vectors = np.linalg.eigh(covariance)
    return np.einsum('...ik,...k,...jk->...ij', vectors,
                     np.maximum(values, minimum_sigma**2), vectors)+np.eye(2)*1e-12


def accuracy(base, out):
    rng = np.random.default_rng(20261104)
    radius = base.dynamic_risk.robot_radius+base.dynamic_risk.obstacle_radius
    families = [('iso', .2, .2, 0.), ('moderate', .12, .4, .7),
                ('narrow_rotated', .08, .8, .7), ('narrow_vertical', .08, .8, 1.55),
                ('very_elongated', .08, 3.2, .45), ('below_floor', .02, .8, -.9)]
    rows = []
    for family, sx, sy, angle in families:
        for count in (1, 5, 20):
            rotation = np.array([[np.cos(angle), -np.sin(angle)],
                                 [np.sin(angle), np.cos(angle)]])
            raw_cov = np.broadcast_to(rotation@np.diag([sx*sx, sy*sy])@rotation.T,
                                      (count, 2, 2, 2)).copy()
            cov = regularize(raw_cov, base.dynamic_risk.minimum_sigma)
            offsets = rng.uniform(-.08, .08, (count, 2, 2))
            weights = rng.dirichlet([2., 2.], count)
            direction = np.array([np.cos(angle+.35), np.sin(angle+.35)])
            for target in (.01, .049, .051, .06):
                def shifted(distance):
                    return offsets+distance*direction
                distance = brentq(lambda d: disk_reference(
                    shifted(d), cov, weights, radius, 64)-target, 0., 40.)
                means = shifted(distance)
                low = disk_reference(means, cov, weights, radius, 96)
                reference = disk_reference(means, cov, weights, radius, 192)
                refinement = abs(reference-low)
                if refinement > 1e-9:
                    raise RuntimeError(f'unresolved reference: {family} {count} {target} {refinement}')
                pred = PredictionSet(means[:, :, None], raw_cov[:, :, None],
                                     np.array([base.dt]), weights)
                for span in (0., .25, .5, 1.5):
                    points = np.array([[[0., 0.]], [[span, span]], [[-span, -span]]])
                    methods = [('disk', 'independent', 0)]+[
                        (method, profile, seed) for method in ('mc1', 'mc8')
                        for profile in ('guide', 'final') for seed in (17, 117, 217)]
                    for method, profile, seed in methods:
                        solver = (GaussianDiskCollisionProbability(minimum_sigma=base.dynamic_risk.minimum_sigma)
                                  if method == 'disk' else estimator(risk_config(base, method), profile, seed))
                        actual = float(solver.evaluate(points, pred, radius)[0, 0])
                        diag = solver.last_diagnostics
                        rows.append(dict(family=family, obstacles=count, target=target, span=span,
                            method=method, profile=profile, seed=seed, reference=reference,
                            reference_refinement=refinement, estimate=actual, error=actual-reference,
                            false_safe=actual <= .05 < reference,
                            false_unsafe=reference <= .05 < actual,
                            underresolved=diag.underresolved_steps,
                            unresolved=diag.unresolved_integrals,
                            samples=int(sum(diag.sample_counts)),
                            quadrature=diag.quadrature_evaluations))
            print('accuracy', family, count, flush=True)
            write_csv(out/'accuracy_raw.csv', rows)
    groups = []
    for method, profile in sorted({(r['method'], r['profile']) for r in rows}):
        for resolved_only in (False, True):
            group = [r for r in rows if r['method'] == method and r['profile'] == profile
                     and (not resolved_only or not (r['underresolved'] or r['unresolved']))]
            error = np.abs([r['error'] for r in group])
            groups.append(dict(method=method, profile=profile, resolved_only=resolved_only,
                count=len(group), mae=float(np.mean(error)), p95=float(np.percentile(error, 95)),
                maximum=float(max(error)), false_safe=sum(r['false_safe'] for r in group),
                unsafe_count=sum(r['reference'] > .05 for r in group),
                false_unsafe=sum(r['false_unsafe'] for r in group),
                safe_count=sum(r['reference'] <= .05 for r in group),
                rejected=sum(bool(r['underresolved'] or r['unresolved']) for r in group)))
    result = dict(groups=groups, reference_refinement_max=max(r['reference_refinement'] for r in rows),
                  scenarios=72, unique_geometries=288,
                  warning='Repeated seeds/spans are not independent scenes. Synthetic threshold-focused suite; no certified bound.')
    (out/'accuracy_summary.json').write_text(json.dumps(result, indent=2))


def timing(base, out, iterations, hybrid=False):
    horizon = base.horizon_steps
    times = base.dt*np.arange(1, horizon+1)
    path = np.array([[0., 0.], [3., 0.]])
    grid = np.zeros((200, 200)); grid[:2] = 100
    field = DistanceField.from_occupancy(grid, GridMetadata(200, 200, .05, -5., -5., 0.))
    theta = .7
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    families = [('iso', np.eye(2)*.04), ('moderate', np.array([[.05, .025], [.025, .025]])),
                ('narrow', rot@np.diag([.08**2, .8**2])@rot.T)]
    rows = []
    for family, matrix in families:
        if hybrid and family == 'iso':
            continue
        for count in (5, 20, 40):
            if hybrid and count == 5:
                continue
            # Fixed occupied area: more objects increase scene density, not distance.
            means = np.zeros((count, 2, horizon, 2))
            means[..., 0] = np.linspace(.8, 3., count)[:, None, None]+.1*times
            means[:, 0, :, 1] = np.linspace(-1.4, 1.4, count)[:, None]+.1*times
            means[:, 1, :, 1] = means[:, 0, :, 1]+.2
            pred = PredictionSet(means, np.broadcast_to(matrix, (count, 2, horizon, 2, 2)),
                                 times, np.tile([.6, .4], (count, 1)))
            methods = ('mc8', 'mc8_final_disk') if hybrid else ('mc1', 'mc8', 'disk')
            controllers = {method: MppiController(replace(base, guide_timeout=10.,
                dynamic_risk=risk_config(base, method))) for method in methods}
            if hybrid:
                controllers['mc8_final_disk']._guide_optimizer._final_risk_estimator = (
                    GaussianDiskCollisionProbability(minimum_sigma=base.dynamic_risk.minimum_sigma))
            candidate_hash = None
            try:
                for iteration in range(iterations+3):
                    # Rotate order to reduce fixed-order drift; no concurrent benchmarks.
                    order = np.roll(np.array(methods), iteration % len(methods))
                    for method in order:
                        c = controllers[method]
                        c.reset()
                        started = time.perf_counter()
                        result = c.compute(np.zeros(3), path, np.zeros(3), pred,
                                           visualization_rollouts=0, distance_field=field)
                        elapsed = (time.perf_counter()-started)*1000
                        digest = hashlib.sha256(result.sampled_controls.tobytes()).hexdigest()
                        if candidate_hash is None:
                            candidate_hash = digest
                        assert candidate_hash == digest, 'Primary candidates must match'
                        if iteration >= 3:
                            d = result.final_risk_diagnostics
                            rows.append(dict(family=family, obstacles=count, method=method,
                                iteration=iteration-3, total_ms=elapsed,
                                primary_wall_ms=result.stage_ms['primary_wall'],
                                final_ms=c.representative_time_ms,
                                primary_samples=c.risk_diagnostics.total_samples,
                                primary_unresolved=c.risk_diagnostics.unresolved_integrals,
                                primary_underresolved=c.risk_diagnostics.underresolved_steps,
                                final_unresolved=d.unresolved_integrals,
                                final_underresolved=d.underresolved_steps,
                                selected=result.selected_mode, max_cp=result.max_collision_probability,
                                status=result.safety_status, candidate_sha256=digest))
            finally:
                for controller in controllers.values():
                    controller.close()
            write_csv(out/'timing_raw.csv', rows)
            print('timing', family, count, {method: round(float(np.mean([
                r['total_ms'] for r in rows if r['family'] == family and r['obstacles'] == count
                and r['method'] == method])), 2) for method in methods}, flush=True)
    summary = []
    for family, count, method in sorted({(r['family'], r['obstacles'], r['method']) for r in rows}):
        group = [r for r in rows if (r['family'], r['obstacles'], r['method']) == (family, count, method)]
        values = [r['total_ms'] for r in group]
        summary.append(dict(family=family, obstacles=count, method=method, n=len(group),
            mean_ms=float(np.mean(values)), p95_ms=float(np.percentile(values,95)), max_ms=float(max(values)),
            over180=sum(v > 180 for v in values), over200=sum(v > 200 for v in values),
            primary_mean_ms=float(np.mean([r['primary_wall_ms'] for r in group])),
            final_mean_ms=float(np.mean([r['final_ms'] for r in group])),
            unresolved=sum(r['primary_unresolved']+r['final_unresolved'] for r in group),
            underresolved=sum(r['primary_underresolved']+r['final_underresolved'] for r in group)))
    (out/'timing_summary.json').write_text(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--suite', choices=('timing', 'accuracy', 'hybrid'), required=True)
    parser.add_argument('--iterations', type=int, default=30)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base = config_from_yaml()
    package = Path(__file__).resolve().parents[1]
    files = list((package/'dra_mppi').glob('*.py'))+[Path(__file__), package/'config/dra_mppi.yaml',
        package/'tools/validate_mc_accuracy.py', package/'tools/validate_current_mc.py',
        package/'tools/benchmark_guide_conditioned.py']
    manifest = dict(config=asdict(base), python=platform.python_version(), numpy=np.__version__,
        scipy=scipy.__version__, platform=platform.platform(), args={**vars(args),'output':str(args.output)},
        source_sha256={str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        scope='Offline synthetic assessment; no ROS timing, actuator execution, or model calibration; YAML unchanged.')
    (args.output/f'{args.suite}_manifest.json').write_text(json.dumps(manifest, indent=2))
    if args.suite in ('timing', 'hybrid'):
        timing(base, args.output, args.iterations, hybrid=args.suite == 'hybrid')
    else:
        accuracy(base, args.output)


if __name__ == '__main__':
    main()

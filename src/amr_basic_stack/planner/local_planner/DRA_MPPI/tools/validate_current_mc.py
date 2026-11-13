#!/usr/bin/env python3
"""Audit current YAML MC budgets against independent Gaussian disk probabilities."""
import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import brentq
from scipy.stats import ncx2

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.dynamic_risk import MonteCarloJointCollisionProbability, PredictionSet
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiController
from validate_mc_accuracy import disk_reference


def estimator(risk, profile, seed):
    prefix = {'guide': 'guide_mc', 'final': 'final_mc'}[profile]
    return MonteCarloJointCollisionProbability(
        num_samples=getattr(risk, prefix+'_samples_min'),
        minimum_sigma=risk.minimum_sigma, seed=seed,
        adaptive_halton=risk.adaptive_halton_mc,
        sample_density=getattr(risk, prefix+'_sample_density'),
        min_samples=getattr(risk, prefix+'_samples_min'),
        max_samples=getattr(risk, prefix+'_samples_max'),
        reject_underresolved=risk.mc_reject_underresolved)


def stats(rows):
    error = np.array([r['error'] for r in rows])
    unsafe = sum(r['reference'] > .05 for r in rows)
    margin = sum(r['reference'] >= .06 for r in rows)
    safe = len(rows)-unsafe
    fs = sum(r['estimate'] <= .05 < r['reference'] for r in rows)
    fm = sum(r['estimate'] <= .05 and r['reference'] >= .06 for r in rows)
    fu = sum(r['reference'] <= .05 < r['estimate'] for r in rows)
    return dict(count=len(rows), mae=float(np.mean(abs(error))),
        absolute_p95=float(np.percentile(abs(error), 95)), absolute_max=float(np.max(abs(error))),
        bias=float(np.mean(error)), false_safe=fs, unsafe_denominator=unsafe,
        false_safe_rate=fs/max(1, unsafe), false_safe_margin=fm, unsafe_margin_denominator=margin,
        false_unsafe=fu, safe_denominator=safe, false_unsafe_rate=fu/max(1, safe),
        minimum_estimate=float(min(r['estimate'] for r in rows)),
        maximum_estimate=float(max(r['estimate'] for r in rows)))


def write_csv(path, rows):
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def point_suite(cfg, repeats):
    """Known target CP while other candidate centers vary the shared rectangle."""
    rng = np.random.default_rng(20261022)
    radius = cfg.dynamic_risk.robot_radius+cfg.dynamic_risk.obstacle_radius
    scenes = []
    for sigma in (.08, .2, .6):
        for target in (.01, .049, .051, .1, .5, .9):
            # Wide distributions cannot attain arbitrary high circle mass.
            if target >= ncx2.cdf(radius**2/(sigma*sigma+1e-12), 2, 0.):
                continue
            distance = brentq(lambda d: ncx2.cdf(radius**2/(sigma*sigma+1e-12), 2, d*d/(sigma*sigma+1e-12))-target, 0, 5)
            means = np.array([[[distance, 0.]]])
            cov = np.eye(2)[None, None]*sigma*sigma
            scenes.append((f'iso_sigma{sigma}_p{target}', 'isotropic', means, cov, np.ones((1, 1)), target, 0.))
    for o in (1, 5, 10, 20):
        for kind in ('narrow_rotated', 'wide_rotated', 'multimodal'):
            angle = .7
            rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            variances = [.08**2, .3**2] if kind != 'wide_rotated' else [.2**2, .6**2]
            cov = np.broadcast_to(rot@np.diag(variances)@rot.T, (o, 2, 2, 2)).copy()
            regularized = cov+np.eye(2)*1e-12
            means = np.zeros((o, 2, 2))
            means[..., 0] = np.linspace(0., .8, o)[:, None]
            means[:, 0, 1] = rng.uniform(-.25, .25, o)
            means[:, 1, 1] = means[:, 0, 1]+(.8 if kind == 'multimodal' else .2)
            weights = np.tile([.6, .4], (o, 1))
            def shifted(d):
                value = means.copy(); value[..., 0] += d; return value
            near = brentq(lambda d: disk_reference(shifted(d), regularized, weights, radius, 64)-.05, 0., 5.)
            for delta in (-.01, .01):
                mu = shifted(near+delta)
                low = disk_reference(mu, regularized, weights, radius, 80)
                high = disk_reference(mu, regularized, weights, radius, 160)
                scenes.append((f'{kind}_o{o}_d{delta}', kind, mu, cov, weights, high, abs(high-low)))
    rows = []
    for name, family, means, covariance, weights, exact, refinement in scenes:
        prediction = PredictionSet(means[:, :, None], covariance[:, :, None], np.array([cfg.dt]), weights)
        for span in (0., .25, .5, 1., 2.):
            points = np.array([[[0., 0.]], [[span, span]], [[-span, -span]]])
            for profile in ('guide', 'final'):
                for repeat in range(repeats):
                    seed = cfg.dynamic_risk.seed+(1009 if profile == 'guide' else 2003)+repeat
                    e = estimator(cfg.dynamic_risk, profile, seed)
                    estimate = float(e.evaluate(points, prediction, radius)[0, 0]); diag = e.last_diagnostics
                    rows.append(dict(scenario=name, family=family, profile=profile, span=span,
                        obstacles=len(means), seed=seed, reference=float(exact), estimate=estimate,
                        error=estimate-exact, reference_refinement=refinement,
                        area=float(diag.region_areas[0]), samples=int(diag.sample_counts[0]),
                        underresolved=diag.underresolved_steps, empty_circle=diag.empty_circle_count,
                        clipped=diag.clipped_marginal_count))
    return rows


def candidate_suite(cfg, repeats):
    """Actual K500 guide fan and its six representatives, analytic oracle."""
    rows, work = [], []
    h = cfg.horizon_steps
    t = cfg.dt*np.arange(1, h+1)
    radius = cfg.dynamic_risk.robot_radius+cfg.dynamic_risk.obstacle_radius
    for yaw in (0., -70., 180.):
        c = MppiController(replace(cfg, static_cost_enabled=False, guide_timeout=10.,
                                  dynamic_risk=replace(cfg.dynamic_risk, mode='off')))
        state = np.array([0., .2, np.deg2rad(yaw)])
        try:
            result = c.compute(state, np.array([[0., 0.], [3., 0.]]))
            points = rollout(state, result.sampled_controls, cfg.dt)[..., :2]
            representatives = result.mode_trajectories[..., :2]
        finally:
            c.close()
        for scenario, o in [('crossing1', 1), ('headon5', 5), ('parallel10', 10), ('multimodal20', 20)]:
            sigma = np.resize(np.array([.08, .2, .6]), o)
            means = np.zeros((o, 2, h, 2))
            means[..., 0] = np.linspace(.7, 2.6, o)[:, None, None]
            means[:, 0, :, 1] = -.6+.25*t
            means[:, 1, :, 1] = .6-.25*t
            if scenario == 'headon5':
                means[..., 0] -= .25*t
            if scenario == 'parallel10':
                means[..., 0] += .3*t
            if scenario == 'multimodal20':
                means[:, 1, :, 0] -= .3*t
            cov = np.broadcast_to(np.eye(2), (o, 2, h, 2, 2))*sigma[:, None, None, None, None]**2
            weights = np.tile([.6, .4], (o, 1))
            pred = PredictionSet(means, cov, t, weights)
            for profile, positions in [('guide', points), ('final', representatives)]:
                exact = np.ones(positions.shape[:2])
                for obstacle in range(o):
                    marginal = np.zeros_like(exact)
                    variance = sigma[obstacle]**2+1e-12
                    for mode in range(2):
                        marginal += weights[obstacle, mode]*ncx2.cdf(radius**2/variance, 2,
                            np.sum((positions-means[obstacle, mode])**2, axis=-1)/variance)
                    exact *= 1-marginal
                exact = 1-exact
                for repeat in range(repeats):
                    estimate = np.zeros_like(exact)
                    groups = ([np.flatnonzero(result.control_mode_ids == g) for g in range(5)]
                              if profile == 'guide' else [np.arange(len(positions))])
                    for g, indices in enumerate(groups):
                        seed = cfg.dynamic_risk.seed+(1009+g if profile == 'guide' else 2003)+repeat
                        e = estimator(cfg.dynamic_risk, profile, seed)
                        estimate[indices] = e.evaluate(positions[indices], pred, radius)
                        d = e.last_diagnostics
                        work.append(dict(yaw=yaw, scenario=scenario, profile=profile, repeat=repeat,
                            group=g, samples_min=int(min(d.sample_counts)), samples_max=int(max(d.sample_counts)),
                            area_min=float(min(d.region_areas)), area_max=float(max(d.region_areas)),
                            underresolved=d.underresolved_steps, empty_circle=d.empty_circle_count,
                            clipped=d.clipped_marginal_count))
                    for k, step in np.ndindex(exact.shape):
                        value, reference = float(estimate[k, step]), float(exact[k, step])
                        rows.append(dict(scenario=scenario, yaw=yaw, profile=profile, repeat=repeat,
                            candidate=k, step=step, reference=reference, estimate=value, error=value-reference))
    return rows, work


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--point-repeats', type=int, default=16)
    parser.add_argument('--candidate-repeats', type=int, default=8)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    cfg = config_from_yaml(); start = time.perf_counter()
    assert cfg.dynamic_risk.mode == 'monte_carlo'
    point = point_suite(cfg, args.point_repeats)
    print('point suite complete', len(point), 'seconds', round(time.perf_counter()-start, 2), flush=True)
    actual, work = candidate_suite(cfg, args.candidate_repeats)
    write_csv(args.output/'point_raw.csv', point); write_csv(args.output/'candidate_raw.csv', actual)
    write_csv(args.output/'candidate_work.csv', work)
    summaries = []
    for profile in ('guide', 'final'):
        for span in (0., .25, .5, 1., 2.):
            group = [r for r in point if r['profile']==profile and r['span']==span]
            summaries.append(dict(suite='known_points', profile=profile, span=span,
                samples_min=min(r['samples'] for r in group), samples_max=max(r['samples'] for r in group),
                underresolved_evaluations=sum(r['underresolved'] for r in group), **stats(group)))
        for scenario in ('crossing1', 'headon5', 'parallel10', 'multimodal20'):
            group = [r for r in actual if r['profile']==profile and r['scenario']==scenario]
            summaries.append(dict(suite='actual_candidates', profile=profile, scenario=scenario, **stats(group)))
    report = dict(config=asdict(cfg), config_sha256=hashlib.sha256((Path(__file__).parents[1]/'config/dra_mppi.yaml').read_bytes()).hexdigest(),
        point_repeats=args.point_repeats, candidate_repeats=args.candidate_repeats,
        known_point_cases=len(set(r['scenario'] for r in point)), total_evaluations=len(point)+len(actual),
        reference_refinement_max=max(r['reference_refinement'] for r in point),
        seconds=time.perf_counter()-start, summaries=summaries,
        overall={profile: stats([r for r in actual if r['profile']==profile]) for profile in ('guide', 'final')},
        limitation='Synthetic Gaussian probabilities, not empirical real-world collision frequencies. Repeats share scenarios and are not independent environment trials. Candidate final representatives generated with risk off to hold geometry fixed across risk scenes; final safety selection not evaluated.')
    (args.output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(dict(seconds=report['seconds'], summaries=summaries), indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Time collision probability only, using the original audit's reference code."""
import os
for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import time
import numpy as np
from scipy.stats import ncx2
from benchmark_guide_conditioned import config_from_yaml, ROOT
from validate_current_mc import estimator
from validate_mc_accuracy import disk_reference
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.mppi_controller import MppiController
from dra_mppi.mecanum_model import rollout


def measure(fn, repeats, warmup=1):
    for _ in range(warmup):
        fn()
    raw = []
    for _ in range(repeats):
        start = time.perf_counter()
        value = fn()
        raw.append(1000*(time.perf_counter()-start))
    return value, dict(raw_ms=raw, mean_ms=float(np.mean(raw)), min_ms=min(raw), max_ms=max(raw))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--polar-repeats', type=int, default=1)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cfg = config_from_yaml()
    cfg = replace(cfg, horizon_steps=10, guide_num_rollouts=500,
                  static_cost_enabled=False, guide_timeout=10.,
                  dynamic_risk=replace(cfg.dynamic_risk, mode='off'))
    state = np.array([0., .2, 0.])
    controller = MppiController(cfg)
    try:
        result = controller.compute(state, np.array([[0., 0.], [3., 0.]]))
        guide = rollout(state, result.sampled_controls, cfg.dt)[..., :2]
        final = result.mode_trajectories[..., :2]
    finally:
        controller.close()
    t = cfg.dt*np.arange(1, 11)
    o, m = 20, 2
    # Same multimodal20 scene as the original K500 accuracy suite, yaw=0.
    means = np.zeros((o, m, 10, 2))
    means[..., 0] = np.linspace(.7, 2.6, o)[:, None, None]
    means[:, 0, :, 1] = -.6+.25*t
    means[:, 1, :, 1] = .6-.25*t
    means[:, 1, :, 0] -= .3*t
    sigma = np.resize(np.array([.08, .2, .6]), o)
    iso = np.broadcast_to(np.eye(2), (o, m, 10, 2, 2))*sigma[:, None, None, None, None]**2
    # Same rotated covariance as original controller timing experiment.
    aniso = np.broadcast_to(np.array([[.05, .025], [.025, .025]]), iso.shape).copy()
    weights = np.tile([.6, .4], (o, 1))
    radius = cfg.dynamic_risk.robot_radius+cfg.dynamic_risk.obstacle_radius
    report = dict(cpu=platform.processor(), platform=platform.platform(),
        numpy=np.__version__, threads=1, horizon=10, obstacles=o, modes=m,
        radius=radius, dt=cfg.dt, guide_shape=list(guide.shape), final_shape=list(final.shape),
        yaml_sha256=hashlib.sha256((ROOT/'config/dra_mppi.yaml').read_bytes()).hexdigest(),
        notes=['Probability evaluation only; excludes controller, rollout generation, and estimator construction.',
               'MC uses five control groups, sequentially, no sample reuse; same inputs for every method.',
               'Polar calls original disk_reference(order=160) for every candidate-step; no extrapolation.',
               'Polar mean with one repeat is a single observed duration, not a stable latency statistic.'], rows=[])
    np.savez_compressed(args.output/'inputs.npz', guide=guide, final=final,
        group_ids=result.control_mode_ids, means=means, isotropic_covariance=iso,
        anisotropic_covariance=aniso, weights=weights, times=t, radius=radius)

    def save(row):
        report['rows'].append(row)
        (args.output/'timing.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(row), flush=True)

    for family, covariance in [('isotropic', iso), ('anisotropic', aniso)]:
        prediction = PredictionSet(means, covariance, t, weights)
        for profile, positions in [('guide', guide), ('final', final)]:
            groups = ([np.flatnonzero(result.control_mode_ids == g) for g in range(5)]
                      if profile == 'guide' else [np.arange(len(positions))])
            for mult in (1, 2, 4, 8, 16, 32):
                changes = {}
                for prefix, baseline in [('guide_mc', (124., 128, 1024)), ('final_mc', (124., 256, 4096))]:
                    for suffix, value in zip(('sample_density', 'samples_min', 'samples_max'), baseline):
                        changes[prefix+'_'+suffix] = value*mult
                risk = replace(cfg.dynamic_risk, adaptive_halton_mc=True, **changes)
                solvers = [estimator(risk, profile, risk.seed+(1009+g if profile == 'guide' else 2003)) for g in range(len(groups))]
                def mc():
                    out = np.zeros(positions.shape[:2])
                    for indices, solver in zip(groups, solvers):
                        out[indices] = solver.evaluate(positions[indices], prediction, radius)
                    return out
                value, timing = measure(mc, 10, 2)
                pdf = sum(s.last_diagnostics.total_samples*o*m for s in solvers)
                comparisons = sum(s.last_diagnostics.candidate_sample_comparisons for s in solvers)
                save(dict(family=family, profile=profile, method='mc', multiplier=mult,
                          pdf_evaluations=pdf, candidate_sample_comparisons=comparisons, **timing))
            if family == 'isotropic':
                def cdf():
                    exact = np.ones(positions.shape[:2])
                    for obstacle in range(o):
                        marginal = np.zeros_like(exact)
                        variance = sigma[obstacle]**2+1e-12
                        for mode in range(m):
                            marginal += weights[obstacle, mode]*ncx2.cdf(radius**2/variance, 2,
                                np.sum((positions-means[obstacle, mode])**2, axis=-1)/variance)
                        exact *= 1-marginal
                    return 1-exact
                exact, timing = measure(cdf, 10, 2)
                save(dict(family=family, profile=profile, method='ncx2_cdf',
                          cdf_evaluations=len(positions)*10*o*m, **timing))
            else:
                def polar():
                    exact = np.zeros(positions.shape[:2])
                    for k in range(len(positions)):
                        for step in range(10):
                            exact[k, step] = disk_reference(means[:, :, step]-positions[k, step],
                                covariance[:, :, step]+np.eye(2)*1e-12, weights, radius, 160)
                        if profile == 'guide' and (k+1) % 25 == 0:
                            print(f'polar guide progress {k+1}/{len(positions)}', flush=True)
                    return exact
                # No full-size warmup: original reference at this size is expensive.
                disk_reference(means[:, :, 0], covariance[:, :, 0]+np.eye(2)*1e-12, weights, radius, 160)
                exact, timing = measure(polar, args.polar_repeats, 0)
                save(dict(family=family, profile=profile, method='polar_order160',
                          pdf_evaluations=len(positions)*10*o*m*160*640, **timing))
            np.save(args.output/f'{family}_{profile}_reference.npy', exact)


if __name__ == '__main__':
    main()

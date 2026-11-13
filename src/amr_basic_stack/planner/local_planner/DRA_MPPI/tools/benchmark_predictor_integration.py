#!/usr/bin/env python3
"""Compare shared MC and conditional disk integration on IMM-generated scenes."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml
from benchmark_guide_conditioned import ROOT, config_from_yaml

STACK = ROOT.parents[2]
sys.path.insert(0, str(STACK/'perception/object_tracker'))
sys.path.insert(0, str(STACK/'perception/motion_predictor'))
from motion_predictor.core.predictor import ImmMotionPredictor
from object_tracker.core.config import TrackerConfig
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability
from dra_mppi.mppi_controller import MppiController
from dra_mppi.mecanum_model import rollout
from validate_current_mc import estimator
from validate_mc_accuracy import disk_reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cfg = config_from_yaml()
    pred_path = STACK/'perception/motion_predictor/config/predictor.yaml'
    params = yaml.safe_load(pred_path.read_text())['motion_predictor_node']['ros__parameters']
    model_cfg = TrackerConfig()
    for key, value in params.items():
        if hasattr(model_cfg, key):
            setattr(model_cfg, key, value)
    predictor = ImmMotionPredictor(model_cfg, prediction_dt=.2, horizon=2.,
        propagate_mode_probabilities=params['propagate_mode_probabilities'])
    cfg = replace(cfg, horizon_steps=10, guide_num_rollouts=500,
        guide_timeout=10., static_cost_enabled=False,
        dynamic_risk=replace(cfg.dynamic_risk, mode='off'))
    controller = MppiController(cfg)
    state = np.array([0., .2, 0.])
    try:
        result = controller.compute(state, np.array([[0., 0.], [3., 0.]]))
        guide = rollout(state, result.sampled_controls, cfg.dt)[..., :2]
        final = result.mode_trajectories[..., :2]
    finally:
        controller.close()
    radius = cfg.dynamic_risk.robot_radius+cfg.dynamic_risk.obstacle_radius
    report = dict(description='Synthetic IMM posteriors propagated by production predictor; not bag replay or measured pedestrian ground truth.',
        threads=1, guide_groups='five sequential groups, no cache reuse',
        repeats=args.repeats, warmup=1, horizon=10, dt=.2, modes=3, radius=radius,
        config_sha256=hashlib.sha256((ROOT/'config/dra_mppi.yaml').read_bytes()).hexdigest(),
        predictor_yaml_sha256=hashlib.sha256(pred_path.read_bytes()).hexdigest(),
        scenes=[], timings=[], accuracy=[], validation=[])

    def save():
        (args.output/'results.json').write_text(json.dumps(report, indent=2))

    def timed(fn):
        fn()
        values, raw = [], []
        for _ in range(args.repeats):
            start = time.perf_counter()
            values.append(fn())
            raw.append((time.perf_counter()-start)*1000)
        return values, dict(raw_ms=raw, mean_ms=float(np.mean(raw)), median_ms=float(np.median(raw)))

    for scene in ('crossing', 'headon', 'turning'):
        for count in (5, 20):
            means, covs, weights = [], [], []
            for j in range(count):
                x = .65+2.*j/max(1,count-1)
                y = -.65+.10*(j % 3)
                speed = .6+.15*(j % 3)
                angle = {'crossing': np.pi/2, 'headon': np.pi, 'turning': 2.4}[scene]
                angle += .08*(j % 3-1)
                vx, vy = speed*np.cos(angle), speed*np.sin(angle)
                omega = (-1 if j % 2 else 1)*(.7 if scene == 'turning' else .3)
                states = np.array([[x,y,vx,vy,0.], [x,y,vx,vy,omega], [x,y,0.,0.,0.]])
                # SPD posterior with a rotated, unequal position/velocity uncertainty.
                c,s = np.cos(angle), np.sin(angle)
                rot = np.array([[c,-s],[s,c]])
                p = np.zeros((5,5))
                p[:2,:2] = rot@np.diag([.12**2,.05**2])@rot.T
                p[2:4,2:4] = rot@np.diag([.20**2,.08**2])@rot.T
                p[4,4] = .25**2
                probs = np.array([.65,.25,.10]) if scene != 'turning' else np.array([.25,.65,.10])
                times, mu, cov, w = predictor.predict(states, np.repeat(p[None],3,axis=0), probs,
                    model_cfg.mean_sojourn_cv, model_cfg.mean_sojourn_ct, model_cfg.mean_sojourn_static)
                means.append(mu); covs.append(cov); weights.append(w)
            pred = PredictionSet(np.array(means), np.array(covs), times, np.array(weights))
            eig = np.linalg.eigvalsh(pred.covariances)
            ratio = np.sqrt(eig[...,1]/eig[...,0])
            report['scenes'].append(dict(scene=scene, obstacles=count,
                raw_axis_ratio_min=float(ratio.min()), raw_axis_ratio_max=float(ratio.max())))
            np.savez_compressed(args.output/f'{scene}_{count}_inputs.npz', guide=guide, final=final,
                group_ids=result.control_mode_ids, means=pred.means, covariances=pred.covariances,
                weights=pred.mode_weights, times=times, radius=radius)
            for profile, positions in [('guide',guide), ('final',final)]:
                direct = GaussianDiskCollisionProbability(minimum_sigma=cfg.dynamic_risk.minimum_sigma)
                references, timing = timed(lambda: direct.evaluate(positions,pred,radius))
                reference = references[-1]
                dd = direct.last_diagnostics
                assert np.all(np.isfinite(reference)) and dd.unresolved_integrals == 0
                report['timings'].append(dict(scene=scene,obstacles=count,profile=profile,method='direct',
                    quadrature_nodes=dd.quadrature_evaluations, cdf_evaluations=dd.analytic_evaluations,
                    unresolved=dd.unresolved_integrals, **timing))
                # Independent high-order polar checks: threshold, broad probability range,
                # and random pairs, selected once independently of MC errors.
                flat=reference.ravel(); rng=np.random.default_rng(20261027)
                indices=list(np.argsort(abs(flat-.05))[:4])
                indices+=list(np.argsort(flat)[np.linspace(0,len(flat)-1,4,dtype=int)])
                indices+=list(rng.choice(len(flat),4,replace=False))
                for index in sorted(set(int(i) for i in indices)):
                    k,step=np.unravel_index(index,reference.shape)
                    covariance=pred.covariances[:,:,step]
                    ev,vec=np.linalg.eigh(covariance)
                    regularized=np.einsum('...ik,...k,...jk->...ij',vec,np.maximum(ev,cfg.dynamic_risk.minimum_sigma**2),vec)+np.eye(2)*1e-12
                    shifted=pred.means[:,:,step]-positions[k,step]
                    low=disk_reference(shifted,regularized,pred.mode_weights[:,:,step],radius,80)
                    high=disk_reference(shifted,regularized,pred.mode_weights[:,:,step],radius,160)
                    report['validation'].append(dict(scene=scene,obstacles=count,profile=profile,
                        candidate=int(k),step=int(step),polar=high,direct=float(reference[k,step]),
                        error=float(reference[k,step]-high),polar_refinement=abs(high-low)))
                groups=([np.flatnonzero(result.control_mode_ids==g) for g in range(5)]
                    if profile=='guide' else [np.arange(len(positions))])
                for mult in (1,2,4,8,16,32):
                    changes={}
                    for prefix,base in [('guide_mc',(124.,128,1024)),('final_mc',(124.,256,4096))]:
                        for suffix,v in zip(('sample_density','samples_min','samples_max'),base):
                            changes[prefix+'_'+suffix]=mult*v
                    risk=replace(cfg.dynamic_risk,adaptive_halton_mc=True,**changes)
                    solvers=[estimator(risk,profile,risk.seed+1009+g) for g in range(len(groups))]
                    def mc():
                        out=np.zeros(positions.shape[:2])
                        for indices,solver in zip(groups,solvers):
                            out[indices]=solver.evaluate(positions[indices],pred,radius)
                        return out
                    estimates,timing=timed(mc)
                    diagnostics=[s.last_diagnostics for s in solvers]
                    report['timings'].append(dict(scene=scene,obstacles=count,profile=profile,method='mc',multiplier=mult,
                        pdf_evaluations=sum(d.total_samples*count*3 for d in diagnostics),
                        candidate_comparisons=sum(d.candidate_sample_comparisons for d in diagnostics),
                        underresolved=sum(d.underresolved_steps for d in diagnostics),**timing))
                    errors=np.concatenate([(v-reference).ravel() for v in estimates])
                    unsafe=np.broadcast_to(reference>.05,(len(estimates),*reference.shape))
                    false_safe=np.stack([v<=.05 for v in estimates]) & unsafe
                    report['accuracy'].append(dict(scene=scene,obstacles=count,profile=profile,multiplier=mult,
                        mae=float(np.mean(abs(errors))),p95=float(np.percentile(abs(errors),95)),
                        max_error=float(np.max(abs(errors))), false_safe=int(false_safe.sum()),
                        unsafe_count=int(unsafe.sum())))
                save()
                print(f'{scene} o={count} {profile}: {reference.shape} probabilities; complete',flush=True)
    print('all scenes complete',flush=True)


if __name__=='__main__':
    main()

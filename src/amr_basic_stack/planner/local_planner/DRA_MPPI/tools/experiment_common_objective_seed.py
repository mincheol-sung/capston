"""Ten real snapshots: constrained common-objective optimization, no obstacle terms.

Multi-start SLSQP is an offline local optimization experiment, not a certified
global optimizer. Obstacle checks are post-hoc and remain enabled in final MPPI.
"""
from dataclasses import replace
from itertools import product
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.optimize import minimize, Bounds, LinearConstraint

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_multiscene_2026-11-04'
OUT=BASE/'common_objective_seed_10_scenes'
sys.path.insert(0,str(BASE/'source_snapshot/tools'))
import experiment_gap_multiscene as ex
from experiment_gap_diversity import options, hard_depth

SCENES=['b0_001','b0_029','b0_033','b0_034','b1_004','b1_037','b1_088','b2_083','b2_084','b2_133']


def load(path):return [json.loads(x) for x in path.read_text().splitlines()]


def objective(c,p,controls,components=False):
    paths=ex.rollout(np.zeros(3),controls,c.config.dt)
    terms=ex.common_costs(c.config,np.zeros(3),p['path'],controls,paths,p['current'],return_components=components)
    return (sum(terms[:4]),terms[-1]) if components else sum(terms)


def constraints(c,current):
    cfg=c.config;n=cfg.horizon_steps;dim=n*3
    # Match the controller's treatment of an initially infeasible measurement.
    initial=c._project_wheel_limit(np.clip(current,cfg.control_min,cfg.control_max)[None])[0]
    rates=np.eye(dim)
    for t in range(1,n):rates[t*3:t*3+3,(t-1)*3:t*3]=-np.eye(3)
    delta=np.tile(np.asarray(cfg.acceleration_limit)*cfg.dt,n)
    center=np.zeros(dim);center[:3]=initial
    signs=np.array(list(product([-1.,1.],repeat=3)))*[1.,1.,cfg.yaw_lever]
    wheel=np.zeros((8*n,dim))
    for t in range(n):wheel[t*8:t*8+8,t*3:t*3+3]=signs
    A=np.vstack((rates,wheel));lower=np.r_[center-delta,np.full(8*n,-np.inf)]
    upper=np.r_[center+delta,np.full(8*n,cfg.speed_limit_l1)]
    bounds=Bounds(np.tile(cfg.control_min,n),np.tile(cfg.control_max,n))
    return bounds,LinearConstraint(A,lower,upper)


def optimize(c,p,starts):
    tick=time.perf_counter();bounds,linear=constraints(c,p['current']);n=c.config.horizon_steps;dim=3*n
    eye=np.eye(dim);h=1e-5;unique=[];runs=[];best=None;fun_batches=0
    for name,control in starts:
        if not any(np.allclose(control,x[1],atol=1e-9,rtol=0) for x in unique):unique.append((name,control))
    def fg(x):
        nonlocal fun_batches
        points=np.concatenate((x[None],x[None]+h*eye,x[None]-h*eye))
        costs=objective(c,p,points.reshape(-1,n,3));fun_batches+=1
        return float(costs[0]),(costs[1:1+dim]-costs[1+dim:])/(2*h)
    for name,control in unique:
        started=time.perf_counter();initial=float(objective(c,p,control[None])[0])
        opt=minimize(fg,control.ravel(),jac=True,method='SLSQP',bounds=bounds,constraints=[linear],
                     options=dict(maxiter=250,ftol=1e-8,disp=False))
        raw=opt.x.reshape(n,3)
        projected=c._constrain_controls(raw[None],p['current'])[0]
        score=float(objective(c,p,projected[None])[0])
        # A failed local solve is never allowed to degrade its own initial seed.
        kept=projected if score<=initial else control.copy()
        kept_score=min(score,initial)
        ax=linear.A@opt.x
        violation=float(max(np.max(bounds.lb-opt.x),np.max(opt.x-bounds.ub),
                            np.max(linear.lb-ax),np.max(ax-linear.ub),0.))
        run=dict(start=name,initial_cost=initial,cost=kept_score,solver_cost=float(opt.fun),
                 success=bool(opt.success),message=str(opt.message),iterations=int(opt.nit),
                 constraint_violation=violation,projection_norm=float(np.linalg.norm(projected-raw)),
                 wall_ms=(time.perf_counter()-started)*1000,kept_optimized=bool(score<=initial))
        runs.append(run)
        if best is None or kept_score<best[0]:best=(kept_score,kept,name,bool(opt.success and score<=initial))
    return best,runs,dict(multistart_ms=(time.perf_counter()-tick)*1000,objective_batches=fun_batches,
                          scalar_equivalent_evaluations=fun_batches*(1+2*dim))


def main():
    OUT.mkdir(exist_ok=True);(OUT/'scenes').mkdir(exist_ok=True)
    audits={a['record_ns']:a for a in load(BASE/'scene_audit.jsonl') if a['accepted'] and a['scene'] in SCENES}
    prior={(r['scene'],r['seed']):r for r in load(BASE/'diversity_and_empty_gap/trials.jsonl') if r['method']=='all_options_hard_fallback'}
    ex.write_json(OUT/'protocol.json',dict(scenes=SCENES,
        selection='Ten diagnostic snapshots chosen before optimization: normal travel, stalled nominal, empty-gap margin/short/narrow cases, and initial-footprint-invalid case. Not representative success statistics.',
        objective='Exactly frozen common_costs: weighted squared path/progress/terminal/heading/speed/delta x,y,omega residuals. No dynamic risk, static soft cost, or static invalidation in seed optimizer.',
        constraints='N=10,dt=.2, control box, per-axis acceleration from the same projected initial input, all eight signed wheel L1 facets per time step; original nonlinear mecanum rollout.',
        solver='SLSQP, central finite-difference batched gradient h=1e-5, ftol1e-8,maxiter250; starts nominal, braking, path guide, and up to3 selected gap seeds. Best feasible found; no global-optimality guarantee.',
        postcheck='After seed generation evaluate static swept collision and recorded dynamic MC risk. Feasible controls are not equivalent to obstacle-safe paths.',
        downstream='Baseline is all_options_hard_fallback. common_as_nominal: replace nominal only if the seed passes static check; unchanged guide counts. common_extra: append if static-clear, not nominal-duplicate (RMS<.22 AND tail<.30) and not gap-duplicate (RMS<.25); keep at most2 existing lowest seed-cost guides; total extra cap3. Dynamic risk always checked by final MPPI. Two noise seeds7/23, K500.',
        timing='Offline multi-start reference solve; not budgeted for 5Hz. Single-start path-guide solve is timed separately, without claiming same quality.',
        sources='Frozen previous experiment planner and YAML. common_cost.py SHA256 matches current implementation.',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    cfg=ex.config_from_yaml();done=0
    with (OUT/'scene_results.jsonl').open('w') as sf,(OUT/'trials.jsonl').open('w') as tf:
        for bag in ex.BAGS:
            print('READ',bag,flush=True);data=ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
            for snap in data['snapshots']:
                if snap['record_ns'] not in audits:continue
                audit=audits[snap['record_ns']];scene=audit['scene']
                p=ex.prepare_snapshot(data,snap);ex.input_check(data,snap,p)
                np.testing.assert_allclose(p['d_free'],np.load(BASE/'scenes'/f'{scene}.npz')['d_free'],atol=1e-10)
                c=ex.MppiController(replace(cfg,strategy='standard'));c.gap_path=p['path']
                predictions=ex.recorded_predictions(p,c)
                nominal=c._constrain_controls(p['nominal'][None],p['current'])[0]
                nompath=ex.rollout(np.zeros(3),nominal,.2)
                pp=p
                if not ex.definitions(p,'valley'):pp={**p,'d_free':hard_depth(p)[0]}
                pool,_=options(pp,c,nompath)
                _,chosen,_=ex.select(pp,pool,'cost',audit['nominal_collision'],audit['nominal_cost'] or np.inf,True)
                pathguide=c._make_path_guide(np.zeros(3),p['path'],p['current'])
                starts=[('nominal',nominal),('braking',c._make_stopping_control(p['current'])),('path_guide',pathguide)]
                starts += [(g['id'],g['controls']) for g in chosen]
                best,runs,timing=optimize(c,p,starts);cost,controls,start,success=best
                trajectory=ex.rollout(np.zeros(3),controls,.2)
                delta=np.linalg.norm(trajectory[:,:2]-nompath[:,:2],axis=1)
                rms=float(np.sqrt(np.mean(delta**2)));tail=float(np.mean(delta[-3:]))
                collision=bool(p['field'].trajectory_collisions(trajectory[None,:,:2],np.zeros(2),ex.RADIUS)[0])
                gap_rms=[float(np.sqrt(np.mean(np.sum((trajectory[:,:2]-g['trajectory'][:,:2])**2,axis=1)))) for g in chosen]
                risk=c._evaluate_dynamic_risk(trajectory[None],predictions)
                maxprob=float(risk.maximum_probability[0]);dynamic_pass=maxprob<=cfg.dynamic_risk.hard_threshold
                nominal_cost=float(objective(c,p,nominal[None])[0]);guide_cost=float(objective(c,p,pathguide[None])[0])
                _,components=objective(c,p,controls[None],components=True)
                reason='static_collision' if collision else 'duplicate_nominal' if rms<.22 and tail<.30 else 'duplicate_gap' if gap_rms and min(gap_rms)<.25 else 'eligible'
                r=dict(scene=scene,bag=bag,offset=snap['elapsed'],prediction_count=audit['prediction_count'],
                       common_cost=cost,nominal_common_cost=nominal_cost,path_guide_common_cost=guide_cost,
                       best_start=start,best_solver_success=success,solver_runs=runs,**timing,
                       static_collision=collision,max_probability=maxprob,dynamic_pass=dynamic_pass,
                       rms_nominal=rms,tail_nominal=tail,gap_rms=gap_rms,extra_seed_reason=reason,
                       progress_m=float(ex.path_progress(p,trajectory[None])[0]),
                       components={k:float(v[0]) for k,v in components.items()},baseline_guides=len(chosen))
                bounds,linear=constraints(c,p['current']);u=controls.ravel()
                assert np.all(u>=bounds.lb-1e-8) and np.all(u<=bounds.ub+1e-8)
                assert np.all(linear.A@u>=linear.lb-1e-8) and np.all(linear.A@u<=linear.ub+1e-8)
                assert cost<=min(float(objective(c,p,x[None])[0]) for _,x in starts)+1e-8
                arrays=dict(common_controls=controls,common_trajectory=trajectory,nominal_controls=nominal,
                            nominal_trajectory=nompath,pathguide_trajectory=ex.rollout(np.zeros(3),pathguide,.2),
                            gap_seeds=np.array([g['trajectory'] for g in chosen]),path=p['path'])
                if predictions is not None and not predictions.empty:
                    arrays.update(prediction_means=predictions.means,prediction_covariances=predictions.covariances)
                for method in ['common_as_nominal','common_extra']:
                    use=(not collision) if method=='common_as_nominal' else reason=='eligible'
                    newnom=controls if use and method=='common_as_nominal' else nominal
                    newguides=chosen
                    if use and method=='common_extra':
                        newguides=chosen[:2]+[dict(id='zz_common_optimized',controls=controls,trajectory=trajectory)]
                    for seed in ex.SEEDS:
                        if use:
                            result,paths=ex.run_optimizer(p,cfg,newnom,newguides,predictions,method,seed)
                        else:
                            result=prior[scene,seed].copy()
                            saved=np.load(BASE/'diversity_and_empty_gap/scenes'/f'{scene}.npz')
                            paths=dict(trajectory=saved['all_options_hard_fallback_trajectory'])
                        tf.write(json.dumps(ex.serial({**result,'scene':scene,'method':method,'seed':seed,
                                                       'seed_used':use,'baseline_reused':not use}))+'\n');tf.flush()
                        if seed==7:arrays[method+'_trajectory']=paths['trajectory']
                sf.write(json.dumps(ex.serial(r))+'\n');sf.flush()
                np.savez_compressed(OUT/'scenes'/f'{scene}.npz',**arrays)
                c.close();done+=1
                print('SCENE',done,scene,'J',round(cost,5),'nomJ',round(nominal_cost,5),'static',collision,
                      'extra',reason,'time_ms',round(timing['multistart_ms']),flush=True)
    assert done==10
    ex.write_json(OUT/'run_summary.json',dict(scenes=done,downstream_comparisons=40,
                                            limitations='Multi-start local best; diagnostic selection; offline snapshot only.'))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Offline reward sweep; augmented seed objective, unchanged allocation score.

Compare production two-solve with one-solve J_base-lambda*gap_displacement.
No production source or YAML modification. Recorded real bag snapshots only.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

from benchmark_gap_latency import inputs, write
from experiment_gap_reach_overhead import Arm, run
from experiment_gap_endpoint_policy import stats
from validate_gap_scheduler import config

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'docs/gap_value_optimization_2026-11-05'
OUT=ROOT/'docs/gap_distance_reward_2026-11-09'


def reward_solver(arm, weight):
    m=arm.opt
    class RewardProblem(m.ValueProblem):
        def objective(self,x):
            cost,grad,parts=super().objective(x)
            states,jac=self.state(x)
            reward=-weight*float(states[-1,:2]@self.e)
            return cost+reward,grad-weight*(self.e@jac[-1,:2]),dict(parts,distance_reward=reward)
    arm.reward_problem=RewardProblem

    def solve(cfg,current,path,angle,depth,start,common_reference=None,
              deadline=float('inf'),diagnostics=None):
        info={} if diagnostics is None else diagnostics
        info.update(m.gap_value_diagnostics())
        info.update(wheel_reach_status='not_computed',optimization_stage='actual_value',
                    actual_value_status='running',distance_reward_weight=weight)
        problem=RewardProblem(cfg,current,path,angle,float(depth),common_reference)
        try:
            actual=problem.solve(start,deadline)
        except m.OptimizationDeadline:
            info['actual_value_status']='budget_exhausted'
            raise
        valid=(actual['success'] and actual['violation']<=cfg.gap.optimization_constraint_tolerance
               and np.isfinite(actual['cost']) and np.all(np.isfinite(actual['controls'])))
        info.update(actual_value_status='converged' if valid else 'not_converged',
                    actual_value_ms=actual['wall_ms'],actual_value_iterations=actual['iterations'])
        if not valid:return None
        rho=float(actual['trajectory'][-1,:2]@problem.e)
        assert cfg.gap.min_displacement_m-1e-5<=rho<=depth+1e-5
        assert abs(actual['trajectory'][-1,:2]@problem.perp)<=1e-5
        common=m.reference(path,cfg) if common_reference is None else common_reference
        base,parts=m.score_controls(cfg,current,common,problem.e,actual['controls'])
        np.testing.assert_allclose(actual['cost'],base-weight*rho,rtol=1e-8,atol=1e-8)
        info.update(actual_target_distance_m=rho,actual_value_cost=base,actual_value_components=parts,
                    optimized_reward_objective=actual['cost'],optimized_base_cost=base,
                    optimized_displacement_m=rho,optimization_stage='complete')
        # Allocation must never see the augmented value. The runtime _assess
        # also recomputes the original score after control projection.
        actual.update(cost=base,components=parts)
        return actual
    return solve


def check_derivatives(arms,cfg):
    rng=np.random.default_rng(20261109);checks=0
    for arm in arms.values():
        if not hasattr(arm,'reward_problem'):continue
        for angle in (-2.8,-.3,1.4):
            p=arm.reward_problem(cfg,np.array([.12,-.02,.1]),np.array([[0.,0.],[2.,.5]]),angle,2.)
            x=rng.normal(0,.15,3*cfg.horizon_steps)
            value,grad,_=p.objective(x)
            eps=1e-6;eye=np.eye(len(x))*eps
            numerical=np.array([(p.objective(x+e)[0]-p.objective(x-e)[0])/(2*eps) for e in eye])
            np.testing.assert_allclose(grad,numerical,rtol=2e-5,atol=2e-6)
            base=m_base=arm.opt.ValueProblem.objective(p,x)[0]
            rho=float(p.state(x)[0][-1,:2]@p.e)
            np.testing.assert_allclose(value,base-arm.weight*rho,rtol=1e-10,atol=1e-10)
            checks+=1
    return checks


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUT)
    parser.add_argument('--iterations',type=int,default=8)
    parser.add_argument('--scene')
    parser.add_argument('--weights',type=float,nargs='+',default=[0.,4.,8.,16.,32.,48.])
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    snapshot=out/'source_snapshot/dra_mppi'
    if snapshot.exists():raise RuntimeError('Use fresh output directory')
    shutil.copytree(ROOT/'dra_mppi',snapshot,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copy2(ROOT/'config/dra_mppi.yaml',out/'configuration.yaml')
    shutil.copy2(__file__,out/'experiment.py')
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')}
    arms={'fixed':Arm('fixed_reward_reference',snapshot)}
    arms['fixed'].label='fixed'
    arms['fixed'].gap.solve_gap_values=arms['fixed'].opt.solve_gap_values
    for weight in args.weights:
        name=f'reward_{weight:g}'
        arm=Arm(name,snapshot);arm.weight=weight
        arm.gap.solve_gap_values=reward_solver(arm,weight);arms[name]=arm
    cfg=config(arms['fixed'],out/'configuration.yaml')
    derivative_checks=check_derivatives(arms,cfg)
    sources=json.loads((SOURCE/'raw_sources.json').read_text())
    if args.scene:sources=[s for s in sources if s['scene']==args.scene]
    generous=replace(cfg,gap=replace(cfg.gap,optimization_timeout=2.,timeout=5.,cache_max_age=3600.))
    rows=[]

    def one(name,c,data,field,scans,predictions,mode,scene,trial):
        arm=arms[name]
        row,r=run(arm,c,data,field,scans,predictions,mode,scene,trial)
        row.update(sampling_seed=c.config.seed,reward_weight=None if name=='fixed' else arm.weight)
        if r is not None:
            d=r.gap_diagnostics
            row.update(final_displacement_m=float(np.linalg.norm(r.trajectory[-1,:2])),
                candidate_names=r.candidate_names,representative_costs=r.mode_final_costs,
                nominal_cost=d['nominal_cost'],proposal_budget_ms=d['proposal_budget_ms'],
                mppi_reserved_ms=d['mppi_reserved_ms'],post_proposal_ms=d['post_proposal_ms'],
                trajectory=r.trajectory,controls=r.control_sequence)
            if trial==0:
                row.update(anchors=r.guide_anchors,regions=r.gap_regions,
                           representative_trajectories=r.mode_trajectories)
            if mode=='uncensored':assert not d['proposal_budget_exhausted']
            if name!='fixed':
                assert row['reach_calls']==0
                for q in d['candidates']:
                    assert q['wheel_distance_m'] is None
                    if q['seed_source']=='optimized':
                        assert 'distance_reward' not in q['cost_components']
                        np.testing.assert_allclose(q['cost'],sum(q['cost_components'].values()),rtol=1e-10)
                        assert q['cost']==q['actual_value_cost']
        return row

    data,field,scans,predictions=inputs(sources[0]['scene'])
    for name,arm in arms.items():
        c=arm.package.MppiController(generous)
        try:one(name,c,data,field,scans,predictions,'warmup',sources[0]['scene'],-1)
        finally:c.close()
    for scene_index,source in enumerate(sources):
        scene=source['scene'];data,field,scans,predictions=inputs(scene)
        for trial in range(args.iterations):
            settings=replace(generous,seed=cfg.seed+trial*31)
            order=np.roll(list(arms),-(trial+scene_index)%len(arms))
            if trial%2:order=order[::-1]
            for name in order:
                c=arms[name].package.MppiController(settings)
                try:rows.append(one(name,c,data,field,scans,predictions,'uncensored',scene,trial))
                finally:c.close()
        # Consecutive runs per arm prevent artificial 7-arm interleaving from
        # expiring the production 0.6s cache. Rotate arm blocks across scenes.
        order=np.roll(list(arms),-scene_index%len(arms))
        if scene_index%2:order=order[::-1]
        for name in order:
            c=arms[name].package.MppiController(cfg)
            try:
                for trial in range(args.iterations):
                    rows.append(one(name,c,data,field,scans,predictions,'production',scene,trial))
            finally:c.close()
        write(out/'partial.json',dict(rows=rows))
        print(scene,'done',flush=True)
    summaries={mode:{name:stats([r for r in rows if r['mode']==mode and r['label']==name])
        for name in arms} for mode in ('uncensored','production')}
    for name,sha in hashes.items():
        assert hashlib.sha256((ROOT/'dra_mppi'/name).read_bytes()).hexdigest()==sha
    assert (ROOT/'config/dra_mppi.yaml').read_bytes()==(out/'configuration.yaml').read_bytes()
    write(out/'benchmark.json',dict(configuration=asdict(cfg),sources=sources,source_hashes=hashes,
        weights=args.weights,iterations=args.iterations,rows=rows,summary=summaries,
        derivative_checks=derivative_checks,
        notes='Saved actual bag snapshots; no fresh bag or ROS playback. Same 500 sample evaluator and allocation. '
        'Reward applied only during seed optimization, never allocation or final MPPI score. '
        'Both methods allow rotation. No production modification. '
        'Uncensored: fresh controller, paired eight sampling seeds, 2s proposal/5s cycle. '
        'Production: current budget, consecutive eight frozen cycles per arm; rotated blocks per scene; '
        'recorded nominal restored each call, cache/schedule persistent. '
        'Parameter exploration on the same 16 scenes, not held-out validation.'))
    print(json.dumps(summaries,indent=2),flush=True)


if __name__=='__main__':main()

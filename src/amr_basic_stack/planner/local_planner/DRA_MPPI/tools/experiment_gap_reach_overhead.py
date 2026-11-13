#!/usr/bin/env python3
"""Paired offline bag-snapshot experiment; production sources are not edited.

Compare current wheel-reach/wheel-value/actual-value, direct reach/actual-value,
and an oracle that supplies identical precomputed reach solutions. The oracle
isolates reach cost; it is NOT a practical current-state lookup-table planner.
"""
import argparse
from dataclasses import asdict, fields, replace
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import time
import numpy as np
import yaml

from benchmark_guide_conditioned import config_from_yaml
from benchmark_gap_latency import inputs, serial, write

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'docs/gap_value_optimization_2026-11-05'
OUT=ROOT/'docs/gap_reach_overhead_2026-11-09'


def summary(values):
    a=np.asarray(values,float)
    if not len(a):return None
    return dict(n=len(a),mean=float(a.mean()),p50=float(np.median(a)),
                p95=float(np.percentile(a,95)),max=float(a.max()))


def load_package(name,path):
    spec=importlib.util.spec_from_file_location(name,path/'__init__.py',submodule_search_locations=[str(path)])
    package=importlib.util.module_from_spec(spec);sys.modules[name]=package;spec.loader.exec_module(package)
    return package,importlib.import_module(name+'.gap_conditioned'),importlib.import_module(name+'.gap_optimization')


class Arm:
    def __init__(self,label,path):
        self.label=label
        self.package,self.gap,self.opt=load_package('reach_experiment_'+label,path)
        self.events=[];self.reach_cache={};self.oracle_cache=None
        original=self.opt.ValueProblem.solve
        arm=self
        def measured(problem,start,deadline=float('inf')):
            reach=isinstance(problem,arm.opt.ReachProblem)
            stage=('reach' if reach else 'wheel_value' if arm.label=='current' and
                   problem.depth==arm.opt.wheel_distance_bound(problem.cfg) else 'actual_value')
            key=(problem.current.tobytes(),problem.e.tobytes(),np.asarray(start).tobytes()) if reach else None
            tick=time.perf_counter();result=None;status='exception'
            try:
                if reach and arm.oracle_cache is not None:
                    if key not in arm.oracle_cache:raise AssertionError('missing exact oracle reach solution')
                    result=arm.oracle_cache[key]
                else:result=original(problem,start,deadline)
                status='converged' if result['success'] and result['violation']<=problem.cfg.gap.optimization_constraint_tolerance else 'not_converged'
                if reach:arm.reach_cache[key]=result
                return result
            except arm.opt.OptimizationDeadline:
                status='deadline';raise
            finally:
                arm.events.append(dict(stage=stage,ms=1000*(time.perf_counter()-tick),status=status,
                    oracle=bool(reach and arm.oracle_cache is not None),
                    iterations=None if result is None else result['iterations']))
        self.opt.ValueProblem.solve=measured
        if label!='current':self.gap.solve_gap_values=self.direct

    def direct(self,cfg,current,path,angle,depth,start,common_reference=None,
               deadline=float('inf'),diagnostics=None):
        """Same reach solve; directly minimize the actual endpoint objective."""
        m=self.opt;info={} if diagnostics is None else diagnostics
        info.update(m.gap_value_diagnostics())
        common=m.reference(path,cfg) if common_reference is None else common_reference
        bound=m.wheel_distance_bound(cfg)
        def stage(problem,u,name):
            info['optimization_stage']=name;info[name+'_status']='running'
            try:r=problem.solve(u,deadline)
            except m.OptimizationDeadline:
                info[name+'_status']='budget_exhausted';raise
            info[name+'_iterations']=r['iterations'];info[name+'_ms']=r['wall_ms']
            valid=(r['success'] and r['violation']<=cfg.gap.optimization_constraint_tolerance and
                   np.isfinite(r['cost']) and np.all(np.isfinite(r['controls'])))
            info[name+'_status']='converged' if valid else 'not_converged'
            return r if valid else None
        problem=m.ReachProblem(cfg,current,path,angle,bound,common)
        reach=stage(problem,start,'wheel_reach')
        if reach is None:return None
        distance=float(reach['trajectory'][-1,:2]@problem.e);target=min(distance,float(depth))
        info.update(wheel_distance_m=distance,actual_target_distance_m=target,lidar_limited=bool(depth<distance))
        if target<cfg.gap.min_displacement_m-cfg.gap.optimization_constraint_tolerance:
            info['optimization_stage']='actual_value';info['actual_value_status']='insufficient_distance';return None
        problem=m.ValueProblem(cfg,current,path,angle,float(depth),common)
        problem.target_distance=target
        actual=stage(problem,reach['controls'],'actual_value')
        if actual is None:return None
        info.update(actual_value_cost=actual['cost'],actual_value_components=actual['components'],
                    optimization_stage='complete')
        return actual


def run(arm,c,data,field,scans,predictions,mode,scene,trial,record=True):
    c._guide_optimizer._warm_nominal=data['nominal'].copy();arm.events=[]
    tick=time.perf_counter()
    try:
        r=c.compute(np.zeros(3),data['path'].copy(),data['current'].copy(),predictions,
                    visualization_rollouts=32,distance_field=field,gap_scans=scans)
        elapsed=1000*(time.perf_counter()-tick);d=r.gap_diagnostics
        assert r.sampled_controls.shape==(500,c.config.horizon_steps,3)
        assert sum(r.guide_rollout_counts)==500
        chosen=r.candidate_names.index(r.selected_mode)
        if r.safety_status=='ok':
            assert not r.final_static_collisions[chosen]
            assert r.max_collision_probability<=c.config.dynamic_risk.hard_threshold
        candidates=d['candidates']
        for q in candidates:
            if q['wheel_distance_m'] is not None:
                assert q['actual_target_distance_m']==min(q['wheel_distance_m'],q['depth_m'])
            if q['seed_source']=='optimized':
                assert q['cost']==q['actual_value_cost']
                assert q['allocation_cost_source']=='optimized_actual'
                if arm.label!='current':assert q['wheel_value_cost'] is None
        row=dict(mode=mode,scene=scene,trial=trial,label=arm.label,status=r.safety_status,
                 total_ms=elapsed,geometry_ms=d['geometry_ms'],proposal_ms=d['proposal_ms'],
                 primary_wall_ms=r.stage_ms['primary_wall'],cost=r.cost,selected_mode=r.selected_mode,
                 counts=r.guide_rollout_counts,probability=r.max_collision_probability,
                 candidates=candidates,selected=d['selected'],
                 optimized=sum(q['seed_source']=='optimized' for q in candidates),
                 selected_optimized=sum(q['seed_source']=='optimized' and q['reason']=='selected' for q in candidates),
                 proposal_budget_exhausted=d['proposal_budget_exhausted'])
    except TimeoutError as exc:
        r=None;row=dict(mode=mode,scene=scene,trial=trial,label=arm.label,status='timeout_hold',
                       total_ms=1000*(time.perf_counter()-tick),error=str(exc))
    row['stages']=arm.events.copy()
    for stage in ('reach','wheel_value','actual_value'):
        row[stage+'_ms']=sum(e['ms'] for e in arm.events if e['stage']==stage)
        row[stage+'_calls']=sum(e['stage']==stage for e in arm.events)
    return row,r


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations',type=int,default=8)
    parser.add_argument('--output',type=Path,default=OUT)
    parser.add_argument('--scene',help='Optional diagnostic single-scene run')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    snapshot=out/'source_snapshot/dra_mppi'
    if snapshot.exists():raise RuntimeError('Use a fresh output directory to preserve experiment provenance')
    shutil.copytree(ROOT/'dra_mppi',snapshot,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copy2(ROOT/'config/dra_mppi.yaml',out/'configuration.yaml')
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'dra_mppi').glob('*.py')}
    # Experiment-only compatibility: no wheel-only diagnostic is computed in
    # direct/oracle arms; keep None rather than fabricating a comparison value.
    experimental=out/'experiment_package/dra_mppi';shutil.copytree(snapshot,experimental)
    path=experimental/'gap_conditioned.py';text=path.read_text()
    old="candidate['value_difference']=assessed['cost']-candidate['wheel_value_cost']"
    assert text.count(old)==1
    path.write_text(text.replace(old,"candidate['value_difference']=(assessed['cost']-candidate['wheel_value_cost'] if candidate['wheel_value_cost'] is not None else None)"))
    arms={name:Arm(name,snapshot if name=='current' else experimental) for name in ('current','direct','oracle')}
    p=yaml.safe_load((out/'configuration.yaml').read_text())['dra_mppi_node']['ros__parameters']
    from dra_mppi.gap_geometry import GapConfig
    gap=GapConfig(**{f.name:p['gap.'+f.name] for f in fields(GapConfig) if 'gap.'+f.name in p})
    cfg=replace(config_from_yaml(),strategy='gap',gap=gap)
    sources=json.loads((SOURCE/'raw_sources.json').read_text());rows=[];checks=[];reach_pairs=[]
    if args.scene:sources=[s for s in sources if s['scene']==args.scene]
    for source in sources:
        scene=source['scene'];data,field,scans,predictions=inputs(scene)
        # Untimed cache expiry must not make the faster oracle use a different
        # fallback in the equal-work comparison. Production retains 0.6 seconds.
        generous=replace(cfg,gap=replace(gap,optimization_timeout=2.,timeout=5.,cache_max_age=3600.))
        # Unmeasured prepass prepares exact reach solutions for this frozen
        # scene and warms numerical imports. Its cost is excluded from oracle.
        arm=arms['direct'];arm.reach_cache={}
        c=arm.package.MppiController(generous)
        try:run(arm,c,data,field,scans,predictions,'precompute',scene,-1)
        finally:c.close()
        arms['oracle'].oracle_cache=dict(arm.reach_cache)
        for mode in ('uncensored','production'):
            settings=generous if mode=='uncensored' else cfg
            controllers={name:arm.package.MppiController(settings) for name,arm in arms.items()}
            try:
                for trial in range(args.iterations):
                    results={};paired={}
                    order=np.roll(np.array(list(arms)),-(trial%3))
                    if trial%2:order=order[::-1]
                    for name in order:
                        row,r=run(arms[name],controllers[name],data,field,scans,predictions,mode,scene,trial)
                        rows.append(row);paired[name]=row;results[name]=r
                    if mode=='uncensored':
                        assert all(r is not None for r in results.values())
                        a,b=results['direct'],results['oracle']
                        if not np.array_equal(a.sampled_controls,b.sampled_controls):
                            write(out/'equivalence_failure.json',paired)
                        np.testing.assert_array_equal(a.sampled_controls,b.sampled_controls)
                        np.testing.assert_array_equal(a.rollout_costs,b.rollout_costs)
                        np.testing.assert_array_equal(a.control_sequence,b.control_sequence)
                        assert a.selected_mode==b.selected_mode
                        checks.append(dict(scene=scene,trial=trial,exact_direct_oracle=True))
                        before={q['id']:q for q in paired['current']['candidates']}
                        for q in paired['direct']['candidates']:
                            a=before[q['id']]
                            assert a['wheel_distance_m']==q['wheel_distance_m']
                            reach_pairs.append(dict(scene=scene,trial=trial,id=q['id'],
                                distance_m=q['wheel_distance_m'],identical=True))
            finally:
                for c in controllers.values():c.close()
        write(out/'partial.json',dict(rows=rows,oracle_checks=checks))
        print(scene,'done',flush=True)
    report=dict(scope=__doc__,configuration=asdict(cfg),iterations=args.iterations,sources=sources,
                source_hashes=hashes,oracle_checks=checks,reach_pairs=reach_pairs,rows=rows)
    stats={}
    for mode in ('uncensored','production'):
        stats[mode]={}
        for label in arms:
            group=[r for r in rows if r['mode']==mode and r['label']==label]
            complete=[r for r in group if 'cost' in r]
            events=[e for r in group for e in r['stages']]
            stats[mode][label]=dict(cycles=len(group),ok=sum(r['status']=='ok' for r in group),
                over_200_ms=sum(r['total_ms']>200 for r in group),timeouts=sum(r['status']=='timeout_hold' for r in group),
                total_ms=summary([r['total_ms'] for r in group]),
                warm_total_ms=summary([r['total_ms'] for r in group if r['trial']>0]),
                proposal_ms=summary([r['proposal_ms'] for r in complete]),
                reach_cycle_ms=summary([r['reach_ms'] for r in group]),
                reach_call_ms=summary([e['ms'] for e in events if e['stage']=='reach']),
                stage_statuses={s:{status:sum(e['stage']==s and e['status']==status for e in events)
                                  for status in ('converged','not_converged','deadline','exception')}
                                for s in ('reach','wheel_value','actual_value')},
                optimized=sum(r['optimized'] for r in complete),
                selected_optimized=sum(r['selected_optimized'] for r in complete),
                budget_exhausted=sum(r['proposal_budget_exhausted'] for r in complete),
                cost=summary([r['cost'] for r in complete if np.isfinite(r['cost'])]))
    for name,sha in hashes.items():assert hashlib.sha256((ROOT/'dra_mppi'/name).read_bytes()).hexdigest()==sha
    report['summary']=stats
    write(out/'benchmark.json',report)
    print(json.dumps(serial(stats),indent=2),flush=True)


if __name__=='__main__':main()

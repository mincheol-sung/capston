"""Paired before/after gap latency, with equal-work and production-budget runs.

Loads the saved pre-change package under a separate namespace. Runs the two
controllers sequentially in alternating order; never runs robot/ROS commands.
"""
import argparse
import hashlib
import importlib.util
import json
from dataclasses import asdict,fields,replace
from pathlib import Path
import sys
import time

import numpy as np
import yaml

from benchmark_guide_conditioned import config_from_yaml,summary
from dra_mppi.mppi_controller import MppiController
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.gap_inputs import GapScan,fuse_scans
from dra_mppi.gap_geometry import GapConfig
from dra_mppi.gap_optimization import rollout_jacobian
from dra_mppi.grid_costmap import DistanceField,GridMetadata

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'docs/gap_value_optimization_2026-11-05'
OUT=ROOT/'docs/gap_latency_optimization_2026-11-06'


def serial(x):
    if isinstance(x,dict):return {k:serial(v)for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [serial(v)for v in x]
    if isinstance(x,np.ndarray):return serial(x.tolist())
    if isinstance(x,np.integer):return int(x)
    if isinstance(x,np.bool_):return bool(x)
    if isinstance(x,(float,np.floating)):return float(x)if np.isfinite(x)else None
    return x


def write(path,data):path.write_text(json.dumps(serial(data),indent=2,allow_nan=False)+'\n')


def load_baseline(path):
    name='gap_latency_baseline'
    spec=importlib.util.spec_from_file_location(name,path/'__init__.py',submodule_search_locations=[str(path)])
    package=importlib.util.module_from_spec(spec);sys.modules[name]=package;spec.loader.exec_module(package)
    from gap_latency_baseline.gap_inputs import fuse_scans as old_fuse
    from gap_latency_baseline.gap_optimization import rollout_jacobian as old_jac
    return package.MppiController,old_fuse,old_jac


def inputs(scene):
    data=dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
    field=DistanceField.from_occupancy(data['grid'],GridMetadata(200,200,.04,-4,-4,0))
    scans=tuple(GapScan(*(data[f'scan_{i}_{k}']for k in ('ranges','valid','origins','hits')))for i in range(2))
    predictions=None
    if 'prediction_means'in data:
        predictions=PredictionSet(data['prediction_means'],data['prediction_covariances'],data['prediction_times'],
            mode_weights=data.get('prediction_mode_weights'),obstacle_radii=data.get('prediction_obstacle_radii'))
    return data,field,scans,predictions


def microbench(sources,cfg,old_fuse,old_jac):
    times={'before':[],'after':[]};jac_times={'before':[],'after':[]}
    for source in sources:
        data,field,scans,_=inputs(source['scene'])
        for repeat in range(6):
            funcs=[('before',old_fuse),('after',fuse_scans)]
            if repeat%2:funcs.reverse()
            result={}
            for label,fun in funcs:
                tick=time.perf_counter();result[label]=fun(scans,np.zeros(3),field,None,cfg.gap,cfg.static_cost_robot_radius)
                ms=1000*(time.perf_counter()-tick)
                if repeat>=2:times[label].append(ms)
            for key in ('occupancy','clearance'):
                np.testing.assert_array_equal(getattr(result['before'][0],key),getattr(result['after'][0],key))
    rng=np.random.default_rng(20261106)
    for n in (1,3,10,17):
        for trial in range(10):
            u=rng.normal(0,.3,3*n)
            for a,b in zip(old_jac(u,n,.2),rollout_jacobian(u,n,.2)):
                np.testing.assert_array_equal(a,b)
    controls=rng.normal(0,.3,30)
    for repeat in range(8):
        funcs=[('before',old_jac),('after',rollout_jacobian)]
        if repeat%2:funcs.reverse()
        for label,fun in funcs:
            tick=time.perf_counter()
            for _ in range(300):fun(controls,10,.2)
            if repeat>=2:jac_times[label].append(1000*(time.perf_counter()-tick)/300)
    return dict(fusion_ms={k:summary(v)for k,v in times.items()},
                jacobian_ms={k:summary(v)for k,v in jac_times.items()},
                identical_grids_and_clearances=len(sources),identical_jacobian_cases=40)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations',type=int,default=8)
    parser.add_argument('--output',type=Path,default=OUT)
    parser.add_argument('--baseline-package',type=Path,default=OUT/'baseline_package/dra_mppi')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    old_controller,old_fuse,old_jac=load_baseline(args.baseline_package)
    p=yaml.safe_load((ROOT/'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    gap=GapConfig(**{f.name:p['gap.'+f.name]for f in fields(GapConfig)if 'gap.'+f.name in p})
    cfg=replace(config_from_yaml(),strategy='gap',gap=gap)
    sources=json.loads((SOURCE/'raw_sources.json').read_text())
    micro=microbench(sources,cfg,old_fuse,old_jac)
    write(args.output/'microbench.json',micro)
    print('MICRO',json.dumps(micro),flush=True)
    rows=[];equivalence=[]
    for mode in ('equal_work','production'):
        settings=cfg if mode=='production'else replace(cfg,gap=replace(gap,optimization_timeout=2.,timeout=5.))
        for source in sources:
            scene=source['scene'];data,field,scans,predictions=inputs(scene)
            controllers={'before':old_controller(settings),'after':MppiController(settings)}
            try:
                for trial in range(args.iterations):
                    order=['before','after']if trial%2==0 else ['after','before']
                    results={}
                    for label in order:
                        c=controllers[label];c._guide_optimizer._warm_nominal=data['nominal'].copy()
                        tick=time.perf_counter()
                        try:
                            r=c.compute(np.zeros(3),data['path'].copy(),data['current'].copy(),predictions,
                                        visualization_rollouts=32,distance_field=field,gap_scans=scans)
                            ms=1000*(time.perf_counter()-tick);d=r.gap_diagnostics
                            assert r.sampled_controls.shape==(500,settings.horizon_steps,3)
                            assert sum(r.guide_rollout_counts)==500
                            selected=r.candidate_names.index(r.selected_mode)
                            if r.safety_status=='ok':
                                assert not r.final_static_collisions[selected]
                                assert r.max_collision_probability<=settings.dynamic_risk.hard_threshold
                            row=dict(mode=mode,scene=scene,trial=trial,label=label,status=r.safety_status,
                                cost=r.cost,total_ms=ms,geometry_ms=d['geometry_ms'],proposal_ms=d['proposal_ms'],
                                mppi_ms=r.compute_time_ms-d['geometry_ms']-d['proposal_ms'],
                                primary_wall_ms=r.stage_ms['primary_wall'],
                                optimized=sum(q['seed_source']=='optimized'for q in d['candidates']),
                                selected_optimized=sum(q['seed_source']=='optimized'and q['reason']=='selected'for q in d['candidates']),
                                actual_value_completed=sum(q['actual_value_cost']is not None for q in d['candidates']),
                                counts=r.guide_rollout_counts,selected_mode=r.selected_mode,
                                risk_probability=r.max_collision_probability)
                            results[label]=r
                        except TimeoutError:
                            row=dict(mode=mode,scene=scene,trial=trial,label=label,status='timeout_hold',
                                     total_ms=1000*(time.perf_counter()-tick))
                        rows.append(row)
                    if mode=='equal_work':
                        assert len(results)==2,'Equal-work benchmark should not timeout'
                        a,b=results['before'],results['after']
                        np.testing.assert_array_equal(a.sampled_controls,b.sampled_controls)
                        np.testing.assert_array_equal(a.rollout_costs,b.rollout_costs)
                        np.testing.assert_array_equal(a.control_sequence,b.control_sequence)
                        np.testing.assert_array_equal(a.mode_final_costs,b.mode_final_costs)
                        assert a.selected_mode==b.selected_mode
                        for x,y in zip(a.gap_diagnostics['candidates'],b.gap_diagnostics['candidates']):
                            assert x['id']==y['id']and x['seed_source']==y['seed_source']
                            for key in ('wheel_distance_m','wheel_value_cost','actual_target_distance_m',
                                        'actual_value_cost','cost'):
                                assert x[key]==y[key],(scene,key,x[key],y[key])
                        equivalence.append(dict(scene=scene,trial=trial,exact_samples_costs_selection=True))
                print(mode,scene,'done',flush=True)
            finally:
                for c in controllers.values():c.close()
    stats={}
    for mode in ('equal_work','production'):
        stats[mode]={}
        for label in ('before','after'):
            group=[r for r in rows if r['mode']==mode and r['label']==label]
            completed=[r for r in group if 'cost'in r]
            stats[mode][label]=dict(cycles=len(group),ok=sum(r['status']=='ok'for r in group),
                timeout_holds=sum(r['status']=='timeout_hold'for r in group),
                total_ms=summary([r['total_ms']for r in group]),
                warm_total_ms=summary([r['total_ms']for r in group if r['trial']>0]),
                geometry_ms=summary([r['geometry_ms']for r in completed]),
                proposal_ms=summary([r['proposal_ms']for r in completed]),
                primary_wall_ms=summary([r['primary_wall_ms']for r in completed]),
                optimized=sum(r['optimized']for r in completed),
                selected_optimized=sum(r['selected_optimized']for r in completed),
                over_200_ms=sum(r['total_ms']>200 for r in group))
    unchanged=('common_cost.py','mppi_controller.py','guide_conditioned.py','dynamic_risk.py','gaussian_disk.py')
    code_hashes={}
    for f in unchanged:
        old=(args.baseline_package/f).read_bytes();new=(ROOT/'dra_mppi'/f).read_bytes()
        assert old==new
        code_hashes[f]=hashlib.sha256(new).hexdigest()
    report=dict(scope=__doc__,configuration=asdict(cfg),iterations=args.iterations,sources=sources,
                summary=stats,microbench=micro,equal_work_equivalence=equivalence,unchanged_evaluator_sha256=code_hashes,
                rows=rows,notes='Same process, alternating sequential order. Equal-work increases only test deadlines '
                'so every stage executes identically; production uses unchanged YAML 50ms/180ms limits and can '
                'complete more optimizations after speedup, changing allocations/results. Frozen bag snapshots, '
                'recorded nominal restored, no ROS callbacks or perception load. Not closed-loop or full-stack Hz certification.')
    write(args.output/'benchmark.json',report)
    print(json.dumps(serial(stats),indent=2),flush=True)


if __name__=='__main__':main()

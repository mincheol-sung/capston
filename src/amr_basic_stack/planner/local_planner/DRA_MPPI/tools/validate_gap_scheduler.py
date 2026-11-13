#!/usr/bin/env python3
"""Paired frozen bag snapshots: original, two-solve-only, production revision."""
import argparse
from dataclasses import asdict,fields,replace
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
import yaml

from benchmark_guide_conditioned import config_from_yaml
from benchmark_gap_latency import inputs,write
from experiment_gap_reach_overhead import Arm,run,summary

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_two_solve_scheduler_2026-11-09'
SOURCE=ROOT/'docs/gap_value_optimization_2026-11-05'


def config(arm,yaml_path):
    p=yaml.safe_load(yaml_path.read_text())['dra_mppi_node']['ros__parameters']
    module=__import__(arm.package.__name__+'.gap_geometry',fromlist=['GapConfig'])
    cls=module.GapConfig
    gap=cls(**{f.name:p['gap.'+f.name] for f in fields(cls) if 'gap.'+f.name in p})
    return replace(config_from_yaml(),strategy='gap',gap=gap)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=BASE/'comparison')
    parser.add_argument('--iterations',type=int,default=8)
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    snapshot=out/'updated_package/dra_mppi'
    shutil.copytree(ROOT/'dra_mppi',snapshot,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy2(ROOT/'config/dra_mppi.yaml',out/'updated_configuration.yaml')
    direct=out/'direct_package/dra_mppi';shutil.copytree(BASE/'baseline_package/dra_mppi',direct)
    path=direct/'gap_conditioned.py';s=path.read_text()
    old="candidate['value_difference']=assessed['cost']-candidate['wheel_value_cost']"
    assert s.count(old)==1
    path.write_text(s.replace(old,"candidate['value_difference']=(assessed['cost']-candidate['wheel_value_cost'] if candidate['wheel_value_cost'] is not None else None)"))
    arms={'current':Arm('current',BASE/'baseline_package/dra_mppi'),
          'direct':Arm('direct',direct),'updated':Arm('updated',snapshot)}
    # Arm provides stage timing instrumentation; the updated arm must call the
    # actual production solver, not the experiment-only two-stage function.
    arms['updated'].gap.solve_gap_values=arms['updated'].opt.solve_gap_values
    configs={k:config(v,out/'updated_configuration.yaml' if k=='updated' else BASE/'baseline_configuration.yaml') for k,v in arms.items()}
    sources=json.loads((SOURCE/'raw_sources.json').read_text());rows=[]
    for source in sources:
        scene=source['scene'];data,field,scans,predictions=inputs(scene)
        controllers={k:a.package.MppiController(configs[k]) for k,a in arms.items()}
        try:
            for trial in range(args.iterations):
                order=np.roll(list(arms),-(trial%3))
                if trial%2:order=order[::-1]
                for name in order:
                    row,result=run(arms[name],controllers[name],data,field,scans,predictions,'production',scene,trial)
                    if result is not None:
                        d=result.gap_diagnostics
                        for key in ('proposal_budget_ms','mppi_reserved_ms','post_proposal_ms','optimization_order','solver_policy'):
                            if key in d:row[key]=d[key]
                        if name=='updated':
                            assert d['solver_policy']=='reach_then_actual'
                            assert d['scheduling_policy']=='oldest_attempt_then_unfinished'
                            for q in d['candidates']:
                                assert q['wheel_value_cost'] is None and q['value_difference'] is None
                    rows.append(row)
        finally:
            for c in controllers.values():c.close()
        write(out/'partial.json',dict(rows=rows))
        print(scene,'done',flush=True)
    stats={}
    for name in arms:
        group=[r for r in rows if r['label']==name];ok=[r for r in group if 'cost' in r]
        qs=[q for r in ok for q in r['candidates']]
        stats[name]=dict(cycles=len(group),ok=sum(r['status']=='ok' for r in group),
            timeouts=sum(r['status']=='timeout_hold' for r in group),
            over_200_ms=sum(r['total_ms']>200 for r in group),total_ms=summary([r['total_ms'] for r in group]),
            cold_ms=summary([r['total_ms'] for r in group if r['trial']==0]),
            warm_ms=summary([r['total_ms'] for r in group if r['trial']>0]),
            candidate_count=len(qs),optimized=sum(q['seed_source']=='optimized' for q in qs),
            reused=sum(q['seed_source']=='reused' for q in qs),fallback=sum(q['seed_source']=='fallback' for q in qs),
            not_attempted=sum(q['optimization_status']=='not_attempted' for q in qs),
            interrupted=sum(q['optimization_status']=='budget_exhausted' for q in qs),
            budget_exhausted=sum(r['proposal_budget_exhausted'] for r in ok),
            selected_optimized=sum(r['selected_optimized'] for r in ok),
            proposal_ms=summary([r['proposal_ms'] for r in ok]),
            cost=summary([r['cost'] for r in ok]),
            proposal_budget_ms=summary([r['proposal_budget_ms'] for r in ok if 'proposal_budget_ms' in r]),
            mppi_reserved_ms=summary([r['mppi_reserved_ms'] for r in ok if 'mppi_reserved_ms' in r]))
    unchanged=('common_cost.py','mppi_controller.py','guide_conditioned.py','dynamic_risk.py','gaussian_disk.py')
    hashes={}
    for name in unchanged:
        old=(BASE/'baseline_package/dra_mppi'/name).read_bytes();new=(ROOT/'dra_mppi'/name).read_bytes()
        assert old==new;hashes[name]=hashlib.sha256(new).hexdigest()
    for path in snapshot.glob('*.py'):assert path.read_bytes()==(ROOT/'dra_mppi'/path.name).read_bytes()
    write(out/'benchmark.json',dict(scope=__doc__,sources=sources,configurations={k:asdict(v) for k,v in configs.items()},
          iterations=args.iterations,rows=rows,summary=stats,unchanged_evaluators=hashes,
          notes='No original MCAP/DB3 currently available; preserved bag-derived raw arrays. '
          'Frozen snapshots, recorded nominal restored; cache and scheduling remain persistent per scene. '
          'Includes dynamic predictions when present. Excludes ROS callbacks and full perception load.'))
    print(json.dumps(stats,indent=2),flush=True)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Offline A/B: fixed min(reach,LiDAR) endpoint versus free distance on gap ray.

Only the experiment namespace replaces the proposal solver. All production
files, 500-sample costs, allocation, duplicate checks and safety checks remain.
"""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

from benchmark_gap_latency import inputs, write
from experiment_gap_reach_overhead import Arm, run, summary
from validate_gap_scheduler import config

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'docs/gap_value_optimization_2026-11-05'
OUT = ROOT/'docs/gap_endpoint_policy_2026-11-09'


def free_solver(arm):
    def solve(cfg, current, path, angle, depth, start, common_reference=None,
              deadline=float('inf'), diagnostics=None):
        m = arm.opt
        info = {} if diagnostics is None else diagnostics
        info.update(m.gap_value_diagnostics())
        info.update(wheel_reach_status='not_computed', optimization_stage='actual_value',
                    actual_value_status='running')
        problem = m.ValueProblem(cfg, current, path, angle, float(depth), common_reference)
        assert problem.target_distance is None
        try:
            actual = problem.solve(start, deadline)
        except m.OptimizationDeadline:
            info['actual_value_status'] = 'budget_exhausted'
            raise
        valid = (actual['success'] and
                 actual['violation'] <= cfg.gap.optimization_constraint_tolerance and
                 np.isfinite(actual['cost']) and np.all(np.isfinite(actual['controls'])))
        info.update(actual_value_status='converged' if valid else 'not_converged',
                    actual_value_ms=actual['wall_ms'], actual_value_iterations=actual['iterations'])
        if not valid:
            return None
        rho = float(actual['trajectory'][-1, :2] @ problem.e)
        assert cfg.gap.min_displacement_m-1e-5 <= rho <= depth+1e-5
        assert abs(actual['trajectory'][-1, :2] @ problem.perp) <= 1e-5
        info.update(actual_target_distance_m=rho, actual_value_cost=actual['cost'],
                    actual_value_components=actual['components'], optimization_stage='complete')
        return actual
    return solve


def stats(rows):
    good = [r for r in rows if 'cost' in r]
    qs = [q for r in good for q in r['candidates']]
    optimized = [q for q in qs if q['seed_source']=='optimized']
    return dict(cycles=len(rows), ok=sum(r['status']=='ok' for r in rows),
        timeouts=sum(r['status']=='timeout_hold' for r in rows),
        over_200_ms=sum(r['total_ms']>200 for r in rows),
        total_ms=summary([r['total_ms'] for r in rows]),
        proposal_ms=summary([r['proposal_ms'] for r in good]),
        reach_ms=summary([r['reach_ms'] for r in rows]),
        actual_value_ms=summary([r['actual_value_ms'] for r in rows]),
        cost=summary([r['cost'] for r in good]),
        candidate_count=len(qs), optimized=len(optimized),
        optimized_at_minimum=sum(q['projected_displacement_m']<=.251 for q in optimized),
        reasons=dict(Counter(q['reason'] for q in qs)),
        statuses=dict(Counter(q['optimization_status'] for q in qs)),
        sources=dict(Counter(q['seed_source'] for q in qs)),
        selected_gaps=summary([len(r['selected']) for r in good]),
        final_modes=dict(Counter(r['selected_mode'] for r in good)),
        final_displacement_m=summary([r['final_displacement_m'] for r in good]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUT)
    parser.add_argument('--iterations',type=int,default=8)
    parser.add_argument('--scene')
    args=parser.parse_args(); out=args.output
    out.mkdir(parents=True,exist_ok=True)
    snapshot=out/'source_snapshot/dra_mppi'
    if snapshot.exists(): raise RuntimeError('Use a fresh output directory')
    shutil.copytree(ROOT/'dra_mppi',snapshot,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copy2(ROOT/'config/dra_mppi.yaml',out/'configuration.yaml')
    shutil.copy2(Path(__file__),out/'experiment.py')
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')}
    arms={name:Arm(name,snapshot) for name in ('fixed','free')}
    arms['fixed'].gap.solve_gap_values=arms['fixed'].opt.solve_gap_values
    arms['free'].gap.solve_gap_values=free_solver(arms['free'])
    cfg=config(arms['fixed'],out/'configuration.yaml')
    sources=json.loads((SOURCE/'raw_sources.json').read_text())
    if args.scene: sources=[s for s in sources if s['scene']==args.scene]
    rows=[]
    # Warm imports and numeric libraries; excluded from measurements.
    data,field,scans,predictions=inputs(sources[0]['scene'])
    generous=replace(cfg,gap=replace(cfg.gap,optimization_timeout=2.,timeout=5.,cache_max_age=3600.))
    for arm in arms.values():
        c=arm.package.MppiController(generous)
        try: run(arm,c,data,field,scans,predictions,'warmup',sources[0]['scene'],-1)
        finally: c.close()
    for source in sources:
        scene=source['scene'];data,field,scans,predictions=inputs(scene)
        for mode in ('uncensored','production'):
            controllers={}
            if mode=='production':
                controllers={k:a.package.MppiController(cfg) for k,a in arms.items()}
            try:
                for trial in range(args.iterations):
                    # Uncensored: independent paired sampling seeds, fresh state.
                    # Production: recorded nominal, persistent scheduling/cache.
                    settings=replace(generous,seed=cfg.seed+trial*31) if mode=='uncensored' else cfg
                    if mode=='uncensored':
                        controllers={k:a.package.MppiController(settings) for k,a in arms.items()}
                    for name in (('fixed','free') if trial%2==0 else ('free','fixed')):
                        arm=arms[name];c=controllers[name]
                        row,r=run(arm,c,data,field,scans,predictions,mode,scene,trial)
                        row['sampling_seed']=settings.seed
                        if r is not None:
                            d=r.gap_diagnostics
                            row.update(final_displacement_m=float(np.linalg.norm(r.trajectory[-1,:2])),
                                trajectory=r.trajectory, controls=r.control_sequence,
                                candidate_names=r.candidate_names, representative_costs=r.mode_final_costs,
                                representative_trajectories=r.mode_trajectories,
                                anchors=r.guide_anchors, regions=r.gap_regions,
                                nominal_cost=d['nominal_cost'],
                                proposal_budget_ms=d['proposal_budget_ms'],
                                mppi_reserved_ms=d['mppi_reserved_ms'],
                                post_proposal_ms=d['post_proposal_ms'])
                            if mode=='uncensored':
                                assert not d['proposal_budget_exhausted']
                            if name=='free':
                                assert row['reach_calls']==0
                                assert all(q['wheel_distance_m'] is None for q in d['candidates'])
                        rows.append(row)
                    if mode=='uncensored':
                        for c in controllers.values(): c.close()
                        controllers={}
            finally:
                for c in controllers.values(): c.close()
        write(out/'partial.json',dict(rows=rows))
        print(scene,'done',flush=True)
    summaries={mode:{name:stats([r for r in rows if r['mode']==mode and r['label']==name])
        for name in arms} for mode in ('uncensored','production')}
    for name,sha in hashes.items():
        assert hashlib.sha256((ROOT/'dra_mppi'/name).read_bytes()).hexdigest()==sha
    write(out/'benchmark.json',dict(configuration=asdict(cfg),sources=sources,source_hashes=hashes,
        rows=rows,summary=summaries,iterations=args.iterations,
        notes='Preserved real bag-derived snapshots, not new bag/ROS playback. '
        'Uncensored: 2s proposal / 5s cycle, eight paired independent sampling seeds, fresh controllers. '
        'Production: current adaptive budget, eight frozen cycles, cache/schedule persistent; '
        'nominal restored each call. Alternating sequential arm order. '
        'Both allow omega, identical four-term value cost and 500-rollout full cost/safety/allocation. '
        'Free uses existing ValueProblem target_distance=None; no reach solve. '
        'Feasibility assertions and unchanged production source hashes checked.'))
    print(json.dumps(summaries,indent=2),flush=True)


if __name__=='__main__': main()

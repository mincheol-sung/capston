#!/usr/bin/env python3
"""Exploratory second-stage refinement, explicitly after inspecting stage one.

Keep the frozen stage-one planner and all accepted snapshots. Within each
valley-bounded angular interval compare peak, midpoint and path-aligned seeds
with the existing objective, then select at most one representative per gap.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base',type=Path)
    args=parser.parse_args();base=args.base.resolve()
    sys.path.insert(0,str(base/'source_snapshot/tools'))
    import experiment_gap_multiscene as ex
    out=base/'refinement';out.mkdir(exist_ok=True);(out/'scenes').mkdir(exist_ok=True)
    methods=['valley_direction_cost','valley_direction_gate']
    ex.write_json(out/'protocol.json',dict(
        stage='Exploratory refinement after stage-one inspection; same evaluation bags, not new untouched validation.',
        motivation='Stage-one peak-only representative lost a lower-cost direction in b2_084.',
        methods=methods, directions='For each existing valley interval: depth peak, angular midpoint, closest angle to the recorded path point at 1 m arc length. Deduplicate identical angle bins.',
        selection='Choose the lowest existing common+static cost among collision-free, reachable, non-nominal-duplicate options; at most one seed per original interval. Then use the original cost or cost_gate selector.',
        fixed='Same frozen source, snapshots, gap boundaries, reach limits, 500-rollout budget, MC settings and seeds as stage one.',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    original=[json.loads(x) for x in (base/'scene_audit.jsonl').read_text().splitlines()]
    accepted={s['record_ns']:s for s in original if s['accepted']}
    cfg=ex.config_from_yaml();done=0
    with (out/'trials.jsonl').open('w') as tf,(out/'scene_audit.jsonl').open('w') as af:
        for bag in ex.BAGS:
            print('READ',bag,flush=True)
            data=ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
            for snap in data['snapshots']:
                if snap['record_ns'] not in accepted:continue
                audit=accepted[snap['record_ns']]
                started=time.perf_counter()
                p=ex.prepare_snapshot(data,snap);ex.input_check(data,snap,p)
                c=ex.MppiController(replace(cfg,strategy='standard'));c.gap_path=p['path']
                predictions=ex.recorded_predictions(p,c)
                nominal=c._constrain_controls(p['nominal'][None],p['current'])[0]
                nominal_path=ex.rollout(np.zeros(3),nominal,.2)
                nominal_collision=bool(p['field'].trajectory_collisions(nominal_path[None,:,:2],np.zeros(2),ex.RADIUS)[0])
                nominal_cost=float(ex.cheap_cost(c,p,nominal[None],nominal_path[None])[0])
                escape=nominal_collision or ex.path_progress(p,nominal_path[None])[0]<.15 or ex.nominal_threat(predictions,nominal_path)
                prep_ms=(time.perf_counter()-started)*1000
                pool,pool_ms,options_count=make_pool(ex,p,c,nominal_path,nominal_collision)
                arrays={};items_by_method={}
                for method in methods:
                    rule='gate' if method.endswith('gate') else 'cost'
                    items,chosen,selection_ms=ex.select(p,pool,rule,nominal_collision,nominal_cost,escape)
                    items_by_method[method]=[{k:v for k,v in g.items() if k not in ('controls','trajectory','ids')}
                                             for g in items]
                    arrays[method+'_seeds']=np.array([g['trajectory'] for g in chosen])
                    arrays[method+'_targets']=np.array([g['target'] for g in chosen])
                    for seed in ex.SEEDS:
                        row=dict(scene=audit['scene'],bag=bag,split=audit['split'],offset=snap['elapsed'],method=method,
                                 seed=seed,prepare_ms=prep_ms,proposal_ms=pool_ms+selection_ms,raw_gaps=len(pool),
                                 seed_options=options_count,selected_gaps=len(chosen),escape=bool(escape),
                                 rearward_seeds=sum(g['progress_proxy']<0 for g in chosen),
                                 reach_rejected=sum(g['reason']=='unreachable' for g in items),
                                 selected_angles=[float(np.rad2deg(g['angle'])) for g in chosen])
                        try:
                            t=time.perf_counter()
                            result,paths=ex.run_optimizer(p,cfg,nominal,chosen,predictions,method,seed)
                            wall_ms=(time.perf_counter()-t)*1000
                            row.update(result);row['offline_total_ms']=prep_ms+pool_ms+selection_ms+wall_ms
                            if seed==7:arrays[method+'_trajectory']=paths['trajectory']
                        except Exception as exc:
                            row.update(status='exception',safe=False,moving=False,error=str(exc))
                        tf.write(json.dumps(ex.serial(row),ensure_ascii=False)+'\n');tf.flush()
                af.write(json.dumps(ex.serial(dict(scene=audit['scene'],bag=bag,offset=snap['elapsed'],proposals=items_by_method)),ensure_ascii=False)+'\n');af.flush()
                np.savez_compressed(out/'scenes'/f'{audit["scene"]}.npz',**arrays)
                done+=1
                if done%10==0:print('PROGRESS',done,flush=True)
    ex.write_json(out/'run_summary.json',dict(scenes=done,expected_trials=done*4))
    assert done==len(accepted)
    print('DONE',done,flush=True)


def make_pool(ex,p,c,nominal_path,nominal_collision):
    started=time.perf_counter();definitions=ex.definitions(p,'valley')
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p['path'],axis=0),axis=1))]
    path_target=p['path'][int(np.argmin(np.abs(arc-min(1.,arc[-1]))))]
    desired=np.arctan2(path_target[1],path_target[0])
    all_options=[];group_options=[]
    for i,definition in enumerate(definitions):
        ids=definition['ids'];angles=p['angles'][ids]
        nearest=ids[np.argmin(np.abs(np.arctan2(np.sin(angles-desired),np.cos(angles-desired))))]
        indices=list(dict.fromkeys([int(definition['index']),int(ids[len(ids)//2]),int(nearest)]))
        options=[]
        for index in indices:
            g=definition.copy();g.update(id=f'valley_{i}',angle=p['angles'][index],index=index)
            unit=np.array([np.cos(g['angle']),np.sin(g['angle'])])
            length=max(.1,p['d_free'][index]-.10)
            far=ex.seed_control(c,length*unit,p['current'])
            reach=float(np.dot(ex.rollout(np.zeros(3),far,.2)[-1,:2],unit))
            length=min(length,max(.25,.90*reach))
            g.update(target_distance=length,target=length*unit)
            controls=ex.seed_control(c,g['target'],p['current']);trajectory=ex.rollout(np.zeros(3),controls,.2)
            delta=np.linalg.norm(trajectory[:,:2]-nominal_path[:,:2],axis=1)
            g.update(controls=controls,trajectory=trajectory,rms=float(np.sqrt(np.mean(delta**2))),
                     tail=float(np.mean(delta[-3:])),reach_error=float(np.linalg.norm(trajectory[-1,:2]-g['target'])),
                     collision=bool(p['field'].trajectory_collisions(trajectory[None,:,:2],np.zeros(2),ex.RADIUS)[0]),
                     displacement=float(np.linalg.norm(trajectory[-1,:2])))
            tangent=p['path'][min(3,len(p['path'])-1)]-p['path'][0];tangent/=max(np.linalg.norm(tangent),1e-9)
            g['progress_proxy']=float(np.dot(trajectory[-1,:2],tangent))
            options.append(g);all_options.append(g)
        group_options.append(options)
    if all_options:
        costs=ex.cheap_cost(c,p,np.array([g['controls'] for g in all_options]),np.array([g['trajectory'] for g in all_options]))
        for g,cost in zip(all_options,costs):g['cheap_cost']=float(cost)
    pool=[]
    for options in group_options:
        eligible=[g for g in options if not g['collision'] and g['reach_error']<=.35
                  and (nominal_collision or not (g['rms']<.22 and g['tail']<.30))]
        chosen=min(eligible or options,key=lambda g:g['cheap_cost']).copy()
        chosen['direction_options']=[dict(angle_deg=float(np.rad2deg(g['angle'])),cost=g['cheap_cost'],
                                           rms=g['rms'],collision=g['collision'],reach_error=g['reach_error']) for g in options]
        pool.append(chosen)
    return pool,(time.perf_counter()-started)*1000,len(all_options)


if __name__=='__main__':main()

"""Exploratory full-option selection and no-gap audit on frozen real snapshots."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'docs/gap_multiscene_2026-11-04'
OUT = BASE/'diversity_and_empty_gap'
sys.path.insert(0, str(BASE/'source_snapshot/tools'))
import experiment_gap_multiscene as ex
from experiment_gap_direction_refinement import make_pool as old_pool


def load(path):
    return [json.loads(x) for x in path.read_text().splitlines()]


def options(p, c, nominal):
    start = time.perf_counter()
    defs = ex.definitions(p, 'valley')
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p['path'], axis=0), axis=1))]
    target = p['path'][int(np.argmin(np.abs(arc-min(1., arc[-1]))))]
    desired = np.arctan2(target[1], target[0])
    tangent = p['path'][min(3,len(p['path'])-1)]-p['path'][0]
    tangent /= max(np.linalg.norm(tangent), 1e-9)
    pool = []
    for gi, definition in enumerate(defs):
        ids = definition['ids']; a = p['angles'][ids]
        near = ids[np.argmin(np.abs(np.arctan2(np.sin(a-desired), np.cos(a-desired))))]
        indices = list(dict.fromkeys([int(definition['index']), int(ids[len(ids)//2]), int(near)]))
        for index in indices:
            angle = p['angles'][index]; unit = np.array([np.cos(angle), np.sin(angle)])
            length = max(.1, p['d_free'][index]-.1)
            far = ex.seed_control(c, length*unit, p['current'])
            reach = float(ex.rollout(np.zeros(3), far, .2)[-1,:2] @ unit)
            length = min(length, max(.25, .9*reach))
            u = ex.seed_control(c, length*unit, p['current'])
            path = ex.rollout(np.zeros(3), u, .2)
            delta = np.linalg.norm(path[:,:2]-nominal[:,:2], axis=1)
            g = dict(id=f'valley_{gi:02d}_dir_{index:03d}', gap=gi, angle=angle, index=index,
                     controls=u, trajectory=path, target=length*unit, target_distance=length,
                     rms=float(np.sqrt(np.mean(delta**2))), tail=float(np.mean(delta[-3:])),
                     reach_error=float(np.linalg.norm(path[-1,:2]-length*unit)),
                     collision=bool(p['field'].trajectory_collisions(path[None,:,:2],np.zeros(2),ex.RADIUS)[0]),
                     progress_proxy=float(path[-1,:2] @ tangent), rear_body=bool(path[-1,0]<-.05))
            pool.append(g)
    if pool:
        costs = ex.cheap_cost(c,p,np.array([g['controls'] for g in pool]),np.array([g['trajectory'] for g in pool]))
        for g,j in zip(pool,costs):g['cheap_cost']=float(j)
    return pool, (time.perf_counter()-start)*1000


def hard_depth(p):
    start=time.perf_counter()
    a=p['angles'];d=np.arange(0.,2.02,.02)
    rays=np.stack([np.cos(a),np.sin(a)],axis=1)[:,None,:]*d[None,:,None]
    clearance=p['field'].query(rays)-np.sqrt(2)*.04-.01
    depth=np.max(np.where(np.logical_and.accumulate(clearance>ex.RADIUS,axis=1),d,0),axis=1)
    return depth,(time.perf_counter()-start)*1000


def light(pool):
    return [{k:v for k,v in g.items() if k not in ('controls','trajectory','ids')} for g in pool]


def main():
    OUT.mkdir(exist_ok=True);(OUT/'scenes').mkdir(exist_ok=True)
    methods=['all_options_cost','all_options_hard_fallback']
    ex.write_json(OUT/'protocol.json',dict(
        purpose='After observing rearward option loss, compare full seed pools and audit empty gaps; exploratory reuse, no independent validation.',
        methods=methods, baseline='valley_direction_cost: one cheapest feasible seed per gap',
        selection='All peak/midpoint/path-aligned options compete globally; static collision, reach <=0.35m, nominal RMS<0.22m AND tail<0.30m gates unchanged; cost-ranked greedy trajectory RMS >=0.25m; max3.',
        fallback='Only if strict raw gap count is zero: repeat the same gap definition using body-radius clearance without the additional 0.05m proposal buffer. Grid/sweep conservatism, unknown blocked, full trajectory collision and MC remain unchanged. Minimum depth0.5m and width6 bins unchanged.',
        fixed='Frozen planner/config; same 127 valid snapshots, noise seeds7/23; total500; 100 per extra seed; nominal remainder; 10s offline timeout.',
        gap_count='A geometric gap may now provide more than one MPPI guide; selected_guides is distinct from geometric gaps.',
        time='Reused input preparation and proposal timings plus fresh solver setup/compute/teardown; no ROS/full-stack claim.',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    valid={r['record_ns']:r for r in load(BASE/'scene_audit.jsonl') if r['accepted']}
    prev={(r['scene'],r['seed']):r for r in load(BASE/'refinement/trials.jsonl') if r['method']=='valley_direction_cost'}
    cfg=ex.config_from_yaml();done=0;identical=0
    with (OUT/'trials.jsonl').open('w') as tf,(OUT/'scene_audit.jsonl').open('w') as af:
        for bag in ex.BAGS:
            print('READ',bag,flush=True)
            data=ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
            for snap in data['snapshots']:
                if snap['record_ns'] not in valid:continue
                audit=valid[snap['record_ns']];scene=audit['scene']
                tick=time.perf_counter();p=ex.prepare_snapshot(data,snap);ex.input_check(data,snap,p)
                saved=np.load(BASE/'scenes'/f'{scene}.npz')
                np.testing.assert_allclose(p['d_free'],saved['d_free'],atol=1e-10)
                c=ex.MppiController(replace(cfg,strategy='standard'));c.gap_path=p['path']
                predictions=ex.recorded_predictions(p,c)
                nominal=c._constrain_controls(p['nominal'][None],p['current'])[0]
                nominal_path=ex.rollout(np.zeros(3),nominal,.2)
                coll=audit['nominal_collision']
                nominal_cost=float(ex.cheap_cost(c,p,nominal[None],nominal_path[None])[0])
                prep_ms=(time.perf_counter()-tick)*1000
                pool,pool_ms=options(p,c,nominal_path)
                old,_,_=old_pool(ex,p,c,nominal_path,coll)
                _,old_chosen,_=ex.select(p,old,'cost',coll,nominal_cost,True)
                old_controls=np.array([g['controls'] for g in sorted(old_chosen,key=lambda g:g['id'])])
                depth,hard_ms=hard_depth(p)
                raw=len(ex.definitions(p,'valley'))
                hard_p={**p,'d_free':depth}
                hard_defs=ex.definitions(hard_p,'valley')
                origin_clear=float(p['field'].query(np.zeros((1,2)))[0]-np.sqrt(2)*.04-.01)
                details=dict(scene=scene,bag=bag,offset=snap['elapsed'],raw_gaps=raw,
                    strict_max_depth=float(p['d_free'].max()),strict_open_rays=int(sum(p['d_free']>=.5)),
                    hard_max_depth=float(depth.max()),hard_open_rays=int(sum(depth>=.5)),hard_raw_gaps=len(hard_defs),
                    origin_adjusted_clearance=origin_clear,nominal_collision=coll,nominal_cost=nominal_cost,
                    baseline_guides=len(old_chosen),baseline_rear=sum(g['trajectory'][-1,0]<-.05 for g in old_chosen),
                    baseline_rear_available=sum(g['rear_body'] and not g['collision'] and g['reach_error']<=.35 and (coll or not(g['rms']<.22 and g['tail']<.30)) for g in pool),
                    methods={})
                arrays=dict(hard_d_free=depth)
                for method in methods:
                    fallback=method.endswith('fallback') and raw==0
                    pp=hard_p if fallback else p
                    candidate,ms=options(pp,c,nominal_path) if fallback else (pool,pool_ms)
                    items,chosen,selection_ms=ex.select(pp,candidate,'cost',coll,nominal_cost,True)
                    details['methods'][method]=dict(fallback=fallback,options=light(items),selected_ids=[g['id'] for g in chosen],
                        selected_rear=sum(g['rear_body'] for g in chosen))
                    chosen_order=sorted(chosen,key=lambda g:g['id'])
                    controls=np.array([g['controls'] for g in chosen_order])
                    same=controls.shape==old_controls.shape and np.allclose(controls,old_controls,atol=1e-10)
                    arrays[method+'_seeds']=np.array([g['trajectory'] for g in chosen_order])
                    arrays[method+'_targets']=np.array([g['target'] for g in chosen_order])
                    for seed in ex.SEEDS:
                        t=time.perf_counter()
                        result,paths=ex.run_optimizer(p,cfg,nominal,chosen,predictions,method,seed)
                        elapsed=(time.perf_counter()-t)*1000
                        r=dict(scene=scene,bag=bag,method=method,seed=seed,raw_gaps=len(ex.definitions(pp,'valley')),
                            selected_guides=len(chosen),rear_seeds=sum(g['rear_body'] for g in chosen),
                            fallback=fallback,same_controls_as_baseline=bool(same),prepare_ms=prep_ms,
                            proposal_ms=ms+selection_ms+(hard_ms if fallback else 0),**result)
                        r['offline_total_ms']=prep_ms+r['proposal_ms']+elapsed
                        if same and r['cost'] is not None and prev[scene,seed]['cost'] is not None:
                            np.testing.assert_allclose(r['cost'],prev[scene,seed]['cost'],rtol=1e-9,atol=1e-9)
                            identical+=1
                        tf.write(json.dumps(ex.serial(r))+'\n');tf.flush()
                        if seed==7:
                            arrays[method+'_trajectory']=paths['trajectory']
                            arrays[method+'_representatives']=paths['representatives']
                af.write(json.dumps(ex.serial(details))+'\n');af.flush()
                np.savez_compressed(OUT/'scenes'/f'{scene}.npz',**arrays)
                c.close();done+=1
                if done%10==0:print('PROGRESS',done,'/127',flush=True)
    assert done==127
    ex.write_json(OUT/'run_summary.json',dict(scenes=done,trials=done*4,identical_control_cost_checks=identical))
    print('DONE',done,'identical checks',identical,flush=True)


if __name__=='__main__':main()

"""Paired offline comparison of midpoint-gap ranking and rollout allocation.

Uses all 127 previously accepted snapshots, two noise seeds, the frozen actual
MPPI/MC implementation, and identical total rollout budgets. No ROS publishing.
"""
from dataclasses import replace
from pathlib import Path
import hashlib
import json
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_multiscene_2026-11-04'
OUT=BASE/'value_rule_comparison'
import experiment_disparity_partition as partitioner
import evaluate_gap_midpoint_seeds as mid
ex=mid.ex
RULES=['common','endpoint','free_area','novelty']
METHODS=RULES+['common_weighted']
NOISE_SEEDS=[7,23]


def arrays_and_partition(p):
    a=p['angles'];dist=np.arange(0.,2.02,.02)
    rays=np.stack([np.cos(a),np.sin(a)],axis=1)[:,None,:]*dist[None,:,None]
    clearance=p['field'].query(rays)-np.sqrt(2)*.04-.01
    hard=np.max(np.where(np.logical_and.accumulate(clearance>.2,axis=1),dist,0),axis=1)
    arrays=dict(strict_depth=p['d_free'],hard_depth=hard)
    for i,scan in enumerate(p['scans']):
        for key in ['ranges','valid','hits']:arrays[f'scan_{i}_{key}']=scan[key]
    parts=partitioner.partition(arrays)
    return parts,hard if parts['fallback'] else p['d_free']


def build_pool(p,c,parts,depth):
    nominal=c._constrain_controls(p['nominal'][None],p['current'])[0]
    nominal_path=ex.rollout(np.zeros(3),nominal,.2)
    nominal_collision=bool(p['field'].trajectory_collisions(nominal_path[None,:,:2],np.zeros(2),ex.RADIUS)[0])
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p['path'],axis=0),axis=1))]
    target=p['path'][int(np.argmin(abs(arc-min(1.,arc[-1]))))]
    c.gap_path=p['path'];items=[]
    for region in parts['regions']:
        ids=np.array(region['bins']);index=int(ids[len(ids)//2]);kind='angular_midpoint'
        if len(ids)==360:
            desired=np.arctan2(target[1],target[0])
            index=int(np.argmin(abs(np.arctan2(np.sin(p['angles']-desired),np.cos(p['angles']-desired)))))
            kind='full_circle_path_bearing'
        angle=p['angles'][index];unit=np.array([np.cos(angle),np.sin(angle)])
        available=max(.1,float(depth[index])-.1)
        far=ex.seed_control(c,available*unit,p['current'])
        reach=float(ex.rollout(np.zeros(3),far,.2)[-1,:2]@unit)
        length=min(available,max(.25,.9*reach))
        u=ex.seed_control(c,length*unit,p['current']);path=ex.rollout(np.zeros(3),u,.2)
        delta=np.linalg.norm(path[:,:2]-nominal_path[:,:2],axis=1)
        items.append(dict(id=f'gap_{region["id"]:02d}',gap=region['id'],angle_deg=float(np.rad2deg(angle)),
            direction_kind=kind,controls=u,trajectory=path,
            target_error_m=float(np.linalg.norm(path[-1,:2]-length*unit)),
            projected_displacement_m=float(path[-1,:2]@unit),
            nominal_rms_m=float(np.sqrt(np.mean(delta**2))),nominal_tail_m=float(np.mean(delta[-3:])),
            collision=bool(p['field'].trajectory_collisions(path[None,:,:2],np.zeros(2),ex.RADIUS)[0]),
            endpoint_score=float(np.sum((path[-1,:2]-target)**2)),
            area_m2=float(.5*np.sum(depth[ids]**2)*np.deg2rad(1.))))
    if items:
        costs=ex.cheap_cost(c,p,np.array([g['controls'] for g in items]),np.array([g['trajectory'] for g in items]))
        for g,j in zip(items,costs):
            g['cost']=float(j);g['reason']=mid.classify(g,nominal_collision)
    return items,nominal,nominal_path,nominal_collision,target


def score(rule,eligible,c,p,target):
    if not eligible:return np.array([])
    if rule=='common':
        return ex.cheap_cost(c,p,np.array([g['controls'] for g in eligible]),np.array([g['trajectory'] for g in eligible]))
    if rule=='endpoint':
        return np.sum((np.array([g['trajectory'][-1,:2] for g in eligible])-target)**2,axis=1)
    if rule=='free_area':return -np.array([g['area_m2'] for g in eligible])
    if rule=='novelty':return -np.array([g['nominal_rms_m'] for g in eligible])
    raise ValueError(rule)


def choose(eligible,values):
    chosen=[];decisions=[]
    for i in sorted(range(len(eligible)),key=lambda i:(values[i],eligible[i]['id'])):
        g=eligible[i]
        duplicate=any(np.sqrt(np.mean(np.sum((g['trajectory'][:,:2]-h['trajectory'][:,:2])**2,axis=1)))<.25 for h in chosen)
        reason='duplicate_selected' if duplicate else 'budget_limit' if len(chosen)>=3 else 'selected'
        if reason=='selected':chosen.append(g)
        decisions.append(dict(gap=g['gap'],score=float(values[i]),reason=reason))
    return sorted(chosen,key=lambda g:g['id']),decisions


def allocation(chosen,weighted=False):
    n=len(chosen)
    if not n:return [500]
    if not weighted:return [500-100*n]+[100]*n
    costs=np.array([g['cost'] for g in chosen]);tau=max(1.,.5*np.ptp(costs))
    weight=np.exp(-(costs-costs.min())/tau);weight/=weight.sum()
    raw=50*n*weight;extra=np.floor(raw).astype(int)
    ids=np.argsort(-(raw-extra),kind='stable');extra[ids[:50*n-int(extra.sum())]]+=1
    counts=[500-100*n]+(50+extra).tolist()
    assert sum(counts)==500 and min(counts[1:])>=50
    return counts


def run(p,cfg,nominal,chosen,counts,predictions,seed):
    names=['nominal']+[g['id'] for g in chosen]
    started=time.perf_counter()
    c=ex.MppiController(replace(cfg,seed=seed,guide_timeout=10.,strategy='standard',
        guide_angles_deg=tuple(float(i) for i in range(len(names)))))
    c.config.strategy='guide_conditioned'
    c._guide_optimizer=ex.SnapshotGapOptimizer(c,np.array([nominal]+[g['controls'] for g in chosen]),names,counts)
    setup=(time.perf_counter()-started)*1000
    try:
        started=time.perf_counter()
        r=c.compute(np.zeros(3),p['path'],p['current'],predictions,visualization_rollouts=0,distance_field=p['field'])
        ms=(time.perf_counter()-started)*1000
        assert len(r.sampled_controls)==500
        np.testing.assert_array_equal(r.control_mode_ids,np.repeat(np.arange(len(counts)),counts))
        safe=r.safety_status=='ok'
        if safe:
            assert r.max_collision_probability<=cfg.dynamic_risk.hard_threshold+1e-12
            assert not p['field'].trajectory_collisions(r.trajectory[None,:,:2],np.zeros(2),ex.RADIUS)[0]
        row=dict(cost=r.cost,status=r.safety_status,safe=safe,moving=safe and r.selected_mode!='stop',
            selected=r.selected_mode,progress_m=float(ex.path_progress(p,r.trajectory[None])[0]),
            max_probability=r.max_collision_probability,compute_ms=ms,setup_ms=setup,
            final_costs=r.mode_final_costs,candidate_names=r.candidate_names,
            finite_fraction=float(np.mean(np.isfinite(r.rollout_costs))))
        return row,r.trajectory.copy()
    finally:c.close()


def main():
    OUT.mkdir(exist_ok=True);(OUT/'scenes').mkdir(exist_ok=True)
    audit=[json.loads(s) for s in (BASE/'scene_audit.jsonl').read_text().splitlines()]
    accepted={r['record_ns']:r for r in audit if r['accepted']}
    reference={r['scene']:r for r in json.loads((BASE/'midpoint_value_v1/results.json').read_text())}
    cfg=ex.config_from_yaml()
    ex.write_json(OUT/'protocol.json',dict(
        purpose='Compare simple ranking proxies against existing common+static seed cost; separately test cost-weighted counts.',
        scene_count=len(accepted),noise_seeds=NOISE_SEEDS,methods=METHODS,
        eligibility=mid.CONFIG,partition=partitioner.CONFIG,
        scores=dict(common='existing common_costs + static soft cost',
            endpoint='squared terminal distance to global-path point about 1m ahead',
            free_area='negative integral .5*d_free(theta)^2 dtheta over gap',
            novelty='negative same-time positional RMS to nominal',
            common_weighted='same common ranking; 50 min each plus softmax weighted remainder'),
        fixed='one midpoint seed/gap (full circle path-bearing exception), same feasibility/nominal/guide duplicate gates, max3, total500, 2 workers, existing MC/final recheck',
        allocation='first four methods: nominal500-100g, guides100 each; fifth: nominal unchanged, guides50+weighted(50g)',
        cache='identical seed IDs/counts/noise seed reuse one exact solver result; reused rows are not independent runtime measurements',
        runtime='offline 10s timeout, warm scoring timer and solver compute only; no ROS full-stack timing',
        limitations='same three bags; prior ten development scenes flagged; not independent closed-loop validation',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    total_unique=0;scene_count=0
    with (OUT/'trials.jsonl').open('w') as trials,(OUT/'scene_audit.jsonl').open('w') as audit_out:
        for bag in ex.BAGS:
            print('READ',bag,flush=True);data=ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
            for snap in data['snapshots']:
                if snap['record_ns'] not in accepted:continue
                src=accepted[snap['record_ns']];name=src['scene'];p=ex.prepare_snapshot(data,snap);ex.input_check(data,snap,p)
                with np.load(BASE/'scenes'/f'{name}.npz') as old:
                    for key in ['grid','d_free','full_path']:np.testing.assert_allclose(p[key],old[key],atol=1e-10)
                parts,depth=arrays_and_partition(p)
                c=ex.MppiController(replace(cfg,strategy='standard'))
                try:
                    items,nominal,nominal_path,nominal_collision,target=build_pool(p,c,parts,depth)
                    eligible=[g for g in items if g['reason']=='eligible']
                    methods={};times={}
                    for rule in RULES:
                        values=score(rule,eligible,c,p,target)
                        elapsed=[]
                        for _ in range(10):
                            tick=time.perf_counter();score(rule,eligible,c,p,target);elapsed.append((time.perf_counter()-tick)*1000)
                        times[rule]=dict(median_ms=float(np.median(elapsed)),samples_ms=elapsed)
                        chosen,decisions=choose(eligible,values)
                        methods[rule]=dict(chosen=chosen,counts=allocation(chosen),decisions=decisions)
                    chosen=methods['common']['chosen']
                    methods['common_weighted']=dict(chosen=chosen,counts=allocation(chosen,True),decisions=methods['common']['decisions'])
                    if name in reference:
                        ref=reference[name]
                        assert len(items)==len(ref['items'])
                        np.testing.assert_allclose([g['cost'] for g in items],[np.inf if g['cost'] is None else g['cost'] for g in ref['items']],rtol=1e-9,atol=1e-9)
                        assert [g['gap'] for g in chosen]==sorted(ref['selected'])
                    predictions=ex.recorded_predictions(p,c)
                finally:c.close()
                public=dict(scene=name,bag=bag,offset=src['offset'],development_ten=name in reference,
                    region_count=len(items),eligible_count=len(eligible),nominal_collision=nominal_collision,
                    gap_fallback=parts['fallback'],target_point=target,score_timing=times,
                    candidates=[{k:v for k,v in g.items() if k not in ('controls','trajectory')} for g in items],
                    selections={method:dict(ids=[g['id'] for g in m['chosen']],counts=m['counts'],decisions=m['decisions']) for method,m in methods.items()})
                audit_out.write(json.dumps(ex.serial(public),ensure_ascii=False)+'\n');audit_out.flush()
                cache={};paths={}
                # Rotate execution order across snapshots to reduce a fixed timing-order bias.
                order=METHODS[scene_count%len(METHODS):]+METHODS[:scene_count%len(METHODS)]
                for seed in NOISE_SEEDS:
                    for method in order:
                        m=methods[method];key=(tuple(g['id'] for g in m['chosen']),tuple(m['counts']),seed)
                        reused=key in cache
                        if not reused:
                            output,path=run(p,cfg,nominal,m['chosen'],m['counts'],predictions,seed)
                            cache[key]=(output,path,method);total_unique+=1
                        output,path,source=cache[key]
                        row=dict(scene=name,method=method,seed=seed,development_ten=name in reference,
                            guide_ids=list(key[0]),counts=m['counts'],reused_exact_result=reused,source_method=source,**output)
                        trials.write(json.dumps(ex.serial(row),ensure_ascii=False)+'\n');trials.flush()
                        paths[f'{method}_{seed}']=path
                np.savez_compressed(OUT/'scenes'/f'{name}.npz',nominal=nominal_path,**paths)
                scene_count+=1
                print(f'{scene_count}/127 {name}: regions={len(items)} eligible={len(eligible)} unique solver calls={len(cache)} total={total_unique}',flush=True)
    ex.write_json(OUT/'run_summary.json',dict(scenes=scene_count,logical_trials=scene_count*len(METHODS)*len(NOISE_SEEDS),
                                           unique_solver_calls=total_unique))
    print('DONE',OUT,flush=True)


if __name__=='__main__':main()

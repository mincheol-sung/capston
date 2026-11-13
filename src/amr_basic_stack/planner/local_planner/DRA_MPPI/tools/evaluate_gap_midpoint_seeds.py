"""Evaluate one midpoint seed per actual disparity region, without MPPI.

Source ROS/workspace before execution. Only reads bags; no robot commands.
"""
from dataclasses import replace
from pathlib import Path
import csv
import hashlib
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_multiscene_2026-11-04'
PARTITION=BASE/'disparity_partition_10_scenes'
OUT=BASE/'midpoint_value_v1'
sys.path.insert(0,str(BASE/'source_snapshot/tools'))
import experiment_gap_multiscene as ex

CONFIG=dict(version='disparity_midpoint_value_v1',
    status='Initial development policy, evaluated on 10 snapshots; no production integration or MPPI validation',
    seed_per_gap=1,midpoint='ids[len(ids)//2] on circular ordered bins',
    full_circle='No unique midpoint; use bearing to path point approximately 1m along remaining path',
    minimum_forward_displacement_m=.25,maximum_target_error_m=.35,
    nominal_duplicate_rms_lt_m=.22,nominal_duplicate_tail_lt_m=.30,
    disable_nominal_dedup_if_nominal_static_collision=True,
    selected_duplicate_rms_lt_m=.25,maximum_extra_guides=3,
    cost='Frozen existing common_costs + static soft cost',
    require_improvement_over_nominal=False,additional_dynamic_mc=False,
    dynamic_safety='Original MPPI rollout and common final MC; not run in this experiment',
    rollout_budget=dict(total=500,per_guide=100,nominal='500-100*g'))


def classify(g,nominal_collision):
    if g['collision']: return 'static_collision'
    if g['target_error_m'] > CONFIG['maximum_target_error_m']: return 'target_error'
    if g['projected_displacement_m'] < CONFIG['minimum_forward_displacement_m']: return 'insufficient_motion'
    if not nominal_collision and g['nominal_rms_m'] < CONFIG['nominal_duplicate_rms_lt_m'] and g['nominal_tail_m'] < CONFIG['nominal_duplicate_tail_lt_m']:
        return 'duplicate_nominal'
    if not np.isfinite(g['cost']): return 'nonfinite_cost'
    return 'eligible'


def evaluate(p,parts):
    cfg=ex.config_from_yaml();controller=ex.MppiController(replace(cfg,strategy='standard'))
    controller.gap_path=p['path'];items=[];traces=[]
    try:
        nominal=controller._constrain_controls(p['nominal'][None],p['current'])[0]
        nominal_path=ex.rollout(np.zeros(3),nominal,.2)
        nominal_collision=bool(p['field'].trajectory_collisions(nominal_path[None,:,:2],np.zeros(2),ex.RADIUS)[0])
        nominal_cost=float(ex.cheap_cost(controller,p,nominal[None],nominal_path[None])[0])
        with np.load(PARTITION/'data'/f'{parts["scene"]}.npz') as cached:
            np.testing.assert_allclose(p['grid'],cached['grid'])
            np.testing.assert_allclose(p['d_free'],cached['strict_depth'],atol=1e-10)
            depth=cached['hard_depth' if parts['fallback'] else 'strict_depth'].copy()
        for region in parts['regions']:
            ids=np.array(region['bins']);index=int(ids[len(ids)//2]);kind='angular_midpoint'
            if len(ids)==360:
                arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p['path'],axis=0),axis=1))]
                point=p['path'][int(np.argmin(abs(arc-min(1.,arc[-1]))))]
                desired=np.arctan2(point[1],point[0])
                index=int(np.argmin(abs(np.arctan2(np.sin(p['angles']-desired),np.cos(p['angles']-desired)))))
                kind='full_circle_path_bearing'
            angle=p['angles'][index];unit=np.array([np.cos(angle),np.sin(angle)])
            available=max(.1,float(depth[index])-.1)
            far=ex.seed_control(controller,available*unit,p['current'])
            projected_far=float(ex.rollout(np.zeros(3),far,.2)[-1,:2]@unit)
            length=min(available,max(.25,.9*projected_far));target=length*unit
            controls=ex.seed_control(controller,target,p['current'])
            trajectory=ex.rollout(np.zeros(3),controls,.2)
            delta=np.linalg.norm(trajectory[:,:2]-nominal_path[:,:2],axis=1)
            weighted=ex.common_costs(cfg,np.zeros(3),p['path'],controls[None],trajectory[None],p['current'],return_components=True)[-1]
            static,_=controller._static_obstacle_cost(np.zeros(3),trajectory[None],p['field'],None)
            components={k:float(v[0]) for k,v in weighted.items()};components['static']=float(static[0])
            g=dict(gap=region['id'],angle_deg=float(np.rad2deg(angle)),direction_kind=kind,
                depth_m=float(depth[index]),target_distance_m=length,target_xy=target.tolist(),
                target_error_m=float(np.linalg.norm(trajectory[-1,:2]-target)),
                projected_displacement_m=float(trajectory[-1,:2]@unit),
                displacement_m=float(np.linalg.norm(trajectory[-1,:2])),
                nominal_rms_m=float(np.sqrt(np.mean(delta**2))),nominal_tail_m=float(np.mean(delta[-3:])),
                collision=bool(p['field'].trajectory_collisions(trajectory[None,:,:2],np.zeros(2),ex.RADIUS)[0]),
                cost=sum(components.values()),components=components,
                controls=controls,trajectory=trajectory)
            g['reason']=classify(g,nominal_collision);items.append(g)
        chosen=[]
        for g in sorted(items,key=lambda x:(x['cost'],x['gap'])):
            if g['reason']!='eligible':continue
            differences=[dict(gap=h['gap'],rms_m=float(np.sqrt(np.mean(np.sum((g['trajectory'][:,:2]-h['trajectory'][:,:2])**2,axis=1))))) for h in chosen]
            if any(d['rms_m']<CONFIG['selected_duplicate_rms_lt_m'] for d in differences):g['reason']='duplicate_selected'
            elif len(chosen)>=CONFIG['maximum_extra_guides']:g['reason']='budget_limit'
            else:g['reason']='selected';chosen.append(g)
            traces.append(dict(gap=g['gap'],cost=g['cost'],differences=differences,reason=g['reason']))
        return dict(nominal_collision=nominal_collision,nominal_cost=nominal_cost,
                    nominal_controls=nominal,nominal_path=nominal_path,items=items,
                    selected=[g['gap'] for g in chosen],selection_trace=traces,
                    planned_rollout_counts=[500-100*len(chosen)]+[100]*len(chosen))
    finally:controller.close()


def main():
    OUT.mkdir(exist_ok=True)
    config={**CONFIG,'source_partition_sha256':hashlib.sha256((PARTITION/'results.json').read_bytes()).hexdigest(),
            'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    ex.write_json(OUT/'configuration.json',config)
    scenes=json.loads((PARTITION/'configuration.json').read_text())['scenes']
    partitions={r['scene']:r for r in json.loads((PARTITION/'results.json').read_text())}
    records={s['record_ns']:s for s in scenes};results=[]
    for bag in dict.fromkeys(s['bag'] for s in scenes):
        print('READ',bag,flush=True);data=ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
        for snapshot in data['snapshots']:
            if snapshot['record_ns'] not in records:continue
            scene=records[snapshot['record_ns']];p=ex.prepare_snapshot(data,snapshot);ex.input_check(data,snapshot,p)
            result=evaluate(p,partitions[scene['scene']])
            np.savez_compressed(OUT/f'{scene["scene"]}_seeds.npz',nominal_controls=result.pop('nominal_controls'),
                nominal_path=result.pop('nominal_path'),
                controls=np.array([g['controls'] for g in result['items']]),
                paths=np.array([g['trajectory'] for g in result['items']]))
            for g in result['items']:g.pop('controls');g.pop('trajectory')
            result.update(**scene);results.append(result)
            print(scene['scene'],[(g['gap'],round(g['angle_deg']),g['reason']) for g in result['items']],flush=True)
    results.sort(key=lambda r:r['scene']);ex.write_json(OUT/'results.json',results)
    cols=['scene','gap','angle_deg','direction_kind','collision','target_error_m','projected_displacement_m',
          'nominal_rms_m','nominal_tail_m','cost','reason']
    with (OUT/'seed_decisions.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=cols);w.writeheader()
        for r in results:
            for g in r['items']:w.writerow({'scene':r['scene'],**{k:g[k] for k in cols[1:]}})
    print('DONE',OUT,flush=True)


if __name__=='__main__':main()

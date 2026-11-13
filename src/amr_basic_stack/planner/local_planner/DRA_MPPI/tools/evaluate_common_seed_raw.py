"""Actually sample around all ten common-objective seeds, even invalid centres.

This follow-up does not filter the centres by static collision or novelty.
Original rollout static costs and dynamic MC/final safety gates stay enabled.
"""
from dataclasses import replace
from pathlib import Path
import sys
import json
import hashlib
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_multiscene_2026-11-04'
OUT=BASE/'common_objective_seed_10_scenes'
sys.path.insert(0,str(BASE/'source_snapshot/tools'))
import experiment_gap_multiscene as ex
from experiment_gap_diversity import options,hard_depth


def load(path):return [json.loads(s) for s in path.read_text().splitlines()]


def main():
    rows=load(OUT/'scene_results.jsonl');scenes={r['scene'] for r in rows}
    accepted={a['record_ns']:a for a in load(BASE/'scene_audit.jsonl') if a['accepted'] and a['scene'] in scenes}
    ex.write_json(OUT/'raw_seed_protocol.json',dict(
        motivation='Inspecting the post-hoc collision gate alone cannot tell whether an unsafe centre can produce useful safe MPPI samples.',
        method='common_extra_raw: append the common-only numerical optimum as an additional sampling centre in all10 scenes, irrespective of centre collision/nominal duplication. Keep up to2 best existing gap guides; total extra guides<=3, nominal gets500-100*extra_count. All original rollout and final safety checks remain enabled.',
        scope='Same prespecified10 scenes and precomputed optimizers. No retuning. Two noise seeds7/23. Exploratory same-data comparison.',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    cfg=ex.config_from_yaml();count=0
    with (OUT/'raw_seed_trials.jsonl').open('w') as tf:
        for bag in ex.BAGS:
            print('READ',bag,flush=True);data=ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
            for snap in data['snapshots']:
                if snap['record_ns'] not in accepted:continue
                audit=accepted[snap['record_ns']];scene=audit['scene'];p=ex.prepare_snapshot(data,snap)
                ex.input_check(data,snap,p)
                c=ex.MppiController(replace(cfg,strategy='standard'));c.gap_path=p['path']
                predictions=ex.recorded_predictions(p,c)
                source=OUT/'scenes'/f'{scene}.npz'
                with np.load(source) as cached:arrays={k:cached[k] for k in cached.files}
                nominal=c._constrain_controls(p['nominal'][None],p['current'])[0]
                np.testing.assert_allclose(nominal,arrays['nominal_controls'],atol=1e-10)
                nompath=ex.rollout(np.zeros(3),nominal,.2);pp=p
                if not ex.definitions(p,'valley'):pp={**p,'d_free':hard_depth(p)[0]}
                pool,_=options(pp,c,nompath)
                _,chosen,_=ex.select(pp,pool,'cost',audit['nominal_collision'],audit['nominal_cost'] or np.inf,True)
                guides=chosen[:2]+[dict(id='zz_common_optimized',controls=arrays['common_controls'],trajectory=arrays['common_trajectory'])]
                for seed in ex.SEEDS:
                    r,paths=ex.run_optimizer(p,cfg,nominal,guides,predictions,'common_extra_raw',seed)
                    index=r['candidate_names'].index('zz_common_optimized')
                    tf.write(json.dumps(ex.serial(dict(scene=scene,seed=seed,method='common_extra_raw',seed_used=True,
                        common_group_final_cost=r['final_costs'][index],common_group_selected=r['selected']=='zz_common_optimized',**r)))+'\n');tf.flush()
                    if seed==7:
                        arrays['common_extra_raw_trajectory']=paths['trajectory']
                        arrays['common_extra_raw_representatives']=paths['representatives']
                np.savez_compressed(source,**arrays);c.close();count+=1
                print('SCENE',count,scene,flush=True)
    assert count==10


if __name__=='__main__':main()

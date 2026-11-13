"""Append one extra objective to completed paired trials without rerunning them."""
import json
import time
from dataclasses import replace
import numpy as np

from experiment_gap_objective_ablation import (
    OUT,SOURCE,POLICIES,METHODS,PreparedOptimizer,prepare,execute,write,serial,hashes,derivative_check)
from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.gap_geometry import GapConfig
from dra_mppi.mppi_controller import MppiController


def main():
    method='linearized_common'
    manifest=json.loads((OUT/'configuration.json').read_text())
    if method in manifest['methods']:raise RuntimeError('Already extended')
    assert hashes()==manifest['source_hashes']
    cfg=replace(config_from_yaml(),strategy='guide_conditioned',gap=GapConfig(**manifest['configuration']['gap']),
                guide_num_rollouts=500,guide_angles_deg=(0.,1.,2.,3.),guide_timeout=10.)
    runs=json.loads((OUT/'runs.json').read_text())
    with (OUT/'runs.jsonl').open('a')as log:
        for source in manifest['sources']:
            scene=source['scene'];data=dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
            record=json.loads((OUT/f'{scene}_seeds.json').read_text())
            generator=MppiController(cfg)
            try:prepared=prepare(data,cfg,generator,methods=(method,),frozen_names=record['names'])
            finally:generator.close()
            assert prepared['names']==record['names']
            for q in prepared['candidates']:
                old=next(c for c in record['candidates']if c['id']==q['id'])
                np.testing.assert_allclose([old['angle'],old['distance']],[q['angle'],q['distance']],atol=1e-9)
                if q['neutral_ok']:
                    np.testing.assert_allclose(old['neutral_controls'],q['neutral_controls'],atol=1e-9)
                    old['variants'][method]=q['variants'][method]
            record['proposals'][method]=prepared['proposals'][method]
            write(OUT/f'{scene}_seeds.json',record)
            for trial in range(manifest['trials']):
                tc=replace(cfg,seed=cfg.seed+trial*10000,
                    dynamic_risk=replace(cfg.dynamic_risk,seed=cfg.dynamic_risk.seed+trial*10000))
                c=MppiController(tc);c._guide_optimizer.shutdown();c._guide_optimizer=PreparedOptimizer(c)
                try:
                    policies=list(POLICIES);np.random.default_rng(314+trial).shuffle(policies)
                    for policy in policies:
                        p=prepared['proposals'][method];counts=p['counts'][policy]
                        tick=time.perf_counter();result=execute(c,prepared,p['controls'],prepared['names'],counts)
                        row=dict(scene=scene,trial=trial,method=method,policy=policy,cost=result.cost,
                            status=result.safety_status,selected=result.selected_mode,counts=counts,names=prepared['names'],
                            final_costs=result.mode_final_costs,probability=result.max_collision_probability,
                            invalid_rollouts=int(np.sum(~np.isfinite(result.rollout_costs))),
                            mppi_ms=(time.perf_counter()-tick)*1000,selected_components=result.selected_cost_weighted)
                        runs.append(row);log.write(json.dumps(serial(row),ensure_ascii=False,allow_nan=False)+'\n');log.flush()
                        if trial==0:
                            np.savez_compressed(OUT/'arrays'/f'{scene}_{method}_{policy}.npz',
                                sampled_controls=result.sampled_controls,rollout_costs=result.rollout_costs,
                                group_ids=result.control_mode_ids,nominal=prepared['nominal'],seeds=p['controls'],
                                representatives=result.mode_trajectories,final=result.trajectory,
                                final_controls=result.control_sequence,counts=counts)
                finally:c.close()
            print(scene,method,'done',flush=True)
    assert hashes()==manifest['source_hashes']
    manifest['methods']=METHODS
    manifest['derivative_errors']=derivative_check(cfg)
    manifest['extra_objective']='linearized_common appended after first five-objective experiment. One Gauss-Newton quadratic of exact common residuals at the shared neutral seed; no static/dynamic cost. Same endpoint, initial controls, frozen groups and paired RNG. Other policies not rerun. Approximation is evaluated using the unchanged actual 500-rollout MPPI, not by its own predicted objective.'
    write(OUT/'configuration.json',manifest);write(OUT/'runs.json',runs)
    print('Total MPPI calls',len(runs),flush=True)


if __name__=='__main__':main()

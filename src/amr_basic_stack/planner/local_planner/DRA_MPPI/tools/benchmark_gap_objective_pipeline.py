"""Actual end-to-end offline timing, one seed objective per planning call.

Uses frozen scene group membership from the paired experiment. Recomputes map,
LiDAR, free/limited reach, seed optimization, allocation and existing 500-rollout
MPPI. No cache, no short wall deadline, no ROS. Does not certify runtime 5 Hz.
"""
import argparse
from dataclasses import replace
import json
import time
import numpy as np

from experiment_gap_objective_ablation import (
    OUT,SOURCE,METHODS,PreparedOptimizer,prepare,execute,write,hashes)
from benchmark_guide_conditioned import config_from_yaml,summary
from dra_mppi.gap_geometry import GapConfig
from dra_mppi.mppi_controller import MppiController


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--trials',type=int,default=3)
    parser.add_argument('--method',choices=METHODS)
    args=parser.parse_args()
    manifest=json.loads((OUT/'configuration.json').read_text())
    cfg=replace(config_from_yaml(),strategy='guide_conditioned',gap=GapConfig(**manifest['configuration']['gap']),
                guide_num_rollouts=500,guide_angles_deg=(0.,1.,2.,3.),guide_timeout=10.)
    locked=hashes();rows=[]
    if args.method and (OUT/'pipeline_timing.json').exists():
        rows=[r for r in json.loads((OUT/'pipeline_timing.json').read_text())['rows']if r['method']!=args.method]
    # Persistent samplers/workers; every scene state/nominal and RNG reset.
    c=MppiController(cfg);c._guide_optimizer.shutdown();c._guide_optimizer=PreparedOptimizer(c)
    try:
        for source in manifest['sources']:
            scene=source['scene'];data=dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
            fixed=json.loads((OUT/f'{scene}_seeds.json').read_text())['names']
            for trial in range(args.trials):
                methods=[args.method]if args.method else list(METHODS)
                np.random.default_rng(790+trial).shuffle(methods)
                for method in methods:
                    tick=time.perf_counter()
                    prepared=prepare(data,cfg,c,methods=(method,),frozen_names=fixed)
                    generated=time.perf_counter()
                    p=prepared['proposals'][method]
                    result=execute(c,prepared,p['controls'],prepared['names'],p['counts']['native'])
                    done=time.perf_counter()
                    rows.append(dict(scene=scene,method=method,trial=trial,
                        total_ms=(done-tick)*1000,proposal_ms=(generated-tick)*1000,
                        mppi_ms=(done-generated)*1000,status=result.safety_status,cost=result.cost,
                        seed_fallbacks=sum(not q['variants'][method]['accepted']for q in prepared['candidates']if method in q['variants'])))
            print(scene,'full offline pipeline done',flush=True)
    finally:c.close()
    assert hashes()==locked
    stats={m:dict(calls=len(rr:=[r for r in rows if r['method']==m]),
        total_ms=summary([r['total_ms']for r in rr]),proposal_ms=summary([r['proposal_ms']for r in rr]),
        over_180_ms=sum(r['total_ms']>180 for r in rr),over_200_ms=sum(r['total_ms']>200 for r in rr))for m in METHODS}
    write(OUT/'pipeline_timing.json',dict(scope=__doc__,summary=stats,rows=rows,source_hashes=locked))
    print(json.dumps(stats,indent=2),flush=True)


if __name__=='__main__':main()

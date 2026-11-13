#!/usr/bin/env python3
"""500x10x3 independent Gaussian/Halton versus existing cubic sampling."""
import os
for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
import argparse
import json
from pathlib import Path
import time
import numpy as np
from scipy.special import ndtri
from scipy.stats import qmc
from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.control_sampling import HaltonSplineSampler
from dra_mppi.mecanum_model import rollout


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cfg=config_from_yaml();std=np.array(cfg.guide_noise_std)
    rng=np.random.default_rng(cfg.seed)
    halton=qmc.Halton(30,scramble=True,seed=cfg.seed)
    spline=HaltonSplineSampler(10,knot_points=4,noise_std=std,seed=cfg.seed)
    def gaussian():
        return rng.standard_normal((500,10,3))*std
    def direct_halton():
        return ndtri(np.clip(halton.random(500),1e-9,1-1e-9)).reshape(500,10,3)*std
    methods={'independent_gaussian':gaussian,'direct_halton_30d':direct_halton,
             'existing_cubic_halton_12d':lambda:spline.sample(500)}
    def summarize(raw):
        return dict(raw_ms=raw,mean_ms=float(np.mean(raw)),median_ms=float(np.median(raw)),
                    p95_ms=float(np.percentile(raw,95)))
    raw={key:[] for key in methods}
    for fn in methods.values():
        for _ in range(20):
            out=fn()
            assert out.shape==(500,10,3) and np.all(np.isfinite(out))
    for repeat in range(1000):
        keys=list(methods);keys=keys[repeat%3:]+keys[:repeat%3]
        for key in keys:
            start=time.perf_counter();methods[key]()
            raw[key].append((time.perf_counter()-start)*1000)
    report=dict(shape=[500,10,3],seed=cfg.seed,noise_std=std.tolist(),threads=1,
        repeats=1000,warmup=20,sampling={k:summarize(v) for k,v in raw.items()})
    def measure(fn,repeats):
        fn();times=[]
        for _ in range(repeats):
            start=time.perf_counter();fn();times.append((time.perf_counter()-start)*1000)
        return summarize(times)
    # Current guide code creates FIVE arrays of 498 during construction, not each cycle.
    def init_bank():
        return [HaltonSplineSampler(10,knot_points=4,noise_std=std,seed=cfg.seed+101+g).sample(498)
                for g in range(5)]
    report['current_five_bank_initialization']=measure(init_bank,50)
    bank=init_bank()
    def fetch():
        center=np.zeros((10,3));controls=np.empty((500,10,3))
        for g in range(5):
            block=controls[g*100:(g+1)*100]
            block[:2]=center
            block[2:]=center+bank[g][:98]
        return controls
    report['cached_fetch_and_center_add']=measure(fetch,1000)
    controls=spline.sample(500)
    report['rollout_only_fixed_controls']=measure(lambda:rollout(np.zeros(3),controls,.2),1000)
    report['cache_noise_values']=5*498*10*3
    report['cache_noise_bytes']=sum(b.nbytes for b in bank)
    (args.output/'timing.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:({n:{s:v for s,v in d.items() if s!='raw_ms'} for n,d in value.items()}
        if k=='sampling' else {s:v for s,v in value.items() if s!='raw_ms'} if isinstance(value,dict) else value)
        for k,value in report.items()},indent=2))


if __name__=='__main__':
    main()

#!/usr/bin/env python3
"""Measured MC accuracy/runtime tradeoff; never modifies production YAML."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np

from benchmark_guide_conditioned import config_from_yaml
from validate_current_mc import point_suite, candidate_suite, stats
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiController


def budget(cfg, multiplier):
    changes = {}
    for prefix in ('mc', 'guide_mc', 'final_mc'):
        for suffix in ('sample_density', 'samples_min', 'samples_max'):
            key = prefix+'_'+suffix
            changes[key] = getattr(cfg.dynamic_risk, key)*multiplier
    return replace(cfg, guide_timeout=10., dynamic_risk=replace(cfg.dynamic_risk, **changes))


def main():
    p = argparse.ArgumentParser();p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stage', choices=('accuracy','timing'), required=True)
    p.add_argument('--multipliers', type=int, nargs='+', default=[1,2,4,8,16,32])
    a=p.parse_args();a.output.mkdir(parents=True, exist_ok=True);base=config_from_yaml();reports=[]
    for mult in a.multipliers:
        cfg=budget(base,mult);started=time.perf_counter()
        if a.stage=='accuracy':
            point=point_suite(cfg,8);actual,work=candidate_suite(cfg,4)
            row=dict(multiplier=mult,density=cfg.dynamic_risk.guide_mc_sample_density,
                point={},actual={},near={},point_span={},work=work)
            for profile in ('guide','final'):
                group=[r for r in actual if r['profile']==profile]
                row['actual'][profile]=stats(group)
                row['near'][profile]=stats([r for r in group if .03<=r['reference']<=.1])
                row['point'][profile]=stats([r for r in point if r['profile']==profile and r['underresolved']==0])
                row['point_span'][profile]=[{**stats([r for r in point if r['profile']==profile and r['span']==span]),'span':span,
                    'underresolved':sum(r['underresolved'] for r in point if r['profile']==profile and r['span']==span)} for span in (0.,.25,.5,1.,2.)]
            from validate_current_mc import write_csv
            write_csv(a.output/f'accuracy_points_x{mult}.csv',point)
            write_csv(a.output/f'accuracy_candidates_x{mult}.csv',actual)
        else:
            h=cfg.horizon_steps;t=cfg.dt*np.arange(1,h+1)
            field=DistanceField.from_occupancy(np.zeros((200,200)),GridMetadata(200,200,.05,-5.,-5.,0.))
            timing=[]
            for o in (5,10,20):
                means=np.zeros((o,2,h,2));means[...,0]=np.linspace(.8,3.,o)[:,None,None]+.1*t
                means[:,0,:,1]=np.linspace(-1.4,1.4,o)[:,None]+.1*t
                means[:,1,:,1]=means[:,0,:,1]+.2
                pred=PredictionSet(means,np.broadcast_to(np.array([[.05,.025],[.025,.025]]),(o,2,h,2,2)),t,np.tile([.6,.4],(o,1)))
                c=MppiController(cfg);raw=[]
                try:
                    for i in range(7):
                        c.reset();tick=time.perf_counter()
                        r=c.compute(np.zeros(3),np.array([[0.,0.],[3.,0.]]),np.zeros(3),pred,distance_field=field,visualization_rollouts=0)
                        if i>=2:raw.append((time.perf_counter()-tick)*1000)
                    d=c.risk_diagnostics
                    timing.append(dict(obstacles=o,K=cfg.guide_num_rollouts,mean_ms=float(np.mean(raw)),max_ms=max(raw),raw_ms=raw,
                        primary_samples=d.total_samples,final_samples=int(np.sum(r.final_mc_sample_counts)),
                        candidate_sample_comparisons=d.candidate_sample_comparisons,
                        primary_node_ms=r.stage_ms['primary_wall'],final_ms=c.representative_time_ms,
                        pdf_evaluations_primary=d.total_samples*o*2))
                finally:c.close()
            row=dict(multiplier=mult,density=cfg.dynamic_risk.guide_mc_sample_density,timing=timing)
        row['seconds']=time.perf_counter()-started;reports.append(row)
        (a.output/(a.stage+'.json')).write_text(json.dumps(reports,indent=2))
        print(a.stage,'x',mult,'seconds',round(row['seconds'],1),flush=True)


if __name__=='__main__':main()

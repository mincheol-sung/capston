"""Compare saved real-bag seeds under gap and existing non-dynamic objectives.

Offline analysis only. Production behavior/configuration is not changed.
Direct solves below use batched finite differences for the existing objective;
their timing is not a bound on an implementation with analytic derivatives.
"""
from dataclasses import asdict, replace
from pathlib import Path
import json
import time

import numpy as np

from benchmark_guide_conditioned import config_from_yaml, summary
from dra_mppi.common_cost import common_costs, project_path
from dra_mppi.gap_inputs import GapScan, fuse_scans
from dra_mppi.gap_optimization import ValueProblem, OptimizationDeadline, reference, score_controls
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiController

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/gap_value_optimization_2026-11-05'
OUT = ROOT / 'docs/gap_objective_comparison_2026-11-05'


def existing(cfg, path, current, controls, field, progress):
    trajectories = rollout(np.zeros(3), controls, cfg.dt)
    terms = common_costs(cfg, np.zeros(3), path, controls, trajectories, current,
                        progress, return_components=True)
    base = sum(terms[:4])
    clearance = field.state_clearances(trajectories[..., :2])-cfg.static_cost_robot_radius
    static = cfg.static_cost_soft_weight*np.mean(
        np.maximum(0., 1.-clearance/cfg.static_cost_decay)**2, axis=1)
    return base, static, terms[-1]


class DirectProblem(ValueProblem):
    def __init__(self, cfg, current, path, angle, depth, common, field, progress, with_static):
        super().__init__(cfg, current, path, angle, depth, common)
        self.path = path; self.field = field; self.progress = progress
        self.with_static = with_static

    def objective(self, x):
        self.check_deadline()
        epsilon = 1e-6
        batch = np.vstack((x, x+epsilon*np.eye(len(x)))).reshape(-1, self.n, 3)
        base, static, weighted = existing(self.cfg, self.path, self.current, batch,
                                          self.field, self.progress)
        values = base+static if self.with_static else base
        parts = {k:float(v[0]) for k,v in weighted.items()}
        if self.with_static: parts['static'] = float(static[0])
        return float(values[0]), (values[1:]-values[0])/epsilon, parts


def main():
    cfg = replace(config_from_yaml(), strategy='standard')
    controller = MppiController(cfg)
    rows = []; evaluation_ms = []; simple_ms = []; solves = []
    try:
        sources = json.loads((SOURCE/'raw_sources.json').read_text())
        for source in sources:
            scene = source['scene']
            data = dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
            saved = dict(np.load(SOURCE/f'{scene}_results.npz'))
            metadata = json.loads((SOURCE/f'{scene}_summary.json').read_text())
            field = DistanceField.from_occupancy(data['grid'], GridMetadata(200,200,.04,-4.,-4.,0.))
            scans = tuple(GapScan(*(data[f'scan_{i}_{k}'] for k in ('ranges','valid','origins','hits'))) for i in range(2))
            field, _, _ = fuse_scans(scans, np.zeros(3), field, None, cfg.gap, cfg.static_cost_robot_radius)
            current = controller._project_wheel_limit(np.clip(data['current'],cfg.control_min,cfg.control_max)[None])[0]
            path = data['path']
            progress = float(project_path(np.zeros((1,1,2)),path,cfg.heading_lookahead,cfg.projection_window)[1][0,0])
            common = reference(path,cfg,progress)
            valid = [i for i,q in enumerate(metadata['candidates']) if q['success'] and not q['collision']]
            nominal = controller._constrain_controls(data['nominal'][None],current)[0]
            controls = np.asarray([nominal]+[saved['seed_controls'][i]for i in valid])
            labels = ['nominal']+[metadata['candidates'][i]['id']for i in valid]
            paths = rollout(np.zeros(3),controls,cfg.dt)
            nominal_direction = paths[0,-1,:2]
            if np.linalg.norm(nominal_direction)<1e-6: nominal_direction=common[0]
            nominal_direction = nominal_direction/np.linalg.norm(nominal_direction) if np.linalg.norm(nominal_direction)>1e-6 else np.array([1.,0.])
            directions = [nominal_direction]+[np.array([np.cos(np.deg2rad(metadata['candidates'][i]['angle_deg'])),np.sin(np.deg2rad(metadata['candidates'][i]['angle_deg']))])for i in valid]
            for repeat in range(12):
                tick=time.perf_counter()
                base,static,parts=existing(cfg,path,current,controls,field,progress)
                if repeat>=2:evaluation_ms.append((time.perf_counter()-tick)*1000)
                tick=time.perf_counter()
                small=[score_controls(cfg,current,common,e,u)[0]for e,u in zip(directions,controls)]
                if repeat>=2:simple_ms.append((time.perf_counter()-tick)*1000)
            collisions=field.trajectory_collisions(paths[...,:2],np.zeros(2),cfg.static_cost_robot_radius)
            ranking=lambda values:[labels[i] for i in sorted(range(1,len(labels)),key=lambda i:values[i])]
            row=dict(scene=scene,path_points=len(path),candidates=[dict(id=label,
                gap_cost=float(small[j]),existing_common=float(base[j]),static_soft=float(static[j]),
                existing_nodynamic=float(base[j]+static[j]),collision=bool(collisions[j]),
                existing_components={k:float(v[j])for k,v in parts.items()})for j,label in enumerate(labels)],
                rank_gap=ranking(small),rank_existing=ranking(base),rank_existing_static=ranking(base+static))
            rows.append(row)
            if valid:
                i=valid[0];q=metadata['candidates'][i]
                # Same saved endpoint and warm start: compare only value-stage
                # solves. Reach solving, geometry and MPPI are not timed here.
                start=saved['seed_controls'][i];angle=np.deg2rad(q['angle_deg'])
                for method in ('current_analytic','existing_numeric','existing_static_numeric'):
                    problem=(ValueProblem(cfg,current,path,angle,q['depth'],common) if method=='current_analytic'
                        else DirectProblem(cfg,current,path,angle,q['depth'],common,field,progress,method=='existing_static_numeric'))
                    problem.target_distance=q['target_distance']
                    tick=time.perf_counter()
                    try:
                        result=problem.solve(start,deadline=tick+.25)
                        solve=dict(scene=scene,candidate=q['id'],method=method,ms=(time.perf_counter()-tick)*1000,
                            success=result['success'],violation=result['violation'],iterations=result['iterations'],
                            objective=result['cost'])
                    except OptimizationDeadline:
                        solve=dict(scene=scene,candidate=q['id'],method=method,ms=(time.perf_counter()-tick)*1000,
                                   success=False,deadline_exceeded=True)
                    solves.append(solve)
            print(scene, row['rank_gap'], '->',row['rank_existing_static'],flush=True)
    finally:controller.close()
    multi=[r for r in rows if len(r['rank_gap'])>=2]
    stats=dict(scenes=len(rows),multi_candidate_scenes=len(multi),
        different_gap_order=sum(r['rank_gap']!=r['rank_existing']for r in multi),
        different_best_gap=sum(r['rank_gap'][0]!=r['rank_existing'][0]for r in multi),
        different_gap_order_with_static=sum(r['rank_gap']!=r['rank_existing_static']for r in multi),
        current_score_batch_ms=summary(simple_ms),existing_score_batch_ms=summary(evaluation_ms),
        solves={method:dict(count=len(group:=[s for s in solves if s['method']==method]),
            success=sum(s['success']for s in group),deadline_exceeded=sum(s.get('deadline_exceeded',False)for s in group),
            ms=summary([s['ms']for s in group]))for method in ('current_analytic','existing_numeric','existing_static_numeric')})
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'comparison.json').write_text(json.dumps(dict(scope='Saved real-bag seeds rescored under current gap objective and existing MPPI common objective plus static soft cost; no dynamic cost. Ranking compares the same feasible saved controls, not separately optimized global optima. Microbenchmarks exclude geometry/MPPI/ROS. Direct value solves use batched numeric gradients (30 controls) with a 250ms cooperative per-candidate cap, not the production 50ms aggregate proposal budget. Their timing does not apply to a future analytic implementation.',
        configuration=asdict(cfg),summary=stats,rows=rows,solves=solves),indent=2,allow_nan=False)+'\n')
    print(json.dumps(stats,indent=2))


if __name__=='__main__':main()

"""Read real bags, evaluate gap value proposals, and benchmark full MPPI kernel.

Offline only. Never starts a ROS node, publishes commands, or edits config.
"""
from dataclasses import asdict, replace
from pathlib import Path
import argparse
import json
import os
import platform
import time
import numpy as np

from render_lidar_gap_flow import read_bag, prepare_snapshot, recorded_predictions, config_from_yaml
from dra_mppi.mppi_controller import MppiController
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.gap_geometry import GapConfig, partition
from dra_mppi.gap_inputs import GapScan, fuse_scans, directional_depth
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.gap_conditioned import GapConditionedOptimizer
from dra_mppi.guide_conditioned import GuideConditionedOptimizer
from dra_mppi.mecanum_model import rollout
from gap_value_prototype import ValueProblem, ReachProblem, WEIGHTS, allocate, check_derivatives

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'docs/gap_multiscene_2026-11-04'
OUT = ROOT/'docs/gap_value_optimization_2026-11-05'
EXTRA = ['b0_001', 'b0_027', 'b1_030', 'b1_088', 'b2_060', 'b2_114']


def serial(x):
    if isinstance(x, dict): return {k:serial(v) for k,v in x.items()}
    if isinstance(x, (list, tuple)): return [serial(v) for v in x]
    if isinstance(x, np.ndarray): return serial(x.tolist())
    if isinstance(x, np.bool_): return bool(x)
    if isinstance(x, np.integer): return int(x)
    if isinstance(x, (float, np.floating)): return float(x) if np.isfinite(x) else None
    return x


def write(path, data):
    path.write_text(json.dumps(serial(data), indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def extract():
    """Read original MCAPs, not only the previously rendered cache."""
    references = json.loads((BASE/'disparity_partition_10_scenes/results.json').read_text())
    scenes = [r['scene'] for r in references]+EXTRA
    audits = {r['record_ns']:r for line in (BASE/'scene_audit.jsonl').read_text().splitlines()
              if (r:=json.loads(line)).get('scene') in scenes}
    (OUT/'raw').mkdir(parents=True, exist_ok=True)
    c = MppiController(replace(config_from_yaml(), strategy='standard'))
    records = []
    try:
        for bag in sorted({r['bag'] for r in audits.values()}):
            bagpath = Path('/home/smoc/2026amr_ws/bags')/bag
            print('READ ORIGINAL BAG', str(bagpath), flush=True)
            data = read_bag(bagpath)
            for snap in data['snapshots']:
                if snap['record_ns'] not in audits: continue
                audit = audits[snap['record_ns']]; scene = audit['scene']
                tick = time.perf_counter(); p = prepare_snapshot(data, snap)
                prepare_ms = (time.perf_counter()-tick)*1000
                tick = time.perf_counter(); predictions = recorded_predictions(p, c)
                align_ms = (time.perf_counter()-tick)*1000
                arrays = {k:p[k] for k in ['grid','scan_grid','map_fill','path','full_path','current','nominal']}
                for i, scan in enumerate(p['scans']):
                    arrays.update({f'scan_{i}_{k}':scan[k] for k in ['ranges','valid','origins','hits']})
                count = 0
                if predictions is not None and not predictions.empty:
                    count = len(predictions.means)
                    for key in ['means','covariances','times','mode_weights','obstacle_radii']:
                        value = getattr(predictions, key)
                        if value is not None: arrays['prediction_'+key] = value
                prior = BASE/f'disparity_partition_10_scenes/data/{scene}.npz'
                match = None
                if prior.exists():
                    old = dict(np.load(prior))
                    for key in ['grid']+[f'scan_{i}_{k}' for i in range(2) for k in ['ranges','valid','hits','origins']]:
                        np.testing.assert_allclose(arrays[key], old[key], atol=1e-8, equal_nan=True)
                    match = True
                np.savez_compressed(OUT/f'raw/{scene}.npz', **arrays)
                records.append(dict(scene=scene, bag=bag, offset=snap['elapsed'], record_ns=snap['record_ns'],
                                    epoch=p['epoch'], map_age=p['map_age'], reference_age=p['ref_age'],
                                    predictions=count, matches_prior_cache=match,
                                    raw_prepare_ms=prepare_ms, prediction_align_ms=align_ms,
                                    bag_files=[dict(name=f.name,size_bytes=f.stat().st_size) for f in bagpath.glob('*.mcap')]))
                print('EXTRACT',scene,'predictions',count,'cache_matches',match,flush=True)
            del data
    finally: c.close()
    assert len(records)==len(scenes)
    write(OUT/'raw_sources.json', records)


class PreparedOptimizer(GuideConditionedOptimizer):
    def __init__(self, controller):
        super().__init__(controller, angles=np.arange(4))
        self.risk_pool = self._guide_risk_estimators[:]
        self.noise_pool = self.noise[:]

    def set_proposals(self, controls, names, counts):
        self.count = len(controls); self.proposals = np.asarray(controls)
        self.names = tuple(names)+('stop',); self.counts = np.array(counts)
        self.nominals = np.zeros_like(self.proposals); self.initialized = np.zeros(self.count, bool)
        self.temperatures = np.full(self.count, self.cfg.temperature)
        self.previous = None; self.entry_yaw = None
        self._guide_risk_estimators = self.risk_pool[:self.count]; self.noise = self.noise_pool[:self.count]

    def anchors(self, *args, **kwargs): return self.proposals.copy()
    def allocate(self, active): return self.counts.copy()


def geometry(data, gap, radius):
    field = DistanceField.from_occupancy(data['grid'], GridMetadata(200,200,.04,-4.,-4.,0.))
    scans = tuple(GapScan(*(data[f'scan_{i}_{key}'] for key in ['ranges','valid','origins','hits'])) for i in range(2))
    return field, scans


def proposals(c, data, field, scans, budget_iter, fixed_target):
    started = time.perf_counter(); cfg = c.config; gap = cfg.gap
    fused, _, local = fuse_scans(scans, np.zeros(3), field, None, gap, cfg.static_cost_robot_radius)
    angles, strict, hard = directional_depth(fused, gap, cfg.static_cost_robot_radius)
    arrays = dict(strict_depth=strict, hard_depth=hard)
    for i, scan in enumerate(local): arrays.update({f'scan_{i}_{k}':scan[k] for k in ['ranges','valid','hits']})
    parts = partition(arrays, asdict(gap)); depth = hard if parts['fallback'] else strict
    geometry_ms = (time.perf_counter()-started)*1000
    tick = time.perf_counter()
    current = c._project_wheel_limit(np.clip(data['current'],cfg.control_min,cfg.control_max)[None])[0]
    nominal = c._constrain_controls(data['nominal'][None], current)[0]
    nominal_path = rollout(np.zeros(3), nominal, cfg.dt)
    nominal_collision = bool(fused.trajectory_collisions(nominal_path[None,:,:2],np.zeros(2),cfg.static_cost_robot_radius)[0])
    candidates = []
    for region in parts['regions']:
        ids = region['bins']
        indexes = [ids[len(ids)//2]] if len(ids)<360 else [0,90,180,270]
        for sub, j in enumerate(indexes):
            label = f'G{region["id"]}'+(f'.{sub+1}' if len(indexes)>1 else '')
            angle = angles[j]
            problem = ValueProblem(cfg, current, data['path'], angle, float(depth[j]))
            # Reuse the existing target follower only as the solver initial guess.
            target = min(depth[j]-.10, .8)*problem.e
            start = GapConditionedOptimizer._seed(c._guide_optimizer, np.zeros(3), data['path'], current, target)
            reach = None
            if fixed_target:
                reach_problem = ReachProblem(cfg, current, data['path'], angle, float(depth[j]))
                reach = reach_problem.solve(start, maxiter=budget_iter)
                problem.target_distance = max(.25, min(float(depth[j]), float(reach['trajectory'][-1,:2]@problem.e)))
                start = reach['controls']
            answer = problem.solve(start, maxiter=budget_iter)
            candidate = dict(id=label, region_id=region['id'], angle_deg=float(np.rad2deg(angle)),
                             depth=float(depth[j]), reference=problem.target.copy(), **answer)
            if reach is not None:
                candidate.update(target_distance=problem.target_distance,reach_success=reach['success'],
                                 reach_violation=reach['violation'],reach_iterations=reach['iterations'],reach_ms=reach['wall_ms'])
            trajectory = candidate['trajectory']
            candidate['distance'] = float(trajectory[-1,:2]@problem.e)
            candidate['collision'] = bool(fused.trajectory_collisions(trajectory[None,:,:2],np.zeros(2),cfg.static_cost_robot_radius)[0])
            delta = np.linalg.norm(trajectory[:,:2]-nominal_path[:,:2],axis=1)
            candidate['nominal_rms'] = float(np.sqrt(np.mean(delta**2)))
            candidate['nominal_tail'] = float(np.mean(delta[-3:]))
            candidate['reason'] = ('reach_failure' if reach is not None and (not reach['success'] or reach['violation']>1e-5) else
                                   'solver_failure' if not candidate['success'] or candidate['violation']>1e-5 else
                                   'static_collision' if candidate['collision'] else
                                   'duplicate_nominal' if not nominal_collision and candidate['nominal_rms']<.22 and candidate['nominal_tail']<.30 else 'eligible')
            candidates.append(candidate)
    chosen = []
    for candidate in sorted(candidates, key=lambda g:(g['cost'],g['id'])):
        if candidate['reason']!='eligible': continue
        if any(np.sqrt(np.mean(np.sum((candidate['trajectory'][:,:2]-other['trajectory'][:,:2])**2,axis=1)))<.25 for other in chosen):
            candidate['reason']='duplicate_selected'
        elif len(chosen)>=3: candidate['reason']='capacity'
        else: candidate['reason']='selected';chosen.append(candidate)
    chosen.sort(key=lambda g:g['id'])
    counts, weights, tau = allocate([g['cost'] for g in chosen])
    for i,g in enumerate(chosen):g['samples']=int(counts[i+1]);g['allocation_weight']=float(weights[i])
    for g in candidates:g.setdefault('samples',0)
    proposal_ms=(time.perf_counter()-tick)*1000
    return dict(parts=parts,depth=depth,angles=angles,field=fused,current=current,nominal=nominal,
                nominal_path=nominal_path,candidates=candidates,chosen=chosen,counts=counts,tau=tau,
                geometry_ms=geometry_ms,proposal_ms=proposal_ms)


def stats(x):
    x=np.asarray(x)
    return dict(n=len(x),mean=float(np.mean(x)),p50=float(np.median(x)),p95=float(np.percentile(x,95)),
                p99=float(np.percentile(x,99)),maximum=float(np.max(x)),over_180=int(np.sum(x>180)),over_200=int(np.sum(x>200)))


def run(iterations=10, maxiter=45, selected_scene=None, fixed_target=True):
    sources=json.loads((OUT/'raw_sources.json').read_text())
    if selected_scene:sources=[r for r in sources if r['scene']==selected_scene]
    gap=GapConfig(timeout=10.)
    cfg=replace(config_from_yaml(),strategy='standard',gap=gap,guide_timeout=10.,guide_num_rollouts=500,guide_angles_deg=(0.,1.,2.,3.))
    print('DERIVATIVE ERRORS',check_derivatives(cfg),flush=True)
    write(OUT/'configuration.json',dict(controller=asdict(cfg),weights=WEIGHTS,maxiter=maxiter,fixed_target=fixed_target,
        implementation='Local nonlinear constrained SLSQP with analytic gradients; no global-optimality certificate.',
        endpoint='pN=rho*e_gap, .25<=rho<=d; all intermediate projections<=d. Intermediate lateral deviations penalized; swept static postcheck mandatory.',
        target_policy='Maximize reachable rho with joint vx,vy,omega and NO heading cost, then minimize common cost within 1cm of that target.' if fixed_target else 'Choose distance and controls together by common-cost minimization; NOT fixed maximum-distance target.',
        reference='Common path point at s0+desired_speed*T (capped at path end); common unwrapped path headings. Neither reference shortened per gap.',
        allocation='nominal200 + gap pool300; max3; minimum30 each, remaining via .2/G+.8*softmax(-(V-minV)/tau), tau=max(1,median(V-minV)); total500.',
        fullcircle='One geometric gap, four direction probes at -180,-90,0,90 degrees; not four artificial gaps.',
        timing='Profiling deadline raised to10s ONLY to observe uncensored runtimes. Persistent worker/sampler pools; fixed snapshots; no ROS publication, perception or closed-loop validation.',
        environment=dict(platform=platform.platform(),python=platform.python_version(),numpy=np.__version__,
                         openblas_threads=os.getenv('OPENBLAS_NUM_THREADS'),omp_threads=os.getenv('OMP_NUM_THREADS'))))
    all_rows=[]; summaries=[]
    for source in sources:
        scene=source['scene'];data=dict(np.load(OUT/f'raw/{scene}.npz'))
        field,scans=geometry(data,gap,cfg.static_cost_robot_radius)
        predictions=None
        if 'prediction_means' in data:
            predictions=PredictionSet(data['prediction_means'],data['prediction_covariances'],data['prediction_times'],
                    mode_weights=data.get('prediction_mode_weights'),obstacle_radii=data.get('prediction_obstacle_radii'))
        c=MppiController(replace(cfg,strategy='standard'));c.config.strategy='guide_conditioned';c._guide_optimizer=PreparedOptimizer(c)
        baseline=MppiController(replace(cfg,strategy='gap'))
        scene_rows=[];first=None
        try:
            for trial in range(iterations+1):
                # Alternate order to reduce a fixed ordering advantage.
                for method in (['baseline','value'] if trial%2==0 else ['value','baseline']):
                    started=time.perf_counter()
                    if method=='baseline':
                        baseline._guide_optimizer.reset();baseline._guide_optimizer._warm_nominal=data['nominal'].copy()
                        result=baseline.compute(np.zeros(3),data['path'].copy(),data['current'].copy(),predictions,
                                                visualization_rollouts=0,distance_field=field,gap_scans=scans)
                        extra={}
                    else:
                        prop=proposals(c,data,field,scans,maxiter,fixed_target)
                        opt=c._guide_optimizer
                        opt.set_proposals([prop['nominal']]+[g['controls']for g in prop['chosen']],
                                          ['nominal']+[g['id']for g in prop['chosen']],prop['counts'])
                        tick=time.perf_counter()
                        result=c.compute(np.zeros(3),data['path'].copy(),prop['current'].copy(),predictions,
                                         visualization_rollouts=0,distance_field=prop['field'])
                        extra=dict(geometry_ms=prop['geometry_ms'],proposal_ms=prop['proposal_ms'],
                                   mppi_ms=(time.perf_counter()-tick)*1000,
                                   candidate_count=len(prop['candidates']),solver_failures=sum(not g['success']for g in prop['candidates']))
                    elapsed=(time.perf_counter()-started)*1000
                    assert sum(result.guide_rollout_counts)==500
                    if result.safety_status=='ok':
                        assert not field.trajectory_collisions(result.trajectory[None,:,:2],np.zeros(2),cfg.static_cost_robot_radius)[0]
                        assert result.max_collision_probability<=cfg.dynamic_risk.hard_threshold+1e-9
                    row=dict(scene=scene,trial=trial,method=method,total_ms=elapsed,predictions=source['predictions'],
                             selected=result.selected_mode,status=result.safety_status,cost=result.cost,
                             max_probability=result.max_collision_probability,counts=result.guide_rollout_counts,
                             primary_risk_ms=result.primary_risk_time_ms,final_risk_ms=result.final_risk_time_ms,**extra)
                    if trial>0:scene_rows.append(row);all_rows.append(row)
                    if method=='value' and trial==0:
                        first=prop
                        arrays=dict(angles=prop['angles'],depth=prop['depth'],nominal_path=prop['nominal_path'],
                                    seed_paths=np.array([g['trajectory']for g in prop['candidates']]),
                                    seed_controls=np.array([g['controls']for g in prop['candidates']]),
                                    selected_path=result.trajectory)
                        np.savez_compressed(OUT/f'{scene}_results.npz',**arrays)
                    if trial==0:print('WARMUP',scene,method,round(elapsed,1),'ms',flush=True)
            # Offline diagnostic only, excluded from proposal/kernel timing:
            # same objective with the LiDAR depth bound removed (4m > physical horizon reach).
            for g in first['candidates']:
                problem=ValueProblem(cfg,first['current'],data['path'],np.deg2rad(g['angle_deg']),4.)
                start=g['controls']
                if fixed_target:
                    reach_problem=ReachProblem(cfg,first['current'],data['path'],np.deg2rad(g['angle_deg']),4.)
                    free_reach=reach_problem.solve(start,maxiter=120)
                    problem.target_distance=max(.25,float(free_reach['trajectory'][-1,:2]@problem.e))
                    start=free_reach['controls']
                    g['without_space_reach_success']=free_reach['success']
                free=problem.solve(start,maxiter=120)
                g['without_space_cost']=free['cost'];g['without_space_distance']=float(free['trajectory'][-1,:2]@problem.e)
                g['without_space_success']=free['success'];g['without_space_violation']=free['violation']
                g['space_cost_increase']=g['cost']-free['cost']
            candidates=[{k:v for k,v in g.items()if k not in ['controls','trajectory']}for g in first['candidates']]
            summary=dict(**source,regions=first['parts']['region_count'],partition=first['parts'],
                         candidates=candidates,counts=first['counts'],tau=first['tau'],
                         baseline_ms=stats([r['total_ms']for r in scene_rows if r['method']=='baseline']),
                         value_ms=stats([r['total_ms']for r in scene_rows if r['method']=='value']),
                         value_geometry_ms=stats([r['geometry_ms']for r in scene_rows if r['method']=='value']),
                         value_proposal_ms=stats([r['proposal_ms']for r in scene_rows if r['method']=='value']),
                         value_mppi_ms=stats([r['mppi_ms']for r in scene_rows if r['method']=='value']))
            summaries.append(summary)
            write(OUT/f'{scene}_summary.json',summary)
            write(OUT/'timings.json',all_rows);write(OUT/'scene_results.json',summaries)
            print('DONE',scene,'gaps',first['parts']['region_count'],'candidates',len(candidates),
                  'counts',first['counts'],'baseline/value p95',round(summary['baseline_ms']['p95'],1),round(summary['value_ms']['p95'],1),flush=True)
        finally:c.close();baseline.close()
    write(OUT/'aggregate.json',{method:stats([r['total_ms']for r in all_rows if r['method']==method])for method in ['baseline','value']})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extract',action='store_true');parser.add_argument('--iterations',type=int,default=10)
    parser.add_argument('--maxiter',type=int,default=45);parser.add_argument('--scene')
    parser.add_argument('--joint-distance',action='store_true')
    args=parser.parse_args();OUT.mkdir(parents=True,exist_ok=True)
    if args.extract:extract()
    else:run(args.iterations,args.maxiter,args.scene,not args.joint_distance)

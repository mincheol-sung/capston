"""Paired, offline seed-objective / sample-allocation ablation on bag snapshots.

Never edits runtime configuration or the MPPI evaluator. Every trial evaluates
exactly 500 rollouts with the existing static AND dynamic objective, then calls
the existing per-group update and final representative selection.
"""
import argparse
import copy
from dataclasses import asdict, fields, replace
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np
import yaml

from benchmark_guide_conditioned import config_from_yaml, summary
from analyze_gap_seed_objectives import DirectProblem, existing
from dra_mppi.common_cost import project_path, common_costs
from dra_mppi.geometry import wrap_angle
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.gap_conditioned import GapConditionedOptimizer
from dra_mppi.gap_geometry import GapConfig, partition
from dra_mppi.gap_inputs import GapScan, directional_depth, fuse_scans
from dra_mppi.gap_optimization import ValueProblem, ReachProblem, reference, allocate
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.guide_conditioned import GuideConditionedOptimizer
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiController

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/gap_value_optimization_2026-11-05'
OUT = ROOT / 'docs/gap_objective_ablation_2026-11-06'
METHODS = ('current', 'smooth', 'smooth_heading', 'guide_tracking', 'existing', 'linearized_common')
POLICIES = ('uniform', 'native', 'common')
LOCKED = ('config/dra_mppi.yaml', 'dra_mppi/common_cost.py',
          'dra_mppi/mppi_controller.py', 'dra_mppi/guide_conditioned.py',
          'dra_mppi/dynamic_risk.py', 'dra_mppi/gap_optimization.py')


def serial(x):
    if isinstance(x, dict): return {k: serial(v) for k, v in x.items()}
    if isinstance(x, (tuple, list)): return [serial(v) for v in x]
    if isinstance(x, np.ndarray): return serial(x.tolist())
    if isinstance(x, (np.bool_,)): return bool(x)
    if isinstance(x, np.integer): return int(x)
    if isinstance(x, (float, np.floating)): return float(x) if np.isfinite(x) else None
    return x


def write(path, value):
    path.write_text(json.dumps(serial(value), indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def hashes():
    return {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in LOCKED}


class AnalyticProblem(ValueProblem):
    """Literature-inspired ablations; not implementations of the cited papers."""
    def __init__(self, *args, method='current', **kwargs):
        super().__init__(*args, **kwargs)
        self.method = method
        if method in ('smooth', 'smooth_heading'):
            self.weights['progress'] = self.weights['straight'] = 0.
        if method == 'smooth': self.weights['heading'] = 0.
        if method == 'guide_tracking': self.weights['straight'] = 0.

    def objective(self, x):
        value, gradient, parts = super().objective(x)
        if self.method == 'guide_tracking':
            state, jac = self.state(x)
            target = np.arange(1, self.n+1)[:, None]/self.n*self.target_distance*self.e
            error = state[:, :2]-target
            weight = self.cfg.gap.value_straight_weight
            parts['guide_tracking'] = weight*float(np.mean(np.sum(error**2, axis=1)))
            value += parts['guide_tracking']
            gradient += 2*weight/self.n*np.einsum('ti,tij->j', error, jac[:, :2])
        return value, gradient, parts


class PreparedOptimizer(GuideConditionedOptimizer):
    def __init__(self, controller):
        super().__init__(controller, angles=np.arange(4))
        self.noise_pool = self.noise[:]
        self.risk_template = copy.deepcopy(self._guide_risk_estimators)
        self.final_template = copy.deepcopy(self._final_risk_estimator)

    def set_proposals(self, controls, names, counts):
        self.count = len(controls)
        self.proposals = np.asarray(controls)
        self.names = tuple(names)+('stop',)
        self.counts = np.asarray(counts, int)
        self.nominals = np.zeros_like(self.proposals)
        self.initialized = np.zeros(self.count, bool)
        self.temperatures = np.full(self.count, self.cfg.temperature)
        self.previous = self.entry_yaw = None
        self.noise = self.noise_pool[:self.count]
        # Also pair the risk quadrature RNG, not just the control noise.
        self._guide_risk_estimators = copy.deepcopy(self.risk_template[:self.count])
        self._final_risk_estimator = copy.deepcopy(self.final_template)

    def anchors(self, *args, **kwargs): return self.proposals.copy()
    def allocate(self, active): return self.counts.copy()


def common_residuals(cfg, path, current, controls, progress):
    """Residual vector of EXACT common_costs, checked against production below.

    Experiment-only duplication to form one Gauss-Newton quadratic. Production
    common_costs remains the authority for every sample and representative.
    """
    states=np.concatenate((np.zeros((len(controls),1,3)),rollout(np.zeros(3),controls,cfg.dt)),axis=1)
    closest,s,heading,_,length=project_path(states[:,:,:2],path,cfg.heading_lookahead,cfg.projection_window,progress)
    n=controls.shape[1]
    near=(length-s[:,0]<=cfg.goal_activation_distance)&(np.linalg.norm(path[-1])<=cfg.goal_activation_distance)
    remaining=np.maximum(length-s[:,0],np.linalg.norm(path[-1]))
    remaining=np.maximum(0.,remaining[:,None]-cfg.desired_speed*cfg.dt*np.arange(n))
    vref=np.minimum(cfg.desired_speed,np.sqrt(2*min(cfg.acceleration_limit[:2])*remaining))
    vref=np.minimum(vref,remaining/cfg.dt)
    target=np.minimum(length,s[:,0]+cfg.dt*np.sum(vref,axis=1))
    angle=states[:,:-1,2]-heading[:,:-1]
    speed=controls[:,:,0]*np.cos(angle)-controls[:,:,1]*np.sin(angle)
    delta=np.diff(np.concatenate((np.broadcast_to(current,(len(controls),1,3)),controls),axis=1),axis=1)
    pieces=[np.sqrt(cfg.tracking_path_weight/n)*(states[:,1:,:2]-closest[:,1:]),
            np.sqrt(cfg.tracking_goal_weight)*near[:,None]*(states[:,-1,:2]-path[-1]),
            np.sqrt(cfg.heading_weight/n)*wrap_angle(states[:,1:,2]-heading[:,1:]),
            np.sqrt(cfg.progress_weight)*(s[:,-1:]-target[:,None]),
            np.sqrt(cfg.speed_weight/n)*(speed-vref),
            delta*np.sqrt(np.asarray(cfg.input_delta_weight)/n)]
    return np.concatenate([p.reshape(len(controls),-1)for p in pieces],axis=1)


class LinearizedCommonProblem(ValueProblem):
    """One local Gauss-Newton objective; true nonlinear motion constraints stay.

    Jquad(U)=||r(U0)+Dr(U0)(U-U0)||^2. No obstacle terms. This is a new
    experimental adaptation, not a result established by the cited papers.
    """
    def linearize(self, path, progress, start):
        self.x0=np.asarray(start).ravel().copy()
        epsilon=1e-6
        batch=np.vstack((self.x0,self.x0+epsilon*np.eye(len(self.x0)))).reshape(-1,self.n,3)
        residual=common_residuals(self.cfg,path,self.current,batch,progress)
        exact=sum(common_costs(self.cfg,np.zeros(3),path,batch,rollout(np.zeros(3),batch,self.cfg.dt),self.current,progress))
        np.testing.assert_allclose(np.sum(residual**2,axis=1),exact,rtol=1e-10,atol=1e-10)
        self.r0=residual[0];self.dr=((residual[1:]-residual[0])/epsilon).T

    def objective(self,x):
        self.check_deadline()
        r=self.r0+self.dr@(x-self.x0)
        value=float(r@r)
        return value,2*r@self.dr,dict(local_quadratic=value)


def make_problem(method, cfg, current, path, angle, depth, common, field, progress, distance):
    if method == 'existing':
        obj = DirectProblem(cfg, current, path, angle, depth, common, field, progress, False)
    elif method == 'linearized_common':
        obj = LinearizedCommonProblem(cfg,current,path,angle,depth,common)
    else:
        obj = AnalyticProblem(cfg, current, path, angle, depth, common, method=method)
    obj.target_distance = distance
    return obj


def safe(field, cfg, u):
    return not bool(field.trajectory_collisions(
        rollout(np.zeros(3), u, cfg.dt)[None, :, :2], np.zeros(2), cfg.static_cost_robot_radius)[0])


def prepare(data, cfg, controller, methods=METHODS, frozen_names=None):
    started = time.perf_counter()
    field = DistanceField.from_occupancy(data['grid'], GridMetadata(200,200,.04,-4.,-4.,0.))
    scans = tuple(GapScan(*(data[f'scan_{i}_{key}'] for key in ('ranges','valid','origins','hits')))
                  for i in range(2))
    field, _, local = fuse_scans(scans, np.zeros(3), field, None, cfg.gap, cfg.static_cost_robot_radius)
    angles, strict, hard = directional_depth(field, cfg.gap, cfg.static_cost_robot_radius)
    arrays = dict(strict_depth=strict, hard_depth=hard)
    for i, scan in enumerate(local):
        arrays.update({f'scan_{i}_{k}': scan[k] for k in ('ranges','valid','hits')})
    regions = partition(arrays, asdict(cfg.gap))
    depth = hard if regions['fallback'] else strict
    geometry_ms = (time.perf_counter()-started)*1000
    current = controller._project_wheel_limit(np.clip(data['current'],cfg.control_min,cfg.control_max)[None])[0]
    nominal = controller._constrain_controls(data['nominal'][None], current)[0]
    path = data['path']
    progress = float(project_path(np.zeros((1,1,2)), path, cfg.heading_lookahead, cfg.projection_window)[1][0,0])
    common = reference(path, cfg, progress)
    candidates = []
    for region in regions['regions']:
        bins = region['bins']
        midpoints = [bins[len(bins)//2]] if len(bins)<360 else [0,90,180,270]
        for sub, j in enumerate(midpoints):
            label = f'G{region["id"]}'+(f'.{sub+1}' if len(midpoints)>1 else '')
            angle, d = float(angles[j]), float(depth[j])
            e = np.array([np.cos(angle), np.sin(angle)])
            start = GapConditionedOptimizer._seed(controller._guide_optimizer,
                    np.zeros(3), path, current, min(d-.1,.8)*e)
            # Heading is free; no obstacle preference in the maximum-reach solve.
            free = ReachProblem(cfg, current, path, angle, 100., common).solve(start)
            D = float(free['trajectory'][-1,:2]@e)
            target_distance = min(D, d)
            # A second reach solve supplies an objective-independent, feasible
            # start at the LiDAR-limited endpoint. Shared by EVERY cost variant.
            bounded = ReachProblem(cfg, current, path, angle, target_distance, common)
            bounded.target_distance = target_distance
            neutral = bounded.solve(free['controls'])
            neutral_ok = (free['success'] and free['violation']<1e-5 and neutral['success']
                          and neutral['violation']<1e-5 and safe(field,cfg,neutral['controls']))
            candidate = dict(id=label, angle=angle, depth=d, free_distance=D,
                distance=target_distance, reach_ms=free['wall_ms']+neutral['wall_ms'],
                reach_success=free['success'], neutral_success=neutral['success'],
                neutral_violation=neutral['violation'], neutral_ok=neutral_ok,
                neutral_controls=neutral['controls'], variants={})
            if neutral_ok:
                for method in methods:
                    tick = time.perf_counter()
                    problem = make_problem(method,cfg,current,path,angle,d,common,field,progress,target_distance)
                    if method=='linearized_common':problem.linearize(path,progress,neutral['controls'])
                    answer = problem.solve(neutral['controls'])
                    valid = answer['success'] and answer['violation']<1e-5
                    collision = not safe(field,cfg,answer['controls'])
                    accepted = bool(valid and not collision)
                    u = answer['controls'] if accepted else neutral['controls'].copy()
                    cost, _, components = problem.objective(u.ravel())
                    candidate['variants'][method] = dict(controls=u, native_cost=cost, components=components,
                        accepted=accepted, solver_success=answer['success'], violation=answer['violation'],
                        collision=collision, iterations=answer['iterations'], message=answer['message'],
                        solve_ms=(time.perf_counter()-tick)*1000)
            candidates.append(candidate)
    # Freeze membership using the current objective, before policy comparisons.
    # This retains production max-3/dedup behavior without favoring a new method.
    selected = []
    nominal_path = rollout(np.zeros(3),nominal,cfg.dt)
    nominal_safe = safe(field,cfg,nominal)
    eligible = [q for q in candidates if q['neutral_ok']]
    selection_method=methods[0]
    for q in sorted(eligible, key=lambda q:(q['variants'][selection_method]['native_cost'],q['id'])):
        if frozen_names is not None:
            q['reason']='selected' if q['id'] in frozen_names else 'frozen_exclusion'
            if q['reason']=='selected':selected.append(q)
            continue
        traj = rollout(np.zeros(3),q['variants'][selection_method]['controls'],cfg.dt)[:,:2]
        diff = np.linalg.norm(traj-nominal_path[:,:2],axis=1)
        if nominal_safe and np.sqrt(np.mean(diff**2))<cfg.gap.nominal_rms_m and np.mean(diff[-3:])<cfg.gap.nominal_tail_m:
            q['reason']='duplicate_nominal'
        elif any(np.sqrt(np.mean(np.sum((traj-rollout(np.zeros(3),s['variants'][selection_method]['controls'],cfg.dt)[:,:2])**2,axis=1)))<cfg.gap.guide_rms_m for s in selected):
            q['reason']='duplicate_selected'
        elif len(selected)>=cfg.gap.max_guides: q['reason']='capacity'
        else: q['reason']='selected';selected.append(q)
    selected.sort(key=lambda q:q['id'])
    names = ['nominal']+[q['id'] for q in selected]
    proposals = {}
    displacement = nominal_path[-1,:2]
    if np.linalg.norm(displacement)<1e-6: displacement=common[0]
    nominal_angle = float(np.arctan2(displacement[1],displacement[0]))
    for method in methods:
        u = np.asarray([nominal]+[q['variants'][method]['controls'] for q in selected])
        # Nominal's own endpoint defines its guide direction for native scoring;
        # the common navigation reference is never shortened for any group.
        problem = make_problem(method,cfg,current,path,nominal_angle,100.,common,field,progress,
                               float(np.linalg.norm(nominal_path[-1,:2])))
        if method=='linearized_common':problem.linearize(path,progress,nominal)
        native = np.array([problem.objective(nominal.ravel())[0]]+
                          [q['variants'][method]['native_cost'] for q in selected])
        base, static, _ = existing(cfg,path,current,u,field,progress)
        common_scores = base+static
        counts = dict(uniform=allocate(np.zeros(len(u)),cfg.gap)[0],
                      native=allocate(native,cfg.gap)[0],
                      common=allocate(common_scores,cfg.gap)[0])
        proposals[method] = dict(controls=u, native_scores=native, common_scores=common_scores, counts=counts)
    predictions = None
    if 'prediction_means' in data:
        predictions = PredictionSet(data['prediction_means'],data['prediction_covariances'],data['prediction_times'],
            mode_weights=data.get('prediction_mode_weights'),obstacle_radii=data.get('prediction_obstacle_radii'))
    return dict(field=field,current=current,nominal=nominal,path=path,common=common,progress=progress,
        geometry_ms=geometry_ms,regions=regions,candidates=candidates,selected=selected,
        proposals=proposals,names=names,predictions=predictions)


def execute(controller, prepared, proposals, names, counts):
    controller._guide_optimizer.set_proposals(proposals,names,counts)
    controller._projection_path = controller._projection_position = controller._projection_progress = None
    result = controller.compute(np.zeros(3),prepared['path'].copy(),prepared['current'].copy(),
        prepared['predictions'],visualization_rollouts=0,distance_field=prepared['field'])
    assert result.sampled_controls.shape==(500,controller.config.horizon_steps,3)
    assert sum(result.guide_rollout_counts)==500
    index=result.candidate_names.index(result.selected_mode)
    if result.safety_status=='ok':
        assert not result.final_static_collisions[index]
        assert result.max_collision_probability<=controller.config.dynamic_risk.hard_threshold
    return result


def derivative_check(cfg):
    rng=np.random.default_rng(42)
    x=rng.normal(0,.08,30)
    path=np.array([[0.,0.],[2.,.5]])
    errors={}
    for method in [m for m in METHODS if m!='existing']:
        problem=make_problem(method,cfg,np.zeros(3),path,.4,2.,reference(path,cfg),None,0.,1.)
        if method=='linearized_common':problem.linearize(path,0.,x.reshape(-1,3))
        _,analytic,_=problem.objective(x)
        eps=1e-6
        numeric=np.array([(problem.objective(x+eps*e)[0]-problem.objective(x-eps*e)[0])/(2*eps) for e in np.eye(len(x))])
        errors[method]=float(np.max(np.abs(analytic-numeric)))
        assert errors[method]<1e-6
    return errors


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials',type=int,default=8)
    parser.add_argument('--scene')
    parser.add_argument('--output',type=Path,default=OUT)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'arrays').mkdir(exist_ok=True)
    params=yaml.safe_load((ROOT/'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    gap=GapConfig(**{f.name:params['gap.'+f.name] for f in fields(GapConfig) if 'gap.'+f.name in params})
    cfg=replace(config_from_yaml(),strategy='guide_conditioned',gap=gap,guide_num_rollouts=500,
                guide_angles_deg=(0.,1.,2.,3.),guide_timeout=10.)
    locked=hashes()
    sources=json.loads((SOURCE/'raw_sources.json').read_text())
    if args.scene: sources=[s for s in sources if s['scene']==args.scene]
    manifest=dict(configuration=asdict(cfg),source_hashes=locked,methods=METHODS,policies=POLICIES,
        sources=sources,trials=args.trials,derivative_errors=derivative_check(cfg),
        environment=dict(python=platform.python_version(),numpy=np.__version__,platform=platform.platform()),
        scope='Frozen original-bag snapshots, no ROS or closed loop. All objectives reoptimized from the SAME objective-independent constrained-reach seed. Common endpoint min(free local reach, LiDAR depth), omega free. Reach/seed SLSQP maxiter45, no wall-clock censoring for objective-quality comparison. Max3 groups chosen once with current objective and frozen. Failed/colliding solutions fall back to shared neutral seed. All 500 sample/final costs unchanged including static and recorded dynamic predictions. Uniform means uniform discretionary budget AFTER identical nominal100/gap30 reserves. Halton control noise and primary-risk RNG paired across policies; risk spatial domains remain group-dependent as in production. Native allocation uses each method own objective; common allocation uses unchanged common_costs plus static soft, no dynamic. No cache/hysteresis state carried between snapshots. Timings do not establish full-stack 5Hz.')
    write(args.output/'configuration.json',manifest)
    runs=[]; scenes=[]
    with (args.output/'runs.jsonl').open('w') as log:
        for source in sources:
            scene=source['scene'];data=dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
            generator=MppiController(cfg)
            try: prepared=prepare(data,cfg,generator)
            finally: generator.close()
            record={k:prepared[k] for k in ('geometry_ms','regions','candidates','names','proposals')}
            record.update(scene=scene,source=source,
                input_sha256=hashlib.sha256((SOURCE/'raw'/f'{scene}.npz').read_bytes()).hexdigest())
            write(args.output/f'{scene}_seeds.json',record)
            scenes.append(record)
            print('SOLVED',scene,'groups',prepared['names'],'accepted',
                {m:sum(q['variants'].get(m,{}).get('accepted',False) for q in prepared['candidates']) for m in METHODS},flush=True)
            for trial in range(args.trials):
                trial_cfg=replace(cfg,seed=cfg.seed+trial*10000,
                    dynamic_risk=replace(cfg.dynamic_risk,seed=cfg.dynamic_risk.seed+trial*10000))
                controller=MppiController(trial_cfg)
                controller._guide_optimizer.shutdown()
                controller._guide_optimizer=PreparedOptimizer(controller)
                combinations=[(m,p) for m in METHODS for p in POLICIES]+[('nominal_only','uniform')]
                # Deterministic shuffle prevents cold/order timing bias by method.
                np.random.default_rng(314+trial).shuffle(combinations)
                try:
                    for method,policy in combinations:
                        if method=='nominal_only':
                            controls=prepared['nominal'][None];names=['nominal'];counts=np.array([500])
                        else:
                            proposal=prepared['proposals'][method]
                            controls=proposal['controls'];names=prepared['names'];counts=proposal['counts'][policy]
                        tick=time.perf_counter()
                        result=execute(controller,prepared,controls,names,counts)
                        wall=(time.perf_counter()-tick)*1000
                        row=dict(scene=scene,trial=trial,method=method,policy=policy,
                            cost=result.cost,status=result.safety_status,selected=result.selected_mode,
                            counts=counts,names=names,final_costs=result.mode_final_costs,
                            probability=result.max_collision_probability,
                            invalid_rollouts=int(np.sum(~np.isfinite(result.rollout_costs))),
                            mppi_ms=wall,selected_components=result.selected_cost_weighted)
                        runs.append(row)
                        log.write(json.dumps(serial(row),ensure_ascii=False,allow_nan=False)+'\n');log.flush()
                        if trial==0:
                            np.savez_compressed(args.output/'arrays'/f'{scene}_{method}_{policy}.npz',
                                sampled_controls=result.sampled_controls,rollout_costs=result.rollout_costs,
                                group_ids=result.control_mode_ids,nominal=prepared['nominal'],seeds=controls,
                                representatives=result.mode_trajectories,final=result.trajectory,
                                final_controls=result.control_sequence,counts=counts)
                finally: controller.close()
            print('MPPI DONE',scene,args.trials*len(combinations),'cycles',flush=True)
    assert hashes()==locked,'Locked evaluator/configuration changed during experiment'
    write(args.output/'runs.json',runs)
    print('DONE',len(scenes),'scenes',len(runs),'MPPI calls; evaluator hashes unchanged',flush=True)


if __name__=='__main__':main()

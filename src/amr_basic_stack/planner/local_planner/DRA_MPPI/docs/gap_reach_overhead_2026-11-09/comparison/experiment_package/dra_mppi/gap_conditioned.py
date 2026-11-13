"""Budgeted gap reach/value seeds with common nominal/guide allocation."""
from dataclasses import asdict, replace
import time
import numpy as np
from .geometry import transform_points, wrap_angle
from .mecanum_model import rollout
from .guide_conditioned import GuideConditionedOptimizer
from .gap_geometry import partition
from .gap_inputs import GapInputUnavailable, fuse_scans, directional_depth
from .gap_optimization import (OptimizationDeadline, reference, score_controls,
                               wheel_distance_bound, solve_gap_values, gap_value_diagnostics,
                               allocate as allocate_values)


class GapConditionedOptimizer(GuideConditionedOptimizer):
    def __init__(self,controller):
        self.gap=controller.config.gap.validate()
        if not controller.config.static_cost_enabled:
            raise ValueError('gap strategy requires static_cost.enabled')
        config=replace(controller.config,guide_num_rollouts=self.gap.total_rollouts,guide_timeout=self.gap.timeout)
        super().__init__(controller,config=config,angles=np.arange(1+self.gap.max_guides))
        self._estimator_pool=self._guide_risk_estimators
        self._noise_pool=self.noise

    def reset(self):
        super().reset()
        self._warm_nominal=None
        self._warm_temperature=float(self.cfg.temperature)
        self._anchors=None
        self._cache=[]
        self._cycle=0
        self._next_probe=0

    def anchors(self,state,path,current,goal=None):
        return self._anchors.copy()

    def allocate(self,active):
        return self._counts.copy()

    def _seed(self,state,path,current,target):
        c=self.controller;cfg=self.cfg
        pose=state.copy();previous=current.copy();controls=[]
        for _ in range(cfg.horizon_steps):
            delta=target-pose[:2];distance=np.linalg.norm(delta)
            velocity=delta/max(distance,1e-9)*min(cfg.desired_speed,distance/.6)
            tangent=c._nearest_path_geometry(pose[None,None,:],path)[3][0,0]
            cs,sn=np.cos(pose[2]),np.sin(pose[2])
            raw=np.array([cs*velocity[0]+sn*velocity[1],-sn*velocity[0]+cs*velocity[1],wrap_angle(tangent-pose[2])/cfg.dt])
            u=self.constrain(raw[None,None],previous)[0,0]
            controls.append(u);pose=rollout(pose,u[None],cfg.dt)[0];previous=u
        return np.array(controls)

    def _match_cache(self, spec, state, now):
        """Match geometry in the current frame, never a frame-local gap ID."""
        g=self.gap; best=None; best_overlap=0.
        for entry in self._cache:
            if now-entry['created'] > g.cache_max_age:
                continue
            if np.linalg.norm(state[:2]-entry['state'][:2]) > g.cache_max_translation_m:
                continue
            angle=wrap_angle(entry['world_angle']-state[2]-spec['angle'])
            if abs(angle)>np.deg2rad(g.cache_max_angle_deg):
                continue
            shift=int(round(np.rad2deg(entry['state'][2]-state[2])))
            old=set((np.asarray(entry['bins'])+shift)%360);new=set(spec['bins'])
            overlap=len(old&new)/max(1,len(old|new))
            if overlap>=g.cache_min_overlap and overlap>best_overlap:
                best=entry;best_overlap=overlap
        return best

    def _assess(self, spec, controls, target_distance, source, state, current,
                fused, transform, nominal_path, nominal_collision, common_reference):
        """Recompute costs and safety even for a previously validated seed."""
        cfg=self.cfg;g=self.gap
        controls=self.constrain(np.asarray(controls)[None],current)[0]
        trajectory=rollout(state,controls,cfg.dt)
        xy=transform_points(trajectory[:,:2],transform)
        direction=spec['direction'];target=target_distance*direction
        motion=float(xy[-1]@direction)
        error=float(np.linalg.norm(xy[-1]-target))
        collision=bool(fused.trajectory_collisions(trajectory[None,:,:2],state[:2],
            cfg.static_cost_robot_radius,source_to_grid=transform)[0])
        delta=np.linalg.norm(trajectory[:,:2]-nominal_path[:,:2],axis=1)
        rms=float(np.sqrt(np.mean(delta**2)));tail=float(np.mean(delta[-3:]))
        bearing=int(round(np.rad2deg(np.arctan2(xy[-1,1],xy[-1,0]))+180))%360
        cost,components=score_controls(cfg,current,common_reference,direction,controls)
        tolerance=(g.optimization_target_tolerance_m+g.optimization_constraint_tolerance*10
                   if source=='optimized' else g.max_target_error_m)
        feasibility=('nonfinite_cost' if not np.isfinite(cost) else
                     'static_collision' if collision else
                     'outside_gap' if bearing not in spec['bins'] else
                     'depth_limit' if np.max(xy@direction)>spec['depth']+g.optimization_constraint_tolerance else
                     'target_error' if error>tolerance else
                     'insufficient_motion' if motion<g.min_displacement_m-g.optimization_constraint_tolerance else None)
        reason=feasibility or ('duplicate_nominal' if not nominal_collision and
                rms<g.nominal_rms_m and tail<g.nominal_tail_m else 'eligible')
        return dict(controls=controls,trajectory=trajectory,cost=float(cost) if np.isfinite(cost) else None,
                    cost_components=components if np.isfinite(cost) else {},
                    target_distance_m=float(target_distance),target_error_m=error,
                    projected_displacement_m=motion,nominal_rms_m=rms,nominal_tail_m=tail,
                    static_collision=collision,reason=reason,feasible=feasibility is None,seed_source=source,
                    allocation_cost_source=('optimized_actual' if source=='optimized' else source+'_seed'))

    def compute(self,state,path,current,predictions,visual_count,field,transform,started,goal=None,scans=None):
        c=self.controller;cfg=self.cfg;g=self.gap
        radius=cfg.static_cost_robot_radius
        try:
            if not c.config.static_cost_enabled:
                raise GapInputUnavailable('gap requires static collision checks')
            cycle_deadline=started+g.timeout
            self._cycle+=1
            fused,world_to_local,local=fuse_scans(scans,state,field,transform,g,radius)
            angles,strict,hard=directional_depth(fused,g,radius)
            arrays=dict(strict_depth=strict,hard_depth=hard)
            for i,scan in enumerate(local):
                for key in ('ranges','valid','hits'):arrays[f'scan_{i}_{key}']=scan[key]
            parts=partition(arrays,asdict(g));depth=hard if parts['fallback'] else strict
            if time.perf_counter()>=cycle_deadline:
                raise TimeoutError('gap proposal deadline exceeded')
            local_path=transform_points(path,world_to_local)
            common_reference=reference(local_path,cfg,c._projection_progress)
            current=c._project_wheel_limit(np.clip(current,cfg.control_min,cfg.control_max)[None])[0]
            nominal=(c._make_path_guide(state,path,current) if self._warm_nominal is None else self._warm_nominal)
            nominal=self.constrain(nominal[None],current)[0]
            nominal_path=rollout(state,nominal,cfg.dt)
            nominal_collision=bool(fused.trajectory_collisions(nominal_path[None,:,:2],state[:2],radius,source_to_grid=world_to_local)[0])
            nominal_end=transform_points(nominal_path[-1,:2],world_to_local)
            nominal_direction=nominal_end if np.linalg.norm(nominal_end)>1e-6 else common_reference[0]
            nominal_direction=(nominal_direction/np.linalg.norm(nominal_direction)
                               if np.linalg.norm(nominal_direction)>1e-6 else np.array([1.,0.]))
            nominal_cost,nominal_components=score_controls(cfg,current,common_reference,nominal_direction,nominal)
            if not np.isfinite(nominal_cost):
                raise ValueError('nonfinite nominal proposal cost')
            regions=[];specs=[]
            for region in parts['regions']:
                ids=np.array(region['bins'])
                outline=depth[ids,None]*np.column_stack((np.cos(angles[ids]),np.sin(angles[ids])))
                regions.append(dict(id=region['id'],first_angle_deg=region['first_angle_deg'],
                    last_angle_deg=region['last_angle_deg'],bin_count=len(ids),
                    outline=transform_points(outline,state).tolist()))
                if g.max_guides==0:continue
                full_circle=len(ids)==360
                if full_circle:
                    # Nominal already covers its own displacement direction.
                    # Use the same local-frame path fallback as nominal scoring.
                    base_angle=np.arctan2(nominal_direction[1],nominal_direction[0])
                    directions=wrap_angle(base_angle+np.deg2rad([90.,-90.,180.]))
                else:
                    directions=[angles[int(ids[len(ids)//2])]]
                for sub,angle in enumerate(directions):
                    angle=float(angle)
                    # Depth is sampled at integer degrees; retain the exact
                    # relative angle and use the smaller bracketing depth.
                    depth_bin=(np.rad2deg(angle)+180.)%360
                    direction_depth=(min(depth[int(np.floor(depth_bin))%360],
                                         depth[int(np.ceil(depth_bin))%360]) if full_circle
                                     else depth[int(ids[len(ids)//2])])
                    specs.append(dict(id=f'gap_{region["id"]:02d}'+(f'_d{sub}' if full_circle else ''),
                        region_id=region['id'],bins=ids.tolist(),angle=angle,angle_deg=float(np.rad2deg(angle)),
                        direction=np.array([np.cos(angle),np.sin(angle)]),depth=float(direction_depth),
                        direction_kind='full_circle_nominal_relative' if full_circle else 'midpoint'))
            geometry_ms=(time.perf_counter()-started)*1000
            proposal_started=time.perf_counter()
            proposal_deadline=min(proposal_started+g.optimization_timeout,cycle_deadline-g.optimization_mppi_reserve)
            candidates=[dict(id=s['id'],region_id=s['region_id'],angle_deg=s['angle_deg'],
                direction_kind=s['direction_kind'],depth_m=s['depth'],cost=None,reason='proposal_budget',
                seed_source='none',allocation_cost_source='none',optimization_status='not_attempted',
                feasible=False,**gap_value_diagnostics()) for s in specs]
            order=list(np.roll(np.arange(len(specs)),-(self._next_probe%max(1,len(specs)))))
            cache_created={}
            # Prepare/revalidate fallback proposals first. They remain available
            # when a native solver reaches its cooperative deadline later.
            for index in order:
                if time.perf_counter()>=proposal_deadline:break
                spec=specs[index];candidate=candidates[index]
                cached=self._match_cache(spec,state,time.perf_counter())
                assessed=None
                if cached is not None:
                    shift=min(cfg.horizon_steps,max(1,self._cycle-cached['cycle']))
                    controls=np.vstack((cached['controls'][shift:],np.repeat(cached['controls'][-1:],shift,axis=0)))
                    trial=self.constrain(controls[None],current)[0]
                    endpoint=transform_points(rollout(state,trial,cfg.dt)[-1,:2],world_to_local)
                    length=max(0.,min(spec['depth'],float(endpoint@spec['direction'])))
                    assessed=self._assess(spec,trial,length,'reused',state,current,fused,world_to_local,
                                          nominal_path,nominal_collision,common_reference)
                    if assessed['feasible']:cache_created[index]=cached['created']
                    else:assessed=None
                if assessed is None:
                    if time.perf_counter()>=proposal_deadline:break
                    available=max(.1,spec['depth']-g.target_reserve_m)
                    world_direction=transform_points(spec['direction'],[0.,0.,state[2]])
                    far=self._seed(state,path,current,state[:2]+available*world_direction)
                    reach=float((rollout(state,far,cfg.dt)[-1,:2]-state[:2])@world_direction)
                    length=min(available,max(g.target_floor_m,g.reach_scale*reach))
                    controls=self._seed(state,path,current,state[:2]+length*world_direction)
                    assessed=self._assess(spec,controls,length,'fallback',state,current,fused,world_to_local,
                                          nominal_path,nominal_collision,common_reference)
                    cache_created[index]=time.perf_counter()
                candidate.update(assessed)
                self._next_probe=(int(index)+1)%len(specs)
            attempts=0;converged=0
            for index in order:
                candidate=candidates[index];spec=specs[index]
                if time.perf_counter()>=proposal_deadline:break
                if 'controls' not in candidate:continue
                attempts+=1;self._next_probe=(int(index)+1)%len(specs)
                try:
                    # The wheel-only problem and its initial guess must not
                    # depend on this cycle's LiDAR depth or a cached gap seed.
                    wheel_start=self._seed(np.zeros(3),local_path,current,
                                           wheel_distance_bound(cfg)*spec['direction'])
                    solution=solve_gap_values(cfg,current,local_path,spec['angle'],spec['depth'],
                        wheel_start,common_reference,proposal_deadline,candidate)
                    if solution is None:
                        stage=candidate['optimization_stage']
                        candidate['optimization_status']=stage+'_'+candidate[stage+'_status'];continue
                    target=candidate['actual_target_distance_m']
                    candidate['reach_distance_m']=target # Backward-compatible actual-distance field.
                    assessed=self._assess(spec,solution['controls'],target,'optimized',state,current,fused,world_to_local,
                                          nominal_path,nominal_collision,common_reference)
                    if assessed['feasible']:
                        candidate.update(assessed);candidate['optimization_status']='converged'
                        # Score the constrained seed actually passed to MPPI;
                        # use this actual value, never the wheel-only value.
                        candidate['actual_value_cost']=assessed['cost']
                        candidate['actual_value_components']=assessed['cost_components']
                        candidate['value_difference']=(assessed['cost']-candidate['wheel_value_cost'] if candidate['wheel_value_cost'] is not None else None)
                        cache_created[index]=time.perf_counter();converged+=1
                    else:
                        candidate['optimization_status']='optimized_rejected_'+assessed['reason']
                except OptimizationDeadline:
                    candidate['optimization_status']='budget_exhausted';break
            proposal_ms=(time.perf_counter()-proposal_started)*1000
            chosen=[]
            for candidate in sorted((q for q in candidates if q['reason']=='eligible'),key=lambda q:(q['cost'],q['id'])):
                if any(np.sqrt(np.mean(np.sum((candidate['trajectory'][:,:2]-other['trajectory'][:,:2])**2,axis=1)))<g.guide_rms_m for other in chosen):
                    candidate['reason']='duplicate_selected'
                elif len(chosen)>=g.max_guides:candidate['reason']='budget_limit'
                else:candidate['reason']='selected';chosen.append(candidate)
            chosen.sort(key=lambda q:q['id'])
            self._counts,weights,tau=allocate_values([nominal_cost]+[q['cost']for q in chosen],g)
            self.count=1+len(chosen);self.names=tuple(['nominal']+[q['id']for q in chosen]+['stop'])
            self._anchors=np.array([nominal]+[q['controls']for q in chosen])
            for i,q in enumerate(chosen):q['samples']=int(self._counts[i+1])
            # All reusable proposals are rechecked next cycle, including ones
            # not selected this cycle. IDs never determine cache association.
            new_cache=[]
            for index,(spec,q) in enumerate(zip(specs,candidates)):
                if q['feasible']:
                    new_cache.append(dict(controls=q['controls'].copy(),bins=spec['bins'],state=state.copy(),
                        world_angle=state[2]+spec['angle'],cycle=self._cycle,created=cache_created[index]))
            self.nominals=np.zeros_like(self._anchors);self.initialized=np.zeros(self.count,bool)
            self.temperatures=np.full(self.count,cfg.temperature);self.temperatures[0]=self._warm_temperature
            self.previous=None;self.entry_yaw=None
            self._guide_risk_estimators=self._estimator_pool[:self.count];self.noise=self._noise_pool[:self.count]
            if time.perf_counter()>=cycle_deadline:raise TimeoutError('gap proposal deadline exceeded')
            result=super().compute(state,path,current,predictions,visual_count,fused,world_to_local,started,goal)
            diagnostics=dict(region_count=len(regions),fallback=parts['fallback'],rank_evaluated=True,
                target_policy='wheel_then_lidar',allocation_policy='cost_softmax_with_reserves',
                value_model='four_term_gap',global_optimality_certified=False,
                nominal_static_collision=nominal_collision,nominal_cost=float(nominal_cost),nominal_cost_components=nominal_components,
                candidates=[{k:v for k,v in q.items()if k not in ('controls','trajectory')}for q in candidates],
                selected=[q['id']for q in chosen],counts=self._counts.tolist(),allocation_weights=weights.tolist(),
                allocation_temperature=float(tau),geometry_ms=geometry_ms,proposal_ms=proposal_ms,
                optimization_attempts=attempts,optimization_converged=converged,
                proposal_budget_exhausted=proposal_started+proposal_ms/1000>=proposal_deadline,
                reused_candidates=sum(q['seed_source']=='reused' for q in candidates))
            if time.perf_counter()>=cycle_deadline:raise TimeoutError('gap final deadline exceeded')
            self._warm_nominal=np.vstack((result.control_sequence[1:],result.control_sequence[-1]))
            self._warm_temperature=result.next_temperature if result.next_temperature>0 else cfg.temperature
            c._nominal[:]=self._warm_nominal;self._cache=new_cache
            if result.safety_status!='ok':self.reset()
            return replace(result,strategy='gap',gap_diagnostics=diagnostics,gap_regions=tuple(regions))
        except Exception:
            self.reset()
            raise


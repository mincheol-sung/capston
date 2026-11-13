"""Allocation, derivatives, deadlines and geometry-based seed reuse contracts."""
from dataclasses import replace
import time
import numpy as np
import pytest

from dra_mppi.gap_geometry import GapConfig
from dra_mppi.gap_optimization import (
    ValueProblem, ReachProblem, OptimizationDeadline, allocate,
    rollout_jacobian, reference, score_controls, solve_gap_values, wheel_distance_bound,
)
from dra_mppi.mecanum_model import rollout
from dra_mppi.geometry import transform_points
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from test_gap_strategy import controller, field, scan, regions


def test_nominal_competes_for_remaining_budget_and_integer_conservation():
    cfg=GapConfig().validate()
    good,weights,_=allocate([0.,20.,40.,60.],cfg)
    bad,_,_=allocate([100.,20.,40.,60.],cfg)
    assert good.sum()==bad.sum()==500
    assert good[0]>bad[0]>=100
    assert np.all(good[1:]>=30) and np.all(bad[1:]>=30)
    assert good[1]>=good[2]>=good[3]
    np.testing.assert_allclose(weights.sum(),1.)
    shifted,_,_=allocate([500.,520.,540.,560.],cfg)
    np.testing.assert_array_equal(good,shifted)
    np.testing.assert_array_equal(allocate([1000.],cfg)[0],[500])
    # The legacy per-guide fixed count must no longer affect allocation.
    np.testing.assert_array_equal(good,allocate([0.,20.,40.,60.],replace(cfg,rollouts_per_guide=250))[0])
    with pytest.raises(ValueError):allocate([0.,float('nan')],cfg)


@pytest.mark.parametrize('kwargs',[
    dict(nominal_min_rollouts=1),dict(guide_min_rollouts=1),dict(total_rollouts=189),
    dict(allocation_exploration=1.01),dict(cache_min_overlap=0.),dict(cache_max_angle_deg=181.),
    dict(value_heading_weight=-1.),dict(optimization_mppi_reserve=.18),
    dict(optimization_target_tolerance_m=.25),dict(optimization_maxiter=1.5),
])
def test_invalid_value_settings_fail_before_planning(kwargs):
    with pytest.raises(ValueError):replace(GapConfig(),**kwargs).validate()


def test_nonlinear_derivatives_match_independent_differences_and_actual_rollout():
    c=controller()
    try:
        cfg=c.config;path=np.array([[0.,0.],[1.,.3],[2.,.6]])
        problem=ValueProblem(cfg,np.array([.1,-.1,.2]),path,.7,1.)
        problem.target_distance=.8
        x=np.random.default_rng(23).normal(0,.2,30);eps=1e-6
        state,jac=rollout_jacobian(x,10,.2)
        actual=rollout(np.zeros(3),x.reshape(10,3),.2)
        np.testing.assert_allclose(state,actual,atol=1e-12)
        for fun,analytic in [(lambda z:problem.objective(z)[0],problem.objective(x)[1]),
                             (lambda z:problem.equal(z)[0],problem.equal(x)[1]),
                             (lambda z:problem.inequality(z)[0],problem.inequality(x)[1]),
                             (lambda z:rollout_jacobian(z,10,.2)[0],jac)]:
            numeric=np.stack([(np.asarray(fun(x+eps*e))-np.asarray(fun(x-eps*e)))/(2*eps) for e in np.eye(30)],axis=-1)
            np.testing.assert_allclose(numeric,analytic,atol=2e-7,rtol=2e-6)
        value,parts=score_controls(cfg,problem.current,(problem.target,problem.heading),problem.e,x.reshape(10,3))
        assert value==pytest.approx(problem.objective(x)[0])
        assert parts==pytest.approx(problem.objective(x)[2])
    finally:c.close()


def test_reach_objective_has_no_heading_preference_but_value_does():
    c=controller()
    try:
        p=np.array([[0.,0.],[3.,0.]])
        common=(np.array([1.5,0.]),np.zeros(10))
        turned=(common[0],np.ones(10))
        x=np.zeros((10,3));x[:,1]=.3
        reach=ReachProblem(c.config,np.zeros(3),p,np.pi/2,2.,common)
        rotated=ReachProblem(c.config,np.zeros(3),p,np.pi/2,2.,turned)
        assert reach.objective(x.ravel())[0]==pytest.approx(-.6)
        assert reach.objective(x.ravel())[0]==rotated.objective(x.ravel())[0]
        a=ValueProblem(c.config,np.zeros(3),p,np.pi/2,2.,common).objective(x.ravel())[0]
        b=ValueProblem(c.config,np.zeros(3),p,np.pi/2,2.,turned).objective(x.ravel())[0]
        assert b-a==pytest.approx(c.config.gap.value_heading_weight)
    finally:c.close()


def test_solver_observes_wall_clock_deadline(monkeypatch):
    import dra_mppi.gap_optimization as module
    c=controller()
    try:
        p=ValueProblem(c.config,np.zeros(3),np.array([[0.,0.],[3.,0.]]),0.,1.)
        with pytest.raises(OptimizationDeadline):p.solve(np.zeros((10,3)),deadline=0.)
        now=[1.]
        monkeypatch.setattr(module.time,'perf_counter',lambda:now[0])
        def fake_solver(fun,x,**kw):
            fun(x)
            now[0]=3.
            kw['callback'](x)
            raise AssertionError('expired callback must interrupt solve')
        monkeypatch.setattr(module,'minimize',fake_solver)
        with pytest.raises(OptimizationDeadline):p.solve(np.zeros((10,3)),deadline=2.)
    finally:c.close()


def test_optimization_timeout_uses_validated_fallback_and_final_mppi(monkeypatch):
    import dra_mppi.gap_conditioned as module
    import dra_mppi.gap_optimization as optimization
    monkeypatch.setattr(module,'partition',lambda *args:regions([-90.,90.]))
    def expired(*args,**kwargs):raise OptimizationDeadline('forced budget')
    monkeypatch.setattr(optimization.ReachProblem,'solve',expired)
    c=controller()
    try:
        c.config.gap.optimization_timeout=.5
        r=c.compute(np.zeros(3),[[0.,0.],[5.,0.]],distance_field=field(),gap_scans=(scan(),scan()))
        assert r.safety_status=='ok' and sum(r.guide_rollout_counts)==500
        assert any(q['optimization_status']=='budget_exhausted' for q in r.gap_diagnostics['candidates'])
        selected=[q for q in r.gap_diagnostics['candidates']if q['reason']=='selected']
        assert selected and all(q['seed_source']=='fallback' and not q['static_collision']for q in selected)
        assert all(q['actual_value_cost'] is None and q['wheel_value_cost'] is None for q in selected)
        assert all(q['allocation_cost_source']=='fallback_seed' for q in selected)
        assert r.gap_diagnostics['nominal_cost']>=0
    finally:c.close()


@pytest.mark.parametrize('direction_deg,pose_yaw_deg,stationary',[
    (30.,0.,False), (30.25,70.,False), (170.,-45.,False),
    (30.,70.,True),
])
def test_full_circle_adds_three_nominal_relative_directions(direction_deg,pose_yaw_deg,stationary):
    c=controller()
    try:
        c.config.gap.optimization_timeout=.5
        state=np.array([.2,-.1,np.deg2rad(pose_yaw_deg)])
        angle=np.deg2rad(direction_deg)
        direction=np.array([np.cos(angle),np.sin(angle)])
        nominal=np.zeros((c.config.horizon_steps,3))
        if not stationary:nominal[:,:2]=.1*direction
        c._guide_optimizer._warm_nominal=nominal
        # Moving nominal intentionally disagrees with the reference path:
        # probes must follow displacement, not path direction or body yaw.
        path_direction=direction if stationary else np.array([1.,0.])
        path=transform_points(np.array([[0.,0.],5.*path_direction]),state)
        r=c.compute(state,path,distance_field=field(),gap_scans=(scan(state),scan(state)))
        assert r.gap_diagnostics['region_count']==1
        q=r.gap_diagnostics['candidates']
        expected=(direction_deg+np.array([90.,-90.,180.])+180.)%360.-180.
        np.testing.assert_allclose([g['angle_deg']for g in q],expected,atol=1e-10)
        assert all(g['direction_kind']=='full_circle_nominal_relative'for g in q)
        assert len(r.guide_rollout_counts)<=4 and sum(r.guide_rollout_counts)==500
    finally:c.close()


def test_cache_matches_world_geometry_and_rejects_age_pose_or_direction_change():
    c=controller()
    try:
        opt=c._guide_optimizer
        entry=dict(state=np.zeros(3),world_angle=0.,bins=list(range(170,191)),created=100.,controls=np.zeros((10,3)),cycle=1)
        opt._cache=[entry]
        spec=dict(id='new_frame_gap_99',angle=-np.pi/2,bins=list(range(80,101)))
        assert opt._match_cache(spec,np.array([0.,0.,np.pi/2]),100.1) is entry
        assert opt._match_cache(spec,np.array([0.,0.,np.pi/2]),101.) is None
        assert opt._match_cache(spec,np.array([.6,0.,np.pi/2]),100.1) is None
        assert opt._match_cache(dict(spec,angle=0.),np.array([0.,0.,np.pi/2]),100.1) is None
        assert opt._match_cache(dict(spec,bins=list(range(200,220))),np.array([0.,0.,np.pi/2]),100.1) is None
    finally:c.close()


def test_cache_rechecks_new_obstacle_and_never_reuses_stale_cost(monkeypatch):
    import dra_mppi.gap_conditioned as module
    monkeypatch.setattr(module,'partition',lambda *args:regions([0.]))
    c=controller()
    try:
        c.config.gap.optimization_timeout=.5
        c._guide_optimizer._warm_nominal=np.zeros((10,3))
        c.compute(np.zeros(3),[[0.,0.],[5.,0.]],distance_field=field(),gap_scans=(scan(),scan()))
        assert c._guide_optimizer._cache
        grid=np.zeros((200,200),int);grid[:,111:114]=100
        blocked=DistanceField.from_occupancy(grid,GridMetadata(200,200,.04,-4,-4,0))
        r=c.compute(np.zeros(3),[[0.,0.],[5.,0.]],distance_field=blocked,gap_scans=(scan(),scan()))
        assert not any(q['seed_source']=='reused' and q['reason']=='selected'for q in r.gap_diagnostics['candidates'])
        assert not any(q['static_collision']for q in r.gap_diagnostics['candidates']if q['reason']=='selected')
    finally:c.close()


def test_gap_values_are_invariant_to_world_pose_transform(monkeypatch):
    import dra_mppi.gap_conditioned as module
    monkeypatch.setattr(module,'partition',lambda *args:regions([-90.]))
    results=[]
    for state in [np.zeros(3),np.array([.4,-.3,.7])]:
        c=controller()
        try:
            c.config.gap.optimization_timeout=1.
            r=c.compute(state,transform_points([[0.,0.],[3.,0.]],state),
                        distance_field=field(),gap_scans=(scan(state),scan(state)))
            results.append(r.gap_diagnostics)
        finally:c.close()
    assert results[0]['nominal_cost']==pytest.approx(results[1]['nominal_cost'],abs=1e-7)
    assert results[0]['candidates'][0]['cost']==pytest.approx(results[1]['candidates'][0]['cost'],abs=1e-4)
    assert results[0]['counts']==results[1]['counts']


def test_exactly_two_solves_and_lidar_independent_reach(monkeypatch):
    c=controller()
    try:
        cfg=c.config;current=np.zeros(3);path=np.array([[0.,0.],[5.,0.]])
        common=reference(path,cfg)
        start=c._guide_optimizer._seed(np.zeros(3),path,current,np.array([wheel_distance_bound(cfg),0.]))
        calls=[]
        original=ValueProblem.solve
        def counted(problem,*args,**kwargs):
            calls.append(type(problem))
            return original(problem,*args,**kwargs)
        monkeypatch.setattr(ValueProblem,'solve',counted)
        results=[]
        for depth in (2.,.7,.9):
            calls.clear()
            info={}
            solution=solve_gap_values(cfg,current,path,0.,depth,start,common,diagnostics=info)
            assert solution is not None
            assert calls==[ReachProblem,ValueProblem]
            assert info['wheel_reach_status']==info['actual_value_status']=='converged'
            assert info['wheel_value_status']=='not_computed'
            assert info['wheel_value_cost'] is None
            assert info['actual_value_cost']==pytest.approx(solution['cost'])
            actual=rollout(np.zeros(3),solution['controls'],cfg.dt)
            assert abs(actual[-1,1])<1e-5
            assert info['actual_target_distance_m']-.01001<=actual[-1,0]<=info['actual_target_distance_m']+1e-5
            assert actual[:,0].max()<=depth+1e-5
            results.append((info,solution))
        free=results[0][0]
        assert not free['actual_value_reused_from_wheel'] and not free['lidar_limited']
        assert free['actual_value_status']=='converged'
        for (info,solution),depth in zip(results[1:],(.7,.9)):
            assert info['wheel_distance_m']==pytest.approx(free['wheel_distance_m'],abs=1e-9)
            assert info['wheel_value_cost'] is None
            assert info['actual_target_distance_m']==depth and info['lidar_limited']
            assert not info['actual_value_reused_from_wheel'] and info['actual_value_status']=='converged'
            # In this forward-path fixture a shortened endpoint is worse.
            # This is deliberately not asserted as a theorem for every gap.
            assert info['actual_value_cost']>free['actual_value_cost']
            assert not np.allclose(solution['controls'],results[0][1]['controls'])
    finally:c.close()


def test_actual_endpoint_timeout_never_uses_wheel_cost_as_allocation_cost(monkeypatch):
    import dra_mppi.gap_conditioned as module
    import dra_mppi.gap_optimization as optimization
    monkeypatch.setattr(module,'partition',lambda *args:regions([0.]))
    original_depth=module.directional_depth
    def short_depth(*args):
        angles,strict,hard=original_depth(*args)
        return angles,np.minimum(strict,.7),np.minimum(hard,.7)
    monkeypatch.setattr(module,'directional_depth',short_depth)
    original_solve=optimization.ValueProblem.solve
    def timeout_actual(problem,start,deadline=float('inf')):
        if type(problem) is optimization.ValueProblem and problem.depth==.7:
            raise OptimizationDeadline('forced actual-endpoint timeout')
        return original_solve(problem,start,deadline)
    monkeypatch.setattr(optimization.ValueProblem,'solve',timeout_actual)
    c=controller()
    try:
        c.config.gap.optimization_timeout=1.
        c._guide_optimizer._warm_nominal=np.zeros((10,3))
        r=c.compute(np.zeros(3),[[0.,0.],[5.,0.]],distance_field=field(),gap_scans=(scan(),scan()))
        q=r.gap_diagnostics['candidates'][0]
        assert q['wheel_distance_m'] is not None
        assert q['wheel_value_cost'] is None
        assert q['actual_value_cost'] is None
        assert q['actual_value_status']=='budget_exhausted'
        assert q['optimization_status']=='budget_exhausted'
        assert q['seed_source']=='fallback' and q['allocation_cost_source']=='fallback_seed'
        expected=allocate([r.gap_diagnostics['nominal_cost'],q['cost']],c.config.gap)[0]
        np.testing.assert_array_equal(r.guide_rollout_counts,expected)
    finally:c.close()


def test_production_allocates_actual_cost_without_unused_wheel_value(monkeypatch):
    import dra_mppi.gap_conditioned as module
    monkeypatch.setattr(module,'partition',lambda *args:regions([0.,90.]))
    original_depth=module.directional_depth
    def short_depth(*args):
        angles,strict,hard=original_depth(*args)
        return angles,np.minimum(strict,.7),np.minimum(hard,.7)
    monkeypatch.setattr(module,'directional_depth',short_depth)
    c=controller()
    try:
        c.config.gap.optimization_timeout=1.
        c._guide_optimizer._warm_nominal=np.zeros((10,3))
        r=c.compute(np.zeros(3),[[0.,0.],[5.,0.]],distance_field=field(),gap_scans=(scan(),scan()))
        chosen=sorted([q for q in r.gap_diagnostics['candidates']if q['reason']=='selected'],key=lambda q:q['id'])
        assert len(chosen)==2
        for q in chosen:
            assert q['seed_source']=='optimized'
            assert q['optimization_status']=='converged'
            assert q['allocation_cost_source']=='optimized_actual'
            assert q['cost']==q['actual_value_cost']
            assert q['wheel_distance_m']>.7 and q['actual_target_distance_m']==.7
            assert q['wheel_value_cost'] is None and q['value_difference'] is None
        costs=[r.gap_diagnostics['nominal_cost']]+[q['actual_value_cost']for q in chosen]
        np.testing.assert_array_equal(r.guide_rollout_counts,allocate(costs,c.config.gap)[0])
        assert r.gap_diagnostics['target_policy']=='wheel_then_lidar'
        assert r.gap_diagnostics['allocation_policy']=='cost_softmax_with_reserves'
        assert not r.gap_diagnostics['global_optimality_certified']
        assert r.sampled_controls.shape==(500,10,3) and r.safety_status=='ok'
    finally:c.close()

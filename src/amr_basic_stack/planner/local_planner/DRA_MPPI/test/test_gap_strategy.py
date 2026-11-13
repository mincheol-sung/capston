"""Gap geometry, sensor handling, budget, and repeated-cycle integration."""
from dataclasses import replace
from types import SimpleNamespace
from collections import deque
from unittest.mock import Mock
import time
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter
from dra_mppi.gap_geometry import GapConfig, partition, runs, detect_edges
from dra_mppi.gap_inputs import GapScan, GapInputUnavailable, deskew_scan, fuse_scans
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiConfig, MppiController
from dra_mppi.dynamic_risk import DynamicRiskConfig
from dra_mppi.dra_mppi_node import DraMppiNode
from dra_mppi.geometry import transform_points


def field(occupied=False):
    return DistanceField.from_occupancy(np.full((200,200),100 if occupied else 0),GridMetadata(200,200,.04,-4,-4,0))


def scan(state=(0.,0.,0.)):
    a=np.deg2rad(np.arange(-180.,180.));r=np.full(360,3.)
    return GapScan(r,np.ones(360,bool),transform_points(np.zeros((360,2)),state),
                   transform_points(r[:,None]*np.column_stack((np.cos(a),np.sin(a))),state))


def controller():
    return MppiController(MppiConfig(strategy='gap',static_cost_enabled=True,dt=.2,horizon_steps=10,
        desired_speed=.75,control_min=(-.375,-.5625,-3.),control_max=(.78,.5625,3.),
        acceleration_limit=(2.25,2.25,6.75),speed_limit_l1=.78,yaw_lever=.19835,
        gap=GapConfig(timeout=5.),dynamic_risk=DynamicRiskConfig(mode='off',robot_radius=.2)))


def regions(centers):
    result=[]
    for i,angle in enumerate(centers):
        ids=(int(angle+180)+np.arange(-10,11))%360
        result.append(dict(id=i+1,bins=ids.tolist(),first_angle_deg=int(ids[0])-180,last_angle_deg=int(ids[-1])-180,bin_count=len(ids)))
    return dict(regions=result,fallback=False)


def test_backward_seed_and_selected_nominal_warm_start(monkeypatch):
    import dra_mppi.gap_conditioned as module
    monkeypatch.setattr(module,'partition',lambda *args:regions([-160.,-60.,60.,150.]))
    c=controller();path=np.array([[0.,0.],[5.,0.]])
    try:
        pool=c._guide_optimizer._risk_executor
        first=c.compute(np.zeros(3),path,distance_field=field(),gap_scans=(scan(),scan()))
        assert first.strategy=='gap' and first.sampled_controls.shape==(500,10,3)
        assert sum(first.guide_rollout_counts)==500 and len(first.guide_rollout_counts)<=4
        rear=[g for g in first.gap_diagnostics['candidates'] if abs(g['angle_deg'])>90]
        assert rear and all(g['projected_displacement_m']>=.25 for g in rear)
        assert all(g['reason'] in ('selected','budget_limit','duplicate_selected') for g in rear)
        expected=c._constrain_controls(np.vstack((first.control_sequence[1:],first.control_sequence[-1]))[None],np.zeros(3))[0]
        monkeypatch.setattr(module,'partition',lambda *args:regions([]))
        second=c.compute(np.zeros(3),path,distance_field=field(),gap_scans=(scan(),scan()))
        np.testing.assert_allclose(second.sampled_controls[0],expected)
        assert second.guide_rollout_counts.tolist()==[500]
        assert c._guide_optimizer._risk_executor is pool
        monkeypatch.setattr(module,'partition',lambda *args:regions([-90.,90.]))
        third=c.compute(np.zeros(3),path,distance_field=field(),gap_scans=(scan(),scan()))
        assert sum(third.guide_rollout_counts)==500
        assert c._guide_optimizer._risk_executor is pool
    finally:c.close()


def test_no_safe_candidate_is_hold_not_safe_stop():
    c=controller()
    try:
        r=c.compute(np.zeros(3),np.array([[0.,0.],[3.,0.]]),distance_field=field(True),gap_scans=(scan(),scan()))
        assert r.gap_diagnostics['region_count']==0 and r.gap_diagnostics['fallback']
        assert r.guide_rollout_counts.tolist()==[500]
        assert r.safety_status=='no_safe_candidate_hold'
        assert r.selected_mode=='stop'
    finally:c.close()


def test_missing_scan_resets_instead_of_silent_nominal_fallback():
    c=controller()
    try:
        c._guide_optimizer._warm_nominal=np.ones((10,3))
        with pytest.raises(GapInputUnavailable):c.compute(np.zeros(3),[[0,0],[3,0]],distance_field=field())
        assert c._guide_optimizer._warm_nominal is None
    finally:c.close()


def test_gap_preserves_hybrid_dynamic_verification_and_budget_guard(monkeypatch):
    import dra_mppi.gap_conditioned as module
    from dra_mppi.dynamic_risk import PredictionSet, PredictionBudgetExceeded
    from dra_mppi.gaussian_disk import GaussianDiskCollisionProbability
    monkeypatch.setattr(module, 'partition', lambda *args: regions([-90., 90.]))
    base = controller()
    cfg = replace(base.config, dynamic_risk=DynamicRiskConfig(
        final_mode='gaussian_disk', max_obstacles=20, robot_radius=.2))
    base.close()
    c = MppiController(cfg)
    predictions = PredictionSet(np.broadcast_to([.9, .45], (1, 1, 10, 2)),
        np.broadcast_to([[.05, .025], [.025, .025]], (1, 1, 10, 2, 2)),
        .2*np.arange(1, 11))
    try:
        r = c.compute(np.zeros(3), [[0., 0.], [3., 0.]], dynamic_predictions=predictions,
                      distance_field=field(), gap_scans=(scan(), scan()))
        assert c.risk_diagnostics.total_samples > 0
        assert r.final_risk_diagnostics.sampling_mode == 'gaussian_disk'
        assert r.final_risk_diagnostics.quadrature_evaluations > 0
        assert np.all(r.final_mc_sample_counts == 0)
        expected = GaussianDiskCollisionProbability().evaluate(
            r.trajectory[None, :, :2], predictions, .45)
        assert r.max_collision_probability == pytest.approx(expected.max(), abs=1e-12)
        excess = PredictionSet(np.repeat(predictions.means, 21, axis=0),
            np.repeat(predictions.covariances, 21, axis=0), predictions.times)
        with pytest.raises(PredictionBudgetExceeded):
            c.compute(np.zeros(3), [[0., 0.], [3., 0.]], dynamic_predictions=excess,
                      distance_field=field(), gap_scans=(scan(), scan()))
        assert c._guide_optimizer._warm_nominal is None
        assert not c._compute_lock.locked()
    finally:
        c.close()


def test_expired_gap_proposal_is_rejected_and_resets_warm_start():
    c = controller()
    c.config.gap.timeout = 1e-9
    c._guide_optimizer._warm_nominal = np.ones((10, 3))
    try:
        with pytest.raises(TimeoutError, match='gap proposal deadline'):
            c.compute(np.zeros(3), [[0., 0.], [3., 0.]],
                      distance_field=field(), gap_scans=(scan(), scan()))
        assert c._guide_optimizer._warm_nominal is None
        assert not c._compute_lock.locked()
    finally:
        c.close()


def test_fusion_preserves_obstacles_unknown_and_frame_transform():
    cfg=GapConfig();state=np.array([.4,-.3,.7]);s=scan(state)
    unknown=DistanceField.from_occupancy(np.full((200,200),-1),GridMetadata(200,200,.04,-4,-4,0))
    f,t,_=fuse_scans((s,s),state,unknown,None,cfg,.2)
    assert f.occupancy_at([[0,0]])[0]==0
    assert f.occupancy_at([[3.8,3.8]])[0]==-1
    np.testing.assert_allclose(transform_points(state[:2],t),[0,0],atol=1e-10)
    blocked,_,_=fuse_scans((s,s),state,field(True),None,cfg,.2)
    assert blocked.occupancy_at([[0,0]])[0]==100


def test_circular_and_far_disparity_geometry():
    mask=np.zeros(360,bool);mask[[358,359,0,1]]=True
    assert runs(mask)[0].tolist()==[358,359,0,1]
    r=np.full(360,4.);r[90:270]=6.;a=np.deg2rad(np.arange(-180,180))
    data=dict(strict_depth=np.full(360,2.),hard_depth=np.full(360,2.))
    for i in range(2):
        data.update({f'scan_{i}_ranges':r,f'scan_{i}_valid':np.full(360,i==0),
                     f'scan_{i}_hits':r[:,None]*np.column_stack((np.cos(a),np.sin(a)))})
    assert len(detect_edges(data))==2
    assert partition(data)['region_count']==1
    data['strict_depth'][:]=1.;data['strict_depth'][90:270]=2.
    result=partition(data)
    assert result['region_count']==2
    assert sorted(b for g in result['regions'] for b in g['bins'])==list(range(360))


@pytest.mark.parametrize('kw',[dict(total_rollouts=150),dict(max_guides=4),dict(timeout=float('nan')),dict(radial_step_m=1.),dict(grid_resolution_m=.03)])
def test_invalid_gap_settings_fail(kw):
    with pytest.raises(ValueError):replace(GapConfig(),**kw).validate()


def laser(stamp=9.95):
    msg=LaserScan();sec=int(stamp);msg.header.stamp=Time(sec=sec,nanosec=round((stamp-sec)*1e9))
    msg.header.frame_id='laser';msg.angle_min=-1.;msg.angle_increment=.01
    msg.time_increment=.001;msg.range_min=.05;msg.range_max=10.;msg.ranges=[2.]*100
    return msg


def history():
    return [(round(t,3),t-9.7,0.,0.) for t in np.arange(9.7,10.001,.01)]


def test_deskew_brackets_beams_and_keeps_sensor_offset():
    msg=laser();obs=deskew_scan(msg,history(),(.1,0.,0.),10.)
    assert 45<=sum(obs.valid)<=51 # future half-scan must not be extrapolated
    np.testing.assert_allclose(obs.origins[0],[.35,0.],atol=1e-6)
    assert obs.origins[40,0]>obs.origins[0,0]
    msg.ranges=[float('inf')]*100
    with pytest.raises(GapInputUnavailable):deskew_scan(msg,history(),(.1,0.,0.),10.)


def adapter():
    tf=SimpleNamespace(translation=SimpleNamespace(x=.1,y=0.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
    return SimpleNamespace(_gap_odometry=deque(history()),_gap_scan_buffers=(deque(),deque()),
        _gap_scan_timeout=.25,_gap_scan_max_skew=.05,_gap_base_frame='base_link',
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_100_000_000)),
        tf_buffer=SimpleNamespace(lookup_transform=Mock(return_value=SimpleNamespace(transform=tf))))


def test_pair_selection_accepts_static_tf_and_rejects_stale_skew():
    n=adapter();now=time.monotonic()
    n._gap_scan_buffers[0].append((laser(9.95),now));n._gap_scan_buffers[1].append((laser(9.96),now))
    assert len(DraMppiNode._gap_observations_for_control(n))==2
    n._gap_scan_buffers[1].clear();n._gap_scan_buffers[1].append((laser(9.86),now))
    with pytest.raises(GapInputUnavailable):DraMppiNode._gap_observations_for_control(n)
    n._gap_scan_buffers[1].clear();n._gap_scan_buffers[1].append((laser(9.96),now-1))
    with pytest.raises(GapInputUnavailable):DraMppiNode._gap_observations_for_control(n)


def test_gap_cannot_disable_static_checks_at_runtime():
    n=SimpleNamespace(controller=SimpleNamespace(config=SimpleNamespace(strategy='gap')))
    result=DraMppiNode._validate_runtime_parameters(n,[Parameter('static_cost.enabled',value=False)])
    assert not result.successful
    result=DraMppiNode._validate_runtime_parameters(n,[Parameter('gap.max_guides',value=2)])
    assert not result.successful


@pytest.mark.parametrize('terminal',[False,True])
def test_missing_scans_hold_before_any_motion(terminal):
    from test_guide_node import node
    n=node();n.controller.config.strategy='gap'
    n._gap_observations_for_control=Mock(side_effect=GapInputUnavailable('stale'))
    n.controller.compute=Mock(side_effect=AssertionError('must not optimize'))
    if terminal:n._state=np.array([1.95,0.,0.])
    try:
        DraMppiNode._control_cycle(n)
        assert n._cycle_status=='stale_scan'
        n._publish_stop.assert_called_once();n.controller.compute.assert_not_called()
    finally:n.controller.close()

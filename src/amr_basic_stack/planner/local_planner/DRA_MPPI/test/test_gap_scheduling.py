"""Fair work across interruptions and measured post-proposal time reserves."""
from types import SimpleNamespace
import numpy as np
import pytest
from test_gap_strategy import controller


def specs(angles):
    return [dict(id=f'new_id_{i}',angle=np.deg2rad(a),bins=((int(a+180)+np.arange(-10,11))%360).tolist())
            for i,a in enumerate(angles)]


def test_interrupted_last_gap_goes_first_even_when_ids_reorder():
    c=controller();o=c._guide_optimizer
    try:
        initial=specs([-90,0,90]);state=np.zeros(3)
        _,records=o._schedule_order(initial,state,10.)
        for record in records:record['last_attempt']=1;record['last_finished']=1
        records[2]['last_finished']=-1 # third started but never finished
        o._schedule=records
        changed=specs([0,90,-90])
        order,records=o._schedule_order(changed,state,10.2)
        assert changed[order[0]]['angle']==pytest.approx(np.pi/2)
        # Even if the interrupted candidate alone consumes this whole cycle,
        # it must not monopolize the next one: older unattempted work wins.
        records[order[0]]['last_attempt']=2;o._schedule=records
        order,_=o._schedule_order(changed,state,10.4)
        assert changed[order[0]]['angle']!=pytest.approx(np.pi/2)
    finally:c.close()


def test_unattempted_gap_precedes_repeated_failed_solve_and_pose_changes_reset():
    c=controller();o=c._guide_optimizer
    try:
        s=specs([-90,0,90]);state=np.zeros(3)
        _,records=o._schedule_order(s,state,10.)
        records[0].update(last_attempt=1,last_finished=1) # nonconverged, but finished
        records[1].update(last_attempt=1,last_finished=1)
        o._schedule=records
        order,_=o._schedule_order(s,state,10.2)
        assert order[0]==2
        _,new=o._schedule_order(s,np.array([2.,0.,0.]),10.2)
        assert all(r['last_attempt']==-1 for r in new)
        _,old=o._schedule_order(s,state,11.)
        assert all(r['last_attempt']==-1 for r in old)
    finally:c.close()


def test_reserve_uses_recent_worst_tail_and_resets_when_prediction_load_changes():
    c=controller();o=c._guide_optimizer
    try:
        assert o._mppi_reserve(None)==.10
        o._tail_history.extend([.02,.025])
        assert o._mppi_reserve(None)==.05
        o._tail_history.append(.09)
        assert o._mppi_reserve(None)==pytest.approx(.105)
        p=SimpleNamespace(empty=False,means=np.zeros((1,1,10,2)),mode_weights=np.ones((1,1)))
        assert o._mppi_reserve(p)==.10 and not o._tail_history
        o._tail_history.append(.12)
        assert o._mppi_reserve(p)==pytest.approx(.135)
        o.reset()
        assert not o._schedule and not o._tail_history
    finally:c.close()


def test_timeout_does_not_restart_with_completed_first_gap(monkeypatch):
    import dra_mppi.gap_conditioned as module
    from dra_mppi.gap_optimization import OptimizationDeadline
    from test_gap_strategy import regions,field,scan
    monkeypatch.setattr(module,'partition',lambda *args:regions([-90.,0.,90.]))
    original=module.solve_gap_values;calls=[]
    def expire_last(*args,**kwargs):
        calls.append(args[3])
        if args[3]>1.:raise OptimizationDeadline('forced last candidate interruption')
        return original(*args,**kwargs)
    monkeypatch.setattr(module,'solve_gap_values',expire_last)
    c=controller();c.config.gap.optimization_timeout=1.
    try:
        for _ in range(2):
            c.compute(np.zeros(3),[[0.,0.],[5.,0.]],distance_field=field(),gap_scans=(scan(),scan()))
        np.testing.assert_allclose(calls[:4],[-np.pi/2,0.,np.pi/2,np.pi/2])
    finally:c.close()

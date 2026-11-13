#!/usr/bin/env python3
"""Offline multi-start reach comparison; no production controller changes.

Positive angles are solved at one-degree spacing. Negative directions use the
exact y/omega reflection symmetry of this rest-start, obstacle-free model.
Numerically feasible lower bounds on the nonlinear global reach maximum only.
"""
from pathlib import Path
from dataclasses import replace
import json
import sys
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.gap_geometry import GapConfig
from dra_mppi.gap_optimization import ReachProblem, wheel_distance_bound
from dra_mppi.mecanum_model import rollout

OUT=ROOT/'docs/rotation_reach_2026-11-09'


def verify(u, angle, cfg):
    trajectory=rollout(np.zeros(3),u,cfg.dt)
    e=np.array([np.cos(angle),np.sin(angle)])
    perp=np.array([-e[1],e[0]])
    tol=1e-7
    assert np.max(np.asarray(cfg.control_min)-u)<=tol
    assert np.max(u-np.asarray(cfg.control_max))<=tol
    assert np.max(np.abs(np.diff(np.vstack((np.zeros(3),u)),axis=0))-
                  np.asarray(cfg.acceleration_limit)*cfg.dt)<=tol
    assert np.max(np.abs(u[:,:2]).sum(axis=1)+cfg.yaw_lever*np.abs(u[:,2])-cfg.speed_limit_l1)<=tol
    assert abs(trajectory[-1,:2]@perp)<=tol
    return float(trajectory[-1,:2]@e),trajectory


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    baseline=json.loads((ROOT/'docs/no_rotation_reach_2026-11-09/results.json').read_text())
    cfg=replace(config_from_yaml(),gap=replace(GapConfig(),optimization_maxiter=250,optimization_ftol=1e-9))
    assert cfg.control_min[1]==-cfg.control_max[1] and cfg.control_min[2]==-cfg.control_max[2]
    assert cfg.dt==baseline['dt'] and cfg.horizon_steps==baseline['horizon_steps']
    np.testing.assert_allclose(cfg.control_min[:2],baseline['control_min'])
    np.testing.assert_allclose(cfg.control_max[:2],baseline['control_max'])
    np.testing.assert_allclose(cfg.acceleration_limit[:2],baseline['acceleration'])
    assert cfg.speed_limit_l1==baseline['wheel_budget']
    positive=[]; previous=None; tick=time.perf_counter(); attempts=0; accepted=0
    for degree in range(181):
        angle=np.deg2rad(degree); idx=(degree+180)%360
        zero=np.column_stack((baseline['controls_xy'][idx],np.zeros(cfg.horizon_steps)))
        best_distance,best_path=verify(zero,angle,cfg); best=zero; chosen='zero_rotation_LP'
        problem=ReachProblem(cfg,np.zeros(3),np.array([[0.,0.],[3.,0.]]),angle,wheel_distance_bound(cfg))
        starts=[zero]
        for w in [-2.5,-1.5,-.7,.7,1.5,2.5]:
            u=np.zeros((cfg.horizon_steps,3));u[:,2]=w
            yaw=cfg.dt*np.r_[0.,np.cumsum(u[:-1,2])]
            u[:,0]=.25*np.cos(angle-yaw);u[:,1]=.25*np.sin(angle-yaw)
            starts.append(u)
        if previous is not None:starts.append(previous)
        for start in starts:
            attempts+=1; result=problem.solve(start)
            if not result['success'] or result['violation']>1e-7:continue
            distance,trajectory=verify(result['controls'],angle,cfg);accepted+=1
            if distance>best_distance:
                best_distance=distance;best=result['controls'];best_path=trajectory;chosen='multi_start_SLSQP'
        previous=best.copy()
        positive.append(dict(angle_deg=degree,distance_m=best_distance,controls=best.tolist(),
                             trajectory=best_path.tolist(),source=chosen))
        if degree%45==0:print(f'{degree} deg: {best_distance:.6f} m',flush=True)
    rows=[]
    for degree in range(-180,181):
        data=positive[abs(degree)]
        u=np.array(data['controls'])
        if degree<0:u[:,1:]*=-1
        distance,trajectory=verify(u,np.deg2rad(degree),cfg)
        base=float(baseline['distance_m'][(degree+180)%360])
        assert distance>=base-1e-8
        assert distance<=wheel_distance_bound(cfg)+1e-7
        rows.append(dict(angle_deg=degree,distance_m=distance,no_rotation_m=base,
                         endpoint_yaw_deg=float(np.rad2deg(trajectory[-1,2])),
                         controls=u.tolist(),trajectory=trajectory.tolist(),source=data['source']))
    metadata=dict(initial_control=[0.,0.,0.],dt=cfg.dt,horizon_steps=cfg.horizon_steps,
                  current_yaml=str(ROOT/'config/dra_mppi.yaml'),control_min=list(cfg.control_min),
                  control_max=list(cfg.control_max),acceleration=list(cfg.acceleration_limit),
                  wheel_budget=cfg.speed_limit_l1,yaw_lever=cfg.yaw_lever,
                  method='multi-start SLSQP; zero-rotation LP baseline retained; exact reflection symmetry',
                  global_maximum_certified=False,terminal_stop=False,terminal_heading_constraint=False,
                  lidar_applied=False,solver_attempts=attempts,converged_feasible_solves=accepted,
                  offline_seconds=time.perf_counter()-tick,rows=rows)
    (OUT/'results.json').write_text(json.dumps(metadata,indent=2)+'\n')
    render(rows,cfg)
    print(json.dumps({d:rows[d+180]['distance_m'] for d in [-180,-135,-90,-45,0,45,90,135]}))
    print(f'offline_seconds={metadata["offline_seconds"]:.2f}; attempts={attempts}; accepted={accepted}')


def render(rows,cfg):
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':12})
    angles=np.array([r['angle_deg'] for r in rows]);theta=np.deg2rad(angles)
    reach=np.array([r['distance_m'] for r in rows]);base=np.array([r['no_rotation_m'] for r in rows])
    directions=np.column_stack((np.cos(theta),np.sin(theta)))
    xy=reach[:,None]*directions;old=base[:,None]*directions
    fig=plt.figure(figsize=(15,10),facecolor='#f5f8fc')
    fig.suptitle('회전을 허용한 2초 도달 거리 — 회전 금지와 비교',fontsize=23,fontweight='bold',y=.97)
    fig.text(.5,.918,'정지 출발 · 동일한 바퀴/속도/가속도 제한 · 1° 간격 · 회전 허용은 여러 초기값으로 찾은 최대 거리 추정',ha='center',fontsize=13)
    ax=fig.add_axes([.07,.28,.46,.59]);chart=fig.add_axes([.61,.52,.35,.32])
    ax.fill(xy[:,0],xy[:,1],color='#f9dfc8',alpha=.8)
    ax.fill(old[:,0],old[:,1],color='#daeaf7',alpha=.8)
    ax.plot(xy[:,0],xy[:,1],color='#d87720',lw=3,label='회전 허용 — 찾은 최장 거리')
    ax.plot(old[:,0],old[:,1],'--',color='#237eb4',lw=2,label='회전 금지 (ω = 0)')
    ax.scatter([0],[0],color='#213a50',s=50)
    for deg,label in [(-180,'뒤쪽 180°'),(-90,'우측 −90°'),(0,'앞쪽 0°'),(90,'좌측 +90°')]:
        point=xy[deg+180]
        ax.scatter(*point,color='#d87720',s=40,zorder=5)
        offset={0:(-15,18),90:(0,16),-90:(0,-40),-180:(5,18)}[deg]
        ax.annotate(f'{label}\n{reach[deg+180]:.3f} m',point,xytext=offset,
                    textcoords='offset points',ha='center',fontsize=12,fontweight='bold')
    ax.set(xlim=(-1.55,1.85),ylim=(-1.6,1.6),xlabel='출발 차체 기준 x [m]  전방 + / 후방 −',ylabel='출발 차체 기준 y [m]  좌측 + / 우측 −')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=10)
    chart.plot(angles,reach,color='#d87720',lw=2.5,label='회전 허용')
    chart.plot(angles,base,'--',color='#237eb4',lw=2,label='회전 금지')
    chart.set(xlim=(-180,180),ylim=(.65,1.62),xticks=[-180,-90,0,90,180],xlabel='종단 목표 방향 [°] — 출발 차체 기준',ylabel='도달 거리 [m]')
    chart.grid(alpha=.25);chart.legend(loc='upper right',fontsize=10)
    tax=fig.add_axes([.60,.275,.37,.17]);tax.axis('off')
    cells=[]
    for d in [-180,-135,-90,-45,0,45,90,135]:
        i=d+180;cells.append([f'{d:+d}°',f'{base[i]:.3f}',f'{reach[i]:.3f}',f'+{reach[i]-base[i]:.3f}'])
    table=tax.table(cellText=cells,colLabels=['방향','회전 금지 [m]','회전 허용 [m]','증가 [m]'],loc='center',cellLoc='center')
    table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.22)
    for (r,c),cell in table.get_celld().items():
        cell.set_edgecolor('#d0dbe5')
        if r==0:cell.set_facecolor('#dce8f3')
    fig.text(.07,.173,'공통 제약: |vx| + |vy| + 0.19835 |ω| ≤ 0.780     |ω| ≤ 3 rad/s     |Δω| ≤ 6.75 × 0.2 rad/s',fontsize=12)
    fig.text(.07,.133,'−0.375 ≤ vx ≤ 0.780,  |vy| ≤ 0.5625 m/s     |ax|, |ay| ≤ 2.25 m/s²     Δt = 0.2 s × 10 step',fontsize=12)
    fig.text(.07,.073,'로봇 중심의 종단 위치입니다. 중간 경로는 휘어질 수 있고, 종단 자세·정지 조건 및 LiDAR·장애물 제한은 없습니다.\n'
             '현재 이산 모델의 오프라인 계산이며 실주행 측정값이 아닙니다. 회전 허용 결과는 전역 최대를 보장하지 않습니다.',fontsize=11,color='#4d657a')
    fig.savefig(OUT/'directional_reach_rotation_comparison.png',dpi=180,facecolor=fig.get_facecolor());plt.close(fig)


if __name__=='__main__':main()

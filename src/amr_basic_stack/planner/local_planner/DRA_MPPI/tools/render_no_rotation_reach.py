#!/usr/bin/env python3
"""Offline omega=0 reach boundary under the production discrete input limits.

LP is used only to verify this figure, not introduced into the runtime planner.
The endpoint lies on each requested ray; intermediate velocity directions may
vary, while body yaw stays fixed. Start at rest, no terminal stop constraint.
"""
from pathlib import Path
import json
import numpy as np
import yaml
from scipy.optimize import linprog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/no_rotation_reach_2026-11-09'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    p = yaml.safe_load((ROOT/'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    dt, n = p['mppi.dt'], p['mppi.horizon_steps']
    lo = np.array(p['limits.control_min'][:2])
    hi = np.array(p['limits.control_max'][:2])
    acc = np.array(p['limits.acceleration'][:2])
    budget = p['limits.wheel_radius']*p['limits.max_wheel_rad_per_sec']*p['limits.load_budget']
    rows, rhs = [], []
    for k in range(n):
        for signs in ((1,1),(1,-1),(-1,1),(-1,-1)):
            row = np.zeros(2*n); row[2*k:2*k+2] = signs
            rows.append(row); rhs.append(budget)
        for j in range(2):
            row = np.zeros(2*n); row[2*k+j] = 1
            if k: row[2*(k-1)+j] = -1
            rows.extend([row, -row]); rhs.extend([acc[j]*dt]*2)
    A, b = np.array(rows), np.array(rhs)
    angles = np.arange(-180.,180.)
    distances, straight, controls = [], [], []
    for degrees in angles:
        angle = np.deg2rad(degrees)
        e = np.array([np.cos(angle), np.sin(angle)])
        perp = np.array([-e[1], e[0]])
        cost = -dt*np.tile(e,n)
        eq = dt*np.tile(perp,n)[None,:]
        result = linprog(cost, A_ub=A, b_ub=b, A_eq=eq, b_eq=[0.],
                         bounds=list(zip(lo,hi))*n, method='highs')
        assert result.success, result.message
        assert np.max(A@result.x-b) < 1e-8
        assert abs(float((eq@result.x)[0])) < 1e-8
        distance = -result.fun
        distances.append(distance); controls.append(result.x.reshape(n,2).tolist())
        # A simpler straight-motion reach rule is feasible and cannot exceed LP.
        speed = budget/np.abs(e).sum()
        acceleration = float('inf')
        for j in range(2):
            if abs(e[j]) > 1e-10:
                speed = min(speed, (hi[j] if e[j]>0 else -lo[j])/abs(e[j]))
                acceleration = min(acceleration,acc[j]/abs(e[j]))
        straight.append(float(dt*np.minimum(speed,acceleration*dt*np.arange(1,n+1)).sum()))
    distances = np.array(distances)
    assert np.all(np.array(straight) <= distances+1e-8)
    for deg, expected in ((0,1.494),(90,1.1025),(-90,1.1025),(-180,.75)):
        assert abs(distances[int(deg+180)]-expected) < 1e-8
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':12})
    fig = plt.figure(figsize=(15,10),facecolor='#f5f8fc')
    fig.suptitle('회전 없이 2초 동안 도달할 수 있는 방향별 최대 거리',fontsize=23,fontweight='bold',y=.97)
    fig.text(.5,.915,'정지 출발 · ω = 0 · 차체 방향 고정 · 현재 YAML의 속도/가속도/바퀴 제한 적용',ha='center',fontsize=14)
    ax = fig.add_axes([.07,.28,.46,.59]); chart = fig.add_axes([.61,.52,.35,.32])
    theta = np.deg2rad(angles); xy = distances[:,None]*np.column_stack((np.cos(theta),np.sin(theta)))
    closed = np.vstack((xy,xy[0]))
    ax.fill(closed[:,0],closed[:,1],color='#d8e9f5')
    ax.plot(closed[:,0],closed[:,1],color='#237eb4',lw=3)
    ax.scatter([0],[0],s=55,color='#213a50',zorder=5)
    cardinals = [(-180,'후진 180°'),(-90,'우측 −90°'),(0,'전진 0°'),(90,'좌측 +90°')]
    for deg,label in cardinals:
        point = xy[deg+180]; distance = distances[deg+180]
        ax.plot([0,point[0]],[0,point[1]],'--',color='#657989',lw=1.4)
        ax.scatter(*point,s=45,color='#237eb4',zorder=5)
        offset = {0:(-12,16),90:(0,14),-90:(0,-40),-180:(-8,18)}[deg]
        ax.annotate(f'{label}\n{distance:.3f} m',point,xytext=offset,textcoords='offset points',
                    ha='center',fontsize=13,fontweight='bold')
    ax.set(xlim=(-1.35,1.85),ylim=(-1.55,1.55),xlabel='x [m]  전방 + / 후방 −',ylabel='y [m]  좌측 + / 우측 −')
    ax.set_aspect('equal'); ax.grid(alpha=.2)
    chart.plot(angles,distances,lw=2.5,color='#237eb4')
    chart.set(xlim=(-180,180),ylim=(.65,1.62),xticks=[-180,-90,0,90,180],xlabel='이동 목표 방향 [°]',ylabel='최대 도달 거리 [m]')
    chart.grid(alpha=.25)
    table_ax = fig.add_axes([.62,.28,.32,.16]); table_ax.axis('off')
    cells = [[f'{d:+d}°',f'{distances[d+180]:.3f} m'] for d in [-180,-135,-90,-45,0,45,90,135]]
    table = table_ax.table(cellText=cells,colLabels=['방향','도달 거리'],loc='center',cellLoc='center',colWidths=[.45,.55])
    table.auto_set_font_size(False); table.set_fontsize(11);table.scale(1,1.22)
    for (r,c),cell in table.get_celld().items():
        cell.set_edgecolor('#d0dbe5')
        if r==0:cell.set_facecolor('#dce8f3')
    fig.text(.07,.175,'바퀴 제한: |vx| + |vy| ≤ 0.780 m/s      축 제한: −0.375 ≤ vx ≤ 0.780,  |vy| ≤ 0.5625 m/s',fontsize=13)
    fig.text(.07,.135,'가속도: |ax|, |ay| ≤ 2.25 m/s²      시간 모델: Δt = 0.2 s × 10 step      종단 정지 조건 없음',fontsize=13)
    fig.text(.07,.08,'파란 경계는 로봇 중심의 종단 도달 위치입니다. 차체는 돌리지 않지만 중간 병진 방향은 바뀔 수 있습니다.\n'
             'LiDAR·지도·장애물 미적용. 실제 현재 속도 대신 정지 출발을 가정한 설정 기반 계산이며 실주행 측정값이 아닙니다.',fontsize=11,color='#4d657a')
    fig.savefig(OUT/'directional_reach_omega_zero.png',dpi=180,facecolor=fig.get_facecolor())
    plt.close(fig)
    data=dict(initial_control=[0.,0.,0.],omega_fixed=0.,dt=dt,horizon_steps=n,
              control_min=lo.tolist(),control_max=hi.tolist(),acceleration=acc.tolist(),wheel_budget=budget,
              endpoint_ray_only=True,terminal_stop=False,lidar_applied=False,
              method='offline linear programming; not a runtime planner change',
              angles_deg=angles.tolist(),distance_m=distances.tolist(),
              straight_motion_distance_m=straight,controls_xy=controls)
    (OUT/'results.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({d:round(float(distances[d+180]),6) for d in [-180,-135,-90,-45,0,45,90,135]}))
    print(OUT/'directional_reach_omega_zero.png')


if __name__=='__main__':main()

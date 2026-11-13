"""Replay saved b2_060 seeds and render every actual MPPI sample and input.

The 500 samples are recovered from returned sampled_controls, not the quota-
limited visualization subset. All samples, including rejected ones, are shown.
"""
from pathlib import Path
from dataclasses import replace
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager, patheffects
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
from matplotlib.lines import Line2D

from experiment_gap_value_optimization import PreparedOptimizer, geometry, serial
from dra_mppi.mppi_controller import MppiController, MppiConfig
from dra_mppi.dynamic_risk import DynamicRiskConfig, PredictionSet
from dra_mppi.gap_geometry import GapConfig
from dra_mppi.gap_inputs import fuse_scans
from dra_mppi.mecanum_model import rollout

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'docs/gap_value_optimization_2026-11-05'
OUT=SOURCE/'nominal_distribution_b2_060'
INK='#20364b';MUTED='#536b7e';NOM='#191c24';UPDATED='#bb268a';FINAL='#e34832'
GROUP_COLORS=['#77849d','#c47b16','#3383ba','#009c91']


def main():
    OUT.mkdir(exist_ok=True)
    experiment=json.loads((SOURCE/'configuration.json').read_text())
    values=experiment['controller'].copy()
    values['gap']=GapConfig(**values['gap']);values['dynamic_risk']=DynamicRiskConfig(**values['dynamic_risk'])
    cfg=MppiConfig(**values)
    data=dict(np.load(SOURCE/'raw/b2_060.npz'))
    saved=dict(np.load(SOURCE/'b2_060_results.npz'))
    summary=json.loads((SOURCE/'b2_060_summary.json').read_text())
    selected=[i for i,g in enumerate(summary['candidates']) if g['reason']=='selected']
    selected.sort(key=lambda i:summary['candidates'][i]['id'])
    names=['nominal']+[summary['candidates'][i]['id'] for i in selected]
    counts=np.array(summary['counts'])
    c=MppiController(replace(cfg,strategy='standard'))
    try:
        c.config.strategy='guide_conditioned';c._guide_optimizer=PreparedOptimizer(c)
        field,scans=geometry(data,cfg.gap,cfg.static_cost_robot_radius)
        fused,_,_=fuse_scans(scans,np.zeros(3),field,None,cfg.gap,cfg.static_cost_robot_radius)
        current=c._project_wheel_limit(np.clip(data['current'],cfg.control_min,cfg.control_max)[None])[0]
        nominal=c._constrain_controls(data['nominal'][None],current)[0]
        anchors=np.array([nominal]+[saved['seed_controls'][i] for i in selected])
        c._guide_optimizer.set_proposals(anchors,names,counts)
        predictions=PredictionSet(data['prediction_means'],data['prediction_covariances'],data['prediction_times'],
                    mode_weights=data.get('prediction_mode_weights'),obstacle_radii=data.get('prediction_obstacle_radii'))
        result=c.compute(np.zeros(3),data['path'].copy(),current.copy(),predictions,
                         visualization_rollouts=0,distance_field=fused)
    finally:c.close()
    controls=result.sampled_controls
    paths=rollout(np.zeros(3),controls,cfg.dt)
    ids=result.control_mode_ids
    np.testing.assert_array_equal(np.bincount(ids,minlength=len(names)),counts)
    np.testing.assert_allclose(result.trajectory,saved['selected_path'],atol=1e-8)
    np.testing.assert_allclose(rollout(np.zeros(3),nominal,cfg.dt),saved['nominal_path'],atol=1e-12)
    assert len(paths)==500 and counts[0]==200
    nominal_mask=ids==0;finite=np.isfinite(result.rollout_costs)
    nominal_paths=paths[nominal_mask];nominal_controls=controls[nominal_mask]
    # Preserve the unconcealed origin and include the start of every curve.
    full_paths=np.concatenate((np.zeros((len(paths),1,2)),paths[:,:,:2]),axis=1)
    nominal_path=np.vstack(([0,0],saved['nominal_path'][:,:2]))
    representative=np.vstack(([0,0],result.mode_trajectories[0,:,:2]))
    final_path=np.vstack(([0,0],result.trajectory[:,:2]))

    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
                         'axes.unicode_minus':False,'font.size':12,'mathtext.fontset':'dejavusans'})
    halo=[patheffects.Stroke(linewidth=5,foreground='white'),patheffects.Normal()]

    def map_base(ax,limits):
        view=np.zeros_like(data['grid']);view[data['grid']<0]=1
        view[data['map_fill']&(data['grid']==0)]=2;view[data['grid']>=65]=3
        ax.imshow(view,extent=(-4,4,-4,4),origin='lower',vmin=0,vmax=3,
                  cmap=ListedColormap(['white','#d8dfe7','#e5eefb','#34495e']),interpolation='nearest')
        for i in range(2):
            hits=data[f'scan_{i}_hits'][data[f'scan_{i}_valid']]
            ax.scatter(hits[:,0],hits[:,1],s=3,color=('#b97e43','#577d9b')[i],alpha=.65,zorder=2)
        ax.add_patch(Circle((0,0),.2,fill=False,ec=INK,ls=':',lw=1.5,zorder=8))
        ax.set(xlim=limits[0],ylim=limits[1],aspect='equal',xlabel='x [m] · 전방 + / 후방 −',ylabel='y [m] · 좌측 + / 우측 −')
        ax.grid(alpha=.12)

    def nominal_marker(ax):
        ax.scatter(0,0,marker='*',s=180,facecolor=NOM,edgecolor='white',linewidth=1.1,zorder=20)
        ax.annotate('입력 nominal: 정지 (0, 0)',xy=(0,0),xytext=(-.85,.65),fontsize=11,color=NOM,
                    bbox=dict(fc='white',ec='#aaa',alpha=.95,pad=3),
                    arrowprops=dict(arrowstyle='->',color=NOM),zorder=21)

    fig=plt.figure(figsize=(18,11),facecolor='#f6f8fb')
    fig.text(.045,.95,'b2_060 · nominal과 실제 MPPI 샘플 궤적',fontsize=26,weight='bold',color=INK)
    fig.text(.045,.908,'이 장면의 입력 nominal은 10단계 모두 [vx, vy, ω] = [0, 0, 0]입니다. 이전 그림에서는 원점의 로봇 표시 아래에 가려져 있었습니다.',fontsize=13,color=MUTED)
    ax=fig.add_axes([.055,.25,.43,.58]);map_base(ax,((-2.,2.1),(-1.6,1.6)))
    for g,name in enumerate(names):
        mask=ids==g
        ax.add_collection(LineCollection(full_paths[mask],colors=GROUP_COLORS[g],linewidths=.7,alpha=.20,zorder=3))
        if g:
            xy=np.vstack(([0,0],result.guide_anchors[g,:,:2]))
            ax.plot(xy[:,0],xy[:,1],'--',lw=2.3,color=GROUP_COLORS[g],path_effects=halo,zorder=9)
    ax.plot(final_path[:,0],final_path[:,1],color=FINAL,lw=3,zorder=11,path_effects=halo)
    nominal_marker(ax)
    handles=[Line2D([0],[0],color=GROUP_COLORS[g],lw=2,label=f'{name} 그룹: {counts[g]}개')for g,name in enumerate(names)]
    handles.append(Line2D([0],[0],color=FINAL,lw=3,label=f'최종 선택: {result.selected_mode}'))
    ax.legend(handles=handles,loc='lower left',fontsize=10,framealpha=.96)
    ax.set_title('전체 500개 · 그룹별 실제 샘플 (탈락 샘플도 포함)',fontsize=14,pad=12)

    ax=fig.add_axes([.55,.25,.40,.58]);map_base(ax,((-1.,1.),(-.85,.85)))
    # Explicitly show invalid samples rather than silently dropping them.
    valid=nominal_mask & finite;invalid=nominal_mask & ~finite
    ax.add_collection(LineCollection(full_paths[valid],colors='#477cc7',linewidths=.9,alpha=.27,zorder=4))
    ax.add_collection(LineCollection(full_paths[invalid],colors='#cf6b62',linewidths=.9,alpha=.25,zorder=4))
    ax.scatter(paths[nominal_mask,-1,0],paths[nominal_mask,-1,1],s=9,color='#477cc7',alpha=.5,zorder=5)
    ax.plot(representative[:,0],representative[:,1],color=UPDATED,lw=3,path_effects=halo,zorder=12)
    nominal_marker(ax)
    handles=[Line2D([0],[0],color='#477cc7',lw=1.8,label=f'1차 비용 유효: {int(valid.sum())}개'),
             Line2D([0],[0],color='#cf6b62',lw=1.8,label=f'1차 평가 탈락: {int(invalid.sum())}개'),
             Line2D([0],[0],marker='*',color=NOM,ls='none',markersize=12,label='최적화 전 nominal (정지)'),
             Line2D([0],[0],color=UPDATED,lw=3,label='MPPI 갱신 후 nominal 그룹 대표 궤적')]
    ax.legend(handles=handles,loc='lower left',fontsize=9,framealpha=.96)
    ax.set_title('nominal 그룹 200개만 확대 · 작은 점은 2초 후 위치',fontsize=14,pad=12)
    endpoint=representative[-1]
    fig.text(.055,.16,f'입력 nominal은 움직이지 않아도, 주변 제어입력에 노이즈를 주면 200개 후보 궤적이 생깁니다. nominal 그룹 대표 종단: ({endpoint[0]:.3f}, {endpoint[1]:.3f})m.',fontsize=13,color=INK)
    fig.text(.055,.115,f'검은 별: 원래 nominal / 자홍색: nominal 그룹을 MPPI로 갱신한 결과 / 빨간 선: 전체 그룹 중 최종 선택 {result.selected_mode}',fontsize=12,color=INK)
    fig.text(.055,.072,'실제 bag mpc_chain_20260923_135606 +129.335s · 동적 예측 3개 · 저장된 동일 설정·seed로 재계산 · 이전 최종 궤적 일치 확인',fontsize=10,color=MUTED)
    fig.text(.055,.04,'원점의 점선 원은 로봇 반경 0.20m입니다. 안전성을 보장하는 분포가 아니라 생성·평가된 모든 샘플을 표시했습니다. 운영 코드 변경 없음.',fontsize=10,color=MUTED)
    fig.savefig(OUT/'01_nominal_and_500_rollouts.png',dpi=160);plt.close(fig)

    fig,axes=plt.subplots(3,1,figsize=(16,11),sharex=True,facecolor='#f6f8fb')
    edges=np.arange(cfg.horizon_steps+1)*cfg.dt
    for axis,ax in enumerate(axes):
        for u in nominal_controls:
            ax.stairs(u[:,axis],edges,baseline=None,color='#477cc7',lw=.6,alpha=.10)
        ax.stairs(nominal[:,axis],edges,baseline=None,color=NOM,lw=3,ls='--',label='입력 nominal: 0',zorder=10)
        ax.stairs(result.mode_control_sequences[0,:,axis],edges,baseline=None,color=UPDATED,lw=2.5,label='갱신된 nominal 그룹 대표 입력',zorder=11)
        ax.set_ylabel(['vx [m/s]','vy [m/s]','ω [rad/s]'][axis]);ax.grid(alpha=.18)
        ax.set_xlim(0,2)
    axes[0].legend(loc='upper right',fontsize=10)
    axes[-1].set_xlabel('예측 시간 [s] · 각 입력을 0.2초 동안 유지')
    fig.suptitle('nominal 제어입력과 주변 200개 MPPI 샘플의 제어입력',fontsize=24,color=INK,y=.97)
    fig.text(.07,.91,'검은 점선은 세 축 모두 0입니다. 파란 얇은 선은 동일한 200개 후보의 실제 제약 적용 후 입력이며, 자홍색은 MPPI 갱신 결과입니다.',fontsize=12,color=MUTED)
    fig.text(.07,.065,'vx·vy는 각 시각의 차체 좌표 속도, ω는 차체 회전 속도입니다. 공간 궤적은 이 세 입력을 메카넘 모델로 적분한 결과입니다.',fontsize=11,color=MUTED)
    fig.text(.07,.035,'b2_060 동일 재생 · 10 steps × 0.2s · 속도·가속도·wheel L1 제한 적용 · 최종 선택 그룹은 '+result.selected_mode,fontsize=11,color=MUTED)
    fig.subplots_adjust(left=.09,right=.96,bottom=.14,top=.86,hspace=.18)
    fig.savefig(OUT/'02_nominal_control_distribution.png',dpi=160);plt.close(fig)

    np.savez_compressed(OUT/'replay_arrays.npz',nominal_controls=nominal,nominal_path=saved['nominal_path'],
        sampled_controls=controls,sampled_paths=paths,group_ids=ids,counts=counts,
        rollout_costs=result.rollout_costs,weights=result.weights,
        representative_controls=result.mode_control_sequences,representative_paths=result.mode_trajectories,
        final_controls=result.control_sequence,final_path=result.trajectory)
    with (OUT/'nominal_controls.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['step','start_s','end_s','vx_mps','vy_mps','omega_radps'])
        for t,u in enumerate(nominal):writer.writerow([t,edges[t],edges[t+1],*u])
    report=dict(scene='b2_060',nominal_all_zero=bool(np.all(nominal==0)),current_measured=data['current'],
                groups=names,counts=counts,all_sample_count=len(paths),nominal_sample_count=int(nominal_mask.sum()),
                nominal_primary_finite_count=int(valid.sum()),nominal_primary_rejected_count=int(invalid.sum()),
                nominal_representative_endpoint=representative[-1],nominal_representative_final_cost=result.mode_final_costs[0],
                final_selected=result.selected_mode,final_cost=result.cost,nominal_controls=nominal,
                reproduces_previous_final_trajectory=True,
                scope='Same saved recorded scene, configuration and optimized seeds. New real MPPI replay; full sampled_controls rolled out, no visualization subsampling; includes rejected samples.')
    (OUT/'results.json').write_text(json.dumps(serial(report),ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(serial(report),ensure_ascii=False,indent=2))


if __name__=='__main__':main()

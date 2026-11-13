"""Detailed, actual-data PNG atlas of the finalized gap v1 selection procedure.

All figures derive from real recorded snapshots and frozen solver replay.
Source ROS and workspace setup. No ROS graph or robot commands are used.
"""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
from matplotlib.patches import Circle,Polygon
from scipy.ndimage import gaussian_filter1d
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_multiscene_2026-11-04'
OUT=BASE/'gap_v1_actual_flow'
sys.path.insert(0,str(BASE/'source_snapshot/tools'))
import experiment_gap_multiscene as ex
from experiment_gap_diversity import options,hard_depth

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=FONT).get_name(),
    'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False,
    'figure.facecolor':'#f7f9fc','savefig.facecolor':'#f7f9fc'})
INK='#19364e';MUTED='#526c82';NOM='#2f67ba';RED='#c04c57';GREEN='#008d79'
COL=['#b46082','#8d56b1','#cf8827','#3898b2','#7e8791',GREEN,'#b65845','#89733f','#4a667d']
GAPCOL=['#8d56b1',GREEN,'#b38134']
REASONS={'selected':'선택','duplicate_gap':'선택 후보와 중복','duplicate_nominal':'nominal 중복',
         'collision':'몸체 충돌','unreachable':'도달 부족','capacity':'3개 상한'}
FILES=[]


def readlines(path):return [json.loads(s) for s in path.read_text().splitlines()]


def collect():
    ids=['b0_029','b0_033','b1_004','b1_088']
    records={a['record_ns']:a for a in readlines(BASE/'scene_audit.jsonl') if a['accepted'] and a['scene'] in ids}
    prior={r['scene']:r for r in readlines(BASE/'diversity_and_empty_gap/trials.jsonl')
           if r['method']=='all_options_hard_fallback' and r['seed']==7}
    scenes={};cfg=ex.config_from_yaml()
    for bag in ex.BAGS[:2]:
        print('READ',bag,flush=True);data=ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/bag)
        for snap in data['snapshots']:
            if snap['record_ns'] not in records:continue
            audit=records[snap['record_ns']];name=audit['scene'];p=ex.prepare_snapshot(data,snap)
            ex.input_check(data,snap,p)
            with np.load(BASE/'scenes'/f'{name}.npz') as old:
                for key in ['grid','angles','d_free','full_path']:np.testing.assert_allclose(p[key],old[key],atol=1e-10)
            c=ex.MppiController(replace(cfg,strategy='standard'));c.gap_path=p['path']
            nominal=c._constrain_controls(p['nominal'][None],p['current'])[0]
            nominal_path=ex.rollout(np.zeros(3),nominal,.2)
            hard=hard_depth(p)[0];strict_defs=ex.definitions(p,'valley')
            fallback=not strict_defs;pp={**p,'d_free':hard} if fallback else p
            defs=ex.definitions(pp,'valley');pool,_=options(pp,c,nominal_path)
            nomJ=float(ex.cheap_cost(c,p,nominal[None],nominal_path[None])[0])
            items,chosen,_=ex.select(pp,pool,'cost',audit['nominal_collision'],nomJ,True)
            for i,g in enumerate(items):
                g['label']=chr(65+i);g['color']=COL[i%len(COL)]
                unit=np.array([np.cos(g['angle']),np.sin(g['angle'])]);available=max(.1,pp['d_free'][g['index']]-.1)
                far=ex.seed_control(c,available*unit,p['current'])
                g['available_depth']=available
                g['projected_reach']=float(ex.rollout(np.zeros(3),far,.2)[-1,:2]@unit)
                # All source predicates are evaluated, even if an earlier gate already rejects.
                g['nominal_duplicate']=(not audit['nominal_collision'] and g['rms']<.22 and g['tail']<.30)
                terms=ex.common_costs(cfg,np.zeros(3),p['path'],g['controls'][None],g['trajectory'][None],p['current'],return_components=True)
                static,_=c._static_obstacle_cost(np.zeros(3),g['trajectory'][None],p['field'],None)
                g['components']={k:float(v[0]) for k,v in terms[-1].items()}
                g['components']['static']=float(static[0])
                if np.isfinite(g['cheap_cost']):np.testing.assert_allclose(sum(g['components'].values()),g['cheap_cost'],atol=1e-9)
            # ex.select returns shared dicts, so labels propagate into chosen.
            chosen=sorted(chosen,key=lambda g:g['id'])
            predictions=ex.recorded_predictions(p,c)
            run_cfg=replace(cfg,seed=7,guide_timeout=10.,strategy='standard',guide_angles_deg=tuple(float(i) for i in range(1+len(chosen))))
            solver=ex.MppiController(run_cfg);solver.config.strategy='guide_conditioned'
            counts=[500-100*len(chosen)]+[100]*len(chosen)
            names=['nominal']+[g['id'] for g in chosen]
            solver._guide_optimizer=ex.SnapshotGapOptimizer(solver,np.array([nominal]+[g['controls'] for g in chosen]),names,counts)
            try:
                result=solver.compute(np.zeros(3),p['path'],p['current'],predictions,visualization_rollouts=500,distance_field=p['field'])
            finally:solver.close();c.close()
            expected=np.array([np.inf if j is None else j for j in prior[name]['final_costs']])
            np.testing.assert_allclose(result.mode_final_costs,expected,atol=1e-9,rtol=1e-9)
            with np.load(BASE/'diversity_and_empty_gap/scenes'/f'{name}.npz') as old:
                np.testing.assert_allclose(result.trajectory,old['all_options_hard_fallback_trajectory'],atol=1e-9)
            assert result.selected_mode==prior[name]['selected'] and result.safety_status==prior[name]['status']
            np.testing.assert_array_equal(result.control_mode_ids,np.repeat(np.arange(len(counts)),counts))
            if result.safety_status=='ok':
                assert not p['field'].trajectory_collisions(result.trajectory[None,:,:2],np.zeros(2),ex.RADIUS)[0]
                assert result.max_collision_probability<=cfg.dynamic_risk.hard_threshold
            trace=[];selected_so_far=[]
            for g in sorted(items,key=lambda g:g['cheap_cost']):
                d=[(h['label'],float(np.sqrt(np.mean(np.sum((g['trajectory'][:,:2]-h['trajectory'][:,:2])**2,axis=1))))) for h in selected_so_far]
                trace.append(dict(label=g['label'],cost=g['cheap_cost'],reason=g['reason'],distances=d))
                if g['reason']=='selected':selected_so_far.append(g)
            scenes[name]=dict(p=p,pp=pp,hard=hard,strict_defs=strict_defs,defs=defs,items=items,chosen=chosen,
                nominal=nominal,nominal_path=nominal_path,result=result,counts=counts,trace=trace,
                audit=audit,prior=prior[name],cfg=cfg,predictions=predictions,fallback=fallback)
            print('VERIFIED',name,'gaps',len(defs),'selected',[g['label'] for g in chosen],flush=True)
    assert len(scenes)==4
    return scenes


def path(ax,tr,color,label=None,lw=2.5,style='-',alpha=1):
    xy=np.vstack((np.zeros(2),tr[:,:2]));ax.plot(xy[:,0],xy[:,1],style,color=color,lw=lw,alpha=alpha,label=label,zorder=6)


def gridplot(ax,s,kind='fused',pathline=True):
    p=s['p'];grid=p['scan_grid'] if kind=='scan' else p['grid'];v=np.zeros_like(grid)
    v[grid<0]=1
    if kind!='scan':v[p['map_fill']&(grid==0)]=2
    v[grid>=65]=3
    ax.imshow(v,extent=(-4,4,-4,4),origin='lower',vmin=0,vmax=3,
        cmap=ListedColormap(['white','#d6dee7','#e2ecfc','#283f54']),interpolation='nearest')
    for scan,col in zip(p['scans'],['#bf7737','#4e78a2']):
        hits=scan['hits'][scan['valid']];ax.scatter(hits[:,0],hits[:,1],s=2,color=col,alpha=.65,zorder=3)
    if pathline:ax.plot(p['full_path'][:,0],p['full_path'][:,1],'--',lw=1.6,color='#6294b1',label='기록 global path')
    ax.add_patch(Circle((0,0),.2,facecolor=INK,edgecolor='white',zorder=10))
    ax.annotate('',(.4,0),(0,0),arrowprops=dict(arrowstyle='->',color=INK,lw=1.8),zorder=11)
    ax.set(xlim=(-2.25,2.25),ylim=(-2.25,2.25),aspect='equal',xlabel='전방 x [m]',ylabel='좌측 y [m]')
    ax.grid(alpha=.13)


def sectors(ax,s):
    p=s['pp']
    for i,g in enumerate(s['defs']):
        ids=g['ids'];a=p['angles'][ids];r=p['d_free'][ids];xy=np.column_stack((r*np.cos(a),r*np.sin(a)))
        col=GAPCOL[i%len(GAPCOL)]
        ax.add_patch(Polygon(np.vstack((np.zeros(2),xy)),facecolor=col,edgecolor=col,alpha=.18))
        ax.plot(xy[:,0],xy[:,1],color=col,lw=1.8)
        mid=ids[len(ids)//2];radius=.72*p['d_free'][mid]
        ax.text(radius*np.cos(p['angles'][mid]),radius*np.sin(p['angles'][mid]),f'G{i+1}',
                color=col,fontsize=16,weight='bold',bbox=dict(facecolor='white',edgecolor='none',alpha=.8))


def shell(number,title,subtitle,s):
    fig=plt.figure(figsize=(16,9),dpi=160)
    fig.text(.045,.94,f'{number:02d}  {title}',fontsize=23,weight='bold',color=INK)
    fig.text(.045,.895,subtitle,fontsize=12,color=MUTED)
    fig.text(.045,.065,f'실측 {s["audit"]["bag"]}  |  +{s["audit"]["offset"]:.6f} s  |  {s["audit"]["scene"]}  |  noise seed 7',fontsize=10,color=MUTED)
    fig.text(.045,.032,'흰색: scan free · 연파랑: 지도 보완 free · 회색: 미확인 · 진한색: 장애물  /  실측 입력의 오프라인 계산, 실제 주행 명령 아님',fontsize=9.5,color=MUTED)
    return fig


def paired(number,title,subtitle,s):
    fig=shell(number,title,subtitle,s)
    ax=fig.add_axes([.055,.16,.46,.66]);right=fig.add_axes([.555,.16,.405,.66]);right.axis('off')
    return fig,ax,right


def note(ax,text,y=.97,size=13):
    ax.text(0,y,text,va='top',fontsize=size,color=INK,linespacing=1.55,transform=ax.transAxes)


def table(ax,rows,headers,bbox=(0,.25,1,.66),size=10.5,widths=None):
    t=ax.table(cellText=rows,colLabels=headers,cellLoc='center',bbox=bbox,colWidths=widths)
    t.auto_set_font_size(False);t.set_fontsize(size)
    for (r,c),cell in t.get_celld().items():
        cell.set_edgecolor('#d3dfea');cell.set_facecolor('#e7eef8' if r==0 else 'white')
    return t


def save(fig,name):
    fig.savefig(OUT/name,dpi=160);plt.close(fig);FILES.append(name)


def main_figures(s):
    p=s['p'];items=s['items'];r=s['result'];cfg=s['cfg']
    fig,ax,side=paired(1,'실제 앞·뒤 LiDAR 관측','원시 range와 시간 정렬한 hit 위치를 함께 확인',s)
    gridplot(ax,s,'scan',False)
    scanax=fig.add_axes([.58,.48,.36,.32])
    for scan,col,label in zip(p['scans'],['#bf7737','#4e78a2'],['앞 LiDAR','뒤 LiDAR']):
        valid=scan['valid'];scanax.plot(np.rad2deg(scan['angles'][valid]),scan['ranges'][valid],'.',ms=1.3,color=col,label=label)
    scanax.set(xlabel='각 센서 자체의 beam 각도 [°]',ylabel='측정 range [m]');scanax.legend(fontsize=9);scanax.grid(alpha=.15)
    note(side,'왼쪽: deskew 후 현재 base 좌표의 실제 hit\n오른쪽: 센서 각도별 유효 원시 range\n\n'+ '\n'.join(f'{label}: 유효 beam {int(scan["valid"].sum())}/{len(scan["ranges"])}개' for scan,label in zip(p['scans'],['앞','뒤']))+
         '\n미측정·무효 beam을 열린 공간으로 채우지 않습니다.',y=.40,size=12)
    save(fig,'01_lidar_observations.png')

    fig=shell(2,'scan 관측과 기록 지도 보완을 구분','각 지도는 같은 snapshot의 실제 데이터 · 미확인 영역은 계속 차단',s)
    for i,(kind,title) in enumerate([('scan','① scan만으로 만든 점유 상태'),('fused','② 기록 지도 보완 후'),('fused','③ 최종 로봇 중심 clearance')]):
        ax=fig.add_axes([.055+i*.315,.25,.28,.56]);gridplot(ax,s,kind,False);ax.set_title(title,fontsize=13)
        if i==2:
            # Actual distance field, with diagonal and sweep padding removed.
            clear=p['field'].clearance-np.sqrt(2)*.04-.01
            ax.contour(np.linspace(-3.98,3.98,200),np.linspace(-3.98,3.98,200),clear,
                       levels=[.20,.25,.40],colors=[RED,GREEN,'#ca9d3c'],linewidths=1.0)
    fig.text(.06,.155,f'지도 age: {p["map_age"]*1000:.1f} ms / reference age: {p["ref_age"]*1000:.1f} ms / 격자: 0.04 m / 영역: 8 × 8 m',fontsize=12,color=INK)
    fig.text(.06,.115,'등고선: 보수적 clearance 0.20 m(몸체 반경), 0.25 m(추가 여유 포함), 0.40 m. LiDAR 직접 관측과 지도 보완을 혼동하지 않습니다.',fontsize=10.5,color=INK)
    save(fig,'02_grid_and_clearance.png')

    fig,ax,side=paired(3,'nominal과 경로 기준 확인','nominal은 당시 기록 reference에서 복원 · 운영 내부 warm start 원본은 아님',s)
    gridplot(ax,s);path(ax,s['nominal_path'],NOM,'nominal',3);ax.legend(loc='lower left',fontsize=9)
    ua=fig.add_axes([.58,.47,.36,.31]);t=np.arange(10)*.2
    for j,label in enumerate(['vx [m/s]','vy [m/s]','ω [rad/s]']):ua.step(t,s['nominal'][:,j],where='post',label=label)
    ua.legend(fontsize=9);ua.set(xlabel='제어 적용 시각 [s]',ylabel='nominal 제어');ua.grid(alpha=.15)
    note(side,f'현재 속도: [{p["current"][0]:.3f}, {p["current"][1]:.3f}, {p["current"][2]:.3f}]\nnominal의 경로 진행: {s["audit"]["nominal_progress"]:.3f} m\nnominal 정적 충돌: {s["audit"]["nominal_collision"]}\n기록 사람 예측: {s["audit"]["prediction_count"]}명\n\n이 장면에서는 nominal 중복 검사를 적용합니다.',y=.39,size=12)
    save(fig,'03_nominal_and_path.png')

    fig,ax,side=paired(4,'방향별 진입 깊이 d_free 계산','360방향 × 0.02 m 거리 표본 · 시작점부터 연속해서 통과 가능한 깊이',s)
    gridplot(ax,s)
    for idx in range(0,360,3):
        a=p['angles'][idx];d=p['d_free'][idx]
        ax.plot([0,d*np.cos(a)],[0,d*np.sin(a)],color=GREEN if d>=.5 else RED,alpha=.45,lw=.8)
    da=fig.add_axes([.58,.48,.36,.31]);da.plot(np.rad2deg(p['angles']),p['d_free'],color=INK,lw=1.6)
    da.axhline(.5,color=RED,ls='--',label='최소 깊이 0.5 m');da.set(xlabel='현재 base 방향 [°]',ylabel='깊이 [m]',ylim=(0,2.15));da.legend(fontsize=9);da.grid(alpha=.15)
    note(side,'몸체 반경 0.20 m + 추가 여유 0.05 m\nEDT에서 격자 대각선 + sweep 보수성을 제외\n\n열린 방향: 255 / 360개\n깊이 범위: 0.22 ~ 2.00 m\n막힌 방향: −10° ~ 94°\n열린 방향은 ±180°를 넘어 연결됩니다.',y=.40,size=12)
    save(fig,'04_directional_depth.png')

    fig,ax,side=paired(5,'연결 구간 1개를 깊이 골짜기에서 2개로 분할','같은 공간을 거리별로 복제하지 않음 · 보라 G1 / 초록 G2',s)
    gridplot(ax,s);sectors(ax,s)
    run=ex.circular_runs(p['d_free']>=.5)[0];degrees=np.rad2deg(np.unwrap(p['angles'][run]));smooth=gaussian_filter1d(p['d_free'],2,mode='wrap')
    da=fig.add_axes([.58,.49,.36,.31]);da.plot(degrees,p['d_free'][run],color='#9caab8',lw=1,label='원래 깊이');da.plot(degrees,smooth[run],color=INK,label='σ=2° 평활화')
    for i,g in enumerate(s['defs']):
        pos=np.flatnonzero(np.isin(run,g['ids']));da.axvspan(degrees[pos[0]],degrees[pos[-1]],color=GAPCOL[i],alpha=.16)
    da.axvline(305.5,color=RED,ls='--');da.set(xlabel='연속 각도 [°] · 305° = −55°',ylabel='깊이 [m]',ylim=(.4,2.1));da.legend(fontsize=8);da.grid(alpha=.15)
    note(side,'G1: 95° → ±180° 경계 → −55° / 211 bins\nG2: −54° → −11° / 44 bins\n\n분할 경계: −55°와 −54° 사이\npeak prominence ≥ 0.25 m / 간격 ≥ 20 bins\n구간 최소 크기: 1° 표본 6개\n충돌 검사는 평활화하지 않은 원래 지도로 수행',y=.38,size=12)
    save(fig,'05_actual_gap_boundaries.png')

    fig,ax,side=paired(6,'각 gap에서 3방향씩 · seed 6개 생성','A~F는 이 장면의 후보 표기 · 최종 선택 전에 모두 유지',s)
    gridplot(ax,s);sectors(ax,s)
    for g in items:
        path(ax,g['trajectory'],g['color'],lw=2)
        ax.scatter(*g['target'],marker='x',color=g['color'],s=50)
        offset={'D':(7,-16),'F':(7,8)}.get(g['label'],(5,5))
        ax.annotate(f'{g["label"]} {np.rad2deg(g["angle"]):.0f}°',g['target'],xytext=offset,textcoords='offset points',color=g['color'],fontsize=10)
    rows=[[g['label'],f'G{g["gap"]+1}',['깊이 peak','각도 중앙','경로 방향'][i%3],f'{np.rad2deg(g["angle"]):.0f}°'] for i,g in enumerate(items)]
    table(side,rows,['후보','gap','방향 종류','각도'])
    note(side,'경로 방향: 경로를 따라 약 1 m 앞의 점을 향함\ngap 밖을 향하면 gap 내부의 가장 가까운 방향 사용\n여기서는 MPPI가 아니라 제약 seed만 생성합니다.',y=.18,size=11.5)
    save(fig,'06_six_direction_seeds.png')

    fig=shell(7,'목표 거리를 2초 도달 범위로 줄인다','실제 제약 rollout의 도달 성분으로 목표를 조절한 뒤, 최종 목표 오차를 다시 검사',s)
    ax=fig.add_axes([.06,.24,.88,.55]);ax.axis('off')
    rows=[[g['label'],f'{np.rad2deg(g["angle"]):.0f}°',f'{p["d_free"][g["index"]]:.3f}',f'{g["available_depth"]:.3f}',
           f'{g["projected_reach"]:.3f}',f'{g["target_distance"]:.3f}',f'{g["reach_error"]:.3f}'] for g in items]
    table(ax,rows,['후보','방향','원래 깊이 m','가용 L m','2초 투영 도달 r m','최종 목표 거리 m','목표 오차 m'],bbox=(0,0,1,1),size=12)
    fig.text(.06,.16,'L = max(0.10, d_free − 0.10)     목표 거리 = min(L, max(0.25, 0.90 × r))',fontsize=15,color=INK)
    fig.text(.06,.115,'표의 모든 후보는 목표 오차 ≤ 0.35 m. 도달성 때문에 제외되는 후보가 없는 실제 장면입니다.',fontsize=12,color=INK)
    save(fig,'07_reachable_targets.png')

    fig=shell(8,'실제 10-step 제어입력열과 구동 제한','병진은 gap 목표를 향하고, yaw는 경로 heading을 따름 · seed 생성 시 매 step 제약 적용',s)
    for j,title in enumerate(['vx [m/s]','vy [m/s]','ω [rad/s]','휠 부하 |vx| + |vy| + k|ω|']):
        ax=fig.add_axes([.065+(j%2)*.48,.52-(j//2)*.36,.40,.27])
        for g in items:
            u=g['controls'];v=u[:,j] if j<3 else abs(u[:,0])+abs(u[:,1])+cfg.yaw_lever*abs(u[:,2])
            ax.step(np.arange(10)*.2,v,where='post',color=g['color'],label=g['label'],lw=1.6)
        if j<3:
            ax.axhline(cfg.control_min[j],color=RED,ls=':',lw=1);ax.axhline(cfg.control_max[j],color=RED,ls=':',lw=1)
        else:ax.axhline(cfg.speed_limit_l1,color=RED,ls='--',label='휠 상한')
        ax.set(title=title,xlabel='제어 적용 시각 [s]');ax.grid(alpha=.15)
        if j==0:ax.legend(ncol=6,fontsize=9)
    fig.text(.065,.105,f'축별 가속도 제한: {tuple(cfg.acceleration_limit)} / dt=0.2 s. 휠 부하 상한={cfg.speed_limit_l1:.3f}, k={cfg.yaw_lever:.5f}.',fontsize=10.5,color=INK)
    save(fig,'08_control_sequences_and_limits.png')

    fig,ax,side=paired(9,'실행 가능성과 nominal 중복 검사','이 장면은 6개 모두 통과 · 실제 탈락 장면은 보충 그림 16에 표시',s)
    gridplot(ax,s);path(ax,s['nominal_path'],NOM,'nominal',3)
    for g in items:path(ax,g['trajectory'],g['color'],g['label'],lw=1.7)
    ax.legend(loc='lower left',fontsize=9,ncol=2)
    rows=[[g['label'],f'{g["reach_error"]:.3f}',f'{g["rms"]:.3f}',f'{g["tail"]:.3f}','없음' if not g['collision'] else '있음'] for g in items]
    table(side,rows,['후보','목표 오차','nom RMS','nom 후반','몸체 충돌'],size=10)
    note(side,'거리 단위: m\n목표 오차 ≤ 0.35 / 몸체 충돌 없음\nnominal 중복 = RMS<0.22 AND 후반<0.30\n모든 후보가 RMS≥0.22 → 중복 기각 없음',y=.20,size=11.5)
    save(fig,'09_feasibility_and_nominal_dedup.png')

    fig=shell(10,'탐색 가치의 비용 순위 · 실제 비용 성분','별도 R 점수 없이 기존 공통 비용 + 정적 soft 비용 사용 · 이 단계에는 추가 MC 없음',s)
    ax=fig.add_axes([.07,.25,.50,.55]);bottom=np.zeros(len(items))
    groups=[('경로/목표',['path_error','terminal'],'#85b9ce'),('진행',['progress'],'#367db1'),('heading',['heading'],'#aa85c7'),
            ('속도',['speed'],'#67b79b'),('입력 변화',['delta_x','delta_y','delta_omega'],'#d6a256'),('정적 soft',['static'],'#c56f76')]
    for label,keys,col in groups:
        values=np.array([sum(g['components'][k] for k in keys) for g in items]);ax.bar([g['label'] for g in items],values,bottom=bottom,color=col,label=label);bottom+=values
    for i,g in enumerate(items):ax.text(i,bottom[i]+.5,f'{g["cheap_cost"]:.3f}',ha='center',fontsize=10)
    ax.set(ylabel='가중 비용',ylim=(0,float(bottom.max())*1.18));ax.legend(ncol=3,fontsize=9);ax.grid(axis='y',alpha=.15)
    side=fig.add_axes([.62,.24,.33,.57]);side.axis('off')
    ranked=sorted(items,key=lambda g:g['cheap_cost']);table(side,[[i+1,g['label'],f'{np.rad2deg(g["angle"]):.0f}°',f'{g["cheap_cost"]:.3f}'] for i,g in enumerate(ranked)],['순위','후보','방향','비용'],bbox=(0,.1,1,.85),size=11)
    fig.text(.07,.15,'비용은 우선순위를 정합니다. 이후 궤적 중복 제거로 서로 다른 움직임을 남깁니다.',fontsize=14,color=INK)
    fig.text(.07,.108,'nominal보다 초기 비용이 높다는 이유만으로 기각하지 않으며, 후방에 강제 가산점이나 할당도 두지 않습니다.',fontsize=11,color=INK)
    save(fig,'10_actual_cost_components.png')

    fig=shell(11,'서로 다른 움직임인가? · 실제 RMS 차이 행렬','각각 10개 위치의 동일 시각 차이로 RMS 계산 · 선택된 후보와 0.25 m 미만이면 중복',s)
    matrix=np.array([[np.sqrt(np.mean(np.sum((g['trajectory'][:,:2]-h['trajectory'][:,:2])**2,axis=1))) for h in items] for g in items])
    ax=fig.add_axes([.065,.25,.41,.55]);im=ax.imshow(matrix,cmap='Blues',vmin=0,vmax=max(matrix.max(),.25))
    ax.set_xticks(range(6),[g['label'] for g in items]);ax.set_yticks(range(6),[g['label'] for g in items])
    for i in range(6):
        for j in range(6):
            color=RED if i!=j and matrix[i,j]<.25 else ('white' if matrix[i,j]>.6*matrix.max() else INK)
            ax.text(j,i,f'{matrix[i,j]:.3f}',ha='center',va='center',fontsize=10,color=color)
    fig.colorbar(im,ax=ax,fraction=.046,pad=.04,label='궤적 위치 RMS [m]')
    side=fig.add_axes([.54,.25,.42,.55]);side.axis('off')
    trace=[]
    for k,v in enumerate(s['trace']):
        ds=' / '.join(f'{label}:{dist:.3f}' for label,dist in v['distances']) or '첫 후보'
        trace.append([k+1,v['label'],ds,REASONS[v['reason']]])
    table(side,trace,['순서','후보','이미 선택한 후보와 RMS m','결정'],bbox=(0,0,1,1),size=10,widths=[.10,.10,.52,.28])
    fig.text(.065,.15,'비용 순서 F → D → E → C → B → A로 검사. 최종 F·C·B 선택: 같은 G1에서 후방 B와 전방 C가 함께 남습니다.',fontsize=12,color=INK)
    fig.text(.065,.108,'RMS가 작은 후보가 여러 개 있어도 비용 순으로 이미 선택한 후보와 비교합니다. 행렬 전체를 한 덩어리로 묶는 clustering은 아닙니다.',fontsize=10.5,color=INK)
    save(fig,'11_pairwise_novelty_and_decisions.png')

    fig,ax,side=paired(12,'최종 탐색 seed 3개 · 후방 B도 보존','gap은 2개지만 guide는 3개 · gap당 하나 제한을 적용하지 않음',s)
    gridplot(ax,s);sectors(ax,s)
    for g in s['chosen']:
        path(ax,g['trajectory'],g['color'],f'{g["label"]}: {np.rad2deg(g["angle"]):.0f}°',3)
        ax.scatter(*g['target'],marker='x',s=70,color=g['color'])
    ax.legend(loc='lower left',fontsize=10)
    table(side,[['nominal','기준 제어열',200],['B / G1','−160° 후방',100],['C / G1','−55° 우측 전방',100],['F / G2','−11° 전방',100]],['sampling 중심','방향','rollout 수'],bbox=(0,.42,1,.49),size=12)
    note(side,'전체 이동 rollout = 500개\n비용 선택 순서: F → C → B\nsolver 순서: nominal → B → C → F\n\n동일 후보 집합은 순위가 바뀌어도 같은 ID 순서로 전달\n기존 2 worker 구조와 MC 평가를 유지',y=.34,size=12)
    save(fig,'12_selected_seeds_and_budget.png')

    fig=shell(13,'실제로 생성한 500개 MPPI rollout','group별 원래 sampling 결과를 모두 표시 · seed 7 · faint 선에 탈락 궤적도 포함',s)
    all_paths=ex.rollout(np.zeros(3),r.sampled_controls,.2)
    labels=['nominal']+[g['label'] for g in s['chosen']];colors=[NOM]+[g['color'] for g in s['chosen']]
    for i,(label,col,count) in enumerate(zip(labels,colors,s['counts'])):
        ax=fig.add_axes([.045+i*.24,.29,.22,.46]);gridplot(ax,s,pathline=False)
        ids=np.flatnonzero(r.control_mode_ids==i);finite=np.isfinite(r.rollout_costs[ids])
        for trajectory,good in zip(all_paths[ids],finite):path(ax,trajectory,col if good else RED,lw=.6,alpha=.17 if good else .12)
        path(ax,r.mode_trajectories[i],col,lw=3)
        ax.set_title(f'{label}: {count}개\n유한 비용 {int(finite.sum())}/{count}',fontsize=12)
        ax.tick_params(labelsize=8);ax.set_xlabel('x [m]',fontsize=9);ax.set_ylabel('y [m]',fontsize=9)
    fig.text(.06,.19,'굵은 선 = 각 sampling 그룹의 최적화 대표. 붉은 얇은 선 = 해당 rollout 평가에서 비용이 무한대로 처리된 후보.',fontsize=12,color=INK)
    fig.text(.06,.135,'서로 다른 그룹의 1차 MC는 로컬 영역에서 계산하므로, 대표 선택은 다음 단계의 공통 재평가로 결정합니다.',fontsize=11,color=INK)
    save(fig,'13_all_actual_rollouts.png')

    fig,ax,side=paired(14,'최종 공통 안전 재평가와 선택','이동 대표 4개 + 독립 Stop 후보를 공통 조건에서 재평가',s)
    gridplot(ax,s)
    selected=r.candidate_names.index(r.selected_mode)
    names=['nominal']+[g['label'] for g in s['chosen']]+['Stop'];colors=[NOM]+[g['color'] for g in s['chosen']]+[MUTED]
    for i in range(len(names)):path(ax,r.mode_trajectories[i],colors[i],names[i]+(' 최종' if i==selected else ''),4 if i==selected else 1.8,'-' if i==selected else '--')
    ax.legend(loc='lower left',fontsize=9)
    maxp=np.max(r.mode_collision_probabilities,axis=1)
    rows=[[name,f'{r.mode_final_costs[i]:.3f}',f'{maxp[i]:.2g}','충돌' if r.final_static_collisions[i] else '통과','선택' if i==selected else ''] for i,name in enumerate(names)]
    table(side,rows,['대표','최종 비용','최대 추정 p','정적 검사','결과'],bbox=(0,.43,1,.48),size=10.5)
    note(side,f'선택: {names[selected]} / 비용 {r.cost:.6f}\n기존 MC hard threshold: {cfg.dynamic_risk.hard_threshold}\nMC 최대 추정확률: {r.max_collision_probability:.3g}\n\n추정 p=0은 실제 충돌 확률 0의 보장이 아닙니다.\n이 장면의 모든 대표가 같은 최종 MC 조건으로 비교됩니다.',y=.35,size=12)
    save(fig,'14_common_recheck_and_selection.png')

    fig,ax,side=paired(15,'선택된 궤적과 실제 제어입력열','최종 선택까지의 결과 · 로봇에 명령을 발행하지 않은 오프라인 재계산',s)
    gridplot(ax,s);path(ax,r.trajectory,GREEN,'최종 선택',3.5)
    for k in [2,5,9]:
        pose=r.trajectory[k];ax.add_patch(Circle(pose[:2],.2,facecolor=GREEN,edgecolor=GREEN,alpha=.15))
        ax.arrow(pose[0],pose[1],.13*np.cos(pose[2]),.13*np.sin(pose[2]),width=.008,head_width=.055,color=GREEN,zorder=9)
    ua=fig.add_axes([.58,.48,.36,.30])
    for j,label in enumerate(['vx [m/s]','vy [m/s]','ω [rad/s]']):ua.step(np.arange(10)*.2,r.control_sequence[:,j],where='post',label=label)
    ua.set(xlabel='제어 적용 시각 [s]',ylabel='선택된 제어');ua.legend(fontsize=9);ua.grid(alpha=.15)
    u=r.control_sequence[0];progress=float(ex.path_progress(p,r.trajectory[None])[0])
    note(side,f'첫 입력: vx={u[0]:.4f}, vy={u[1]:.4f}, ω={u[2]:.4f}\n2초 예측 경로 진행: {progress:.3f} m\n최종 상태: {r.safety_status}\n\n입력 배열·seed 비용·최종 궤적을 기존 실험과 대조 완료\n새 주기에는 새 scan과 nominal로 같은 절차를 반복\n실제 주행 성공률·안정적 5 Hz를 검증한 그림은 아닙니다.',y=.38,size=12)
    save(fig,'15_final_controls_and_trajectory.png')


def supplementary(scenes):
    s=scenes['b1_004'];fig,ax,side=paired(16,'다른 실제 장면: nominal 중복·충돌·상한 제외','13.431초 장면 · 주 장면에서 발생하지 않은 필터도 실제 데이터로 확인',s)
    gridplot(ax,s);path(ax,s['nominal_path'],NOM,'nominal',3)
    for g in s['items']:
        color=GREEN if g['reason']=='selected' else RED if g['reason']=='collision' else '#929ca7'
        path(ax,g['trajectory'],color,lw=2 if g['reason']=='selected' else 1.2,alpha=.8)
    rows=[[g['label'],f'{np.rad2deg(g["angle"]):.0f}°',f'{g["rms"]:.3f}',f'{g["tail"]:.3f}',REASONS[g['reason']]] for g in s['items']]
    table(side,rows,['후보','방향','nom RMS','nom 후반','판정'],bbox=(0,.20,1,.72),size=10,widths=[.10,.12,.19,.19,.40])
    note(side,'A·B·C: nominal 중복 / I: 몸체 충돌\nG: 선택 후보와 중복 / E: 3개 상한\nD·F·H만 선택. 순방향이라는 이유로 모두 남기지 않습니다.',y=.15,size=11)
    save(fig,'16_real_filter_rejections.png')

    s=scenes['b0_029'];fig=shell(17,'raw gap 0개 → 추가 여유만 제외해 재시도','65.299초 장면 · 원래 몸체/격자/미확인/최종 MC 기준은 유지',s)
    for i,(depth,title) in enumerate([(s['p']['d_free'],'① 추가 여유 0.05 m: gap 0개'),(s['hard'],'② 추가 여유 0 m: gap 1개')]):
        ax=fig.add_axes([.055+i*.32,.30,.285,.48]);gridplot(ax,s)
        for index in range(0,360,3):
            a=s['p']['angles'][index];d=depth[index]
            ax.plot([0,d*np.cos(a)],[0,d*np.sin(a)],color=GREEN,alpha=.3,lw=.8)
        if i==1:
            for g in s['chosen']:path(ax,g['trajectory'],g['color'],lw=2.5)
        ax.set_title(title,fontsize=12)
    side=fig.add_axes([.705,.30,.25,.48]);side.axis('off')
    clearance=float(s['p']['field'].query(np.zeros((1,2)))[0]-np.sqrt(2)*.04-.01)
    note(side,f'시작점 보정 clearance\n{clearance:.5f} m\n\n0.25 m 기준은 시작점부터 실패\n0.20 m 기준은 통과\n\n−176° / −104° seed 선택\nnominal 300 + 100 + 100\n최종 비용 {s["result"].cost:.3f}',size=12)
    fig.text(.06,.185,'기존 추가 여유 기준에서는 d_free가 모든 방향에서 0. 재시도에서 최대 2 m, 0.5 m 이상 145방향이 복구됩니다.',fontsize=12,color=INK)
    fig.text(.06,.125,'복구한 후보도 기존 몸체 충돌·도달성·중복 검사와 MPPI 공통 재평가를 통과해야 합니다. 물리적 안전 여유 전체를 제거한 것이 아닙니다.',fontsize=10.5,color=INK)
    save(fig,'17_empty_gap_margin_retry.png')

    s=scenes['b0_033'];fig,ax,side=paired(18,'재시도해도 gap이 없는 실제 장면','73.491초 장면 · gap 없음과 안전한 정지 궤적 확보는 서로 다른 판정',s)
    gridplot(ax,s);path(ax,s['result'].trajectory,RED,'기존 hold 출력 궤적',3)
    clearance=float(s['p']['field'].query(np.zeros((1,2)))[0]-np.sqrt(2)*.04-.01)
    note(side,f'시작점의 보수적 clearance: {clearance:.3f} m\n로봇 반경: 0.200 m\n\n추가 여유를 제외해도 시작점부터 검사 실패\nraw gap 0개 → 재시도 후에도 0개\n추가 guide 없음 → nominal 500 rollout\n\n최종 상태: no_safe_candidate_hold\nnominal·Stop 모두 안전 기준 통과 후보 없음\n\n이 판정은 센서·지도·격자 보수성을 포함하며,\n실제 물리 접촉을 측정했다는 뜻은 아닙니다.',size=13)
    save(fig,'18_no_gap_and_no_safe_candidate.png')


def overview(s):
    fig,axs=plt.subplots(2,3,figsize=(19,13),dpi=170)
    fig.subplots_adjust(top=.87,bottom=.14,hspace=.40,wspace=.20)
    fig.suptitle('확정한 Gap v1 · 실제 LiDAR로 보는 전체 flow',fontsize=26,color=INK,y=.96)
    fig.text(.06,.915,'mpc_chain_20260923_113151 +188.248 s  |  공간 gap 2개 → seed 6개 → guide 3개 → 총 500 rollout → 최종 선택',fontsize=13,color=INK)
    for i,ax in enumerate(axs.flat):
        gridplot(ax,s,'scan' if i==0 else 'fused');ax.tick_params(labelsize=8);ax.set_xlabel('x [m]',fontsize=10);ax.set_ylabel('y [m]',fontsize=10)
        if i==0:
            ax.set_title('① 실제 LiDAR + 지도 / nominal 입력',fontsize=13)
            path(ax,s['nominal_path'],NOM,lw=3);caption='기록 nominal 진행 0.000 m · 원시 관측에서 시작'
        elif i==1:
            sectors(ax,s);ax.set_title('② 열린 구간을 깊이 골짜기로 분할',fontsize=13);caption='연결 구간 1개 → G1 / G2 · 경계 −55° / −54°'
        elif i==2:
            for g in s['items']:path(ax,g['trajectory'],g['color'],lw=2)
            ax.set_title('③ 각 gap에서 3방향 seed 생성',fontsize=13);caption='peak / 중앙 / 경로 방향 · 6개 모두 자격 검사 통과'
        elif i==3:
            for g in s['chosen']:path(ax,g['trajectory'],g['color'],f'{g["label"]} {np.rad2deg(g["angle"]):.0f}°',3)
            ax.legend(fontsize=9,loc='lower left');ax.set_title('④ 비용 + 궤적 중복 제거로 3개 선택',fontsize=13);caption='F −11° / C −55° / B −160° · 후방도 유지'
        elif i==4:
            r=s['result'];trajectories=ex.rollout(np.zeros(3),r.sampled_controls,.2)
            for group,col in enumerate([NOM]+[g['color'] for g in s['chosen']]):
                for trajectory in trajectories[r.control_mode_ids==group]:path(ax,trajectory,col,lw=.5,alpha=.12)
            ax.set_title('⑤ nominal + 3개 guide에서 sampling',fontsize=13);caption='nominal 200 + B 100 + C 100 + F 100 = 500'
        else:
            for i2,pathdata in enumerate(s['result'].mode_trajectories[:-1]):path(ax,pathdata,([NOM]+[g['color'] for g in s['chosen']])[i2],lw=1.5,style='--')
            path(ax,s['result'].trajectory,GREEN,lw=3.5)
            ax.set_title('⑥ 공통 안전 재평가 → F 최종 선택',fontsize=13);caption=f'noise seed 7 최종 비용 {s["result"].cost:.3f} · 기존 안전 기준 통과'
        ax.text(.5,-.23,caption,transform=ax.transAxes,ha='center',fontsize=10,color=INK)
    fig.text(.06,.035,'이 그림의 각 단계는 01~15 상세 PNG로 확대 제공. 16~18은 실제 필터 탈락·gap 없음 재시도·hold 장면. 실측 입력의 오프라인 계산이며 5 Hz/주행 성공률 검증 아님.',fontsize=11,color=MUTED)
    fig.savefig(OUT/'00_full_flow.png',dpi=170);plt.close(fig)
    tiles=[]
    for file in FILES:
        with Image.open(OUT/file) as im:tiles.append(im.resize((1440,810),Image.Resampling.LANCZOS))
    atlas=Image.new('RGB',(4320,810*6),'#f7f9fc')
    for i,tile in enumerate(tiles):atlas.paste(tile,((i%3)*1440,(i//3)*810))
    atlas.save(OUT/'00_all_18_steps.png')


def export(scenes):
    manifest=dict(design='gap_design_v1',source_spec_sha256=hashlib.sha256((BASE/'gap_design_v1.json').read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        method='all_options_hard_fallback',noise_seed=7,figures=FILES,scenes={})
    for name,s in scenes.items():
        p=s['p'];r=s['result']
        manifest['scenes'][name]=dict(bag=s['audit']['bag'],offset=s['audit']['offset'],record_ns=s['audit']['record_ns'],
            fallback=s['fallback'],strict_gap_count=len(s['strict_defs']),gap_count=len(s['defs']),
            options=[{k:v for k,v in g.items() if k not in ('controls','trajectory')} for g in s['items']],
            selection_trace=s['trace'],selected=[g['label'] for g in s['chosen']],solver_counts=s['counts'],
            final_names=r.candidate_names,final_costs=r.mode_final_costs,final_probability=r.mode_collision_probabilities,
            final_static_collisions=r.final_static_collisions,status=r.safety_status,selected_mode=r.selected_mode,
            validation='Input grid/depth/path and final cost/trajectory matched frozen previous experiment.')
        arrays=dict(grid=p['grid'],scan_grid=p['scan_grid'],map_fill=p['map_fill'],angles=p['angles'],strict_depth=p['d_free'],
            hard_depth=s['hard'],nominal_controls=s['nominal'],nominal_path=s['nominal_path'],full_path=p['full_path'],
            seed_controls=np.array([g['controls'] for g in s['items']]),seed_paths=np.array([g['trajectory'] for g in s['items']]),
            sampled_controls=r.sampled_controls,sampled_paths=ex.rollout(np.zeros(3),r.sampled_controls,.2),
            sampled_costs=r.rollout_costs,control_mode_ids=r.control_mode_ids,representatives=r.mode_trajectories,
            final_controls=r.control_sequence,final_trajectory=r.trajectory)
        for i,scan in enumerate(p['scans']):
            for key in ['angles','ranges','valid','beam_times','origins','hits']:arrays[f'scan_{i}_{key}']=scan[key]
        np.savez_compressed(OUT/f'{name}_actual_arrays.npz',**arrays)
    ex.write_json(OUT/'manifest.json',manifest)
    index=['확정 Gap v1 실제 LiDAR 상세 flow','',
        '00_full_flow.png: 주 장면 전체 흐름 6개 패널',
        '00_all_18_steps.png: 상세 PNG 18장의 전체 모음',
        '주 장면: b1_088, bag mpc_chain_20260923_113151 +188.247981 s',
        '주 장면 확정 v1 결과: G1/G2 공간 구간에서 6개 seed 생성; B(-160), C(-55), F(-11) 선택.',
        '이전 gap당 대표 하나 방식과 다르다. 공통 비용 별도 최적화 seed는 포함하지 않는다.',
        '실제 입력/seed 비용/최종 결과를 frozen 이전 실험과 대조했다. 실제 주행 실행이나 full-stack 5Hz 검증이 아니다.',
        'solver timeout은 모든 오프라인 결과를 확보하기 위한10초이며 운영 deadline 설정으로 쓰면 안 된다.',
        '',*FILES,'',
        '수치: manifest.json / *_actual_arrays.npz',
        '실행: ROS Jazzy와 workspace setup을 source한 뒤 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/python3 tools/render_gap_v1_detailed_flow.py']
    (OUT/'INDEX.txt').write_text('\n'.join(index)+'\n')
    with zipfile.ZipFile(OUT/'gap_v1_actual_flow_pngs.zip','w',zipfile.ZIP_DEFLATED) as z:
        for f in sorted(OUT.iterdir()):
            if f.suffix in ('.png','.json','.npz','.txt'):z.write(f,f.name)
        z.write(Path(__file__),Path(__file__).name)
        for name in ['GAP_DESIGN_V1.md','gap_design_v1.json']:z.write(BASE/name,name)
    print('CREATED',len(FILES)+2,'PNGs',OUT,flush=True)


if __name__=='__main__':
    OUT.mkdir(exist_ok=True)
    scenes=collect();main_figures(scenes['b1_088']);supplementary(scenes);overview(scenes['b1_088']);export(scenes)

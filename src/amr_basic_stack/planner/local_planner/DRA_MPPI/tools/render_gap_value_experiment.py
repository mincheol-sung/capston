"""Scientific figures and evidence report for the recorded-bag value experiment."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, FancyBboxPatch

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/gap_value_optimization_2026-11-05'
INK='#20364b';MUTED='#536b7e'
COLORS=['#c47b16','#3383ba','#009c91','#7c55b5','#d75a70','#678225']
REASONS={'selected':'선택','capacity':'최대 3개 제한','duplicate_nominal':'nominal 중복',
         'duplicate_selected':'선택 seed와 중복','static_collision':'정적 충돌',
         'solver_failure':'비용 최적화 미수렴','reach_failure':'도달거리 최적화 미수렴'}


def main():
    rows=json.loads((OUT/'scene_results.json').read_text())
    times=json.loads((OUT/'timings.json').read_text())
    agg=json.loads((OUT/'aggregate.json').read_text())
    cfg=json.loads((OUT/'configuration.json').read_text())
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
                         'axes.unicode_minus':False,'font.size':11,'mathtext.fontset':'dejavusans'})
    (OUT/'scenes').mkdir(exist_ok=True)

    def color(g):return COLORS[(g['region_id']-1)%len(COLORS)]

    def map_plot(ax,row,small=False):
        data=dict(np.load(OUT/f'raw/{row["scene"]}.npz'))
        result=dict(np.load(OUT/f'{row["scene"]}_results.npz'))
        view=np.zeros_like(data['grid']);view[data['grid']<0]=1
        view[data['map_fill']&(data['grid']==0)]=2;view[data['grid']>=65]=3
        ax.imshow(view,extent=(-4,4,-4,4),origin='lower',vmin=0,vmax=3,
                  cmap=ListedColormap(['white','#d8dfe7','#e5eefb','#34495e']))
        for i in range(2):
            hits=data[f'scan_{i}_hits'][data[f'scan_{i}_valid']]
            ax.scatter(hits[:,0],hits[:,1],s=1 if small else 3,color=('#bd824d','#577d9b')[i],alpha=.7)
        angles=result['angles'];depth=result['depth']
        xy=depth[:,None]*np.column_stack((np.cos(angles),np.sin(angles)))
        ax.plot(xy[:,0],xy[:,1],color='#93a8b9',lw=.7)
        nominal=np.vstack(([0,0],result['nominal_path'][:,:2]))
        ax.plot(nominal[:,0],nominal[:,1],'--',color=INK,lw=1.6,label='nominal')
        for g,tr in zip(row['candidates'],result['seed_paths']):
            selected=g['reason']=='selected';valid=g['success'] and g.get('reach_success',True)
            xy=np.vstack(([0,0],tr[:,:2]))
            ax.plot(xy[:,0],xy[:,1],color=color(g),lw=2.2 if selected else 1,
                    alpha=1 if selected else .4,ls='-' if valid else ':')
            if selected or not small:
                target=g.get('target_distance',g['distance'])*np.array([np.cos(np.deg2rad(g['angle_deg'])),np.sin(np.deg2rad(g['angle_deg']))])
                ax.scatter(*target,color=color(g),s=22,marker='x')
                ax.annotate(g['id']+(f" / {g['samples']}개" if not small else ''),target,
                            xytext=(4,5),textcoords='offset points',color=color(g),fontsize=8 if small else 10,
                            bbox=dict(fc='white',ec='none',alpha=.8,pad=.5))
        if row['candidates']:
            p=np.array(row['candidates'][0]['reference'])
            ax.scatter(*p,marker='*',s=70 if small else 120,color='#db3333',label='공통 진행 기준점')
        ax.add_patch(Circle((0,0),.2,fc=INK,ec='white',zorder=10))
        ax.set(xlim=(-2.15,2.15),ylim=(-2.15,2.15),aspect='equal')
        if not small:ax.set(xlabel='x [m] · 전방 + / 후방 −',ylabel='y [m] · 좌측 + / 우측 −')
        ax.grid(alpha=.1)

    fig,axes=plt.subplots(4,4,figsize=(18,18),facecolor='#f6f8fb')
    for ax,row in zip(axes.flat,rows):
        map_plot(ax,row,True)
        labels=', '.join(f"{g['id']}:{g['samples']}"for g in row['candidates']if g['samples']) or '추가 seed 없음'
        ax.set_title(f"{row['scene']} · 동적 객체 {row['predictions']}개\n{labels}",fontsize=10)
        ax.tick_params(labelsize=8)
    fig.suptitle('실제 bag 16장면 · gap 목표점 최적화와 비용별 샘플 배분',fontsize=23,color=INK,y=.98)
    fig.text(.5,.025,'색 선: gap별 최적화 seed (진한 선: 선택) · 점선: nominal · 빨간 별: 모든 gap의 공통 진행 기준점\n'
             '각 장면 총 500개 · nominal 기본 200개 · 추가 후보가 없으면 nominal 500개 · 오프라인 단일 장면 검증',
             ha='center',fontsize=12,color=MUTED)
    fig.subplots_adjust(top=.93,bottom=.08,hspace=.32,wspace=.20)
    fig.savefig(OUT/'01_all_16_scenes.png',dpi=145);plt.close(fig)

    for row in rows:
        fig=plt.figure(figsize=(17,11),facecolor='#f6f8fb')
        fig.text(.045,.95,f"{row['scene']} · 실제 방향별 비용과 샘플 수",fontsize=25,weight='bold',color=INK)
        fig.text(.045,.907,f"{row['bag']} +{row['offset']:.3f}s | gap {row['regions']}개 · 방향 후보 {len(row['candidates'])}개 · 동적 예측 {row['predictions']}개",color=MUTED,fontsize=12)
        ax=fig.add_axes([.055,.30,.43,.55]);map_plot(ax,row)
        ax.legend(loc='lower right',fontsize=10)
        candidates=sorted(row['candidates'],key=lambda g:g['angle_deg'])
        ax=fig.add_axes([.57,.60,.38,.24])
        for g in candidates:
            x=g['angle_deg']
            ax.plot([x,x],[g['without_space_cost'],g['cost']],color=color(g),lw=1.2,alpha=.65)
            ax.scatter(x,g['without_space_cost'],marker='o',facecolors='none',edgecolors=color(g),s=65)
            ax.scatter(x,g['cost'],marker='x' if not g['success'] or not g.get('reach_success',True) else 'o',color=color(g),s=50)
            ax.annotate(g['id'],(x,max(g['cost'],g['without_space_cost'])),xytext=(3,7),textcoords='offset points',fontsize=9)
        ax.scatter([],[],facecolors='none',edgecolors=INK,label='공간 제한 없는 도달점의 비용')
        ax.scatter([],[],color=INK,label='실제 d로 제한한 도달점의 비용')
        ax.set(xlabel='gap 목표점 방향 [°]',ylabel='비용',xlim=(-190,190))
        ymax=max([max(g['cost'],g['without_space_cost']) for g in candidates] or [1.])
        ax.set_ylim(0,ymax*1.5+1);ax.grid(alpha=.15);ax.legend(fontsize=8,loc='upper center')
        ax.set_title('샘플 배분에는 채워진 점의 비용 사용',fontsize=13)
        ax=fig.add_axes([.57,.35,.38,.16])
        labels=['nominal']+[g['id']for g in candidates]
        values=[int(row['counts'][0])]+[g['samples']for g in candidates]
        ax.bar(labels,values,color=[INK]+[color(g)for g in candidates])
        for i,v in enumerate(values):ax.text(i,v+4,str(v),ha='center',fontsize=10)
        ax.set(ylabel='MPPI 샘플 수',ylim=(0,max(values)*1.2+10));ax.grid(axis='y',alpha=.12)
        tax=fig.add_axes([.06,.12,.88,.14]);tax.axis('off')
        cells=[[g['id'],f"{g['angle_deg']:.0f}°",f"{g.get('target_distance',g['distance']):.3f}",
                f"{g['without_space_cost']:.3f}",f"{g['cost']:.3f}",str(g['samples']),REASONS[g['reason']]]for g in row['candidates']]
        if cells:
            table=tax.table(cellText=cells,colLabels=['후보','방향','목표 거리 [m]','공간 제한 전 비용','실제 공간 비용','샘플','판정'],
                            cellLoc='center',loc='center',colWidths=[.10,.10,.15,.17,.16,.10,.22])
            table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.45)
            for (r,col),cell in table.get_celld().items():
                cell.set_edgecolor('#cfd9e5')
                if r==0:cell.set_facecolor('#e5ecf4')
        fig.text(.055,.065,'빈 점과 채운 점은 서로 다른 도달점을 평가합니다. 짧아졌다고 비용이 항상 증가하는 것은 아닙니다. 미수렴 후보는 배분하지 않습니다.',fontsize=10,color=MUTED)
        fig.text(.055,.035,'최대거리 단계: heading 비용 없음 / 비용 단계: heading 포함 · 비선형 국소 해 · 최종 MPPI는 기존 정적·동적 충돌 평가 유지',fontsize=10,color=MUTED)
        fig.savefig(OUT/f'scenes/{row["scene"]}.png',dpi=150);plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(17,9),facecolor='#f6f8fb',gridspec_kw={'width_ratios':[1.25,1]})
    y=np.arange(len(rows))
    axes[0].barh(y-.18,[r['baseline_ms']['p95']for r in rows],height=.34,color='#6e91b3',label='현재 gap 방식 p95')
    axes[0].barh(y+.18,[r['value_ms']['p95']for r in rows],height=.34,color='#d68a38',label='교수님 방식 프로토타입 p95')
    axes[0].scatter([r['value_ms']['maximum']for r in rows],y+.18,marker='|',s=85,color='#a83434',label='프로토타입 최대')
    axes[0].set(yticks=y,yticklabels=[f"{r['scene']} (동적 {r['predictions']})"for r in rows],xlabel='전체 계산 시간 [ms]')
    axes[0].invert_yaxis();axes[0].axvline(180,color='#947c2d',ls='--');axes[0].axvline(200,color='#b73232',ls='--')
    axes[0].legend(fontsize=9,loc='lower right');axes[0].grid(axis='x',alpha=.15)
    axes[1].axis('off')
    text=(f"기존: 평균 {agg['baseline']['mean']:.1f} / p95 {agg['baseline']['p95']:.1f} / 최대 {agg['baseline']['maximum']:.1f} ms\n\n"
          f"새 방식: 평균 {agg['value']['mean']:.1f} / p95 {agg['value']['p95']:.1f} / 최대 {agg['value']['maximum']:.1f} ms\n\n"
          f"새 방식 180ms 초과: {agg['value']['over_180']} / {agg['value']['n']}회\n"
          f"새 방식 200ms 초과: {agg['value']['over_200']} / {agg['value']['n']}회\n\n"
          '포함: scan/지도 융합 → gap → 도달점/비용 최적화\n'
          '→ 배분 → 500개 MPPI → 정적·동적 최종 평가\n\n'
          '제외: bag I/O, 초기화, scan deskew/TF 콜백,\n'
          '예측 메시지 정렬, ROS 입출력, perception 추론\n\n'
          '각 장면 워밍업 1회 + 측정 12회 / 방식\n'
          '동일 snapshot 반복 · worker pool 재사용\n'
          '시간 제한은 측정을 위해 10초로 완화\n\n'
          '200ms는 5Hz 전체 주기 예산입니다.\n'
          '이 결과만으로 실기 5Hz를 보장할 수 없습니다.')
    axes[1].text(.02,.97,text,va='top',fontsize=12,color=INK,linespacing=1.5)
    fig.suptitle('5Hz 검토 · 실제 bag 16장면 / 방식별 192회 측정',fontsize=23,color=INK)
    fig.tight_layout(rect=[0,.01,1,.94]);fig.savefig(OUT/'02_timing.png',dpi=150);plt.close(fig)

    fig,ax=plt.subplots(figsize=(15,12),facecolor='#f6f8fb');ax.axis('off')
    boxes=[('① 실제 전·후방 LiDAR + 지도','로봇 몸체·안전 여유를 반영한 d(θ) → 거리 급변으로 gap 분할'),
           ('② gap 중앙 방향별 도달점 계산','vx, vy, ω 공동 최적화 · 구동 제약 + LiDAR 깊이 · 이 단계는 heading 비용 없음'),
           ('③ 그 도달점까지 비용 최소화','공통 진행 기준점 오차 + 직진성 + heading + 입력 변화 · 장애물 비용 제외'),
           ('④ 실제 이동 가능성·중복 확인','구동 제약·수렴 확인 + 몸체 충돌 검사 → nominal/다른 seed 중복 제외'),
           ('⑤ 실제 공간에서 나온 비용으로 배분','최대 3개 gap 후보 · 낮은 비용에 더 많이 · 최소 탐색 몫 · 총 500개 유지'),
           ('⑥ 기존 MPPI로 최종 선택','정적·동적 충돌 비용과 최종 안전 검사 유지 · 이 실험에서는 제어 명령 발행 없음')]
    for i,(title,body)in enumerate(boxes):
        y=.87-i*.145
        ax.add_patch(FancyBboxPatch((.05,y-.075),.90,.105,boxstyle='round,pad=.012',fc='white',ec='#a9bdce',transform=ax.transAxes))
        ax.text(.075,y,title,transform=ax.transAxes,fontsize=18,color=INK,weight='bold')
        ax.text(.075,y-.043,body,transform=ax.transAxes,fontsize=12,color=MUTED)
        if i<5:ax.annotate('',xy=(.5,y-.11),xytext=(.5,y-.085),xycoords='axes fraction',arrowprops=dict(arrowstyle='->',color=INK,lw=2))
    fig.suptitle('교수님 제안의 구체화 · 실제 bag로 검증한 흐름',fontsize=24,color=INK,y=.97)
    fig.savefig(OUT/'00_flow.png',dpi=150);plt.close(fig)

    candidates=[g for r in rows for g in r['candidates']]
    success=sum(g['success'] and g.get('reach_success',True) for g in candidates)
    reasons={reason:sum(g['reason']==reason for g in candidates)for reason in sorted({g['reason']for g in candidates})}
    decision=dict(scenes=len(rows),dynamic_scenes=sum(r['predictions']>0 for r in rows),
                  candidate_directions=len(candidates),both_solvers_converged=success,reasons=reasons,
                  safety_statuses={m:{s:sum(t['method']==m and t['status']==s for t in times)for s in sorted({t['status']for t in times})}for m in ['baseline','value']},
                  timing=agg)
    (OUT/'decision_summary.json').write_text(json.dumps(decision,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(decision,ensure_ascii=False,indent=2))


if __name__=='__main__':main()

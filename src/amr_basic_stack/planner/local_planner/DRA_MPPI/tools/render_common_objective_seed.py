"""Ten actual-scene PNGs for obstacle-free common-objective local optima."""
from collections import Counter
import csv
import json
from pathlib import Path
import zipfile
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_multiscene_2026-11-04'
OUT=BASE/'common_objective_seed_10_scenes'
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=FONT).get_name(),
    'axes.unicode_minus':False,'font.size':11,'figure.facecolor':'#f8fafc',
    'savefig.facecolor':'#f8fafc','axes.spines.top':False,'axes.spines.right':False})
INK='#19354d';MUTED='#526a80';ORANGE='#d96b13';BLUE='#3b80c5';GREEN='#008c77';RED='#be3d48'


def load(path):return [json.loads(x) for x in path.read_text().splitlines()]


def trajectory(ax,path,color,label=None,ls='-',lw=2.5,zorder=7):
    xy=np.vstack((np.zeros(2),path[:,:2]))
    ax.plot(xy[:,0],xy[:,1],ls,color=color,lw=lw,label=label,zorder=zorder)


def sceneplot(ax,p,q,r,previous,compact=False):
    grid=p['grid'];im=np.zeros_like(grid);im[grid<0]=1;im[p['map_fill']&(grid==0)]=2;im[grid>=65]=3
    ax.imshow(im,extent=(-4,4,-4,4),origin='lower',vmin=0,vmax=3,
              cmap=ListedColormap(['white','#d4dde6','#e5edfc','#263d52']),interpolation='nearest')
    for key,col in [('scan_0_hits','#c37b3d'),('scan_1_hits','#4676a3')]:
        ax.scatter(p[key][:,0],p[key][:,1],color=col,s=1.5,alpha=.65,zorder=3)
    ax.plot(p['full_path'][:,0],p['full_path'][:,1],':',color='#5a8ba4',lw=1.8,label='기록 global path')
    trajectory(ax,q['nominal_trajectory'],BLUE,'기록 nominal','--',1.8)
    trajectory(ax,previous,'#7b8b98','기존 MPPI 결과','--',1.6)
    trajectory(ax,q['common_extra_raw_trajectory'],GREEN,'seed 추가 후 MPPI',lw=2.6)
    trajectory(ax,q['common_trajectory'],ORANGE,'충돌 비용 제외 최적화',lw=3.3)
    xy=q['common_trajectory'][-1,:2]
    ax.scatter(*xy,color=ORANGE,s=45,zorder=9)
    if not compact:
        for point in q['common_trajectory'][[2,5,9],:2]:
            ax.add_patch(Circle(point,.2,edgecolor=ORANGE,facecolor=ORANGE,alpha=.10,zorder=5))
    if r['static_collision']:
        # Visual marker for a conservative padded swept-disc violation, not
        # a claim that a physical contact was observed in the recording.
        path=np.vstack((np.zeros(2),q['common_trajectory'][:,:2]))
        dense=np.concatenate([a[None]+np.linspace(0,1,101)[:,None]*(b-a) for a,b in zip(path[:-1],path[1:])])
        cell=np.floor((dense+4)/.04).astype(int)
        field=distance_transform_edt((grid>=0)&(grid<65))*.04
        clearance=field[cell[:,1],cell[:,0]]
        hits=np.flatnonzero(clearance<=.2+np.sqrt(2)*.04+.01)
        if len(hits):
            point=dense[hits[0]]
            ax.scatter(*point,marker='x',color=RED,s=110,linewidths=2.5,zorder=15,label='몸체 충돌 판정 위치')
    if 'prediction_means' in q:
        means=q['prediction_means']
        for person in means:
            for mode in person:
                ax.plot(mode[:,0],mode[:,1],':',color='#b65d9c',lw=1.2,alpha=.8,zorder=4)
            ax.scatter(*person[0,0],marker='s',s=22,color='#b65d9c',zorder=5)
    ax.add_patch(Circle((0,0),.2,facecolor=INK,edgecolor='white',zorder=8))
    ax.annotate('',(.4,0),(0,0),arrowprops=dict(arrowstyle='->',color=INK,lw=1.5),zorder=11)
    ax.set(xlim=(-2,2.3),ylim=(-2,2.3),aspect='equal',xlabel='전방 x [m]',ylabel='좌측 y [m]')
    ax.grid(alpha=.12)
    if not compact:ax.legend(loc='lower left',fontsize=9,framealpha=.95)


def main():
    rows=load(OUT/'scene_results.jsonl');trials=load(OUT/'trials.jsonl')
    raw_trials=load(OUT/'raw_seed_trials.jsonl')
    baseline={(r['scene'],r['seed']):r for r in load(BASE/'diversity_and_empty_gap/trials.jsonl') if r['method']=='all_options_hard_fallback'}
    assert len(rows)==10 and len(trials)==40 and len(raw_trials)==20
    runs=[s for r in rows for s in r['solver_runs']]
    summary=dict(scenes=10,solver_starts=len(runs),converged_starts=sum(s['success'] for s in runs),
        chosen_solver_converged=sum(r['best_solver_success'] for r in rows),
        static_collision=sum(r['static_collision'] for r in rows),dynamic_failed=sum(not r['dynamic_pass'] for r in rows),
        extra_seed_reasons=dict(Counter(r['extra_seed_reason'] for r in rows)),
        multistart_ms=dict(min=min(r['multistart_ms'] for r in rows),median=float(np.median([r['multistart_ms'] for r in rows])),max=max(r['multistart_ms'] for r in rows)),
        path_start_ms=dict(min=min(s['wall_ms'] for s in runs if s['start']=='path_guide'),
            median=float(np.median([s['wall_ms'] for s in runs if s['start']=='path_guide'])),
            max=max(s['wall_ms'] for s in runs if s['start']=='path_guide')),
        max_multistart_cost_spread=max(max(s['cost'] for s in r['solver_runs'])-min(s['cost'] for s in r['solver_runs']) for r in rows),
        max_constraint_violation=max(s['constraint_violation'] for s in runs),
        downstream=[])
    raw_comparison=[]
    for r in rows:
        scene=r['scene'];old=[baseline[scene,s] for s in [7,23]]
        new=[t for t in raw_trials if t['scene']==scene]
        valid=all(t['moving'] and t['cost'] is not None for t in old+new)
        before=float(np.mean([t['cost'] for t in old])) if valid else None
        after=float(np.mean([t['cost'] for t in new])) if valid else None
        raw_comparison.append(dict(scene=scene,comparable=valid,before=before,after=after,
            delta=after-before if valid else None,selected_count=sum(t['common_group_selected'] for t in new),
            static_invalid_seed=r['static_collision']))
    deltas=[r['delta'] for r in raw_comparison if r['comparable']]
    summary['unfiltered_extra_seed']=dict(comparison=raw_comparison,
        better=sum(d<-.1 for d in deltas),similar=sum(abs(d)<=.1 for d in deltas),worse=sum(d>.1 for d in deltas),
        noncomparable=sum(not r['comparable'] for r in raw_comparison),
        common_group_selected_trials=sum(t['common_group_selected'] for t in raw_trials),
        moving_trials=sum(t['moving'] for t in raw_trials),
        static_invalid_centres_with_selected_safe_result=sum(r['static_invalid_seed'] and r['selected_count']>0 for r in raw_comparison))
    files=[]
    for i,r in enumerate(rows):
        scene=r['scene'];p=np.load(BASE/'scenes'/f'{scene}.npz');q=np.load(OUT/'scenes'/f'{scene}.npz')
        previous=np.load(BASE/'diversity_and_empty_gap/scenes'/f'{scene}.npz')['all_options_hard_fallback_trajectory']
        safe=not r['static_collision'] and r['dynamic_pass']
        fig=plt.figure(figsize=(16,9),dpi=150)
        fig.text(.045,.94,f'{i+1:02d}  {scene} · 충돌 비용을 제외하고 구한 궤적',fontsize=23,color=INK,weight='bold')
        fig.text(.045,.89,f'{r["bag"]}  +{r["offset"]:.6f} s  |  2초 / 10-step / 속도·가속도·휠 제약 유지',fontsize=12,color=MUTED)
        ax=fig.add_axes([.055,.17,.47,.65]);side=fig.add_axes([.56,.16,.40,.67]);side.axis('off')
        sceneplot(ax,p,q,r,previous)
        side.text(0,.98,'장애물 제외 공통 비용 J ↓',va='top',fontsize=17,weight='bold',color=INK)
        table=side.table(cellText=[['기록 nominal',f'{r["nominal_common_cost"]:.5f}'],
                                  ['기존 path guide',f'{r["path_guide_common_cost"]:.5f}'],
                                  ['수치 최적화 결과',f'{r["common_cost"]:.5f}']],
                         colLabels=['제어입력열','공통 비용'],cellLoc='center',bbox=[0,.65,1,.24],colWidths=[.60,.40])
        table.auto_set_font_size(False);table.set_fontsize(12)
        for (row,col),cell in table.get_celld().items():
            cell.set_edgecolor('#d3dfea');cell.set_facecolor('#e9eff8' if row==0 else 'white')
        static='통과' if not r['static_collision'] else '충돌 판정'
        dynamic='통과' if r['dynamic_pass'] else '기준 초과'
        text=(f'사후 정적 몸체 검사: {static}\n'
              f'사후 동적 MC 검사: {dynamic} (최대 추정 p={r["max_probability"]:.4g})\n'
              f'nominal과 위치 RMS 차이: {r["rms_nominal"]:.3f} m\n'
              f'예측 경로 진행량: {r["progress_m"]:.3f} m\n\n'
              f'초기값 {len(r["solver_runs"])}개 / 선택 결과 수렴: {"예" if r["best_solver_success"] else "아니오"}\n'
              f'전체 다중 초기값 최적화: {r["multistart_ms"]:.1f} ms')
        side.text(0,.59,text,va='top',fontsize=12.5,color=INK,linespacing=1.65)
        if not r['static_collision']:
            before=float(np.mean([baseline[scene,s]['cost'] for s in [7,23]]))
            after=float(np.mean([t['cost'] for t in trials if t['scene']==scene and t['method']=='common_as_nominal']))
            summary['downstream'].append(dict(scene=scene,before_mean_cost=before,after_mean_cost=after))
        comparison=next(x for x in raw_comparison if x['scene']==scene)
        if comparison['comparable']:
            decision=(f'최적화 seed 자체: {"충돌 판정" if r["static_collision"] else "정적 통과"}\n'
                      'seed 추가 후 MPPI: 안전 기준 통과\n'
                      f'최종 MPPI 비용: {comparison["before"]:.3f} → {comparison["after"]:.3f} (두 seed 평균)\n'
                      f'공통 비용 seed 그룹이 최종 선택된 횟수: {comparison["selected_count"]}/2')
        else:
            decision='시작 몸체부터 충돌로 판정된 장면입니다.\nseed를 추가해도 안전 후보 없음 (기존 방식과 동일).'
        side.text(0,.19,decision,va='top',fontsize=12,color=GREEN if comparison['comparable'] else RED,linespacing=1.55)
        fig.text(.045,.078,'흰색: scan 관측 free / 연파랑: 지도 보완 free / 회색: 미확인 / 주황색 원: 로봇 몸체 / 분홍색: 기록 사람 예측',fontsize=9.5,color=MUTED)
        fig.text(.045,.041,'실측 입력의 오프라인 수치 최적화 · 전역 최적해 보장 없음 · 주황색 궤적에는 장애물 비용·충돌 제한을 적용하지 않음',fontsize=10,color=MUTED)
        file=f'{i+1:02d}_{scene}.png';fig.savefig(OUT/file,dpi=150);plt.close(fig);files.append(file)
    fig,axs=plt.subplots(2,5,figsize=(24,11),dpi=160)
    fig.subplots_adjust(top=.84,bottom=.11,hspace=.42,wspace=.23)
    fig.suptitle('실제 10장면 · 장애물 비용을 제외한 공통 목적함수 최적화',fontsize=27,color=INK,y=.965)
    fig.text(.055,.90,'주황: 공통 비용 최적화 seed   |   초록: seed 추가 후 MPPI   |   파랑 점선: nominal   |   회색 점선: 기존 MPPI   |   빨강 ×: seed 몸체 충돌 판정',fontsize=13,color=INK)
    for i,(ax,r) in enumerate(zip(axs.flat,rows)):
        p=np.load(BASE/'scenes'/f'{r["scene"]}.npz');q=np.load(OUT/'scenes'/f'{r["scene"]}.npz')
        previous=np.load(BASE/'diversity_and_empty_gap/scenes'/f'{r["scene"]}.npz')['all_options_hard_fallback_trajectory']
        sceneplot(ax,p,q,r,previous,compact=True)
        ax.set_title(f'{i+1:02d} {r["scene"]} / +{r["offset"]:.3f}s\nJ {r["nominal_common_cost"]:.3f} → {r["common_cost"]:.3f}',fontsize=12,color=INK)
        comparison=next(x for x in raw_comparison if x['scene']==r['scene'])
        state='seed: 정적 충돌' if r['static_collision'] else 'seed: 정적 통과'
        state+=f' / 최종 J {comparison["before"]:.2f}→{comparison["after"]:.2f}' if comparison['comparable'] else ' / 안전 후보 없음'
        ax.text(.5,-.24,state,transform=ax.transAxes,ha='center',fontsize=9.5,color=RED if not comparison['comparable'] else GREEN)
        ax.tick_params(labelsize=8);ax.set_xlabel('전방 x [m]',fontsize=9);ax.set_ylabel('좌측 y [m]',fontsize=9)
    fig.text(.055,.027,'주황색은 실행 경로가 아닌 sampling 중심. seed 추가 후 최종 비용: 7장면 개선 / 2장면 유사 / 1장면 안전 후보 없음. 오프라인 진단이며 전역 최적·실제 주행 성능 보장 없음.',fontsize=12,color=INK)
    fig.savefig(OUT/'00_ten_scenes_overview.png',dpi=160);plt.close(fig)
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    with (OUT/'scene_summary.csv').open('w') as f:
        fields=['scene','bag','offset','common_cost','nominal_common_cost','path_guide_common_cost','static_collision',
                'max_probability','dynamic_pass','extra_seed_reason','rms_nominal','progress_m','multistart_ms']
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    report=['충돌 비용 제외 공통 목적함수 seed · 실제 10장면',
        '다중 초기값 SLSQP로 기존 common_costs만 최소화. 속도/가속도/휠 제한 및 기존 비선형 rollout 유지.',
        '정적 soft/hard 항과 동적 위험 항은 seed 최적화에서 모두 제외하고 사후 평가에서만 사용.',
        '수치적으로 수렴한 feasible 최저 관측해이며 전역 최적해 보장은 없다.',
        f'선정된 진단 장면 10개: 정적 충돌 {summary["static_collision"]}개, 나머지 3개는 nominal과 중복.',
        f'로컬 최적화 {summary["converged_starts"]}/{summary["solver_starts"]}회 수렴.',
        '공통 비용 수치와 장애물 포함 최종 MPPI 비용은 서로 다른 목적함수이므로 혼동하지 않는다.',
        '1차 gated 추가 seed 방식은 10개 모두 충돌/중복 gate에서 제외돼 원래 결과를 재사용했다. 이것만으로 sampling 중심의 유용성을 평가할 수 없다.',
        '정적 통과 3개를 nominal 초기값으로 사용한 실제 MPPI 6회는 모두 비용이 줄었다.',
        '초기값 교체: 동일500 rollout 예산, 기존 gap 가이드 및 최종 충돌/MC 검사 유지.',
        '2차 직접 추가: 충돌/중복으로 seed 중심을 기각하지 않고 실제10장면x2seed=20회 MPPI 실행. 기존 rollout 및 최종 안전 검사는 유지.',
        '직접 추가 결과: 비용 비교 가능한9장면 중7개 개선,2개 유사,0개 악화. 나머지1개는 두 방식 모두 안전 후보 없음. 유사 기준은 장면별 두 seed 평균 차이 ±0.1.',
        '공통 seed 그룹이 14/20 trial에서 최종 선택됐다. seed 중심이 정적 충돌했던4장면에서도 주변 sampling 후 안전 기준 통과 결과가 선택됐다.',
        '권고 개발 구조: nominal + 공통 비용 최적화 seed 1개 + gap 최대2개. 총500 rollout, 공통 seed100개. gap 다양성 필터와 별개인 목표추종 최적화 역할.',
        '기존 gap이3개인 경우 비용 우선순위가 낮은 gap을 대체하며 nominal 배정도 변경된다. 따라서 목적함수 개선만 고립한 실험은 아니다.',
        '이10개는 결과 확인 전 선정한 진단 장면이며 기존 데이터 재사용이다. 전체127장면/새bag/폐루프 일반화 증거는 아니다.',
        f'다중 초기값 전체 시간 ms: {summary["multistart_ms"]}',
        f'path guide 하나에서 시작한 최적화 시간 ms: {summary["path_start_ms"]}',
        '다중 초기값 계산을 매 주기 추가하는 것은 5Hz 예산에 맞지 않는다. 단일 초기값/제한 반복 실시간 구현은 별도 검증 필요.',
        '도구: tools/experiment_common_objective_seed.py, tools/evaluate_common_seed_raw.py, tools/render_common_objective_seed.py',
        '실행: ROS Jazzy와 workspace setup을 source한 뒤 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/python3 tools/experiment_common_objective_seed.py',
        '직접 추가 후속 실행: 같은 ROS/스레드 환경에서 /usr/bin/python3 tools/evaluate_common_seed_raw.py',
        '그림만 재생성: /usr/bin/python3 tools/render_common_objective_seed.py',
        '',json.dumps(summary['downstream'],ensure_ascii=False,indent=2),
        '',json.dumps(summary['unfiltered_extra_seed'],ensure_ascii=False,indent=2)]
    (OUT/'RESULTS.txt').write_text('\n'.join(report)+'\n')
    with zipfile.ZipFile(OUT/'ten_scene_paths.zip','w',zipfile.ZIP_DEFLATED) as z:
        for f in sorted(OUT.rglob('*')):
            if f.is_file() and f.suffix in ('.png','.json','.jsonl','.npz','.csv','.txt'):z.write(f,str(f.relative_to(OUT)))
        for name in ['experiment_common_objective_seed.py','evaluate_common_seed_raw.py','render_common_objective_seed.py']:z.write(ROOT/'tools'/name,name)
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()

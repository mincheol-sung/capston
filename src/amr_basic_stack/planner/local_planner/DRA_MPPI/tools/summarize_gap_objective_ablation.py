"""Summarize paired seed-objective experiments; no runtime planner changes."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

from experiment_gap_objective_ablation import OUT, SOURCE, METHODS, POLICIES, write, serial
from benchmark_guide_conditioned import summary
from dra_mppi.mecanum_model import rollout

LABELS=dict(current='현재 4항',smooth='입력 변화만',smooth_heading='입력 변화+heading',
            guide_tracking='중앙 가이드 추종',existing='기존 MPPI 주행 비용',nominal_only='nominal 단독',
            linearized_common='기존 주행 비용의 1회 근사')
PLABELS=dict(uniform='균등 잔여 배분',native='자체 비용 배분',common='공통 비용 배분')
COLORS=['#bb791c','#68983b','#009c90','#8552b5','#327fb3','#bf556f']


def paired(rows,method,policy,base_method,base_policy):
    lookup={(r['scene'],r['trial'],r['method'],r['policy']):r for r in rows}
    diffs={}; raw={}; loss_safe=gain_safe=0
    for r in rows:
        if (r['method'],r['policy'])!=(method,policy): continue
        base=lookup[(r['scene'],r['trial'],base_method,base_policy)]
        valid=r['status']=='ok' and r['cost'] is not None
        bvalid=base['status']=='ok' and base['cost'] is not None
        loss_safe+=bvalid and not valid; gain_safe+=valid and not bvalid
        if not (valid and bvalid):continue
        diff=100*(r['cost']-base['cost'])/max(abs(base['cost']),1e-9)
        diffs.setdefault(r['scene'],[]).append(diff)
        raw.setdefault(r['scene'],[]).append(r['cost']-base['cost'])
    values=np.array([np.mean(v) for v in diffs.values()])
    rng=np.random.default_rng(20261106)
    bootstrap=np.mean(rng.choice(values,(10000,len(values)),replace=True),axis=1)
    return dict(relative_mean_percent=float(np.mean(values)),relative_median_percent=float(np.median(values)),
        scene_bootstrap_95=np.percentile(bootstrap,[2.5,97.5]),
        better_scenes=int(np.sum(values<-.1)),worse_scenes=int(np.sum(values>.1)),
        tied_scenes=int(np.sum(np.abs(values)<=.1)),lost_safe=int(loss_safe),gained_safe=int(gain_safe),
        scene_relative_percent={s:float(np.mean(v)) for s,v in diffs.items()},
        scene_absolute_difference={s:float(np.mean(v))for s,v in raw.items()})


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=OUT)
    args=parser.parse_args();out=args.output
    rows=json.loads((out/'runs.json').read_text())
    config=json.loads((out/'configuration.json').read_text())
    scenes=[json.loads((out/f'{s["scene"]}_seeds.json').read_text()) for s in config['sources']]
    comparisons={}
    for m in METHODS:
        comparisons[m]={p:dict(vs_current_native=paired(rows,m,p,'current','native'),
                               vs_own_uniform=paired(rows,m,p,m,'uniform'),
                               vs_current_uniform=paired(rows,m,p,'current','uniform')) for p in POLICIES}
    objectives={}
    for m in METHODS:
        values=[q['variants'][m]for s in scenes for q in s['candidates'] if m in q['variants']]
        per_scene=[sum(q['variants'][m]['solve_ms']for q in s['candidates']if m in q['variants'])for s in scenes]
        rank={}
        for p in ('native','common'):
            hit=total=0;regrets=[]
            for r in rows:
                if r['method']!=m or r['policy']!='uniform' or len(r['names'])<3:continue
                s=next(s for s in scenes if s['scene']==r['scene'])
                scores=s['proposals'][m][p+'_scores']
                costs=np.array([np.inf if x is None else x for x in r['final_costs'][1:-1]])
                choice=int(np.argmin(scores[1:])); best=int(np.argmin(costs))
                if not np.isfinite(costs[best]):continue
                hit+=choice==best;total+=1
                if np.isfinite(costs[choice]):regrets.append(float(costs[choice]-costs[best]))
            rank[p]=dict(best_gap_match=hit,total=total,rate=hit/total if total else None,
                         mean_regret=float(np.mean(regrets))if regrets else None)
        objectives[m]=dict(candidate_count=len(values),accepted=sum(v['accepted']for v in values),
            solver_failed=sum(not v['solver_success'] for v in values),collision_rejected=sum(v['collision']for v in values),
            solve_one_ms=summary([v['solve_ms']for v in values]),solve_all_per_scene_ms=summary(per_scene),
            rank_prediction=rank)
        gap_rows=[]
        for r in rows:
            if r['method']not in ('current',m)or r['policy']!='uniform':continue
            gaps=[c for c in r['final_costs'][1:-1]if c is not None]
            gap_rows.append(dict(r,cost=min(gaps)if gaps else None))
        objectives[m]['best_finite_gap_vs_current_uniform']=paired(gap_rows,m,'uniform','current','uniform')
    policies={}
    for m in (*METHODS,'nominal_only'):
        for p in (POLICIES if m!='nominal_only' else ('uniform',)):
            selected=[r for r in rows if r['method']==m and r['policy']==p]
            policies[m+'/'+p]=dict(calls=len(selected),ok=sum(r['status']=='ok'for r in selected),
                stop=sum(r['selected']=='stop'for r in selected),
                mean_cost=float(np.mean([r['cost']for r in selected if r['cost']is not None])),
                mppi_ms=summary([r['mppi_ms']for r in selected]),
                max_probability=max(r['probability']for r in selected),
                mean_invalid_rollouts=float(np.mean([r['invalid_rollouts']for r in selected])))
    result=dict(scenes=len(scenes),calls=len(rows),trials=config['trials'],
        dynamic_scenes=sum(s['source']['predictions']>0 for s in scenes),
        candidates=sum(len(s['candidates'])for s in scenes),
        neutral_rejected=sum(not q['neutral_ok']for s in scenes for q in s['candidates']),
        selected_gaps=sum(len(s['names'])-1 for s in scenes),objectives=objectives,policies=policies,
        comparisons=comparisons,nominal_only_vs_current=paired(rows,'nominal_only','uniform','current','native'),
        geometry_ms=summary([s['geometry_ms']for s in scenes]),
        common_reach_per_scene_ms=summary([sum(q['reach_ms']for q in s['candidates'])for s in scenes]))
    write(out/'summary.json',result)
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,
                         'figure.facecolor':'#f5f8fc','axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,3,figsize=(18,6),gridspec_kw={'width_ratios':[1.2,1.4,1.]})
    ys=np.arange(len(METHODS))
    for i,m in enumerate(METHODS):
        value=comparisons[m]['uniform']['vs_current_uniform'];mean=value['relative_mean_percent'];lo,hi=value['scene_bootstrap_95']
        axes[0].errorbar(mean,i,xerr=np.array([[mean-lo],[hi-mean]]),fmt='o',color=COLORS[i],capsize=4)
    axes[0].axvline(0,color='#aab3bc',lw=1);axes[0].set_yticks(ys,[LABELS[m]for m in METHODS]);axes[0].invert_yaxis()
    axes[0].set_xlabel('현재 비용 대비 최종 비용 변화 [%]\n음수: 개선 / 선: 장면 단위 bootstrap 95% 구간')
    axes[0].set_title('① 배분을 고정하고 seed 비용만 변경')
    for offset,p,color in [(-.15,'native','#bb791c'),(.15,'common','#327fb3')]:
        values=[comparisons[m][p]['vs_own_uniform']['relative_mean_percent']for m in METHODS]
        axes[1].barh(ys+offset,values,height=.28,label=PLABELS[p],color=color)
    axes[1].set_yticks(ys,[LABELS[m]for m in METHODS]);axes[1].invert_yaxis();axes[1].axvline(0,color='#aab3bc',lw=1)
    axes[1].set_xlabel('같은 seed의 균등 잔여 배분 대비 변화 [%]');axes[1].legend(fontsize=9)
    axes[1].set_title('② seed를 고정하고 배분 기준만 변경')
    axes[2].barh(ys,[objectives[m]['solve_all_per_scene_ms']['p50']for m in METHODS],color=COLORS)
    axes[2].set_yticks(ys,[LABELS[m]for m in METHODS]);axes[2].invert_yaxis()
    axes[2].set_xlabel('한 장면의 전체 gap 비용 최적화 [ms]\n중앙값 / 도달거리·500개 평가 시간 제외')
    axes[2].set_title('③ 비용함수에 따른 계산량')
    fig.suptitle(f'실제 bag {len(scenes)}장면 · {config["trials"]}개 노이즈 · {len(rows):,}회 MPPI · 매회 500개',fontsize=19,y=1.02)
    fig.text(.02,-.04,'500개 및 최종 대표의 평가 비용은 그대로 유지. 순위 변화는 정지 장면 재실행 결과이며 주행 성공률이 아닙니다.',fontsize=11)
    fig.tight_layout();fig.savefig(out/'01_comparison.png',dpi=160,bbox_inches='tight');plt.close(fig)
    matrix=np.array([[comparisons[m]['native']['vs_current_native']['scene_relative_percent'][s['scene']]for s in scenes]for m in METHODS])
    fig,ax=plt.subplots(figsize=(18,4.5));lim=max(5,float(np.percentile(abs(matrix),95)))
    im=ax.imshow(matrix,cmap='RdBu_r',vmin=-lim,vmax=lim,aspect='auto')
    for i in range(len(METHODS)):
        for j in range(len(scenes)):ax.text(j,i,f'{matrix[i,j]:+.1f}',ha='center',va='center',fontsize=9)
    ax.set_xticks(np.arange(len(scenes)),[s['scene']for s in scenes],rotation=45,ha='right')
    ax.set_yticks(ys,[LABELS[m]for m in METHODS]);fig.colorbar(im,ax=ax,label='최종 비용 변화 [%] / 음수: 개선')
    ax.set_title('각 비용으로 seed 최적화 + 그 비용으로 배분: 현재 4항/자체 배분 대비\n각 칸은 동일 장면의 8개 paired 노이즈 평균')
    fig.tight_layout();fig.savefig(out/'02_scene_comparison.png',dpi=160);plt.close(fig)
    # Fixed familiar scene, not picked according to the best result.
    scene='b2_060'
    if any(s['scene']==scene for s in scenes):
        data=dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
        fig,axes=plt.subplots(2,3,figsize=(16,10));axes=axes.ravel()
        for i,m in enumerate(METHODS):
            ax=axes[i];a=dict(np.load(out/'arrays'/f'{scene}_{m}_native.npz'))
            row=next(r for r in rows if r['scene']==scene and r['method']==m and r['policy']=='native'and r['trial']==0)
            for sensor in range(2):
                hits=data[f'scan_{sensor}_hits'];valid=data[f'scan_{sensor}_valid']
                ax.scatter(hits[valid,0],hits[valid,1],s=1,color='#657e90',alpha=.6)
            paths=rollout(np.zeros(3),a['sampled_controls'],.2)
            for g in range(len(a['counts'])):
                color='#1f3649'if g==0 else COLORS[(g-1)%5]
                for trajectory in paths[a['group_ids']==g]:
                    ax.plot(np.r_[0,trajectory[:,0]],np.r_[0,trajectory[:,1]],color=color,lw=.35,alpha=.09)
                seed=rollout(np.zeros(3),a['seeds'][g],.2)
                ax.plot(np.r_[0,seed[:,0]],np.r_[0,seed[:,1]],color=color,lw=1.8,ls='--')
            final=a['final'];ax.plot(np.r_[0,final[:,0]],np.r_[0,final[:,1]],color='#e1323c',lw=2.7,label='선택된 MPPI 대표')
            ax.plot(data['path'][:,0],data['path'][:,1],color='gray',lw=1,ls=':')
            ax.plot(0,0,'ko',ms=5);ax.set_xlim(-1.5,2);ax.set_ylim(-1.25,1.25);ax.set_aspect('equal');ax.grid(alpha=.15)
            ax.set_title(f'{LABELS[m]}\n배분 {row["counts"]} / 최종 비용 {row["cost"]:.3f}')
            ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.legend(fontsize=9,loc='lower left')
        fig.suptitle('seed 비용 변경이 샘플과 최종 대표에 미친 영향',fontsize=21)
        fig.text(.02,-.02,'b2_060, trial 0 | 가는 선: 500개 샘플 / 점선: seed / 빨강: 선택 대표 | 모두 자체 비용 배분 · 같은 노이즈 · 기존 정적·동적 평가 유지',fontsize=12)
        fig.tight_layout();fig.savefig(out/'03_b2_060_rollouts.png',dpi=150,bbox_inches='tight');plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,6))
    for i,m in enumerate(METHODS):
        v=objectives[m]['best_finite_gap_vs_current_uniform'];mean=v['relative_mean_percent'];lo,hi=v['scene_bootstrap_95']
        ax.errorbar(mean,i,xerr=np.array([[mean-lo],[hi-mean]]),fmt='o',color=COLORS[i],capsize=4)
    ax.set_yticks(ys,[LABELS[m]for m in METHODS]);ax.invert_yaxis();ax.axvline(0,color='gray',lw=1)
    ax.set_xlabel('현재 seed 비용 대비 gap 대표의 비용 변화 [%]\n음수: 개선 / 선: 장면 단위 bootstrap 95% 구간')
    ax.set_title('nominal을 제외하고, 각 장면의 최선 gap 대표만 비교\n동일 배분 · 최종 평가는 기존 MPPI 비용 그대로')
    fig.tight_layout();fig.savefig(out/'04_gap_representatives.png',dpi=160);plt.close(fig)
    for m in METHODS:
        for p in POLICIES:
            r=comparisons[m][p]['vs_current_native']
            print(m,p,'vs current native',round(r['relative_mean_percent'],3),'CI',np.round(r['scene_bootstrap_95'],3),
                  'W/L/T',r['better_scenes'],r['worse_scenes'],r['tied_scenes'])
    print('SOLVES',json.dumps(serial(objectives),indent=2))


if __name__=='__main__':main()

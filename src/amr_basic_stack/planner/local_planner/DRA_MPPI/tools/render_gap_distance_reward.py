#!/usr/bin/env python3
"""Summarize reward sweep without changing the production planner."""
import argparse
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/gap_distance_reward_2026-11-09'
SOURCE=ROOT/'docs/gap_value_optimization_2026-11-05'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--focus',type=float,default=32.)
    args=parser.parse_args()
    b=json.loads((OUT/'benchmark.json').read_text());s=b['summary'];rows=b['rows']
    keys=['fixed']+[f'reward_{v:g}' for v in b['weights']]
    labels=['현재 2회']+[f'λ={v:g}' for v in b['weights']]
    ids=[x['scene'] for x in b['sources']]
    paired={};scene_stats={}
    for mode in ('uncensored','production'):
        paired[mode]={};scene_stats[mode]={}
        lookup={(r['scene'],r['trial'],r['label']):r for r in rows if r['mode']==mode}
        for name in keys:
            pairs=[(lookup[scene,t,'fixed'],lookup[scene,t,name]) for scene in ids for t in range(b['iterations'])]
            complete=[(a,z) for a,z in pairs if 'cost' in a and 'cost' in z]
            diff=np.array([z['cost']-a['cost'] for a,z in complete])
            paired[mode][name]=dict(pairs=len(complete),better=int(sum(diff < -1e-6)),
                worse=int(sum(diff>1e-6)),ties=int(sum(abs(diff)<=1e-6)),
                cost_change_percent=100*(np.mean([z['cost'] for a,z in complete])/
                                         np.mean([a['cost'] for a,z in complete])-1),
                time_change_percent=100*(s[mode][name]['total_ms']['mean']/s[mode]['fixed']['total_ms']['mean']-1))
            scene_stats[mode][name]={}
            for scene in ids:
                p=[(a,z) for a,z in complete if a['scene']==scene]
                if not p:continue
                a=np.mean([a['cost'] for a,z in p]);z=np.mean([z['cost'] for a,z in p])
                scene_stats[mode][name][scene]=dict(cost=z,baseline_cost=a,change_percent=100*(z/a-1))
    (OUT/'analysis.json').write_text(json.dumps(dict(paired=paired,scenes=scene_stats),indent=2)+'\n')
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11})
    fig,axs=plt.subplots(2,2,figsize=(16,11),facecolor='#f5f8fc')
    x=np.arange(len(keys));colors=['#526b83','#aa775c','#3b9296','#56a47e','#c39121','#7468b2','#c45771'][:len(keys)]
    for k,mode in enumerate(('uncensored','production')):
        ax=axs[0,k]
        vals=[s[mode][n]['total_ms']['mean'] for n in keys]
        bars=ax.bar(x,vals,color=colors);ax.bar_label(bars,fmt='%.1f',padding=3,fontsize=9)
        ax.plot(x,[s[mode][n]['total_ms']['p95'] for n in keys],'ko--',lw=1,label='p95')
        ax.plot(x,[s[mode][n]['total_ms']['max'] for n in keys],'r^:',lw=1,label='최대')
        ax.axhline(200,color='#a14b4b',ls='--',lw=.8)
        ax.set(title='시간 제한 완화 · 서로 다른 샘플 seed 8개' if k==0 else '현재 운영 예산 · 같은 장면 8주기',
               ylabel='전체 계산 시간 [ms]',ylim=(0,max(225,ax.get_ylim()[1])))
        ax.legend(fontsize=9,loc='upper left',ncol=2)
    ax=axs[1,0]
    for k,mode in enumerate(('uncensored','production')):
        vals=[paired[mode][name]['cost_change_percent'] for name in keys]
        bars=ax.bar(x+(k-.5)*.34,vals,.34,color=['#287ab0','#df963b'][k],label=['시간 제한 완화','현재 운영 예산'][k])
        ax.bar_label(bars,fmt='%+.2f',padding=3,fontsize=8)
    ax.axhline(0,color='black',lw=.7);ax.set(title='최종 MPPI 비용 · 현재 2회 방식 대비',ylabel='비용 변화 [%] · 음수 = 개선')
    ax.legend(fontsize=9)
    ax=axs[1,1]
    vals=[s['uncensored'][n]['optimized_at_minimum'] for n in keys]
    bars=ax.bar(x,vals,color=colors);ax.bar_label(bars,padding=3)
    ax.set(title='25cm에 머문 최적화 후보 · 총 400개 중',ylabel='후보 수',ylim=(0,max(vals)*1.2+1))
    for ax in axs.flat:
        ax.set_xticks(x,labels);ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True)
    fig.suptitle('이동 거리 보상으로 최대 거리 최적화를 대신할 수 있는가?',fontsize=20,fontweight='bold')
    fig.text(.06,.025,'seed 최적화: J기존 − λ × gap 방향 이동 거리 / 배분 점수: J기존 / 최종 500개 평가 비용 그대로\n'
             f'실제 bag 추출 16장면 × 8회 × 7방식 × 2조건 = {len(rows)}회. 원본 bag 재생·폐루프 주행·ROS 전체 주기 검증은 아님.',fontsize=10)
    fig.tight_layout(rect=(.02,.09,.98,.94));fig.savefig(OUT/'comparison.png',dpi=170);plt.close(fig)

    focus=f'reward_{args.focus:g}'
    selected=['fixed','reward_0',focus]
    for scene in ('b2_060','b2_100'):
        data=dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
        fig,axes=plt.subplots(1,3,figsize=(18,8),facecolor='#f5f8fc')
        for ax,name in zip(axes,selected):
            r=next(r for r in rows if r['scene']==scene and r['mode']=='uncensored' and r['trial']==0 and r['label']==name)
            for i in (0,1):
                h=data[f'scan_{i}_hits'];ax.scatter(h[:,0],h[:,1],s=2,color='#567083',alpha=.4)
            for region in r['regions']:
                pts=np.array(region['outline']);ax.plot(pts[:,0],pts[:,1],color='#91adbf',lw=.7)
            for i,anchor in enumerate(r['anchors']):
                p=np.vstack((np.zeros(2),np.array(anchor)[:,:2]));n=r['candidate_names'][i]
                color='#111111' if i==0 else plt.cm.tab10((i-1)%10)
                ax.plot(p[:,0],p[:,1],ls='--' if i==0 else '-',color=color,lw=2)
                ax.scatter(*p[-1],s=30,marker='x',color=color)
                ax.annotate(f'{n}/{r["counts"][i]}개',p[-1],xytext=(0,9 if i%2 else -16),textcoords='offset points',fontsize=8,ha='center')
            p=np.vstack((np.zeros(2),np.array(r['trajectory'])[:,:2]))
            ax.plot(p[:,0],p[:,1],color='#ce345a',lw=3,label='최종 MPPI')
            ax.plot(data['path'][:,0],data['path'][:,1],':',color='#536451',label='기준 경로')
            label='현재 2회' if name=='fixed' else f'1회 / λ={name.split("_")[1]}'
            ax.set(xlim=(-2.2,2.2),ylim=(-2.2,2.2),aspect='equal',xlabel='x [m]',ylabel='y [m]',
                   title=f'{label}\n최종 비용 {r["cost"]:.4f} / 시간 {r["total_ms"]:.1f}ms')
            ax.grid(alpha=.15);ax.legend(fontsize=8,loc='lower left')
            lines=[f'{q["id"]}: {q.get("projected_displacement_m",0):.2f}m / {q["reason"]}' for q in r['candidates']]
            ax.text(0,-.16,'\n'.join(lines),transform=ax.transAxes,fontsize=8,va='top')
        fig.suptitle(f'{scene} · 실제 scan / 동일 입력·샘플 난수 seed / 시간 제한 완화',fontsize=18)
        fig.subplots_adjust(top=.85,bottom=.28,wspace=.2)
        fig.savefig(OUT/f'{scene}_comparison.png',dpi=170);plt.close(fig)

    lines=['gap 이동 거리 보상 실험 — 2026-11-09','',
        '목적: 최대 거리 최적화를 빼고 J_base-lambda*rho 단일 최적화로 충분히 뻗는 seed를 만들 수 있는지 확인.',
        'rho=e_gap^T(p_N-p_0). 회전 허용. 기존 속도/가속도/휠 제한, LiDAR 깊이, 끝점 방향, 최소 이동 0.25m 동일.',
        'J_base=8||p_N-p_ref||²+4mean((e_perp^T p_t)²)+3mean((yaw_t-yaw_ref_t)²)',
        '       +mean(delta_u^T diag(0.5,0.5,0.055) delta_u).',
        'seed 최적화에만 -lambda*rho 적용. 샘플 배분은 결과 seed의 기존 J_base로 계산.',
        '최종 500개 MPPI 평가 비용/안전 검사/대표 선택/nominal 및 gap 중복 제거/배분 규칙은 동일.',
        '실험 namespace만 수정. 운영 소스와 YAML SHA/바이트 동일 검증. 노드 재시작 없음.',
        '', '방법:',
        '실제 bag 추출 저장 16장면(6장면 동적 예측 포함), 가중치 0/4/8/16/32/48 및 현재 2회 방식.',
        f'총 {len(rows)}회. 각 방식/조건별 128회. 원본 MCAP/DB3 신규 재생은 아님.',
        '시간 제한 완화: 후보 2초/주기 5초, 8개 paired sampling seed, fresh controller; 순서 순환/역순.',
        '현재 예산: 140ms 후보 상한/180ms 주기 마감/동적 MPPI 예약. 같은 장면에서 방식당 8회 연속, 블록 순서는 장면별 순환.',
        '7개 방식을 끼워 넣어 캐시가 인위적으로 만료되는 문제를 피하기 위해 운영 조건은 연속 실행.',
        '양쪽 모두 기록 nominal을 매회 복원. 운영 조건은 캐시/순회 이력 유지. 같은 장면 반복이므로 폐루프 아님.',
        '시간은 geometry/seed 생성/500개 평가/최종 검사 포함, ROS 입출력 및 전체 인식 부하 제외.',
        f'새 보상 목적함수 analytic gradient 유한차분 검사 {b["derivative_checks"]}개 통과.',
        '유효 수치해 제약, 보상 분리, 배분 점수 합계, 총 500개, 최종 안전 상태 확인.',
        '', '결과:']
    for mode in ('uncensored','production'):
        lines.append(mode)
        for name in keys:
            v=s[mode][name];t=v['total_ms'];a=paired[mode][name]
            lines += [f'{name}: 시간 평균/p95/최대={t["mean"]:.3f}/{t["p95"]:.3f}/{t["max"]:.3f}ms; 비용 {v["cost"]["mean"]:.6f} ({a["cost_change_percent"]:+.3f}%)',
                f'  정상={v["ok"]}/{v["cycles"]}; timeout={v["timeouts"]}; 200ms 초과={v["over_200_ms"]}; 최적화 성공·검사 통과={v["optimized"]}/{v["candidate_count"]}',
                f'  25cm 후보={v["optimized_at_minimum"]}; 평균 선택 gap={v["selected_gaps"]["mean"]:.3f}; nominal 중복={v["reasons"].get("duplicate_nominal",0)}; 비용 개선/악화/동일={a["better"]}/{a["worse"]}/{a["ties"]}']
    lines+=['',f'장면별 {focus} 비용 변화 (시간 제한 완화):']
    for scene,v in scene_stats['uncensored'][focus].items():
        lines.append(f'{scene}: 현재 {v["baseline_cost"]:.6f} -> 보상 {v["cost"]:.6f}, {v["change_percent"]:+.3f}%')
    lines+=['','판단 및 다음 검증 후보:',
        '1. 가중치 32/48에서 25cm 최적화 후보가 0개이고 평균 선택 gap 수가 현재 방식 수준으로 복원됨.',
        '   최대 거리 최적화 없이도 짧은 seed 현상을 줄일 수 있다는 효과는 확인됨.',
        '2. 가중치 32: 현재 운영 평균 110.45->91.49ms(-17.17%), p95 142.45ms, 최대 159.88ms.',
        '   비용은 운영 -0.81%, 시간 제한 완화 +0.97%. 일관된 최종 비용 개선은 아님.',
        '3. b2_060: 후방 두 seed 0.25/0.25m -> 0.85/0.56m (가중치 32).',
        '   이 장면의 시간 제한 완화 평균 비용 악화는 무보상 +37.98%에서 보상32 +1.91%로 줄어듦.',
        '4. 반대로 보상32에서 b0_024 +13.73%, b2_100 +7.86% 비용 악화. 평균만으로 전면 우위 판단 불가.',
        '5. 가중치 4는 평균 비용이 두 조건 모두 낮지만 여전히 184/400개 seed가 25cm에 머묾.',
        '6. 충분히 뻗는 탐색 seed를 유지하는 목적이라면 다음 검증 후보는 32.',
        '   시험한 값 중 25cm 현상을 없앤 가장 작은 값이며, 최적화 성공·검사 통과 392/400으로 48의 368/400보다 높음.',
        '   48에서는 정적 충돌 검사로 최적화 seed를 탈락시킨 경우도 8개 있어 단순히 보상을 키울 이유는 없음.',
        '7. 따라서 1회 보상 구조는 유망한 대체 후보이나 운영 교체를 확정하지 않음. 현재 2회 구현/YAML 유지.',
        '   32는 현재 비용 스케일과 이 장면 집합에서의 실험값이며 범용/최적 가중치가 아님.']
    lines+=['','해석 주의:',
        '이동 보상이 커도 정확한 최대 거리 보장은 아님. 다른 비용과 절충하며 SLSQP 국소 수치해임.',
        '25cm 후보 감소는 seed가 뻗는 현상 확인이며, 실차 후진 탈출/도착률 개선 증명은 아님.',
        '기존 2회와 seed 비용 가중치/목표 선택 정책이 달라짐. 이번에는 의도적으로 seed 목적함수를 변경한 비교임.',
        '가중치는 같은 16장면에서 탐색함. 별도 held-out 검증이나 통계적 유의성 주장은 하지 않음.',
        '시간 제한 완화와 운영 조건은 seed/캐시 조건도 달라 비용 부호 차이를 예산만의 효과로 해석하지 않음.',
        '낮은 평균 비용이 모든 장면 개선을 뜻하지 않음. 최종 MPPI 비용을 기준으로 비교하고 보상 포함 목적함수 값은 직접 비교하지 않음.',
        '', '재현:',
        'source /opt/ros/jazzy/setup.bash; source /home/smoc/2026amr_ws/install/setup.bash',
        'PYTHONPATH=planner/local_planner/DRA_MPPI:$PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 planner/local_planner/DRA_MPPI/tools/experiment_gap_distance_reward.py --output /tmp/new_reward_experiment',
        '', 'benchmark.json: 전체 결과 / analysis.json: paired 비용 분석 / comparison.png: 비교표 / experiment.py: 실험 스냅샷']
    (OUT/'RESULTS.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(paired,indent=2))


if __name__=='__main__':main()

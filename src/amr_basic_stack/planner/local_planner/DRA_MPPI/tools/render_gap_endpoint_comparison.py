#!/usr/bin/env python3
"""Render measured endpoint-policy comparison, including paired scene costs."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/gap_endpoint_policy_2026-11-09'
SOURCE=ROOT/'docs/gap_value_optimization_2026-11-05'


def main():
    report=json.loads((OUT/'benchmark.json').read_text());rows=report['rows'];s=report['summary']
    ids=[q['scene'] for q in report['sources']]
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11})
    colors=['#287ab0','#d47b26'];labels=['최대 거리 고정 · 2회','거리 자유 · 1회']
    paired={};scene_stats={}
    for mode in ('uncensored','production'):
        lookup={(r['scene'],r['trial'],r['label']):r for r in rows if r['mode']==mode}
        pairs=[(lookup[scene,t,'fixed'],lookup[scene,t,'free'])
               for scene in ids for t in range(report['iterations'])]
        complete=[(a,b) for a,b in pairs if 'cost' in a and 'cost' in b]
        dif=np.array([b['cost']-a['cost'] for a,b in complete])
        paired[mode]=dict(pairs=len(complete),free_better=int(sum(dif < -1e-6)),
            free_worse=int(sum(dif > 1e-6)),ties=int(sum(abs(dif)<=1e-6)),
            free_cost_change_percent=100*(np.mean([b['cost'] for a,b in complete])/
                                           np.mean([a['cost'] for a,b in complete])-1),
            free_time_change_percent=100*(s[mode]['free']['total_ms']['mean']/
                                           s[mode]['fixed']['total_ms']['mean']-1))
        scene_stats[mode]={}
        for scene in ids:
            group=[(a,b) for a,b in complete if a['scene']==scene]
            if not group:continue
            a=np.mean([a['cost'] for a,b in group]);b=np.mean([b['cost'] for a,b in group])
            scene_stats[mode][scene]=dict(fixed_cost=a,free_cost=b,free_change_percent=100*(b/a-1),
                fixed_time_ms=np.mean([a['total_ms'] for a,b in group]),
                free_time_ms=np.mean([b['total_ms'] for a,b in group]))
    (OUT/'analysis.json').write_text(json.dumps(dict(paired=paired,scenes=scene_stats),indent=2)+'\n')
    fig,axs=plt.subplots(2,2,figsize=(16,11),facecolor='#f5f8fc')
    for j,mode in enumerate(('uncensored','production')):
        ax=axs[0,j]
        for k,name in enumerate(('fixed','free')):
            vals=[s[mode][name]['total_ms'][v] for v in ('mean','p95','max')]
            bars=ax.bar(np.arange(3)+(k-.5)*.34,vals,.34,color=colors[k],label=labels[k])
            ax.bar_label(bars,fmt='%.1f',padding=3,fontsize=10)
        ax.axhline(200,ls='--',color='#b54141',label='5Hz = 200ms')
        ax.set_xticks(range(3),['평균','p95','최대'])
        ax.set(title='시간 제한 완화 · 서로 다른 샘플 seed 8개' if j==0 else '현재 운영 예산 · 같은 장면 8주기',
               ylabel='전체 계산 시간 [ms]',ylim=(0,max(220,ax.get_ylim()[1]+20)))
        ax.legend(fontsize=9,loc='upper left')
    ax=axs[1,0]
    values=[scene_stats['uncensored'][scene]['free_change_percent'] for scene in ids]
    ax.bar(range(len(ids)),values,color=[colors[1] if x>0 else colors[0] for x in values])
    ax.axhline(0,color='#333333',lw=1)
    ax.set_xticks(range(len(ids)),ids,rotation=65,ha='right',fontsize=9)
    ax.set(title='거리 자유 방식의 최종 MPPI 비용 변화 (시간 제한 완화)',
           ylabel='최대 거리 고정 대비 [%] · 양수 = 나빠짐')
    ax=axs[1,1]
    for k,name in enumerate(('fixed','free')):
        v=s['uncensored'][name]
        vals=[v['optimized_at_minimum'],v['reasons'].get('duplicate_nominal',0),v['optimized']]
        bars=ax.bar(np.arange(3)+(k-.5)*.34,vals,.34,color=colors[k],label=labels[k])
        ax.bar_label(bars,padding=3)
    ax.set_xticks(range(3),['25cm까지 축소','nominal 중복 제외','최적화 성공·검사 통과'])
    ax.set(title='후보 seed 결과 · 총 400개 (16장면 × 8회)',ylabel='후보 수')
    ax.legend(fontsize=9)
    for ax in axs.flat:ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True)
    fig.suptitle('gap 최대 거리 최적화가 필요한가? 실제 저장 장면 A/B 비교',fontsize=20,fontweight='bold',y=.98)
    fig.text(.06,.03,'두 방식 모두 회전 허용 / gap 방향·비용 가중치·500개 MPPI 평가·배분 규칙 동일\n'
        '원본 bag 추출 데이터 16장면, 총 512회. ROS 전체 주기 및 폐루프 주행 검증은 아님. p95 = 측정값 95백분위.',fontsize=10)
    fig.tight_layout(rect=(.02,.08,.98,.94));fig.savefig(OUT/'comparison.png',dpi=170);plt.close(fig)

    # Representative fixed snapshot: show both a large regression and an improvement.
    ordered=sorted(ids,key=lambda x:scene_stats['uncensored'][x]['free_change_percent'])
    scenes=list(dict.fromkeys(['b2_060',ordered[-1],ordered[0]]))
    for scene in scenes:
        data=dict(np.load(SOURCE/'raw'/f'{scene}.npz'))
        fig,axes=plt.subplots(1,2,figsize=(14,8),facecolor='#f5f8fc')
        for k,name in enumerate(('fixed','free')):
            r=next(r for r in rows if r['scene']==scene and r['mode']=='uncensored' and r['trial']==0 and r['label']==name)
            ax=axes[k]
            for i in (0,1):
                h=data[f'scan_{i}_hits'];ax.scatter(h[:,0],h[:,1],s=2,color='#567083',alpha=.5)
            for region in r['regions']:
                pts=np.array(region['outline']);ax.plot(pts[:,0],pts[:,1],color='#9ab1c3',lw=.7)
            for i,anchor in enumerate(r['anchors']):
                # MppiResult.guide_anchors contains rolled-out states, not controls.
                p=np.vstack((np.zeros(2),np.array(anchor)[:,:2]));name_i=r['candidate_names'][i]
                ax.plot(p[:,0],p[:,1],ls='--' if i==0 else '-',lw=2,
                        color='#111111' if i==0 else plt.cm.tab10((i-1)%10),alpha=.85)
                ax.scatter(*p[-1],s=40,marker='x',color='#111111' if i==0 else plt.cm.tab10((i-1)%10))
                ax.annotate(f'{name_i} / {r["counts"][i]}개',p[-1],xytext=(0,8 if i%2 else -16),
                            textcoords='offset points',fontsize=9,ha='center')
            p=np.vstack((np.zeros(2),np.array(r['trajectory'])[:,:2]))
            ax.plot(p[:,0],p[:,1],color='#d32f4d',lw=3,label='최종 MPPI 궤적')
            ax.plot(data['path'][:,0],data['path'][:,1],':',color='#536451',lw=1.2,label='기준 경로')
            ax.scatter(0,0,s=45,color='#213b50',zorder=5)
            ax.set(xlim=(-2.2,2.2),ylim=(-2.2,2.2),aspect='equal',xlabel='x [m] · 전방 +',ylabel='y [m] · 좌측 +',
                   title=f'{labels[k]}\n최종 비용 {r["cost"]:.4f} / 계산 {r["total_ms"]:.1f}ms')
            ax.grid(alpha=.15);ax.legend(loc='lower left',fontsize=9)
            bottom=' | '.join(f'{q["id"]}: {q.get("projected_displacement_m",0):.2f}m ({q["reason"]})' for q in r['candidates'])
            ax.text(0,-.15,'\n'.join(bottom.split(' | ')),transform=ax.transAxes,fontsize=9,va='top')
        fig.suptitle(f'{scene} · 같은 입력 / 같은 샘플 난수 seed / 시간 제한 완화',fontsize=17)
        fig.subplots_adjust(top=.85,bottom=.25,wspace=.2)
        fig.savefig(OUT/f'{scene}_comparison.png',dpi=170);plt.close(fig)

    lines=['최대 거리 고정(2회) vs gap 방향 내 거리 자유(1회) — 2026-11-09','',
        '실험 목적: 최대 거리 계산의 시간뿐 아니라 고정된 끝점이 최종 MPPI 결과에 주는 영향 비교.',
        '운영 구현 및 설정은 수정하지 않음. 별도 import namespace에서 free solver만 교체.',
        '', '동일 조건:',
        '- gap 구역/중앙 방향/nominal/지도/현재 속도/전후방 scan/동적 예측/공통 진행 기준점 동일.',
        '- 회전 omega는 양쪽 모두 허용. 속도, 가속도, 메카넘 휠 제한 동일.',
        '- 4항 seed 비용, 최소 이동 0.25m, nominal 중복 제거, 최대 3 gap, 비용별 총 500개 배분 동일.',
        '- 최종 500개 평가 비용/정적·동적 안전 검사/대표 선택 코드 동일. 소스 SHA256 검증.',
        '', '비교 정의:',
        'A fixed: D=max_U e^T p_N; L=min(D,d); L-0.01<=e^T p_N<=L, e_perp^T p_N=0에서 min_U J.',
        'B free: 별도 D 계산 없음. 0.25<=e^T p_N<=d, e_perp^T p_N=0에서 min_U J.',
        '두 방식 모두 각 중간 위치 e^T p_t<=d 및 운동 제약 적용. 정적 footprint 검사도 동일.',
        'J=8||p_N-p_ref||² +4 mean((e_perp^T p_t)²)+3 mean((yaw_t-yaw_ref_t)²)',
        '  +mean(delta_u^T diag(0.5,0.5,0.055) delta_u). 동적 장애물 항은 seed 비용에 없음.',
        'SLSQP 국소 수치해이며 전역 최적성 인증 없음. free는 단순히 D를 근사한 것이 아니라 끝점 선택 정책도 바꿈.',
        '', '자료와 방법:',
        '- 원본 MCAP/DB3 없이 보존된 실제 bag 추출 16장면. raw_sources.json의 bag 이름/시간 유지.',
        '- 동적 예측이 저장된 6장면은 최종 MPPI에 그대로 입력.',
        '- 시간 제한 완화: 후보 2초/전체 5초; 8개 다른 paired sampling seed, 각 fresh controller.',
        '- 운영 예산: 현재 140ms 후보 상한/180ms 전체 마감/동적 예약; 같은 장면 8주기, 캐시/순회 유지.',
        '- 매회 기록 nominal 복원. 두 방식 순차 교대 실행. 초기 라이브러리 warmup 제외.',
        '- 총 512번 compute. 128개 반복은 128개 독립 장면이 아님. CPU 단일 BLAS/OpenMP thread.',
        '- 시간은 geometry, 후보 생성, 500개 MPPI, 최종 검사 포함; ROS 입출력/센서 처리 부하 미포함.',
        '', '결과 (ms):']
    for mode in ('uncensored','production'):
        lines.append(mode)
        for name in ('fixed','free'):
            v=s[mode][name];t=v['total_ms']
            lines += [f'  {name}: 평균 {t["mean"]:.3f}, p95 {t["p95"]:.3f}, 최대 {t["max"]:.3f}; 정상 {v["ok"]}/{v["cycles"]}, 200ms 초과 {v["over_200_ms"]}, timeout {v["timeouts"]}',
                f'    후보 생성 평균 {v["proposal_ms"]["mean"]:.3f}; 최대 거리 풀이 합계/주기 {v["reach_ms"]["mean"]:.3f}; 비용 풀이 합계/주기 {v["actual_value_ms"]["mean"]:.3f}',
                f'    최종 비용 평균 {v["cost"]["mean"]:.6f}; 최적화 성공·검사 통과 {v["optimized"]}/{v["candidate_count"]}; 25cm 후보 {v["optimized_at_minimum"]}',
                f'    nominal 중복 제외 {v["reasons"].get("duplicate_nominal",0)}; 선택 gap 수 평균 {v["selected_gaps"]["mean"]:.3f}']
        p=paired[mode]
        lines.append(f'  free 비용 변화 {p["free_cost_change_percent"]:+.3f}%; 시간 변화 {p["free_time_change_percent"]:+.3f}%; 더 좋음/나쁨/동일={p["free_better"]}/{p["free_worse"]}/{p["ties"]}')
    lines+=['','장면별 평균 최종 비용 (시간 제한 완화; seed 8개):']
    for scene,v in scene_stats['uncensored'].items():
        lines.append(f'{scene}: fixed {v["fixed_cost"]:.6f}, free {v["free_cost"]:.6f}, 변화 {v["free_change_percent"]:+.3f}%')
    lines+=['','판단:',
        '1. 최대 거리 최적화의 필수성/평균 성능 우위는 입증되지 않음. free는 평균 약 19ms 빠름.',
        '2. 그러나 같은 비용을 쓰면 free는 400개 후보 중 208개를 25cm까지 줄이고, 선택 gap 수 평균이 2.44->2.00으로 감소.',
        '3. b2_060에서 gap_01 seed 거리는 1.49m->1.25m, seed 비용은 3.055->2.535로 낮아짐.',
        '   반면 최종 MPPI 비용은 8개 sampling seed 평균 1.967->2.714(+37.98%). seed 비용 감소가 최종 성능 개선은 아님.',
        '4. 운영 예산에서는 free 평균 최종 비용이 1.19% 더 낮고, 제한 완화에서는 1.00% 더 높음. 일관된 우승 방식 없음.',
        '5. 현재 목표가 gap 방향으로 충분히 뻗는 탐색 seed 보존이라면 기존 2회 구조를 당장 제거하지 않는 것을 권장.',
        '   이는 도달 한계 계산이 수학적으로 필수여서가 아니라, 동일 비용에서의 거리 축소를 막는 현재 설계의 효과 때문.',
        '6. 1회 방식 채택 여부를 결정하려면 후진 탈출/우회 폐루프 성공률로 이 거리 축소의 실제 영향을 추가 확인해야 함.',
        '   이번 작업은 비교 실험만 했으며 운영 planner/config 변경 없음.']
    lines+=['','해석 한계:',
        '- 낮은 seed 최적화 비용과 낮은 최종 MPPI 비용은 서로 다른 지표임.',
        '- 거리 자유는 후방 등 공통 기준점 반대쪽 후보를 짧게 만들 수 있고 중복 제거/배분/최종 샘플 분포도 달라짐.',
        '- 계산 시간 이익은 최대 거리 풀이 제거 시간과 같지 않음. 비용 풀이 반복 수, 선택 group 수도 달라짐.',
        '- 제한 완화/운영 실험은 샘플 seed 및 캐시 유지 조건도 다르므로 비용 변화의 부호 차이를 시간 제한만의 효과로 해석하지 않음.',
        '- 고정 snapshot 실험은 실제 도착률/탈출률/폐루프 정체/실차 5Hz 보장이 아님.',
        '- 결과는 이 가중치/중복 제거/초기값/16장면에서의 정책 비교. 최대 거리 최적화가 수학적으로 필수라는 증명 아님.',
        '', '재현:',
        'source /opt/ros/jazzy/setup.bash; source /home/smoc/2026amr_ws/install/setup.bash',
        'PYTHONPATH=planner/local_planner/DRA_MPPI:$PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 planner/local_planner/DRA_MPPI/tools/experiment_gap_endpoint_policy.py --output /tmp/new_gap_endpoint_experiment',
        '', '원자료: benchmark.json / 분석: analysis.json / 비교 그림: comparison.png / 실험코드: experiment.py']
    (OUT/'RESULTS.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(paired,indent=2))


if __name__=='__main__':main()

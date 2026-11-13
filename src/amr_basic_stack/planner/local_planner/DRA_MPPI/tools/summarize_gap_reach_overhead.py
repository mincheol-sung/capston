#!/usr/bin/env python3
"""Render measured reach-solver overhead and paired end-to-end comparisons."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/gap_reach_overhead_2026-11-09'


def stats(x):
    a=np.asarray(x,float)
    return dict(n=len(a),mean=float(a.mean()),p50=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max())) if len(a) else None


def main():
    report=json.loads((OUT/'comparison/benchmark.json').read_text())
    rows=report['rows'];summary=report['summary']
    lookup={(r['mode'],r['scene'],r['trial'],r['label']):r for r in rows}
    pairs={}
    for mode in ('uncensored','production'):
        pairs[mode]={}
        for other in ('current','oracle'):
            delta=[];changes=0;cost_delta=[];scene_costs={}
            for a in rows:
                if a['mode']!=mode or a['label']!='direct':continue
                b=lookup[mode,a['scene'],a['trial'],other]
                delta.append(a['total_ms']-b['total_ms'])
                if 'cost' in a and 'cost' in b:
                    changes+=a['selected_mode']!=b['selected_mode']
                    cost_delta.append(a['cost']-b['cost'])
                    scene_costs.setdefault(a['scene'],[]).append((a['cost'],b['cost']))
            pairs[mode]['direct_minus_'+other]=dict(total_ms=stats(delta),cost_delta=stats(cost_delta),
                selected_mode_changes=changes,
                per_scene_cost_percent={s:100*(np.mean(np.array(v)[:,0])-np.mean(np.array(v)[:,1]))/
                                            max(1e-9,abs(np.mean(np.array(v)[:,1]))) for s,v in scene_costs.items()})
    per_scene=[]
    for source in report['sources']:
        scene=source['scene'];item=dict(scene=scene,predictions=source['predictions'])
        for mode in ('uncensored','production'):
            item[mode]={}
            for label in ('current','direct','oracle'):
                r=[q for q in rows if q['scene']==scene and q['mode']==mode and q['label']==label]
                item[mode][label]=dict(total_ms=stats([q['total_ms'] for q in r]),
                    reach_ms=stats([q['reach_ms'] for q in r]),
                    optimized=sum(q.get('optimized',0) for q in r),
                    candidate_evaluations=sum(len(q.get('candidates',[])) for q in r))
        per_scene.append(item)
    details=dict(paired=pairs,per_scene=per_scene,
                 exact_direct_oracle_cycles=len(report['oracle_checks']),identical_reach_pairs=len(report['reach_pairs']))
    (OUT/'analysis.json').write_text(json.dumps(details,indent=2)+'\n')
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11})
    fig,axes=plt.subplots(2,2,figsize=(15,10),facecolor='#f5f8fc')
    labels=['현재 2~3회','제안 2회','거리 계산 제외*'];colors=['#587389','#d17c23','#368f91']
    for ax,mode,title in zip(axes[0],('uncensored','production'),('시간 제한 완화: 동일 작업량 비교','운영 예산: seed 50ms / 전체 180ms')):
        x=np.arange(3)
        for j,(key,alpha) in enumerate([('mean',1.),('p95',.65),('max',.35)]):
            values=[summary[mode][label]['total_ms'][key] for label in ('current','direct','oracle')]
            bars=ax.bar(x+(j-1)*.23,values,width=.23,color=colors,alpha=alpha,label={'mean':'평균','p95':'p95','max':'최대'}[key])
            for bar,value in zip(bars,values):ax.text(bar.get_x()+bar.get_width()/2,value+2,f'{value:.1f}',ha='center',fontsize=9)
        ax.axhline(200,color='#b83c3c',ls='--',label='5Hz 예산 200ms')
        ax.set_xticks(x,labels);ax.set_ylabel('전체 계산 시간 [ms]');ax.set_title(title,fontweight='bold')
        ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True);ax.set_ylim(0,max(220,ax.get_ylim()[1]+15))
        ax.text(.98,.97,'왼쪽부터 평균 / p95 / 최대',transform=ax.transAxes,ha='right',va='top',fontsize=9)
    ax=axes[1,0]
    x=np.arange(len(per_scene))
    reach=[s['uncensored']['direct']['reach_ms']['mean'] for s in per_scene]
    ax.bar(x,reach,color='#d17c23')
    ax.set_xticks(x,[s['scene'] for s in per_scene],rotation=55,ha='right',fontsize=9)
    ax.set_title('거리 최대화만 걸린 시간 — 장면 내 모든 gap 합계',fontweight='bold')
    ax.set_ylabel('평균 [ms / 주기]');ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
    ax=axes[1,1];ax.axis('off')
    direct=summary['uncensored']['direct'];prod=summary['production']['direct']
    count=sum(len(r.get('candidates',[])) for r in rows if r['mode']=='production' and r['label']=='direct')
    text=(f"거리 최대화 1회: 평균 {direct['reach_call_ms']['mean']:.2f}ms / p95 {direct['reach_call_ms']['p95']:.2f}ms\n\n"
          f"주기당 거리 최대화 합계: 평균 {direct['reach_cycle_ms']['mean']:.2f}ms\n\n"
          f"동일 작업에서 거리 계산의 전체 시간 증가: {pairs['uncensored']['direct_minus_oracle']['total_ms']['mean']:.2f}ms\n\n"
          f"운영 예산 제안 방식: 정상 {prod['ok']}/{prod['cycles']}회\n"
          f"200ms 초과 {prod['over_200_ms']}회 / timeout hold {prod['timeouts']}회\n\n"
          f"새 최적화 seed: {prod['optimized']}/{count}개 후보\n"
          "나머지는 재사용/기본 seed 또는 탈락 — 모든 gap 완료 보장 아님\n\n"
          f"운영 최종 비용 평균: 기존 대비 {100*(prod['cost']['mean']/summary['production']['current']['cost']['mean']-1):+.2f}%\n"
          "완료 후보 증가가 최종 비용 개선을 보장하지는 않음")
    ax.text(0,.94,text,va='top',fontsize=12,color='#213a50')
    fig.suptitle('bag 추출 16개 장면: 거리 최대화의 시간 부담과 두 단계 풀이 비교',fontsize=20,fontweight='bold',y=.98)
    fig.text(.5,.925,'각 방식·각 조건 128주기 · 같은 입력/500개 MPPI 평가 · 순서를 바꿔 순차 실행',ha='center',fontsize=12)
    fig.text(.06,.025,'* 거리 계산 제외군은 해당 장면의 동일한 거리 해를 사전 제공한 측정용 대조군입니다. 실시간 lookup 구현이 아닙니다.\n'
             '원본 MCAP/DB3는 현재 없음: 보존된 실제 bag 추출 데이터 재생. ROS 입출력·센서 처리 부하·폐루프 주행은 포함하지 않음.',fontsize=10,color='#4d657a')
    fig.subplots_adjust(top=.86,bottom=.15,hspace=.38,wspace=.24)
    fig.savefig(OUT/'comparison.png',dpi=180,facecolor=fig.get_facecolor());plt.close(fig)
    lines=['거리 최대화 계산 시간 비교 — 2026-11-09','',
           '데이터: 기존 세 bag에서 추출해 보존한 16개 장면, 각 8회. 6개 장면에 장애물 예측 포함.',
           '현재 bag 디렉터리는 metadata.yaml만 있으며 /home/smoc 및 /media에서 원본 MCAP/DB3를 찾지 못했다.',
           '운영 소스는 수정하지 않음. 소스 스냅샷/해시/설정/장면 출처/원시 측정은 comparison/benchmark.json 및 주변 파일.',
           '', '비교 방식:',
           'current: 운영 코드 그대로. 거리 최대화 -> wheel 비용 -> 필요한 경우 실제 목표점 비용.',
           'direct: 같은 거리 최대화 -> 실제 목표점 비용. wheel 비용 생략. actual 최적화 초기값은 reach 해.',
           'oracle: direct와 동일하지만 같은 현재 속도/방향/초기값의 reach 해를 사전 제공. 사전 계산 비용 제외.',
           'oracle은 비용 분리 실험용이며 실제 lookup, 근사 목표점, 회전 금지 거리와의 비교가 아니다.',
           '', '통제:',
           '총 500개 rollout, MPPI 비용/위험 평가/대표 선택/배분 규칙/현재 속도/nominal 입력 유지.',
           '반복마다 기록된 nominal 복원. mode 간/arm 간 별도 controller. 순서 교대; 병렬 벤치마크 안 함.',
           'uncensored: 후보 예산 2초, 전체 5초, 캐시 수명 3600초. 시간 만료 차이를 제거하는 진단용 설정.',
           'production: 원래 YAML 후보 50ms, 전체 180ms, 캐시 0.6초. 준비 시간에 따라 실제 남은 후보 예산도 달라짐.',
           '현재 360도 nominal 상대 세 방향 구현을 포함한다.',
           '같은 장면을 반복하는 frozen snapshot 실험이다. 실제 다음 위치로 이동하지 않는다.',
           '따라서 실제 주행 성능/실시간 ROS 5Hz의 보장은 아니다.',
           '', '측정값:']
    for mode in ('uncensored','production'):
        lines.append(mode)
        for label in ('current','direct','oracle'):
            s=summary[mode][label];t=s['total_ms']
            lines.append(f"  {label}: 평균 {t['mean']:.3f}, p95 {t['p95']:.3f}, 최대 {t['max']:.3f} ms; 정상 {s['ok']}/{s['cycles']}; 200ms 초과 {s['over_200_ms']}; 새 최적화 seed {s['optimized']}")
    lines += ['',f"direct 거리 최대화 단일 호출: {direct['reach_call_ms']}",
              f"direct 거리 최대화 주기 합계: {direct['reach_cycle_ms']}",
              f"동일 작업량에서 direct - oracle: {pairs['uncensored']['direct_minus_oracle']['total_ms']}",
              f"direct/oracle의 sampled_controls, rollout_costs, 최종 제어열, 선택 mode 완전 일치: {len(report['oracle_checks'])}회",
              f"current/direct 도달 거리 동일 검사: {len(report['reach_pairs'])} 후보 (실패 None 포함)",
              '', '주의: current와 direct의 실제 목표점은 같아도 초기값이 wheel 해 vs reach 해이므로 국소 해와 후보 순위가 달라질 수 있다.',
              '비용 변화, 선택 변화, 수렴/예산 초과는 analysis.json 및 comparison/benchmark.json에 기록했다.',
              '', '해석:',
              '거리 최대화는 후보당 평균 7.17ms, 모든 후보의 주기 합계 평균 22.41ms. 무시할 만큼 작은 비용은 아니다.',
              'b2_100은 후보 6개 거리 계산에만 평균 69.39ms. 50ms 후보 예산 안에 모든 후보를 완료할 수 없다.',
              '그럼에도 direct 방식의 운영 예산 전체 시간은 평균 96.56ms, p95 143.13ms, 최대 155.32ms이며 모두 200ms 이내였다.',
              '따라서 이 샘플들의 계산 시간만으로 거리 최대화 자체를 제거할 필요는 없다. 모든 gap의 새 해를 매번 구한다는 뜻은 아니다.',
              '운영 예산 새 최적화 seed는 current 213/400, direct 225/400. direct 81/128 주기에서 후보 예산 소진.',
              '시간 제한을 완화하면 최종 선택 mode는 current/direct 128회 모두 같고 평균 비용 차이 -0.019%.',
              '운영 예산에서는 direct 평균 비용 +0.986%, 선택 mode 7/128회 변경. b0_024는 +23.26%, b1_088은 +8.58%.',
              'b0_024 direct에서는 앞선 두 후보 계산 후 gap_03이 반복적으로 reach 단계에서 시간 소진.',
              'gap_03의 기본 seed는 duplicate_nominal로 제외되지만 current의 최적화 seed는 선택되고 최종 승자가 됨.',
              '그 때문에 두 단계로 줄이는 것만으로 최종 성능이 무조건 좋아진다고 결론 내릴 수 없다.',
              '제안: 거리 최대화는 유지할 만함. 두 단계 구조는 합리적이나 50ms 내 후보 순회/재사용을 보완해 이 회귀를 해결한 후 운영 반영.',
              '벽시계 캐시 만료로 동등 비교가 깨진 첫 시도는 상위 partial.json에 남아 있으며 최종 통계에서 제외했다.',
              '', '재현: ROS Jazzy/install setup을 source한 뒤 아래 명령. 출력 디렉터리는 새 경로 사용.',
              'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 tools/experiment_gap_reach_overhead.py --iterations 8 --output <new_output>',
              '그림 및 요약: /usr/bin/python3 tools/summarize_gap_reach_overhead.py']
    (OUT/'RESULTS.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(details['paired'],indent=2))


if __name__=='__main__':main()

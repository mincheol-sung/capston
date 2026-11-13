#!/usr/bin/env python3
"""Summarize the measured production two-solve/scheduling revision."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/gap_two_solve_scheduler_2026-11-09'


def main():
    report=json.loads((OUT/'comparison/benchmark.json').read_text());s=report['summary']
    labels=['변경 전','2회 풀이만 적용','2회 + 순서/예산 보완'];keys=['current','direct','updated']
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':12})
    fig,axes=plt.subplots(1,3,figsize=(16,6),facecolor='#f5f8fc');colors=['#667f94','#ca8b40','#168d8b']
    for j,metric in enumerate(['mean','p95','max']):
        values=[s[k]['total_ms'][metric] for k in keys]
        bars=axes[0].bar(np.arange(3)+(j-1)*.23,values,.23,color=colors,alpha=[1,.7,.4][j])
        for bar,v in zip(bars,values):axes[0].text(bar.get_x()+bar.get_width()/2,v+2,f'{v:.1f}',ha='center',fontsize=8)
    axes[0].axhline(200,color='#b93a3a',ls='--')
    axes[0].set(ylim=(0,220),ylabel='전체 계산 시간 [ms]',title='각 묶음: 평균 / p95 / 최대')
    for ax,metric,title in [(axes[1],'optimized','새 최적화 완료 후보 / 총 400개'),
                            (axes[2],'budget_exhausted','후보 시간 예산 소진 / 총 128주기')]:
        values=[s[k][metric] for k in keys]
        bars=ax.bar(np.arange(3),values,color=colors,width=.6)
        for bar,v in zip(bars,values):ax.text(bar.get_x()+bar.get_width()/2,v+2,str(v),ha='center')
        ax.set_title(title);ax.set_ylim(0,430 if metric=='optimized' else 135)
    for ax in axes:
        ax.set_xticks(np.arange(3),labels,fontsize=10);ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True)
    fig.suptitle('운영 구현 검증: 두 단계 최적화 + 공정한 후보 순회 + 동적 시간 예산',fontsize=19,fontweight='bold',y=.96)
    fig.text(.5,.875,'실제 bag 추출 16장면 × 8회 / 방식별 128주기 / 500개 MPPI 비용·안전 검사 동일',ha='center',fontsize=12)
    fig.text(.06,.08,'변경 후 128회 모두 정상 반환, 200ms 초과 0회. 최종 비용 평균은 변경 전 대비 −0.62%.\n'
             '원본 MCAP는 현재 없음: 보존된 추출 데이터 재생. ROS 전체 부하·폐루프 주행 검증이 아니며 모든 후보의 매주기 완료를 보장하지 않음.',fontsize=10,color='#486176')
    fig.subplots_adjust(top=.77,bottom=.22,wspace=.28)
    fig.savefig(OUT/'comparison.png',dpi=180,facecolor=fig.get_facecolor());plt.close(fig)
    lines=['두 단계 풀이 및 gap 순회/예산 보완 구현 — 2026-11-09','',
           '변경:',
           '1. 회전 허용 ReachProblem -> min(D,d) 고정 Actual ValueProblem. 별도 wheel-only 비용 계산 제거.',
           '2. 마지막 시도 주기가 오래된 후보 우선. 같은 주기에 시도했다면 종료하지 못한 후보 우선.',
           '   이력은 세계 좌표 방향, 구역 겹침, 위치/시간 조건으로 연결하며 frame-local gap ID에 의존하지 않음.',
           '   계산 착수 후 시간 초과로 마지막 후보가 계속 밀리던 경우를 해결하고 반복 실패 후보의 독점도 방지.',
           '3. 후보 예산 = min(140ms, 180ms 마감까지 남은 시간 - MPPI 예약).',
           '   최초/예측 배열 차원 변경: 100ms 예약. 이후 max(50ms, 최근 16주기 후보 이후 계산시간 최대 + 15ms).',
           '   단순한 부하 추정이며 실행시간 보장은 아님. cooperative deadline 검사, 초과 결과 폐기 및 hold 유지.',
           '4. 총 500개 배분, 그룹 MPPI 비용, 정적/동적 최종 검사, nominal 갱신, 5Hz 설정 유지.',
           '5. wheel_value_cost/value_difference는 null, wheel_value_status=not_computed. 실제 비용 출처 구분 유지.',
           '', '검증:',
           'pytest planner/local_planner/DRA_MPPI/test -q: 276 passed.',
           'colcon build --packages-select dra_mppi --symlink-install: 성공. 실행 중 노드는 재시작하지 않음.',
           '이전 세 bag에서 추출해 보존한 실제 16개 장면(6개 동적 예측 포함), 각 방식 8회, 총 384회.',
           '현재 원본 MCAP/DB3는 발견되지 않아 새 bag 추출/ROS 재생은 수행하지 않음.',
           '순서를 교대하며 순차 실행. 장면마다 별도 controller, 반복마다 기록 nominal 복원, 캐시/순회 이력 유지.',
           'baseline_package는 수정 전 운영 소스, comparison/updated_package는 검증한 변경 후 소스.',
           'common_cost/mppi_controller/guide_conditioned/dynamic_risk/gaussian_disk는 수정 전과 바이트 동일 확인.',
           '', '같은 실행에서 측정한 결과 (ms):']
    for k in keys:
        t=s[k]['total_ms']
        lines.append(f"{k}: 평균 {t['mean']:.3f}, p95 {t['p95']:.3f}, 최대 {t['max']:.3f}; 정상 {s[k]['ok']}/{s[k]['cycles']}; 시간초과 {s[k]['timeouts']}; 200ms 초과 {s[k]['over_200_ms']}")
        lines.append(f"  완료 {s[k]['optimized']}/400; 미착수 {s[k]['not_attempted']}; 시간 중단 {s[k]['interrupted']}; 예산 소진 주기 {s[k]['budget_exhausted']}/128; 비용 평균 {s[k]['cost']['mean']:.6f}")
    lines += ['', '해석 및 한계:',
              '새 최적화 완료율 51.5% -> 80.75%, 미착수 103 -> 15개. 후보 예산 소진 63.28% -> 31.25%.',
              '더 많은 후보를 풀어 평균 전체 시간이 97.61 -> 111.08ms로 증가. p95 164.13ms, 최대 168.12ms.',
              '최종 비용 평균은 기존 대비 -0.62%, 두 단계만 바꾼 버전 대비 -0.73%. 일반적 주행 성능 개선 증명은 아님.',
              '문제였던 b0_024: 새 버전 8회 모두 gap_03 선택, 평균 비용 2.073. 기존 2.169, 두 단계만 변경 2.674.',
              'b1_088은 기존 3.141, 두 단계만 변경 2.681, 보완 후 3.140. 모든 장면이 모든 비교군보다 개선되지는 않음.',
              '실제 연속 주행, ROS 입출력 및 전체 인지 부하는 포함하지 않음. 샘플 비용/안전 통과는 폐루프 안전 보장이 아님.',
              '시간 기반 캐시/solver deadline 때문에 이전 실험 숫자와 조금 달라질 수 있음. 최종 비교는 같은 실행의 paired 수치 사용.',
              '', '재현:',
              'ROS Jazzy와 workspace install setup을 source한다. 새 output 경로를 지정한다.',
              'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 tools/validate_gap_scheduler.py --iterations 8 --output <fresh_path>',
              '기본 결과 요약 그림: /usr/bin/python3 tools/render_gap_scheduler_validation.py']
    (OUT/'IMPLEMENTATION.txt').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()

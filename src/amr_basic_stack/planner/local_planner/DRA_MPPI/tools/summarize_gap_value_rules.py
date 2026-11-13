"""Summarize the paired midpoint-gap ranking experiment and make a figure."""
from pathlib import Path
import collections
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/gap_multiscene_2026-11-04/value_rule_comparison'
METHODS=['common','endpoint','free_area','novelty','common_weighted']
LABELS={'common':'기존 비용 / 균등 배분','endpoint':'경로 앞 목표점까지 거리','free_area':'열린 면적 우선',
        'novelty':'nominal과 차이 우선','common_weighted':'기존 비용 / 비용별 배분'}


def load(name):return [json.loads(s) for s in (OUT/name).read_text().splitlines()]


def paired(base,rows):
    deltas=[];by_scene=collections.defaultdict(list)
    for row in rows:
        b=base[row['scene'],row['seed']]
        if b['safe'] and row['safe'] and b['cost'] is not None and row['cost'] is not None:
            difference=row['cost']-b['cost'];deltas.append(difference);by_scene[row['scene']].append(difference)
    scene=np.array([np.mean(v) for v in by_scene.values() if len(v)==2]);deltas=np.array(deltas)
    return dict(paired_safe_trials=len(deltas),paired_safe_scenes=len(scene),
        trial_better=int(sum(deltas<-.1)),trial_similar=int(sum(abs(deltas)<=.1)),trial_worse=int(sum(deltas>.1)),
        scene_better=int(sum(scene<-.1)),scene_similar=int(sum(abs(scene)<=.1)),scene_worse=int(sum(scene>.1)),
        mean_delta=float(deltas.mean()),median_delta=float(np.median(deltas)),
        min_delta=float(deltas.min()),max_delta=float(deltas.max()),
        exact_equal_trials=int(sum(abs(deltas)<1e-9)))


def main():
    audit=load('scene_audit.jsonl');trials=load('trials.jsonl')
    assert len(audit)==127 and len(trials)==1270
    assert len({(r['scene'],r['method'],r['seed']) for r in trials})==1270
    base={(r['scene'],r['seed']):r for r in trials if r['method']=='common'}
    summary=dict(scenes=len(audit),logical_trials=len(trials),
        actual_solver_calls=sum(not r['reused_exact_result'] for r in trials),
        raw_regions=sum(r['region_count'] for r in audit),eligible_seeds=sum(r['eligible_count'] for r in audit),
        scenes_all_eligible_retained=sum(r['eligible_count']==len(r['selections']['common']['ids']) for r in audit),
        scenes_no_eligible=sum(r['eligible_count']==0 for r in audit),
        scenes_over_three_eligible=sum(r['eligible_count']>3 for r in audit),
        methods={})
    for method in METHODS:
        rows=[r for r in trials if r['method']==method]
        assert len(rows)==254
        for r in rows:
            assert sum(r['counts'])==500
            if r['reused_exact_result']:
                source=next(s for s in trials if s['scene']==r['scene'] and s['seed']==r['seed'] and s['method']==r['source_method'])
                assert source['guide_ids']==r['guide_ids'] and source['counts']==r['counts'] and source['cost']==r['cost']
        score_key='common' if method=='common_weighted' else method
        elapsed=[r['score_timing'][score_key]['median_ms'] for r in audit if r['eligible_count']]
        result=dict(safe=sum(r['safe'] for r in rows),moving=sum(r['moving'] for r in rows),
            nominal_selected=sum(r['selected']=='nominal' for r in rows),
            changed_selection_scenes=sum(r['selections'][method]['ids']!=r['selections']['common']['ids'] for r in audit),
            score_median_ms=float(np.median(elapsed)),score_p95_ms=float(np.percentile(elapsed,95)),
            all=paired(base,rows),development_ten=paired(base,[r for r in rows if r['development_ten']]),
            additional_117=paired(base,[r for r in rows if not r['development_ten']]))
        summary['methods'][method]=result
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    lines=['Gap 탐색 가치 평가 기준 비교 — 실제 127 snapshot, noise seed 7/23','',
        '결론: 실행 가능성/nominal 중복으로 자격을 먼저 판단하고, 대안 선택이 필요할 때 기존 공통+정적 비용을 사용한다.',
        '기존 비용의 순위 계산 추가 부담은 약 0.76 ms로 작았다. 목표점 거리만 쓰는 대안도 대부분 같았지만 우월성을 보이지 않았다.',
        '면적/nominal과의 차이는 유용성 자체를 잘 대표하지 못했다. 비용별 rollout 배분은 이번 실험에서 일관된 이득이 확인되지 않았다.','',
        f'논리 비교 {len(trials)}건, 동일 입력/seed/배분을 재사용한 실제 solver 호출 {summary["actual_solver_calls"]}건.',
        '각 조건: 127장면 x 2 noise seed = 254건. 모두 중앙 seed, 같은 자격 검사, 최대3 guide, 총500 rollout.',
        '4개 순위 기준은 동일한 균등 배분으로 비교. 5번째만 기존 비용 순위를 유지하고 최소50+비용 가중 배분을 적용.',
        'scene의 순위 점수 계산을 10회 반복한 중앙값을 수집했다. 아래 시간은 그 값들의 중앙값이다.',
        'seed 생성/충돌 검사/nominal RMS/면적 준비/입력 구성/MC/MPPI/ROS는 점수 계산 시간에 포함하지 않는다.',
        '면적과 novelty는 준비된 수치를 재사용한다. 따라서 표의 극소 시간은 전체 후보 평가 시간의 의미가 아니다.','',
        '기준: 기존 비용+균등 배분. ΔJ는 대안의 최종 MPPI 공통 비용에서 기준 비용을 뺀 값(양수=악화).',
        '유리/유사/불리는 두 noise seed의 장면 평균 ΔJ로 계산하고 ±0.1을 유사 범위로 둔다.',
        '이는 표시 편의를 위한 절대 문턱이며 통계적 유의성 검정이 아니다. 양쪽 안전 판정을 통과한 125장면만 비용을 비교한다.','',
        '방법 | 점수 계산 median ms | 선택 집합 변경 장면 | 유리/유사/불리 | 평균 ΔJ | 안전 검사 통과']
    for m in METHODS:
        r=summary['methods'][m];d=r['all']
        lines.append(f'{LABELS[m]} | {r["score_median_ms"]:.6f} | {r["changed_selection_scenes"]}/127 | '
            f'{d["scene_better"]}/{d["scene_similar"]}/{d["scene_worse"]} | {d["mean_delta"]:+.6f} | {r["safe"]}/254')
    lines += ['',f'자격 통과 seed를 모두 남긴 장면: {summary["scenes_all_eligible_retained"]}/127 (적격 후보0인 {summary["scenes_no_eligible"]}장면 포함).',
        '적격 후보가 최대3개이고 상호 중복이 없으면 순위 평가를 생략하고 전부 탐색할 수 있다.',
        '37장면은 적격 후보가3개를 초과했고, 추가3장면은 후보3개 이하에서도 후보끼리 중복되어 우선순위가 필요했다.',
        '비용별 개수 배분까지 쓰면 선택 순위를 생략할 수 있는 장면에서도 배분을 위한 비용 계산은 필요할 수 있다.','',
        '실제 예: b1_088의 면적 기준은 큰 후방 공간을 우선하고 작은 전방 G4를 제외했다.',
        '기존 비용 선택 G2/G3/G4 → 최종 비용 seed7 4.40745, seed23 4.16162.',
        '면적 우선 선택 G1/G2/G3 → 최종 비용 seed7 10.39655, seed23 9.71759.',
        '넓게 열렸다는 것만으로 그 gap의 탐색 유용성이 높다고 볼 수 없음을 보여주는 장면이다.','',
        '비용별 배분: 안전한250 paired trial에서 3개 개선/245개 유사/2개 악화(±0.1).',
        '두 seed를 장면별 평균하면 0개 개선/124개 유사/1개 악화. 평균 ΔJ는 약 -0.000918.',
        '배분식을 무조건 채택할 근거로 보기 어렵다. 초기 기본값은 균등 배분을 권고하며 비용별 배분은 선택 기능으로 남긴다.','',
        '한계: 같은3개 bag의 오프라인 단일시점 재계산이며 실제 폐루프 회피/도착 성공률을 측정하지 않았다.',
        '250/254는 기존 MC/정적 최종 검사 통과 횟수이지 실제 무충돌 주행률이 아니다.',
        '기준 방식에서 nominal이 최종 선택된 경우가201/254로 많아 순위 변경 효과가 작게 드러날 수 있다.',
        '2개 noise seed와 같은 데이터에서의 설정 개발만으로 평가 방법의 일반적 우열이나 통계적 동등성을 주장하지 않는다.',
        'runtime은 offline timeout10초로 결과를 모두 얻었다. full-stack5Hz 검증이 아니다.',
        '개발에 사용한10장면과 나머지117장면의 통계는 summary.json에 별도로 저장했다. 나머지117도 같은 bag의 기존 기록이다.','',
        '재현: ROS/workspace setup 후 tools/compare_gap_value_rules.py 실행, 이어 tools/summarize_gap_value_rules.py 실행.',
        '원본: protocol.json / scene_audit.jsonl / trials.jsonl / scenes/*.npz.']
    (OUT/'RESULTS.txt').write_text('\n'.join(lines)+'\n')
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False})
    fig=plt.figure(figsize=(16,9),facecolor='#f7f9fc')
    fig.text(.045,.93,'gap 평가 기준 비교 · 실제 127장면 × noise seed 2개',fontsize=24,weight='bold',color='#19364e')
    fig.text(.045,.87,'중앙 seed 1개/gap · 같은 자격 검사 · 총 500 rollout · 기존 MPPI/MC로 최종 비용 비교',fontsize=13,color='#526c82')
    ax=fig.add_axes([.045,.31,.91,.47]);ax.axis('off')
    table_rows=[]
    for m in METHODS:
        r=summary['methods'][m];d=r['all']
        table_rows.append([LABELS[m],f'{r["score_median_ms"]:.4f}',f'{r["changed_selection_scenes"]}/127',
            f'{d["scene_better"]} / {d["scene_similar"]} / {d["scene_worse"]}',f'{d["mean_delta"]:+.4f}',f'{r["safe"]}/254'])
    table=ax.table(cellText=table_rows,colLabels=['평가 / 배분','점수 시간 ms','선택 집합 변경','개선 / 유사 / 악화','평균 ΔJ','최종 검사 통과'],
                   cellLoc='center',colWidths=[.29,.12,.13,.18,.13,.15],bbox=(0,0,1,1))
    table.auto_set_font_size(False);table.set_fontsize(12)
    for (row,col),cell in table.get_celld().items():
        cell.set_edgecolor('#d3dfea');cell.set_facecolor('#e7eef8' if row==0 else 'white')
    fig.text(.05,.24,'권고: 간단한 자격 검사 → 선택 경쟁이 있을 때만 기존 비용으로 순위 결정',fontsize=18,color='#19364e',weight='bold')
    fig.text(.05,.18,'87/127장면에서는 적격 후보를 모두 유지. 비용별 rollout 배분의 일관된 이득은 확인되지 않음.',fontsize=13,color='#19364e')
    fig.text(.05,.12,'개선/유사/악화: 두 noise seed의 장면 평균 ΔJ, ±0.1은 유사. 양쪽 안전 검사 통과한 125장면 비교.',fontsize=11,color='#526c82')
    fig.text(.05,.08,'점수 시간은 seed·공통 필터 계산을 제외한 추가 평가 시간. 비용별 배분 행은 비용 평가 시간만 표시.',fontsize=11,color='#526c82')
    fig.text(.05,.04,'오프라인 단일시점 실험. 실제 주행 성공률·전체 5Hz·통계적 우월성 검증 아님. 동일 조건은 결과 재사용.',fontsize=11,color='#526c82')
    fig.savefig(OUT/'comparison.png',dpi=170);plt.close(fig)
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()

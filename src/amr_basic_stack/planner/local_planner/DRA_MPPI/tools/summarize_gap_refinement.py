#!/usr/bin/env python3
"""Report stage-two results separately from the preregistered comparison."""
import argparse
import json
from pathlib import Path
import shutil
import zipfile
import numpy as np
import matplotlib.pyplot as plt
import summarize_gap_multiscene as summary

NEW=['valley_direction_cost','valley_direction_gate']
summary.METHODS+=NEW
summary.LABELS.update(valley_direction_cost='구간 내 3방향 비용 비교',valley_direction_gate='3방향 비용 비교 + 필요성 gate')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('base',type=Path)
    args=parser.parse_args();base=args.base;out=base/'refinement'
    old=summary.load_lines(base/'trials.jsonl');new=summary.load_lines(out/'trials.jsonl');rows=old+new
    audit=summary.load_lines(base/'scene_audit.jsonl');valid={x['scene']:x for x in audit if x['accepted']}
    refinement={x['scene']:x for x in summary.load_lines(out/'scene_audit.jsonl')}
    assert len(new)==len(valid)*4
    assert len({(r['scene'],r['seed'],r['method']) for r in new})==len(new)
    result={}
    for split in ['all','design','holdout']:
        part=[r for r in rows if split=='all' or r['split']==split]
        result[split]=dict(methods={m:summary.reduce_rows([r for r in part if r['method']==m]) for m in summary.METHODS},
                           paired_to_radial=summary.paired(part),
                           paired_to_fixed5=summary.paired(part,'fixed5_warm'),
                           paired_to_direction_cost=summary.paired(part,'valley_direction_cost'))
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    chosen=['fixed5_warm','radial_R','radial_noR','connected_noR','valley_cost','valley_direction_cost','valley_direction_gate']
    fig,ax=plt.subplots(figsize=(17,8.5),dpi=150);ax.axis('off')
    fig.text(.04,.93,'추가 실험 · gap의 대표 방향도 비용으로 선택',fontsize=23,color=summary.INK,weight='bold')
    fig.text(.04,.875,'1차 결과를 보고 추가한 탐색적 실험 · 동일 127개 장면을 재사용 · 별도의 새 검증 데이터는 아님',fontsize=12,color=summary.MUTED)
    cells=[]
    for method in chosen:
        s=result['all']['methods'][method];p=result['all']['paired_to_radial'][method]
        cells.append([summary.LABELS[method],f'{s["gap_mean"]:.2f}',f'{p["better"]}/{p["tied"]}/{p["worse"]}',
                      f'{p["delta_mean"]:+.3f}',f'{s["time_p95"]:.1f}',f'{s["over_200"]}/{s["completed"]}'])
    table=ax.table(cellText=cells,colLabels=['방식','추가 gap 평균','비용 감소/유사/증가 장면','비용 차이 평균¹','offline p95 ms','>200 ms'],
                   colWidths=[.30,.12,.23,.12,.12,.11],cellLoc='center',bbox=[0,.16,1,.68])
    table.auto_set_font_size(False);table.set_fontsize(11)
    for (r,c),cell in table.get_celld().items():
        cell.set_edgecolor('#d6e1ec');cell.set_facecolor('#e8eff8' if r==0 else 'white')
        if c==0 and r:cell.set_text_props(ha='left')
    fig.text(.04,.105,'¹ 기존 거리별 gap+점수 대비. 두 방식 모두 안전 이동 선택인 124개 장면; 각 장면 두 seed 차이를 평균. 비용 중앙값 차이는 모두 0에 가까움.',fontsize=10,color=summary.MUTED)
    fig.text(.04,.06,'대표 후보: 가장 깊은 방향 / 구간 중앙 / 경로 방향을 구간 안에 투영한 방향. 제약 rollout 후 기존 비용이 낮은 후보를 gap 대표로 사용.',fontsize=10,color=summary.MUTED)
    fig.savefig(out/'01_refinement_comparison.png',dpi=150);plt.close(fig)
    lookup={(r['scene'],r['seed'],r['method']):r for r in rows}
    gallery=out/'gallery';(gallery/'scenes').mkdir(parents=True,exist_ok=True)
    for scene,label in [('b2_084','깊은 방향만 선택했을 때의 손실 보완'),('b1_088','먼 목표 때문에 기각하던 전방 후보 복구')]:
        a=np.load(base/'scenes'/f'{scene}.npz');b=np.load(out/'scenes'/f'{scene}.npz')
        np.savez_compressed(gallery/'scenes'/f'{scene}.npz',**{**dict(a),**dict(b)})
        merged=valid[scene].copy();merged['proposals']={**valid[scene]['proposals'],**refinement[scene]['proposals']}
        summary.draw_scene(gallery,merged,lookup,f'{scene}.png',label,
                           methods=['radial_R','valley_cost','valley_direction_cost'])
    write_findings(base,result,old,new,audit,refinement)
    for name in ['experiment_gap_direction_refinement.py','summarize_gap_multiscene.py','summarize_gap_refinement.py']:
        shutil.copy2(Path(__file__).parent/name,out/name)
    # Portable review bundle includes data summaries, plots, sources and scene arrays.
    with zipfile.ZipFile(base/'gap_multiscene_results.zip','w',zipfile.ZIP_DEFLATED) as z:
        for file in sorted(base.rglob('*')):
            if file.is_file() and file.suffix in ('.md','.png','.json','.jsonl','.csv','.npz','.py','.yaml') and '__pycache__' not in file.parts:
                z.write(file,str(file.relative_to(base)))
    for method in NEW:
        print(method,result['all']['methods'][method],
              {k:v for k,v in result['all']['paired_to_radial'][method].items() if k!='scene_deltas'})


def write_findings(base,result,old,new,audit,refinement):
    a=result['all']['methods'];p=result['all']['paired_to_radial']
    b=result['all']['paired_to_direction_cost']['valley_direction_gate']
    fixed=result['all']['paired_to_fixed5']['valley_direction_cost']
    scene_means={}
    for scene in ['b2_077','b1_088','b2_084']:
        scene_means[scene]={method:float(np.mean([r['cost'] for r in old+new if r['scene']==scene and r['method']==method]))
                            for method in ['radial_R','radial_noR','valley_cost','valley_direction_cost']}
    candidates=refinement['b2_084']['proposals']['valley_direction_cost']
    direction_options=candidates[0]['direction_options']
    option_rows='\n'.join(f'| {x["angle_deg"]:.0f}° | {x["cost"]:.3f} | {x["reach_error"]:.3f} |' for x in direction_options)
    report=f'''# 실험을 바탕으로 한 설계 판단

**권고 후보는 “깊이 골짜기로 나눈 각도 구간 + 구간 내 여러 대표 방향의 기존 비용 비교 + 도달 가능한 목표 거리”다.** 별도 경로 근접성 R은 기존 MPPI 경로 비용을 사용하는 selector로 대체할 수 있다. 다만 이번 결과로 운영 기본값을 바꾸거나 실시간 5 Hz 통과를 선언할 근거는 아직 부족하다.

## 실험량과 한계

세 실제 bag, 약 2초 간격 후보 363개 중 입력 유효 127개. 최초 9방식×2개 제어 noise seed=2,286회, 추가 2방식×2개 seed=508회, 합계 **2,794회**다. 기준 방식과 안전 이동 후보를 함께 얻은 비용 비교 장면은 124개다. 나머지는 공통 hold/stop 등으로 비용 우열 표에서 제외하되 전체 상태/시간에는 포함했다.

첫 9방식은 결과 확인 전에 설정을 고정했다. 추가 2방식은 1차 결과의 실패 사례를 보고 만든 **탐색적 후속 실험**이며 같은 bag을 재사용했다. 추가 두 bag에서도 결과를 나누어 표시했지만 후속 방법에 대해서는 그 bag들이 더 이상 완전히 손대지 않은 검증 집합이 아니다. 새로운 독립 bag의 검증이 필요하다.

이번은 같은 기록 pose에서 새 제어 후보를 계산한 snapshot 비교다. 실제 주행 성공률, 교착 탈출률, 충돌률을 측정하지 않았다.

## 1. “거리 분할을 없애면 R은 필요 없다”는 인과관계는 성립하지 않았다

기존 거리별 후보에서 R 항만 빼고 다른 가중치 및 V≥0.25를 유지하자 후보 집합이 **24/127장면**에서 달라졌다. 비용 비교 가능한 124개 중 0개 개선, 2개 악화, 122개 유사(장면 평균 비용차 ±0.1)였다.

특히 `135606` bag **164.607초**에서 유용한 174° gap의 점수가 R 포함 0.366 → R 제거 0.236으로 떨어져 0.25 기준에 못 미쳤다. 그 gap의 seed 기존 비용은 15.762로 기록 nominal의 43.743보다 좋았다.

- 기존 방식 최종 비용: 두 seed 평균 **{scene_means['b2_077']['radial_R']:.3f}**.
- R만 제거: **{scene_means['b2_077']['radial_noR']:.3f}**.
- 구간 대표 방향도 기존 비용으로 선택: **{scene_means['b2_077']['valley_direction_cost']:.3f}**.

이 사례는 R 자체가 필수라는 증거가 아니다. R 제거 시 threshold를 그대로 둔 것이 기각을 일으켰다. **경로 관련 판단이 필요하며, 그것을 수작업 R 점수로 할지 기존 MPPI 비용으로 할지가 설계 선택**이라는 뜻이다. 후보 수를 맞춘 threshold 재조정 실험까지 한 것은 아니다.

![R 제거 실제 사례](06_scene_b2_077.png)

## 2. 거리별 후보 반복은 줄이되, 큰 열린 각도 구간 하나로 뭉치지는 않는 편이 좋다

연결 각도 구간만 사용한 방식은 후보를 단순화했지만 기준보다 5장면 개선/14장면 악화였다. 가까운 범위에서 연결돼 있어도 먼 곳에서 서로 다른 통로로 나뉘는 방향을 하나의 중앙 방향으로 압축할 수 있다.

반면 d_free의 큰 골짜기로 열린 방향을 구분한 방식은 13장면 개선/11장면 악화였다. raw gap 평균은 기존 **{a['radial_R']['raw_gap_mean']:.2f} → {a['valley_noR']['raw_gap_mean']:.2f}개**였다. 수치 차이는 작고 대부분 장면의 최종 결과는 비슷했다.

따라서 거리 값을 버리는 것이 아니라 **거리를 방향별 깊이 profile로 사용**하는 것을 제안한다. 0.5/1.0/1.5 m별로 같은 공간을 후보로 복제하지 않고, 깊이가 뚜렷하게 낮아지는 방향을 경계로 둔다. 현재 실험값은 최소 접근 깊이 0.5 m, peak prominence 0.25 m, 방향 간격 20°, 최소 구간 폭 6°다. 이 숫자들이 최적이라는 근거는 없다.

## 3. gap 중앙이나 가장 깊은 방향 하나만 고정하지 않는 편이 낫다

1차 골짜기 방식은 각 gap의 가장 깊은 방향을 대표로 사용했다. `135606` bag **179.100초**에서는 43°가 대표가 되어, 기준 22° 후보보다 경로 비용이 커졌다.

같은 gap 안에서 **가장 깊은 방향, 각도 중앙, 기록 경로 방향을 gap 안으로 투영한 방향**을 비교하는 후속 실험을 했다. 각 방향에 대해 제약을 지키는 2초 seed를 만들고, 충돌/중복/도달 조건을 통과한 것 중 기존 `common_costs+static cost`가 낮은 것을 대표로 남겼다. 한 gap당 MPPI를 세 개 돌린 것이 아니라, 저렴한 seed 평가를 최대 세 번 한 뒤 MPPI는 대표 하나에 배정했다.

179.100초 장면의 실제 seed 후보:

| 방향 | 기존 seed 비용 | 목표 오차 m |
|---|---:|---:|
{option_rows}

같은 장면의 최종 MPPI 비용(두 seed 평균):

- 기존 거리별 방식: **{scene_means['b2_084']['radial_R']:.3f}**.
- 가장 깊은 방향 하나: **{scene_means['b2_084']['valley_cost']:.3f}**.
- 구간 내 방향을 비용으로 비교: **{scene_means['b2_084']['valley_direction_cost']:.3f}**.

![대표 방향 비교](refinement/gallery/b2_084.png)

먼 목표의 도달 조건 때문에 좋은 전방 방향을 버리는 문제도 있었다. `113151` bag **188.248초**에서는 기존 -17°/1.5 m seed가 목표 도달 부족으로 제외됐다. 목표 거리를 2초 접근 가능 범위로 조절하고 구간 안에서 방향을 비교하자 최종 비용 평균이 **{scene_means['b1_088']['radial_R']:.3f} → {scene_means['b1_088']['valley_direction_cost']:.3f}**로 줄었다.

![목표 거리와 방향 비교](refinement/gallery/b1_088.png)

## 4. 탐색 가치는 별도 가중치 합보다 기존 비용과 nominal 상태를 활용하는 방향을 권한다

실험에서 사용한 절차:

1. nominal을 한 번 rollout하고 정적 충돌, 2초 경로 arc 진행, 사람 예측 envelope와의 겹침을 확인한다.
2. 열린 각도 구간별로 최대 세 seed를 만들고, 실제 제약을 적용한다.
3. nominal 또는 이미 채택한 seed와 중복되는 후보를 제외한다.
4. 충돌 없고 접근 가능한 seed를 기존 MPPI 공통 비용+정적 비용으로 순위 매긴다. 기존 비용 안에 경로 오차·진행·heading·속도·입력 변화량이 있으므로 별도 R 항을 추가하지 않는다.
5. nominal이 정상적으로 진행하면 `J_seed < J_nominal - max(0.5, 0.05|J_nominal|)`인 후보만 추가한다. nominal이 막히거나 2초 arc 진행<0.15 m이거나 사람의 2σ envelope와 겹치면 이 개선 조건을 해제하고 최대 3개를 탐색한다.
6. 선택한 seed 주변에 MPPI sampling을 배정하고, 기존 MC 및 최종 공통 안전 재평가를 유지한다.

2σ envelope는 탐색 확장 신호이며 안전 확률을 보장하지 않는다. seed 비용이 높은 방향도 sampling 후 개선될 수 있으므로 5의 gate는 탐색 기회를 놓칠 수 있다. 이번 결과만으로 강한 기각 규칙의 보편성을 주장하지 않는다.

후속 실험의 실제 결과:

| 방식 | 추가 gap 평균 | 기준 대비 개선/유사/악화 장면 | 비용 차이 평균 | offline p95 ms | >200 ms |
|---|---:|---:|---:|---:|---:|
| 구간 내 방향 비용 비교 | {a['valley_direction_cost']['gap_mean']:.2f} | {p['valley_direction_cost']['better']}/{p['valley_direction_cost']['tied']}/{p['valley_direction_cost']['worse']} | {p['valley_direction_cost']['delta_mean']:+.3f} | {a['valley_direction_cost']['time_p95']:.1f} | {a['valley_direction_cost']['over_200']}/254 |
| 위 방식 + 필요성 gate | {a['valley_direction_gate']['gap_mean']:.2f} | {p['valley_direction_gate']['better']}/{p['valley_direction_gate']['tied']}/{p['valley_direction_gate']['worse']} | {p['valley_direction_gate']['delta_mean']:+.3f} | {a['valley_direction_gate']['time_p95']:.1f} | {a['valley_direction_gate']['over_200']}/254 |

gate 전후 직접 비교에서는 비용 감소/유사/증가 장면이 **{b['better']}/{b['tied']}/{b['worse']}개**였다. 후보 수는 줄었지만 모든 남는 예산을 nominal에 다시 배정했으므로 rollout 총량은 여전히 500개다. 따라서 후보 수 감소가 속도 향상으로 이어진다고 판단하면 안 된다. 이번 gate 구현은 후보를 모두 만든 뒤 거르는 구조이므로 후보 생성 시간도 줄이지 않는다.

![후속 비교](refinement/01_refinement_comparison.png)

## 5. 현재 권고의 강도와 다음 작업

**우선 개발할 설계:** 각도 구간을 깊이 골짜기로 분리하고, 목표 거리는 horizon 도달성으로 조절하며, 구간 내 대표 방향과 탐색 우선순위는 기존 MPPI 비용으로 정한다. 정상 진행/정체·위험 상태에 따라 추가 탐색을 다르게 허용하는 gate는 유망하지만, 놓치는 후보와 시간 효과를 별도로 검증해야 한다.

**아직 운영 대체를 권하지 않는 이유:** 후속 방식도 9~10개 장면에서는 기준보다 비용이 나빠졌고 대부분 장면의 차이는 작았다. 고정 5방향 참고 기준과 구간 내 방향 비용 비교를 직접 비교하면 새 방식의 개선/유사/악화는 {fixed['better']}/{fixed['tied']}/{fixed['worse']}개였고, 비용 차이 중앙값은 {fixed['delta_median']:+.3f}, 평균은 {fixed['delta_mean']:+.3f}였다. 일부 큰 개선과 다수 작은 악화가 섞여 있다. 초기 warm start 조건이 달라 완전한 우열 비교도 아니다. 모든 방식의 전체 안전 이동 선택은 대체로 동일했다. 폐루프 탈출/도달 성능이 개선됐다는 증거는 없다.

**5 Hz:** 대표 방향 비용 비교 방식도 offline 측정에서 254회 중 6회, gate 방식은 10회가 200 ms를 넘었다. ROS/full-stack 부하는 포함하지 않았다. 현재 Python prototype이 안정적으로 5 Hz를 만족한다고 말할 수 없다. 동일 조건에서 초기화/입력 cache와 persistent worker를 실제 노드 구조로 옮긴 뒤 전체 callback과 발행 간격을 다시 측정해야 한다.

새 bag에서 독립 검증할 항목은 열린 공간, 좁은 문, 양쪽 우회, nominal 정체, 보행자 횡단, 후진이 필요한 장면이다. gap ID의 시간 연결과 제어주기 수준의 후보 전환 안정성도 아직 구현·평가하지 않았다.

## 상세 기록

- [1차 전체 프로토콜과 결과](README.md)
- [1차 전체 비교 PNG](01_method_comparison.png)
- [후속 비교 PNG](refinement/01_refinement_comparison.png)
- `refinement/protocol.json`: 후속 실험의 사후 설계 여부와 고정 조건.
- `refinement/trials.jsonl`, `refinement/scene_audit.jsonl`, `refinement/summary.json`: 추가 508회 전체 결과.
'''
    (base/'FINDINGS.md').write_text(report,encoding='utf-8')


if __name__=='__main__':main()

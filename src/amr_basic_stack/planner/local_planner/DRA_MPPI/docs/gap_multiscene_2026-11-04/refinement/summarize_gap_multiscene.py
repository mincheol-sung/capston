#!/usr/bin/env python3
"""Summarize the paired recorded-scene experiment and render actual results."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle

METHODS = ['fixed5_warm','nominal','radial_R','radial_noR','radial_adaptive_noR',
           'connected_noR','valley_noR','valley_cost','valley_cost_gate']
LABELS = {
    'fixed5_warm':'고정 5방향·공통 warm start',
    'nominal':'nominal만 500',
    'radial_R':'거리별 gap + 기존 점수',
    'radial_noR':'거리별 gap + R 제거',
    'radial_adaptive_noR':'거리별 방향 + 목표거리 조절',
    'connected_noR':'연결 각도 구간 + R 제거',
    'valley_noR':'깊이 골짜기 분리 + R 제거',
    'valley_cost':'골짜기 분리 + 기존 비용 순위',
    'valley_cost_gate':'골짜기 분리 + 비용·필요성 gate',
}
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=FONT).get_name(),
                     'font.size':11,'axes.unicode_minus':False,
                     'figure.facecolor':'#f8fafc','savefig.facecolor':'#f8fafc',
                     'axes.spines.top':False,'axes.spines.right':False})
INK, BLUE, TEAL, ORANGE, MUTED = '#16334e','#2463d3','#078778','#d26a36','#586b7d'


def load_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def reduce_rows(rows):
    good = [r for r in rows if 'offline_total_ms' in r]
    times = [r['offline_total_ms'] for r in good]
    return dict(trials=len(rows), completed=len(good),
                safe=sum(r['safe'] for r in rows), moving=sum(r['moving'] for r in rows),
                exceptions=sum(r['status']=='exception' for r in rows),
                gap_mean=float(np.mean([r['selected_gaps'] for r in rows])),
                raw_gap_mean=float(np.mean([r['raw_gaps'] for r in rows])),
                rear_seed_fraction=sum(r['rearward_seeds'] for r in rows)/max(1,sum(r['selected_gaps'] for r in rows)),
                finite_mean=float(np.mean([r['finite_fraction'] for r in good])) if good else None,
                progress_mean=float(np.mean([r['progress_m'] for r in good if r['moving']])) if any(r['moving'] for r in good) else None,
                time_median=float(np.median(times)) if times else None,
                time_p95=float(np.percentile(times,95)) if times else None,
                time_max=float(np.max(times)) if times else None,
                over_200=sum(t>200 for t in times),
                reach_rejected_mean=float(np.mean([r['reach_rejected'] for r in rows])))


def paired(rows, reference='radial_R'):
    lookup={(r['scene'],r['seed'],r['method']):r for r in rows}
    output={}
    for method in METHODS:
        delta_by_scene=defaultdict(list)
        progress_by_scene=defaultdict(list)
        recover,lost=0,0
        for r in rows:
            if r['method']!=method:
                continue
            ref=lookup.get((r['scene'],r['seed'],reference))
            if ref is None:
                continue
            recover+=bool(r['moving'] and not ref['moving'])
            lost+=bool(ref['moving'] and not r['moving'])
            if not (r['moving'] and ref['moving'] and r.get('cost') is not None and ref.get('cost') is not None):
                continue
            delta_by_scene[r['scene']].append(r['cost']-ref['cost'])
            progress_by_scene[r['scene']].append(r['progress_m']-ref['progress_m'])
        deltas=np.array([np.mean(v) for v in delta_by_scene.values()])
        output[method]=dict(paired_scenes=len(deltas),
                            better=int(np.sum(deltas<-.1)),worse=int(np.sum(deltas>.1)),
                            tied=int(np.sum(np.abs(deltas)<=.1)),
                            delta_median=float(np.median(deltas)) if len(deltas) else None,
                            delta_mean=float(np.mean(deltas)) if len(deltas) else None,
                            progress_delta_mean=float(np.mean([np.mean(v) for v in progress_by_scene.values()])) if progress_by_scene else None,
                            recovery_trials=recover,lost_moving_trials=lost,
                            scene_deltas={k:float(np.mean(v)) for k,v in delta_by_scene.items()})
    return output


def membership(audit, a, b):
    values=[]
    for s in audit:
        if not s['accepted']:
            continue
        sets=[]
        for method in (a,b):
            sets.append({g['id'] for g in s['proposals'][method] if g['reason']=='selected'})
        values.append(dict(scene=s['scene'],changed=sets[0]!=sets[1],
                           left=sorted(sets[0]),right=sorted(sets[1]),split=s['split']))
    return values


def overview(out, summary, comparisons, scene_count, split='all'):
    fig, ax=plt.subplots(figsize=(18,9),dpi=150);ax.axis('off')
    fig.text(.04,.93,'여러 실제 장면 · gap 정의와 가치 판단 비교',fontsize=24,color=INK,weight='bold')
    fig.text(.04,.88,f'{scene_count}개 입력 유효 장면 × 제어 noise seed 2개 × 9방식 / 총 rollout 500 고정',fontsize=13,color=MUTED)
    cells=[]
    for method in METHODS:
        s=summary[method];p=comparisons[method]
        delta=p['delta_median']
        cells.append([LABELS[method],f'{s["gap_mean"]:.2f}',f'{s["moving"]}/{s["trials"]}',
                      f'{100*s["finite_mean"]:.1f}%',f'{delta:+.2f}' if delta is not None else '—',
                      f'{s["time_p95"]:.1f}',f'{s["over_200"]}/{s["completed"]}'])
    table=ax.table(cellText=cells,colLabels=['방식','추가 gap 평균','안전 이동 선택','유한 rollout','비용 차이 중앙값¹','offline p95 ms','>200 ms'],
                   colWidths=[.29,.11,.12,.10,.15,.12,.11],cellLoc='center',bbox=[0,.15,1,.73])
    table.auto_set_font_size(False);table.set_fontsize(11)
    for (r,c),cell in table.get_celld().items():
        cell.set_edgecolor('#d7e1ec');cell.set_facecolor('#e8eff8' if r==0 else 'white')
        if c==0 and r:
            cell.set_text_props(ha='left',color=INK)
    fig.text(.04,.10,'¹ 기존 거리별 gap+점수 대비, 양쪽 모두 안전 이동 후보를 선택한 장면만 비교. 장면별 두 seed 비용 차이를 평균한 뒤 중앙값.',fontsize=10,color=MUTED)
    fig.text(.04,.055,'시간은 입력 정렬·후보 생성·solver 초기화/종료를 포함한 offline 합계. ROS/full-stack 5 Hz 검증 및 실제 주행 성공률은 아님.',fontsize=10,color=MUTED)
    fig.savefig(out/'01_method_comparison.png',dpi=150);plt.close(fig)


def paired_plot(out, comparisons, title, filename):
    selected=[m for m in METHODS if m!='radial_R']
    fig,ax=plt.subplots(figsize=(14,8),dpi=150)
    y=np.arange(len(selected))
    better=np.array([comparisons[m]['better'] for m in selected])
    tied=np.array([comparisons[m]['tied'] for m in selected])
    worse=np.array([comparisons[m]['worse'] for m in selected])
    ax.barh(y,better,color=TEAL,label='비용 감소 > 0.1')
    ax.barh(y,tied,left=better,color='#d2dde8',label='차이 ≤ 0.1')
    ax.barh(y,worse,left=better+tied,color=ORANGE,label='비용 증가 > 0.1')
    for i,m in enumerate(selected):
        ax.text(better[i]+tied[i]+worse[i]+1,i,
                f'{better[i]} / {tied[i]} / {worse[i]}',va='center',fontsize=10)
    ax.set_yticks(y,[LABELS[m] for m in selected]);ax.invert_yaxis()
    ax.set_xlabel('장면 수 (각 장면의 두 seed 비용 차이 평균)')
    ax.set_title(title,loc='left',fontsize=19,pad=22,color=INK)
    ax.legend(loc='lower right');ax.grid(axis='x',alpha=.2)
    ax.set_xlim(0,max(better+tied+worse)*1.22+1)
    fig.text(.30,.035,'기준: 거리별 gap + 기존 점수. 양쪽 모두 안전 이동 선택인 장면만 포함; 실패 전환은 별도 표에 기록.',fontsize=9,color=MUTED)
    fig.subplots_adjust(left=.30,bottom=.14,top=.88)
    fig.savefig(out/filename,dpi=150);plt.close(fig)


def draw_scene(out, scene, trial_lookup, filename, reason, methods=None):
    data=np.load(out/'scenes'/f'{scene["scene"]}.npz')
    methods=methods or ['radial_R','valley_noR','valley_cost_gate']
    fig,axes=plt.subplots(1,3,figsize=(18,7.3),dpi=150)
    fig.suptitle(f'{reason} · {scene["bag"]} +{scene["offset"]:.2f} s',x=.04,ha='left',fontsize=19,color=INK)
    for ax,method in zip(axes,methods):
        category=np.zeros_like(data['grid']);category[data['grid']<0]=1
        category[data['map_fill'] & (data['grid']==0)]=2;category[data['grid']>=65]=3
        ax.imshow(category,origin='lower',extent=(-4,4,-4,4),cmap=ListedColormap(['white','#dbe3eb','#e6edfc','#294259']),vmin=0,vmax=3)
        for i,col in enumerate([ORANGE,TEAL]):
            pts=data[f'scan_{i}_hits'];ax.scatter(pts[:,0],pts[:,1],s=2,color=col,alpha=.6)
        path=data['full_path'];ax.plot(path[:,0],path[:,1],'--',color='#6cabc8',lw=1.4)
        nominal=data['nominal_path'];ax.plot(nominal[:,0],nominal[:,1],color=BLUE,lw=2,label='기록 nominal')
        for i,seed in enumerate(data[method+'_seeds']):
            xy=np.vstack((np.zeros(2),seed[:,:2]));ax.plot(xy[:,0],xy[:,1],color=TEAL,lw=1.8,alpha=.7,label='추가 gap seed' if i==0 else None)
        candidates=scene['proposals'][method]
        for g in candidates:
            target=g['target'];chosen=g['reason']=='selected'
            ax.scatter(*target,marker='x',color=TEAL if chosen else '#b3bec9',s=45 if chosen else 20)
        row=trial_lookup[(scene['scene'],7,method)]
        if method+'_trajectory' in data:
            tr=data[method+'_trajectory'];xy=np.vstack((np.zeros(2),tr[:,:2]))
            ax.plot(xy[:,0],xy[:,1],color=INK,lw=3,label='최종 선택')
        ax.add_patch(Circle((0,0),.2,color=INK,zorder=10))
        ax.annotate('',(.4,0),(0,0),arrowprops=dict(arrowstyle='-|>',color=INK,lw=2),zorder=11)
        ax.set(xlim=(-2.3,2.3),ylim=(-2.3,2.3),aspect='equal',xlabel='전방 x [m]',ylabel='좌측 y [m]')
        ax.grid(alpha=.15)
        cost=row.get('cost');cost_text=f'{cost:.2f}' if cost is not None else 'invalid'
        selected=sum(g['reason']=='selected' for g in candidates)
        ax.set_title(f'{LABELS[method]}\n후보 {len(candidates)} → 추가 {selected} / 최종 비용 {cost_text}',fontsize=12,color=INK,pad=13)
    axes[-1].legend(loc='lower right',fontsize=8)
    fig.text(.04,.07,'흰색: scan free / 연파랑: 기록 지도 보완 / 회색: unknown / x: gap 목표점. 동일 실제 장면에서 계산한 결과이며 실제 실행 궤적은 아님.',fontsize=10,color=MUTED)
    fig.text(.04,.03,'설명용 장면은 전체 결과의 차이가 드러나는 사례로 선택. 전체 집계에는 모든 입력 유효 장면이 포함됨.',fontsize=10,color=MUTED)
    fig.subplots_adjust(left=.05,right=.98,top=.78,bottom=.17,wspace=.22)
    fig.savefig(out/filename,dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    args=parser.parse_args();out=args.output
    rows=load_lines(out/'trials.jsonl');audit=load_lines(out/'scene_audit.jsonl')
    accepted=[s for s in audit if s['accepted']]
    assert len(rows)==len(accepted)*9*2
    assert len({(r['scene'],r['seed'],r['method']) for r in rows})==len(rows)
    result={}
    for split in ['all','design','holdout']:
        subset=[r for r in rows if split=='all' or r['split']==split]
        result[split]={'methods':{m:reduce_rows([r for r in subset if r['method']==m]) for m in METHODS},
                       'paired_to_radial_R':paired(subset),
                       'paired_to_nominal':paired(subset,'nominal')}
    result['by_bag']={bag:{m:reduce_rows([r for r in rows if r['bag']==bag and r['method']==m]) for m in METHODS}
                      for bag in sorted({r['bag'] for r in rows})}
    result['by_escape']={str(escape):dict(methods={m:reduce_rows([r for r in rows if r['escape']==escape and r['method']==m]) for m in METHODS},
                                         paired=paired([r for r in rows if r['escape']==escape])) for escape in [False,True]}
    changed=membership(audit,'radial_R','radial_noR')
    result['R_ablation']=dict(total=len(changed), changed=sum(x['changed'] for x in changed),
                              changed_design=sum(x['changed'] and x['split']=='design' for x in changed),
                              changed_holdout=sum(x['changed'] and x['split']=='holdout' for x in changed),details=changed)
    result['audit']=dict(total_snapshots=len(audit),accepted=len(accepted),excluded=len(audit)-len(accepted),
                         exclusions=dict(Counter(s['reason'] for s in audit if not s['accepted'])),
                         accepted_by_bag=dict(Counter(s['bag'] for s in accepted)),
                         nominal_static_invalid=sum(s['nominal_collision'] for s in accepted),
                         escape=sum(s['escape'] for s in accepted),
                         with_predictions=sum(s['prediction_count']>0 for s in accepted),
                         maximum_prediction_count=max(s['prediction_count'] for s in accepted))
    groups=Counter()
    for s in audit:
        if s['accepted']:continue
        reason=s['reason']
        if 'matmul' in reason:
            verification=json.loads((out/'exclusion_verification.json').read_text())
            assert s['scene'] in {v['scene'] for v in verification['details']}
            category='기록 global path 비어 있음 (원본 재확인)'
        elif 'terminal_path' in reason or 'No remaining recorded global path' in reason:
            category='남은 경로 없음 또는 terminal 구간'
        elif 'scan_pair_skew' in reason:category='앞·뒤 scan 시각 차이 > 50 ms'
        elif 'stale_reference' in reason:category='기록 reference 오래됨'
        elif 'traversability grid is stale' in reason:category='기록 지도 오래됨'
        elif 'PredictionChainUnavailable' in reason:category='예측 입력 오래됨 또는 유효 person 예측 소실'
        else:category=reason
        groups[category]+=1
    result['audit']['exclusion_groups']=dict(groups)
    # An unchanged selected set in the isolated R ablation must give identical
    # inputs and results, not differences from rank-dependent random streams.
    lookup={(r['scene'],r['seed'],r['method']):r for r in rows}
    unchanged_checks=0
    for item in changed:
        if item['changed']:
            continue
        for seed in [7,23]:
            a=lookup[item['scene'],seed,'radial_R'];b=lookup[item['scene'],seed,'radial_noR']
            if a.get('cost') is not None and b.get('cost') is not None:
                assert np.isclose(a['cost'],b['cost'],rtol=1e-10,atol=1e-10),(item['scene'],a['cost'],b['cost'])
                unchanged_checks+=1
    result['validation']=dict(unchanged_R_set_identical_cost_trials=unchanged_checks,
                              complete_paired_trial_matrix=True)
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    columns=['scene','bag','split','offset','method','seed','status','moving','cost','progress_m','finite_fraction',
             'raw_gaps','selected_gaps','rearward_seeds','prepare_ms','proposal_ms','setup_ms','compute_ms','offline_total_ms']
    with (out/'trials.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    overview(out,result['all']['methods'],result['all']['paired_to_radial_R'],len(accepted))
    paired_plot(out,result['all']['paired_to_radial_R'],'전체 장면: 같은 장면의 비용 변화','02_paired_cost_all.png')
    paired_plot(out,result['holdout']['paired_to_radial_R'],'추가 두 bag: 같은 장면의 비용 변화','03_paired_cost_holdout.png')
    scenes={s['scene']:s for s in accepted}
    deltas=result['holdout']['paired_to_radial_R']['valley_cost_gate']['scene_deltas']
    picks=[]
    if deltas:
        picks.extend([(min(deltas,key=deltas.get),'비용 감소가 큰 장면'),
                      (max(deltas,key=deltas.get),'비용 증가가 큰 장면')])
    R_changes=result['all']['paired_to_radial_R']['radial_noR']['scene_deltas']
    if R_changes:
        picks.append((max(R_changes,key=R_changes.get),'R 제거 시 필요한 후보가 기각된 장면'))
    obstructed=[s for s in accepted if s['nominal_collision']]
    if obstructed:
        picks.append((obstructed[0]['scene'],'기록 nominal이 정적 장애물과 겹치는 장면'))
    unique=[]
    for scene,reason in picks:
        if scene not in [x[0] for x in unique]:unique.append((scene,reason))
    for i,(scene,reason) in enumerate(unique):
        methods=['radial_R','radial_noR','valley_cost_gate'] if reason.startswith('R 제거') else None
        draw_scene(out,scenes[scene],lookup,f'{4+i:02d}_scene_{scene}.png',reason,methods)
    write_report(out,result,rows,audit,unique)
    (out/'report_source_sha256.json').write_text(json.dumps({str(Path(__file__).name):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['audit','R_ablation','validation']},ensure_ascii=False,indent=2)[:8000])
    print('SUMMARY')
    for m in METHODS:
        print(m,json.dumps(result['all']['methods'][m]),json.dumps({k:v for k,v in result['all']['paired_to_radial_R'][m].items() if k!='scene_deltas'}))


def write_report(out,result,rows,audit,picks):
    all_stats=result['all']['methods'];paired_stats=result['all']['paired_to_radial_R']
    a=result['audit'];r=result['R_ablation']
    table='\n'.join(f'| {LABELS[m]} | {all_stats[m]["gap_mean"]:.2f} | {all_stats[m]["moving"]}/{all_stats[m]["trials"]} | {100*all_stats[m]["finite_mean"]:.1f}% | {paired_stats[m]["delta_median"]:+.3f} | {paired_stats[m]["better"]}/{paired_stats[m]["tied"]}/{paired_stats[m]["worse"]} | {all_stats[m]["time_p95"]:.1f} | {all_stats[m]["over_200"]} |' for m in METHODS)
    exclusions='\n'.join(f'- {reason}: {count}장면' for reason,count in a['exclusion_groups'].items())
    perbag='\n'.join(f'- `{bag}`: {count}장면' for bag,count in a['accepted_by_bag'].items())
    scenes='\n'.join(f'![{reason}]({4+i:02d}_scene_{scene}.png)\n\n{scene}: {reason}.\n' for i,(scene,reason) in enumerate(picks))
    separate=''
    for split in ['design','holdout']:
        separate+=f'\n### {split}\n\n| 방법 | 비교 장면 | 비용 감소/유사/증가 | 비용 차이 중앙값 | 안전 이동 회복/손실 trial |\n|---|---:|---:|---:|---:|\n'
        for m in METHODS:
            p=result[split]['paired_to_radial_R'][m]
            separate+=f'| {LABELS[m]} | {p["paired_scenes"]} | {p["better"]}/{p["tied"]}/{p["worse"]} | {p["delta_median"]:+.3f} | {p["recovery_trials"]}/{p["lost_moving_trials"]} |\n'
    report=f'''# 실제 여러 장면에서의 gap 정의·탐색 가치 비교

핵심 결론과 후속 대표 방향 비교는 [실험을 바탕으로 한 설계 판단](FINDINGS.md)을 먼저 읽으면 된다.

2026-11-04. **실측 LiDAR·지도·reference·person 예측을 입력으로 사용한 단일 snapshot 최적화 비교**다. 원래 기록의 로봇 pose는 각 장면에서 동일하다. 새 제어를 실제 실행하거나 새로운 궤적을 따라 LiDAR를 다시 관측한 폐루프 실험이 아니다.

## 입력과 실험 범위

원본 세 bag에서 +5초 이후 약 2초 간격으로 후보 snapshot **{a['total_snapshots']}개**를 읽었다. 결과를 보고 장면을 골라 집계하지 않았다. 입력 검사 통과 **{a['accepted']}개**, 제외 **{a['excluded']}개**다.

{perbag}

첫 bag `135325`는 앞선 설계 예제와 같은 기록이다. 나머지 `113151`, `135606`은 이번에 처음 같은 설정을 적용한 추가 두 bag이다. 모든 설정을 `protocol.json`에 먼저 고정했고, 추가 bag 결과를 보고 파라미터를 바꾸지 않았다. 세 bag이 같은 날짜·환경의 연속된 데이터일 수 있으므로 서로 독립적인 환경 일반화 증거는 아니다.

각 장면에 제어 noise seed 7/23, 9방식을 모두 적용했다. 총 **{len(rows)}회**이며 각 회는 K=500, horizon=2초, 기존 cost 및 8배 MC 설정을 유지했다. 이 반복들은 독립적인 주행 성공률 표본이 아니다.

제외 내역:

{exclusions}

통과 장면 중 기록 nominal의 정적 충돌이 {a['nominal_static_invalid']}개, escape trigger가 {a['escape']}개, person 예측이 있는 장면이 {a['with_predictions']}개다. 최대 예측 객체 수는 {a['maximum_prediction_count']}개다. 제외된 장면은 실제 운용에서 안전 입력 문제에 따른 hold 또는 terminal 처리가 필요하며, 비교 성공 장면으로 세지 않았다.

LiDAR 시간 정렬·scan free-space·기록 지도 보완은 앞선 예제와 같다. 그림의 연파란 부분은 scan 직접 관측이 아니라 기록 지도에서 보완한 free space다. 동일한 입력을 모든 방식에 제공했다. snapshot 동안 scan hit를 정적으로 두므로 움직이는 사람의 현재 위치가 정적/동적 비용에 함께 들어갈 수 있다.

## 비교한 정의

1. **기존 거리별 gap:** d_free가 0.5/1.0/1.5 m 이상인 각도 구간을 각각 만든다. 12° 이내 중앙 방향의 중복을 합친다.
2. **거리별 방향 + 적응 목표 거리:** 1의 방향 후보는 유지하되, 목표를 실제 2초 제약 rollout으로 접근할 수 있는 거리로 당긴다. 거리 분할과 도달성 기각의 영향을 분리한다.
3. **연결 각도 구간:** d_free≥0.5 m인 연속 방향을 하나의 gap으로 만든다. 같은 공간을 여러 깊이별 후보로 복제하지 않는다. 넓은 구간의 중앙을 쓴다.
4. **깊이 골짜기로 분리한 각도 구간:** 3의 구간 내부에서도 d_free의 큰 골짜기로 나뉘는 열린 방향들을 분리한다. 각도 profile을 2° sigma로 평활화하고 prominence≥0.25 m, peak 간격≥20°로 peak를 찾는다. 대표 방향은 각 구간의 깊은 peak다. 평활화는 경계/대표 검출에만 쓰며 실제 clearance와 충돌 검사는 원래 데이터로 한다.

모든 구간 최소 각도 폭은 6°다. '거리로 분할하지 않는다'는 거리를 버린다는 뜻이 아니다. 깊이는 열린 방향의 구분과 목표 도달 거리를 정하는 데 필요하며, 0.5/1.0/1.5 m별로 후보를 반복 생성하는 것을 없앴다는 뜻이다.

적응 목표 거리: 먼저 가능한 깊이에서 0.10 m를 빼고, 그 목표에 대한 2초 제약 rollout의 방향 성분 도달 거리를 계산한다. 목표는 그 도달 거리의 90%로 줄이며 가용 깊이를 넘지 않는다. 목표 하한 0.25 m도 가용 깊이를 넘을 수 없다.

## 탐색 가치 비교

- **기존 점수:** V=0.40P+0.25C+0.20D+0.15R, V≥0.25. 정의는 앞선 예제와 동일하다.
- **R 제거:** 다른 가중치와 threshold를 고정하고 0.15R 항만 제거했다. 따라서 점수 순위뿐 아니라 threshold 통과 여부도 달라질 수 있다. 단순히 비중을 다른 항목에 재분배한 실험은 아니다.
- **기존 비용 순위:** seed의 기존 `common_costs + static cost`가 낮은 순서로 최대 3개를 채택한다. 별도의 P/C/D/R 가중치와 V threshold는 없다. 경로 추종/진행 비용은 기존 MPPI 비용 안에 남는다. 즉 별도 경로 근접성 R을 없애도 경로 관련 정보 자체를 없앤 것은 아니다.
- **필요성 gate 추가:** nominal이 건강하면 seed가 nominal 비용보다 `max(0.5, 5% × |nominal cost|)` 이상 낮을 때만 추가한다. nominal이 정적 충돌, 2초 arc 진행<0.15 m, 또는 사람의 2σ envelope와 겹치면 escape로 보고 이 개선 조건을 해제한다. 2σ envelope는 탐색을 넓히는 trigger이며 확률 안전판정이 아니다.

공통으로 nominal과 seed의 RMS<0.22 m 및 후반 차이<0.30 m이면 중복 제외한다. nominal이 정적 충돌하면 이 제거를 해제한다. seed 충돌 및 목표 오차>0.35 m를 제외한다. 새 seed끼리 RMS<0.25 m이면 중복 제외하고 최대 3개를 남긴다.

추가 gap당 100개, nominal은 `500-100×gap수`개다. nominal만 남으면 500개를 받는다. 선택된 후보는 안정적인 ID 순서로 solver에 전달하여 점수 순서만 바뀐 경우 random stream이 바뀌지 않도록 했다. R 제거 전후 같은 후보 집합이면 비용이 동일한지 **{result['validation']['unchanged_R_set_identical_cost_trials']}개 trial에서 확인**했다.

고정 5방향 비교는 각 guide의 warm start를 같은 기록 reference로 초기화한 뒤 기존 0.25 anchor blend를 사용한다. 당시 독립 guide별 내부 상태를 복원한 비교가 아니므로 gap 정의의 직접적인 우열 근거보다는 참고 기준이다.

## 전체 결과

![방법별 결과](01_method_comparison.png)

| 방식 | 추가 gap 평균 | 안전 이동 선택 | 유한 rollout | 비용 차이 중앙값 | 비용 감소/유사/증가 장면 | offline p95 ms | >200 ms 회수 |
|---|---:|---:|---:|---:|---:|---:|---:|
{table}

비용 차이 기준은 기존 거리별 gap+점수다. 양쪽 모두 안전 이동 후보를 선택한 trial에서 장면별 두 seed 차이를 평균하고, 장면들을 집계했다. ±0.1 이하는 유사로 세었다. 안전 이동 여부가 달라지는 trial은 비용 차이에서 제외하고 회복/손실로 별도 기록했다. 유한 rollout 비율은 정적 invalid 제거율과 관련된 진단이며, 그 자체로 실제 충돌률이나 최종 후보의 동적 안전 비율이 아니다.

![전체 paired 결과](02_paired_cost_all.png)

{separate}

![추가 두 bag 결과](03_paired_cost_holdout.png)

## R 항 제거로 실제 달라진 것

추가 후보 집합이 달라진 장면은 **{r['changed']}/{r['total']}개**다. 앞선 bag {r['changed_design']}개, 추가 두 bag {r['changed_holdout']}개다. 변화 내역은 `summary.json`의 `R_ablation.details`, 후보별 수치/기각 이유는 `scene_audit.jsonl`에 모두 기록했다.

R 제거 여부와 gap 정의는 별개의 축이다. 거리별 중복을 없애도 서로 다른 방향의 후보는 경로에서 멀어질 수 있다. 별도 R의 필요성을 판단하려면 경로 정보를 다른 비용/진행 기준이 이미 충분히 제공하는지를 함께 봐야 한다.

## 실제 장면 그림

아래 사례는 차이를 설명하기 위해 전체 결과에서 선택했다. 비용 차이가 큰 사례는 두 seed 평균으로 골랐고 그림은 seed 7의 실제 결과를 보여준다. 전체 통계의 장면 선택에는 영향을 주지 않았다.

{scenes}

## 시간과 해석 범위

시간은 단일 snapshot의 입력 정렬·필드 생성·nominal/prediction 준비, 해당 방식의 seed pool/선정, controller 초기화, solver와 worker 종료 wall을 합쳤다. 공유 입력을 한 번 계산해 각 방식에 동일하게 더했고, 관련 없는 다른 방식의 pool 계산은 포함하지 않았다. 각 방식 실행 순서는 장면마다 회전했다. 두 seed는 제어 sampling seed이며 MC 설정은 고정했다.

cache/코어 경쟁·ROS 입력 callback·TF 서비스·publish·전체 인지/MPC 부하는 재현하지 않았다. **offline p95가 200 ms 미만이어도 실제 5 Hz 통과가 아니다.** 원본 운영 timeout 0.18초 대신 계산 결과를 수집하기 위해 10초 timeout을 썼다. 느린 실행을 통계에서 숨기지 않았다. 표의 >200 ms는 offline 측정 구간 초과이며 실제 deadline miss 수가 아니다.

장면 간 로봇 pose/지도/예측은 원래 기록을 사용했다. 새 후보로 주행했을 때의 다음 관측이나 사람 반응을 생성하지 않았다. 그러므로 도착률, 교착 탈출률, 실제 충돌률, gap ID 안정성 및 5 Hz 실시간 동작은 아직 미검증이다. 권고는 이 범위에서의 후보 효율/목적함수 결과에 한정해야 한다.

## 재현과 파일

`source_snapshot/`에 이번 실험에 사용한 planner·설정·실험 코드를 고정해 보관했다. 현재 작업 중인 소스가 바뀌어도 이 복사본으로 같은 비교를 실행할 수 있다. 원본 bag은 수정하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/python3 \\
  planner/local_planner/DRA_MPPI/docs/gap_multiscene_2026-11-04/source_snapshot/tools/experiment_gap_multiscene.py \\
  --output /tmp/gap_multiscene_reproduction
/usr/bin/python3 planner/local_planner/DRA_MPPI/tools/summarize_gap_multiscene.py /tmp/gap_multiscene_reproduction
```

- `protocol.json`: 결과 확인 전에 고정한 조건과 비교 범위.
- `source_hashes.json`, `configuration.json`: 소스/설정 provenance.
- `scene_audit.jsonl`: 모든 snapshot의 포함/제외 이유와 모든 후보 수치.
- `trials.jsonl`, `trials.csv`: 전체 solver 결과와 시간.
- `summary.json`: 전체·bag별·추가 bag·nominal 상태별 결과와 paired 비교.
- `scenes/*.npz`: 장면별 실제 grid/scan point와 seed·최종 궤적.
'''
    (out/'README.md').write_text(report,encoding='utf-8')


if __name__=='__main__':main()

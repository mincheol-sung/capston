"""Render real-scene evidence and audited counts for the diversity experiment."""
from collections import Counter, defaultdict
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

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/gap_multiscene_2026-11-04'
OUT=BASE/'diversity_and_empty_gap'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':11,
                     'figure.facecolor':'#f8fafc','savefig.facecolor':'#f8fafc',
                     'axes.spines.top':False,'axes.spines.right':False})
INK='#17344d';BLUE='#3076ba';GREEN='#008976';PURPLE='#9852b2';RED='#c35b55'


def load(path):return [json.loads(x) for x in path.read_text().splitlines()]


def baseplot(ax,p):
    g=p['grid'];d=np.zeros_like(g);d[g<0]=1;d[p['map_fill']&(g==0)]=2;d[g>=65]=3
    ax.imshow(d,extent=(-4,4,-4,4),origin='lower',vmin=0,vmax=3,
              cmap=ListedColormap(['white','#d3dce5','#e5edfa','#294158']))
    for k,col in [('scan_0_hits','#c77a36'),('scan_1_hits','#4c77a2')]:
        ax.scatter(p[k][:,0],p[k][:,1],s=2,color=col,alpha=.7)
    ax.add_patch(Circle((0,0),.2,facecolor=INK,edgecolor='white',zorder=9))
    ax.annotate('',(.4,0),(0,0),arrowprops=dict(arrowstyle='->',color=INK),zorder=10)
    ax.set(xlim=(-2.2,2.2),ylim=(-2.2,2.2),aspect='equal',xlabel='전방 x [m]',ylabel='좌측 y [m]')
    ax.grid(alpha=.12)


def pathplot(ax,path,color,label,lw=2.5,ls='-'):
    xy=np.vstack([np.zeros(2),path[:,:2]])
    ax.plot(xy[:,0],xy[:,1],color=color,label=label,lw=lw,ls=ls)


def main():
    trials=load(OUT/'trials.jsonl');audit=load(OUT/'scene_audit.jsonl')
    old={(r['scene'],r['seed']):r for r in load(BASE/'refinement/trials.jsonl') if r['method']=='valley_direction_cost'}
    assert len(trials)==508 and len(audit)==127
    assert len({(r['scene'],r['seed'],r['method']) for r in trials})==508
    result=dict(scenes=127,new_trials=508,baseline_rear_scenes=sum(a['baseline_rear']>0 for a in audit),
        eligible_rear_scenes=sum(a['baseline_rear_available']>0 for a in audit),methods={})
    for method in sorted({r['method'] for r in trials}):
        rows=[r for r in trials if r['method']==method];diff=defaultdict(list)
        for r in rows:
            prior=old[r['scene'],r['seed']]
            if r['moving'] and prior['moving'] and r['cost'] is not None and prior['cost'] is not None:
                diff[r['scene']].append(r['cost']-prior['cost'])
        deltas={k:float(np.mean(v)) for k,v in diff.items()};v=np.array(list(deltas.values()))
        result['methods'][method]=dict(better=int(sum(v<-.1)),similar=int(sum(abs(v)<=.1)),worse=int(sum(v>.1)),
            paired_scenes=len(v),mean_delta=float(v.mean()),median_delta=float(np.median(v)),scene_deltas=deltas,
            safe=sum(r['safe'] for r in rows),moving=sum(r['moving'] for r in rows),
            guide_mean=float(np.mean([r['selected_guides'] for r in rows])),
            rear_scenes=sum(r['rear_seeds']>0 for r in rows if r['seed']==7),
            p95_ms=float(np.percentile([r['offline_total_ms'] for r in rows],95)),
            over_200=sum(r['offline_total_ms']>200 for r in rows))
    empty=[]
    for a in audit:
        if a['raw_gaps']:continue
        if a['origin_adjusted_clearance']<=.2:reason='current_footprint_invalid'
        elif a['hard_raw_gaps']>0:reason='extra_margin_blocked'
        elif a['hard_max_depth']<.5:reason='short_straight_depth'
        else:reason='angular_width_below_minimum'
        r=next(r for r in trials if r['scene']==a['scene'] and r['seed']==7 and r['method']=='all_options_hard_fallback')
        empty.append(dict(scene=a['scene'],bag=a['bag'],offset=a['offset'],reason=reason,
            origin_clearance=a['origin_adjusted_clearance'],strict_max_depth=a['strict_max_depth'],
            hard_max_depth=a['hard_max_depth'],hard_open_rays=a['hard_open_rays'],hard_raw_gaps=a['hard_raw_gaps'],
            guides_after_fallback=r['selected_guides'],before_selected=old[a['scene'],7]['selected'],
            after_selected=r['selected'],after_status=r['status']))
    result['empty_gaps']=dict(count=len(empty),fraction=len(empty)/127,details=empty,
        reason_counts=dict(Counter(r['reason'] for r in empty)),
        recovered=sum(r['guides_after_fallback']>0 for r in empty),
        all_filtered_after_nonempty=sum(a['raw_gaps']>0 and not a['methods']['all_options_cost']['selected_ids'] for a in audit))
    (OUT/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')

    fig,ax=plt.subplots(figsize=(15,8),dpi=160);ax.axis('off')
    fig.text(.045,.93,'후방 후보 보존과 gap 없음 · 실제 127장면 확인',fontsize=22,color=INK,weight='bold')
    fig.text(.045,.87,'후속 508회 offline 비교 · 같은 데이터를 재사용한 탐색적 실험 · 주행 성공률 검증 아님',fontsize=12,color=INK)
    cells=[['gap당 대표 1개 (기존)','74','기준','248 / 254','8 / 127']]
    for method,label in [('all_options_cost','전체 seed에서 궤적 중복 제거'),('all_options_hard_fallback','위 방식 + gap 없음 재시도')]:
        s=result['methods'][method]
        zero=sum(not a['methods'][method]['selected_ids'] for a in audit)
        cells.append([label,str(s['rear_scenes']),f'{s["better"]} / {s["similar"]} / {s["worse"]}',f'{s["moving"]} / 254',f'{zero} / 127'])
    t=ax.table(cellText=cells,colLabels=['방식','후방 seed 보유 장면','최종 비용 개선 / 유사 / 악화¹','안전 이동 선택 trial','추가 후보 없음'],
               colWidths=[.30,.14,.25,.16,.15],cellLoc='center',bbox=[0,.39,1,.40])
    t.auto_set_font_size(False);t.set_fontsize(11)
    for (r,c),cell in t.get_celld().items():cell.set_edgecolor('#d3dfe9');cell.set_facecolor('#e7eef7' if r==0 else 'white')
    fig.text(.045,.28,'¹ 두 방식 모두 안전 이동인 124장면의 두 noise seed 평균 비용 비교. ±0.1은 유사. 후방은 seed 종점 x < −0.05 m.',fontsize=10,color=INK)
    fig.text(.045,.20,'gap 없음 8장면 = 추가 여유 때문에 차단 3 + 짧은 직진 가능 거리 2 + 열린 폭 부족 1 + 현재 몸체 충돌 판정 2',fontsize=13,color=INK)
    fig.text(.045,.13,'gap 없음 재시도: 추가 5 cm 여유만 후보 생성에서 제외. 기존 몸체 충돌 검사·미확인 차단·MC 기준은 유지.',fontsize=12,color=INK)
    fig.text(.045,.065,'후방 탐색 범위는 늘었지만 평균 성능 향상은 확인되지 않음. gap 없음은 정지 명령과 동의어가 아님.',fontsize=13,color=RED)
    fig.savefig(OUT/'01_summary.png');plt.close(fig)

    scene='b1_088';p=np.load(BASE/'scenes'/f'{scene}.npz');extra=np.load(OUT/'scenes'/f'{scene}.npz')
    prior=np.load(BASE/'refinement/scenes'/f'{scene}.npz');a=next(a for a in audit if a['scene']==scene)
    newrows=[r for r in trials if r['scene']==scene and r['method']=='all_options_cost']
    fig,axs=plt.subplots(1,2,figsize=(14,8),dpi=160)
    fig.subplots_adjust(top=.80,bottom=.20,wspace=.22)
    fig.suptitle('같은 실제 LiDAR 장면 · 후방 후보가 사라지던 원인과 수정 결과',y=.96,fontsize=21,color=INK)
    for ax in axs:baseplot(ax,p)
    oldaudit=next(x for x in load(BASE/'refinement/scene_audit.jsonl') if x['scene']==scene)
    for g in oldaudit['proposals']['valley_direction_cost']:
        if g['reason']=='selected':
            # Stored baseline arrays are in cost-priority order.
            order=sorted([g for g in oldaudit['proposals']['valley_direction_cost'] if g['reason']=='selected'],key=lambda g:g['cheap_cost'])
            index=next(i for i,x in enumerate(order) if x['id']==g['id'])
            pathplot(axs[0],prior['valley_direction_cost_seeds'][index],GREEN if g['id']=='valley_1' else PURPLE,
                     f'{np.rad2deg(g["angle"]):.0f}° seed')
    selected=sorted([g for g in a['methods']['all_options_cost']['options'] if g['reason']=='selected'],key=lambda g:g['id'])
    for i,g in enumerate(selected):
        pathplot(axs[1],extra['all_options_cost_seeds'][i],RED if g['rear_body'] else GREEN,
                 f'{np.rad2deg(g["angle"]):.0f}°'+(' 후방 유지' if g['rear_body'] else ' seed'))
    axs[0].set_title('기존: G1·G2 각각 1개 → −55°, −11°',fontsize=13)
    axs[1].set_title('수정: 전체 후보 중 다른 궤적 3개 → −160°, −55°, −11°',fontsize=12)
    for ax in axs:ax.legend(loc='lower left',fontsize=10)
    oldmean=np.mean([old[scene,s]['cost'] for s in [7,23]]);newmean=np.mean([r['cost'] for r in newrows])
    fig.text(.06,.105,f'두 seed 평균 최종 비용: 기존 {oldmean:.3f} → 수정 {newmean:.3f}. 후방은 보존됐지만 이 장면의 최종 비용은 약간 증가.',fontsize=12,color=INK)
    fig.text(.06,.055,'bag mpc_chain_20260923_113151 +188.248 s · 선은 실측 입력으로 계산한 seed이며 실행 궤적이 아님',fontsize=10,color=INK)
    fig.savefig(OUT/'02_rear_candidates.png');plt.close(fig)

    examples=[('b0_029','추가 여유 5 cm가 막은 경우'),('b0_034','직진 가능한 깊이가 짧은 경우'),
              ('b2_083','깊지만 각도 폭이 부족한 경우'),('b0_033','현재 몸체가 충돌로 판정된 경우')]
    fig,axs=plt.subplots(2,2,figsize=(14,15),dpi=160)
    fig.subplots_adjust(top=.87,bottom=.16,hspace=.55,wspace=.19)
    fig.suptitle('gap 없음은 한 가지 상황이 아니다 · 실제 기록 4장면',y=.97,fontsize=22,color=INK)
    fig.text(.07,.93,'청록선: 몸체 충돌 기준의 직진 깊이 / 붉은선: 추가 여유 5 cm를 포함한 깊이 / 미확인 공간은 계속 차단',fontsize=11,color=INK)
    for ax,(scene,title) in zip(axs.flat,examples):
        p=np.load(BASE/'scenes'/f'{scene}.npz');q=np.load(OUT/'scenes'/f'{scene}.npz');a=next(x for x in empty if x['scene']==scene)
        baseplot(ax,p)
        for depth,col,lw in [(q['hard_d_free'],GREEN,1.0),(p['d_free'],RED,1.8)]:
            # Include sparse valid rays; every positive ray is actual computed data.
            for idx in np.flatnonzero(depth>0):
                angle=p['angles'][idx];r=depth[idx]
                ax.plot([0,r*np.cos(angle)],[0,r*np.sin(angle)],color=col,lw=lw,alpha=.20)
        ax.set_title(f'{title}\n{scene} / +{a["offset"]:.3f} s',fontsize=13,pad=10)
        detail=f'몸체 기준 최대 깊이 {a["hard_max_depth"]:.2f} m / 0.5 m 이상 {a["hard_open_rays"]}방향\n재시도 gap {a["hard_raw_gaps"]}개 / 탐색 guide {a["guides_after_fallback"]}개'
        if scene=='b0_033':detail+='\n보수적 시작점 여유 0.167 m < 반경 0.20 m → hold'
        elif scene=='b0_029':detail+='\n기존 Stop → 재시도에서 후방 이동 후보 선택 (offline)'
        elif scene=='b2_083':detail+='\n깊이 0.5 m 이상은 4방향뿐 → 최소 폭 6방향 미달'
        else:detail+='\ngap 없이도 nominal MPPI가 이동 후보 선택'
        ax.text(0,-.23,detail,transform=ax.transAxes,va='top',fontsize=10.5,color=INK,linespacing=1.5)
    fig.text(.07,.018,'실측 LiDAR + 기록 지도 보완 · 같은 시작점에서 계산한 snapshot 결과 · 몸체 충돌은 센서·지도 기반 보수적 판정',fontsize=10,color=INK)
    fig.savefig(OUT/'03_empty_gap_examples.png');plt.close(fig)
    with zipfile.ZipFile(OUT/'results.zip','w',zipfile.ZIP_DEFLATED) as z:
        for f in sorted(OUT.rglob('*')):
            if f.is_file() and f.suffix in ('.png','.json','.jsonl','.npz'):z.write(f,str(f.relative_to(OUT)))
        for name in ['experiment_gap_diversity.py','summarize_gap_diversity.py']:z.write(ROOT/'tools'/name,name)
    print(json.dumps({k:v for k,v in result.items() if k!='methods'},ensure_ascii=False,indent=2))
    for name,v in result['methods'].items():print(name,{k:x for k,x in v.items() if k!='scene_deltas'})


if __name__=='__main__':main()

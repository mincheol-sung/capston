"""Offline aggregation of existing bench logs. No CAN access."""
import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('bench','/home/junseo/can_motor_identification.py')
bench=importlib.util.module_from_spec(spec); spec.loader.exec_module(bench)
root=Path.home()/'motor_identification_logs'
selected={}; all_runs=[]
for folder in sorted(root.glob('*_BENCH')):
    try:
        meta=json.loads((folder/'metadata.json').read_text())
        if meta.get('simulation') or meta.get('status')=='running': continue
        with (folder/'samples.csv').open() as stream: rows=list(csv.DictReader(stream))
    except (OSError,ValueError): continue
    all_runs.append({'folder':str(folder),'status':meta['status'],'reason':meta.get('reason',''),'rows':len(rows)})
    for mid in meta['motor_ids']:
        part=[r for r in rows if int(r['motor_id'])==mid]
        if part and (mid not in selected or len(part)>=len(selected[mid][2])):
            selected[mid]=(folder,meta,part)
report={'runs':all_runs,'motors':{},'note':'Command-torque based provisional estimates. No robot config changed; no measured current.'}
lines=['# AK40-10 실기 토크 시험 결과','',
       '모든 추정값은 출력축 단위·명령토크 추종을 가정한 후보값이다. 실제 전류는 측정하지 않았다.',
       '차체 질량·차체 yaw 관성·노면 마찰은 이번 시험으로 측정하지 않았다. 로봇 설정은 변경하지 않았다.','',
       '## 비교용 J,C 단순 모델 (T = J·dω/dt + C·ω)','',
       '| 바퀴 | 상태 | 응답/기록 | 최대 속도 rad/s | 최대 온도 °C | J kg·m² | C N·m·s/rad |',
       '|---|---|---:|---:|---:|---:|---:|']
extra=[]
for mid,(folder,meta,rows) in sorted(selected.items()):
    fits={}
    for source in ('lpf','raw'):
        with contextlib.redirect_stdout(io.StringIO()): bench.analyze(folder,source)
        fits[source]=json.loads((folder/'fit_candidates.json').read_text())
        (folder/f'fit_candidates_{source}.json').write_text(json.dumps(fits[source],indent=2))
    (folder/'fit_candidates.json').write_text(json.dumps(fits['lpf'],indent=2))
    item=fits['lpf']['motors'][str(mid)]; good=[r for r in rows if r['rx_ok']=='1']
    temps=[float(r['temperature_c']) for r in good]
    counts={name:sum(r['phase']==name for r in rows) for name in sorted({r['phase'] for r in rows})}
    simple=item.get('primary_model_JC',{}); ext=item.get('candidate',{})
    quality={'folder':str(folder),'status':meta['status'],'reason':meta.get('reason',''),
             'cleanup_errors':meta.get('cleanup_errors'), 'rows':len(rows),'responses':len(good),
             'duration_s':float(rows[-1]['tx_time_s']), 'temperature_max_c':max(temps) if temps else None,
             'phase_counts':counts,'fit_lpf':item,'fit_raw':fits['raw']['motors'][str(mid)]}
    if ext:
        quality['extended_to_simple_rmse_ratio']=item['integral_rmse_Nms']/simple['integral_rmse_Nms']
    report['motors'][str(mid)]=quality
    wheel=bench.WHEELS[mid]
    lines.append(f"| {wheel}({mid}) | {meta['status']} | {len(good)}/{len(rows)} | {item['peak_speed_rad_s']:.3f} | {max(temps):.0f} | {simple.get('wheel_inertia_kgm2',float('nan')):.6g} | {simple.get('C_Nms',float('nan')):.6g} |")
    if ext:
        extra.append(f"| ID {mid} / {wheel} | {ext['wheel_inertia_kgm2']:.6g} | {ext['tau_c_Nm']:.6g} | {ext['b_v_Nms']:.6g} | {quality['extended_to_simple_rmse_ratio']:.3f} |")
    lines.extend([])
    print('ID',mid,wheel,'status',meta['status'],'rows',len(rows),'extended',ext)
lines+=['','## 최종 피팅 모델: 관성 + 쿨롱 마찰 + 점성 마찰','',
        'T = J·dω/dt + τc·sign(ω) + bv·ω + offset. 입력 토크와 각속도에 5 Hz LPF를 적용한 뒤 적분식으로 피팅했다. 잔차비가 작을수록 단순 J,C 모델보다 잘 맞는다.','',
        '| 바퀴 | J kg·m² | τc N·m | bv N·m·s/rad | 확장/단순 잔차비 |','|---|---:|---:|---:|---:|',*extra,'',
        '## 모터 ID별 데이터와 중단 사유','', *[f"- ID {mid} / {bench.WHEELS[mid]}: [{v['folder']}]({v['folder']}) — {v['status']}, {v['duration_s']:.2f}초, {v['rows']}행. {v['reason'] or '정상 완료'}. disable 송신 오류: {v['cleanup_errors']}" for mid,v in [(int(k),v) for k,v in report['motors'].items()]], '', '## 해석','',
        '기본 피팅은 원본을 보존한 5 Hz LPF 신호를 사용한다. raw/LPF 후보를 각 폴더에 따로 저장했다.',
        '단순 모델에서 쿨롱 마찰이 C에 흡수될 수 있으므로 두 모델의 C와 bv는 같은 물리량으로 확정하면 안 된다.',
        '이번 토크 자극은 짧고 속도 보호로 일부 잘리므로 정상상태 속도 전체 범위의 마찰을 확정하지 않는다.',
        '명령토크와 실제 토크 사이 지연·스케일, 센서 잡음, 축 기준, 반복 재현성 검토가 남아 있다.',
        'aborted 시험은 일부 데이터만 사용했으며 완료 시험으로 간주하지 않는다.','', '## 시험 이력','']
for run in all_runs:
    lines.append(f"- {run['folder']}: {run['status']}, {run['rows']}행. {run['reason']}")
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes=plt.subplots(len(selected),2,figsize=(14,3*len(selected)),squeeze=False)
    for index,(mid,(_,meta,rows)) in enumerate(sorted(selected.items())):
        good=[r for r in rows if r['rx_ok']=='1']
        t=np.array([float(r['rx_time_s']) for r in good])
        for col,raw_key,filtered_key,unit in [(0,'velocity_rad_s','velocity_lpf_rad_s','rad/s'),
                                             (1,'torque_fb_nm','torque_fb_lpf_nm','Nm')]:
            ax=axes[index,col]
            ax.plot(t,[float(r[raw_key]) for r in good],color='.7',linewidth=.5,label='raw feedback')
            ax.plot(t,[float(r[filtered_key]) for r in good],linewidth=1,label='5 Hz LPF')
            if col==1:
                ax.plot(t,[float(r['torque_encoded_nm']) for r in good],color='tab:red',linewidth=.6,label='command')
            ax.set(title=f'ID {mid} / {bench.WHEELS[mid]} ({meta["status"]})',xlabel='time (s)',ylabel=unit)
            ax.grid(alpha=.25); ax.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(root/'BENCH_SIGNALS.png',dpi=160)
    plt.close(figure)
    lines+=['','## 원본과 필터 신호','', '![모터 ID별 각속도와 토크](BENCH_SIGNALS.png)','']
except ImportError:
    lines+=['','그래프 패키지가 없어 CSV와 수치 결과만 저장했다.','']

(root/'BENCH_RESULTS.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
(root/'BENCH_RESULTS_KO.md').write_text('\n'.join(lines).replace('内部','내부')+'\n')
print('REPORT',root/'BENCH_RESULTS_KO.md')

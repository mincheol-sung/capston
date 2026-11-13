"""Plot measured before/after planner latency, without changing planner data."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/gap_latency_optimization_2026-11-06'
d=json.loads((OUT/'benchmark.json').read_text())
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':12,
                     'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#f5f8fc'})
s=d['summary']['production'];labels=['개선 전','개선 후'];keys=['before','after']
fig,axes=plt.subplots(1,2,figsize=(13,6))
x=np.arange(2);bottom=np.zeros(2)
geometry=np.array([s[k]['geometry_ms']['mean']for k in keys])
proposal=np.array([s[k]['proposal_ms']['mean']for k in keys])
total=np.array([s[k]['total_ms']['mean']for k in keys])
for values,name,color in [(geometry,'LiDAR·gap 준비','#327fb3'),(proposal,'seed 준비·최적화','#bb791c'),
                          (total-geometry-proposal,'500개 MPPI·대표 평가 등','#009c90')]:
    axes[0].bar(x,values,bottom=bottom,label=name,color=color,width=.55)
    for i,v in enumerate(values):axes[0].text(i,bottom[i]+v/2,f'{v:.1f}',ha='center',va='center',color='white',fontweight='bold')
    bottom+=values
for i,v in enumerate(total):axes[0].text(i,v+2,f'{v:.1f}ms',ha='center',fontweight='bold')
axes[0].set_xticks(x,labels);axes[0].set_ylim(0,140);axes[0].set_ylabel('평균 처리 시간 [ms]')
axes[0].set_title('기본 50ms seed 예산·180ms 전체 제한 유지');axes[0].legend(loc='upper right',fontsize=10)
for j,(key,color) in enumerate(zip(keys,['#8295a6','#327fb3'])):
    values=[s[key]['total_ms']['p95'],s[key]['total_ms']['max']]
    positions=x+(j-.5)*.34;axes[1].bar(positions,values,width=.32,label=labels[j],color=color)
    for p,v in zip(positions,values):axes[1].text(p,v+2,f'{v:.1f}',ha='center',fontsize=11)
axes[1].axhline(200,color='#c34848',ls='--',lw=1,label='5Hz의 200ms 간격')
axes[1].set_xticks(x,['p95','최대']);axes[1].set_ylim(0,225);axes[1].set_ylabel('전체 처리 시간 [ms]')
axes[1].set_title('실제 bag 16장면 × 8회 / 전후 각각 128회');axes[1].legend(loc='upper right',fontsize=10)
fig.suptitle('동일 계산 구조를 유지하면서 평균 계산 시간 13.9% 단축',fontsize=20,y=1.02)
fig.text(.02,-.03,'LiDAR 격자 생성 28.3 → 6.0ms. 동일 계산량 비교에서는 128쌍 모두 샘플·비용·최종 선택이 정확히 동일.',fontsize=11)
fig.text(.02,-.075,'운영 예산에서는 더 많은 gap 최적화 완료: 168 → 208개. ROS/TF/발행·다른 노드 부하 및 최초 모듈 로딩은 제외.',fontsize=11)
fig.tight_layout();fig.savefig(OUT/'latency_comparison.png',dpi=160,bbox_inches='tight')

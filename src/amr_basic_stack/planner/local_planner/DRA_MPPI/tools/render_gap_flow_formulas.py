"""Add the complete ten-step explanation and equations to actual gap replay PNGs.

Reads saved production replay outputs; does not rerun or change the controller.
"""
from pathlib import Path
import json
import zipfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Polygon, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/gap_implementation_flow_2026-11-05'
OUT = SOURCE / 'with_formulas'
OUT.mkdir(parents=True, exist_ok=True)
R = json.loads((SOURCE / 'results.json').read_text())
A = dict(np.load(SOURCE / 'replay_arrays.npz'))
D = dict(np.load(ROOT / f'docs/gap_multiscene_2026-11-04/disparity_partition_10_scenes/data/{R["scene"]}.npz'))
COLORS = ['#c47b16', '#3383ba', '#009c91', '#7c55b5', '#d75a70', '#678225']
INK, MUTED, BG = '#20364b', '#536b7e', '#f6f8fb'
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font).get_name(),
                     'mathtext.fontset': 'dejavusans', 'axes.unicode_minus': False,
                     'font.size': 12, 'text.color': INK, 'axes.labelcolor': MUTED})
FILES, CONTENT = [], []


def page(number, title, subtitle):
    fig = plt.figure(figsize=(16, 11), facecolor=BG)
    fig.text(.045, .95, f'{number:02d}  {title}', fontsize=25, weight='bold')
    fig.text(.045, .91, subtitle, fontsize=13, color=MUTED)
    fig.text(.045, .055, f"실제 장면 {R['scene']} | {R['source']} +{R['offset']:.3f}s | 기존 재생 결과를 그대로 시각화", fontsize=10, color=MUTED)
    fig.text(.045, .028, '저장된 deskew scan·지도·nominal 재사용 · 동적 예측 미주입 · ROS 입출력/실주행/전체 5Hz 검증 아님', fontsize=10, color=MUTED)
    CONTENT.append(f'\n{number:02d}. {title}\n{subtitle}\n')
    return fig


def explain(fig, blocks, x=.51, y=.84):
    for kind, value in blocks:
        CONTENT.append(value.replace('$', '')+'\n')
        if kind == 'title':
            fig.text(x, y, value, fontsize=16, weight='bold', va='top')
            y -= .043
        elif kind == 'math':
            fig.text(x, y, value, fontsize=19, va='top')
            y -= .100 if '\\sum' in value else .070
        else:
            fig.text(x, y, value, fontsize=13, va='top', linespacing=1.6, color=INK)
            y -= .029*len(value.splitlines())+.018
    assert y >= .095, (blocks, y)


def takeaway(fig, text):
    fig.text(.055, .105, text, fontsize=12, color=INK,
             bbox=dict(facecolor='#e5edf5', edgecolor='none', pad=8))
    CONTENT.append('결과: '+text+'\n')


def save(fig, filename):
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    boxes = [(t.get_window_extent(renderer), t.get_text()) for t in fig.texts]
    for box, text in boxes:
        assert box.x0 >= 0 and box.x1 <= fig.bbox.width, (filename, 'horizontal clipping', text)
        assert box.y0 >= 0 and box.y1 <= fig.bbox.height, (filename, 'vertical clipping', text)
    for i, (box, text) in enumerate(boxes):
        for other, other_text in boxes[i+1:]:
            assert not box.expanded(.99, .97).overlaps(other.expanded(.99, .97)), (filename, 'text overlap', text, other_text)
    fig.savefig(OUT / filename, dpi=150, facecolor=BG)
    plt.close(fig)
    FILES.append(filename)


def scene(fig, raw=False, rect=(.055, .31, .40, .52), limit=2.4):
    ax = fig.add_axes(rect)
    if not raw:
        grid = D['grid']
        view = np.zeros_like(grid)
        view[grid < 0] = 1
        view[D['map_fill'] & (grid == 0)] = 2
        view[grid >= 65] = 3
        ax.imshow(view, extent=(-4, 4, -4, 4), origin='lower', vmin=0, vmax=3,
                  cmap=ListedColormap(['white', '#d8dfe7', '#e5eefb', '#34495e']), interpolation='nearest')
    for i, col in enumerate(('#df8646', '#468dc4')):
        hits = D[f'scan_{i}_hits'][D[f'scan_{i}_valid']]
        ax.scatter(hits[:, 0], hits[:, 1], s=2, color=col, alpha=.55,
                   label='전방 LiDAR' if i == 0 else '후방 LiDAR')
    ax.add_patch(Circle((0, 0), .2, fc=INK, ec='white', zorder=12))
    ax.annotate('', (.36, 0), (0, 0), arrowprops=dict(arrowstyle='->', color=INK, lw=2), zorder=13)
    ax.set(xlim=(-limit, limit), ylim=(-limit, limit), aspect='equal',
           xlabel='x [m] · 전방 + / 후방 −', ylabel='y [m] · 좌측 + / 우측 −')
    ax.grid(alpha=.13)
    return ax


def line(ax, trajectory, **kw):
    xy = np.vstack(([0, 0], trajectory[:, :2]))
    ax.plot(xy[:, 0], xy[:, 1], **kw)


def regions(ax, labels=True):
    for r in R['partition']['regions']:
        ids = np.array(r['bins'])
        a, d = D['angles'][ids], D['strict_depth'][ids]
        xy = d[:, None]*np.column_stack((np.cos(a), np.sin(a)))
        col = COLORS[r['id']-1]
        ax.add_patch(Polygon(np.vstack(([0, 0], xy)), fc=col, alpha=.16, ec='none'))
        ax.plot(xy[:, 0], xy[:, 1], color=col, lw=1.9)
        if labels:
            k = len(ids)//2
            at = max(.6, d[k]*.8)*np.array([np.cos(a[k]), np.sin(a[k])])
            ax.text(*at, f'G{r["id"]}', color=col, weight='bold', ha='center', fontsize=13,
                    bbox=dict(fc='white', ec='none', alpha=.85, pad=1))


def small_table(fig, rows, labels, rect=(.055, .16, .40, .14), fontsize=11):
    ax = fig.add_axes(rect); ax.axis('off')
    table = ax.table(cellText=rows, colLabels=labels, loc='center', cellLoc='center')
    table.auto_set_font_size(False); table.set_fontsize(fontsize); table.scale(1, 1.6)
    for (i, j), cell in table.get_celld().items():
        cell.set_edgecolor('#d4dde6')
        if i == 0: cell.set_facecolor('#e4ebf3')


C = R['diagnostics']['candidates']

# 01: input roles, online timing gates, and offline evidence are distinct.
fig = page(1, '전·후방 LiDAR를 같은 기준으로 맞춥니다', '입력의 역할 · 원본/수신 시각 검사 · odometry와 센서 TF를 이용한 보정')
ax = scene(fig, raw=True, limit=3.5); ax.legend(loc='upper right', fontsize=10)
small_table(fig, [['전방', int(D['scan_0_valid'].sum()), len(D['scan_0_ranges'])],
                  ['후방', int(D['scan_1_valid'].sum()), len(D['scan_1_ranges'])]], ['센서', '유효 beam', '전체 beam'])
explain(fig, [
    ('title', '사용하는 입력과 역할'),
    ('text', '전·후방 LiDAR: 장애물 위치와 거리 급변 지점\nTraversability 지도: 장애물과 미관측 영역 보완\nOdometry: 현재 위치·방향·속도, scan 시간 보정\nGlobal path: 경로 비용과 차체 heading\n동적 예측: MPPI의 동적 충돌확률 계산'),
    ('title', '입력 유효성 조건'),
    ('text', '각 scan의 원본 시각 나이와 수신 후 경과시간 ≤ 0.25초\n전·후방 scan의 시각 차이 ≤ 0.05초\n로봇 이동을 고려해 beam마다 odometry를 보간합니다.\n센서 장착 위치·방향은 TF로 반영합니다.'),
    ('title', '입력이 불충분하면'),
    ('text', 'odometry로 보정할 수 없는 beam은 사용하지 않습니다.\n입력 누락·만료·TF/보정 실패이면 해당 주기는 hold합니다.\n이 그림은 저장된 시간 보정 좌표를 재사용했습니다.\n온라인 freshness/TF 검사를 다시 실행한 결과는 아닙니다.')])
takeaway(fig, '실제 입력: 전방 1,752개 + 후방 1,804개 유효 beam. 원본 장면 audit의 예측 객체 수는 0개입니다.')
save(fig, '01_inputs_and_alignment.png')

# 02: body reach, rather than raw range, defines the open angular runs.
fig = page(2, '로봇이 실제로 들어갈 수 있는 공간을 계산합니다', 'LiDAR 측정 거리 r와 로봇 중심의 이동 가능 거리 d(θ)는 구분합니다.')
ax = scene(fig)
from dra_mppi.gap_geometry import runs
for ids in runs(D['strict_depth'] >= .5):
    a, d = D['angles'][ids], D['strict_depth'][ids]
    xy = d[:, None]*np.column_stack((np.cos(a), np.sin(a)))
    ax.add_patch(Polygon(np.vstack(([0, 0], xy)), fc='#25aaa0', alpha=.26))
    ax.plot(xy[:, 0], xy[:, 1], color='#008f85', lw=2)
ax.add_patch(Circle((0, 0), .25, fill=False, ls='--', ec='#d75a70'))
fig.text(.065, .23, '흰색: scan free  |  연파랑: 저장된 지도 보완 free\n진한색: 장애물  |  회색: unknown', fontsize=12, color=MUTED, linespacing=1.7)
explain(fig, [
    ('title', '로봇 중심의 로컬 지도'),
    ('text', '8m × 8m, 해상도 4cm의 로컬 지도를 만듭니다.\n라이다 관측을 넣고 미관측 영역은 유효 지도로 보완합니다.\n장애물 정보는 유지하고, 남은 unknown은 막힌 영역입니다.'),
    ('title', '방향별 이동 가능 거리'),
    ('text', '360도 전체를 1도 간격으로 확인합니다.\n최대 확인 거리 2m, 거리 간격 2cm입니다.\n로봇 반경 0.20m와 추가 여유 0.05m를 적용합니다.\n격자·swept 검사에 필요한 보수적인 여유도 반영됩니다.'),
    ('math', r'$d(\theta)\geq 0.5\,\mathrm{m}$'),
    ('text', 'd(θ): 해당 방향으로 로봇 중심이 이동 가능한 거리\n위 조건을 만족하는 방향들을 연결해 열린 구역을 만듭니다.\n연속 구간 폭이 최소 6도인 경우에만 유지합니다.')])
takeaway(fig, '실제 결과: 열린 연결 구간 3개. 이 단계에서는 아직 raw 거리 급변으로 추가 분할하지 않았습니다.')
save(fig, '02_reachable_space_equation.png')

# 03: retain the earlier displayed formula and state the exact implementation.
fig = page(3, '열린 공간을 거리 급변 지점에서 나눕니다', 'raw LiDAR 거리 급변 + 지속성 + 로봇 이동 공간 변화 → gap 경계')
ax = scene(fig, rect=(.055, .39, .40, .44)); regions(ax)
edges = [e for e in R['partition']['edges'] if e['status'] == 'selected']
small_table(fig, [[e['beam_before'], f"{e['range_before']:.2f} → {e['range_after']:.2f}", e['cut_bin']-180]
                  for e in edges], ['후방 beam', '실제 거리 [m]', '최종 경계 [°]'], rect=(.055, .19, .40, .15))
explain(fig, [
    ('title', '앞서 설명한 거리 급변 기준'),
    ('math', r'$|r_{i+1}-r_i|\geq\max(0.4\,\mathrm{m},\;0.12\min(r_i,r_{i+1}))$'),
    ('text', 'r: 같은 센서의 인접 beam 측정 거리\n40cm 이상 급변을 찾고, 먼 물체는 거리 비례 기준을 적용합니다.'),
    ('title', '구현에서는 양쪽 3-beam 중앙값으로 안정화'),
    ('math', r'$T=\max(0.4\,\mathrm{m},\;0.12\min(\tilde r_L,\tilde r_R))$'),
    ('text', '실제 코드의 비례 기준은 인접값 대신 양쪽 중앙값을 씁니다.\n인접 차이와 중앙값 차이 모두 T 이상, 변화 방향도 같아야 합니다.\n양쪽 각 3 beam의 흔들림은 중앙값 차이의 35% 이하여야 합니다.'),
    ('title', '실제 이동 공간에서도 경계인지 검사'),
    ('text', '원래 경계 ±8도에서 양쪽 진입 깊이 차이 ≥ 0.25m를 확인합니다.\n강한 경계를 우선 남겨 경계 간격 ≥ 20도, 구역 폭 ≥ 6도를 유지합니다.\n먼 벽의 거리만 달라지고 이동 공간이 같으면 추가 분할하지 않습니다.\n열린 구역이 전혀 없을 때만 추가 여유 5cm를 제거해 재시도합니다.\n로봇 반경은 유지합니다. 이번 장면에서는 재시도하지 않았습니다.')])
takeaway(fig, '실제 결과: 열린 구간 3개 + 내부 경계 3개 → G1~G6. 앞선 간략 수식과 코드의 중앙값 적용 차이를 함께 명시했습니다.')
save(fig, '03_disparity_equations.png')

# 04: central seed, feasible distance and independent body yaw.
fig = page(4, '각 gap 중앙 방향으로 seed를 하나 만듭니다', '중앙은 각도 구간의 중앙입니다. XY 영역의 무게중심을 계산하는 방식은 아닙니다.')
ax = scene(fig, rect=(.055, .39, .40, .44)); regions(ax, labels=False)
line(ax, A['nominal_path'], color=INK, lw=2, ls='--', label='nominal')
for i, tr in enumerate(A['seed_paths']):
    line(ax, tr, color=COLORS[i], lw=2)
    target = R['seed_targets'][i]
    ax.scatter(*target, color=COLORS[i], marker='x', s=60)
    ax.annotate(f'G{i+1}', target, xytext=(5, 4), textcoords='offset points', color=COLORS[i], weight='bold')
ax.legend(fontsize=10)
small_table(fig, [[f'G{i+1}', f"{c['angle_deg']:.0f}°", f"{np.linalg.norm(R['seed_targets'][i]):.2f}"]
                  for i, c in enumerate(C)], ['gap', '중앙 각도', '목표 거리 [m]'], rect=(.055, .16, .40, .18))
explain(fig, [
    ('title', '목표점과 제어열 생성'),
    ('text', '① gap의 중앙 각도를 구합니다.\n② 그 방향의 이동 가능 거리에서 10cm를 남깁니다.\n③ 현재 속도와 입력 제약으로 2초 내 도달 거리를 계산합니다.\n④ 도달 거리의 약 90%를 반영해 목표점을 정합니다.\n⑤ 목표점을 향하는 10-step 제어열을 만듭니다.'),
    ('math', r'$u_t=[v_x,\;v_y,\;\omega],\quad\Delta t=0.2\,\mathrm{s},\quad N=10$'),
    ('text', 'vx, vy: 차체 기준 병진 속도  /  ω: 차체 회전 속도\n속도·가속도·바퀴 속도 제한을 적용합니다.\n병진 이동은 gap 중앙을, 차체 yaw는 경로 방향을 따릅니다.\n뒤쪽 gap으로 가기 위해 차체를 반드시 뒤로 돌릴 필요는 없습니다.'),
    ('title', '360도가 하나의 열린 구역일 때'),
    ('text', '중앙 각도가 없으므로 경로를 약 1m 따라간 점의\n방향으로 seed 하나를 만듭니다. 이번 장면에는 해당하지 않습니다.')])
takeaway(fig, '실제 결과: 6개 gap에 중앙 seed 6개 생성. 색 실선은 rollout, ×는 목표점, 검은 점선은 기존 nominal입니다.')
save(fig, '04_seed_control_equation.png')

# 05: explicit feasibility equations and their actual measured values.
fig = page(5, '탐색할 만한가? 먼저 도달 가능성을 검사합니다', '최적 경로인지보다, 그 방향으로 실제 움직일 수 있는 중앙 seed인지 확인합니다.')
ax = scene(fig, rect=(.055, .44, .40, .39), limit=1.1)
for i, tr in enumerate(A['seed_paths']):
    line(ax, tr, color=COLORS[i], lw=2)
    ax.text(*tr[-1, :2], f'G{i+1}', color=COLORS[i], weight='bold')
small_table(fig, [[f'G{i+1}', '없음' if not R['seed_static_collisions'][i] else '충돌',
                   f"{c['target_error_m']:.3f}", f"{c['projected_displacement_m']:.3f}"] for i,c in enumerate(C)],
            ['gap', '정적 충돌', '목표 오차 [m]', 'gap 이동 [m]'], rect=(.055, .18, .40, .19), fontsize=10)
explain(fig, [
    ('title', '세 조건을 모두 만족해야 합니다'),
    ('text', '1. 전체 이동 구간의 swept footprint에 정적 충돌이 없음\n2. 2초 뒤 위치와 목표점의 거리 ≤ 0.35m\n3. 해당 gap 방향으로 이동한 거리 ≥ 0.25m'),
    ('math', r'$E_g=\|p_{g,N}-q_g\|\leq0.35\,\mathrm{m}$'),
    ('math', r'$m_g=(p_{g,N}-p_0)^{\mathsf{T}}e_g\geq0.25\,\mathrm{m}$'),
    ('text', 'p0: 현재 위치  /  pg,N: gap seed의 마지막 위치\nqg: seed 목표점  /  eg: gap 방향의 단위벡터'),
    ('title', '후진도 같은 기준입니다'),
    ('text', '후방 gap이면 eg도 뒤를 향합니다.\n뒤로 제대로 이동하면 mg는 양수이므로 같은 기준으로 통과합니다.\n이번 장면은 6개 모두 충돌·오차·이동량 조건을 통과했습니다.'),
    ('text', '중앙 seed 하나에 대한 근사 판단입니다. 중앙이 탈락해도\ngap 내부의 모든 경로가 불가능하다는 뜻은 아닙니다.')])
takeaway(fig, '실제 G4(165°): 목표 오차 0.078m, gap 방향 이동 0.652m → 후방 후보도 도달 가능성 통과.')
save(fig, '05_feasibility_equations.png')

# 06: both nominal duplicate conditions and the invalid-nominal exception.
fig = page(6, 'nominal이 이미 탐색하는 방향을 중복 제거합니다', '같은 시간의 XY 위치를 비교합니다. RMS와 마지막 3점 조건을 동시에 만족해야 제외합니다.')
ax = scene(fig, rect=(.055, .42, .40, .41), limit=1.1)
line(ax, A['nominal_path'], color=INK, ls='--', lw=2.5, label='nominal')
for i in (4, 5):
    line(ax, A['seed_paths'][i], color=COLORS[i], lw=2.5, label=f'G{i+1}')
ax.legend(loc='upper right', fontsize=11)
small_table(fig, [[f'G{i+1}', f"{c['nominal_rms_m']:.3f}", f"{c['nominal_tail_m']:.3f}",
                   '중복 제외' if c['reason']=='duplicate_nominal' else '중복 아님'] for i,c in enumerate(C)],
            ['gap', 'RMS [m]', '끝 3점 [m]', '판정'], rect=(.055, .17, .40, .19), fontsize=10)
explain(fig, [
    ('title', 'nominal은 어디서 오는가?'),
    ('text', '직전 최종 선택 제어열을 한 step 이동한 값입니다.\n최초 실행 또는 reset 직후에는 기존 경로 추종 seed를 사용합니다.'),
    ('math', r'$D_{\mathrm{RMS}}=\sqrt{\frac{1}{N}\sum_{t=1}^{N}\|p_{g,t}-p_{\mathrm{nom},t}\|^2}$'),
    ('math', r'$D_{\mathrm{tail}}=\frac{1}{3}\sum_{t=N-2}^{N}\|p_{g,t}-p_{\mathrm{nom},t}\|$'),
    ('text', 'pg,t / pnom,t: 같은 시점의 gap / nominal XY 위치'),
    ('math', r'$D_{\mathrm{RMS}}<0.22\,\mathrm{m}\quad\wedge\quad D_{\mathrm{tail}}<0.30\,\mathrm{m}$'),
    ('text', '두 조건을 모두 만족하면 nominal 중복으로 제외합니다.\nnominal 자체가 정적 충돌이면 이 중복 제외를 적용하지 않습니다.\n이번 장면의 nominal은 정적 충돌이 없어 중복 검사를 적용했습니다.'),
    ('title', '실제 G5와 G6의 차이'),
    ('text', 'G5: RMS 0.127m AND 끝 3점 0.176m → 중복 제외\nG6: RMS 0.192m지만 끝 3점 0.304m → 중복 아님')])
takeaway(fig, '실제 결과: G5만 nominal 중복 제외. 후방이라는 이유로 제거한 것이 아니라 기존 nominal과 같은 탐색이기 때문입니다.')
save(fig, '06_nominal_duplicate_equations.png')

# 07: conditional ranking, exact existing objective, no improvement gate.
fig = page(7, '필요한 경우에만 비용으로 순위를 정합니다', '통과 gap이 3개 이하이고 서로 충분히 다르면 모두 유지합니다.')
rank = sorted([c for c in C if c['cost'] is not None], key=lambda c: c['cost'])
ax = fig.add_axes([.09, .42, .35, .37])
for i, c in enumerate(rank[::-1]):
    idx = int(c['id'].split('_')[1])-1
    ax.barh(i, c['cost'], color=COLORS[idx], alpha=1 if c['reason']=='selected' else .4)
    ax.text(c['cost']+.25, i, f"{c['cost']:.2f}", va='center', fontsize=12)
ax.set(yticks=np.arange(len(rank)), yticklabels=[c['id'] for c in rank[::-1]], xlim=(0, 32), xlabel='사전 비용 (낮을수록 우선)')
fig.text(.065, .31, 'G5는 nominal 중복으로 먼저 제외되어 비용 미계산.\n남은 5개 중 비용 순서 G4 → G6 → G3을 유지합니다.\nG1·G2는 도달 불가능이 아니라 예산 제한으로 제외됩니다.', fontsize=13, linespacing=1.7)
explain(fig, [
    ('title', '사전 비용을 계산하는 경우'),
    ('text', '• 통과한 gap이 3개 초과\n• 또는 gap seed끼리 XY RMS 거리가 0.25m 미만으로 겹침'),
    ('math', r'$J_{\mathrm{pre}}=J_{\mathrm{common}}+J_{\mathrm{static}}$'),
    ('text', 'Jcommon: 경로 오차, 진행량 오차, heading, 속도,\n입력 변화량, 목표 근처 종점 오차\nJstatic: 장애물과 가까운 정도에 대한 정적 soft cost'),
    ('title', '선택 순서'),
    ('text', '비용이 낮은 후보부터 확인하며 서로 겹치는 후보를 제거하고\n최대 3개를 남깁니다. 이 사전 단계에서는 동적 MC를 추가하지 않습니다.\nnominal보다 비용이 낮아야 통과한다는 조건도 없습니다.'),
    ('title', '후진 후보에 대한 의미'),
    ('text', '후진·우회를 사전에 무조건 제거하지는 않습니다.\n다만 후보가 많아 비용 경쟁을 하면 경로 추종 비용 때문에\n후진이 밀릴 수 있습니다. 항상 후진을 보존하는 구조는 아닙니다.')])
takeaway(fig, '실제 사전 비용: G4 13.18 < G6 14.36 < G3 19.19 < G1 22.89 < G2 27.56. 선택 gap은 G3·G4·G6입니다.')
save(fig, '07_value_cost_equation.png')

# 08: fixed sampling budget and MPPI, preserving the original explanation.
fig = page(8, '총 500개를 배분하고 그룹별 MPPI를 실행합니다', '현재는 비용 비례 배분이 아니라 gap당 고정 100개, 나머지는 nominal입니다.')
ax = scene(fig, rect=(.055, .39, .40, .44))
for i, name in enumerate(R['candidate_names'][:-1]):
    col = INK if i == 0 else COLORS[int(name.split('_')[1])-1]
    for tr in A['sampled_paths'][A['group_ids']==i]: line(ax, tr, color=col, alpha=.12, lw=.6)
small_table(fig, [[0, 500, '—', 500], [1, 400, 100, 500], [2, 300, 100, 500], [3, 200, 100, 500]],
            ['gap 수', 'nominal', 'gap별', '총합'], rect=(.055, .18, .40, .15), fontsize=11)
explain(fig, [
    ('title', '이동 rollout 예산'),
    ('math', r'$K_g=100,\quad K_{\mathrm{nom}}=500-100G,\quad 0\leq G\leq3$'),
    ('text', 'G: 선택된 gap 수. 독립 감속 Stop은 최종 검사에 별도로 추가합니다.\n이번 장면은 nominal 200개 + G3·G4·G6 각 100개입니다.'),
    ('title', '그룹별 MPPI'),
    ('text', '① nominal 또는 중앙 seed 주변에 Halton spline noise 추가\n② 입력 제약을 적용하고 각 후보를 rollout\n③ 기존 공통 비용 + 정적 비용 + 동적 위험으로 평가\n④ MPPI 가중평균으로 그룹별 대표 제어열 하나 생성\n기본 2개 worker가 그룹 평가를 나눠 처리합니다.'),
    ('title', 'gap은 sampling의 시작 방향입니다'),
    ('text', '최적화된 궤적을 gap 안에 묶거나 중앙 목표점에 반드시\n도착하도록 강제하지 않습니다. 평균 대표도 다시 검사해야 합니다.\n그림은 실제 생성한 전체 500개이며 무효 표본도 포함합니다.\n이번 재생에는 동적 예측이 없어 동적 위험 계산이 생략됐습니다.')])
takeaway(fig, '실제 결과: nominal 200 + G3 100 + G4 100 + G6 100 = 500개. 그룹별 최적화 뒤 이동 대표는 4개입니다.')
save(fig, '08_allocation_and_mppi.png')

# 09: final selection and exact nominal shift, plus stop semantics.
fig = page(9, '대표 궤적 재검사 → 최종 선택 → 다음 nominal', '평균 대표의 안전성을 다시 확인합니다. Stop은 유효한 이동 대표가 없을 때 사용하는 대안입니다.')
ax = scene(fig, rect=(.055, .43, .40, .40))
for i, name in enumerate(R['candidate_names']):
    col = INK if name=='nominal' else '#929aa2' if name=='stop' else COLORS[int(name.split('_')[1])-1]
    line(ax, A['representatives'][i], color=col, ls='--', lw=2)
line(ax, A['selected_path'], color='#e54836', lw=3.2, label='최종 선택 nominal'); ax.legend(fontsize=10)
small_table(fig, [[n, f'{R["final_costs"][i]:.3f}', '없음', '선택' if n==R['selected_mode'] else '대안' if n=='stop' else '미선택']
                  for i,n in enumerate(R['candidate_names'])], ['대표', '최종 비용', '정적 충돌', '결과'],
            rect=(.055, .17, .40, .18), fontsize=10)
explain(fig, [
    ('title', '기존 최종 안전 검사를 유지합니다'),
    ('text', 'sampling 동적 위험: Monte Carlo\n최종 대표·Stop 동적 위험: Gaussian disk 직접 적분\n정적 충돌: 이동 구간 전체의 footprint 검사\n이번 재생에는 동적 예측이 없어 동적 검증은 실행하지 않았습니다.'),
    ('title', '최종 선택 규칙'),
    ('text', '① 검사를 통과한 이동 대표가 있으면 그중 최저 비용 선택\n② 통과한 이동 대표가 없으면 현재 속도에서 감속하는 Stop\n③ Stop도 불가하면 no_safe_candidate_hold\nStop은 유효한 이동 대표와 최저 비용을 경쟁하지 않습니다.'),
    ('title', '출력과 다음 주기 연결'),
    ('text', '선택한 10-step pose·velocity reference를 MPC에 전달합니다.\n직전 최종 선택 제어열을 한 step 이동해 다음 nominal로 씁니다.'),
    ('math', r'$U_{\mathrm{nom}}^{\mathrm{next}}=[u_2^*,u_3^*,\ldots,u_N^*,u_N^*]$'),
    ('text', 'gap 번호는 매 주기 다시 붙입니다. 이전 번호의 제어열이나\nswitching hysteresis를 새 구역에 그대로 이어 붙이지 않습니다.')])
takeaway(fig, '실제 최종 비용: nominal 6.547 < G4 8.627 < G6 9.545 < G3 15.756. nominal 선택, safety_status=ok(이번 재생 범위).')
save(fig, '09_final_selection_and_shift.png')

# 10: exceptions and timing, explicitly distinguished from executed branches.
fig = page(10, '예외와 시간 제한을 처리합니다', '아래 예외는 구현 동작 설명입니다. 이 정상 입력 장면에서 모든 예외를 발생시킨 것은 아닙니다.')
small_table(fig, [
    ['정상 입력 + 선택 gap 0개', 'nominal에 500개 배정'],
    ['scan 누락·만료·시각 차이·TF 오류', '해당 주기 hold'],
    ['지도·예측 입력 유효성 실패', '기존 입력 오류 처리로 hold'],
    ['gap 생성·MPPI 계산 시간 초과', '결과 폐기 후 hold'],
    ['유효한 이동 대표 없음', '감속 Stop 검사'],
    ['Stop도 검사 불통과', 'no_safe_candidate_hold'],
    ['목표 근처 도착', '기존 최종 위치·yaw 정렬']],
    ['상황', '처리'], rect=(.055, .45, .40, .33), fontsize=11)
fig.text(.065, .32, '이번 장면에서 실제 실행된 분기\n유효한 cached 입력 → gap 6개 → 선택 gap 3개\n→ nominal 대표 선택 → 다음 nominal 준비', fontsize=13, linespacing=1.8)
explain(fig, [
    ('title', '계산 제한과 5Hz의 관계'),
    ('math', r'$t_{\mathrm{gap+MPPI}}>0.18\,\mathrm{s}\quad\Rightarrow\quad\mathrm{reject\ result\ /\ hold}$'),
    ('math', r'$T_{\mathrm{cycle}}=\frac{1}{5\,\mathrm{Hz}}=0.20\,\mathrm{s}$'),
    ('text', '0.18초는 늦은 계산 결과를 사용하지 않기 위한 제한입니다.\nworker 종료를 기다리는 시간이 있으므로 전체 실행시간을\n반드시 0.18초 안으로 제한하는 장치는 아닙니다.\nROS 입출력·TF·동시 실행 부하까지 포함해 5Hz를 확인해야 합니다.'),
    ('title', '현재까지 확인한 범위'),
    ('text', '구현 당시 전체 240개 테스트 통과\n실제 기록 10개 장면의 gap 경계가 기존 실험과 일치\n패키지 빌드 및 세 전략의 ROS 초기화 확인\n동적 장애물+ROS 전체 부하의 5Hz 충족은 별도 검증 필요'),
    ('title', '이번 PNG의 범위'),
    ('text', '같은 b2_100의 저장된 실제 계산 결과를 그림으로 설명합니다.\n동적 예측, 온라인 freshness/TF, 실제 MPC 추종은 재생하지 않았습니다.\n다음 nominal은 제어열 shift이며 다음 센서 장면은 아닙니다.')])
takeaway(fig, '빈 gap은 정상적인 탐색 결과이며 입력 오류와 다릅니다. 0개면 nominal 탐색을 계속하고, 입력 오류이면 hold합니다.')
save(fig, '10_failures_and_deadline.png')

# Overview follows the same ten numbered explanatory stages.
fig = page(0, '전체 FLOW · 앞서 설명한 ①~⑩과 같은 순서', '분할 → 중앙 seed → 도달 가능성 → 중복 제외 → 필요할 때 순위 → MPPI → 최종 검사')
box_ax = fig.add_axes([.04, .53, .92, .32]); box_ax.axis('off')
titles = ['① 입력·시간 보정', '② 이동 공간 계산', '③ 급변으로 분할', '④ 중앙 seed 생성', '⑤ 도달 가능성',
          '⑥ nominal 중복', '⑦ 필요 시 비용 순위', '⑧ 500개 MPPI', '⑨ 재검사·최종 선택', '⑩ 예외·시간 제한']
subs = ['전후방 scan + 지도', '로봇 반경·여유 적용', '열린 3구간 → 6 gap', '2초 · 10 step', '충돌·오차·이동량',
        'G5 제외', 'G3·G4·G6 선택', '200 + 100×3', 'nominal 비용 6.547', '오류·초과 시 hold']
for i, (title, sub) in enumerate(zip(titles, subs)):
    col, row = i % 5, i // 5
    x, y = col*.2+.008, .60-row*.53
    box_ax.add_patch(FancyBboxPatch((x, y), .178, .30, boxstyle='round,pad=0.008',
                                   fc='#e4edf5', ec='#a3b7c9'))
    box_ax.text(x+.089, y+.20, title, ha='center', va='center', fontsize=14, weight='bold')
    box_ax.text(x+.089, y+.09, sub, ha='center', va='center', fontsize=11, color=MUTED)
    if col < 4:
        box_ax.annotate('', (x+.196, y+.15), (x+.179, y+.15), arrowprops=dict(arrowstyle='->', color=INK))
box_ax.annotate('⑤ 이후 아래 ⑥으로 이어짐', (.10, .42), (.80, .45),
                arrowprops=dict(arrowstyle='->', color=MUTED), fontsize=11, color=MUTED, ha='left')
ax = scene(fig, rect=(.065, .15, .32, .32)); regions(ax)
fig.text(.44, .43, '모든 그림에 포함한 내용', fontsize=19, weight='bold', va='top')
fig.text(.44, .375, '앞서 설명한 ①~⑩의 순서와 판단 기준\n수식 + 변수 뜻 + 임계값 + 예외 조건\n같은 실제 LiDAR 장면의 seed·수치·선택 결과', fontsize=15, linespacing=1.8, va='top')
fig.text(.44, .23, '실제 결과', fontsize=18, weight='bold', va='top')
fig.text(.44, .18, '6개 gap → G5 중복 제외 → G1·G2 예산 제외\nG3·G4·G6을 추가 탐색 → 최종 nominal 선택', fontsize=15, linespacing=1.7, va='top')
save(fig, '00_full_flow.png')

(OUT / 'EXPLANATION.txt').write_text('수식 포함 전체 설명 · 실제 장면 b2_100\n'+''.join(CONTENT)+
    '\n원본 수치: ../results.json, ../replay_arrays.npz\n'
    '③ 원래 설명의 거리 급변 수식은 보존하고, 코드가 중앙값을 쓰는 차이를 바로 아래 명시함.\n')
(OUT / 'INDEX.txt').write_text('\n'.join(sorted(FILES))+'\n\n재현: tools/render_gap_flow_formulas.py\n'
    '새 계산/설정 변경 없이 이전 production 재생 결과를 사용함.\n')
with zipfile.ZipFile(OUT / 'gap_flow_with_formulas.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
    for filename in sorted(FILES)+['EXPLANATION.txt', 'INDEX.txt']:
        archive.write(OUT / filename, filename)
    archive.write(SOURCE / 'results.json', 'results.json')
print(json.dumps({'output': str(OUT), 'png_count': len(FILES), 'files': sorted(FILES)}, ensure_ascii=False))

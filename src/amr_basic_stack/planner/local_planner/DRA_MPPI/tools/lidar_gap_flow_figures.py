"""Publication-sized PNGs and a traceable report for the offline gap example."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, FancyBboxPatch, FancyArrowPatch, Ellipse
from PIL import Image, ImageOps
import numpy as np

from render_lidar_gap_flow import ROOT, RES, EXTENT, RADIUS, MARGIN, LEVELS, stamp
from dra_mppi.mecanum_model import rollout

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family': font_manager.FontProperties(fname=FONT).get_name(),
                     'font.size': 11, 'axes.unicode_minus': False,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.facecolor': '#f8fafc', 'savefig.facecolor': '#f8fafc'})
INK, MUTED, BLUE, TEAL, ORANGE = '#142b45', '#526479', '#2463d3', '#11877f', '#d66b32'
COLORS = [BLUE, TEAL, '#a052b5', ORANGE, '#64748b', '#b34b59', '#96812a']


def clean_json(v):
    if isinstance(v, np.ndarray):
        return clean_json(v.tolist())
    if isinstance(v, dict):
        return {k: clean_json(x) for k, x in v.items()}
    if isinstance(v, (tuple, list)):
        return [clean_json(x) for x in v]
    if isinstance(v, (np.bool_,)):
        return bool(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return float(v) if np.isfinite(v) else None
    return v


def canvas(num, title, subtitle, p):
    fig = plt.figure(figsize=(16, 9), dpi=150)
    fig.text(.05, .93, f'{num:02d}', fontsize=30, weight='bold', color=BLUE)
    fig.text(.115, .937, title, fontsize=24, weight='bold', color=INK)
    fig.text(.115, .891, subtitle, fontsize=11.5, color=MUTED)
    fig.text(.05, .038,
             f'실측 bag: mpc_chain_20260923_135325  |  +{p["snap"]["elapsed"]:.3f} s  |  현재 base 기준 [m]',
             fontsize=9, color=MUTED)
    fig.text(.95, .038, '오프라인 설계 예제 · 실제 주행 명령 아님', ha='right', fontsize=9, color=MUTED)
    ax = fig.add_axes([.065, .15, .48, .68])
    side = fig.add_axes([.61, .15, .34, .68])
    return fig, ax, side


def body(ax, text, y=.96, size=13, color=INK):
    ax.text(0, y, text, transform=ax.transAxes, va='top', fontsize=size,
            color=color, linespacing=1.7)


def map_plot(ax, p, terrain=True, limits=2.3, path=True):
    if terrain:
        colors = np.zeros_like(p['grid'], dtype=int)
        colors[p['grid'] < 0] = 1
        colors[p['map_fill'] & (p['grid'] == 0)] = 2
        colors[p['grid'] >= 65] = 3
        ax.imshow(colors, extent=(-EXTENT, EXTENT, -EXTENT, EXTENT), origin='lower',
                  cmap=ListedColormap(['#ffffff', '#d8e0e8', '#e6edfc', '#263c50']),
                  vmin=0, vmax=3, interpolation='nearest', alpha=.95)
    for s, col in zip(p['scans'], [ORANGE, TEAL]):
        pts = s['hits'][s['valid']]
        ax.scatter(pts[:, 0], pts[:, 1], s=3, color=col, alpha=.70,
                   rasterized=True, zorder=3)
    if path:
        ax.plot(p['full_path'][:, 0], p['full_path'][:, 1], '--', color='#65a8cb',
                lw=2, alpha=.9, label='기록 global path')
    ax.add_patch(Circle((0, 0), RADIUS, facecolor='#193653', edgecolor='white', zorder=9))
    ax.annotate('', xy=(.40, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle='-|>', lw=2, color=INK), zorder=10)
    ax.set(xlim=(-limits, limits), ylim=(-limits, limits), aspect='equal',
           xlabel='전방 x [m]', ylabel='좌측 y [m]')
    ax.grid(alpha=.15, color=MUTED)


def trajectory(ax, arr, color, label=None, lw=2.5, alpha=1., style='-', zorder=6):
    xy = np.vstack((np.zeros(2), arr[:, :2]))
    ax.plot(xy[:, 0], xy[:, 1], style, color=color, lw=lw, alpha=alpha,
            label=label, zorder=zorder)


def gap_arc(ax, p, g, color, alpha=1., label=True):
    a = np.unwrap(p['angles'][g['ids']])
    ax.plot(g['level']*np.cos(a), g['level']*np.sin(a), color=color, lw=5,
            alpha=alpha, solid_capstyle='round', zorder=5)
    target = g['target']
    ax.scatter(*target, color=color, s=46, edgecolor='white', zorder=7)
    if label:
        xy = target+np.array([.08, .09])
        ax.text(*xy, g['id'], color=color, fontsize=13, weight='bold', zorder=12,
                bbox=dict(facecolor='white', alpha=.85, edgecolor='none', pad=1))


def save(fig, out, name):
    fig.savefig(out/name, dpi=150)
    plt.close(fig)


def overall(p, q, opt, out):
    fig = plt.figure(figsize=(16, 10), dpi=150)
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis('off')
    ax.text(.05, .94, 'LiDAR gap 기반 DRA-MPPI · 전체 flow', fontsize=25, color=INK, weight='bold')
    ax.text(.05, .895, '5 Hz 설계: nominal 1개 + gap 최대 3개 / 이동 rollout 총 500개 / MC 밀도 유지', fontsize=14, color=MUTED)
    boxes = [
        ('01  실제 입력 정렬', '앞·뒤 scan + odometry / TF\n기록 global path / 이전 reference\n지도 + person predictions'),
        ('02  통과 공간 계산', '유효 beam만 free-space 구성\n미관측은 최신 기록 지도로 보완\n반경 0.20 + 여유 0.05 m'),
        ('03  gap / 임시 seed', 'L = 0.5 / 1.0 / 1.5 m\n연속 통과 각도, 폭 ≥ 6°\n거리별 중복 병합 → 중앙 seed'),
        ('04  nominal 중복 제거', '시간 정렬된 기준 입력열 유지\nseed 궤적 RMS / 후반부 거리\nnominal이 막히면 제거 완화'),
        ('05  탐색 가치 / Top-3', '충돌·도달 가능성 우선 검사\n진행 + 여유 + 차별성 + 재합류\n후보끼리도 궤적 중복 제거'),
        ('06  제어입력열 생성', 'gap 중앙을 향하는 vx, vy\nyaw는 global path 방향 추종\n속도·가속도·wheel 제약 적용'),
        ('07  guide별 MPPI', 'nominal 200 + gap 100 × 3\nHalton spline / 2 workers\n정적 swept 검사 + 기존 MC'),
        ('08  공통 최종 선택', '대표 궤적을 공통 MC로 재평가\n안전 이동 후보 중 최소 비용\n없으면 Stop, 이후 warm start'),
    ]
    pos = [(.05+i*.235, .57) for i in range(4)] + [(.05+i*.235, .27) for i in reversed(range(4))]
    for i, ((title, text), (x, y)) in enumerate(zip(boxes, pos)):
        ax.add_patch(FancyBboxPatch((x, y), .205, .235, boxstyle='round,pad=.01,rounding_size=.015',
                                   facecolor='white', edgecolor='#cad6e4', linewidth=1.3))
        ax.text(x+.012, y+.19, title, fontsize=14, weight='bold', color=BLUE if i<4 else TEAL)
        ax.text(x+.012, y+.137, text, fontsize=11, va='top', linespacing=1.75, color=INK)
        if i:
            px, py = pos[i-1]
            if i == 4:
                a, b = (px+.1025, py-.005), (x+.1025, y+.245)
            elif i < 4:
                a, b = (px+.215, py+.12), (x-.012, y+.12)
            else:
                a, b = (px-.012, py+.12), (x+.215, y+.12)
            ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=18, color=MUTED, lw=1.6))
    ax.text(.05, .19, f'이 장면: {len(q["gaps"])}개 gap → nominal 중복 / 도달성 검사 → {len(q["selected"])}개 탐색 → 최종 {opt["result"].selected_mode}',
            fontsize=17, color=INK, weight='bold')
    ax.text(.05, .13, '실시간 통합 시: gap ID 연결·전환 hysteresis·입력 freshness gate·남은 시간에 따른 후보 수 제한', fontsize=12, color=MUTED)
    ax.text(.05, .085, '갱신: 선택 입력열을 한 step 이동 → 다음 nominal / gap 실패 시 고정 guide fallback / 기존 안전검사 유지', fontsize=12, color=MUTED)
    ax.text(.05, .035, '실측 데이터로 수행한 단일 장면 계산. 5 Hz 달성·폐루프 성능 검증은 별도이며, 시간 연속 gap ID는 이 예제에서 미구현.', fontsize=10, color=MUTED)
    save(fig, out, '00_overall_flow.png')


def stages(p, q, opt, out):
    r = opt['result']
    gcolors = {g['id']: COLORS[(i+1) % len(COLORS)] for i, g in enumerate(q['gaps'])}
    ncolors = {name: BLUE if name == 'nominal' else gcolors[name] for name in opt['names']}
    fig, ax, side = canvas(1, '실제 LiDAR와 당시 주행 문맥', '측정점·경로·기준 입력열의 출처를 고정한 뒤 같은 장면을 끝까지 사용합니다.', p)
    map_plot(ax, p, terrain=False, limits=4)
    trajectory(ax, q['nominal_path'], BLUE, '기록 입력열 재전개', lw=3)
    ax.legend(loc='upper right', fontsize=9)
    side.axis('off')
    ns = [int(s['valid'].sum()) for s in p['scans']]
    skew = abs(p['scans'][0]['stamp']-p['scans'][1]['stamp'])*1000
    body(side, f'실측 데이터\n\n주황: 전방 LiDAR  {ns[0]:,} valid beams\n청록: 후방 LiDAR  {ns[1]:,} valid beams\n스캔 시작 시각 차이: {skew:.1f} ms\n\nbeam 시각별 odometry 보간으로\n현재 로봇 좌표에 정렬했습니다.\n\n파란 궤적은 기록 reference의\n몸체 속도를 현재 시각에 맞춰\n재전개한 nominal입니다.\n\nHorizon: 2.0 s  /  dt: 0.20 s', size=13)
    save(fig, out, '01_recorded_lidar.png')

    fig, ax, side = canvas(2, '로봇 크기를 반영한 통과 가능 공간', 'scan에서 확인한 자유공간과 지도 보완 영역을 구분하고, unknown은 장애물로 취급합니다.', p)
    map_plot(ax, p)
    coords = -EXTENT+(np.arange(p['grid'].shape[0])+.5)*RES
    ax.contour(coords, coords, p['center_clearance'], levels=[RADIUS+MARGIN+.01], colors=[ORANGE], linewidths=1.5)
    ax.add_patch(Circle((0, 0), RADIUS+MARGIN, fill=False, ls='--', ec=ORANGE, lw=2, zorder=11))
    side.axis('off')
    body(side, '흰색     LiDAR로 확인한 자유공간\n연파랑  기록 지도로 보완한 자유공간\n회색     미관측 / unknown\n짙은색  장애물\n주황선  로봇 중심의 통과 경계\n\n로봇 반경       0.20 m\n탐색 여유       0.05 m\n격자 해상도    0.04 m\n\n셀 대각선과 ray 간격도 보수적으로\n반영해 경계 통과를 판정합니다.\n\ninf / NaN을 빈 공간으로\n바꾸지 않습니다.', size=13)
    save(fig, out, '02_free_space_and_clearance.png')

    fig, ax, side = canvas(3, '다중 거리에서 gap 나누기', 'd_free(θ): 해당 방향으로 로봇 중심이 연속 이동할 수 있는 최대 거리', p)
    map_plot(ax, p)
    for level in LEVELS:
        ax.add_patch(Circle((0, 0), level, fill=False, ec='#9aaec4', ls=':', lw=1))
    for g in q['gaps']:
        gap_arc(ax, p, g, gcolors[g['id']])
    side.plot(np.rad2deg(p['angles']), p['d_free'], color=INK, lw=1.8)
    for level, color in zip(LEVELS, [TEAL, BLUE, ORANGE]):
        side.axhline(level, color=color, ls='--', lw=1, label=f'L = {level:.1f} m')
    side.set(xlim=(-180, 180), ylim=(0, 2.15), xlabel='방향 θ [deg]', ylabel='d_free [m]')
    side.legend(loc='upper right', fontsize=9)
    side.grid(alpha=.2)
    side.set_position([.61, .44, .34, .36])
    fig.text(.61, .32, f'최소 각도 폭 6° · 원형 연결(-180° ↔ +180°)\n거리별 같은 방향 후보 병합 후 {len(q["gaps"])}개 gap',
             fontsize=13, color=INK, linespacing=1.8, va='top')
    fig.text(.61, .19, '넓은 gap의 중앙을 첫 seed로 사용합니다.\n장애물 뒤 통로의 연결성까지 보장하지는 않습니다.', fontsize=11, color=MUTED, linespacing=1.8, va='top')
    save(fig, out, '03_multiscale_gaps.png')

    fig, ax, side = canvas(4, 'nominal이 이미 탐색하는 후보 제외', '첫 방향 각도만 비교하지 않고, 같은 시각의 전체 seed 궤적을 비교합니다.', p)
    map_plot(ax, p)
    trajectory(ax, q['nominal_path'], BLUE, 'nominal', lw=4)
    for g in q['gaps']:
        col = '#9ba8b5' if g['duplicate'] else gcolors[g['id']]
        trajectory(ax, g['trajectory'], col, g['id']+(' 중복' if g['duplicate'] else ''),
                   alpha=.65, style='--' if g['duplicate'] else '-')
        ax.text(*g['target'], g['id'], fontsize=11, color=col,
                bbox=dict(facecolor='white', edgecolor='none', alpha=.8))
    ax.legend(loc='lower right', fontsize=9)
    side.axis('off')
    rows = '\n'.join(f'{g["id"]}   RMS {g["rms_distance"]:.2f} m / 후반 {g["tail_distance"]:.2f} m'
                     + ('  → 제외' if g['duplicate'] else '') for g in q['gaps'])
    body(side, '초기 중복 기준\n\n전체 위치 RMS < 0.22 m\nAND 마지막 3 step 평균 < 0.30 m\n\n'+rows+
         '\n\nnominal이 충돌하면 이 제거를 해제합니다.\n넓은 gap 전체를 삭제하지 않고\n이 gap의 대표 seed만 중복 처리합니다.', size=12.5)
    save(fig, out, '04_nominal_deduplication.png')

    fig, ax, side = canvas(5, '탐색 가치 평가와 Top-3 선택', '저렴한 geometry 평가로 예산을 배정하고, 확률적 안전성은 뒤의 기존 MC 단계에서 판정합니다.', p)
    map_plot(ax, p)
    for g in q['gaps']:
        chosen = g['reason'] == '채택'
        gap_arc(ax, p, g, gcolors[g['id']] if chosen else '#9ba8b5', alpha=1 if chosen else .4)
        trajectory(ax, g['trajectory'], gcolors[g['id']] if chosen else '#9ba8b5',
                   alpha=.9 if chosen else .35, style='-' if chosen else '--')
    side.axis('off')
    body(side, 'V = 0.40 진행 + 0.25 여유\n     + 0.20 차별성 + 0.15 경로 근접\n\n모든 항목은 [0, 1] 범위입니다.\n충돌 없음 / 목표 오차 ≤ 0.35 m / V ≥ 0.25', size=12)
    rows = [[g['id'], f'{g["score"]:.3f}', f'{g["reach_error"]:.3f}', g['reason']] for g in q['gaps']]
    table = side.table(cellText=rows, colLabels=['gap', '가치 V', '목표 오차 m', '판정'],
                       cellLoc='center', colWidths=[.14, .17, .28, .31], bbox=[0, .27, 1., .41])
    table.auto_set_font_size(False); table.set_fontsize(10.5)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#d7e0e8')
        cell.set_facecolor('#e8eff8' if row==0 else 'white')
    body(side, '새 후보끼리 RMS < 0.25 m이면 하나만 유지.\n이 점수는 탐색 순위이며 충돌확률이 아닙니다.', y=.19, size=11, color=MUTED)
    save(fig, out, '05_exploration_value.png')

    fig, ax, side = canvas(6, 'gap 중앙을 향하는 제어입력열', '목표 위치 → 몸체 vx, vy, yaw_rate → 축별·가속도·wheel 제약 → 10 step rollout', p)
    map_plot(ax, p)
    for name, seed in zip(opt['names'], opt['seeds']):
        trajectory(ax, rollout(np.zeros(3), seed, .2), ncolors[name], name)
    for g in q['selected']:
        ax.scatter(*g['target'], color=gcolors[g['id']], marker='x', s=75, zorder=12)
        ax.text(*(g['target']+np.array([.08,.07])), g['id'], color=gcolors[g['id']], fontsize=12)
    ax.legend(loc='lower right', fontsize=10)
    side.remove()
    times = np.arange(1, 11)*.2
    for i, label in enumerate(['vx [m/s]', 'vy [m/s]', 'yaw rate [rad/s]']):
        a = fig.add_axes([.63, .64-i*.21, .31, .16])
        for name, seed in zip(opt['names'], opt['seeds']):
            a.plot(times, seed[:, i], color=ncolors[name], lw=2)
        a.set(ylabel=label, xlim=(.2, 2.));a.grid(alpha=.2)
        if i==2:
            a.set_xlabel('미래 시간 [s]')
    fig.text(.62, .105, 'x 표시 = gap 중앙 목표 / 병진 방향과 yaw를 분리해 제어', fontsize=10, color=MUTED)
    save(fig, out, '06_control_sequence_seeds.png')

    fig, ax, side = canvas(7, 'guide별 병렬 MPPI sampling', '현재 Halton spline sampler·동역학·공통 비용·MC 평가 코드를 그대로 사용한 오프라인 계산', p)
    map_plot(ax, p)
    # Reconstruct all 500 rollouts in original sampling order for honest counts.
    all_paths = rollout(np.zeros(3), r.sampled_controls, .2)
    for i, name in enumerate(opt['names']):
        paths = all_paths[r.control_mode_ids == i]
        for tr in paths:
            trajectory(ax, tr, ncolors[name], lw=.65, alpha=.12, zorder=4)
        trajectory(ax, r.mode_trajectories[i], ncolors[name], name, lw=3)
    ax.legend(loc='lower right', fontsize=10)
    side.axis('off')
    rows = '\n'.join(f'{name:8}  {count:3d}개' for name,count in zip(opt['names'],opt['counts']))
    valid_counts = [int(np.isfinite(r.rollout_costs[r.control_mode_ids==i]).sum()) for i in range(len(opt['names']))]
    valid_rows = '\n'.join(f'{name}: 유한 비용 {n}/{count}' for name,n,count in zip(opt['names'],valid_counts,opt['counts']))
    body(side, '전체 이동 rollout = 500\n\n'+rows+'\n\n2 workers · 2초 horizon\n각 guide 내부에서만 weighted update\n\n'+valid_rows+
         '\n\n얇은 선: 실제 계산한 500개 rollout\n굵은 선: guide별 최적화 대표', size=12.5)
    save(fig, out, '07_parallel_mppi_sampling.png')

    fig, ax, side = canvas(8, '공통 재평가와 최종 후보 선택', '최적화 대표의 swept 충돌·동적 확률을 다시 계산한 뒤 이동 후보를 비교합니다.', p)
    map_plot(ax, p)
    pred = opt['predictions']
    if pred is not None:
        ax.scatter([], [], color='#c14855', marker='D', s=35, label='기록 person (2σ)')
        for o in range(len(pred.means)):
            k = int(np.argmax(pred.mode_weights[o, :, 0]))
            mean, cov = pred.means[o, k, 0], pred.covariances[o, k, 0]
            values, vectors = np.linalg.eigh(cov)
            angle = np.rad2deg(np.arctan2(vectors[1, -1], vectors[0, -1]))
            ax.add_patch(Ellipse(mean, 4*np.sqrt(values[-1]), 4*np.sqrt(values[0]),
                                 angle=angle, facecolor='#c14855', alpha=.14, edgecolor='#c14855'))
            ax.scatter(*mean, color='#c14855', marker='D', s=35, zorder=10)
    for i, name in enumerate(opt['names']):
        trajectory(ax, r.mode_trajectories[i], ncolors[name], name,
                   lw=2, alpha=.55, style='--')
    trajectory(ax, r.mode_trajectories[-1], '#6b7280', 'Stop', style=':')
    trajectory(ax, r.trajectory, INK, '선택: '+r.selected_mode, lw=4.5, zorder=13)
    ax.legend(loc='lower right', fontsize=9)
    side.axis('off')
    rows = [[name, 'invalid' if not np.isfinite(cost) else f'{cost:.2f}', f'{risk:.2e}']
            for name,cost,risk in zip(r.candidate_names, r.mode_final_costs,
                                     np.max(r.mode_collision_probabilities, axis=1))]
    body(side, f'최종 선택: {r.selected_mode}\n\n대표 {len(opt["names"])}개 + 독립 Stop 1개\n기록 person 예측 {0 if pred is None else len(pred.means)}개 사용\n동적 확률 기준: 각 step ≤ 0.05', size=13)
    table = side.table(cellText=rows, colLabels=['후보', '최종 비용', 'max P(collision)'],
                       cellLoc='center', colWidths=[.22,.26,.45], bbox=[0,.28,1,.37])
    table.auto_set_font_size(False);table.set_fontsize(10.5)
    for (row,col),cell in table.get_celld().items():
        cell.set_edgecolor('#d7e0e8');cell.set_facecolor('#e8eff8' if row==0 else 'white')
    body(side, 'gap은 탐색 기회를 추가합니다.\n새 gap이 항상 선택되는 것은 아닙니다.\n이 결과는 실제 실행 궤적이 아닙니다.', y=.2, size=11.5, color=MUTED)
    save(fig, out, '08_common_recheck_and_selection.png')


def produce(data, p, q, opt, out):
    overall(p, q, opt, out)
    stages(p, q, opt, out)
    pngs = sorted(out.glob('0[0-8]_*.png'))
    # Contact sheet is assembled from rendered scientific figures, not AI images.
    thumbs = []
    for file in pngs:
        with Image.open(file) as im:
            thumbs.append(ImageOps.pad(im.convert('RGB'), (800, 500), color='#f8fafc'))
    sheet = Image.new('RGB', (2464, 1564), '#dbe3ed')
    for i, im in enumerate(thumbs):
        sheet.paste(im, (16+(i%3)*816, 16+(i//3)*516))
    sheet.save(out/'09_all_steps.png')
    r = opt['result']
    source = dict(bag=data['bag'], record_ns=p['snap']['record_ns'],
                  elapsed_s=p['snap']['elapsed'], evaluation_epoch_s=p['epoch'],
                  pose_odom=p['pose'], measured_body_velocity=p['current'],
                  reference_age_s=p['ref_age'], map_age_s=p['map_age'],
                  scan_range_sha256={s['topic']: hashlib.sha256(s['ranges'].tobytes()).hexdigest() for s in p['scans']},
                  topics={name: dict(record_ns=t, header_s=stamp(m), frame=m.header.frame_id)
                          for name,(t,m) in p['snap']['topics'].items()},
                  metadata_sha256=hashlib.sha256((Path(data['bag'])/'metadata.yaml').read_bytes()).hexdigest())
    manifest = dict(source=source,
                    parameters=dict(grid_resolution=RES, robot_radius=RADIUS, gap_margin=MARGIN,
                                    levels=LEVELS, minimum_width_deg=6, direction_merge_deg=12,
                                    duplicate_rms_m=.22, duplicate_tail_m=.30, reach_error_max_m=.35,
                                    min_score=.25, gap_pair_rms_m=.25,
                                    score_weights=[.4,.25,.2,.15], rollout_counts=opt['counts'],
                                    horizon_steps=10, dt=.2, workers=2, offline_timeout_s=10.),
                    gap_candidates=[{k:v for k,v in g.items() if k not in ('controls','trajectory','ids')}
                                    for g in q['gaps']],
                    nominal_collision=q['nominal_collision'], selected_gaps=[g['id'] for g in q['selected']],
                    final=dict(names=r.candidate_names, costs=r.mode_final_costs,
                               max_collision_probability=np.max(r.mode_collision_probabilities,axis=1),
                               selected=r.selected_mode, safety_status=r.safety_status,
                               finite_rollouts=int(np.isfinite(r.rollout_costs).sum()),
                               prediction_count=0 if opt['predictions'] is None else len(opt['predictions'].means)),
                    single_run_ms={k:opt[k] for k in ('prepare_ms','proposal_ms','optimizer_ms')},
                    timing_scope='One offline cold snapshot, not a 5 Hz or p99 validation. Preparation includes recorded-data alignment and grid construction; plotting and bag I/O excluded.',
                    controller_config=asdict(q['config']),
                    deviations=['Recorded reference controls time-resampled; not recovered internal warm-start state.',
                                'Scan-derived static grid supplemented by recorded traversability grid; not the exact original live static-cost input.',
                                'Offline optimizer timeout 10 s, production timeout 0.18 s.',
                                'No persistent gap ID association or closed-loop replay in this single snapshot.',
                                'Geometry-only proposal score; MC risk is applied to every sampled and final trajectory.'],
                    sources_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
                                    for f in [Path(__file__), ROOT/'tools/render_lidar_gap_flow.py', ROOT/'config/dra_mppi.yaml',
                                              ROOT/'dra_mppi/guide_conditioned.py', ROOT/'dra_mppi/mppi_controller.py',
                                              ROOT/'dra_mppi/dynamic_risk.py']})
    (out/'manifest.json').write_text(json.dumps(clean_json(manifest), ensure_ascii=False, indent=2)+'\n')
    arrays = dict(grid=p['grid'], scan_grid=p['scan_grid'], map_fill=p['map_fill'],
                  d_free=p['d_free'], angles=p['angles'], full_path=p['full_path'],
                  path=p['path'], nominal=q['nominal'], nominal_path=q['nominal_path'],
                  current=p['current'], pose=p['pose'], reference_times=p['ref_times'],
                  reference_controls=p['ref_controls'], seeds=opt['seeds'],
                  sampled_controls=r.sampled_controls, sampled_costs=r.rollout_costs,
                  mode_ids=r.control_mode_ids, representatives=r.mode_trajectories,
                  representative_controls=r.mode_control_sequences,
                  final_costs=r.mode_final_costs, final_probabilities=r.mode_collision_probabilities,
                  selected_trajectory=r.trajectory, guide_mc_areas=r.guide_mc_region_areas,
                  guide_mc_samples=r.guide_mc_sample_counts, final_mc_samples=r.final_mc_sample_counts)
    for i,s in enumerate(p['scans']):
        for name in ('ranges','angles','beam_times','valid','origins','hits'):
            arrays[f'scan_{i}_{name}'] = s[name]
    if opt['predictions'] is not None:
        for name in ('means','covariances','mode_weights','times','obstacle_radii'):
            arrays['prediction_'+name] = getattr(opt['predictions'],name)
    np.savez_compressed(out/'snapshot_and_results.npz', **arrays)
    write_report(data,p,q,opt,out)
    with zipfile.ZipFile(out/'lidar_gap_flow_pngs.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for f in sorted(out.iterdir()):
            if f.suffix in ('.png','.md','.json','.npz'):
                archive.write(f,f.name)
        archive.write(ROOT/'tools/render_lidar_gap_flow.py','tools/render_lidar_gap_flow.py')
        archive.write(Path(__file__),'tools/lidar_gap_flow_figures.py')
    print(json.dumps(clean_json(dict(output=str(out), selected=r.selected_mode,
          gaps=len(q['gaps']), chosen=[g['id'] for g in q['selected']],
          counts=opt['counts'], timings=manifest['single_run_ms'])),ensure_ascii=False,indent=2))


def write_report(data,p,q,opt,out):
    r=opt['result']
    rows='\n'.join(f'| {g["id"]} | {np.rad2deg(g["angle"]):.0f}° | {g["level"]:.1f} | {g["width_deg"]}° | {g["rms_distance"]:.3f} | {g["reach_error"]:.3f} | {g["score"]:.3f} | {g["reason"]} |' for g in q['gaps'])
    final='\n'.join(f'| {name} | {cost:.5f} | {risk:.6f} |' for name,cost,risk in zip(r.candidate_names,r.mode_final_costs,np.max(r.mode_collision_probabilities,axis=1)))
    report=f'''# LiDAR gap 기반 DRA-MPPI flow — 실제 기록 예제

2026-11-04 작성. 실제 bag `{Path(data['bag']).name}`의 시작 후 **{p['snap']['elapsed']:.3f}초** 장면에서 계산했다. 모든 PNG는 수치 데이터로 그렸으며 장애물이나 gap을 그림용으로 추가하지 않았다. 원본 bag과 운영 planner는 수정하지 않았다.

![전체 단계](09_all_steps.png)

## 파일

| PNG | 내용 |
|---|---|
| [00_overall_flow.png](00_overall_flow.png) | 전체 설계 flow와 실시간 통합 시 추가할 기능 |
| [01_recorded_lidar.png](01_recorded_lidar.png) | 실측 LiDAR·global path·기록 reference 기반 nominal |
| [02_free_space_and_clearance.png](02_free_space_and_clearance.png) | 자유공간·미관측·지도 보완·로봇 크기 |
| [03_multiscale_gaps.png](03_multiscale_gaps.png) | 다중 거리 gap과 방향별 도달 거리 |
| [04_nominal_deduplication.png](04_nominal_deduplication.png) | nominal과의 궤적 중복 제거 |
| [05_exploration_value.png](05_exploration_value.png) | 가치 점수와 후보 채택/제외 이유 |
| [06_control_sequence_seeds.png](06_control_sequence_seeds.png) | 실제 계산한 seed 궤적과 세 축 제어입력열 |
| [07_parallel_mppi_sampling.png](07_parallel_mppi_sampling.png) | 실제 500개 rollout과 guide별 대표 |
| [08_common_recheck_and_selection.png](08_common_recheck_and_selection.png) | 최종 공통 MC 재평가 및 선택 |
| [09_all_steps.png](09_all_steps.png) | 전체 그림 모아보기 |

## 1. 입력과 정렬

- 원본: `{data['bag']}`. record timestamp `{p['snap']['record_ns']}` ns.
- `/scan_front_carto`, `/scan_rear_carto`는 기록된 측정 range와 beam timing을 그대로 사용한다. 유효한 finite return만 사용하며 inf/NaN을 free로 채우지 않는다.
- 해당 snapshot까지 수신된 `/odometry/filtered`만 사용한다. 각 beam의 `header + index*time_increment`에 odometry를 보간하고 기록 TF로 sensor→base→odom→현재 base를 변환한다. 마지막 odometry 시각 뒤 beam은 버린다. odometry의 첫/끝 밖 값을 외삽해 살리지 않는다.
- 현재 속도 `[vx,vy,yaw_rate] = {p['current'].round(4).tolist()}`. 기준 pose epoch는 `{p['epoch']:.9f}` s이다.
- 기록 `/global_path`는 당시 map→odom TF로 정렬하고 가장 가까운 waypoint부터 최대 80점을 사용한다.
- 기록 `/dra_mppi/reference_trajectory`의 몸체 속도를 현재 horizon 시각으로 보간하고 운동 제약을 다시 적용한다. **이 nominal은 당시 planner 내부 warm start를 복원한 값이 아니다.** 저장된 외부 reference를 이용한 재구성이다. 기록 horizon 끝 뒤는 마지막 제어를 유지한다.

## 2. gap 정의

각 scan에서 연속 유효 beam 두 개 사이를 자유공간으로 rasterize한다. 두 range 차이가 0.25 m보다 크면 가림 경계로 보고 연결하지 않는다. 연결할 때 두 range 중 작은 값에서 0.04 m를 빼므로 장애물 뒤를 채우지 않는다. 8×8 m, 해상도 0.04 m grid에 finite hit를 장애물로 넣는다.

앞·뒤 센서의 원점과 마스킹 섹터가 다르기 때문에 가까운 측면에 관측 틈이 생긴다. 그 틈을 임의로 free로 만들지 않고 **기록 `/traversable_map_grid`의 알려진 free cell로 보완**한다. 지도 age는 {p['map_age']:.3f} s이며 1초 초과면 예제를 거부한다. LiDAR 또는 지도에서 occupied인 cell은 occupied가 우선이다. 흰색=scan free, 연파랑=map 보완 free, 회색=unknown이다. 이는 순수 scan-only 버전과 구별되는 설계 선택이다.

EDT에서 cell 대각선을 빼고, 로봇 반경 0.20 m + 탐색 여유 0.05 m + ray 간격 padding 0.01 m를 적용한다. 방향은 1°, 거리는 0.02 m 간격으로 검사한다. 시작점부터 계속 통과되는 최대 거리가 `d_free(θ)`다. 중간 장애물 뒤의 free는 연결하지 않는다.

`d_free(θ) >= L`인 연속 각도 구간을 gap으로 정의한다. L은 0.5, 1.0, 1.5 m, 최소 폭은 6°이며 ±180° 경계도 연결한다. 중심 방향 차이가 12° 미만이고 각도 구간이 겹치는 거리별 후보는 먼 거리 후보부터 병합한다. 이 gap은 로봇 중심의 관측 가능한 접근 방향이며 전역 homotopy class나 장애물 뒤 연결성을 뜻하지 않는다.

## 3. nominal 중복 제거

각 gap 중앙 목표로 제약을 지키는 seed를 먼저 만든다. 따라서 flow의 04~05단계에서도 저렴한 seed rollout은 필요하고, 06단계는 채택된 seed를 확정·표시하는 단계다.

같은 미래 시각의 seed와 nominal 위치를 비교해 RMS<0.22 m **그리고** 마지막 3 step 평균 거리<0.30 m이면 그 대표 seed를 중복으로 처리한다. nominal의 swept collision이 있으면 이 제거를 해제한다. 넓은 gap 전체를 탐색 불필요로 선언하는 것은 아니다. 이 거리 기준은 초기 실험값이며 실제 noise 분산으로 보정해야 한다.

## 4. 탐색 가치

먼저 seed swept collision 및 horizon 끝의 목표 오차>0.35 m를 제외한다. 남은 후보는 아래 명시적 점수로 순위를 매긴다.

`V = 0.40 P + 0.25 C + 0.20 D + 0.15 R`

- P = clip(초기 local path 방향으로의 종단 이동 / 1.5 m, 0, 1).
- C = clip(seed의 최소 swept footprint 여유 / 0.4 m, 0, 1).
- D = clip(nominal과의 위치 RMS / 0.8 m, 0, 1).
- R = exp(-종단점의 기록 path waypoint까지 최소 거리 / 0.6 m).

V≥0.25인 후보를 높은 순서로 검토한다. 이미 채택한 gap seed와 RMS<0.25 m이면 중복으로 제외하며 최대 3개를 남긴다. P/R은 저렴한 근사이며 굽은 경로에서는 선분 투영·arc progress로 개선할 수 있다. 이 점수는 충돌확률이 아니다. 이 단일 예제에서는 시간 지속성/동적 위험의 사전 점수를 만들지 않았고, 기록 person 예측을 이용한 기존 MC를 모든 MPPI rollout과 최종 대표에 적용했다.

| gap | 중앙 각도 | L m | 각도 폭 | nominal RMS m | 목표 오차 m | V | 판정 |
|---|---:|---:|---:|---:|---:|---:|---|
{rows}

## 5. seed와 MPPI

병진 속도는 gap 중앙을 향하며 목표까지의 거리/0.6초와 원하는 속도 0.75 m/s 중 작은 값으로 정한다. yaw는 global path 접선을 따른다. 매 step 기존 `_constrain_controls`로 축별 속도·가속도·wheel L1 제약을 적용한다.

기존 `GuideConditionedOptimizer`를 offline subclass로 감싸 `anchors()`와 `allocate()`만 바꿨다. `{opt['names']}`에 `{opt['counts']}`개, 합계 500개를 배정한다. 기존 Halton spline noise, 2 workers, guide별 weighted update, 정적 swept 검사, 8배 MC density와 최종 공통 재평가는 유지한다. 다른 gap의 제어열끼리 평균하지 않는다.

`/dra_mppi/predictions_used`의 기록 객체를 현재 노드의 `_aligned_predictions`로 다시 age/filter/시간보간/좌표변환했다. 이번 계산에 사용된 예측 객체는 {0 if opt['predictions'] is None else len(opt['predictions'].means)}개다. 기록에서 stationary safety-disc로 표현된 객체는 움직이는 보행자로 새로 꾸미지 않았다. scan hit를 snapshot 동안 정적 장애물로 쓰므로 사람이 scan에도 있으면 정적·동적 비용에 동시에 반영될 수 있다.

## 6. 최종 선택 결과

| 후보 | 최종 비용 | step 최대 충돌확률 |
|---|---:|---:|
{final}

최종 선택은 **{r.selected_mode}**, 상태는 `{r.safety_status}`다. 기존 정책대로 안전한 이동 후보가 있으면 Stop보다 우선한다. gap의 목적은 대안 탐색이지 새 후보의 강제 선택이 아니다. 이 결과를 실제로 로봇이 실행한 궤적이나 기존 방식 대비 성능 향상으로 해석하면 안 된다.

## 7. 5 Hz 적용 계획과 실제 측정 범위

5 Hz는 200 ms 주기다. 운영 목표는 총 rollout 500 고정, gap 최대 3, worker 2 유지다. scan 전처리/EDT는 최신 입력 cache로 분리하고, cycle에는 고정 snapshot과 추출한 gap을 넘기는 구성이 적절하다. gap 변경이 작으면 ID·seed를 유지하고 여유 시간이 부족하면 다음 cycle의 gap 수부터 줄인다. MC 밀도·최종 재검사를 줄여 시간을 맞추지는 않는다.

이번 **단일 cold offline 실행**의 참고 wall time:

- 입력 정렬·scan/map grid·거리 계산: {opt['prepare_ms']:.2f} ms.
- 후보 seed·중복·점수·선정: {opt['proposal_ms']:.2f} ms.
- 기존 MPPI sampling/MC/최종 재검사: {opt['optimizer_ms']:.2f} ms.

bag I/O, PNG rendering, ROS TF 조회/publish, 실제 perception/MPC 동시 부하는 포함하지 않는다. 최초 sampler/controller 준비 시간도 일부 proposal 시간에 들어 있다. **p99나 5 Hz 보장 값이 아니다.** 그림 생성을 완료하려고 offline optimizer timeout은 10초로 두었고 운영 0.18초 timeout과 구분했다. 기존 timeout은 실행 중인 Python worker를 즉시 중단하지 못한다.

실시간 통합 전에는 gap의 odom 기준 ID 매칭, split/merge 처리, 상태별 hysteresis, path/odom/prediction freshness gate, gap 추출 실패 시 기존 fixed-guide fallback을 추가해야 한다. map/prediction 등 필수 안전 입력 자체가 stale이면 기존 hold 정책을 따라야 한다. `G1` 등의 이름은 이 snapshot 안의 번호이며 시간 연속 ID가 아니다.

## 재현

저장소 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 \\
  planner/local_planner/DRA_MPPI/tools/render_lidar_gap_flow.py \\
  --bag {data['bag']} \\
  --offset {p['snap']['elapsed']:.6f}
```

`--inspect`는 같은 bag의 2초 간격 후보 장면을 검사한다. 그림 설명에 적합하도록 중복 제거와 도달성 제외가 함께 나타나는 장면을 선택했으며, 성공률 평가용 무작위 표본은 아니다.

`manifest.json`에 입력 시각·TF 정렬 기준·설정·후보별 수치·최종 비용·소스 해시를 기록했다. `snapshot_and_results.npz`에는 사용한 실제 scan range/유효 mask/정렬된 point, grid, nominal, 500개 제어열과 최종 결과를 저장했다. 원본 bag 재생 없이도 그림의 수치 검토가 가능하다. ZIP에는 PNG·보고서·수치와 생성 스크립트가 함께 들어 있다. 스크립트 실행에는 이 저장소의 planner 모듈과 ROS 환경이 필요하다.
'''
    (out/'README.md').write_text(report,encoding='utf-8')

"""Illustrate the production gap strategy with one recorded dual-LiDAR scene.

Offline only: cached deskewed beams/map/nominal, no ROS publishing. Plots use
captured production seeds, decisions, sampled controls and representatives.
"""
from dataclasses import asdict, fields, replace
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Polygon

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.gap_geometry import GapConfig, partition, runs
from dra_mppi.gap_inputs import GapScan, directional_depth, fuse_scans
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiController


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'docs/gap_multiscene_2026-11-04'
COLORS = ['#c47b16', '#3383ba', '#009c91', '#7c55b5', '#d75a70', '#678225']
INK = '#20364b'
MUTED = '#536b7e'
NOMINAL = '#172b44'
REASONS = {'selected': '선택', 'duplicate_nominal': 'nominal 중복',
           'budget_limit': '3개 예산 초과', 'duplicate_selected': 'gap끼리 중복',
           'static_collision': '정적 충돌', 'target_error': '도달 오차 초과',
           'insufficient_motion': '이동량 부족', 'nonfinite_cost': '비용 무효'}


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default='b2_100')
    parser.add_argument('--output', type=Path, default=ROOT / 'docs/gap_implementation_flow_2026-11-05')
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font).get_name(),
                         'axes.unicode_minus': False, 'font.size': 12, 'text.color': INK,
                         'axes.labelcolor': MUTED, 'axes.titlecolor': INK,
                         'xtick.color': MUTED, 'ytick.color': MUTED})
    p = yaml.safe_load((ROOT / 'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    gap = GapConfig(**{f.name: p['gap.'+f.name] for f in fields(GapConfig) if 'gap.'+f.name in p})
    cfg = replace(config_from_yaml(), strategy='gap', gap=gap)
    cache = BASE / f'disparity_partition_10_scenes/data/{args.scene}.npz'
    seed_file = BASE / f'midpoint_value_v1/{args.scene}_seeds.npz'
    with np.load(cache) as f:
        data = dict(f)
    with np.load(seed_file) as f:
        nominal = f['nominal_controls'].copy()
    audit = next(json.loads(line) for line in (BASE / 'scene_audit.jsonl').read_text().splitlines()
                 if json.loads(line)['scene'] == args.scene)
    old_parts = next(r for r in json.loads((BASE / 'disparity_partition_10_scenes/results.json').read_text())
                     if r['scene'] == args.scene)
    scans = tuple(GapScan(*(data[f'scan_{i}_{k}'] for k in ('ranges', 'valid', 'origins', 'hits')))
                  for i in range(2))
    state = np.zeros(3)
    current = np.array(audit['current'])
    path = data['full_path']
    field = DistanceField.from_occupancy(data['grid'], GridMetadata(200, 200, .04, -4., -4., 0.))
    fused, transform, local = fuse_scans(scans, state, field, None, gap, cfg.static_cost_robot_radius)
    angles, strict, hard = directional_depth(fused, gap, cfg.static_cost_robot_radius)
    arrays = dict(strict_depth=strict, hard_depth=hard)
    for i, scan in enumerate(local):
        arrays.update({f'scan_{i}_{k}': scan[k] for k in ('ranges', 'valid', 'hits')})
    parts = partition(arrays, asdict(gap))
    assert [r['bins'] for r in parts['regions']] == [r['bins'] for r in old_parts['regions']]
    depth = hard if parts['fallback'] else strict
    controller = MppiController(cfg)
    capture = []
    optimizer = controller._guide_optimizer
    original_seed = optimizer._seed

    def capture_seed(state, path, current, target):
        controls = original_seed(state, path, current, target)
        capture.append((target.copy(), controls.copy()))
        return controls

    optimizer._seed = capture_seed
    optimizer._warm_nominal = nominal.copy()
    try:
        result = controller.compute(state, path, current, distance_field=field,
                                    gap_scans=scans, visualization_rollouts=500)
        next_nominal = optimizer._warm_nominal.copy()
    finally:
        controller.close()
    assert len(capture) == 2*parts['region_count']
    seeds = [capture[2*i+1] for i in range(parts['region_count'])]
    seed_controls = np.array([s[1] for s in seeds])
    seed_paths = rollout(state, seed_controls, cfg.dt)
    nominal_path = rollout(state, nominal, cfg.dt)
    sampled_paths = rollout(state, result.sampled_controls, cfg.dt)
    candidates = result.gap_diagnostics['candidates']
    selected = result.gap_diagnostics['selected']
    selected_ids = [int(name.split('_')[1]) for name in selected]
    assert sum(result.guide_rollout_counts) == 500
    assert result.safety_status == 'ok'
    np.testing.assert_allclose(next_nominal, np.vstack((result.control_sequence[1:], result.control_sequence[-1])))
    seed_collision = fused.trajectory_collisions(seed_paths[:, :, :2], state[:2],
                                                cfg.static_cost_robot_radius, source_to_grid=transform)
    source = f"{audit['bag']}  +{audit['offset']:.3f}s  |  {args.scene}"
    footer = '실제 기록의 시간 보정 scan·지도·nominal 재사용 | 동적 예측 미주입 | 실주행·전체 5Hz 검증 아님'
    files = []

    def canvas(number, title, subtitle):
        fig = plt.figure(figsize=(15, 9), facecolor='#f6f8fb')
        fig.text(.045, .946, f'{number:02d}  {title}', fontsize=24, weight='bold')
        fig.text(.045, .90, subtitle, fontsize=12, color=MUTED)
        fig.text(.045, .055, source, fontsize=10, color=MUTED)
        fig.text(.045, .025, footer, fontsize=10, color=MUTED)
        return fig

    def save(fig, filename):
        fig.savefig(out / filename, dpi=150, facecolor=fig.get_facecolor())
        plt.close(fig)
        files.append(filename)

    def note(fig, x, y, title, body, color=INK):
        fig.text(x, y, title, fontsize=17, weight='bold', color=color, va='top')
        fig.text(x, y-.046, body, fontsize=13, color=INK, va='top', linespacing=1.65)

    def map_ax(fig, rect=(.055, .17, .49, .66), raw=False, limit=2.4):
        ax = fig.add_axes(rect)
        if not raw:
            grid = fused.occupancy
            view = np.zeros_like(grid)
            view[grid < 0] = 1
            view[data['map_fill'] & (grid == 0)] = 2
            view[grid >= 65] = 3
            ax.imshow(view, extent=(-4, 4, -4, 4), origin='lower', vmin=0, vmax=3,
                      cmap=ListedColormap(['#ffffff', '#d8dfe7', '#e5eefb', '#34495e']),
                      interpolation='nearest', zorder=0)
        for i, col in enumerate(('#df8646', '#468dc4')):
            hit = data[f'scan_{i}_hits'][data[f'scan_{i}_valid']]
            ax.scatter(hit[:, 0], hit[:, 1], s=2.5, color=col, alpha=.5, zorder=2,
                       label='전방 LiDAR' if i == 0 else '후방 LiDAR')
        ax.add_patch(Circle((0, 0), cfg.static_cost_robot_radius, fc=INK, ec='white', zorder=9))
        ax.annotate('', (.37, 0), (0, 0), arrowprops=dict(arrowstyle='->', color=INK, lw=2), zorder=10)
        ax.set(xlim=(-limit, limit), ylim=(-limit, limit), aspect='equal',
               xlabel='x [m]  ·  전방 + / 후방 −', ylabel='y [m]  ·  좌측 + / 우측 −')
        ax.grid(alpha=.14)
        return ax

    def path_line(ax, points, **kwargs):
        xy = np.vstack((state[:2], points[:, :2]))
        ax.plot(xy[:, 0], xy[:, 1], **kwargs)

    def backdrop_paths(ax):
        ax.plot(path[:, 0], path[:, 1], '--', color='#9babb7', lw=1.5, label='Global path')
        path_line(ax, nominal_path, color=NOMINAL, lw=2.5, ls='--', label='nominal', zorder=7)

    def draw_regions(ax, labels=True, alpha=.20):
        for region in parts['regions']:
            ids = np.array(region['bins'])
            xy = depth[ids, None]*np.column_stack((np.cos(angles[ids]), np.sin(angles[ids])))
            color = COLORS[(region['id']-1) % len(COLORS)]
            ax.add_patch(Polygon(np.vstack(([0, 0], xy)), fc=color, ec='none', alpha=alpha, zorder=3))
            ax.plot(xy[:, 0], xy[:, 1], color=color, lw=2, zorder=4)
            for index in (ids[0], ids[-1]):
                end = depth[index]*np.array([np.cos(angles[index]), np.sin(angles[index])])
                ax.plot([0, end[0]], [0, end[1]], color=color, lw=1, alpha=.7)
            if labels:
                k = ids[len(ids)//2]
                point = max(.60, depth[k]*.80)*np.array([np.cos(angles[k]), np.sin(angles[k])])
                ax.text(*point, f'G{region["id"]}', color=color, fontsize=13, weight='bold', ha='center',
                        bbox=dict(fc='white', ec='none', alpha=.85, pad=1.5), zorder=12)

    def mode_color(name):
        return NOMINAL if name == 'nominal' else '#8d959d' if name == 'stop' else COLORS[int(name.split('_')[1])-1]

    # 01: actual measured geometry, not an invented room.
    fig = canvas(1, '입력 확인 · 실제 전·후방 LiDAR', '두 센서의 시간 보정된 hit를 현재 로봇 좌표계에서 표시')
    ax = map_ax(fig, raw=True, limit=3.5)
    ax.legend(loc='upper right', fontsize=10)
    counts = [int(np.sum(data[f'scan_{i}_valid'])) for i in range(2)]
    note(fig, .59, .81, '이 장면의 실제 입력',
         f'전방: {counts[0]:,} / {len(scans[0].ranges):,} beam 사용\n후방: {counts[1]:,} / {len(scans[1].ranges):,} beam 사용\n좌표 원점: 현재 로봇 위치\n화살표 +x: 차체 정면')
    note(fig, .59, .55, '온라인 구현에서 먼저 확인하는 조건',
         'scan 원본·수신 나이 ≤ 0.25초\n전후방 scan 시각 차이 ≤ 0.05초\nodometry 보간과 센서 TF로 beam 위치 보정\n누락·만료·보정 실패 → 해당 주기 hold')
    note(fig, .59, .28, '이 그림의 재생 범위',
         f'저장된 시간 보정 좌표를 재사용했습니다.\n원본 장면 audit의 예측 객체 수: {audit["prediction_count"]}\n이번 실행에는 동적 예측을 주입하지 않았습니다.', MUTED)
    save(fig, '01_actual_lidar.png')

    # 02: local free space and radial body reach on the same axes.
    fig = canvas(2, '이동 가능한 공간 계산', '장애물까지의 거리에서 로봇 몸체·여유를 고려하여 방향별 진입 깊이를 계산')
    ax = map_ax(fig)
    supported = runs(depth >= gap.minimum_depth_m)
    for ids in supported:
        xy = depth[ids, None]*np.column_stack((np.cos(angles[ids]), np.sin(angles[ids])))
        ax.add_patch(Polygon(np.vstack(([0, 0], xy)), fc='#25aaa0', alpha=.25, ec='none', zorder=3))
        ax.plot(xy[:, 0], xy[:, 1], color='#008f85', lw=2)
    ax.add_patch(Circle((0, 0), .25, fill=False, ec='#d24a53', ls='--', lw=1.5, zorder=11))
    note(fig, .59, .81, '지도와 라이다를 함께 사용',
         '흰색: scan 기반 free\n연파랑: 저장된 지도 보완 free\n진한색: 장애물  /  회색: unknown\nunknown은 이동 불가로 취급')
    note(fig, .59, .54, '초록 외곽 = 로봇 중심의 진입 깊이',
         '로봇 반경 20cm + 추가 여유 5cm\n360방향 · 1° 간격 · 최대 2m\n거리 확인 간격 2cm\n깊이 0.5m 이상인 방향만 열린 공간')
    note(fig, .59, .27, f'이 장면: 열린 연결 구간 {parts["open_runs"]}개',
         f'5cm 여유 제거 재시도: {"사용" if parts["fallback"] else "사용 안 함"}\n아직 raw 거리 급변으로 추가 분할하기 전입니다.')
    save(fig, '02_reachable_space.png')

    # 03: cuts are supported by raw measured discontinuities.
    fig = canvas(3, '거리 급변에서 gap 분할', f'열린 구간 {parts["open_runs"]}개 + 내부 급변 경계 {parts["selected_cut_count"]}개 → gap {parts["region_count"]}개')
    ax = map_ax(fig, rect=(.055, .18, .46, .64))
    draw_regions(ax)
    right = fig.add_axes([.59, .54, .36, .28])
    degrees = np.rad2deg(angles)
    right.plot(degrees, depth, color='#97a7b4', lw=1)
    for r in parts['regions']:
        ids = np.array(r['bins'])
        right.scatter(degrees[ids], depth[ids], s=6, color=COLORS[r['id']-1])
    right.axhline(.5, color='#9a5059', ls='--', lw=1)
    for e in parts['edges']:
        if e['status'] == 'selected':
            right.axvline(e['cut_bin']-180, color=INK, ls=':', lw=1.4)
    right.set(xlabel='로봇 기준 각도 [°]', ylabel='진입 깊이 [m]', xlim=(-180, 180), ylim=(0, 2.2))
    right.set_xticks([-180, -90, 0, 90, 180])
    right.grid(alpha=.15)
    actual_edges = [e for e in parts['edges'] if e['status'] == 'selected']
    edge_lines = [f"후방 beam {e['beam_before']}: {e['range_before']:.2f} → {e['range_after']:.2f}m"
                  for e in actual_edges]
    note(fig, .59, .43, '선택된 경계를 만든 실제 거리 변화', '\n'.join(edge_lines))
    note(fig, .59, .22, '경계 검증',
         '인접 거리 급변 + 양쪽 3 beam의 지속성\n몸체 진입 깊이 차이 ≥ 0.25m\n경계 간격 ≥ 20° · 구역 폭 ≥ 6°')
    save(fig, '03_disparity_gaps.png')

    # 04: captured seed outputs include rejected proposals.
    fig = canvas(4, '각 gap 중앙에 제어열 1개 생성', '색 실선: 실제 중앙 seed rollout  |  ×: 목표점  |  점선: 기존 nominal')
    ax = map_ax(fig)
    draw_regions(ax, labels=False, alpha=.08)
    backdrop_paths(ax)
    for i, (target, _) in enumerate(seeds):
        path_line(ax, seed_paths[i], color=COLORS[i], lw=2, zorder=7)
        ax.scatter(*target, marker='x', color=COLORS[i], s=65, zorder=8)
        ax.annotate(f'G{i+1}', target, xytext=(5, 6), textcoords='offset points', color=COLORS[i], weight='bold')
    ax.legend(loc='upper right', fontsize=9)
    rows = [[f'G{i+1}', f"{c['angle_deg']:.0f}°", f'{np.linalg.norm(seeds[i][0]):.2f}m',
             f"{seed_paths[i,-1,0]:+.2f}, {seed_paths[i,-1,1]:+.2f}"] for i, c in enumerate(candidates)]
    tax = fig.add_axes([.58, .46, .38, .35]); tax.axis('off')
    table = tax.table(cellText=rows, colLabels=['gap', '중앙 각도', '목표 거리', '2초 후 XY [m]'],
                      cellLoc='center', loc='center', colWidths=[.14, .21, .24, .41])
    table.auto_set_font_size(False); table.set_fontsize(11); table.scale(1, 1.85)
    for i in range(len(rows)):
        table[i+1, 0].get_text().set_color(COLORS[i])
    note(fig, .59, .38, '병진 방향과 차체 yaw는 별도',
         '병진: gap의 각도 중앙을 향함\nyaw: Global path의 방향을 따름\n0.2초 × 10 step, 속도·가속도·바퀴 제약 적용')
    note(fig, .59, .18, '목표점까지 무조건 2m를 보내지 않습니다',
         '몸체 진입 깊이와 2초 내 도달 거리를 함께 반영합니다.', MUTED)
    save(fig, '04_midpoint_seeds.png')

    # 05: all actual gate values and ranking numbers.
    fig = canvas(5, '탐색 가치 검사 · 중복 제거 · 최대 3개 선택', '도달 가능성 → nominal과 다른가? → 후보가 많으면 기존 비용으로 순위')
    tax = fig.add_axes([.045, .43, .91, .39]); tax.axis('off')
    rows = []
    for i, c in enumerate(candidates):
        rows.append([f'G{i+1}', '충돌' if seed_collision[i] else '없음',
                     f"{c['target_error_m']:.3f}", f"{c['projected_displacement_m']:.3f}",
                     f"{c['nominal_rms_m']:.3f}", f"{c['nominal_tail_m']:.3f}",
                     '미계산' if c['cost'] is None else f"{c['cost']:.2f}", REASONS[c['reason']]])
    table = tax.table(cellText=rows, colLabels=['gap', '정적 충돌', '목표 오차\n≤0.35m', 'gap 방향 이동\n≥0.25m',
                        'nominal RMS\n<0.22m?', '마지막 3점\n<0.30m?', '사전 비용', '최종 판단'],
                      cellLoc='center', loc='center', colWidths=[.055, .09, .125, .135, .13, .13, .105, .23])
    table.auto_set_font_size(False); table.set_fontsize(12); table.scale(1, 2.6)
    for (i, j), cell in table.get_celld().items():
        cell.set_edgecolor('#d4dde6')
        if i == 0:
            cell.set_facecolor('#e4ebf3'); cell.get_text().set_weight('bold'); cell.set_height(.19)
        else:
            cell.set_facecolor('#eaf5ef' if candidates[i-1]['reason'] == 'selected' else '#f5eee9')
            if j == 0: cell.get_text().set_color(COLORS[i-1])
    duplicate = [c for c in candidates if c['reason'] == 'duplicate_nominal']
    chosen_text = ' · '.join(f'G{i}' for i in selected_ids)
    note(fig, .055, .34, '1. 이동 가능성',
         '정적 충돌 없음 + 목표 오차 기준 통과\n+ 해당 gap 방향으로 25cm 이상 이동\n후진도 해당 방향 내적으로 검사')
    note(fig, .37, .34, '2. 기존 탐색과 중복인가?',
         'RMS < 0.22m AND 끝 3점 < 0.30m\n두 조건을 동시에 만족하면 제외\n이 장면 제외: '+', '.join(c['id'].replace('gap_0', 'G') for c in duplicate))
    note(fig, .69, .34, '3. 남은 후보의 비용 비교',
         '기존 공통 비용 + 정적 soft cost\n최대 3개만 유지, 동적 MC는 추가 안 함\n최종 선택: '+chosen_text)
    fig.text(.055, .125, '이 표의 값은 모두 실제 생성 seed에서 계산했습니다. 후진 후보도 남지만, 예산 경쟁에서는 경로 비용의 영향을 받습니다.',
             fontsize=12, color=MUTED)
    save(fig, '05_candidate_decisions.png')

    # 06: actual complete 500 control rollouts, including invalid samples.
    fig = canvas(6, '500개를 배분하고 그룹별 MPPI 실행', '표시한 선은 실제 생성된 전체 rollout입니다. 충돌 등으로 나중에 제외되는 표본도 포함합니다.')
    ax = map_ax(fig)
    for i, name in enumerate(result.candidate_names[:-1]):
        trajectories = sampled_paths[result.control_mode_ids == i]
        for trajectory in trajectories:
            path_line(ax, trajectory, color=mode_color(name), lw=.65, alpha=.13, zorder=4)
        path_line(ax, result.guide_anchors[i], color=mode_color(name), lw=2.7, zorder=7, label=name)
    ax.legend(loc='upper right', fontsize=10)
    bax = fig.add_axes([.64, .56, .29, .23])
    names = list(result.candidate_names[:-1])
    bax.barh(names[::-1], result.guide_rollout_counts[::-1], color=[mode_color(n) for n in names[::-1]])
    bax.set(xlim=(0, max(result.guide_rollout_counts)*1.25), xlabel='rollout 개수')
    for i, n in enumerate(result.guide_rollout_counts[::-1]):
        bax.text(n+3, i, str(n), va='center', fontsize=13)
    note(fig, .59, .46, f'총 {sum(result.guide_rollout_counts)}개 유지',
         f'nominal: {result.guide_rollout_counts[0]}개\n선택 gap: 각 {gap.rollouts_per_guide}개\n중앙 seed 주변에 Halton spline noise 추가')
    note(fig, .59, .25, '그룹마다 대표 제어열 1개로 압축',
         '기존 비용 평가 → MPPI 가중평균\nseed의 중앙이나 gap 내부에 고정하지 않음\n이 장면은 동적 예측이 없어 동적 위험 계산 생략')
    save(fig, '06_budget_and_rollouts.png')

    # 07: final representatives and stop, with actual final costs/status.
    fig = canvas(7, '대표 궤적 재검사 → 최종 선택', f'이번 실제 재생의 선택: {result.selected_mode}  |  safety_status: {result.safety_status}')
    ax = map_ax(fig)
    for i, name in enumerate(result.candidate_names):
        path_line(ax, result.mode_trajectories[i], color=mode_color(name), lw=2,
                  ls=':' if name == 'stop' else '--', label=name, zorder=6)
    path_line(ax, result.trajectory, color='#e54836', lw=3.6, label='최종 선택', zorder=8)
    ax.legend(loc='upper right', fontsize=10)
    tax = fig.add_axes([.58, .52, .38, .29]); tax.axis('off')
    rows = [[n, '무효' if not np.isfinite(result.mode_final_costs[i]) else f'{result.mode_final_costs[i]:.3f}',
             '충돌' if result.final_static_collisions[i] else '없음',
             '선택' if n == result.selected_mode else '대안' if n == 'stop' else '미선택']
            for i, n in enumerate(result.candidate_names)]
    table = tax.table(cellText=rows, colLabels=['대표', '최종 비용', '정적 충돌', '결과'],
                      cellLoc='center', loc='center', colWidths=[.34, .23, .23, .20])
    table.auto_set_font_size(False); table.set_fontsize(12); table.scale(1, 1.9)
    note(fig, .59, .43, '선택 규칙',
         '유효한 이동 대표 중 최저 비용 선택\n이동 대표가 모두 탈락하면 감속 Stop\nStop도 불가하면 안전 보장 없는 hold')
    note(fig, .59, .22, '동적 재검사의 범위',
         '운영 설정: sampling MC / 최종 Gaussian disk\n이 장면: 예측 미주입 → 동적 안전 검증 미실행\ngap 생성·탐색과 최종 선택은 서로 다른 단계', MUTED)
    save(fig, '07_final_selection.png')

    # 08: actual shifted controls, not a fabricated next physical observation.
    fig = canvas(8, 'MPC로 전달하고 다음 nominal 준비', '실제로 선택된 제어열을 한 step 이동합니다. 다음 센서 장면을 재생한 그림은 아닙니다.')
    for j, (label, unit) in enumerate((('vx', 'm/s'), ('vy', 'm/s'), ('yaw rate', 'rad/s'))):
        ax = fig.add_axes([.075, .64-j*.23, .46, .17])
        ax.plot(np.arange(1, 11), result.control_sequence[:, j], 'o-', ms=4, color=NOMINAL, label='이번 선택 제어열')
        ax.plot(np.arange(1, 11), next_nominal[:, j], 's--', ms=3, color='#008f85', label='다음 nominal')
        ax.set(xlim=(.8, 10.2), ylabel=f'{label} [{unit}]', xticks=np.arange(1, 11))
        ax.grid(alpha=.2)
        if j == 0: ax.legend(fontsize=9, loc='best')
        if j == 2: ax.set_xlabel('제어열 step (각 0.2초)')
    note(fig, .60, .80, '이번 주기 출력',
         '선택한 pose·velocity reference 10 step\n→ 하위 MPC가 추종\n첫 제어 입력 [vx, vy, ω]:\n'+np.array2string(result.control, precision=4))
    note(fig, .60, .52, '다음 nominal',
         '[u2, u3, …, u10, u10]\n직전 최종 선택 결과를 이어서 탐색\n새 scan에서 gap은 다시 분할\ngap 번호별 과거 제어열은 이어 붙이지 않음')
    note(fig, .60, .25, '실패와 빈 gap은 다르게 처리',
         '유효 입력 + gap 0개: nominal 500개\n입력 오류·계산 시간 초과: hold\n0.18초 제한은 전체 실행시간 보장이 아님')
    save(fig, '08_output_and_next_nominal.png')

    # A readable overview of the same actual scene; detailed panels are separate.
    fig = plt.figure(figsize=(18, 11), facecolor='#f6f8fb')
    fig.text(.035, .95, '구현된 GAP 전략 · 실제 LiDAR 한 장면의 전체 흐름', fontsize=25, weight='bold')
    titles = ['① 전·후방 scan', f'② 열린 공간 {parts["open_runs"]}개', f'③ gap {parts["region_count"]}개',
              '④ gap 중앙 seed', '⑤ 후보 판정', '⑥ 총 500개 sampling', '⑦ 최종 대표 선택', '⑧ 다음 주기 연결']
    subtitles = ['시간 보정 좌표 재사용', '로봇 반경 + 5cm 여유', 'raw 급변 + 이동 공간 변화',
                 '2초 · 10 step 제어열', '중복 제거 + 비용으로 3개', 'nominal 200 + gap 100씩',
                 f'선택: {result.selected_mode}', '선택 제어열을 1 step 이동']
    for k in range(8):
        col, row = k % 4, k // 4
        rect = (.045+col*.245, .53-row*.40, .205, .29)
        ax = map_ax(fig, rect=rect, raw=(k == 0))
        ax.set_xlabel(''); ax.set_ylabel(''); ax.tick_params(labelsize=8)
        ax.set_title(titles[k], fontsize=15, weight='bold', pad=10)
        if k == 1:
            for ids in supported:
                xy = depth[ids, None]*np.column_stack((np.cos(angles[ids]), np.sin(angles[ids])))
                ax.add_patch(Polygon(np.vstack(([0, 0], xy)), fc='#25aaa0', alpha=.3))
        if k in (2, 3, 4): draw_regions(ax, labels=(k == 2), alpha=.13)
        if k == 3:
            for i, tr in enumerate(seed_paths): path_line(ax, tr, color=COLORS[i], lw=1.7)
        if k == 4:
            path_line(ax, nominal_path, color=NOMINAL, lw=2, ls='--')
            for i in selected_ids: path_line(ax, seed_paths[i-1], color=COLORS[i-1], lw=2)
        if k == 5:
            for i, name in enumerate(result.candidate_names[:-1]):
                for tr in sampled_paths[result.control_mode_ids == i]:
                    path_line(ax, tr, color=mode_color(name), alpha=.11, lw=.5)
        if k in (6, 7):
            for i, name in enumerate(result.candidate_names[:-1]):
                path_line(ax, result.mode_trajectories[i], color=mode_color(name), ls='--', lw=1)
            path_line(ax, result.trajectory, color='#e54836', lw=2.6)
        if k == 7:
            ax.text(.5, .04, '[u2, …, u10, u10]', transform=ax.transAxes, ha='center', fontsize=11,
                    bbox=dict(fc='white', ec='none', alpha=.9))
        fig.text(rect[0]+rect[2]/2, rect[1]-.025, subtitles[k], ha='center', fontsize=11, color=MUTED)
    fig.text(.035, .06, source, fontsize=12, color=MUTED)
    fig.text(.035, .03, footer, fontsize=11, color=MUTED)
    save(fig, '00_overall_flow.png')

    report = dict(scene=args.scene, source=audit['bag'], offset=audit['offset'],
                  scope=footer, recorded_prediction_count=audit['prediction_count'],
                  cache_sha256=hashlib.sha256(cache.read_bytes()).hexdigest(),
                  configuration=asdict(cfg), partition=parts, diagnostics=result.gap_diagnostics,
                  seed_targets=[s[0] for s in seeds], seed_static_collisions=seed_collision,
                  candidate_names=result.candidate_names, final_costs=result.mode_final_costs,
                  final_static_collisions=result.final_static_collisions,
                  final_collision_probabilities=result.mode_collision_probabilities,
                  selected_mode=result.selected_mode, selected_control=result.control,
                  safety_status=result.safety_status, figures=files)
    (out / 'results.json').write_text(json.dumps(clean(report), indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    np.savez_compressed(out / 'replay_arrays.npz', seed_controls=seed_controls, seed_paths=seed_paths,
                        nominal_controls=nominal, nominal_path=nominal_path,
                        sampled_controls=result.sampled_controls, sampled_paths=sampled_paths,
                        group_ids=result.control_mode_ids, representatives=result.mode_trajectories,
                        selected_controls=result.control_sequence, selected_path=result.trajectory,
                        next_nominal=next_nominal)
    (out / 'README.txt').write_text(
        '구현된 gap 전략: 실제 LiDAR 단계별 PNG (2026-11-05)\n\n'+source+'\n'+footer+'\n\n'+
        '\n'.join(files)+'\n\n'+
        '같은 장면과 같은 좌표계를 모든 단계에 사용. gap별 색은 고정.\n'
        '그림의 seed·선택 사유·500개 rollout·최종 대표는 현재 production 구현의 실제 출력.\n'
        'scan은 저장된 deskew 좌표, upstream map은 저장된 융합 grid를 재사용.\n'
        '온라인 ROS freshness/TF 검사는 재실행하지 않음. 동적 예측은 주입하지 않음.\n'
        '다음 nominal은 제어열 shift를 표시하며, 다음 센서 장면/폐루프 실주행 결과가 아님.\n'
        '그림 생성은 ROS 토픽/로봇 명령을 발행하지 않음.\n\n'
        '재현: ROS와 workspace setup을 source한 뒤 패키지 디렉터리에서\n'
        'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 tools/render_production_gap_flow.py\n')
    with zipfile.ZipFile(out / 'gap_flow_pngs.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for filename in files+['README.txt', 'results.json']:
            archive.write(out / filename, filename)
    print(json.dumps(clean(dict(output=str(out), figures=len(files), diagnostics=result.gap_diagnostics,
                               final_costs=result.mode_final_costs, selected=result.selected_mode)), ensure_ascii=False))


if __name__ == '__main__':
    main()

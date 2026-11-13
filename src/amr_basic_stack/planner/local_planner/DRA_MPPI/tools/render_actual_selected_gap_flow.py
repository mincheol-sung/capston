"""Actual-bag, step-by-step visualization of the chosen direction-cost method.

Source ROS Jazzy and the workspace before running. Uses the frozen experiment
planner; never starts a node or publishes control commands.
"""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Polygon
from PIL import Image
from scipy.ndimage import gaussian_filter1d

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'docs/gap_multiscene_2026-11-04'
OUT = BASE / 'actual_selected_flow_b1_088'
sys.path.insert(0, str(BASE / 'source_snapshot/tools'))
import experiment_gap_multiscene as ex
from experiment_gap_direction_refinement import make_pool

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family': font_manager.FontProperties(fname=FONT).get_name(),
                     'axes.unicode_minus': False, 'font.size': 12,
                     'figure.facecolor': '#f8fafc', 'savefig.facecolor': '#f8fafc',
                     'axes.spines.top': False, 'axes.spines.right': False})
INK, MUTED, NOM = '#16334e', '#53697e', '#2870c4'
COL = ['#9750b8', '#008c77']
LABELS = ['최대 깊이', '각도 중앙', '경로 방향']


def row(path, scene, method=None):
    for line in path.read_text().splitlines():
        r = json.loads(line)
        if r['scene'] == scene and (method is None or (r['method'] == method and r['seed'] == 7)):
            return r
    raise KeyError(scene)


def collect():
    audit = row(BASE/'scene_audit.jsonl', 'b1_088')
    previous = row(BASE/'refinement/trials.jsonl', 'b1_088', 'valley_direction_cost')
    print('Reading actual bag:', audit['bag'], flush=True)
    data = ex.read_bag(Path('/home/smoc/2026amr_ws/bags')/audit['bag'])
    snap = next(s for s in data['snapshots'] if s['record_ns'] == audit['record_ns'])
    p = ex.prepare_snapshot(data, snap)
    ex.input_check(data, snap, p)
    saved = np.load(BASE/'scenes/b1_088.npz')
    for k in ['grid', 'd_free', 'angles', 'full_path']:
        np.testing.assert_allclose(p[k], saved[k], rtol=0, atol=1e-10)
    cfg = ex.config_from_yaml()
    c = ex.MppiController(replace(cfg, strategy='standard'))
    c.gap_path = p['path']
    nominal = c._constrain_controls(p['nominal'][None], p['current'])[0]
    nominal_path = ex.rollout(np.zeros(3), nominal, .2)
    np.testing.assert_allclose(nominal_path, saved['nominal_path'], atol=1e-10)
    predictions = ex.recorded_predictions(p, c)
    nominal_collision = bool(p['field'].trajectory_collisions(nominal_path[None, :, :2], np.zeros(2), ex.RADIUS)[0])
    pool, _, _ = make_pool(ex, p, c, nominal_path, nominal_collision)
    nominal_cost = float(ex.cheap_cost(c, p, nominal[None], nominal_path[None])[0])
    items, chosen, _ = ex.select(p, pool, 'cost', nominal_collision, nominal_cost, True)
    chosen = sorted(chosen, key=lambda g:g['id'])
    definitions = ex.definitions(p, 'valley')
    options = []
    for gi, g in enumerate(items):
        for oi, recorded in enumerate(g['direction_options']):
            angle = np.deg2rad(recorded['angle_deg'])
            index = int(np.argmin(np.abs(p['angles']-angle)))
            unit = np.array([np.cos(angle), np.sin(angle)])
            length = max(.1, p['d_free'][index]-.1)
            far = ex.seed_control(c, length*unit, p['current'])
            reach = float(ex.rollout(np.zeros(3), far, .2)[-1, :2] @ unit)
            length = min(length, max(.25, .9*reach))
            controls = ex.seed_control(c, length*unit, p['current'])
            trajectory = ex.rollout(np.zeros(3), controls, .2)
            cost = float(ex.cheap_cost(c, p, controls[None], trajectory[None])[0])
            np.testing.assert_allclose(cost, recorded['cost'], atol=1e-10)
            delta = np.linalg.norm(trajectory[:, :2]-nominal_path[:, :2], axis=1)
            options.append(dict(**recorded, gap=gi, label=LABELS[oi], target=length*unit,
                                trajectory=trajectory, controls=controls, tail=float(np.mean(delta[-3:]))))
    run_cfg = replace(cfg, seed=7, guide_timeout=10., strategy='standard',
                      guide_angles_deg=tuple(float(i) for i in range(1+len(chosen))))
    solver = ex.MppiController(run_cfg)
    solver.config.strategy = 'guide_conditioned'
    names = ['nominal']+[g['id'] for g in chosen]
    counts = [500-100*len(chosen)]+[100]*len(chosen)
    solver._guide_optimizer = ex.SnapshotGapOptimizer(solver,
        np.array([nominal]+[g['controls'] for g in chosen]), names, counts)
    try:
        result = solver.compute(np.zeros(3), p['path'], p['current'], predictions,
                                visualization_rollouts=500, distance_field=p['field'])
    finally:
        solver.close()
        c.close()
    np.testing.assert_allclose(result.mode_final_costs, previous['final_costs'], rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(result.trajectory,
        np.load(BASE/'refinement/scenes/b1_088.npz')['valley_direction_cost_trajectory'], atol=1e-9)
    assert result.selected_mode == previous['selected'] and result.safety_status == 'ok'
    assert not p['field'].trajectory_collisions(result.trajectory[None, :, :2], np.zeros(2), ex.RADIUS)[0]
    assert len(result.sampled_controls) == 500
    np.testing.assert_array_equal(result.control_mode_ids, np.repeat(np.arange(3), counts))
    print('Verified actual inputs, all seed costs and final result against saved experiment.', flush=True)
    return p, nominal_path, definitions, options, items, chosen, result, previous, audit


def draw_path(ax, path, color, label=None, lw=2.5, style='-', alpha=1):
    xy = np.vstack((np.zeros(2), path[:, :2]))
    ax.plot(xy[:, 0], xy[:, 1], style, color=color, lw=lw, alpha=alpha, label=label)


def terrain(ax, p, scan_only=False):
    grid = p['scan_grid'] if scan_only else p['grid']
    display = np.zeros_like(grid)
    display[grid < 0] = 1
    if not scan_only:
        display[p['map_fill'] & (grid == 0)] = 2
    display[grid >= 65] = 3
    ax.imshow(display, extent=(-4, 4, -4, 4), origin='lower', vmin=0, vmax=3,
              cmap=ListedColormap(['white', '#d5dde6', '#e5edfc', '#273e53']), interpolation='nearest')
    for scan, color, label in zip(p['scans'], ['#d27730', '#436aa3'], ['앞 LiDAR', '뒤 LiDAR']):
        hits = scan['hits'][scan['valid']]
        ax.scatter(hits[:, 0], hits[:, 1], s=3, alpha=.65, color=color, label=label)
    ax.plot(p['full_path'][:, 0], p['full_path'][:, 1], '--', color='#4a91ba', lw=1.7, label='기록 global path')
    ax.add_patch(Circle((0, 0), .2, facecolor=INK, edgecolor='white', zorder=10))
    ax.annotate('', (.42, 0), (0, 0), arrowprops=dict(arrowstyle='->', color=INK, lw=2), zorder=11)
    ax.set(xlim=(-2.3, 2.3), ylim=(-2.3, 2.3), xlabel='전방 x [m]', ylabel='좌측 y [m]', aspect='equal')
    ax.grid(alpha=.12)


def sectors(ax, p, defs):
    for i, d in enumerate(defs):
        ids = d['ids']
        a, r = p['angles'][ids], p['d_free'][ids]
        xy = np.column_stack((r*np.cos(a), r*np.sin(a)))
        ax.add_patch(Polygon(np.vstack((np.zeros(2), xy, np.zeros(2))),
                             facecolor=COL[i], edgecolor=COL[i], alpha=.20))
        ax.plot(xy[:, 0], xy[:, 1], color=COL[i], lw=2)
        mid = ids[len(ids)//2]
        rr = .72*p['d_free'][mid]
        ax.text(rr*np.cos(p['angles'][mid]), rr*np.sin(p['angles'][mid]), f'G{i+1}',
                weight='bold', fontsize=16, color=COL[i], bbox=dict(facecolor='white', alpha=.8, edgecolor='none'))


def canvas(number, title, subtitle):
    fig = plt.figure(figsize=(16, 9), dpi=150)
    fig.text(.045, .938, f'{number:02d}  {title}', fontsize=23, weight='bold', color=INK)
    fig.text(.045, .89, subtitle, fontsize=12, color=MUTED)
    ax = fig.add_axes([.065, .17, .43, .65])
    right = fig.add_axes([.55, .17, .41, .65]); right.axis('off')
    fig.text(.045, .078, '실측 bag: mpc_chain_20260923_113151  |  +188.247981 s  |  scene b1_088  |  현재 base 좌표', fontsize=10, color=MUTED)
    fig.text(.045, .043, '흰색: scan 관측 free · 연파랑: 기록 지도 보완 free · 회색: 미확인 · 진한색: 장애물', fontsize=10, color=MUTED)
    fig.text(.96, .043, '실측 입력의 오프라인 계산 · 실제 실행 궤적 아님', ha='right', fontsize=9.5, color=MUTED)
    return fig, ax, right


def note(ax, text, y=.96, size=15):
    ax.text(0, y, text, va='top', color=INK, fontsize=size, linespacing=1.65, transform=ax.transAxes)


def table(ax, cells, columns, bbox=(0, .28, 1, .60), widths=None):
    t = ax.table(cellText=cells, colLabels=columns, cellLoc='center', bbox=bbox, colWidths=widths)
    t.auto_set_font_size(False); t.set_fontsize(11)
    for (r, c), cell in t.get_celld().items():
        cell.set_edgecolor('#d3dfeb'); cell.set_facecolor('#e6eef7' if r == 0 else 'white')
    return t


def render(p, nominal, defs, options, items, chosen, result, previous, audit):
    files = []
    def save(fig, name):
        fig.savefig(OUT/name, dpi=150); plt.close(fig); files.append(name)

    fig, ax, right = canvas(1, '실제 LiDAR 입력과 현재 기준 궤적', '합성 장애물 없이, 기록된 동일 장면을 모든 단계에서 사용')
    terrain(ax, p, scan_only=True); draw_path(ax, nominal, NOM, '기록 reference에서 복원한 nominal', 3)
    ax.legend(loc='lower left', fontsize=9)
    note(right, '이 장면의 실제 입력\n\n앞·뒤 LiDAR의 유효 hit를 시간 정렬\n현재 속도: vx=0.400, vy=−0.075 m/s\n현재 yaw rate: −0.123 rad/s\n\n기록 nominal의 2초 경로 진행: 0.000 m\n정적 충돌: 없음\n사용 가능한 사람 예측: 1명\n\nnominal은 당시 reference로 복원했으며,\n실행 중인 내부 warm start 원본은 아닙니다.')
    save(fig, '01_actual_lidar.png')

    fig, ax, right = canvas(2, '로봇이 들어갈 수 있는 깊이 계산', '몸체 반경과 여유를 반영한 d_free(θ) · 탐색 깊이는 최대 2 m')
    terrain(ax, p)
    for idx in range(0, 360, 4):
        a, d = p['angles'][idx], p['d_free'][idx]
        ax.plot([0, d*np.cos(a)], [0, d*np.sin(a)], color='#28a99d' if d >= .5 else '#cd6363', lw=.8, alpha=.6)
    note(right, '직진 가능한 깊이가 0.5 m 이상인가?\n\n청록색: gap 후보로 사용할 수 있는 방향\n붉은색: 최소 진입 깊이에 못 미치는 방향\n\n이 장면의 d_free 범위: 0.22 ~ 2.00 m\n0.5 m 이상인 방향: 255개 / 360개\n\n열린 구간은 원형 각도에서 1개입니다.\n95° → 180° 경계 → −11°로 이어집니다.\n\n연파란 영역은 LiDAR 직접 관측이 아닌\n기록 지도로 보완한 빈 공간입니다.')
    save(fig, '02_traversable_depth.png')

    fig, ax, right = canvas(3, '열린 구간 1개 → 깊이 골짜기로 gap 2개', '0.5 / 1.0 / 1.5 m별 복제 없이, 같은 구간 내부의 깊이 변화를 사용')
    terrain(ax, p); sectors(ax, p, defs)
    profile = fig.add_axes([.57, .47, .38, .33])
    run = ex.circular_runs(p['d_free'] >= .5)[0]
    degrees = np.rad2deg(np.unwrap(p['angles'][run]))
    smooth = gaussian_filter1d(p['d_free'], 2, mode='wrap')
    profile.plot(degrees, p['d_free'][run], color='#8a9aaa', lw=1, label='실제 깊이')
    profile.plot(degrees, smooth[run], color=INK, lw=1.8, label='경계 검출용 평활화')
    for i, d in enumerate(defs):
        positions = np.flatnonzero(np.isin(run, d['ids']))
        profile.axvspan(degrees[positions[0]], degrees[positions[-1]], color=COL[i], alpha=.16)
        profile.text(degrees[positions[len(positions)//2]], 1.83, f'G{i+1}', color=COL[i], weight='bold')
    boundary = np.rad2deg(p['angles'][defs[1]['ids'][0]])
    profile.axvline(boundary+360, color='#d16932', ls='--')
    profile.set(ylim=(.4, 2.12), xlabel='연속 각도 [°] · 305° = −55°', ylabel='진입 깊이 [m]')
    profile.legend(fontsize=9, loc='lower left'); profile.grid(alpha=.15)
    note(right, f'실제로 생긴 gap\nG1: 95° → ±180° 경계 → −55°  ({len(defs[0]["ids"])}개 방향)\nG2: −54° → −11°  ({len(defs[1]["ids"])}개 방향)\n\n−55° / −54° 사이에서 나눕니다.\n최대 깊이 방향: G1 −147°, G2 −14°', y=.33, size=13)
    save(fig, '03_actual_gap_split.png')

    fig, ax, right = canvas(4, '두 gap에서 실제 seed 6개 생성', '최대 깊이 · 각도 중앙 · 경로 방향 / 2초 제약을 적용한 제어입력열')
    terrain(ax, p)
    for o in options:
        style = {'최대 깊이': '--', '각도 중앙': ':', '경로 방향': '-'}[o['label']]
        draw_path(ax, o['trajectory'], COL[o['gap']], lw=2, style=style)
        ax.scatter(*o['target'], color=COL[o['gap']], marker='x', s=60)
        offset = (5, -14) if o['angle_deg'] == -14 else (5, 7)
        ax.annotate(f'{o["angle_deg"]:.0f}°', o['target'], xytext=offset,
                    textcoords='offset points', color=COL[o['gap']], fontsize=10)
    cells = [[f'G{o["gap"]+1}', o['label'], f'{o["angle_deg"]:.0f}°', f'{np.linalg.norm(o["target"]):.3f}'] for o in options]
    table(right, cells, ['gap', '방향 종류', '각도', '목표 거리 m'])
    note(right, '선: 실제 제약 seed 궤적 / ×: 목표 위치\n각도는 현재 로봇 전방을 0°로 표시\n이 단계는 6개 seed 평가이며 MPPI 6회가 아닙니다.', y=.22, size=12)
    save(fig, '04_six_actual_seeds.png')

    fig, ax, right = canvas(5, '실행 가능성·nominal 중복 확인', '이 장면에서는 6개 seed 모두 통과 · 제외된 후보를 임의로 만들지 않음')
    terrain(ax, p); draw_path(ax, nominal, NOM, 'nominal', 3)
    for o in options: draw_path(ax, o['trajectory'], COL[o['gap']], alpha=.65, lw=1.8)
    ax.legend(loc='lower left', fontsize=10)
    cells = [[f'G{o["gap"]+1} / {o["angle_deg"]:.0f}°', f'{o["reach_error"]:.3f}', f'{o["rms"]:.3f}', '없음', '통과'] for o in options]
    table(right, cells, ['후보', '목표 오차 m', 'RMS 차이 m', '충돌', '판정'])
    note(right, '목표 오차는 모두 0.35 m 이하입니다.\nnominal과의 RMS 차이는 모두 0.22 m 이상입니다.\n따라서 nominal 중복 조건에 걸리는 후보도 없습니다.', y=.22, size=12)
    save(fig, '05_actual_feasibility.png')

    fig, ax, right = canvas(6, '기존 비용으로 G1 −55° / G2 −11° 선택', '각 gap에서 대표 1개 선택 → gap 간 중복 검사 → 두 gap 모두 탐색')
    terrain(ax, p); sectors(ax, p, defs)
    for i, g in enumerate(items):
        draw_path(ax, g['trajectory'], COL[i], f'G{i+1}: {np.rad2deg(g["angle"]):.0f}°', 3)
        ax.scatter(*g['target'], color=COL[i], s=70, marker='x')
    ax.legend(loc='lower left', fontsize=10)
    cells = [[f'G{o["gap"]+1}', f'{o["angle_deg"]:.0f}°', f'{o["cost"]:.3f}',
              '대표 선택' if abs(np.deg2rad(o['angle_deg'])-items[o['gap']]['angle'])<1e-9 else '비용이 더 큼'] for o in options]
    table(right, cells, ['gap', '방향', '기존 seed 비용 ↓', '결과'])
    diff = np.sqrt(np.mean(np.sum((items[0]['trajectory'][:, :2]-items[1]['trajectory'][:, :2])**2, axis=1)))
    note(right, f'대표 간 RMS 차이: {diff:.3f} m ≥ 0.25 m → 서로 다른 후보\nG2 비용 3.629 < G1 비용 18.317 → G2가 우선순위 높음\n유효 gap이 2개이므로 둘 다 선택합니다. 별도 R 점수 없음.', y=.22, size=12)
    save(fig, '06_actual_value_selection.png')

    fig, ax, right = canvas(7, '실제 MPPI sampling: 300 + 100 + 100', '동일 기록 입력·고정 소스·noise seed 7로 재계산하고 저장 결과와 일치 확인')
    terrain(ax, p)
    # sampled_controls retains every actual rollout, independently of display subsampling.
    rollouts = ex.rollout(np.zeros(3), result.sampled_controls, .2)
    for lo, hi, color in [(0, 300, NOM), (300, 400, COL[0]), (400, 500, COL[1])]:
        for path in rollouts[lo:hi]: draw_path(ax, path, color, lw=.65, alpha=.12)
    for i, path in enumerate(result.mode_trajectories[:3]):
        draw_path(ax, path, [NOM, *COL][i], ['nominal 최적화', 'G1 최적화', 'G2 최적화'][i], 3)
    ax.legend(loc='lower left', fontsize=10)
    finite = np.isfinite(result.rollout_costs)
    counts = [int(finite[:300].sum()), int(finite[300:400].sum()), int(finite[400:].sum())]
    note(right, f'실제로 생성한 rollout 500개를 모두 표시\n\nnominal: 300개 / 유한 비용 {counts[0]}개\nG1 −55°: 100개 / 유한 비용 {counts[1]}개\nG2 −11°: 100개 / 유한 비용 {counts[2]}개\n\n얇은 선: 실제 sampling 궤적\n굵은 선: 각 그룹의 최적화 대표 궤적\n\n흐린 선에는 충돌 등으로 탈락한 궤적도 포함됩니다.\n총 유한 비용: {int(finite.sum())}/500개', size=14)
    save(fig, '07_actual_mppi_rollouts.png')

    fig, ax, right = canvas(8, '공통 재평가 후 G2 궤적 최종 선택', '초기 seed의 점수가 아니라, 최적화된 대표 궤적을 공통 조건에서 비교')
    terrain(ax, p)
    for i, path in enumerate(result.mode_trajectories[:3]):
        draw_path(ax, path, [NOM, *COL][i], ['nominal 최적화', 'G1 최적화', 'G2 최종 선택'][i], 4 if i==2 else 2, '-' if i==2 else '--')
    for point in result.trajectory[::2, :2]:
        ax.add_patch(Circle(point, .2, facecolor=COL[1], edgecolor=COL[1], alpha=.13))
    ax.legend(loc='lower left', fontsize=10)
    table(right, [[label, f'{cost:.3f}', '최종 선택' if i==2 else ''] for i,(label,cost) in enumerate(zip(['nominal', 'G1', 'G2', 'Stop'], result.mode_final_costs))],
          ['최종 대표', '공통 재평가 비용 ↓', '결과'], bbox=(0,.47,1,.4))
    note(right, f'G2 비용 {result.cost:.3f} → 최종 선택\n2초 예측 경로 진행: {previous["progress_m"]:.3f} m\n최종 궤적 정적 충돌 검사 통과 / MC 기준 통과\n\n실제 주행 결과가 아닌 오프라인 예측입니다.\n이 장면 145.3 ms는 과거 측정값이며,\n전체 실험에서 안정적인 5 Hz는 입증되지 않았습니다.', y=.39, size=13)
    save(fig, '08_actual_final_selection.png')
    # Contact sheet retains actual data plots, rather than replacing them with text boxes.
    w, h = 1920, 1080
    sheet = Image.new('RGB', (2*w, 4*h), '#f8fafc')
    for i, file in enumerate(files):
        with Image.open(OUT/file) as im:
            sheet.paste(im.resize((w,h), Image.Resampling.LANCZOS), ((i%2)*w, (i//2)*h))
    sheet.save(OUT/'00_actual_lidar_full_flow.png')
    manifest = dict(scene='b1_088', bag=audit['bag'], offset=audit['offset'], record_ns=audit['record_ns'],
        method='valley_direction_cost (no necessity gate)', noise_seed=7,
        source='Original recorded ROS bag, reprocessed with frozen experiment sources',
        validation='Input arrays, every seed cost and final costs/trajectory matched saved experiment',
        gap_intervals=[dict(id=f'G{i+1}', angles_deg=np.rad2deg(p['angles'][g['ids']])) for i,g in enumerate(defs)],
        options=[{k:v for k,v in o.items() if k not in ('controls','trajectory')} for o in options],
        selected=[{k:v for k,v in g.items() if k not in ('controls','trajectory','ids')} for g in chosen],
        final_costs=result.mode_final_costs, final_names=result.candidate_names,
        historical_trial=previous, files=files,
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    ex.write_json(OUT/'manifest.json', manifest)
    np.savez_compressed(OUT/'actual_flow_arrays.npz', grid=p['grid'], scan_grid=p['scan_grid'],
        map_fill=p['map_fill'], angles=p['angles'], d_free=p['d_free'], nominal=nominal,
        full_path=p['full_path'], scan_front=p['scans'][0]['hits'][p['scans'][0]['valid']],
        scan_rear=p['scans'][1]['hits'][p['scans'][1]['valid']],
        seed_controls=np.array([o['controls'] for o in options]), seed_paths=np.array([o['trajectory'] for o in options]),
        sampled_controls=result.sampled_controls, rollouts=rollouts, rollout_costs=result.rollout_costs,
        representatives=result.mode_trajectories, final_trajectory=result.trajectory)
    with zipfile.ZipFile(OUT/'actual_lidar_flow_pngs.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for f in sorted(OUT.iterdir()):
            if f.suffix in ('.png', '.json', '.npz'): z.write(f, f.name)
        z.write(Path(__file__), Path(__file__).name)
    print('Created', OUT, flush=True)


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    render(*collect())

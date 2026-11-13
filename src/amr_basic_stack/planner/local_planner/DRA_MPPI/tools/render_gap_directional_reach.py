"""Plot measured-space depth and conditional maximum straight-ray reach.

Scientific offline calculation, not a production planner change. For each of
360 robot-frame directions, solve an LP with ten nonnegative world-ray speeds.
The yaw-rate sequence is fixed either to zero or to the saved nominal's rates.
This is NOT the global maximum over freely optimized yaw or curved paths.
"""
from pathlib import Path
import csv
import hashlib
import json

import numpy as np
from scipy.optimize import linprog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Polygon

from dra_mppi.gap_geometry import GapConfig
from dra_mppi.gap_inputs import GapScan, directional_depth, fuse_scans
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mecanum_model import rollout


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/gap_implementation_flow_2026-11-05'
BASE = ROOT / 'docs/gap_multiscene_2026-11-04'
OUT = ROOT / 'docs/gap_directional_reach_2026-11-05'
COLORS = ['#c47b16', '#3383ba', '#009c91', '#7c55b5', '#d75a70', '#678225']
INK, MUTED = '#20364b', '#536b7e'
KIN, SPACE, BOTH = '#e07816', '#2878ba', '#15915f'


def envelope(angles, omega, current, cfg):
    """Exact LP optimum for the discrete model, fixed yaw, straight rays."""
    n, dt = cfg['horizon_steps'], cfg['dt']
    lo, hi = np.array(cfg['control_min']), np.array(cfg['control_max'])
    acceleration = np.array(cfg['acceleration_limit'])
    yaw = np.r_[0., np.cumsum(omega[:-1]) * dt]
    assert np.all(omega >= lo[2]-1e-9) and np.all(omega <= hi[2]+1e-9)
    assert np.max(np.abs(np.diff(np.r_[current[2], omega]))) <= acceleration[2]*dt+1e-9
    controls, distances = [], []
    for angle in angles:
        body_direction = np.column_stack((np.cos(angle-yaw), np.sin(angle-yaw)))
        A, b = [], []
        for t in range(n):
            for axis in range(2):
                row = np.zeros(n); row[t] = body_direction[t, axis]
                A.extend((row.copy(), -row.copy())); b.extend((hi[axis], -lo[axis]))
                if t:
                    row[t-1] = -body_direction[t-1, axis]
                    previous = 0.
                else:
                    previous = current[axis]
                A.extend((row.copy(), -row.copy()))
                b.extend((acceleration[axis]*dt+previous, acceleration[axis]*dt-previous))
            row = np.zeros(n); row[t] = np.sum(np.abs(body_direction[t]))
            A.append(row); b.append(cfg['speed_limit_l1']-cfg['yaw_lever']*abs(omega[t]))
        answer = linprog(-np.ones(n)*dt, A_ub=np.array(A), b_ub=np.array(b),
                         bounds=[(0., None)]*n, method='highs')
        if not answer.success:
            raise RuntimeError(f'LP failed at {np.rad2deg(angle)}: {answer.message}')
        assert np.max(np.array(A)@answer.x-np.array(b)) < 1e-7
        controls.append(np.column_stack((answer.x[:, None]*body_direction, omega)))
        distances.append(-answer.fun)
    return np.array(distances), np.array(controls)


def validate(controls, distances, angles, current, cfg):
    lo, hi = np.array(cfg['control_min']), np.array(cfg['control_max'])
    assert np.all(controls >= lo-1e-7) and np.all(controls <= hi+1e-7)
    loads = np.abs(controls[..., :2]).sum(axis=-1)+cfg['yaw_lever']*np.abs(controls[..., 2])
    assert loads.max() <= cfg['speed_limit_l1']+1e-7
    previous = np.broadcast_to(current, (len(angles), 1, 3))
    changes = np.diff(np.concatenate((previous, controls), axis=1), axis=1)
    assert np.all(np.abs(changes) <= np.array(cfg['acceleration_limit'])*cfg['dt']+1e-7)
    paths = rollout(np.zeros(3), controls, cfg['dt'])
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    np.testing.assert_allclose(paths[:, -1, :2], distances[:, None]*directions, atol=1e-7)
    transverse = paths[..., 0]*(-directions[:, None, 1])+paths[..., 1]*directions[:, None, 0]
    assert np.max(np.abs(transverse)) < 1e-7
    return paths


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    old = json.loads((SOURCE/'results.json').read_text())
    cfg = old['configuration']; n, dt = cfg['horizon_steps'], cfg['dt']
    scene_file = BASE / 'disparity_partition_10_scenes/data/b2_100.npz'
    data = dict(np.load(scene_file))
    replay = dict(np.load(SOURCE/'replay_arrays.npz'))
    audit_file = BASE/'scene_audit.jsonl'
    audit = next(json.loads(line) for line in audit_file.read_text().splitlines()
                 if json.loads(line)['scene'] == old['scene'])
    current = np.array(audit['current'])
    field = DistanceField.from_occupancy(data['grid'], GridMetadata(200, 200, .04, -4., -4., 0.))
    scans = tuple(GapScan(*(data[f'scan_{i}_{key}'] for key in ('ranges', 'valid', 'origins', 'hits')))
                  for i in range(2))
    gap = GapConfig(**cfg['gap'])
    fused, _, _ = fuse_scans(scans, np.zeros(3), field, None, gap, cfg['static_cost_robot_radius'])
    angles, depth, _ = directional_depth(fused, gap, cfg['static_cost_robot_radius'])
    np.testing.assert_allclose(depth, data['strict_depth'], atol=1e-12)
    regions = old['partition']['regions']
    mids = np.array([r['bins'][len(r['bins'])//2] for r in regions])
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    cases = {}
    saved_arrays = dict(angles=angles, depth=depth, current=current)
    for name, omega in [('fixed_heading', np.zeros(n)), ('recorded_nominal_yaw', replay['nominal_controls'][:, 2])]:
        distance, controls = envelope(angles, omega, current, cfg)
        paths = validate(controls, distance, angles, current, cfg)
        combined = np.minimum(distance, depth)
        feasible_controls = controls.copy()
        feasible_controls[..., :2] *= (combined/distance)[:, None, None]
        feasible_paths = validate(feasible_controls, combined, angles, current, cfg)
        assert np.all(np.linalg.norm(feasible_paths[..., :2], axis=-1) <= depth[:, None]+1e-7)
        collisions = fused.trajectory_collisions(feasible_paths[..., :2], np.zeros(2),
                                                  cfg['static_cost_robot_radius'])
        assert not np.any(collisions), 'Combined trajectories must pass production swept-body collision checks.'
        rows = []
        for region, j in zip(regions, mids):
            rows.append(dict(gap=f'G{region["id"]}', angle_deg=int(round(np.rad2deg(angles[j]))),
                             kinematic_m=float(distance[j]), space_depth_m=float(depth[j]),
                             combined_m=float(combined[j]),
                             limiting='space' if depth[j] < distance[j] else 'kinematics'))
        cases[name] = dict(distance=distance, combined=combined, omega=omega, rows=rows)
        saved_arrays.update({f'{name}_kinematic_m': distance, f'{name}_combined_m': combined,
                             f'{name}_controls': controls, f'{name}_paths': paths,
                             f'{name}_combined_controls': feasible_controls,
                             f'{name}_combined_paths': feasible_paths})

    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font).get_name(),
                         'mathtext.fontset': 'dejavusans', 'axes.unicode_minus': False, 'font.size': 11})

    def draw(name, number, title, subtitle):
        case = cases[name]
        fig = plt.figure(figsize=(17, 12), facecolor='#f6f8fb')
        fig.text(.045, .95, title, color=INK, fontsize=25, weight='bold')
        fig.text(.045, .912, subtitle, color=MUTED, fontsize=12)
        ax = fig.add_axes([.055, .365, .44, .49])
        view = np.zeros_like(data['grid'])
        view[data['grid'] < 0] = 1
        view[data['map_fill'] & (data['grid'] == 0)] = 2
        view[data['grid'] >= 65] = 3
        ax.imshow(view, extent=(-4, 4, -4, 4), origin='lower', vmin=0, vmax=3,
                  cmap=ListedColormap(['white', '#d8dfe7', '#e5eefb', '#34495e']), interpolation='nearest')
        for region, j in zip(regions, mids):
            ids = np.array(region['bins']); color = COLORS[region['id']-1]
            xy = depth[ids, None]*directions[ids]
            ax.add_patch(Polygon(np.vstack(([0, 0], xy)), fc=color, alpha=.09, ec='none'))
            for endpoint in (xy[0], xy[-1]):
                ax.plot([0, endpoint[0]], [0, endpoint[1]], color=color, alpha=.35, lw=.8)
            pos = (depth[j]+.16)*directions[j]
            ax.text(*pos, f'G{region["id"]}', color=color, weight='bold', fontsize=12,
                    ha='center', va='center', bbox=dict(fc='white', ec='none', alpha=.8, pad=1))
        for i, color in enumerate(('#b27347', '#677a8a')):
            hits = data[f'scan_{i}_hits'][data[f'scan_{i}_valid']]
            ax.scatter(hits[:, 0], hits[:, 1], s=4, color=color, alpha=.8,
                       label=('실제 전방 LiDAR 점' if i == 0 else '실제 후방 LiDAR 점'))
        for values, color, style, label, width in [
                (depth, SPACE, '-', '공간 깊이 d (LiDAR+지도)', 1.8),
                (case['distance'], KIN, '--', '2초 구동 최대거리 D', 2.5),
                (case['combined'], BOTH, '-', '두 제한 적용 min(D, d)', 3.0)]:
            xy = values[:, None]*directions
            xy = np.vstack((xy, xy[0]))
            ax.plot(xy[:, 0], xy[:, 1], style, color=color, lw=width, label=label)
        ax.add_patch(Circle((0, 0), .2, fc=INK, ec='white', zorder=9))
        ax.annotate('', (.37, 0), (0, 0), arrowprops=dict(arrowstyle='->', color=INK, lw=2), zorder=10)
        ax.set(xlim=(-2.4, 2.4), ylim=(-2.4, 2.4), aspect='equal', xlabel='x [m] · 전방 + / 후방 −', ylabel='y [m] · 좌측 + / 우측 −')
        ax.grid(alpha=.12)
        ax.legend(loc='lower right', fontsize=9, framealpha=.96)
        ax.set_title('원본과 동일한 측정점·공간·G1~G6', fontsize=14, pad=12)

        ax = fig.add_axes([.565, .60, .385, .25])
        degrees = np.rad2deg(angles)
        for region in regions:
            ids = np.array(region['bins']); color = COLORS[region['id']-1]
            split = np.split(ids, np.where(np.diff(ids) != 1)[0]+1)
            for chunk in split:
                ax.axvspan(degrees[chunk[0]]-.5, degrees[chunk[-1]]+.5, color=color, alpha=.06)
        ax.plot(degrees, depth, color=SPACE, lw=1.8, label='공간 d')
        ax.plot(degrees, case['distance'], '--', color=KIN, lw=2.3, label='구동 D')
        ax.plot(degrees, case['combined'], color=BOTH, lw=2.7, label='min(D, d)')
        ax.scatter(degrees[mids], case['combined'][mids], color=BOTH, s=20, zorder=5)
        ax.set(xlim=(-180, 180), ylim=(0, 2.18), xticks=[-180, -90, 0, 90, 180],
               xlabel='방향 θ [°] · 0° 전방 / +90° 좌측', ylabel='로봇 중심 이동 거리 [m]')
        ax.grid(alpha=.15); ax.legend(loc='upper right', ncol=3, fontsize=9)
        ax.set_title('360개 방향을 1° 간격으로 계산', fontsize=14, pad=12)

        fig.text(.555, .535, '주황선: 장애물이 없을 때, 해당 회전 조건의 구동 한계\n'
                 '파란선: 몸체와 여유를 고려한 관측 공간의 깊이\n'
                 '초록선: 두 제한 중 짧은 거리까지 이동 가능',
                 color=INK, fontsize=14, linespacing=1.8, va='top')
        fig.text(.555, .385, r'$D_{\mathrm{use}}(\theta)=\min\{D_{\mathrm{kin}}(\theta),\ d(\theta)\}$',
                 color=INK, fontsize=20)
        tax = fig.add_axes([.055, .165, .53, .145]); tax.axis('off')
        cells = [[r['gap'], f"{r['angle_deg']}°", f"{r['kinematic_m']:.3f}",
                  f"{r['space_depth_m']:.3f}"+(' *' if r['space_depth_m'] >= gap.max_depth_m else ''),
                  f"{r['combined_m']:.3f}", '공간' if r['limiting']=='space' else '구동'] for r in case['rows']]
        table = tax.table(cellText=cells, colLabels=['gap', '중앙 방향', '구동 D [m]', '공간 d [m]', '적용 거리 [m]', '제한 원인'],
                          cellLoc='center', loc='center', colWidths=[.10, .15, .18, .18, .21, .18])
        table.auto_set_font_size(False); table.set_fontsize(10); table.scale(1, 1.5)
        for (row, col), cell in table.get_celld().items():
            cell.set_edgecolor('#cfd9e5')
            if row == 0: cell.set_facecolor('#e5ecf4')
            elif col == 0: cell.get_text().set_color(COLORS[row-1]); cell.get_text().set_weight('bold')
        if name == 'fixed_heading':
            fig.text(.625, .29, '이번 계산 조건', fontsize=15, color=INK, weight='bold')
            fig.text(.625, .255, 'T = 2초, Δt = 0.2초, N = 10\n'
                     'ω = 0 · 각 방향으로 직선 이동\n'
                     '|vx| + |vy| + 0.19835|ω| ≤ 0.78\n'
                     '현재 속도·축별 속도·가속도 제한 반영',
                     fontsize=11, color=INK, linespacing=1.6, va='top')
        else:
            wax = fig.add_axes([.65, .175, .29, .115])
            times = np.arange(n+1)*dt
            wax.stairs(case['omega'], times, baseline=None, color='#7c55b5', lw=2)
            wax.set(xlabel='시간 [s]', ylabel='ω [rad/s]', xlim=(0, 2))
            wax.set_title('저장 nominal의 회전 명령을 고정', fontsize=11)
            wax.grid(alpha=.15)
        fig.text(.055, .105, '* d=2.000m는 탐색 상한: 2m까지 확인되었다는 뜻이며, 그곳에 벽이 있다는 뜻은 아닙니다.', color=MUTED, fontsize=10)
        fig.text(.055, .078, 'd는 raw 센서 거리 r이 아닙니다. 실제 전·후방 scan + 지도 보완, 로봇 반경 0.20m + 여유 0.05m + 격자 보정을 반영했습니다.', color=MUTED, fontsize=10)
        fig.text(.055, .051, f"실제 {old['scene']} | {old['source']} +{old['offset']:.3f}s | 고정 회전열·직선 이동 조건의 LP 최대값 · 자유 회전 최적화 결과 아님", color=MUTED, fontsize=10)
        fig.text(.055, .026, '원본 seed 목표점 축소(10cm·90%) 미적용 · 종단 정지 제약 없음 · 오프라인 모델 계산 · 동적 장애물 미반영 · 운영 코드 변경 없음', color=MUTED, fontsize=10)
        fig.savefig(OUT/f'{number:02d}_{name}.png', dpi=160)
        plt.close(fig)

    draw('fixed_heading', 1, '실제 LiDAR 장면 · 방향별 최대 거리 비교',
         '조건 A: 현재 차체 방향 유지(ω=0). 2초 동안 직선으로 갈 수 있는 구동 한계와 관측 공간 깊이를 비교합니다.')
    draw('recorded_nominal_yaw', 2, '같은 장면 · 회전 명령까지 반영하면?',
         '조건 B: 저장 nominal의 ω(t)를 그대로 사용. vx·vy는 각 방향의 도달 거리를 최대화하도록 새로 계산합니다.')
    np.savez_compressed(OUT/'arrays.npz', **saved_arrays)
    with (OUT/'all_directions.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['angle_deg', 'space_depth_m', 'fixed_heading_kinematic_m', 'fixed_heading_combined_m',
                         'recorded_yaw_kinematic_m', 'recorded_yaw_combined_m'])
        for j, a in enumerate(angles):
            writer.writerow([round(np.rad2deg(a)), depth[j], cases['fixed_heading']['distance'][j],
                             cases['fixed_heading']['combined'][j], cases['recorded_nominal_yaw']['distance'][j],
                             cases['recorded_nominal_yaw']['combined'][j]])
    report = dict(scene=old['scene'], source=old['source'], offset=old['offset'], configuration=cfg,
                  current=current.tolist(),
                  scope='Conditional maximum straight-ray distance with fixed yaw-rate sequences; NOT free-yaw global reachability.',
                  method='360 linear programs per case, 10 speed variables; axis, acceleration and wheel-L1 constraints; maximize dt*sum(speed).',
                  depth='Recomputed using production fusion and directional_depth, exactly matches original cached strict_depth; capped at 2m.',
                  combined='min(D,d); all scaled control sequences independently checked for feasibility and straight-ray endpoints.',
                  validation='720 LP optima and 720 combined sequences satisfy all constraints and production rollout endpoint checks. All 720 combined paths pass production swept-body static collision checks.',
                  sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                          [scene_file, SOURCE/'results.json', SOURCE/'replay_arrays.npz', audit_file]},
                  cases={name: dict(omega=case['omega'].tolist(), rows=case['rows']) for name, case in cases.items()})
    (OUT/'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    (OUT/'README.txt').write_text(
        '같은 장면 방향별 거리 비교\n'
        '01_fixed_heading.png: 차체 방향 유지(omega=0).\n'
        '02_recorded_nominal_yaw.png: 저장 nominal 회전 명령을 고정하고 병진 속도를 최적화.\n'
        '두 그림 모두 조건부 직선 최대 거리이며, 자유 회전/곡선 경로를 포함한 전역 최대가 아니다.\n'
        '원본 LiDAR+지도 공간 깊이를 재계산하여 기존 결과와 일치함을 확인했다.\n'
        '회전열과 몸체 좌표 축 제한이 함께 작용하므로 조건 B가 모든 방향에서 A보다 작을 필요는 없다.\n'
        'd=2m는 상한까지 관측 공간이 확인됨을 의미한다. raw range나 벽 위치가 아니다.\n'
        '현재 명령은 기록값을 사용. 정지에 가깝지만 정확한 0으로 바꾸지 않았다.\n'
        '표는 원본 G1~G6 중앙 방향. CSV는 360방향 전부. NPZ는 구동 최대 및 공간 제한 후 제어열/궤적.\n'
        '기존 seed 여유 10cm, 도달거리 90%, desired_speed 0.75는 최대 구동 거리 계산에 적용하지 않았다.\n'
        '종단 정지는 요구하지 않는다. 해당 시간까지 도달하는 최대 거리이며 정지 가능 거리 계산은 아니다.\n'
        '이 계산은 5Hz 성능 검증도 새 비용 최적화 구현도 아니다. 운영 코드/YAML 미변경.\n'
        '재생성: PYTHONPATH=planner/local_planner/DRA_MPPI /usr/bin/python3 '
        'planner/local_planner/DRA_MPPI/tools/render_gap_directional_reach.py\n')
    print(json.dumps(dict(output=str(OUT), current=current.tolist(),
                          cases={name: case['rows'] for name, case in cases.items()}), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

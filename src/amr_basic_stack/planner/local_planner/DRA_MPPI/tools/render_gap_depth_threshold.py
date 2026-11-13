"""Compare d(theta)>=0.5m and >=1.0m on the same saved actual LiDAR scene.

Only the offline gap minimum_depth_m changes; production YAML is untouched.
"""
from dataclasses import asdict, replace
from pathlib import Path
import json

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Polygon

from dra_mppi.gap_geometry import GapConfig, partition
from dra_mppi.gap_inputs import GapScan, directional_depth, fuse_scans
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiConfig, MppiController
from dra_mppi.dynamic_risk import DynamicRiskConfig
from dra_mppi.mecanum_model import rollout
from render_production_gap_flow import clean, COLORS, REASONS


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/gap_implementation_flow_2026-11-05'
BASE = ROOT / 'docs/gap_multiscene_2026-11-04'
OUT = ROOT / 'docs/gap_depth_1m_2026-11-05'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    old = json.loads((SOURCE / 'results.json').read_text())
    cfg_data = dict(old['configuration'])
    cfg_data['gap'] = GapConfig(**cfg_data['gap'])
    cfg_data['dynamic_risk'] = DynamicRiskConfig(**cfg_data['dynamic_risk'])
    baseline = MppiConfig(**cfg_data)
    cfg = replace(baseline, gap=replace(baseline.gap, minimum_depth_m=1.0))
    scene_id = old['scene']
    data = dict(np.load(BASE / f'disparity_partition_10_scenes/data/{scene_id}.npz'))
    replay = dict(np.load(SOURCE / 'replay_arrays.npz'))
    audit = next(json.loads(line) for line in (BASE / 'scene_audit.jsonl').read_text().splitlines()
                 if json.loads(line)['scene'] == scene_id)
    state = np.zeros(3)
    field = DistanceField.from_occupancy(data['grid'], GridMetadata(200, 200, .04, -4., -4., 0.))
    scans = tuple(GapScan(*(data[f'scan_{i}_{key}'] for key in ('ranges', 'valid', 'origins', 'hits')))
                  for i in range(2))
    fused, transform, local = fuse_scans(scans, state, field, None, cfg.gap, cfg.static_cost_robot_radius)
    angles, strict, hard = directional_depth(fused, cfg.gap, cfg.static_cost_robot_radius)
    np.testing.assert_allclose(strict, data['strict_depth'], atol=1e-12)
    arrays = dict(strict_depth=strict, hard_depth=hard)
    for i, s in enumerate(local):
        arrays.update({f'scan_{i}_{k}': s[k] for k in ('ranges', 'valid', 'hits')})
    parts_old = partition(arrays, asdict(baseline.gap))
    parts_new = partition(arrays, asdict(cfg.gap))
    assert [r['bins'] for r in parts_old['regions']] == [r['bins'] for r in old['partition']['regions']]
    assert not parts_new['fallback'], 'This visualization expects strict 5cm-margin depths.'
    capture = []
    c = MppiController(cfg)
    optimizer = c._guide_optimizer
    original = optimizer._seed

    def record(state, path, current, target):
        controls = original(state, path, current, target)
        capture.append((target.copy(), controls.copy()))
        return controls

    optimizer._seed = record
    optimizer._warm_nominal = replay['nominal_controls'].copy()
    try:
        result = c.compute(state, data['full_path'], np.array(audit['current']),
                           distance_field=field, gap_scans=scans)
    finally:
        c.close()
    assert len(capture) == 2*parts_new['region_count']
    seeds = capture[1::2]
    seed_paths = rollout(state, np.array([x[1] for x in seeds]), cfg.dt)
    assert result.gap_diagnostics['region_count'] == parts_new['region_count']
    matching = []
    for region in parts_new['regions']:
        overlap = [len(set(region['bins']) & set(r['bins'])) for r in parts_old['regions']]
        matching.append(int(np.argmax(overlap))+1)
    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font).get_name(),
                         'mathtext.fontset': 'dejavusans', 'axes.unicode_minus': False, 'font.size': 12})
    ink, muted, background = '#20364b', '#536b7e', '#f6f8fb'

    def base(title, subtitle):
        fig = plt.figure(figsize=(16, 11), facecolor=background)
        fig.text(.045, .95, title, fontsize=25, weight='bold', color=ink)
        fig.text(.045, .91, subtitle, fontsize=13, color=muted)
        fig.text(.045, .052, f"실제 {scene_id} | {old['source']} +{old['offset']:.3f}s | minimum_depth_m만 0.5 → 1.0m 변경", fontsize=10, color=muted)
        fig.text(.045, .026, '저장된 scan·지도·nominal 사용 · 동적 예측 미주입 · 오프라인 비교 · 운영 YAML 변경 없음', fontsize=10, color=muted)
        return fig

    def map_ax(fig, rect, part, old_ids=None):
        ax = fig.add_axes(rect)
        view = np.zeros_like(data['grid'])
        view[data['grid'] < 0] = 1
        view[data['map_fill'] & (data['grid'] == 0)] = 2
        view[data['grid'] >= 65] = 3
        ax.imshow(view, extent=(-4, 4, -4, 4), origin='lower', vmin=0, vmax=3,
                  cmap=ListedColormap(['white', '#d8dfe7', '#e5eefb', '#34495e']), interpolation='nearest')
        for i, color in enumerate(('#df8646', '#468dc4')):
            hits = data[f'scan_{i}_hits'][data[f'scan_{i}_valid']]
            ax.scatter(hits[:, 0], hits[:, 1], s=2, color=color, alpha=.6)
        for i, region in enumerate(part['regions']):
            ids = np.array(region['bins']); a = angles[ids]
            xy = strict[ids, None]*np.column_stack((np.cos(a), np.sin(a)))
            color = COLORS[(old_ids[i] if old_ids else region['id'])-1]
            ax.add_patch(Polygon(np.vstack(([0, 0], xy)), fc=color, alpha=.20, ec='none'))
            ax.plot(xy[:, 0], xy[:, 1], color=color, lw=2)
            for endpoint in (xy[0], xy[-1]): ax.plot([0, endpoint[0]], [0, endpoint[1]], color=color, alpha=.6, lw=1)
            j = ids[len(ids)//2]
            pos = max(.62, strict[j]*.8)*np.array([np.cos(angles[j]), np.sin(angles[j])])
            label = f'N{region["id"]}' if old_ids else f'G{region["id"]}'
            ax.text(*pos, label, color=color, ha='center', fontsize=14, weight='bold',
                    bbox=dict(fc='white', ec='none', alpha=.85, pad=1))
        ax.add_patch(Circle((0, 0), .2, fc=ink, ec='white', zorder=9))
        ax.annotate('', (.36, 0), (0, 0), arrowprops=dict(arrowstyle='->', color=ink, lw=2), zorder=10)
        ax.set(xlim=(-2.4, 2.4), ylim=(-2.4, 2.4), aspect='equal', xlabel='x [m] · 전방 + / 후방 −', ylabel='y [m] · 좌측 + / 우측 −')
        ax.grid(alpha=.12)
        return ax

    fig = base('같은 실제 LiDAR · 열린 공간 기준 0.5m vs 1.0m', '측정점·지도·로봇 반경·추가 여유는 동일합니다. d(θ)는 장애물까지의 raw 거리가 아닌 로봇 중심의 진입 깊이입니다.')
    left = map_ax(fig, (.055, .40, .40, .43), parts_old)
    right = map_ax(fig, (.55, .40, .40, .43), parts_new, matching)
    left.set_title(r'$d(\theta)\geq0.5\,\mathrm{m}$'+'  |  열린 구간 3개 → gap 6개', fontsize=16, pad=12)
    right.set_title(r'$d(\theta)\geq1.0\,\mathrm{m}$'+'  |  열린 구간 1개 → gap 3개', fontsize=16, pad=12)
    ax = fig.add_axes([.09, .14, .82, .14])
    deg = np.rad2deg(angles)
    ax.plot(deg, strict, color=ink, lw=1.8)
    ax.axhline(.5, color='#b77e18', ls='--', label='기존 기준 0.5m')
    ax.axhline(1., color='#d94f48', ls='--', label='새 기준 1.0m')
    ax.fill_between(deg, 0, strict, where=strict>=1., color='#7c55b5', alpha=.18, label='1m 기준 통과 방향')
    ax.set(xlim=(-180, 180), ylim=(0, 2.15), ylabel='진입 깊이 [m]', xlabel='로봇 기준 각도 [°]', xticks=[-180, -90, 0, 90, 180])
    ax.legend(loc='upper center', ncol=3, fontsize=10); ax.grid(alpha=.15)
    fig.text(.55, .34, 'N1←기존 G3 일부  ·  N2←기존 G4  ·  N3←기존 G5 일부\n색은 기존 공간과 대응하며, N은 새로 붙인 gap 번호입니다.', fontsize=12, color=muted, linespacing=1.6, va='top')
    fig.savefig(OUT / '01_threshold_comparison.png', dpi=150)
    plt.close(fig)

    fig = base('d(θ) ≥ 1m · 남은 gap과 중앙 seed의 실제 결과', '1m는 열린 공간의 진입 깊이 기준입니다. seed가 2초 안에 반드시 1m 이동해야 한다는 조건은 아닙니다.')
    ax = map_ax(fig, (.055, .30, .43, .53), parts_new, matching)
    xy = np.vstack(([0, 0], replay['nominal_path'][:, :2]))
    ax.plot(xy[:, 0], xy[:, 1], '--', lw=2.5, color=ink, label='기존 nominal')
    for i, tr in enumerate(seed_paths):
        xy = np.vstack(([0, 0], tr[:, :2]))
        color = COLORS[matching[i]-1]
        ax.plot(xy[:, 0], xy[:, 1], lw=2.3, color=color, label=f'N{i+1} seed')
        ax.scatter(*seeds[i][0], marker='x', s=70, color=color)
    ax.legend(loc='lower right', fontsize=10)
    rows = []
    for i, (region, candidate) in enumerate(zip(parts_new['regions'], result.gap_diagnostics['candidates'])):
        rows.append([f'N{i+1}', f"{region['first_angle_deg']}° → {region['last_angle_deg']}°",
                     f"{candidate['angle_deg']:.0f}°", REASONS[candidate['reason']]])
    tax = fig.add_axes([.53, .58, .43, .22]); tax.axis('off')
    table = tax.table(cellText=rows, colLabels=['새 gap', '각도 구간', '중앙', '판정'],
                      colWidths=[.12, .38, .16, .34], cellLoc='center', loc='center')
    table.auto_set_font_size(False); table.set_fontsize(12); table.scale(1, 2)
    text = ('N2의 각도는 +180°/−180°를 지나 이어집니다.\n'
            '1m 미만인 전방·측방 공간이 제외되어 후방 위주로 남습니다.\n'
            f'5cm 추가 여유 제거 재시도: {"사용" if parts_new["fallback"] else "사용 안 함"}')
    fig.text(.54, .53, text, fontsize=13, color=ink, va='top', linespacing=1.7)
    rows = [[f'N{i+1}', f"{q['target_error_m']:.3f}", f"{q['projected_displacement_m']:.3f}",
             f"{q['nominal_rms_m']:.3f}", f"{q['nominal_tail_m']:.3f}"] for i,q in enumerate(result.gap_diagnostics['candidates'])]
    tax = fig.add_axes([.53, .27, .43, .16]); tax.axis('off')
    table = tax.table(cellText=rows, colLabels=['gap', '목표 오차', 'gap 이동', 'nom. RMS', '끝 3점'], cellLoc='center', loc='center')
    table.auto_set_font_size(False); table.set_fontsize(11); table.scale(1, 1.8)
    fig.text(.54, .255, '표의 거리 단위: m. 도달 가능성·nominal 중복 기준은 기존과 동일.', fontsize=10, color=muted)
    selected = [n.replace('gap_0', 'N') for n in result.gap_diagnostics['selected']]
    fig.text(.055, .185, '실제 선택: '+', '.join(selected)+f"  |  rollout 배분: {result.guide_rollout_counts.tolist()}  |  최종 MPPI: {result.selected_mode}", fontsize=15, color=ink, weight='bold')
    fig.text(.055, .13, 'gap 번호는 재분할할 때 새로 붙습니다. raw LiDAR 거리가 아닌 몸체 진입 깊이에 1m 기준을 적용했습니다.', fontsize=12, color=muted)
    fig.savefig(OUT / '02_one_meter_gaps_and_seeds.png', dpi=150)
    plt.close(fig)

    report = dict(scene=scene_id, source=str(SOURCE / 'results.json'), changed_parameter='gap.minimum_depth_m',
                  before=.5, after=1., production_yaml_modified=False,
                  partition_before=parts_old, partition_after=parts_new,
                  corresponding_old_gap_ids=matching, diagnostics=result.gap_diagnostics,
                  selected_mode=result.selected_mode, final_costs=result.mode_final_costs,
                  configuration=asdict(cfg), scope='Offline actual cached scans/map/nominal; no dynamic predictions or ROS publication.')
    (OUT / 'results.json').write_text(json.dumps(clean(report), indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    np.savez_compressed(OUT / 'one_meter_arrays.npz', seed_paths=seed_paths, seed_targets=np.array([s[0] for s in seeds]),
                        selected_path=result.trajectory, strict_depth=strict)
    print(json.dumps(clean(dict(output=str(OUT), open_runs=parts_new['open_runs'], regions=parts_new['regions'],
                               diagnostics=result.gap_diagnostics, final=result.selected_mode)), ensure_ascii=False))


if __name__ == '__main__':
    main()

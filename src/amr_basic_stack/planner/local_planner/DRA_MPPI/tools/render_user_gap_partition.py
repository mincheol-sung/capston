"""Draw the user's exact four boundary rays on the previously replayed scan.

This is a manual interpretation diagram, not a new gap detector or solver run.
"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Polygon

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'docs/gap_multiscene_2026-11-04'
OUT = BASE / 'user_gap_partition'
OUT.mkdir(exist_ok=True)
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font).get_name(),
                     'axes.unicode_minus': False, 'font.size': 12})
points = np.array([[2., -.5], [-.4, -2.], [-2., .4], [0., .4]])
angles = np.unwrap(np.arctan2(points[:, 1], points[:, 0]))
assert np.all(np.diff(angles) < 0)
colors = ['#008e85', '#8853b5', '#cf8a22']

with np.load(BASE / 'gap_v1_actual_flow/b1_088_actual_arrays.npz') as data:
    fig = plt.figure(figsize=(15, 10), facecolor='#f7f9fc')
    fig.text(.055, .95, '말씀하신 경계로 나눈 3개 gap', fontsize=25, weight='bold', color='#19364e')
    fig.text(.055, .91, '동일한 실제 전·후방 LiDAR 장면 · 원점에서 A → B → C → D 방향으로 수동 분할', color='#526c82')
    ax = fig.add_axes([.065, .15, .53, .70])
    grid = data['grid']
    view = np.zeros_like(grid)
    view[grid < 0] = 1
    view[data['map_fill'] & (grid == 0)] = 2
    view[grid >= 65] = 3
    ax.imshow(view, extent=(-4, 4, -4, 4), origin='lower', vmin=0, vmax=3,
              cmap=ListedColormap(['white', '#d6dee7', '#e2ecfc', '#283f54']), interpolation='nearest')
    for i, col in enumerate(['#bf7737', '#4e78a2']):
        hits = data[f'scan_{i}_hits'][data[f'scan_{i}_valid']]
        ax.scatter(hits[:, 0], hits[:, 1], s=2, c=col, alpha=.65)
    label_positions = [(0.64, -.76), (-1.13, -.73), (-.72, .22)]
    for i in range(3):
        a = np.linspace(angles[i], angles[i+1], 400)
        depth = np.interp(a, data['angles'], data['strict_depth'], period=2*np.pi)
        xy = depth[:, None] * np.column_stack((np.cos(a), np.sin(a)))
        ax.add_patch(Polygon(np.vstack(([0, 0], xy)), facecolor=colors[i], edgecolor='none', alpha=.24))
        ax.plot(xy[:, 0], xy[:, 1], color=colors[i], lw=2.2)
        ax.text(*label_positions[i], f'Gap {i+1}', ha='center', color=colors[i], fontsize=18,
                weight='bold', bbox=dict(facecolor='white', edgecolor='none', alpha=.88, pad=3))
    offsets = [(5, -25), (-35, -25), (-2, 13), (12, 18)]
    for name, point, offset in zip('ABCD', points, offsets):
        ax.plot([0, point[0]], [0, point[1]], '--', color='#23394a', lw=1.6)
        ax.scatter(*point, s=52, c='#23394a', edgecolors='white', zorder=8)
        ax.annotate(f'{name} ({point[0]:.1f}, {point[1]:.1f})', point, xytext=offset,
                    textcoords='offset points', fontsize=11, color='#19364e',
                    bbox=dict(facecolor='white', edgecolor='none', alpha=.85, pad=2))
    ax.add_patch(Circle((0, 0), .20, facecolor='#19364e', edgecolor='white', zorder=9))
    ax.annotate('', (.36, 0), (0, 0), arrowprops=dict(arrowstyle='->', color='#19364e', lw=2), zorder=10)
    ax.set(xlim=(-2.3, 2.4), ylim=(-2.3, 2.3), aspect='equal', xlabel='x [m] · 전방 +', ylabel='y [m] · 좌측 +')
    ax.grid(alpha=.14)
    for i, (title, detail) in enumerate([
        ('Gap 1 · A → B', '(2.0, −0.5) → (−0.4, −2.0)\n전방 우측부터 우측 후방까지\n방향: −14.0° → −101.3°'),
        ('Gap 2 · B → C', '(−0.4, −2.0) → (−2.0, 0.4)\n로봇 뒤쪽으로 크게 열린 구간\n방향: −101.3° → ±180° → +168.7°'),
        ('Gap 3 · C → D', '(−2.0, 0.4) → (0.0, 0.4)\n후방 좌측부터 좌측까지\n방향: +168.7° → +90.0°'),
    ]):
        y = .80-i*.20
        fig.text(.65, y, title, color=colors[i], fontsize=19, weight='bold')
        fig.text(.65, y-.045, detail, va='top', linespacing=1.7, color='#19364e', fontsize=12)
    fig.text(.65, .19, '점선: 지정하신 네 좌표를 향하는 경계\n색칠 외곽: 기존 몸체 진입 가능 깊이\n좌표점은 실제 장애물 hit를 뜻하지 않음',
             fontsize=11, color='#526c82', linespacing=1.6, va='top')
    fig.text(.055, .075, '실제 입력: mpc_chain_20260923_113151 +188.247981 s (b1_088) · 흰색: scan free / 연파랑: 지도 보완 / 진한색: 장애물',
             fontsize=10, color='#526c82')
    fig.text(.055, .04, '사용자 지정 분할을 확인하기 위한 그림입니다. 자동 거리 급변 검출·후보 선택·MPPI 재실행 결과가 아닙니다.',
             fontsize=11, color='#19364e')
    fig.savefig(OUT / 'user_three_gaps_actual_lidar.png', dpi=180)
    plt.close(fig)

(OUT / 'partition.json').write_text(json.dumps({
    'source': 'gap_v1_actual_flow/b1_088_actual_arrays.npz',
    'method': 'manual boundary rays specified by user, clockwise A-B-C-D',
    'points_xy': points.tolist(), 'unwrapped_angles_deg': np.rad2deg(angles).tolist(),
    'automatic_detection': False, 'mppi_rerun': False,
}, indent=2)+'\n')
print(OUT / 'user_three_gaps_actual_lidar.png')

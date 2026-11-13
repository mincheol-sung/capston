"""Explain body reach with two actual nearby directions from scene b2_100."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle

root = Path(__file__).resolve().parents[1]
data = np.load(root / 'docs/gap_multiscene_2026-11-04/disparity_partition_10_scenes/data/b2_100.npz')
out = root / 'docs/gap_depth_1m_2026-11-05'
out.mkdir(exist_ok=True)
font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font).get_name(),
                     'axes.unicode_minus': False, 'font.size': 13})
fig = plt.figure(figsize=(14, 10), facecolor='#f6f8fb')
ink = '#20364b'
fig.text(.045, .945, '실제 장면에서 로봇 몸체를 두 방향으로 밀어보면', fontsize=25, color=ink, weight='bold')
fig.text(.045, .902, '두 그림의 벽과 출발점은 같습니다. 이동 방향만 5° 다릅니다.', fontsize=15, color='#536b7e')
grid = data['grid']
view = np.zeros_like(grid)
view[grid < 0] = 1
view[grid >= 65] = 2
radius = .2
guard = radius + .05 + np.sqrt(2)*.04 + .01
for i, (deg, last, color, title) in enumerate([
        (123, 1.34, '#d94f48', 'A 방향: 1.32m까지만 이동 가능'),
        (128, 2., '#008e85', 'B 방향: 2.00m까지 이동 가능')]):
    ax = fig.add_axes([.065+i*.49, .27, .395, .55])
    ax.imshow(view, extent=(-4, 4, -4, 4), origin='lower', vmin=0, vmax=2,
              cmap=ListedColormap(['white', '#d8dfe7', '#34495e']), interpolation='nearest')
    direction = np.array([np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg))])
    for distance in [.45, .85, 1.34] if i else [.45, .85]:
        q = distance*direction
        ax.add_patch(Circle(q, radius, fc=color, ec=color, alpha=.13))
        if i and distance == 1.34:
            ax.add_patch(Circle(q, guard, fill=False, ec=color, ls='--', alpha=.8))
    q = last*direction
    ax.plot([0, q[0]], [0, q[1]], color=color, lw=2)
    ax.add_patch(Circle(q, radius, fc=color, ec='white', lw=1.5, alpha=.95, zorder=5))
    ax.add_patch(Circle(q, guard, fill=False, ec=color, lw=2, ls='--', zorder=6))
    ax.plot(*q, 'o', ms=4, color='white', zorder=7)
    ax.add_patch(Circle((0, 0), radius, fc=ink, ec='white', zorder=5))
    ax.text(0, -.27, '같은 출발점', ha='center', color=ink, fontsize=12)
    ax.plot(-.66, 1.42, marker='s', color='#f1aa2a', markersize=7, zorder=7)
    ax.annotate('같은 벽 모서리', (-.66, 1.42), xytext=(.12, 1.90), ha='center',
                fontsize=13, color=ink, arrowprops=dict(arrowstyle='->', color='#a77415', lw=1.6),
                bbox=dict(fc='white', ec='none', alpha=.95, pad=3))
    if i == 0:
        ax.annotate('옆 벽에 너무 가까워짐', q+np.array([0, .05]), xytext=(-1.6, .65),
                    ha='left', fontsize=13, color=color,
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5),
                    bbox=dict(fc='white', ec='none', alpha=.95, pad=3))
    else:
        ax.text(-1.65, .63, '벽을 비켜 지나감\n→ 더 멀리 갈 수 있음', color=color, fontsize=13,
                linespacing=1.6, bbox=dict(fc='white', ec='none', alpha=.9, pad=3))
    ax.set_title(title, fontsize=17, color=color, weight='bold', pad=15)
    ax.set(xlim=(-1.85, .55), ylim=(-.4, 2.25), aspect='equal', xlabel='x [m]', ylabel='y [m]')
    ax.grid(alpha=.1)
fig.text(.065, .195, '색칠한 원 = 로봇 몸체     점선 원 = 부딪히지 않도록 남기는 안전 여유', fontsize=15, color=ink)
fig.text(.065, .144, '이렇게 “몸체가 안전하게 움직일 수 있는 거리”가 d(θ)입니다.', fontsize=19, color=ink, weight='bold')
fig.text(.065, .095, 'A와 B의 이동 가능 거리가 다릅니다. 그래서 로봇 입장에서도 서로 다른 탐색 구역으로 나눌 이유가 있습니다.', fontsize=13, color=ink)
fig.text(.045, .051, '실제 b2_100 지도·strict_depth 사용. A 빨간 원은 허용 끝 1.32m 다음 검사 위치(1.34m). 점선은 격자 보정 여유도 포함.', fontsize=10, color='#536b7e')
fig.text(.045, .027, 'B는 최대 계산 거리 2m까지 확인됨. 그 지점에 벽이 있다는 뜻은 아닙니다. 라이다 beam 길이가 아니라 몸체의 이동을 표시합니다.', fontsize=10, color='#536b7e')
fig.savefig(out / '03_body_reach_simple.png', dpi=150)
print(out / '03_body_reach_simple.png')

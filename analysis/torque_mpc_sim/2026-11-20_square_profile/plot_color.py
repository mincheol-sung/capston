#!/usr/bin/python3
"""profile.npz → fig_square_color.png: 경로를 속력(순차 색)과 가속·감속(발산 색: 가속 파랑 / 감속 빨강 / 등속·정지 회색)으로,
아래에 속력–시간을 같은 가감속 색으로."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
ACC, DEC, ZERO, INK, MUTED = "#2a78d6", "#e34948", "#8a8985", "#0b0b0b", "#52514e"
z = np.load(os.path.join(HERE, "profile.npz"))
t, X, spd, at, S, corner = z["t"], z["X"], z["spd"], z["at"], z["S"], z["corner"]
T_path, T_total, t_start = float(z["T_path"]), float(z["T_total"]), z["t_start"]
col = np.where(at > 0, ACC, np.where(at < 0, DEC, ZERO))
SPD_CM = ListedColormap(plt.get_cmap("Oranges")(np.linspace(0.25, 1.0, 256)))

fig = plt.figure(figsize=(12, 9))
gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 1], hspace=0.28, wspace=0.28)
pts = X[:, :2]; segs = np.stack([pts[:-1], pts[1:]], 1); run = (t[:-1] > 0) & (t[:-1] < T_path)


def frame(ax, title):
    ax.set_aspect("equal"); ax.set_xlim(-0.3, 1.85); ax.set_ylim(-0.3, 1.85)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title(title, loc="left", fontsize=10, color=INK)
    for j in range(1, 4):                                    # 헤딩 회전 구간 경계 (꼭짓점 ±0.15 m) 표시
        m = (np.abs(S - j * 1.5) <= 0.15) & (spd > 0)
        ax.plot(pts[m, 0], pts[m, 1], color=INK, lw=7.5, alpha=0.12, solid_capstyle="butt", zorder=1)
    ax.annotate("출발·도착", (0, 0), (-0.25, -0.22), fontsize=8, color=MUTED)
    ax.annotate("", (0.45, -0.12), (0.15, -0.12), arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=0.8))
    ax.text(0.17, -0.2, "진행 방향 (반시계)", fontsize=8, color=MUTED)


# (a) 속력
ax = fig.add_subplot(gs[0, 0])
lc = LineCollection(segs[run], cmap=SPD_CM, norm=plt.Normalize(0.0, 1.5), lw=5, capstyle="butt", zorder=2)
lc.set_array(spd[:-1][run]); ax.add_collection(lc)
frame(ax, "(a) 속력")
cb = fig.colorbar(lc, ax=ax, fraction=0.045, pad=0.02); cb.set_label("속력 [m/s]")
for i in range(4):                                           # 변 가운데 최고속도
    k = int(np.argmax(np.where((S > i * 1.5) & (S < (i + 1) * 1.5), spd, 0)))
    off = [(0, 0.06), (-0.14, 0), (0, -0.1), (0.14, 0)][i]
    ax.text(pts[k, 0] + off[0], pts[k, 1] + off[1], f"{spd[k]:.2f}", fontsize=8, color=INK, ha="center")
for j in range(1, 4):
    c = pts[np.argmin(np.abs(S - j * 1.5))]
    ax.text(c[0] + (0.06 if c[0] > 0.7 else -0.06), c[1] + (0.06 if c[1] > 0.7 else -0.1), "0.10", fontsize=8,
            color=INK, ha="left" if c[0] > 0.7 else "right")

# (b) 가속·감속
ax = fig.add_subplot(gs[0, 1])
lc2 = LineCollection(segs[run], colors=col[:-1][run], lw=5, capstyle="butt", zorder=2); ax.add_collection(lc2)
frame(ax, "(b) 가속 · 감속  (접선 가속 ±1.5 m/s²)"); ax.set_ylabel("")
ax.legend(handles=[Line2D([], [], color=ACC, lw=5, label="가속 +1.5"), Line2D([], [], color=DEC, lw=5, label="감속 −1.5"),
                   Line2D([], [], color=ZERO, lw=5, label="등속 (변마다 3~7 mm)"),
                   Line2D([], [], color=INK, lw=7.5, alpha=0.12, label="헤딩 회전 구간 (꼭짓점 ±0.15 m)")],
          loc="center", fontsize=8, frameon=False)

# (c) 속력–시간, 가감속 색
ax = fig.add_subplot(gs[1, :])
spans, on = [], None
for n in range(len(t)):
    if corner[n] and on is None:
        on = t[n]
    if not corner[n] and on is not None:
        spans.append((on, t[n - 1])); on = None
for s0, s1 in spans:
    ax.axvspan(s0, s1, color=INK, alpha=0.07, lw=0)
ax.axvspan(T_path, T_total, color=INK, alpha=0.04, lw=0)
tseg = np.stack([np.stack([t[:-1], spd[:-1]], 1), np.stack([t[1:], spd[1:]], 1)], 1)
ax.add_collection(LineCollection(tseg, colors=col[:-1], lw=2.4))
ax.set_xlim(0, T_total + 0.15); ax.set_ylim(0, 1.75)
ax.set_xlabel("t [s]"); ax.set_ylabel("속력 [m/s]")
ax.set_title("(c) 속력 – 시간  (회색 띠 = 헤딩 회전 구간, 옅은 띠 = 끝에서 서서 +90° 정렬)", loc="left", fontsize=10, color=INK)
for i, ts in enumerate(t_start):
    ax.axvline(ts, color=MUTED, lw=0.5, ls=":")
    ax.text(ts + 0.04, 1.66, f"변 {i}  ({ts:.2f} s)", fontsize=8, color=MUTED)
ax.text(T_path + 0.05, 0.08, f"주행 끝 {T_path:.2f} s", fontsize=8, color=MUTED)
ax.text(T_total - 0.02, 0.08, f"{T_total:.2f} s", fontsize=8, color=MUTED, ha="right")
k = int(np.argmax(spd)); ax.text(t[k], spd[k] + 0.04, "1.50", fontsize=8, color=INK, ha="center")
ax.text(t_start[1], 0.14, "0.10", fontsize=8, color=INK, ha="center")
ax.text(0.12, 1.15, "+1.5 m/s²", color=ACC, fontsize=9)
ax.text(1.55, 1.08, "−1.5 m/s²", color=DEC, fontsize=9)

fig.suptitle("정사각형 기준  side 1.5 m · corner 0.1 m/s · accel 1.5 m/s² · v_max 1.5 m/s · tangent (yaw_blend 0.3 m)",
             x=0.08, y=0.97, ha="left", fontsize=11, color=INK)
fig.savefig(os.path.join(HERE, "fig_square_color.png"), dpi=140, bbox_inches="tight")
print("saved")

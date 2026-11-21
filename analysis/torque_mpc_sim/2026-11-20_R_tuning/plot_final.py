#!/usr/bin/python3
"""최종 검증 (정사각형 1.5 m, 가속 1.5 m/s², 최고 1.5 m/s, 바닥 μ 0.4): PP vs MPC B·C, R = w_dudt 0.01 · r_u 10 (지금 0.001 · 0.1)."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); F = os.path.join(HERE, "final")
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2, REF = "#0b0b0b", "#52514e", "#a9a8a2"
S = json.load(open(os.path.join(F, "final_summary.json")))
SER = [("pp", "퓨어 퍼슛 + 바퀴 PI", "#1baf7a"), ("mpcB_wd0.01_ru10", "MPC B", "#2a78d6"),
       ("mpcC04_wd0.01_ru10", "MPC C (μ_hat·c_s 0.32)", "#eb6834"), ("mpcC05_wd0.01_ru10", "MPC C (μ_hat·c_s 0.40 = 바닥)", "#6250d6")]
fig = plt.figure(figsize=(13, 6.2))
ax0 = fig.add_axes([0.03, 0.08, 0.40, 0.80])
ax0.plot([0, 1.5, 1.5, 0, 0], [0, 0, 1.5, 1.5, 0], color=REF, lw=7, alpha=0.45, solid_capstyle="round")
for key, name, col in SER:
    raw = np.load(os.path.join(F, f"{key}_r1.npz"))["raw"]
    ax0.plot(raw[:, 1], raw[:, 2], color=col, lw=1.8, label=name)
ax0.set_aspect("equal"); ax0.set_xlim(-0.4, 1.95); ax0.set_ylim(-0.35, 1.95)
ax0.legend(loc="center", frameon=False, fontsize=9)
ax0.set_title("궤적 (반복 1)", loc="left", color=INK)
mets = [("contour", "윤곽 오차 RMS [mm]", "{:.0f}"), ("spin", "헛돎 |λ|>0.5 시간 [%]", "{:.0f}"), ("lmax", "λ max", "{:.2f}")]
for k, (m, title, fmt) in enumerate(mets):
    ax = fig.add_axes([0.62, 0.08 + (2 - k) * 0.295, 0.35, 0.215])
    for i, (key, name, col) in enumerate(SER):
        vals = [r[m] for r in S[key]]
        ax.barh(i, np.mean(vals), height=0.62, color=col, zorder=2)
        ax.scatter(vals, [i] * len(vals), s=14, color=INK, zorder=3)
        ax.text(max(vals) * 1.02 + 0.01, i, fmt.format(np.mean(vals)), va="center", fontsize=9, color=INK)
    ax.set_yticks(range(len(SER)), [s[1] for s in SER], fontsize=8.5); ax.invert_yaxis()
    ax.set_title(title, loc="left", color=INK, fontsize=10); ax.grid(axis="y", visible=False)
    ax.set_xlim(0, ax.get_xlim()[1] * 1.15)
fig.suptitle("정사각형 1.5 m · 가속 1.5 m/s² · 최고 1.5 m/s · 바닥 μ 0.4 — MPC 가중치 w_dudt 0.01 · r_u 10 (Q 고정, use_w). 막대 = 평균, 점 = 각 회 (2~3 회)",
             x=0.01, ha="left", color=INK2, fontsize=9.5)
fig.savefig(os.path.join(HERE, "fig_final_square.png"), dpi=130)
print("saved")

#!/usr/bin/python3
"""summary.json (plot_real_square.py) → 가속 1.5 · 1.5 m/s 판 요약 막대 (퓨어 퍼슛 1 판, MPC C 2 판)."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
S = json.load(open(os.path.join(HERE, "summary.json")))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK = "#0b0b0b"
runs = [("171640", "퓨어 퍼슛", "#1baf7a"), ("171531", "MPC C (1)", "#eb6834"), ("171735", "MPC C (2)", "#6250d6")]
mets = [("contour", "윤곽 오차 RMS [mm]\n(경로 모양, 작을수록 좋음)", 1, "{:.0f}"),
        ("et_mean", "진행 방향 평균 오차 [mm]\n(− = 기준보다 뒤처짐)", 1e3, "{:+.0f}"),
        ("yaw", "요 오차 RMS [°]", 1, "{:.1f}"),
        ("slip", "슬립 RMS x·y 합 [m/s]\n(SLAM 잡음 바닥 ≈ 0.14)", 1, "{:.3f}"),
        ("slip03", "슬립 > 0.3 m/s 시간 [%]", 1, "{:.0f}"),
        ("dtau", "틱간 Δτ RMS [N·m]\n(토크 떨림)", 1, "{:.3f}")]
for b, *_ in runs:
    S[b]["slip"] = float(np.hypot(S[b]["slip_x"], S[b]["slip_y"]))
fig, axes = plt.subplots(1, len(mets), figsize=(15, 3.9))
for ax, (k, title, sc, fmt) in zip(axes, mets):
    for i, (b, name, col) in enumerate(runs):
        v = S[b][k] * sc
        ax.bar(i, v, color=col, width=0.7, zorder=2)
        ax.text(i, v + (0.02 * abs(v) if v >= 0 else -0.02 * abs(v)), fmt.format(v), ha="center", va="bottom" if v >= 0 else "top", fontsize=9, color=INK)
    ax.set_xticks(range(len(runs)), [r[1] for r in runs], fontsize=8.5, rotation=0)
    ax.set_title(title, loc="left", fontsize=9.5, color=INK); ax.grid(axis="x", visible=False)
    lo, hi = ax.get_ylim(); ax.set_ylim(min(lo, 0) * 1.15, hi * 1.15 if hi > 0 else 0)
    ax.axhline(0, color="#b8b7b0", lw=0.8)
fig.suptitle("실차 정사각형 1.5 m · 가속 1.5 m/s² · 최고 1.5 m/s · SLAM 기준 (2026-11-20, 판마다 1 회)", x=0.01, ha="left", color="#52514e", fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig(os.path.join(HERE, "fig_real_summary.png"), dpi=130)
print("saved")

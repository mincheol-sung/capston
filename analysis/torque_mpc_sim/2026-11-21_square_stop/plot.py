#!/usr/bin/python3
"""fig_square_stop.png — pass vs stop, r1 판: 경로 (전체 + 꼭짓점 (1.5, 0) 확대), 위치 오차 |p − p_r| vs 시간."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analyze import R, CORNERS, contour

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
COL = {"PP": "#2a78d6", "B": "#eb6834", "C": "#1baf7a"}; NAME = {"PP": "A 퓨어 퍼슛", "B": "B MPC motor_only", "C": "C MPC friction"}
INK, MUTED = "#0b0b0b", "#52514e"
fig, ax = plt.subplots(2, 3, figsize=(14, 8.6), gridspec_kw=dict(width_ratios=[1, 1, 0.8], hspace=0.3, wspace=0.25))
for col, mode in enumerate(("pass", "stop")):
    a, az, ae = ax[0, col], ax[1, 2] if col else ax[0, 2], ax[1, col]
    sq = np.vstack([CORNERS, CORNERS[:1]])
    for A_ in (a, az):
        A_.plot(sq[:, 0], sq[:, 1], color=INK, lw=0.8, ls="--", zorder=1)
    for g in ("PP", "B", "C"):
        f = f"{R}/{g}_{mode}_r1.npz"
        if not os.path.exists(f):
            continue
        raw = np.load(f)["raw"]; t = raw[:, 0]
        vr = np.hypot(raw[:, 7], raw[:, 8]); act = (vr > 0.02) | (np.abs(raw[:, 9]) > 0.02)
        i0 = np.argmax(act); t0 = t[i0]
        a.plot(raw[:, 1], raw[:, 2], color=COL[g], lw=1.5, label=NAME[g])
        az.plot(raw[:, 1], raw[:, 2], color=COL[g], lw=1.6, label=NAME[g])
        e = np.hypot(raw[:, 1] - raw[:, 4], raw[:, 2] - raw[:, 5])
        ae.plot(t - t0, 1e3 * e, color=COL[g], lw=1.3, label=NAME[g])
        # 꼭짓점 회전 구간 (기준이 서서 돈다) 표시: stop 만
        if mode == "stop" and g == "PP":
            rot = (vr <= 0.02) & (np.abs(raw[:, 9]) > 0.02)
            on = None
            for n in range(len(t)):
                if rot[n] and on is None:
                    on = t[n] - t0
                if not rot[n] and on is not None:
                    ae.axvspan(on, t[n - 1] - t0, color=INK, alpha=0.07, lw=0); on = None
    a.set_aspect("equal"); a.set_xlim(-0.25, 1.85); a.set_ylim(-0.25, 1.85); a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_title(f"꼭짓점 {mode}  " + ("(0.1 m/s 로 지나며 헤딩 회전)" if mode == "pass" else "(서서 0.1 s → +90° 1.0 s → 0.1 s)"),
                loc="left", fontsize=10, color=INK)
    ae.set_xlabel("주행 시작부터 t [s]" + ("   (회색 띠 = 꼭짓점 제자리 회전)" if mode == "stop" else ""))
    ae.set_ylabel("위치 오차 |p − p_r| [mm]"); ae.set_title(f"시간 추종 오차 — {mode}", loc="left", fontsize=10, color=INK)
    az.set_aspect("equal"); az.set_xlim(1.2, 1.75); az.set_ylim(-0.2, 0.35)
    az.set_title(f"꼭짓점 (1.5, 0) 확대 — {mode}", loc="left", fontsize=10, color=INK)
ax[0, 0].legend(loc="center", fontsize=8.5, frameon=False)
ymax = max(ax[1, 0].get_ylim()[1], ax[1, 1].get_ylim()[1])
for a in (ax[1, 0], ax[1, 1]):
    a.set_ylim(0, ymax)
fig.suptitle("정사각형 1.5 m · accel 1.5 · v_max 1.5 · tangent · 시뮬 바닥 μ 0.45 — 점선 = 기준 정사각형, r1 판", x=0.06, y=0.97,
             ha="left", fontsize=11, color=INK)
fig.savefig(os.path.join(HERE, "fig_square_stop.png"), dpi=140, bbox_inches="tight")
print("saved")

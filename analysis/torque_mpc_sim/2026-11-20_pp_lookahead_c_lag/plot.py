#!/usr/bin/python3
"""그림: (1) PP 룩어헤드 비례 vs 상수 0.3 m (궤적·토크), (2) C 뒤처짐 해결책 비교 (진행 방향 평균 오차·윤곽)."""
import importlib.util, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, "runs"); SIDE = 1.5
exec(open(os.path.join(HERE, "analyze.py")).read().split("groups = [")[0].replace('os.path.dirname(os.path.abspath(__file__))', repr(HERE)))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False, "axes.edgecolor": "#b8b7b0",
                     "axes.labelcolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e6e5e0", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2, REF = "#0b0b0b", "#52514e", "#a9a8a2"
# (1) PP
fig = plt.figure(figsize=(13, 5.6))
ax0 = fig.add_axes([0.04, 0.10, 0.36, 0.80]); ax1 = fig.add_axes([0.47, 0.10, 0.51, 0.80])
ax0.plot([0, SIDE, SIDE, 0, 0], [0, 0, SIDE, SIDE, 0], color=REF, lw=7, alpha=0.45, solid_capstyle="round")
for lab, name, col in [("pp_var_a1.5_r1", "비례 L_d (0.15~0.60 m)", "#a9a8a2"), ("ppL0.3_a1.5", "상수 L_d 0.30 m (지금)", "#1baf7a")]:
    z = np.load(f"{R}/{lab}.npz"); raw, tau, tt = z["raw"], z["tau"], z["tau_t"]; m = met(lab)
    ax0.plot(raw[:, 1], raw[:, 2], color=col if col != "#a9a8a2" else "#52514e", lw=1.8, ls="-" if "상수" in name else "--",
             label=f"{name}: 윤곽 {m['contour']:.0f} mm")
    vr = np.hypot(raw[:, 7], raw[:, 8]); t0 = raw[np.argmax(vr > 0.02), 0]
    ax1.plot(tt - t0, np.abs(tau).max(axis=1), color=col if col != "#a9a8a2" else "#52514e", lw=1.3, ls="-" if "상수" in name else "--",
             label=f"{name}: τ max {m['tau']:.2f}, Δτ RMS {m['dtau']:.3f}")
ax0.set_aspect("equal"); ax0.set_xlim(-0.3, 1.9); ax0.set_ylim(-0.3, 1.9); ax0.legend(loc="center", frameon=False, fontsize=8.5)
ax0.set_title("퓨어 퍼슛 궤적 (가속 1.5 · 1.5 m/s)", loc="left", color=INK)
ax1.set_xlim(-0.5, 8.5); ax1.set_ylabel("최대 바퀴 토크 [N·m]"); ax1.set_xlabel("주행 시작 뒤 시간 [s]")
ax1.legend(loc="upper right", frameon=False, fontsize=8.5); ax1.set_title("최대 바퀴 토크 (비례: 출발 0.72, 꼭짓점 0.35~0.37 · 상수: 꼭짓점 0.35~0.43)", loc="left", color=INK)
fig.suptitle("시뮬 정사각형 1.5 m · 바닥 μ 0.45 · 참값 — 퓨어 퍼슛 룩어헤드 비례 vs 상수", x=0.01, ha="left", color=INK2, fontsize=10)
fig.savefig(os.path.join(HERE, "fig_pp_lookahead.png"), dpi=130); plt.close(fig)
# (2) C
cases = [("mpcC_a1.5_r1", "지금\n(측정 μ_hat·c_s 0.8,\nr_u 10, 기준 가속 1.5)"), ("mpcC_a1.0_r1", "기준 가속\n1.5 → 1.0"),
         ("mpcC_ru1_a1.5", "r_u\n10 → 1"), ("mpcC_muhat0.45_a1.5", "μ_hat 바퀴 같게 0.45\n(FR 병목 없앰)"),
         ("mpcC_muhat0.5625_a1.5", "μ_hat·c_s = 바닥\n(여유 0)"), ("mpcC_ru1_muhat0.5625_a1.5", "μ_hat·c_s = 바닥\n+ r_u 1")]
M = [met(l) for l, _ in cases]
fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
for a, (k, title, fmt) in zip(ax, [("et", "진행 방향 평균 오차 [mm] (− = 뒤처짐)", "{:+.0f}"), ("contour", "윤곽 오차 RMS [mm]", "{:.0f}"), ("vmax", "최고 속도 [m/s] (기준 1.5)", "{:.2f}")]):
    vals = [m[k] for m in M]
    a.barh(range(len(cases)), vals, color=["#a9a8a2"] + ["#eb6834"] * 5, height=0.62, zorder=2)
    for i, v in enumerate(vals):
        a.text(v + (0.02 * max(abs(x) for x in vals)) * (1 if v >= 0 else -1), i, fmt.format(v), va="center", ha="left" if v >= 0 else "right", fontsize=9, color=INK)
    a.set_yticks(range(len(cases)), [c[1] for c in cases] if a is ax[0] else [""] * len(cases), fontsize=8.5); a.invert_yaxis()
    a.set_title(title, loc="left", color=INK, fontsize=10); a.grid(axis="y", visible=False)
    lo, hi = a.get_xlim(); a.set_xlim(lo * 1.25 if lo < 0 else 0, hi * 1.2 if hi > 0 else 0)
fig.suptitle("시뮬 MPC C 정사각형 (최고 1.5 m/s, 바닥 μ 0.45, 참값) — 뒤처짐 원인 하나씩 바꿔 보기 (판마다 1 회)", x=0.01, ha="left", color=INK2, fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig(os.path.join(HERE, "fig_c_lag_fixes.png"), dpi=130)
print("saved")

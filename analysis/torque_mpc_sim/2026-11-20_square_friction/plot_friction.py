#!/usr/bin/python3
"""정사각형 마찰 초과 비교 (2026-11-20): 한 변 1.5 m, tangent, 꼭짓점 0.1 m/s, 가속 2.5 m/s², 최고 1.5 m/s.
   바닥 μ 0.3 (필요 μ 0.360 → 초과) / 0.4. C 는 μ̂ = 바닥 μ. 판마다 3 회. runs/*.npz (diag_rec.py)."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, "runs")
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2, REF = "#0b0b0b", "#52514e", "#a9a8a2"
SER = {"pp": ("퓨어 퍼슛 + 바퀴 PI", "#1baf7a"), "mpcB": ("MPC B (모터 상한)", "#2a78d6"), "mpcC": ("MPC C (마찰 상한)", "#eb6834")}
js = lambda lab: json.loads(open(os.path.join(R, f"{lab}.json")).read().strip().splitlines()[-1])
npz = lambda lab: np.load(os.path.join(R, f"{lab}.npz"))

# ── 그림 1: 궤적 (μ 0.3 / 0.4, 반복 1)
fig, axes = plt.subplots(1, 2, figsize=(12, 6.6))
for ax, mu in zip(axes, ("0.3", "0.4")):
    ax.plot([0, 1.5, 1.5, 0, 0], [0, 0, 1.5, 1.5, 0], color=REF, lw=6, alpha=0.45, solid_capstyle="round")
    for c, (name, col) in SER.items():
        z = npz(f"{c}_mu{mu}_r1"); raw = z["raw"]; s = js(f"{c}_mu{mu}_r1")
        ax.plot(raw[:, 1], raw[:, 2], color=col, lw=1.8, label=name + ("  (이탈 정지)" if "track_error" in (s.get("fault") or "") else ""))
        if "track_error" in (s.get("fault") or ""):
            ax.plot(raw[-1, 1], raw[-1, 2], marker="X", ms=10, color=col, mec="#fcfcfb")
    ax.set_aspect("equal"); ax.set_xlim(-1.3, 2.8); ax.set_ylim(-1.3, 2.8)
    ax.set_title(f"바닥 μ {mu}  (기준 가속이 요구하는 μ 0.360{' → 초과' if mu == '0.3' else ''})", loc="left", color=INK, fontsize=10.5)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
axes[0].legend(loc="lower right", frameon=False, fontsize=9)
fig.suptitle("정사각형 1.5 m · 가속 2.5 m/s² · 최고 1.5 m/s · 꼭짓점 0.1 m/s — 시뮬 참값 궤적 (반복 1)", x=0.01, ha="left", color=INK, fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(os.path.join(HERE, "fig_friction_traj.png"), dpi=130); plt.close(fig)

# ── 그림 2: 슬립률 시계열 (μ 0.3, 반복 1) — 패널마다 한 제어기, 같은 y 축
fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True, gridspec_kw={"height_ratios": [0.8, 1, 1, 1]})
z = npz("mpcC_mu0.3_r1"); raw = z["raw"]; vr = np.hypot(raw[:, 7], raw[:, 8]); t0 = raw[np.argmax(vr > 0.02), 0]
axes[0].plot(raw[:, 0] - t0, vr, color=INK2, lw=2); axes[0].set_ylabel("기준 속도 [m/s]")
for ax, (c, (name, col)) in zip(axes[1:], SER.items()):
    z = npz(f"{c}_mu0.3_r1"); lam = z["lam"]; raw = z["raw"]
    vr = np.hypot(raw[:, 7], raw[:, 8]); t0c = raw[np.argmax(vr > 0.02), 0]
    ax.plot(lam[:, 0] - t0c, np.nanmax(np.abs(lam[:, 1:5]), axis=1), color=col, lw=1.4)
    ax.axhline(0.1, color=INK2, lw=0.9, ls="--")
    s = js(f"{c}_mu0.3_r1")
    ax.set_title(f"{name} — 주행 중 |λ|>0.1 {100*s['slip_frac10']:.0f} %, λ max {s['slip_max']:.2f}"
                 + ("  (이탈로 정지)" if "track_error" in (s.get("fault") or "") else ""), loc="left", color=INK, fontsize=10)
    ax.set_ylabel("바퀴 최대 |λ|"); ax.set_ylim(0, 2.1)
axes[-1].set_xlabel("주행 시작 뒤 시간 [s]"); axes[1].text(0.1, 0.13, "|λ| = 0.1", color=INK2, fontsize=8.5)
axes[0].set_xlim(-0.5, 12)
axes[0].set_title("바닥 μ 0.3 (마찰 초과) — 참값 슬립률, 반복 1", loc="left", color=INK, fontsize=11)
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_friction_slip.png"), dpi=130); plt.close(fig)

# ── 그림 3: 요약 (3 회 반복 점 + 평균 막대)
fig, axes = plt.subplots(1, 3, figsize=(13, 4.3))
mets = [("slip_frac10", 100, "주행 중 |λ|>0.1 시간 [%]"), ("slip_max", 1, "슬립률 최대 |λ|"), ("moving_rms_mm", 1, "주행 위치 오차 RMS [mm]")]
for ax, (key, k, title) in zip(axes, mets):
    for gi, mu in enumerate(("0.3", "0.4")):
        for si, (c, (name, col)) in enumerate(SER.items()):
            vals = [js(f"{c}_mu{mu}_r{r}")[key] * k for r in (1, 2, 3)]
            x = gi + (si - 1) * 0.27
            ax.bar(x, np.mean(vals), width=0.25, color=col, label=name if gi == 0 else None, zorder=2)
            ax.scatter([x] * 3, vals, s=14, color=INK, zorder=3)
            div = sum("track_error" in (js(f"{c}_mu{mu}_r{r}").get("fault") or "") for r in (1, 2, 3))
            ax.text(x, np.mean(vals) * 1.02 + (0.02 if k == 1 and key == "slip_max" else 1), ("이탈 " if div else "") + f"{np.mean(vals):.0f}" if key != "slip_max" else f"{np.mean(vals):.2f}",
                    ha="center", va="bottom", fontsize=8.5, color=INK)
    ax.set_xticks([0, 1], ["바닥 μ 0.3 (초과)", "바닥 μ 0.4"]); ax.grid(axis="x", visible=False)
    ax.set_title(title, loc="left", color=INK, fontsize=10.5); ax.set_ylim(0, ax.get_ylim()[1] * 1.1)
fig.legend(*axes[0].get_legend_handles_labels(), frameon=False, loc="upper right", ncol=3, fontsize=9)
fig.suptitle("막대 = 3 회 평균, 점 = 각 회. MPC B 는 6/6 판 이탈 (기준에서 1.5 m → fault_monitor 정지, 그때까지 값)", x=0.01, ha="left", color=INK2, fontsize=9.5)
fig.tight_layout(rect=(0, 0, 1, 0.9)); fig.savefig(os.path.join(HERE, "fig_friction_summary.png"), dpi=130)
print("saved")

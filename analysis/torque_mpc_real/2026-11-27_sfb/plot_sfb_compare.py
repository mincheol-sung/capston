"""circles8 friction MPC: 슬립 되먹임 끔 / ik 0.25 / lqr 0.25 바퀴별 슬립 시간 비율 + 11-26 끔 여섯 판 범위 (같은 조건의 판 사이 흔들림).
python3 -I plot_sfb_compare.py <npz 폴더>"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
from scipy.signal import butter, filtfilt

HERE = sys.argv[1]
W = ("FL", "FR", "RL", "RR")
GROUPS = [
    ("되먹임 끔 · 11-26 여섯 판", ["184758_s2_mpc_r1", "184937_s2_mpc_r2", "185209_s2_mpc_r3", "185937_s2_mpc_r4", "190248_s2_mpc_r5", "190352_s2_mpc_r6"], "#c9c8c3"),
    ("되먹임 끔 · 11-27 (sfbw r3)", ["165139_sfbw_lqr025_r3"], "#6f6e69"),
    ("ik 0.25 (sfbw r2)", ["164956_sfbw_lqr025_r2"], "#eb6834"),
    ("lqr 0.25 (sfbw r1)", ["164829_sfbw_lqr025_r1"], "#2a78d6"),
]
INK, INK2, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb"


def metrics(name):
    A = np.load(os.path.join(HERE, name + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(t)))), L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05); X = np.abs(Lf[w])
    return dict(p50=100 * (X > 0.5).mean(0), p25=100 * (X > 0.25).mean(0), m50=100 * np.mean(X.max(1) > 0.5), m25=100 * np.mean(X.max(1) > 0.25))


R = [(lab, [metrics(n) for n in names], col) for lab, names, col in GROUPS]
for lab, ms, _ in R:
    for key in ("p50", "p25"):
        a = np.array([m[key] for m in ms])
        rng = (lambda i: f" ({a[:, i].min():.1f}~{a[:, i].max():.1f})") if len(ms) > 1 else (lambda i: "")
        print(f"{lab:28s} {'>50' if key == 'p50' else '>25'} % " + "  ".join(f"{W[i]} {a[:, i].mean():5.1f}{rng(i)}" for i in range(4)))
    mm = np.array([m["m50"] for m in ms])
    print(f"{'':28s} 네 바퀴 중 최대 >50 % {mm.mean():.1f}" + (f" ({mm.min():.1f}~{mm.max():.1f})" if len(ms) > 1 else ""))

fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0])
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10.5, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "figure.facecolor": SURF, "axes.facecolor": SURF})
fig, axes = plt.subplots(1, 3, figsize=(15.5, 5.8), dpi=150, gridspec_kw=dict(width_ratios=[4, 4, 1.35], wspace=0.22))
bw, x, ng = 0.2, np.arange(4), len(R)
for ax, key, title, ymax in ((axes[0], "p50", "|슬립률| > 50 % 인 시간 비율", 40), (axes[1], "p25", "|슬립률| > 25 % 인 시간 비율 (보조)", 50)):
    for j, (lab, ms, col) in enumerate(R):
        a = np.array([m[key] for m in ms]); mean = a.mean(0)
        xs = x + (j - (ng - 1) / 2) * (bw + 0.015)
        ax.bar(xs, mean, width=bw, color=col, edgecolor=SURF, linewidth=1.0, zorder=2)
        top = mean.copy()
        if len(ms) > 1:
            ax.vlines(xs, a.min(0), a.max(0), color=INK, lw=1.1, zorder=3)
            ax.hlines(a.min(0), xs - 0.045, xs + 0.045, color=INK, lw=1.1, zorder=3)
            ax.hlines(a.max(0), xs - 0.045, xs + 0.045, color=INK, lw=1.1, zorder=3)
            top = a.max(0)
        for i in range(4):
            ax.text(xs[i], top[i] + ymax * 0.012, f"{mean[i]:.1f}", ha="center", va="bottom", fontsize=8, color=INK2, zorder=4)
    ax.set_xticks(x); ax.set_xticklabels(["FL (앞 왼)", "FR (앞 오)", "RL (뒤 왼)", "RR (뒤 오)"])
    ax.set_ylim(0, ymax); ax.set_ylabel("창 안 시간 중 %"); ax.grid(axis="y", color=GRID, lw=0.6, zorder=0)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_title(title, loc="left", fontsize=11, color=INK, pad=8)
ax = axes[2]
for j, (lab, ms, col) in enumerate(R):
    mm = np.array([m["m50"] for m in ms])
    ax.bar(j, mm.mean(), width=0.7, color=col, edgecolor=SURF, zorder=2)
    top = mm.mean()
    if len(ms) > 1:
        ax.vlines(j, mm.min(), mm.max(), color=INK, lw=1.1, zorder=3)
        ax.hlines([mm.min(), mm.max()], j - 0.12, j + 0.12, color=INK, lw=1.1, zorder=3); top = mm.max()
    ax.text(j, top + 1.0, f"{mm.mean():.1f}", ha="center", va="bottom", fontsize=8.5, color=INK2)
ax.set_xticks([]); ax.set_ylim(0, 80); ax.grid(axis="y", color=GRID, lw=0.6, zorder=0)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
ax.set_title("네 바퀴 중 최대 > 50 %", loc="left", fontsize=11, color=INK, pad=8)
h = [Patch(color=c) for _, _, c in R] + [plt.Line2D([], [], color=INK, lw=1.1)]
fig.text(0.05, 1.0, "circles8 (width 2.0 · speed 1.0 · accel 1.0) · MPC · limit_mode friction — 슬립률 10 Hz 1차 Butterworth filtfilt, "
         "창 = 무장 중 · 추정 속력 > 0.05 m/s. 11-27 세 판은 한 판씩, 이름이 아니라 params 덤프의 실제 조건",
         fontsize=9, color=MUTED, va="top")
fig.legend(h, [lab for lab, _, _ in R] + ["11-26 여섯 판 최소~최대"], loc="upper left", bbox_to_anchor=(0.045, 0.965), ncol=5, frameon=False,
           fontsize=10, labelcolor=INK2)
fig.subplots_adjust(top=0.8)
out = os.path.join(HERE, "slip_sfbw_compare.png")
fig.savefig(out, bbox_inches="tight", facecolor=SURF)
print("저장:", out)

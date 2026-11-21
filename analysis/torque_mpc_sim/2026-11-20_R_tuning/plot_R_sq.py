#!/usr/bin/python3
"""정사각형 (가속 1.5, μ 0.4, 최고 1.5 m/s) R 스윕 히트맵: B·C × (윤곽 RMS, 헛돎 |λ|>0.5 시간, λ max). 입력 summary.json"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
D, OUT = sys.argv[1], sys.argv[2]
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9.5, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK = "#0b0b0b"
rows = json.load(open(os.path.join(D, "summary.json")))
WD, RU = [0.0001, 0.0003, 0.001, 0.003, 0.01], [0.01, 0.1, 1, 10]
MET = [("contour", "윤곽 오차 RMS [mm]", "{:.0f}"), ("spin", "헛돎 |λ|>0.5 시간 [%]", "{:.0f}"), ("lmax", "λ max", "{:.2f}")]
LIM = [("motor_only", "MPC B (모터 상한)"), ("friction", "MPC C (마찰 상한)")]
CUR, CAND = (0.001, 0.1), [(0.003, 10), (0.01, 10)]
fig, axes = plt.subplots(2, 3, figsize=(13, 7.4))
for i, (lim, lname) in enumerate(LIM):
    for j, (key, title, fmt) in enumerate(MET):
        ax = axes[i][j]
        Z = np.full((len(WD), len(RU)), np.nan); div = np.zeros_like(Z, bool)
        for r in rows:
            if r["lim"] == lim:
                Z[WD.index(r["wd"]), RU.index(r["ru"])] = r[key]; div[WD.index(r["wd"]), RU.index(r["ru"])] = r["div"]
        allv = np.array([r[key] for r in rows if not r["div"]])
        lo, hi = np.nanmin(allv), np.nanpercentile(allv, 95)
        ax.imshow(np.clip(Z, lo, hi), cmap="Blues", vmin=lo, vmax=hi, aspect="auto")
        for a in range(len(WD)):
            for b in range(len(RU)):
                v = Z[a, b]
                if np.isfinite(v):
                    dark = (min(v, hi) - lo) / (hi - lo + 1e-12) > 0.55
                    ax.text(b, a, "이탈" if div[a, b] else fmt.format(v), ha="center", va="center", fontsize=8.5,
                            color="#fcfcfb" if dark else INK)
        ax.add_patch(plt.Rectangle((RU.index(CUR[1]) - 0.5, WD.index(CUR[0]) - 0.5), 1, 1, fill=False, ec=INK, lw=1.6, ls="--"))
        for wd, ru in CAND:
            ax.add_patch(plt.Rectangle((RU.index(ru) - 0.5, WD.index(wd) - 0.5), 1, 1, fill=False, ec="#e34948", lw=2.2))
        ax.set_xticks(range(len(RU)), [f"{r:g}" for r in RU]); ax.set_yticks(range(len(WD)), [f"{w:g}" for w in WD])
        if i == 1: ax.set_xlabel("r_u (입력 편차 벌점)")
        if j == 0: ax.set_ylabel("w_dudt (변화율 벌점)")
        ax.set_title(f"{lname} — {title}", loc="left", color=INK, fontsize=10)
fig.suptitle("정사각형 1.5 m · 가속 1.5 m/s² · 최고 1.5 m/s · 바닥 μ 0.4 · use_w · Q = [5,5,2.5,2,2,1] 고정.  점선 = 지금, 빨간 칸 = 후보.  "
             "같은 조건 PP: 윤곽 71 mm, 헛돎 29 %, λ max 1.47", x=0.01, ha="left", color="#52514e", fontsize=9.5)
fig.tight_layout(rect=(0, 0, 1, 0.95))
fig.savefig(os.path.join(OUT, "fig_R_sweep_square.png"), dpi=125)
print("saved")

#!/usr/bin/python3
"""topp_*.npz → fig_topp.png: 경로 호길이에 따른 속도 (시간 최적 vs 같은 랩타임 등속) 와 C 상한 비."""
import glob, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
CM, CP, INK, LIM, MUTED = "#1baf7a", "#2a78d6", "#0b0b0b", "#d03b3b", "#52514e"
cases = [("topp_circles8_3.0.npz", "circles8 지름 1.5 m"), ("topp_gerono_width.npz", "gerono 8자 3.0 × 1.5 m")]
fig, ax = plt.subplots(2, 2, figsize=(13, 7.4), gridspec_kw=dict(hspace=0.38, wspace=0.2))
for c, (f, name) in enumerate(cases):
    z = np.load(f); s = z["s"]
    a = ax[0, c]
    a.plot(s, z["v"], color=CM, lw=1.6, label=f"시간 최적 (C 상한 안) — 랩 {float(z['T']):.2f} s")
    a.plot(s, z["vv"], color=CP, lw=1.4, label=f"같은 랩타임 등속 {float(z['vc']):.2f} m/s")
    a.plot(s, np.minimum(z["vlim"], 1.6), color=MUTED, lw=0.8, ls=":", label="속도 한계 곡선 (가속 0)")
    a.set_ylim(0, 1.7); a.set_xlabel("호길이 s [m]"); a.set_ylabel("속도 [m/s]"); a.legend(loc="lower center", fontsize=8, frameon=False)
    a.set_title(f"({'ab'[c]}1) {name} — 속도", loc="left", fontsize=10, color=INK)
    b = ax[1, c]
    b.plot(s, np.minimum(z["ro"], 3), color=CM, lw=1.4, label="시간 최적")
    b.plot(s, np.minimum(z["rc"], 3), color=CP, lw=1.2, label="같은 랩타임 등속")
    b.axhline(1.0, color=LIM, lw=0.9, ls="--"); b.text(0.02, 1.03, "C 토크 상한 (μ_hat × 0.8)", color=LIM, fontsize=8, transform=b.get_yaxis_transform())
    b.axhline(1.25, color=LIM, lw=0.9, ls=":"); b.text(0.02, 1.28, "μ_hat (바닥 마찰 추정)", color=LIM, fontsize=8, transform=b.get_yaxis_transform())
    b.set_ylim(0, 3.05); b.set_xlabel("호길이 s [m]"); b.set_ylabel("필요 토크 / C 상한 (최대 바퀴, 3 에서 자름)")
    b.legend(loc="upper right", fontsize=8, frameon=False)
    b.set_title(f"({'ab'[c]}2) 필요 토크 — 등속은 {100*np.mean(z['rc'] > 1.25):.0f} % 구간에서 μ_hat 넘음", loc="left", fontsize=10, color=INK)
fig.suptitle("같은 랩타임: 마찰을 아는 속도 배분 (시간 최적) vs 등속 — 해석값, 노드 0.2 s 차분·교차점 완화 없음", x=0.05, y=0.99, ha="left", fontsize=11, color=INK)
fig.savefig("fig_topp.png", dpi=130, bbox_inches="tight"); print("saved")

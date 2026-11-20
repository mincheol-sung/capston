#!/usr/bin/python3
"""퓨어 퍼슛 꼭짓점 토크 튐 — runs/ppdbg_a1.0.npz (pp_dbg_rec.py: /pure_pursuit/status, /wheel_pi/status 원시)."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2 = "#0b0b0b", "#52514e"
C1, C2 = "#2a78d6", "#eb6834"
z = np.load(os.path.join(HERE, "runs", "ppdbg_a1.0.npz"))
pp, pi = z["pp"], z["pi"]
k = int(np.argmax(np.abs(pi[:, 9:13]).max(axis=1)))       # 가장 큰 토크 행
k = int(np.where(np.abs(pi[:, 9:13]).max(axis=1) > 0.35)[0][0])   # 첫 꼭짓점의 튐
t0 = pi[k, 0]
wp = (pp[:, 0] > t0 - 0.5) & (pp[:, 0] < t0 + 0.5)
wi = (pi[:, 0] > t0 - 0.5) & (pi[:, 0] < t0 + 0.5)
tp, ti = 1e3 * (pp[wp, 0] - t0), 1e3 * (pi[wi, 0] - t0)
fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
ax[0].plot(tp, pp[wp, 9], color=INK2, lw=2, drawstyle="steps-post")
ax[0].set_ylabel("lookahead 점이 기준보다\n앞선 시간 lead [s]")
ax[1].plot(tp, pp[wp, 8], color=INK2, lw=2, drawstyle="steps-post")
ax[1].set_ylabel("곡률 κ = 2y/L² [1/m]")
ax[2].plot(tp, pp[wp, 5], color=INK2, lw=2, drawstyle="steps-post")
ax[2].set_ylabel("PP 명령 ω [rad/s]")
ax[3].plot(ti, pi[wi, 2], color=C1, lw=2, drawstyle="steps-post", label="FR 기준 ω_ref (×K_p 0.03 → τ)")
ax[3].plot(ti, pi[wi, 6], color=C2, lw=2, drawstyle="steps-post", label="FR 엔코더 ω")
ax[3].set_ylabel("FR 바퀴 [rad/s]"); ax[3].set_xlabel("FR 토크가 튄 틱 기준 시간 [ms]")
ax[3].legend(loc="lower left", frameon=False)
i0 = int(np.argmin(np.abs(ti)))
ax[3].annotate(f"τ_FR {pi[wi][i0, 10]:+.2f} N·m", (0, pi[wi][i0, 2]), (120, 18), color=INK,
               arrowprops=dict(arrowstyle="-", color=INK2, lw=0.7))
for a in ax:
    a.axvline(0, color="#a9a8a2", lw=0.9, ls=":")
ax[0].set_title("퓨어 퍼슛 꼭짓점 직전 (정사각형 a 1.0, 첫 꼭짓점) — lookahead 점이 다음 변 → 지금 변으로 되돌아가는 한 틱",
                color=INK, loc="left", fontsize=10.5)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig_pp_spike.png"), dpi=130)
print("saved", t0)

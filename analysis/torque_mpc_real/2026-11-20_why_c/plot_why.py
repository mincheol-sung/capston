#!/usr/bin/python3
"""190506 (새 μ̂ C): 앞먹임 |U_s|/τ̄ · 실제 |τ|/τ̄ · MPC 가 보는 오차, 꼭짓점(|ω_r| > 0.3) 음영."""
import importlib.util, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", os.path.join(HERE, "why_c.py")); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False, "axes.edgecolor": "#b8b7b0",
                     "axes.labelcolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0",
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
bag = "190506"; S, E, P, A = w.load(f"{WS}/bags/2026-11-20/{bag}"); D = cs.load(f"{WS}/bags/2026-11-20/{bag}"); tb = w.TB["new"]
t = S[:, 0]; tau = S[:, 4:8]; us = S[:, 22:26]; hold = S[:, 20] > 0.5
run = cs.armed_mask(t, A) & ~hold & (np.abs(tau).max(1) > 1e-6); t0 = t[run][0]
rt = np.array([a for a, _ in D["ref"]]); ur = np.array([D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1].at(x)[1] for x in t])
v = np.hypot(ur[:, 0], ur[:, 1]); corner = np.abs(ur[:, 2]) > 0.3
m = run; T = t - t0
fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
for a in ax:
    on = False
    for i in range(len(T)):
        if m[i] and corner[i] and not on: s0 = T[i]; on = True
        if on and (not corner[i] or i == len(T) - 1): a.axvspan(s0, T[i], color="#eda100", alpha=0.18, lw=0); on = False
ax[0].plot(T[m], v[m], color="#52514e", lw=1.8); ax[0].set_ylabel("기준 속도 [m/s]")
ax[0].set_title("190506 (새 μ_hat 의 MPC C) — 노란 띠 = 꼭짓점 헤딩 회전 구간 (|ω_r| > 0.3 rad/s)", loc="left", color="#0b0b0b")
ax[1].plot(T[m], (np.abs(us) / tb).max(1)[m], color="#eb6834", lw=1.6, label="기준을 따라가는 데 필요한 토크 |U_s| / τ̄ (앞먹임)")
ax[1].plot(T[m], (np.abs(tau) / tb).max(1)[m], color="#2a78d6", lw=1.2, label="MPC 가 실제로 낸 |τ| / τ̄")
ax[1].axhline(1.0, color="#0b0b0b", lw=1, ls="--"); ax[1].set_ylabel("가장 센 바퀴의 상한 대비"); ax[1].legend(frameon=False, loc="upper right", fontsize=9)
ax[1].set_ylim(0, 2.2)
ax[2].plot(T[m], 1e3 * np.hypot(S[m, 1], S[m, 2]), color="#6250d6", lw=1.6); ax[2].set_ylabel("MPC 가 보는 위치 오차 [mm]\n(추정기 − 기준)")
ax[2].set_xlabel("주행 시작 뒤 [s]")
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_why_c.png"), dpi=130); print("saved")

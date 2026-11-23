#!/usr/bin/python3
"""η 전용 ω_ε 후보 비교 (2026-11-23) — floor_samples.npz (lam_floor.py, 정확한 λ·인과 10 Hz LPF) 로.
  오작동: 바닥 (|u| < 0.05, 지면 힘 거의 없음) 에서 η < 0.5 인 비율
  놓침  : 확실한 견인 한계 슬립 (|u| ≥ 0.15, u 와 λ̃ 같은 부호, filtfilt |λ̃| > 0.2 — ω_ε 2.5 기준이라 2.5 에 유리) 에서 η ≥ 0.5 인 비율
  η = 1/(1 + exp((|λ̃| − 0.20)/0.0167)) (교수님 곡선).  그리고 속도 구간별 μ–λ 곡선 (line 앞·square) — 꺾임이 비율 (λ) 에 있나 절대 속도에 있나.
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-23_slip_lpf/eps_tradeoff.py   → out_eps.md"""
import os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
Z = np.load(os.path.join(H, "floor_samples.npz"))
Lc, Lz, U, WN, LINE, C8, AE, EPSS = (Z[k] for k in ("Lc", "Lz", "U", "WN", "LINE", "C8", "AE", "EPSS"))
lc, s = 0.20, 0.0167
eta = lambda l: 1 / (1 + np.exp((np.abs(l) - lc) / s))
bands = ((2.5, 5), (5, 10), (10, 17.5), (17.5, 45))
bl = [f"{a}~{b} rad/s ({a * 0.04:.2f}~{b * 0.04:.2f} m/s)" for a, b in bands]
fl = np.abs(U) < 0.05
conf = (np.abs(U) >= 0.15) & (np.sign(U) == np.sign(Lz)) & (np.abs(Lz) > 0.2)
out = ["# η 전용 ω_ε 후보 비교 (정확한 λ, 인과 10 Hz, 교수님 곡선 λ_c 0.20·s 0.0167)\n",
       "바퀴-시간 중 바퀴 기준 속도 < 17.5 rad/s: " + f"{100 * np.mean(WN < 17.5):.1f} %, < 10: {100 * np.mean(WN < 10):.1f} % (모터 정격 38.75 rad/s)\n",
       "## 1. 오작동 (바닥에서 η<0.5 %) / 놓침 (확실한 슬립에서 η≥0.5 %)\n",
       "| ω_ε [rad/s] | " + " | ".join(f"오작동 {b}" for b in bl) + " | 오작동 전체 | " + " | ".join(f"놓침 {b}" for b in bl) + " | 놓침 전체 |",
       "|---" * (2 * len(bands) + 3) + "|"]
cands = [(2.5, Lc)] + [(float(e), AE[:, i, :]) for i, e in enumerate(EPSS)]
for e, L in cands:
    E = eta(L)
    fa = [100 * np.mean(E[fl & (WN >= a) & (WN < b)] < 0.5) for a, b in bands]
    mi = [100 * np.mean(E[conf & (WN >= a) & (WN < b)] >= 0.5) for a, b in bands]
    out.append(f"| {e:g} | " + " | ".join(f"{x:.1f}" for x in fa) + f" | **{100 * np.mean(E[fl & (WN >= 2.5)] < 0.5):.1f}** | " +
               " | ".join(f"{x:.1f}" for x in mi) + f" | **{100 * np.mean(E[conf & (WN >= 2.5)] >= 0.5):.1f}** |")
out.append(f"\n확실한 슬립 표본 수 (구간별): {[int((conf & (WN >= a) & (WN < b)).sum()) for a, b in bands]}")
out += ["\n## 2. 속도 구간별 μ–λ 곡선 (line 앞·square, filtfilt λ̃ ω_ε 2.5) — u 중앙값 (표본)\n",
        "| λ̃ 구간 중앙 | " + " | ".join(bl) + " |", "|---" * (len(bands) + 1) + "|"]
edges = np.r_[-0.6, -0.3, -0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.6]
for a, b in zip(edges[:-1], edges[1:]):
    cells = []
    for lo, hi in bands:
        m = LINE & (WN >= lo) & (WN < hi) & (Lz >= a) & (Lz < b)
        cells.append(f"{np.median(U[m]):+.3f} ({m.sum()})" if m.sum() > 40 else "-")
    out.append(f"| {(a + b) / 2:+.3f} | " + " | ".join(cells) + " |")
open(os.path.join(H, "out_eps.md"), "w").write("\n".join(out) + "\n")
print("\n".join(out))

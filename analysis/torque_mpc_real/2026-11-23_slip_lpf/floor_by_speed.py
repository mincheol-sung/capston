#!/usr/bin/python3
"""floor_samples.npz (lam_floor.py) 로: 바닥 λ̃ 를 속도별·ω_ε 별로, 절대 미끄럼 속도 바닥, μ–λ 곡선을 속도·시나리오별로 (2026-11-23).
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-23_slip_lpf/floor_by_speed.py   → out_floor_speed.md"""
import os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
Z = np.load(os.path.join(H, "floor_samples.npz"))
Lc, Lz, U, WN, A5, A10, DV, LINE, C8 = (Z[k] for k in ("Lc", "Lz", "U", "WN", "A5", "A10", "DV", "LINE", "C8"))
q = (90, 95, 99)
P = lambda a: " / ".join(f"{np.percentile(a, x):.3f}" for x in q)
out = ["# 바닥 λ̃ 의 속도 의존·ω_ε·μ–λ 곡선 (플래그 없는 실차 판)\n",
       "## 1. 바닥 (|u| < 0.05) |λ̃| 90 / 95 / 99 % — 바퀴 기준 속도 구간별, ω_ε 별\n",
       "| ω_nom [rad/s] (바퀴 둘레 속도) | 표본 | ω_ε 2.5 (지금) | ω_ε 5 | ω_ε 10 | 절대 미끄럼 속도 \\|r(ω−ω_nom)\\| [m/s] |",
       "|---|---|---|---|---|---|"]
fl = np.abs(U) < 0.05
for lo, hi in ((2.5, 5), (5, 10), (10, 15), (15, 20), (20, 30), (30, 45)):
    s = fl & (WN >= lo) & (WN < hi)
    out.append(f"| {lo}~{hi} ({lo * 0.04:.2f}~{hi * 0.04:.2f} m/s) | {s.sum()} | {P(np.abs(Lc[s]))} | {P(np.abs(A5[s]))} | {P(np.abs(A10[s]))} | {P(np.abs(DV[s]))} |")
for e, X in (("2.5", Lc), ("5", A5), ("10", A10)):
    s = fl & (WN >= 2.5)
    out.append(f"\nω_ε {e}: 바닥 전체 (ω_nom ≥ 2.5) |λ̃| 90/95/99 % = {P(np.abs(X[s]))}")
out += ["\n## 2. μ–λ 곡선: λ̃ (filtfilt) 구간별 u 중앙값 [표본] — 바퀴 기준 속도 ω_nom ≥ 10 rad/s (0.4 m/s) 만\n",
        "| λ̃ 구간 | 전체 | line 앞·square | circles8 |", "|---|---|---|---|"]
edges = np.r_[-1.0, -0.4, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.6, 1.0]
fast = WN >= 10
for a, b in zip(edges[:-1], edges[1:]):
    cell = []
    for sub in (np.ones_like(fast), LINE, C8):
        s = fast & sub & (Lz >= a) & (Lz < b)
        cell.append(f"{np.median(U[s]):+.3f} [{s.sum()}]" if s.sum() > 30 else "-")
    out.append(f"| {a:+.2f} ~ {b:+.2f} | " + " | ".join(cell) + " |")
open(os.path.join(H, "out_floor_speed.md"), "w").write("\n".join(out) + "\n")
print("\n".join(out))

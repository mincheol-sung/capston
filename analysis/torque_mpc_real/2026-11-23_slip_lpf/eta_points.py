#!/usr/bin/python3
"""η 시그모이드의 시작·0.5·0 지점 후보 비교 (2026-11-23). ω_ε 2.5 고정 (slip_logger 와 같은 λ), 인과 10 Hz λ̃.
η = 1/(1 + exp((|λ̃| − λ_c)/s)) 는 λ_c 에 대해 대칭 → 시작 (η 0.95) = λ_c − ln19·s, 0 지점 (η 0.05) = λ_c + ln19·s.
  평형 슬립 λ*: MPC 가 상한에 붙을 때 η(λ*)·τ̄ = 지면이 받아 주는 토크 → η* = u_avail / (μ_max·c_s), λ* = λ_c + s·ln(1/η* − 1)
  오작동: 바닥 (|u| < 0.05) 에서 η < 0.5
  놓침  : 꺾임 뒤 확실한 슬립 (filtfilt |λ̃| > 0.175, |u| ≥ 0.15, 같은 부호) 에서 η ≥ 0.5
  이른 깎음: 힘이 아직 느는 구간 (0.075 ≤ |λ̃| < 0.125, 같은 부호, |u| ≥ 0.1) 에서 η·μ_max·c_s < |u| (상한이 실제로 걸림)
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-23_slip_lpf/eta_points.py   → out_eta_points.md"""
import os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
Z = np.load(os.path.join(H, "floor_samples.npz"))
Lc, Lz, U, WN = (Z[k] for k in ("Lc", "Lz", "U", "WN"))
CAP = 1.0 * 0.8                    # μ_max 1.0 × c_s 0.8 — τ̄ 를 u 단위 ((r/√2)·N_i) 로
L19 = np.log(19)
C = {"회의록 10~25 %, 20 % 에서 0.5 (지금 기본)": (0.20, 0.0167),
     "전사문 10·15·20 %": (0.15, 0.05 / L19),
     "중간 12.5·17.5·22.5 %": (0.175, 0.05 / L19),
     "좁게 12.5·15·17.5 %": (0.15, 0.025 / L19)}
eta = lambda l, c, s: 1 / (1 + np.exp((np.abs(l) - c) / s))
fl = np.abs(U) < 0.05
conf = (np.abs(U) >= 0.15) & (np.sign(U) == np.sign(Lz)) & (np.abs(Lz) > 0.175)
grow = (np.abs(Lz) >= 0.075) & (np.abs(Lz) < 0.125) & (np.sign(U) == np.sign(Lz)) & (np.abs(U) >= 0.1)
bands = ((2.5, 10), (10, 17.5), (17.5, 45))
out = ["# η 시작·0.5·0 지점 후보 (ω_ε 2.5, 인과 10 Hz λ̃, μ_max 1.0·c_s 0.8)\n",
       "| 후보 | λ_c | s | 시작 (η 0.95) | 0 지점 (η 0.05) | 평형 λ* (u_avail 0.2 / 0.3 / 0.5) | 오작동 0.1~0.4 / 0.4~0.7 / 0.7 m/s 위 / 전체 % | 놓침 % | 이른 깎음 % |",
       "|---|---|---|---|---|---|---|---|---|"]
for name, (c, s) in C.items():
    ls = [c + s * np.log(1 / (ua / CAP) - 1) for ua in (0.2, 0.3, 0.5)]
    E = eta(Lc, c, s)
    fa = [100 * np.mean(E[fl & (WN >= a) & (WN < b)] < 0.5) for a, b in bands] + [100 * np.mean(E[fl & (WN >= 2.5)] < 0.5)]
    mi = 100 * np.mean(E[conf & (WN >= 2.5)] >= 0.5)
    ea = 100 * np.mean(E[grow & (WN >= 2.5)] * CAP < np.abs(U[grow & (WN >= 2.5)]))
    out.append(f"| {name} | {c:.3f} | {s:.4f} | {c - L19 * s:.3f} | {c + L19 * s:.3f} | " + " / ".join(f"{x:.3f}" for x in ls) +
               " | " + " / ".join(f"{x:.1f}" for x in fa) + f" | {mi:.1f} | {ea:.1f} |")
out.append(f"\n표본: 바닥 {int((fl & (WN >= 2.5)).sum())}, 꺾임 뒤 확실한 슬립 {int((conf & (WN >= 2.5)).sum())}, 힘이 느는 구간 {int((grow & (WN >= 2.5)).sum())}")
open(os.path.join(H, "out_eta_points.md"), "w").write("\n".join(out) + "\n")
print("\n".join(out))

#!/usr/bin/python3
"""안쪽·바깥 바퀴 큰 슬립 (참값 λ): 기준 ω > 0 (왼쪽 선회) 이면 안쪽 = FL·RL, ω < 0 이면 FR·RR. 주행 중 (기준 속력 > 0.02, |ω| > 0.05)."""
import os
import numpy as np
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.environ.get("RUNS", "runs"))
print("| 판 | 안쪽 앞 | 안쪽 뒤 | 바깥 앞 | 바깥 뒤 | (|λ| > 0.5 시간 %, 선회 중) |")
for lab in os.environ.get("LABS", "Coff_r1 Coff_r2 Con_r1 Con_r2 PP_r1 PP_r2").split():
    f = f"{R}/{lab}.npz"
    if not os.path.exists(f):
        continue
    z = np.load(f); raw, lam = z["raw"], z["lam"]
    om = np.interp(lam[:, 0], raw[:, 0], raw[:, 9]); sp = np.interp(lam[:, 0], raw[:, 0], np.hypot(raw[:, 7], raw[:, 8]))
    m = (sp > 0.02) & (np.abs(om) > 0.05); L = lam[m, 1:5]; o = om[m]
    left = o > 0
    inf = np.where(left, L[:, 0], L[:, 1]); inr = np.where(left, L[:, 2], L[:, 3]); outf = np.where(left, L[:, 1], L[:, 0]); outr = np.where(left, L[:, 3], L[:, 2])
    pc = lambda x: 100 * np.mean(np.abs(x) > 0.5)
    print(f"| {lab} | {pc(inf):.1f} | {pc(inr):.1f} | {pc(outf):.1f} | {pc(outr):.1f} |")

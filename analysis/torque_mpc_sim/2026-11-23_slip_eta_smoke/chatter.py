#!/usr/bin/python3
"""η_s 떨림 확인: η_s 0.5 가로지름 횟수, 틱간 Δτ RMS (2026-11-23)."""
import importlib.util, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(H, "check.py")).read().split('print("| 판')[0]
ns = {"__file__": os.path.join(H, "check.py")}
exec(src, ns)
for lab in ("eta_off", "eta_on"):
    D = ns["load"](lab); st, sl = D["/torque_mpc/status"], D["/torque_mpc/slip"]
    n = min(len(st), len(sl)); st, sl = st[:n], sl[:n]; act = st[:, 20] == 0
    tau = st[act, 4:8]; e = sl[act, 14:18]; ec = sl[act, 10:14]; t = st[act, 0]; T = t[-1] - t[0]
    cr = lambda E: sum(int(np.sum(np.diff((E[:, i] < 0.5).astype(int)) != 0)) for i in range(4))
    dtau = np.sqrt(np.mean(np.diff(tau, axis=0) ** 2))
    print(f"{lab}: 추종 {T:.1f} s, η_s 0.5 가로지름 곱한 {cr(e)} / 계산한 {cr(ec)} 회 ({cr(ec) / T / 4:.1f} 회/s/바퀴), "
          f"틱간 Δτ RMS {dtau:.4f} N·m, |τ| 최대 {np.abs(tau).max():.3f}, 오차 최대 {1e3 * np.hypot(st[act, 1], st[act, 2]).max():.0f} mm")

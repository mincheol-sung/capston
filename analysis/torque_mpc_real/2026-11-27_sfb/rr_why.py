"""circles8: 원 (왼쪽 wz>0 / 오른쪽 wz<0) 별 · 바퀴별 슬립과 토크.
  u = 바닥 토크 / ((r/√2)·N_i)  (2026-11-23_slip_lpf 5 절과 같은 정의, μ 단위), 바닥 토크 τ_g = τ_meas − J_w ω̇ − τ_fric(ω)
python3 -I rr_why.py <npz 폴더> <판 ...>"""
import os
import sys
import numpy as np
from scipy.signal import butter, filtfilt

HERE = sys.argv[1]
W = ("FL", "FR", "RL", "RR")
r, Jw, m, g = 0.04, 8.9e-4, 4.56, 9.81
N = np.array([0.239, 0.239, 0.261, 0.261]) * m * g
coul = np.array([0.0174, 0.0206, 0.0155, 0.0157]); visc = np.array([0.001066, 0.001158, 0.001112, 0.001177])
cap = r / np.sqrt(2) * 0.33 * N * 0.8          # MPC 바닥 토크 상한 τ̄ (μ̂ 0.33 · c_s 0.8)


def lp(x, fs, fc=10.0):
    return filtfilt(*butter(1, fc / (fs / 2)), x, axis=0)


for name in sys.argv[2:]:
    A = np.load(os.path.join(HERE, name + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    ws, od, S = A["ws"], A["od"], A["slip"]
    t = ws[:, 0]; fs = 1 / np.median(np.diff(t))
    om = ws[:, 5:9]; tq = lp(ws[:, 9:13], fs)
    omd = np.gradient(lp(om, fs), t, axis=0)
    tg = tq - Jw * omd - (coul * np.tanh(om / 0.5) + visc * om)
    wz = np.interp(t, od[:, 0], od[:, 6]); sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    ts = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0); Lf = lp(L, 1 / np.median(np.diff(ts)))
    lam = np.column_stack([np.interp(t, ts, Lf[:, i]) for i in range(4)]); omn = np.column_stack([np.interp(t, ts, S[:, 6 + i]) for i in range(4)])
    mot = A["mot"]; cmd = np.column_stack([np.interp(t, mot[:, 0], mot[:, 2 + i]) for i in range(4)])
    has_st = "st" in A.files and len(A["st"])
    if has_st:
        st = A["st"]; us = np.column_stack([np.interp(t, st[:, 0], st[:, 23 + i]) for i in range(4)])
    base = (t >= t_on) & (t <= t_off) & (sp > 0.3)
    print(f"── {name}")
    for lab, msk in (("왼쪽 원 (wz>0, 안쪽 = FL·RL)", base & (wz > 1.0)), ("오른쪽 원 (wz<0, 안쪽 = FR·RR)", base & (wz < -1.0))):
        sg = np.sign(omn[msk])
        u = tg[msk] * sg / ((r / np.sqrt(2)) * N)
        print(f"   {lab}  {msk.sum() / fs:.1f} s")
        for i in range(4):
            line = (f"      {W[i]}: >50 % {100 * np.mean(np.abs(lam[msk, i]) > 0.5):5.1f} (헛돎 {100 * np.mean(lam[msk, i] > 0.5):5.1f})"
                    f" · 공칭 ω {np.median(omn[msk, i]):5.1f} · 명령 τ {np.mean(cmd[msk, i] * sg[:, i]):+.3f}"
                    f" · 바닥 τ_g {np.mean(tg[msk, i] * sg[:, i]):+.3f} (상한 {cap[i]:.3f}) · u 중앙 {np.median(u[:, i]):+.2f}")
            if has_st:
                line += f" · U_s {np.mean(us[msk, i] * sg[:, i]):+.3f}"
            print(line)

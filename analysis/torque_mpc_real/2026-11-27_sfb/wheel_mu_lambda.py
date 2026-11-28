"""바퀴별 μ–λ 곡선: 2026-11-23_slip_lpf 5 절과 같은 정의를 바퀴마다 따로.
  u = τ_g·sgn(ω_nom) / ((r/√2)·N_i),  τ_g = τ_meas − J_w ω̇ − τ_fric(ω),  λ̃ = 10 Hz filtfilt,  |ω_nom| ≥ 10 rad/s 인 표본만.
  N_i 는 정적 몫 [0.239 0.239 0.261 0.261] — 바퀴의 실제 수직력이 다르면 그 차이가 u 의 꺾임 높이 차이로 보인다.
python3 -I wheel_mu_lambda.py <npz 폴더> <판 ...>"""
import os
import sys
import numpy as np
from scipy.signal import butter, filtfilt

HERE = sys.argv[1]
W = ("FL", "FR", "RL", "RR")
r, Jw, m, g = 0.04, 8.9e-4, 4.56, 9.81
N = np.array([0.239, 0.239, 0.261, 0.261]) * m * g
coul = np.array([0.0174, 0.0206, 0.0155, 0.0157]); visc = np.array([0.001066, 0.001158, 0.001112, 0.001177])
bins = np.array([0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0])


def lp(x, fs, fc=10.0):
    return filtfilt(*butter(1, fc / (fs / 2)), x, axis=0)


LAM, U = [[] for _ in range(4)], [[] for _ in range(4)]
for name in sys.argv[2:]:
    A = np.load(os.path.join(HERE, name + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    ws, S = A["ws"], A["slip"]; t = ws[:, 0]; fs = 1 / np.median(np.diff(t))
    om = ws[:, 5:9]; tq = lp(ws[:, 9:13], fs); omd = np.gradient(lp(om, fs), t, axis=0)
    tg = tq - Jw * omd - (coul * np.tanh(om / 0.5) + visc * om)
    ts = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0); Lf = lp(L, 1 / np.median(np.diff(ts)))
    for i in range(4):
        lam = np.interp(t, ts, Lf[:, i]); omn = np.interp(t, ts, S[:, 6 + i])
        m_ = (t >= t_on) & (t <= t_off) & (np.abs(omn) >= 10.0)
        LAM[i].append(lam[m_]); U[i].append((tg[m_, i] * np.sign(omn[m_])) / ((r / np.sqrt(2)) * N[i]))
print(f"판 {len(sys.argv) - 2} 개, |ω_nom| ≥ 10 rad/s 표본")
for side, sgn in (("구동 (헛돎 쪽, λ̃ > 0)", 1), ("제동 (끌림 쪽, λ̃ < 0)", -1)):
    print(f"\n[{side}] λ̃ 구간별 u 중앙값 (표본 수)")
    print("        " + "  ".join(f"{bins[k]:.2f}~{bins[k + 1]:.2f}" for k in range(len(bins) - 1)))
    for i in range(4):
        lam = sgn * np.concatenate(LAM[i]); u = sgn * np.concatenate(U[i])
        cells = []
        for k in range(len(bins) - 1):
            sel = (lam >= bins[k]) & (lam < bins[k + 1])
            cells.append(f"{np.median(u[sel]):.3f}({sel.sum():4d})" if sel.sum() >= 20 else "   -       ")
        print(f"   {W[i]}  " + "  ".join(cells))

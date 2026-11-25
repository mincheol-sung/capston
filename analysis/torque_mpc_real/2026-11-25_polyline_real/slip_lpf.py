# 11-23 회의 슬립 지표 (analysis/torque_mpc_real/2026-11-23_slip_lpf/all_runs.py 와 같은 식) 로 꺾은선 MPC·PP 두 판:
#   λ = /slip_logger/slip, 10 Hz 1 차 Butterworth filtfilt (위상 0), 창 = 무장 구간 ∩ 추정 속도 > 0.05 m/s (제자리 회전은 빠진다)
#   지표 = max_i |λ̃_i| > 0.25 / > 0.5 시간 %, λ̃ RMS, max|λ̃|, > 0.5 중 헛돎 몫.  원신호와 나란히.  + 국면별·바퀴별 (|λ̃_i| > 0.25 %)
#   /usr/bin/python3 slip_lpf.py   (read_bags.py 의 npz)
import os
import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
for lab in ("143238_sq2_mpc_r1", "144114_sq2_mpc_wheels_r1", "144240_sq2_mpc_wheels_r2", "143351_sq2_pp_r1"):
    A = np.load(os.path.join(HERE, lab + ".npz"))
    arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    fs = 1 / np.median(np.diff(t)); bb, aa = butter(1, 10.0 / (fs / 2)); Lf = filtfilt(bb, aa, L, axis=0)
    od = A["od"]; vx = np.interp(t, od[:, 0], od[:, 4]); vy = np.interp(t, od[:, 0], od[:, 5])
    sp = np.hypot(vx, vy)
    win = (t >= t_on) & (t <= t_off) & (sp > 0.05)
    vs = np.convolve(np.interp(t, od[:, 0], od[:, 4]), np.ones(int(fs * 0.1)) / int(fs * 0.1), mode="same")
    acc = np.gradient(vs, t) * np.sign(vs)                                 # 진행 방향 가속 (+ 빨라짐)

    def met(X, w):
        a = np.abs(X[w]); mx = a.max(1); big = mx > 0.5
        sg = np.sign(X[w][np.arange(w.sum()), a.argmax(1)])
        return 100 * np.mean(mx > 0.25), 100 * big.mean(), np.sqrt(np.mean(X[w] ** 2)), mx.max(), (100 * (sg[big] > 0).mean() if big.any() else np.nan)
    r, f = met(L, win), met(Lf, win)
    print(f"== {lab}: 창 {win.sum() / fs:.1f} s ({fs:.0f} Hz)")
    print(f"   원신호   >25 % {r[0]:5.1f}  >50 % {r[1]:5.1f}  RMS {r[2]:.3f}  최대 {r[3]:.2f}  헛돎 몫 {r[4]:.0f} %")
    print(f"   10 Hz LPF >25 % {f[0]:5.1f}  >50 % {f[1]:5.1f}  RMS {f[2]:.3f}  최대 {f[3]:.2f}  헛돎 몫 {f[4]:.0f} %")
    for nm, sel in (("가속 (> 0.5 m/s²)", win & (acc > 0.5)), ("감속 (< −0.5)", win & (acc < -0.5)), ("순항", win & (np.abs(acc) <= 0.5))):
        print(f"   {nm:16s} {sel.sum() / fs:4.1f} s  LPF |λ̃_i|>0.25 % [FL FR RL RR] {np.round(100 * np.mean(np.abs(Lf[sel]) > 0.25, 0))}"
              f"   원신호 {np.round(100 * np.mean(np.abs(L[sel]) > 0.25, 0))}   LPF λ̃ 평균 {np.round(Lf[sel].mean(0), 3)}")

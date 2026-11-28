"""시뮬 슬립 되먹임 판 점검: 명령 = clamp(τ_MPC + Δτ), Δτ = −K_ω e_ω 재현, 바퀴별 슬립, 추종 오차, Δτ 크기.
python3 -I sim_check.py <npz 폴더> <라벨 ...>"""
import os
import sys
import numpy as np
from scipy.signal import butter, filtfilt

HERE = sys.argv[1]
W = ("FL", "FR", "RL", "RR")
for lab in sys.argv[2:]:
    A = np.load(os.path.join(HERE, lab + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(t)))), L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05)
    X = Lf[w]
    F = A["sfb"]; fm = (F[:, 0] >= t_on) & (F[:, 0] <= t_off)
    on = F[fm, 2].mean() > 0.5
    dt, tm, to, ew = F[fm, 7:11], F[fm, 11:15], F[fm, 15:19], F[fm, 19:23]
    want = np.clip(tm + dt, -1.2, 1.2) if on else tm
    # Δτ = −K_ω e_ω 의 K_ω 를 기록에서 최소제곱으로 되찾기 (대각이면 대각만 남아야)
    K_fit = -np.linalg.lstsq(ew, dt, rcond=None)[0].T
    st = A["st"]; sm = (st[:, 0] >= t_on) & (st[:, 0] <= t_off); epos = np.hypot(st[sm, 2], st[sm, 3])
    print(f"── {lab}: 되먹임 {'켬' if on else '끔'}, 무장 {t_off - t_on:.1f} s, 창 {w.sum()} 행")
    print(f"   낸 τ − clamp(τ_MPC + Δτ) 최대 {np.abs(to - want).max():.2e} N·m")
    print("   기록에서 되찾은 K_ω 대각", np.diag(K_fit).round(4), f" 비대각 최대 |{np.abs(K_fit - np.diag(np.diag(K_fit))).max():.1e}|")
    print("   e_ω RMS [rad/s] " + "  ".join(f"{W[i]} {np.sqrt(np.mean(ew[:, i] ** 2)):.2f}" for i in range(4)))
    print("   Δτ RMS [N·m]    " + "  ".join(f"{W[i]} {np.sqrt(np.mean(dt[:, i] ** 2)):.4f}" for i in range(4)) + f"   최대 {np.abs(dt).max():.3f}")
    print("   > 50 %          " + "  ".join(f"{W[i]} {100 * np.mean(np.abs(X[:, i]) > 0.5):5.1f}" for i in range(4)) +
          f"   네 바퀴 중 최대 {100 * np.mean(np.abs(X).max(1) > 0.5):.1f}")
    print("   > 25 %          " + "  ".join(f"{W[i]} {100 * np.mean(np.abs(X[:, i]) > 0.25):5.1f}" for i in range(4)))
    print(f"   추종 위치 오차 RMS {100 * np.sqrt(np.mean(epos ** 2)):.2f} cm, 최대 {100 * epos.max():.2f} cm;  토크 RMS {np.sqrt(np.mean(to ** 2)):.4f} N·m")

# 2026-11-26 켬·끔 판의 남은 슬립이 어느 국면에서 나나 — 11-23 회의 지표 (10 Hz LPF max_i|λ̃_i| > 0.5) 를 국면별로
#   국면 (기준열 첫 점 기준): 회전 (|ω_r| > 0.3 rad/s) / 가속 (기준 속력 기울기 > 0.3 m/s²) / 감속 (< −0.3) / 순항 (나머지, 기준 속력 > 0.05)
#   + 그때 MPC 제약이 걸려 있던 몫 (status 활성 제약 수 > 0), 바퀴별 몫
import glob, os
import numpy as np
from scipy.signal import butter, filtfilt
HERE = os.path.dirname(os.path.abspath(__file__))
W = "FL FR RL RR".split()
for grp in ("off", "on"):
    agg = {}
    for f in sorted(glob.glob(os.path.join(HERE, f"*_rr_{grp}_r*.npz"))):
        A = np.load(f); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
        t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
        S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
        fs = 1 / np.median(np.diff(t)); bb, aa = butter(1, 10.0 / (fs / 2)); Lf = filtfilt(bb, aa, L, axis=0)
        od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
        win = (t >= t_on) & (t <= t_off) & (sp > 0.05)
        R = A["ref"]; vr = np.hypot(R[:, 3], R[:, 4]); k = max(1, int(0.1 / np.median(np.diff(R[:, 0]))))
        vs = np.convolve(vr, np.ones(k) / k, mode="same"); ar = np.gradient(vs, R[:, 0])
        wz = np.interp(t, R[:, 0], R[:, 6]); a = np.interp(t, R[:, 0], ar)
        st = A["st"]; act = np.interp(t, st[:, 0], st[:, 10]) > 0.5
        ph = np.where(np.abs(wz) > 0.3, "회전", np.where(a > 0.3, "가속", np.where(a < -0.3, "감속", "순항")))
        mx = np.abs(Lf).max(1); big = win & (mx > 0.5)
        for p in ("가속", "감속", "회전", "순항"):
            sel = win & (ph == p)
            d = agg.setdefault(p, [0.0, 0.0, 0.0, np.zeros(4)])
            d[0] += sel.sum() / fs; d[1] += (sel & big).sum() / fs; d[2] += (sel & big & act).sum() / fs
            wi = np.argmax(np.abs(Lf[sel & big]), axis=1) if (sel & big).any() else np.array([], int)
            d[3] += np.bincount(wi, minlength=4) / fs
    tot = sum(v[1] for v in agg.values())
    print(f"== {grp} (3 판 합): 큰 슬립 (> 50 %) 시간 {tot:.2f} s")
    for p, (T, B, Bc, wb) in agg.items():
        print(f"  {p}: 국면 시간 {T:5.2f} s, 그 중 큰 슬립 {100 * B / T:5.1f} %, 큰 슬립 전체의 {100 * B / tot:4.1f} %, 그때 제약 걸림 {100 * Bc / max(B, 1e-9):4.0f} %, "
              f"가장 큰 바퀴 [{' '.join(f'{w} {100 * x / max(B, 1e-9):.0f}' for w, x in zip(W, wb))}] %")

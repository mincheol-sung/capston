"""J 측정 CSV 로 마찰의 바퀴 각도 의존(주기 성분)과 응답/명령 토크 비를 본다.
τ_f = τ_명령 − J·ω̇,  직선 맞춤 c + b·ω 를 뺀 나머지를 바퀴 각도 θ = unwrap(p) 에 대해 푸리에 분해 (1 = 바퀴 한 바퀴에 한 번, 10 = 로터 한 바퀴)."""
import csv, glob, numpy as np
from scipy.ndimage import uniform_filter1d
D = "/home/smoc/capston/capston/analysis/torque_mpc_real/2026-11-16_jw"
J = {1: 9.76e-4, 2: 9.35e-4, 3: 7.3e-4, 4: 9.12e-4}
print("바퀴 | 표본 | 직선 c·b      | 각도 성분 진폭 [mN·m]: 1차  2차  3차  10차  20차 | 각도 성분 RMS / 전체 잔차 RMS | 응답/명령 토크 (정상 부근)")
for wid, nm in zip((1, 2, 3, 4), ("FL", "FR", "RL", "RR")):
    TH, W, TF, RR = [], [], [], []
    files = sorted(glob.glob(f"{D}/jw_{wid}_*.csv")) + (sorted(glob.glob(f"{D}/lo_jw_{wid}_*.csv")) if wid == 3 else [])
    for f in files:
        rows = [r for r in csv.DictReader(open(f)) if r["mode"] == "torque"]
        t = np.array([float(r["t"]) for r in rows]); p = np.array([float(r["p"]) for r in rows]); v = np.array([float(r["v"]) for r in rows])
        tc = np.array([float(r["tau_cmd"]) for r in rows]); tr = np.array([float(r["tau_reply"]) for r in rows])
        dp = np.diff(p); span = 25.0                                     # MIT 위치 범위 ±12.5 rad 에서 감긴다
        dp = np.where(dp < -span / 2, dp + span, np.where(dp > span / 2, dp - span, dp))
        th = np.r_[p[0], p[0] + np.cumsum(dp)]
        vs = uniform_filter1d(v, 9); wd = np.gradient(vs, t)
        chg = np.r_[False, np.abs(np.diff(tc)) > 1e-4]; near = uniform_filter1d(chg.astype(float), 15) > 0
        ok = (np.abs(v) > 3) & (np.abs(v) < 32) & ~near
        TH.append(th[ok]); W.append(np.abs(v[ok])); TF.append(((tc - J[wid] * wd) * np.sign(v))[ok])
        steady = ok & (np.abs(wd) < 3.0) & (np.abs(tc) > 0.02); RR.append((tr[steady] / tc[steady]))
    TH, W, TF, RR = map(np.concatenate, (TH, W, TF, RR))
    A = np.column_stack([np.ones_like(W), W]); cb, *_ = np.linalg.lstsq(A, TF, rcond=None); res = TF - A @ cb
    amps = []
    for n in (1, 2, 3, 10, 20):
        X = np.column_stack([np.cos(n * TH), np.sin(n * TH)]); ab, *_ = np.linalg.lstsq(X, res, rcond=None); amps.append(np.hypot(*ab))
    Xall = np.column_stack([f(n * TH) for n in range(1, 31) for f in (np.cos, np.sin)]); co, *_ = np.linalg.lstsq(Xall, res, rcond=None)
    per = Xall @ co
    print(f" {nm}  | {len(TH):4d} | {cb[0]:.4f} {cb[1]:.5f} | " + "  ".join(f"{1e3*a:4.1f}" for a in amps)
          + f"                 | {1e3*np.sqrt(np.mean(per**2)):.1f} / {1e3*np.sqrt(np.mean(res**2)):.1f}            | {np.median(RR):.3f}")

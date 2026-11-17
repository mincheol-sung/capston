"""※ 2026-11-16 저녁: 이 방법의 결론(FR 마찰 2 배)은 틀렸다 — 바퀴마다 속도 범위가 다른 계단 과도 구간을 직선으로 맞췄다.
   정상 상태 비교는 friction_steady.py (FR 은 +0.003~0.005 N·m). 기록용으로만 남긴다.
J 측정 CSV(바퀴 띄움)로 지금 구동계 마찰 곡선 τ_f(ω) = τ_명령 − J·ω̇ 을 바퀴별로 맞춘다 (J = 바퀴별 측정 중앙, ω > 3 rad/s).
설정(torque_plant.yaml friction, 2026-11-14 내림 계단)과 비교."""
import csv, glob, numpy as np
from scipy.ndimage import uniform_filter1d
D = "/home/smoc/capston/capston/analysis/torque_mpc_real/2026-11-16_jw"
J = {1: 9.76e-4, 2: 9.35e-4, 3: 7.3e-4, 4: 9.12e-4}
CFG_C = [0.02142, 0.02151, 0.02170, 0.01830]; CFG_B = [0.002062, 0.001799, 0.001767, 0.001679]
print("바퀴 | 지금 τc N·m  bv N·m·s/rad (잔차 RMS) | 설정 τc  bv        | 10/20 rad/s 에서 지금 vs 설정 τ_f [N·m] | 보상 0.9×설정 − 지금 @20 rad/s")
for wid, nm in zip((1, 2, 3, 4), ("FL", "FR", "RL", "RR")):
    W, T = [], []
    for f in sorted(glob.glob(f"{D}/jw_{wid}_*.csv")) + (sorted(glob.glob(f"{D}/lo_jw_{wid}_*.csv")) if wid == 3 else []):
        rows = [r for r in csv.DictReader(open(f)) if r["mode"] == "torque"]
        t = np.array([float(r["t"]) for r in rows]); v = np.array([float(r["v"]) for r in rows]); tc = np.array([float(r["tau_cmd"]) for r in rows])
        vs = uniform_filter1d(v, 9); wd = np.gradient(vs, t)
        chg = np.r_[False, np.abs(np.diff(tc)) > 1e-4]
        near = uniform_filter1d(chg.astype(float), 9) > 0             # 계단 ±4 샘플은 뺀다
        ok = (np.abs(v) > 3) & ~near & (np.abs(v) < 32)
        W.append(np.abs(v[ok])); T.append((tc - J[wid] * wd)[ok] * np.sign(v[ok]))
    W, T = np.concatenate(W), np.concatenate(T)
    A = np.column_stack([np.ones_like(W), W]); (c, b), *_ = np.linalg.lstsq(A, T, rcond=None)
    rms = np.sqrt(np.mean((A @ [c, b] - T) ** 2))
    i = wid - 1; now = lambda w: c + b * w; cfg = lambda w: CFG_C[i] + CFG_B[i] * w
    print(f" {nm}  | {c:+.4f}  {b:.5f} ({rms*1e3:.1f} mN·m)          | {CFG_C[i]:.4f} {CFG_B[i]:.5f} | {now(10):.4f}/{now(20):.4f} vs {cfg(10):.4f}/{cfg(20):.4f}        | {0.9*cfg(20)-now(20):+.4f} ({'과보상' if 0.9*cfg(20) > now(20) else '덜 보상'})")

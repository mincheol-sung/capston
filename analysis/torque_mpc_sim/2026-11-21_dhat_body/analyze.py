#!/usr/bin/python3
"""시뮬 8자 (width 1.2, lap_speeds 0.43/0.61/0.80) d̂ 끔 / 예전 / 새 — 참값 기준 랩별 e_c (+ 원 안쪽) 평균·RMS, e_t 평균, 큰 슬립, Δτ RMS, d̂ (status) 곡률 중심 쪽 â_d."""
import importlib.util, json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"
spec = importlib.util.spec_from_file_location("cs", f"{WS}/src/amr_basic_stack/controller/mpc/tools/compare_slam.py")
cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
LAPS = [0.0, 9.0, 15.2, 1e9]; Ts = 0.01
NAME = {"D0_r1": "d̂ 끔", "D1_r1": "예전 d̂ (월드 α0.1, 위치 잔차)", "D2_r1": "새 d̂ (몸체 α0.01, 가속도만, 멈춤·상한)", "D2_r2": "새 d̂ 반복", "D3_r1": "새 d̂ α 0.03", "D4_r1": "새 d̂ 멈춤 끔"}
import sys
LABS = sys.argv[1:] or ["D0_r1", "D1_r1", "D2_r1"]
for lab in LABS:
    z = np.load(f"{HERE}/runs/{lab}.npz"); raw = z["raw"]; j = json.load(open(f"{HERE}/runs/{lab}.json"))
    t = raw[:, 0]; et, ec, en, ey, sp = cs.path_errors(raw[:, 1], raw[:, 2], raw[:, 3], raw[:, 4], raw[:, 5], raw[:, 6], raw[:, 7], raw[:, 8], raw[:, 9])
    mv = sp > 0.02; T0 = t[np.argmax(mv)]; tr = t - T0
    lap = lambda k: mv & (tr >= LAPS[k]) & (tr < LAPS[k + 1])
    lam = z["lam"]; L = lam[np.all(np.isfinite(lam[:, 1:5]), axis=1)]; Lm = (L[:, 0] >= T0) & (np.interp(L[:, 0], t, sp) > 0.3)
    big = 100 * np.mean(np.abs(L[Lm, 1:5]).max(1) > 0.5)
    print(f"== {NAME[lab]}  Δτ RMS {j['dtau_rms']:.4f}  τ max {j['tau_max']:.3f}  시간 추종 RMS {j['rms_mm']:.1f} mm  큰 슬립 (참값 λ, v_ref>0.3) {big:.1f} %")
    print("   e_c 평균 (+ 안쪽) [mm] " + " / ".join(f"{1e3*np.mean(ec[lap(k)]):+5.1f}" for k in range(3)) +
          "   e_c RMS " + " / ".join(f"{1e3*np.sqrt(np.mean(ec[lap(k)]**2)):4.1f}" for k in range(3)) +
          "   e_t 평균 " + " / ".join(f"{1e3*np.mean(et[lap(k)]):+5.1f}" for k in range(3)) +
          f"   yaw RMS {np.degrees(np.sqrt(np.mean(ey[mv]**2))):.2f}°")
    st = z["st"]
    if len(st):
        ts = st[:, 0]; d = st[:, 14:20]; m = ts >= T0
        print(f"   d̂ 속도 성분 /Ts: |â_d,xy| 평균 {np.mean(np.hypot(d[m,3], d[m,4]))/Ts:.3f} 표준편차 {np.std(np.hypot(d[m,3], d[m,4]))/Ts:.3f} m/s²,  α_d 표준편차 {np.std(d[m,5])/Ts:.2f} rad/s²")

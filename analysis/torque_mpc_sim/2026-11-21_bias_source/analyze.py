#!/usr/bin/python3
"""시뮬 8자 (width 1.2, lap_speeds 0.43/0.61/0.80) 안쪽 치우침 원인 가르기 (d̂ 끔): 추정기 / 바퀴 속도 / 참값 / 슬립 없음 / 상한 B — 참값 기준 랩별 e_c (+ 원 안쪽) 평균·RMS, e_t 평균, 큰 슬립, Δτ RMS, d̂ (status) 곡률 중심 쪽 â_d."""
import importlib.util, json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"
spec = importlib.util.spec_from_file_location("cs", f"{WS}/src/amr_basic_stack/controller/mpc/tools/compare_slam.py")
cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
LAPS = [0.0, 9.0, 15.2, 1e9]; Ts = 0.01
NAME = {"B0_r1": "B0 기준 (추정기, μ 0.45, C)", "B0_r2": "B0 반복", "B1_r1": "B1 속도 = 바퀴 기구학", "B2_r1": "B2 참값 상태",
        "B3_r1": "B3 참값 + 슬립 없음", "B4_r1": "B4 참값 + 슬립 없음 + 상한 B", "B5_r1": "B5 B4 + 마찰 = 보상량", "B5_r2": "B5 반복"}
import sys
LABS = sys.argv[1:] or list(NAME)
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
    om = raw[:, 9]
    for k in range(3):
        Lm_, Rm_ = lap(k) & (om > 0.3), lap(k) & (om < -0.3)
        if Lm_.any() and Rm_.any():
            l, r = 1e3 * np.mean(ec[Lm_]), 1e3 * np.mean(ec[Rm_])
            print(f"   랩 {k+1}: e_c 왼원 {l:+5.1f} 오른원 {r:+5.1f} mm → s (공통 안쪽) {(l+r)/2:+5.1f}  d (몸체 왼쪽) {(l-r)/2:+5.1f}   e_t 평균 {1e3*np.mean(et[lap(k)]):+5.1f} mm")
    st = z["st"]
    if len(st):
        ts = st[:, 0]; d = st[:, 14:20]; m = ts >= T0
        print(f"   d̂ 속도 성분 /Ts: |â_d,xy| 평균 {np.mean(np.hypot(d[m,3], d[m,4]))/Ts:.3f} 표준편차 {np.std(np.hypot(d[m,3], d[m,4]))/Ts:.3f} m/s²,  α_d 표준편차 {np.std(d[m,5])/Ts:.2f} rad/s²")
    ad = z["ad"] if "ad" in z.files else np.zeros((0, 7))
    if len(ad):
        m = ad[:, 0] >= T0
        print(f"   ρ̂ 끝 {ad[-1, 1]:.3f}  (주행 중 최소 {ad[m, 1].min():.3f} · 최대 {ad[m, 1].max():.3f}),  η 측정 주기 {100*np.mean(ad[m, 3]):.0f} %,  d̂ 건너뜀 {100*np.mean(ad[m, 6] > 0):.0f} %")

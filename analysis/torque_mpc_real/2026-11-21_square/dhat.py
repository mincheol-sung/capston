#!/usr/bin/python3
"""d̂ (외란 추정치, torque_mpc.cpp update_disturbance 식-9·11) 를 켜면 왜 나빠지나 — 오늘 8자 width 1.2 (lap_speeds 0.43/0.61/0.80, accel 1.0).
  d = x̂_k − (x̂_{k−1} 한 스텝 예측),  d̂_f = 0.9 d̂_f + 0.1 d  (α 0.1, Ts 0.01 → 시정수 0.1 s).  status[14:20] = d̂ (x, y, θ, ẋ, ẏ, θ̇ 잔차, 한 스텝 단위).
  쓰임: (1) 예측 50 스텝마다 상태에 + d̂ (2) feedforward U_s 에서 â_d = d̂_속도 / Ts 를 뺀다 (식-14).
  출력: 랩별 e_c (옆, + 원 안쪽) 평균·RMS, â_d 의 경로 법선 (n̂, + 원 왼쪽 = 기준 속도 왼쪽) · 접선 성분 평균 (느린 몫) 과 표준편차 (빠른 몫),
        d̂ 위치 성분 × 50 (예측 끝에서 위치에 쌓이는 양), Δτ RMS, 포화.
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/dhat.py"""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; OFF = w.off; Ts = 0.01
LAPS = [0.0, 9.0, 15.2, 1e9]
TB0 = np.array([0.1093, 0.1113, 0.1154, 0.1055]); TB2 = TB0 * 0.972 / 0.432

RUNS = [("C 예전 Q, d̂ 끔", "201047", TB0), ("C 새 Q, d̂ 끔", "203133", TB0), ("C 새 Q, d̂ 켬", "205044", TB0),
        ("C 새 Q, μ̂×2.25, d̂ 켬", "205817", TB2), ("PP", "205937", None),
        ("C 새 Q, 새 d̂ (몸체 α0.01·멈춤·상한)", "212105", TB0),
        ("C 새 Q, μ̂×2.25, 새 d̂, crossing 0.1", "211454", TB2), ("PP crossing 0.1", "211616", None)]
for lab, b, tb in RUNS:
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path)
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF)
    rt = np.array([a for a, _ in D["ref"]]); rows = []
    for t, x, y, th in zip(P[:, 0], xc, yc, P[:, 3]):
        i = np.searchsorted(rt, t, side="right") - 1
        if i >= 0:
            Xr, ur = D["ref"][i][1].at(t)[:2]; rows.append([t, x, y, th, *Xr, *ur])
    a = np.array(rows); t = a[:, 0]
    et, ec, en, ey, sp = cs.path_errors(a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6], a[:, 7], a[:, 8], a[:, 9])
    mv = cs.armed_mask(t, D["armed"]) & (sp > 0.02); T0 = t[np.flatnonzero(mv)[0]]; tr = t - T0
    lap = lambda k, tt: (tt >= LAPS[k]) & (tt < LAPS[k + 1])
    c2 = cs.analyse(path, OFF, 0.04, 0.1932)
    print(f"== {lab} ({b})   Δτ RMS {c2['dtau']:.4f}  τ max {c2['tau_max']:.2f}  yaw RMS {c2['yaw']:.1f}°")
    print("   e_c 평균 (+ 안쪽) 랩 1/2/3 [mm]: " + " / ".join(f"{1e3*np.mean(ec[mv & lap(k, tr)]):+5.0f}" for k in range(3)) +
          "   e_c RMS: " + " / ".join(f"{1e3*np.sqrt(np.mean(ec[mv & lap(k, tr)]**2)):4.0f}" for k in range(3)) +
          "   e_t 평균: " + " / ".join(f"{1e3*np.mean(et[mv & lap(k, tr)]):+4.0f}" for k in range(3)))
    if tb is None:
        continue
    S, E, _, A = w.load(path)
    ts = S[:, 0]; tau = S[:, 4:8]; d = S[:, 14:20]; hold = S[:, 20] > 0.5
    run = cs.armed_mask(ts, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
    tv, om = [], []
    for x in ts:
        R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]; Xr, ur = R_.at(x)[:2]
        c, s = math.cos(Xr[2]), math.sin(Xr[2]); vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]])
        tv.append(vw / max(np.linalg.norm(vw), 1e-9)); om.append(ur[2])
    tv = np.array(tv); nv = np.c_[-tv[:, 1], tv[:, 0]]; om = np.array(om)
    if b in ("212105", "211454"):                             # 새 d̂: status 가 몸체 좌표 → 추정기 헤딩으로 월드로 돌린다
        th = np.interp(ts, E[:, 0], np.unwrap(E[:, 3])); cth, sth = np.cos(th), np.sin(th)
        d = d.copy(); vx, vy = d[:, 3].copy(), d[:, 4].copy(); d[:, 3] = cth * vx - sth * vy; d[:, 4] = sth * vx + cth * vy
    ad = d[:, 3:5] / Ts                                       # â_d (월드) [m/s²]
    ad_n = (ad * nv).sum(1) * np.sign(om); ad_t = (ad * tv).sum(1)  # 법선 성분을 곡률 중심 쪽 + 로
    ath = d[:, 5] / Ts
    sat = (np.abs(tau) / tb).max(1) >= 0.98
    tsr = ts - T0; m = run & (tsr >= 0)
    if np.abs(d[m]).max() < 1e-12:
        print("   d̂ = 0 (꺼짐)  포화 %.0f %%" % (100 * sat[m].mean())); continue
    print("   â_d 곡률 중심 쪽 (+) 랩 평균 [m/s²]: " + " / ".join(f"{np.mean(ad_n[m & lap(k, tsr)]):+.2f}" for k in range(3)) +
          "   표준편차: " + " / ".join(f"{np.std(ad_n[m & lap(k, tsr)]):.2f}" for k in range(3)))
    print("   â_d 접선 (+ 앞) 랩 평균: " + " / ".join(f"{np.mean(ad_t[m & lap(k, tsr)]):+.2f}" for k in range(3)) +
          "   표준편차: " + " / ".join(f"{np.std(ad_t[m & lap(k, tsr)]):.2f}" for k in range(3)) +
          f"   요 α_d 표준편차 {np.std(ath[m]):.1f} rad/s²")
    dp = 50 * np.hypot(d[m, 0], d[m, 1])
    print(f"   d̂ 위치 × 50 스텝 (예측 끝에 쌓이는 위치) 중앙 {1e3*np.median(dp):.0f} mm, 95 % {1e3*np.percentile(dp, 95):.0f} mm   |â_d| 95 % {np.percentile(np.hypot(*ad[m].T), 95):.2f} m/s²   포화 {100*sat[m].mean():.0f} %")
    # â_d 의 빠른 몫 스펙트럼 최대 (SLAM 10 Hz 인가)
    x = ad_n[m] - np.mean(ad_n[m]); f = np.fft.rfftfreq(len(x), Ts); Pw = np.abs(np.fft.rfft(x)) ** 2
    top = f[1:][np.argsort(Pw[1:])[-3:]][::-1]
    print("   â_d 법선 성분 파워 큰 주파수 [Hz]: " + ", ".join(f"{v:.1f}" for v in top))

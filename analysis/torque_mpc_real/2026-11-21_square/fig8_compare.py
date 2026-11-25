#!/usr/bin/python3
"""오늘 8자 (circles8 width 1.2, tangent, lap_speeds [0.43, 0.61, 0.80], accel 1.0) — PP 200944 vs C 201047 (Q = diag(5, 5, 2.5, 2, 2, 1)).
잣대: compare_slam.path_errors (SLAM 중심 − 같은 시각 기준, 기준 접선 성분 e_t (− 뒤처짐) · 곡률 중심 쪽 e_c · |e| · 요 오차), 랩별 (기준 출발 뒤 0 / 9.0 / 15.2 s 로 나눔 — figure8 로그).
λ (slip_logger, v_ref > 0.3), C 는 옆 feedback 회귀·SLAM 튐 (compare_all.extra 와 같은 식)."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)        # analyze 머리 + extra()
cs, WS, read_topic, extra = g["cs"], g["WS"], g["g"]["read_topic"], g["extra"]
TB = g["TB"]; OFF = g["w"].off
LAPS = [0.0, 9.0, 15.2, 1e9]

for lab, bag, tb in [("PP", "2026-11-21/200944", None), ("C (Q=diag(5,5,2.5,2,2,1))", "2026-11-21/201047", np.array([0.1093, 0.1113, 0.1154, 0.1055]))]:
    path = f"{WS}/bags/{bag}"; D = cs.load(path)
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF)
    rt = np.array([a for a, _ in D["ref"]]); rows = []
    for t, x, y, th in zip(P[:, 0], xc, yc, P[:, 3]):
        i = np.searchsorted(rt, t, side="right") - 1
        if i >= 0:
            Xr, ur = D["ref"][i][1].at(t)[:2]; rows.append([t, x, y, th, *Xr, *ur])
    a = np.array(rows); t = a[:, 0]
    et, ec, en, ey, sp = cs.path_errors(a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6], a[:, 7], a[:, 8], a[:, 9])
    arm = cs.armed_mask(t, D["armed"]); mv = arm & (sp > 0.02); T0 = t[np.flatnonzero(mv)[0]]; tr = t - T0
    print(f"== {lab} {bag[-6:]}  (판 전체 · 랩 1 0.43 / 랩 2 0.61 / 랩 3 0.80 m/s)")
    for name, f in [("|e| RMS (시간 추종) [mm]", lambda m: 1e3 * np.sqrt(np.mean(en[m] ** 2))),
                    ("e_c RMS (옆, 경로 모양) [mm]", lambda m: 1e3 * np.sqrt(np.mean(ec[m] ** 2))),
                    ("e_c 평균 (+ 원 안쪽) [mm]", lambda m: 1e3 * np.mean(ec[m])),
                    ("e_t 평균 (− 뒤처짐) [mm]", lambda m: 1e3 * np.mean(et[m])),
                    ("yaw RMS [°]", lambda m: np.degrees(np.sqrt(np.mean(ey[m] ** 2))))]:
        vals = [f(mv)] + [f(mv & (tr >= LAPS[k]) & (tr < LAPS[k + 1])) for k in range(3)]
        print(f"   {name:30s} {vals[0]:7.1f} | " + " / ".join(f"{v:6.1f}" for v in vals[1:]))
    L = read_topic(path, "/slip_logger/slip"); L = L[np.all(np.isfinite(L[:, 1:5]), axis=1)]
    vr = np.interp(L[:, 0], t, sp); gm = (L[:, 0] >= T0) & (vr > 0.3); mx = np.abs(L[gm, 1:5]).max(1)
    sg = np.sign(L[gm, 1:5][np.arange(gm.sum()), np.abs(L[gm, 1:5]).argmax(1)])
    print(f"   큰 슬립 (max|λ|>0.5, v_ref>0.3) {100*np.mean(mx>0.5):.1f} %, 헛돎 몫 {100*np.mean(sg[mx>0.5]>0):.0f} %, λ RMS {np.sqrt(np.mean(L[gm,1:5]**2)):.3f}")
    x = extra(bag, tb, None)
    print(f"   SLAM 튐 {x['jumps']} / 최대 {x['jump_max']:.0f} mm, 추정기−SLAM {x['est_slam']:.0f} mm" +
          (f", 옆 feedback k_p {x['kp']:.2f} k_d {x['kd']:.2f} (R² {x['r2']:.2f}), 포화 {x['sat']:.0f} %" if "kp" in x else ""))
    c2 = cs.analyse(path, OFF, 0.04, 0.1932)
    print(f"   Δτ RMS {c2['dtau']:.4f}, τ max {c2['tau_max']:.2f}, 슬립 크기 x·y {c2['slip_x']:.3f}·{c2['slip_y']:.3f}")

#!/usr/bin/python3
"""왼원 (ω_r > 0, 반시계) · 오른원 (ω_r < 0, 시계) 나눠서 — e_c (+ 원 안쪽) 평균, 몸체 좌표 옆 오차 (+ 몸체 왼쪽) 평균, 공통 몫 s = (L+R)/2 (안쪽) · 몸체 몫 d = (L−R)/2 (+ 몸체 왼쪽 쪽으로 치우침),
바퀴 옆 힘 / m·a_n 비, C 는 d̂ 몸체 y 원값 (부호 안 곱함) — 몸체 좌표에서 일정한 옆 힘이면 두 원에서 같은 부호로 나온다."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; OFF = w.off; Ts = 0.01
r, k, m = 0.04, 0.1932, 4.56
FM = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
RUNS = [("C 새 d̂ w2.0", "212502", True), ("PP w2.0", "212629", False), ("PP w2.0 (20:43)", "204329", False), ("C μ̂×2.25 d̂끔 w2.0", "204536", True),
        ("C 새 d̂ w1.2", "212105", True), ("C d̂끔 w1.2", "203133", True), ("PP w1.2", "205937", False)]
print("| 판 | e_c 평균 (+ 안쪽) 왼원 / 오른원 [mm] | 공통 s (안쪽) | 몸체 d (+ 몸체 왼쪽) | 옆 힘 / m·a_n 왼 / 오른 | d̂ 몸체 y 왼 / 오른 [m/s²] |")
for lab, b, isC in RUNS:
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path)
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF); tp = P[:, 0]
    rt = np.array([a for a, _ in D["ref"]]); rows = []
    for t, x, y, th in zip(tp, xc, yc, P[:, 3]):
        i = np.searchsorted(rt, t, side="right") - 1
        if i >= 0:
            Xr, ur = D["ref"][i][1].at(t)[:2]; rows.append([t, x, y, th, *Xr, *ur])
    a = np.array(rows); t = a[:, 0]
    et, ec, en, ey, sp = cs.path_errors(a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6], a[:, 7], a[:, 8], a[:, 9])
    om = a[:, 9]; mv = cs.armed_mask(t, D["armed"]) & (sp > 0.3) & (np.abs(om) > 0.3)
    L, R = mv & (om > 0), mv & (om < 0)
    eL, eR = 1e3 * np.mean(ec[L]), 1e3 * np.mean(ec[R])
    s, dd = 0.5 * (eL + eR), 0.5 * (eL - eR)          # 왼원 안쪽 = 몸체 왼쪽 (tangent), 오른원 안쪽 = 몸체 오른쪽
    # 옆 힘 비
    T = np.array(D["tau"]); T = T[T[:, 1] != 1]; h = 0.1
    tg = np.arange(T[0, 0] + 0.3, T[-1, 0] - 0.3, 0.02); X = lambda arr, tt: np.interp(tt, tp, arr)
    vx = (X(xc, tg + h) - X(xc, tg - h)) / (2 * h); vy = (X(yc, tg + h) - X(yc, tg - h)) / (2 * h)
    ax = (X(xc, tg + h) - 2 * X(xc, tg) + X(xc, tg - h)) / h ** 2; ay = (X(yc, tg + h) - 2 * X(yc, tg) + X(yc, tg - h)) / h ** 2
    v = np.hypot(vx, vy); a_n = (vx * ay - vy * ax) / np.maximum(v, 1e-6)
    tau = np.c_[[np.interp(tg, T[:, 0], T[:, 2 + i]) for i in range(4)]].T; Fy = (FM @ tau.T).T[:, 1]
    omg = np.interp(tg, t, om); spg = np.interp(tg, t, sp); mg = cs.armed_mask(tg, D["armed"]) & (spg > 0.3) & (np.abs(omg) > 0.3)
    rat = lambda mm: np.mean(Fy[mm] * np.sign(omg[mm])) / (m * np.mean(a_n[mm] * np.sign(omg[mm])))
    rL, rR = rat(mg & (omg > 0)), rat(mg & (omg < 0))
    dh = ""
    if isC:
        S, E, _, A = w.load(path); ts = S[:, 0]; d = S[:, 14:20]
        th = np.interp(ts, E[:, 0], np.unwrap(E[:, 3])); oms = np.interp(ts, t, om); sps = np.interp(ts, t, sp)
        ms = cs.armed_mask(ts, A) & (sps > 0.3) & (np.abs(oms) > 0.3)
        frame = str(D.get("dist_frame", ""))
        # 새 d̂ (212502·212105) 는 status 가 몸체 좌표, 나머지는 월드 → 몸체로 돌린다
        if b in ("212502", "212105", "211454"):
            dby = d[:, 4]
        else:
            dby = -np.sin(th) * d[:, 3] + np.cos(th) * d[:, 4]
        dh = f"{np.mean(dby[ms & (oms>0)])/Ts:+.3f} / {np.mean(dby[ms & (oms<0)])/Ts:+.3f}"
    print(f"| {lab} {b} | {eL:+.0f} / {eR:+.0f} | {s:+.0f} | {dd:+.0f} | {rL:.2f} / {rR:.2f} | {dh} |")

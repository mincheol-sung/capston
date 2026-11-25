#!/usr/bin/python3
"""뒤처짐을 경로 진행량 (arc length) 으로: lag(t) = s_robot(t) − s_ref(t).
s_ref: 기준점이 그 시각까지 경로를 따라 간 거리. s_robot: 로봇 위치를 기준 경로 polyline 에 투영한 거리 (직전 값 근처 창에서 찾아 꼭짓점에서 건너뛰지 않게).
로봇 위치 둘: 추정기 (MPC 가 보는 값, 기준 − Xe) 와 SLAM (/tracked_pose → 로봇 중심)."""
import importlib.util, math, os
import numpy as np
WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"; WHY = f"{WS}/analysis/torque_mpc_real/2026-11-20_why_c"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", f"{WHY}/why_c.py"); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)


def project(px, py, X, Y, Sc, s0, lo=-0.3, hi=0.6):
    m = (Sc >= s0 + lo) & (Sc <= s0 + hi); j = np.flatnonzero(m)
    if len(j) < 2:
        return s0
    best, bs = 1e9, s0
    for k in j[:-1]:
        ax, ay, bx, by = X[k], Y[k], X[k + 1], Y[k + 1]; dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
        u = 0.0 if L2 < 1e-12 else min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        d = (ax + u * dx - px) ** 2 + (ay + u * dy - py) ** 2
        if d < best:
            best, bs = d, Sc[k] + u * math.sqrt(L2)
    return bs


for bag, which in [("171531", "old"), ("171735", "old"), ("190506", "new"), ("190642", "new")]:
    S, E, P, A = w.load(f"{WS}/bags/2026-11-20/{bag}"); D = cs.load(f"{WS}/bags/2026-11-20/{bag}")
    t = S[:, 0]; tau = S[:, 4:8]; hold = S[:, 20] > 0.5
    run = cs.armed_mask(t, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
    rt = np.array([a for a, _ in D["ref"]]); pr, ur = [], []
    for x in t:
        i = np.searchsorted(rt, x, side="right") - 1
        p, u = D["ref"][max(i, 0)][1].at(x)[:2]; pr.append(p); ur.append(u)
    pr = np.array(pr); ur = np.array(ur); idx = np.flatnonzero(run); a0, a1 = idx[0], idx[-1]
    X, Y = pr[a0:a1 + 1, 0], pr[a0:a1 + 1, 1]; Sc = np.r_[0.0, np.cumsum(np.hypot(np.diff(X), np.diff(Y)))]
    v = np.hypot(ur[:, 0], ur[:, 1]); vd = np.gradient(v, t); om = ur[:, 2]
    corner = np.abs(om) > 0.3; acc = ~corner & (vd > 0.2); dec = ~corner & (vd < -0.2); cru = ~corner & ~acc & ~dec
    # 추정기 위치 = 기준 − Xe
    ex, ey = pr[:, 0] - S[:, 1], pr[:, 1] - S[:, 2]
    # SLAM 중심, status 시각으로 보간
    xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], w.off)
    sx, sy = np.interp(t, P[:, 0], xc), np.interp(t, P[:, 0], yc)
    res = {}
    for lab, qx, qy in [("추정기", ex, ey), ("SLAM", sx, sy)]:
        lag = np.full(len(t), np.nan); s_prev = 0.0
        for n, k in enumerate(range(a0, a1 + 1)):
            s_prev = project(qx[k], qy[k], X, Y, Sc, s_prev); lag[k] = s_prev - Sc[n]
        res[lab] = lag
    print(f"== {bag} ({which} μ̂)  side 합 {Sc[-1]:.2f} m")
    for lab, lag in res.items():
        L = lag[run]; dl = np.r_[0.0, np.diff(lag)]; ok = run & np.r_[False, run[:-1]]
        parts = "  ".join(f"{nm} {1e3*dl[ok & m].sum():+5.0f}" for nm, m in [("가속", acc), ("감속", dec), ("꼭짓점", corner), ("그밖", cru)])
        print(f"   [{lab:3s}] lag 평균 {1e3*np.nanmean(L):+5.0f} 최저 {1e3*np.nanmin(L):+5.0f} mm | 구간별 lag 변화 합 [mm]: {parts}")
    # 변마다: 꼭짓점 앞 (회전 구간 진입) 의 lag 와 회전 구간 끝 lag (SLAM)
    cr = run & corner; e = np.flatnonzero(np.diff(cr.astype(int)))
    st = res["추정기"]; ss = res["SLAM"]
    line = []
    for k in range(0, len(e) - 1, 2):
        a, b = e[k] + 1, e[k + 1]
        if 0.3 < (b - a) * 0.01 < 1.0:
            line.append(f"{1e3*st[a]:+4.0f}→{1e3*st[b]:+4.0f} ({1e3*ss[a]:+4.0f}→{1e3*ss[b]:+4.0f})")
    print("   꼭짓점 진입→이탈 lag [mm], 추정기 (SLAM):  " + " | ".join(line))
    # 변별: 변 시작 (출발 또는 꼭짓점 이탈) 의 진행 속도 결손 Δṡ = ṡ_robot − ṡ_ref, 그 변 직선에서 lag 변화 (추정기)
    lag = res["추정기"]; sr = np.full(len(t), np.nan); sr[a0:a1 + 1] = Sc; srob = lag + sr
    def rate(q, k, h=10):
        return (q[k + h] - q[k - h]) / (t[k + h] - t[k - h])
    segs = [k for k in range(0, len(e) - 1, 2) if 0.3 < (e[k + 1] - e[k]) * 0.01 < 1.0]
    starts = [a0 + 12] + [e[k + 1] + 1 for k in segs]; ends = [e[k] + 1 for k in segs] + [e[-2] + 1 if len(e) % 2 == 0 else a1]
    for n, (b0, b1) in enumerate(zip(starts, ends)):
        dv = rate(srob, b0) - rate(sr, b0)
        print(f"   변 {n}: 시작 {'출발' if n == 0 else '꼭짓점 뒤'}  ṡ_ref {rate(sr, b0):.2f} ṡ_rob {rate(srob, b0):.2f} (Δ {dv:+.2f} m/s)  lag {1e3*lag[b0]:+4.0f} → 변 끝 {1e3*lag[b1]:+4.0f} mm  (직선에서 {1e3*(lag[b1]-lag[b0]):+4.0f})")

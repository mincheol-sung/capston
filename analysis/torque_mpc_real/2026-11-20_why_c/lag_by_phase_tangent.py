#!/usr/bin/python3
"""C 의 진행 방향 오차 e_t (MPC 가 보는 값: 추정기 − 기준, 기준 접선 성분, − = 뒤처짐) 가 어느 구간에서 쌓이고 풀리나.
구간 분류는 why_c/phase.py 와 같다: 꼭짓점 |ω_r| > 0.3, 가속 v̇_r > 0.2, 감속 v̇_r < −0.2, 그 밖."""
import importlib.util, math, os
import numpy as np
WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"; WHY = f"{WS}/analysis/torque_mpc_real/2026-11-20_why_c"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", f"{WHY}/why_c.py"); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)

for bag, which in [("171531", "old"), ("171735", "old"), ("190506", "new"), ("190642", "new")]:
    S, E, P, A = w.load(f"{WS}/bags/2026-11-20/{bag}"); D = cs.load(f"{WS}/bags/2026-11-20/{bag}")
    t = S[:, 0]; tau = S[:, 4:8]; hold = S[:, 20] > 0.5
    run = cs.armed_mask(t, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
    rt = np.array([a for a, _ in D["ref"]]); pr, ur = [], []
    for x in t:
        i = np.searchsorted(rt, x, side="right") - 1
        if i >= 0:
            p, u = D["ref"][i][1].at(x)[:2]; pr.append(p); ur.append(u)
        else:
            pr.append(np.full(3, np.nan)); ur.append(np.full(3, np.nan))
    pr = np.array(pr); ur = np.array(ur)
    c, s = np.cos(pr[:, 2]), np.sin(pr[:, 2])
    vw = np.stack([c * ur[:, 0] - s * ur[:, 1], s * ur[:, 0] + c * ur[:, 1]], 1)
    v = np.linalg.norm(vw, axis=1); tt = vw / np.maximum(v, 1e-9)[:, None]
    et = -(S[:, 1:3] * tt).sum(1)          # Xe = 기준 − 추정기 → 추정기 − 기준 의 접선 성분
    vd = np.gradient(v, t); om = ur[:, 2]
    corner = np.abs(om) > 0.3; acc = ~corner & (vd > 0.2); dec = ~corner & (vd < -0.2); cru = ~corner & ~acc & ~dec
    det = np.r_[0.0, np.diff(et)]; ok = run & np.r_[False, run[:-1]] & np.isfinite(det) & (v > 0.05)
    idx = np.where(run)[0]
    print(f"== {bag} ({which} μ̂)   e_t: 평균 {1e3*np.nanmean(et[run]):+.0f} mm, 최저 {1e3*np.nanmin(et[run]):+.0f} mm")
    for name, m in [("가속", acc), ("감속", dec), ("꼭짓점", corner), ("그 밖", cru)]:
        mm = ok & m
        print(f"   {name:5s} 시간 {mm.sum()*0.01:4.2f} s   e_t 변화 합 {1e3*det[mm].sum():+6.0f} mm   (쌓임 {1e3*det[mm][det[mm]<0].sum():+5.0f} / 풀림 {1e3*det[mm][det[mm]>0].sum():+5.0f})")
    # 꼭짓점마다 들어갈 때·나올 때 e_t
    cr = run & corner; edges = np.flatnonzero(np.diff(cr.astype(int)))
    segs = [(edges[k] + 1, edges[k + 1]) for k in range(0, len(edges) - 1, 2)] if cr[0] == 0 else []
    for a, b in segs:
        if (b - a) * 0.01 < 0.3:
            continue
        print(f"     꼭짓점 t={t[a]-t[idx[0]]:4.2f}~{t[b]-t[idx[0]]:4.2f} s: 들어갈 때 {1e3*et[a]:+5.0f} → 나올 때 {1e3*et[b]:+5.0f} mm,  v_r {v[a]:.2f}→{v[b]:.2f}")

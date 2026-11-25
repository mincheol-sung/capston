#!/usr/bin/python3
"""원 안쪽 치우침을 MPC 도 보나 — 212502 (C 새 d̂, width 2.0): status Xe (기준 − 추정기) 의 곡률 중심 쪽 성분 (MPC 가 보는 e_n) vs SLAM e_c, 랩별 평균.
추정기 − SLAM 위치 차의 곡률 중심 쪽 성분도 같이."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; OFF = w.off; LAPS = [0.0, 14.8, 25.2, 1e9]
for b in ("212502", "204536"):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path); S, E, P, A = w.load(path)
    ts = S[:, 0]; Xe = S[:, 1:3]; rt = np.array([a for a, _ in D["ref"]])
    nv, om, vr, pr = [], [], [], []
    for x in ts:
        R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]; Xr, ur = R_.at(x)[:2]
        c, s = math.cos(Xr[2]), math.sin(Xr[2]); vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]]); n = np.linalg.norm(vw)
        tv = vw / max(n, 1e-9); nv.append([-tv[1], tv[0]]); om.append(ur[2]); vr.append(n); pr.append(Xr[:2])
    nv = np.array(nv); om = np.array(om); vr = np.array(vr); pr = np.array(pr); sg = np.sign(om)
    e_mpc = (-(Xe) * nv).sum(1) * sg                       # 추정기 − 기준, 곡률 중심 쪽 + (MPC 가 보는 값)
    xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF)
    sx, sy = np.interp(ts, P[:, 0], xc), np.interp(ts, P[:, 0], yc)
    e_slam = ((np.c_[sx, sy] - pr) * nv).sum(1) * sg       # SLAM − 기준
    run = cs.armed_mask(ts, A) & (vr > 0.02); T0 = ts[np.flatnonzero(run)[0]]; tr = ts - T0
    lap = lambda k: run & (tr >= LAPS[k]) & (tr < LAPS[k + 1])
    print(f"== {b}  곡률 중심 쪽 + 랩 1/2/3 평균 [mm]")
    print("   MPC 가 보는 (추정기 − 기준) " + " / ".join(f"{1e3*np.mean(e_mpc[lap(k)]):+5.1f}" for k in range(3)))
    print("   SLAM − 기준              " + " / ".join(f"{1e3*np.mean(e_slam[lap(k)]):+5.1f}" for k in range(3)))
    print("   추정기 − SLAM            " + " / ".join(f"{1e3*np.mean(e_mpc[lap(k)]-e_slam[lap(k)]):+5.1f}" for k in range(3)))

#!/usr/bin/python3
"""속력 스케일 — 원 위 순항에서 SLAM 위치 차분 속력 · 추정기 속력 (odom twist 가 아니라 추정기 위치 차분) · 바퀴 기구학 속력 (T ω) · 기준 속력의 비.
SLAM 차분은 0.2 s 중앙 차분 (10 Hz 잡음 줄임). 바퀴 기구학은 미끄럼 없음 가정 (헛돌면 크게 나온다)."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; OFF = w.off; LAPS = [0.0, 14.8, 25.2, 1e9]; r_w = 0.04
for b in ("212502", "212629", "204536"):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path); S, E, P, A = w.load(path)
    xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF); tp = P[:, 0]
    rt = np.array([a for a, _ in D["ref"]])
    tg = np.arange(max(tp[0], E[0, 0]) + 0.3, min(tp[-1], E[-1, 0]) - 0.3, 0.05)
    h = 0.1
    v_slam = np.hypot(np.interp(tg + h, tp, xc) - np.interp(tg - h, tp, xc), np.interp(tg + h, tp, yc) - np.interp(tg - h, tp, yc)) / (2 * h)
    v_est = np.hypot(np.interp(tg + h, E[:, 0], E[:, 1]) - np.interp(tg - h, E[:, 0], E[:, 1]), np.interp(tg + h, E[:, 0], E[:, 2]) - np.interp(tg - h, E[:, 0], E[:, 2])) / (2 * h)
    W = np.array(D["wheel"]); q = r_w / 4
    vwx = q * W[:, 1:5].sum(1); vwy = q * (-W[:, 1] + W[:, 2] + W[:, 3] - W[:, 4])
    v_whl = np.hypot(np.interp(tg, W[:, 0], vwx), np.interp(tg, W[:, 0], vwy))
    vr, om = [], []
    for x in tg:
        i = np.searchsorted(rt, x, side="right") - 1
        u = D["ref"][i][1].at(x)[1] if i >= 0 else np.zeros(3); vr.append(math.hypot(u[0], u[1])); om.append(u[2])
    vr = np.array(vr); om = np.array(om)
    arm = cs.armed_mask(tg, D["armed"]); dv = np.gradient(vr, tg); m = arm & (vr > 0.3) & (np.abs(dv) < 0.1) & (np.abs(om) > 0.3)
    print(f"== {b}  원 위 순항 표본 {m.sum()}  속력 비 (평균의 비)")
    print(f"   SLAM/기준 {v_slam[m].mean()/vr[m].mean():.3f}   추정기/기준 {v_est[m].mean()/vr[m].mean():.3f}   추정기/SLAM {v_est[m].mean()/v_slam[m].mean():.3f}   바퀴 T ω/SLAM {v_whl[m].mean()/v_slam[m].mean():.3f}")

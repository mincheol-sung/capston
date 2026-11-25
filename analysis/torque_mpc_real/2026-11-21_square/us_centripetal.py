#!/usr/bin/python3
"""U_s 의 구심 성분이 모델이 그 원을 도는 데 필요한 양과 맞나 — status U_s (22:26, 바퀴 토크) → 몸체 힘 F = (T⁻¹)ᵀ U_s → 월드 (추정기 헤딩) → 곡률 중심 쪽 성분 F_n,
필요한 구심력 m_eff v_r² / R (v_r = 기준 속력, R = 0.5 m) 과 비교. 접선 성분 (등속이면 0 이어야), 요 모멘트도. 원 위 순항 (기준 가속 작음) 만."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; LAPS = [0.0, 14.8, 25.2, 1e9]
r, k, m_eff, I_eff, R = 0.04, 0.1932, 6.785, 0.1431, 0.5
FM = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
for b in ("212502", "204536"):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path); S, E, P, A = w.load(path)
    ts = S[:, 0]; us = S[:, 22:26]; tau = S[:, 4:8]; rt = np.array([a for a, _ in D["ref"]])
    tv, om, vr, th_r = [], [], [], []
    for x in ts:
        R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]; Xr, ur = R_.at(x)[:2]
        c, s = math.cos(Xr[2]), math.sin(Xr[2]); vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]]); n = np.linalg.norm(vw)
        tv.append(vw / max(n, 1e-9)); om.append(ur[2]); vr.append(n); th_r.append(Xr[2])
    tv = np.array(tv); nv = np.c_[-tv[:, 1], tv[:, 0]]; om = np.array(om); vr = np.array(vr); th_r = np.array(th_r)
    def world_force(T, th):
        Fb = (FM @ T.T).T; c, s = np.cos(th), np.sin(th)
        return np.c_[c * Fb[:, 0] - s * Fb[:, 1], s * Fb[:, 0] + c * Fb[:, 1]], Fb[:, 2]
    th_e = np.interp(ts, E[:, 0], np.unwrap(E[:, 3]))
    Fu, Mu = world_force(us, th_r); Ft, Mt = world_force(tau, th_e)
    sg = np.sign(om); Fn_us = (Fu * nv).sum(1) * sg; Ft_us = (Fu * tv).sum(1); Fn_tau = (Ft * nv).sum(1) * sg
    dv = np.gradient(vr, ts); run = cs.armed_mask(ts, A) & (vr > 0.2) & (np.abs(dv) < 0.1) & (np.abs(om) > 0.3)
    T0 = ts[np.flatnonzero(cs.armed_mask(ts, A) & (vr > 0.02))[0]]; tr = ts - T0
    print(f"== {b}  원 위 순항 (|v̇_r| < 0.1, |ω_r| > 0.3)")
    for q in range(3):
        m = run & (tr >= LAPS[q]) & (tr < LAPS[q + 1])
        need = m_eff * np.mean(vr[m] ** 2) / R
        print(f"   랩 {q+1} v_r {np.mean(vr[m]):.2f}: 필요 구심 m v²/R {need:.2f} N | U_s 구심 {np.mean(Fn_us[m]):.2f} N ({np.mean(Fn_us[m])/need:.2f} 배) · 접선 {np.mean(Ft_us[m]):+.2f} N · 요 M {np.mean(Mu[m]*sg[m]):+.3f} N·m"
              f" | 실제 τ 구심 {np.mean(Fn_tau[m]):.2f} N ({np.mean(Fn_tau[m])/need:.2f} 배)")

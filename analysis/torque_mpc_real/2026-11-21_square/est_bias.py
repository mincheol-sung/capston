#!/usr/bin/python3
"""원 안쪽 치우침 — (1) C 포화율 (랩별) (2) d̂ 몸체 옆 성분 (곡률 중심 쪽 +) (3) 추정기 가속도계 바이어스 b_ay 가 원 방향 (왼원 ω>0 / 오른원 ω<0) 에 따라 뒤집히나
(4) SLAM 보정 잔차 (/state_estimator/status 의 잔차 x·y) 의 곡률 중심 쪽 성분 — 추정기 전파가 반지름 방향으로 계속 틀리면 일정하게 남는다.
/state_estimator/status = [t, 잔차 x, y, ψ, b_ax, b_ay, b_g, 튐 수, 보정 수, 포즈 나이] (state_estimator.py)."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; read_topic = g["g"]["read_topic"]; Ts = 0.01
TB0 = np.array([0.1093, 0.1113, 0.1154, 0.1055]); LAPS = [0.0, 14.8, 25.2, 1e9]


def refinfo(D, rt, ts):
    nv, om, vr = [], [], []
    for x in ts:
        R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]; Xr, ur = R_.at(x)[:2]
        c, s = math.cos(Xr[2]), math.sin(Xr[2]); vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]]); n = np.linalg.norm(vw)
        tv = vw / max(n, 1e-9); nv.append([-tv[1], tv[0]]); om.append(ur[2]); vr.append(n)
    return np.array(nv), np.array(om), np.array(vr)


for b, kind in (("212502", "C"), ("212629", "PP")):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path); S, E, P, A = w.load(path); rt = np.array([a for a, _ in D["ref"]])
    ES = read_topic(path, "/state_estimator/status")
    print(f"== {kind} {b}")
    if kind == "C":
        ts = S[:, 0]; tau = S[:, 4:8]; d = S[:, 14:20]; nvS, omS, vrS = refinfo(D, rt, ts)
        run = cs.armed_mask(ts, A) & (vrS > 0.02); T0 = ts[np.flatnonzero(run)[0]]; tr = ts - T0
        sat = (np.abs(tau) / TB0).max(1) >= 0.98
        dn = d[:, 4] / Ts * np.sign(omS)                    # 몸체 y (tangent 라 경로 왼쪽) × 곡률 부호 → 곡률 중심 쪽 +
        print("   포화 랩 1/2/3: " + " / ".join(f"{100*sat[run & (tr>=LAPS[k]) & (tr<LAPS[k+1])].mean():.0f} %" for k in range(3)) +
              "   d̂ 곡률 중심 쪽 [m/s²]: " + " / ".join(f"{np.mean(dn[run & (tr>=LAPS[k]) & (tr<LAPS[k+1])]):+.3f}" for k in range(3)) +
              "   d̂ 접선: " + " / ".join(f"{np.mean(d[:,3][run & (tr>=LAPS[k]) & (tr<LAPS[k+1])])/Ts:+.3f}" for k in range(3)))
    te = ES[:, 0]; nvE, omE, vrE = refinfo(D, rt, te)
    armE = cs.armed_mask(te, D["armed"]) & (vrE > 0.3) & (np.abs(omE) > 0.3)
    b_ay = ES[:, 5]; b_ax = ES[:, 4]; res = ES[:, 1:3]
    rn = (res * nvE).sum(1) * np.sign(omE)                  # 보정 잔차 (측정 − 예측? 부호는 아래 표시) 곡률 중심 쪽
    pre = te < (te[armE][0] if armE.any() else te[-1])
    print(f"   b_a 기동 값 ({np.median(b_ax[pre]):+.3f}, {np.median(b_ay[pre]):+.3f}) m/s²  →  왼원 (ω>0) b_ay {np.mean(b_ay[armE & (omE>0)]):+.3f} · 오른원 (ω<0) {np.mean(b_ay[armE & (omE<0)]):+.3f}"
          f"   b_ax 왼 {np.mean(b_ax[armE & (omE>0)]):+.3f} · 오른 {np.mean(b_ax[armE & (omE<0)]):+.3f}")
    print(f"   SLAM 보정 잔차 곡률 중심 쪽 평균 {1e3*np.mean(rn[armE]):+.2f} mm (왼원 {1e3*np.mean(rn[armE & (omE>0)]):+.2f} · 오른원 {1e3*np.mean(rn[armE & (omE<0)]):+.2f}),"
          f" 표준편차 {1e3*np.std(rn[armE]):.2f} mm, 보정 {armE.sum()} 번")

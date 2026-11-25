#!/usr/bin/python3
"""d̂ 를 더 느리게 거르면 — 기록된 d̂ (α 0.1, 시정수 0.1 s) 에 1 차 저역통과를 하나 더 (시정수 0.5 / 1 / 2 s, = α 0.02 / 0.01 / 0.005 근사) 걸어
  â_d (= d̂_속도 / Ts) 의 곡률 중심 쪽 성분 랩 평균 (쓸모 있는 느린 몫) 과 표준편차 (흔들림), 요 α_d 표준편차, 바퀴당 feedforward 흔들림 토크를 본다.
  203133 (d̂ 끔 — d̂ 가 루프에 안 들어간 깨끗한 추정) 과 205044 (d̂ 켬).
  필요한 보정 크기 비교: 203133 의 정상상태 옆 어긋남 e_c × 옆 이득 k_p (정사각형 C2·C3 회귀 5.5~7.8 /s²)."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; Ts = 0.01; LAPS = [0.0, 9.0, 15.2, 1e9]
r, k, m_eff, I_eff = 0.04, 0.1932, 6.785, 0.1431


def lpf(x, tau):
    if tau <= 0:
        return x.copy()
    a = Ts / tau; y = np.empty_like(x); acc = x[0]
    for i, v in enumerate(x):
        acc = (1 - a) * acc + a * v; y[i] = acc
    return y


import sys
FRAME = sys.argv[1] if len(sys.argv) > 1 else "world"
print(f"### 거르는 좌표: {FRAME}")
for b in ("203133", "205044"):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path); S, E, _, A = w.load(path)
    ts = S[:, 0]; tau = S[:, 4:8]; d = S[:, 14:20]; hold = S[:, 20] > 0.5
    run = cs.armed_mask(ts, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
    rt = np.array([a for a, _ in D["ref"]]); tv, om, vr = [], [], []
    for x in ts:
        R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]; Xr, ur = R_.at(x)[:2]
        c, s = math.cos(Xr[2]), math.sin(Xr[2]); vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]])
        tv.append(vw / max(np.linalg.norm(vw), 1e-9)); om.append(ur[2]); vr.append(np.linalg.norm(vw))
    tv = np.array(tv); nv = np.c_[-tv[:, 1], tv[:, 0]]; om = np.array(om); vr = np.array(vr)
    T0 = ts[np.flatnonzero(run & (vr > 0.02))[0]]; tsr = ts - T0; m = run & (tsr >= 0)
    lap = lambda kk: m & (tsr >= LAPS[kk]) & (tsr < LAPS[kk + 1])
    print(f"== {b}  (곡률 중심 쪽 â_d 랩 평균 / 표준편차 [m/s²], 요 α_d 표준편차, 바퀴당 흔들림 토크 = 옆 m σ r/4 · 요 I σ_α r/(4k))")
    for tau_f in (0.0, 0.5, 1.0, 2.0):
        if FRAME == "world":      # 지금 구현: 월드 좌표에서 거른 뒤 경로 법선에 투영
            dv = np.c_[lpf(d[:, 3], tau_f), lpf(d[:, 4], tau_f)] / Ts
            an = (dv * nv).sum(1) * np.sign(om)
        else:                     # 경로 (≈ 몸체, tangent 헤딩) 좌표로 먼저 돌린 뒤 거른다
            an = lpf((d[:, 3:5] * nv).sum(1) * np.sign(om) / Ts, tau_f)
        al = lpf(d[:, 5], tau_f) / Ts
        mean = [np.mean(an[lap(kk)]) for kk in range(3)]; sd = [np.std(an[lap(kk)]) for kk in range(3)]
        sda = np.std(al[m]); tq_n = m_eff * max(sd) * r / 4; tq_a = I_eff * sda * r / (4 * k)
        lab = "기록 그대로 (0.1 s)" if tau_f == 0 else f"+ 시정수 {tau_f:.1f} s"
        print(f"   {lab:18s} 평균 " + " / ".join(f"{v:+.2f}" for v in mean) + "   표준편차 " + " / ".join(f"{v:.2f}" for v in sd) +
              f"   요 α_d σ {sda:.2f} rad/s²   흔들림 토크 옆 {tq_n:.4f} · 요 {tq_a:.4f} N·m")
print("필요한 보정 (203133 의 e_c 평균 +29 / +35 / +44 mm × k_p 5.5~7.8): " +
      " / ".join(f"{0.001*e*5.5:.2f}~{0.001*e*7.8:.2f}" for e in (29, 35, 44)) + " m/s²")

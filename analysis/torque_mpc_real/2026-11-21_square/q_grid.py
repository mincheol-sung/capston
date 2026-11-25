#!/usr/bin/python3
"""다음 가중치 후보 — pole.py 의 무제약 증강 closed loop (torque_mpc.cpp H·g 그대로, 2026-11-18 주석값 4 개로 검증) 로 Q 성분 조합을 훑는다.
  R_Δ = 100 I₄ (w_dudt 0.01), R_U = 10 I₄ 그대로. 열: x·y 가장 느린 pole (시정수), x·y 우세 pole 감쇠 ζ, θ 가장 느린 pole, 가장 빠른 pole,
  첫 스텝 feedback 옆 가속: 옆 위치 오차 0.10 m / 옆 속도 오차 0.30 m/s (SLAM 튐 때 추정기 거짓 속도 크기), 요 오차 5° → 첫 스텝 α."""
import numpy as np
import pole as P

def row(Wn):
    P.W = np.array(Wn, float)
    Pl = P.poles(100, 10); Kx, _ = P.gains(100, 10)
    xy = [p for a, p in Pl if a == "x·y"]; th = [p for a, p in Pl if a == "θ"]
    sl = max(xy, key=lambda p: p.real)
    dom = [p for p in xy if abs(p.real - sl.real) < 1e-6 or abs(p - sl) < 0.5]
    zeta = min((-p.real / abs(p) for p in xy if abs(p) < 20), default=1.0)
    fa = max((p for _, p in Pl), key=abs)
    ay_p = (P.F @ (-Kx @ np.array([0, 0.10, 0, 0, 0, 0])))[1] / P.m_eff
    ay_v = (P.F @ (-Kx @ np.array([0, 0, 0, 0, 0.30, 0])))[1] / P.m_eff
    al = (P.F @ (-Kx @ np.array([0, 0, np.radians(5), 0, 0, 0])))[2] / P.I_eff
    return sl, zeta, max(th, key=lambda p: p.real), fa, ay_p, ay_v, al

print("| Q = diag(Q11..Q66) | x·y 가장 느린 pole (시정수) | x·y ζ 최소 | θ 가장 느린 | 가장 빠른 | 옆 0.10 m → a | 옆 0.30 m/s → a | 요 5° → α |")
for Wn in ([5, 5, 2.5, 2, 2, 1], [20, 20, 2.5, 2, 2, 1], [35, 35, 2.5, 2, 2, 1], [50, 50, 2.5, 2, 2, 1],
           [20, 20, 2.5, 1, 1, 1], [35, 35, 2.5, 1, 1, 1], [20, 20, 2.5, 0.5, 0.5, 1],
           [20, 20, 10, 2, 2, 1], [20, 20, 10, 1, 1, 0.5], [35, 35, 10, 1, 1, 1]):
    sl, z, th, fa, ap, av, al = row(Wn)
    print(f"| {Wn} | {sl.real:+.2f} ({-1/sl.real:.2f} s) | {z:.2f} | {th.real:+.2f} | {abs(fa)/(2*np.pi):.1f} Hz | {ap:+.2f} | {av:+.2f} | {al:+.1f} rad/s² |")

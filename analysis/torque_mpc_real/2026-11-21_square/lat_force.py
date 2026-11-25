#!/usr/bin/python3
"""실제로 원을 도는 데 쓴 옆 힘 vs 몸체 질량 구심력 — C 212502 · PP 212629 (width 2.0, R 0.5).
최종 바퀴 토크 명령 (/wheel_torque_cmd, 마찰 보상 포함 — 원 위 등속에서 마찰 보상의 옆 합은 0) → 몸체 힘 F = (T⁻¹)ᵀ τ, 몸체 y × 곡률 부호 = 곡률 중심 쪽.
필요한 구심력은 실제 궤적으로: m · v_SLAM² / ρ, ρ = 순간 곡률 반지름 (SLAM 중심 궤적의 0.2 s 차분 속도·가속도) — 기준 R 이 아니라 실제로 돈 원.
IMU (/imu/base_link) 옆 가속도로도 같은 값 (구심 가속 = a_y, 몸체 y 가 반지름 방향)."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; read_topic = g["g"]["read_topic"]; OFF = w.off
r, k, m = 0.04, 0.1932, 4.56
FM = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
LAPS = [0.0, 14.8, 25.2, 1e9]
for b, kind in (("212502", "C"), ("212629", "PP")):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path)
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF); tp = P[:, 0]
    T = np.array(D["tau"]); T = T[T[:, 1] != 1]                   # [t, mode, τ×4]
    rt = np.array([a for a, _ in D["ref"]])
    tg = np.arange(T[0, 0] + 0.3, T[-1, 0] - 0.3, 0.02); h = 0.1
    X = lambda arr, tt: np.interp(tt, tp, arr)
    vx = (X(xc, tg + h) - X(xc, tg - h)) / (2 * h); vy = (X(yc, tg + h) - X(yc, tg - h)) / (2 * h)
    ax = (X(xc, tg + h) - 2 * X(xc, tg) + X(xc, tg - h)) / h ** 2; ay = (X(yc, tg + h) - 2 * X(yc, tg) + X(yc, tg - h)) / h ** 2
    v = np.hypot(vx, vy); a_n = (vx * ay - vy * ax) / np.maximum(v, 1e-6)        # 부호 있는 법선 가속 (왼쪽 +)
    tau = np.c_[[np.interp(tg, T[:, 0], T[:, 2 + i]) for i in range(4)]].T
    Fb = (FM @ tau.T).T                                            # 몸체 [F_x, F_y, M_z]
    I = read_topic(path, "/imu/base_link") if False else None
    om_r, vr = [], []
    for x in tg:
        i = np.searchsorted(rt, x, side="right") - 1
        u = D["ref"][i][1].at(x)[1] if i >= 0 else np.zeros(3); om_r.append(u[2]); vr.append(math.hypot(u[0], u[1]))
    om_r = np.array(om_r); vr = np.array(vr); sg = np.sign(om_r)
    arm = cs.armed_mask(tg, D["armed"]); dvr = np.gradient(vr, tg); m_ = arm & (vr > 0.3) & (np.abs(dvr) < 0.1) & (np.abs(om_r) > 0.3)
    T0 = tg[np.flatnonzero(arm & (vr > 0.02))[0]]; tr = tg - T0
    print(f"== {kind} {b}  (원 위 순항, 몸체 옆 힘 F_y × 곡률 부호 vs m · a_n(SLAM 궤적))")
    for q in range(3):
        mm = m_ & (tr >= LAPS[q]) & (tr < LAPS[q + 1])
        Fn = np.mean(Fb[mm, 1] * sg[mm]); need = m * np.mean(a_n[mm] * sg[mm])
        print(f"   랩 {q+1}: 실제 구심 가속 {np.mean(a_n[mm]*sg[mm]):.2f} m/s² (v {np.mean(v[mm]):.2f}, ρ {np.mean(v[mm]**2/np.maximum(np.abs(a_n[mm]),1e-3)):.3f} m)"
              f"  →  m·a_n {need:.2f} N  vs  토크가 낸 옆 힘 {Fn:.2f} N  (비 {Fn/need:.2f}),  접선 힘 {np.mean(Fb[mm,0]):+.2f} N, 요 M {np.mean(Fb[mm,2]*sg[mm]):+.3f} N·m")

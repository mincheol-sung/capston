#!/usr/bin/python3
"""8자 width 2.0 (지름 1.0 m 원 두 개, lap_speeds 0.43/0.61/0.80, accel 1.0, tangent) — C 새 d̂ 212502 · PP 212629 (2026-11-21 21:25, Claude 원격),
같이 PP 204329 · C (μ̂×2.25, d̂ 끔) 204536 (같은 날 20:43, 다른 사용자). 랩 시작 0 / 14.8 / 25.2 s (figure8 로그).
잣대 (SLAM 중심): e_c (+ 원 안쪽) 랩 평균·RMS, |e| 경로 거리 (기준 경로까지, 교차점 0.15 m 안 / 밖) — 2026-11-21_pp_vs_mpc 7 절과 같은 정의 근사:
  경로 거리 = 같은 판 기준 궤적 (100 Hz 조밀 표본) polyline 까지 최소 거리, 교차점 = 주행 시작 순간의 기준 위치.
  yaw 흔들림 = 추정기 요 각속도 (요 차분 0.02 s) 에서 0.5 s 이동평균을 뺀 나머지의 RMS (주행 중).
  큰 슬립 (slip_logger λ, v_ref>0.3), Δτ RMS, τ max (compare_slam.analyse)."""
import os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]; read_topic = g["g"]["read_topic"]; OFF = w.off
LAPS = [0.0, 14.8, 25.2, 1e9]

for lab, b in [("C 새 d̂ (기본 μ̂)", "212502"), ("PP", "212629"), ("PP (20:43)", "204329"), ("C μ̂×2.25 d̂ 끔 (20:43)", "204536")]:
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path)
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF)
    rt = np.array([a for a, _ in D["ref"]]); rows = []
    for t, x, y, th in zip(P[:, 0], xc, yc, P[:, 3]):
        i = np.searchsorted(rt, t, side="right") - 1
        if i >= 0:
            Xr, ur = D["ref"][i][1].at(t)[:2]; rows.append([t, x, y, th, *Xr, *ur])
    a = np.array(rows); t = a[:, 0]
    et, ec, en, ey, sp = cs.path_errors(a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6], a[:, 7], a[:, 8], a[:, 9])
    mv = cs.armed_mask(t, D["armed"]) & (sp > 0.02); i0 = np.flatnonzero(mv)[0]; T0 = t[i0]; tr = t - T0; i1 = np.flatnonzero(mv)[-1]
    # 기준 경로 polyline (조밀) 과 경로 거리
    td = np.arange(T0, t[i1], 0.01)
    RD = np.array([D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1].at(x)[0][:2] for x in td])
    seg_a, seg_b = RD[:-1], RD[1:]; u = seg_b - seg_a; L2 = np.maximum((u ** 2).sum(1), 1e-12)
    def pdist(p):
        tt = np.clip(((p - seg_a) * u).sum(1) / L2, 0, 1); q = seg_a + tt[:, None] * u
        return np.min(np.hypot(*(q - p).T))
    pd = np.array([pdist(np.array([x, y])) if m else np.nan for x, y, m in zip(a[:, 1], a[:, 2], mv)])
    cx, cy = a[i0, 4], a[i0, 5]; near = np.hypot(a[:, 1] - cx, a[:, 2] - cy) < 0.15
    rms = lambda v, m: 1e3 * float(np.sqrt(np.nanmean(v[m] ** 2)))
    lap = lambda k: mv & (tr >= LAPS[k]) & (tr < LAPS[k + 1])
    # yaw 흔들림 (추정기 요)
    S, E, _, A = w.load(path); te = E[:, 0]; yaw = np.unwrap(E[:, 3])
    tg = np.arange(T0, t[i1], 0.01); yg = np.interp(tg, te, yaw); om = np.gradient(yg, 0.01)
    k = 50; ma = np.convolve(om, np.ones(k) / k, mode="same"); jit = float(np.sqrt(np.mean((om - ma)[k:-k] ** 2)))
    L = read_topic(path, "/slip_logger/slip"); L = L[np.all(np.isfinite(L[:, 1:5]), axis=1)]
    gm = (L[:, 0] >= T0) & (L[:, 0] <= t[i1]) & (np.interp(L[:, 0], t, sp) > 0.3); mx = np.abs(L[gm, 1:5]).max(1)
    c2 = cs.analyse(path, OFF, 0.04, 0.1932)
    print(f"== {lab} ({b})  Δτ RMS {c2['dtau']:.4f}  τ max {c2['tau_max']:.2f}  큰 슬립 {100*np.mean(mx>0.5):.1f} %  yaw 흔들림 {jit:.2f} rad/s  yaw RMS {np.degrees(np.sqrt(np.mean(ey[mv]**2))):.1f}°")
    print("   e_c 평균 (+ 안쪽) 랩 1/2/3 [mm] " + " / ".join(f"{1e3*np.mean(ec[lap(q)]):+4.0f}" for q in range(3)) +
          "   e_c RMS " + " / ".join(f"{1e3*np.sqrt(np.mean(ec[lap(q)]**2)):3.0f}" for q in range(3)) +
          "   e_t 평균 " + " / ".join(f"{1e3*np.mean(et[lap(q)]):+4.0f}" for q in range(3)))
    print(f"   경로 거리 RMS 전체 {rms(pd, mv):.0f} mm | 교차점 0.15 m 안 {rms(pd, mv & near):.0f} (최대 {1e3*np.nanmax(pd[mv & near]):.0f}) | 밖 {rms(pd, mv & ~near):.0f}"
          + "   랩별 " + " / ".join(f"{rms(pd, lap(q)):.0f}" for q in range(3)))

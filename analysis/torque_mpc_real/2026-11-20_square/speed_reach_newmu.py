#!/usr/bin/python3
"""변별 최고 속도와 그 위치 — 기준 vs 실제 (SLAM 중심 속도, ±0.1 s 차분). 위치 = 변 시작 꼭짓점에서 변을 따라 잰 거리 [m]."""
import importlib.util, json, math, os
import numpy as np, yaml
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]; off = (est["imu_offset_x"], est["imu_offset_y"])
SIDE = 1.5
out = {}
for bag, name in [("190506", "MPC C 새 μ̂ (1)"), ("190642", "MPC C 새 μ̂ (2)")]:
    D = cs.load(f"{WS}/bags/2026-11-20/{bag}")
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], off); ts = P[:, 0]
    rt = np.array([a for a, _ in D["ref"]])
    R = []
    for t in ts:
        i = np.searchsorted(rt, t, side="right") - 1
        R.append(D["ref"][i][1].at(t)[0].tolist() + D["ref"][i][1].at(t)[1].tolist() if i >= 0 else [np.nan] * 6)
    R = np.array(R)
    arm = cs.armed_mask(ts, D["armed"]); vref = np.hypot(R[:, 3], R[:, 4]); mv = arm & (vref > 0.02)
    i0 = np.flatnonzero(mv)[0]; o = R[i0, :3]; c, s = math.cos(o[2]), math.sin(o[2])
    lx = c * (xc - o[0]) + s * (yc - o[1]); ly = -s * (xc - o[0]) + c * (yc - o[1])
    rx = c * (R[:, 0] - o[0]) + s * (R[:, 1] - o[1]); ry = -s * (R[:, 0] - o[0]) + c * (R[:, 1] - o[1])
    h = 0.1
    vx = (np.interp(ts + h, ts, xc) - np.interp(ts - h, ts, xc)) / (2 * h); vy = (np.interp(ts + h, ts, yc) - np.interp(ts - h, ts, yc)) / (2 * h)
    v = np.hypot(vx, vy)
    # 변 번호: 0 아래(y≈0, x↑), 1 오른쪽(x≈1.5, y↑), 2 위(y≈1.5, x↓), 3 왼쪽(x≈0, y↓). 거리 = 변 시작 꼭짓점부터
    def edge(px, py):
        d = np.stack([np.abs(py), np.abs(px - SIDE), np.abs(py - SIDE), np.abs(px)], 1)
        e = np.argmin(d, 1); along = np.choose(e, [px, py, SIDE - px, SIDE - py]); return e, along
    e_r, a_r = edge(rx, ry); e_m, a_m = edge(lx, ly)
    T = ts - ts[i0]; t_end = T[np.flatnonzero(mv)[-1]]
    rows = []
    for k in range(4):
        # 기준이 그 변에 있는 시간 구간 (주행 중)
        wr = mv & (e_r == k) & (a_r > 0.05) & (a_r < SIDE - 0.05)
        if not wr.any(): continue
        tr0, tr1 = T[wr].min(), T[wr].max()
        j = np.flatnonzero(wr)[np.argmax(vref[wr])]
        # 로봇이 그 변 위에 있던 구간 (기준 구간 −0.3 ~ +1.5 s 안)
        wm = arm & (T > tr0 - 0.3) & (T < tr1 + 1.5) & (e_m == k) & (a_m > 0.05) & (a_m < SIDE - 0.05)
        jm = np.flatnonzero(wm)[np.argmax(v[wm])]
        rows.append(dict(edge=k, ref_v=float(vref[j]), ref_pos=float(a_r[j]), ref_t=float(T[j]),
                         v=float(v[jm]), pos=float(a_m[jm]), t=float(T[jm]), lag_at_ref_peak=float(np.interp(T[j], T, a_m) - a_r[j]) if e_m[j] == k else float("nan")))
    out[bag] = dict(name=name, edges=rows)
    print(f"\n== {name} ({bag})   변: 기준 최고 v @ 위치 (시각)  |  실제 최고 v @ 위치 (시각)")
    for r in rows:
        print(f"  변 {r['edge']}: 기준 {r['ref_v']:.2f} m/s @ {r['ref_pos']:.2f} m ({r['ref_t']:.2f} s)  |  실제 {r['v']:.2f} m/s @ {r['pos']:.2f} m ({r['t']:.2f} s)"
              f"  → 속도 {100*r['v']/r['ref_v']:.0f} %, 위치 {r['pos']-r['ref_pos']:+.2f} m, 시각 {r['t']-r['ref_t']:+.2f} s")
json.dump(out, open(os.path.join(HERE, "speed_reach_newmu.json"), "w"), ensure_ascii=False, indent=1)

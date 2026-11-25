#!/usr/bin/python3
"""SLAM 튐이 빠른 회전 때 몰리나 — 판마다 주행 −0.5~9.5 s 에서
  튐 = 연속 /tracked_pose 이동이 추정기 이동과 3 cm 넘게 다름 (compare_all.py 와 같음),
  ω = 추정기 요 각도 (/state_estimator/odom) 의 0.1 s 중앙 차분 (튐 순간의 SLAM 두 표본 가운데 시각).
  출력: 시간 중 |ω| > 0.5 rad/s 몫, 튐 중 |ω| > 0.5 몫, 회전 빠름 / 느림 구간별 초당 튐 수.
근거: ../2026-11-21_lidar_timing/RESULTS_KO.md — LaserScan 칸 순서로 낸 점별 시각이 틀려 |ω| > 0.5 rad/s 에서 지도 정합 거리가 77 → 94~106 mm.
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/jump_vs_rotation.py"""
import importlib.util, json, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"
MPC = f"{WS}/src/amr_basic_stack/controller/mpc"; WHY = f"{WS}/analysis/torque_mpc_real/2026-11-20_why_c"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", f"{WHY}/why_c.py"); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)
rows = json.load(open(os.path.join(HERE, "compare_all.json")))
out = []
print("| 판 | 조건 | 튐 | |ω|>0.5 시간 몫 | 튐 중 |ω|>0.5 몫 | 초당 튐 |ω|>0.5 / ≤0.5 | 최대 |ω| [rad/s] |")
for x in rows:
    path = f"{WS}/bags/{x['bag']}"
    S, E, P, A = w.load(path); D = cs.load(path)
    Pp = np.array(D["pose"]); xc, yc = cs.imu_to_center(Pp[:, 1], Pp[:, 2], Pp[:, 3], w.off)
    ex, ey = np.interp(Pp[:, 0], E[:, 0], E[:, 1]), np.interp(Pp[:, 0], E[:, 0], E[:, 2])
    jump = np.linalg.norm(np.c_[np.diff(xc), np.diff(yc)] - np.c_[np.diff(ex), np.diff(ey)], axis=1)
    tm = 0.5 * (Pp[1:, 0] + Pp[:-1, 0])
    yaw = np.unwrap(E[:, 3]); te = E[:, 0]
    om = lambda t: np.abs((np.interp(t + 0.05, te, yaw) - np.interp(t - 0.05, te, yaw)) / 0.1)
    rt = np.array([a for a, _ in D["ref"]]); arm = cs.armed_mask(Pp[:, 0], D["armed"])
    sp = np.array([np.hypot(*D["ref"][i][1].at(t)[1][:2]) if i >= 0 else 0.0
                   for t, i in zip(Pp[:, 0], np.searchsorted(rt, Pp[:, 0], side="right") - 1)])
    T0 = Pp[np.flatnonzero(arm & (sp > 0.02))[0], 0]
    win = (tm > T0 - 0.5) & (tm < T0 + 9.5); jm = win & (jump > 0.03)
    tt = np.arange(T0 - 0.5, T0 + 9.5, 0.01); ot = om(tt); fast_t = ot > 0.5
    oj = om(tm[jm]); fj = oj > 0.5
    rate_f = fj.sum() / (fast_t.sum() * 0.01) if fast_t.any() else float("nan")
    rate_s = (~fj).sum() / ((~fast_t).sum() * 0.01)
    o = dict(lab=x["lab"], cond=x["cond"], jumps=int(jm.sum()), fast_time=100 * float(fast_t.mean()),
             fast_jump=100 * float(fj.mean()) if jm.any() else float("nan"), rate_fast=float(rate_f), rate_slow=float(rate_s), om_max=float(ot.max()))
    out.append(o)
    print(f"| {o['lab']} | {o['cond']} | {o['jumps']} | {o['fast_time']:.0f} % | {o['fast_jump']:.0f} % | {o['rate_fast']:.2f} / {o['rate_slow']:.2f} | {o['om_max']:.2f} |")
allf = sum(o["jumps"] * o["fast_jump"] / 100 for o in out); allj = sum(o["jumps"] for o in out)
print(f"전체: 튐 {allj} 개 중 |ω|>0.5 에서 {allf:.0f} 개 ({100*allf/allj:.0f} %), 시간 몫 평균 {np.mean([o['fast_time'] for o in out]):.0f} %")
json.dump(out, open(os.path.join(HERE, "jump_vs_rotation.json"), "w"), ensure_ascii=False, indent=1)

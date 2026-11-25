#!/usr/bin/python3
"""FL 조인 뒤 circles8 다시 (2026-11-22): 앞방향 어긋남 β_SLAM − β_바퀴 (좌·우회전), 바퀴별 c_x·c_ω, λ 바퀴별 중앙값·지표, SLAM 기준 추종.
비교: 2026-11-21 A/B '지금' 판 (같은 명령, 같은 추정기 게인)."""
import math, os, sys
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, H); sys.path.insert(0, os.path.join(H, "../2026-11-21_kf_gain_ab")); sys.path.insert(0, os.path.join(H, "../2026-11-22_slip_timing"))
import fit_kinematics as F
import analyze as AB
import slip_timing as ST
RUNS = [("2026-11-21", "225012"), ("2026-11-21", "225104"), ("2026-11-21", "225157"), ("2026-11-22", "205437")]
WH = ["FL", "FR", "RL", "RR"]
print("| bag | β 어긋남 전체 ° | 좌회전 ° | 우회전 ° | 선회 무관 (좌+우)/2 ° | c_x FL/FR/RL/RR | c_ω FL/FR/RL/RR | 속력 배율 SLAM/바퀴 |")
print("|---|---|---|---|---|---|---|---|")
for day, run in RUNS:
    S = F.bag_samples(f"bags/{day}/{run}")
    vx, vy, wz, W = S[:, 0], S[:, 1], S[:, 2], S[:, 3:7]
    vxw = 0.01 * W.sum(1); vyw = 0.01 * (-W[:, 0] + W[:, 1] + W[:, 2] - W[:, 3])
    m = np.hypot(vx, vy) > 0.3
    db = np.degrees(np.angle(np.exp(1j * (np.arctan2(vy, vx) - np.arctan2(vyw, vxw)))))
    L, R = np.median(db[m & (wz > 0.3)]), np.median(db[m & (wz < -0.3)])
    cx, cw = [], []
    for i in range(4):
        y = W[:, i] - F.SY[i] * vy / F.RW
        c, _ = F.robust_fit(np.stack([vx, wz], 1), y); cx.append(c[0]); cw.append(c[1])
    print(f"| {run} | {np.median(db[m]):+.2f} | {L:+.2f} | {R:+.2f} | {(L+R)/2:+.2f} | {' / '.join(f'{x:.2f}' for x in cx)} | {' / '.join(f'{x:+.2f}' for x in cw)} | "
          f"{np.median(np.hypot(vx,vy)[m]/np.hypot(vxw,vyw)[m]):.4f} |")
print("\n| bag | τ̄ (로그) | SLAM 추종 |e| RMS cm | 경로 거리 RMS cm | 헤딩 RMS ° | 큰 슬립 % (SLAM λ) | λ RMS (SLAM) | 큰 슬립 % (logger) | logger λ 바퀴별 중앙값 FL/FR/RL/RR |")
print("|---|---|---|---|---|---|---|---|---|")
for day, run in RUNS:
    AB.ROOT = os.path.join(H, "../../../bags", day)
    m = AB.met(run)
    D = ST.read(f"bags/{day}/{run}")
    St = np.array([[t, mm.data[20] if len(mm.data) > 20 else 0] for t, mm in D["/torque_mpc/status"]]); act = St[St[:, 1] == 0, 0]
    E = np.array([[ST.st(mm.header), mm.twist.twist.linear.x, mm.twist.twist.linear.y, mm.twist.twist.angular.z] for _, mm in D["/state_estimator/odom"]])
    Wt = np.array([[ST.st(mm.header), *mm.velocity] for _, mm in D["/wheel_states"]])
    k = (E[:, 0] >= act[0]) & (E[:, 0] <= act[-1]) & (np.hypot(E[:, 1], E[:, 2]) > 0.05)
    w = np.stack([np.interp(E[k, 0], Wt[:, 0], Wt[:, 1 + i]) for i in range(4)], 1)
    lam = ST.lam(w, E[k, 1], E[k, 2], E[k, 3])
    tb = AB.tau_bar(run)
    print(f"| {run} | {np.round(tb, 3).tolist() if tb is not None else '-'} | {m['exy']*100:.1f} | {m['dist']*100:.1f} | {m['eth']:.2f} | {m['sl_big']:.1f} | {m['sl_lrms']:.3f} | "
          f"{m['lg_big']:.1f} | {' / '.join(f'{x:+.3f}' for x in np.nanmedian(lam, 0))} |")

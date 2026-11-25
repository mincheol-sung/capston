#!/usr/bin/python3
"""③ 옆걸음 판 분석 (2026-11-22): 옆으로 갈 때의 진행 방향 어긋남, 바퀴별 회전 (공칭 대비), 바퀴별 옆방향 계수 c_y.
기준 = SLAM (중심 중앙 차분 속도·yaw rate, 바퀴 ω 는 같은 0.2 s 창 평균, 정속 표본) — fit_kinematics.bag_samples."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fit_kinematics as F
WH = ["FL", "FR", "RL", "RR"]
for b in sys.argv[1:]:
    S = F.bag_samples(b)
    vx, vy, wz, W = S[:, 0], S[:, 1], S[:, 2], S[:, 3:7]
    vxw = 0.01 * W.sum(1); vyw = 0.01 * (-W[:, 0] + W[:, 1] + W[:, 2] - W[:, 3])
    lat = np.abs(vy) > 0.15
    print(f"== {b}: 정속 표본 {len(S)}, 옆걸음 표본 {lat.sum()} (왼쪽 {(lat & (vy > 0)).sum()}, 오른쪽 {(lat & (vy < 0)).sum()})")
    for lab, m in (("왼쪽 (v_y > 0)", lat & (vy > 0)), ("오른쪽 (v_y < 0)", lat & (vy < 0)), ("옆걸음 전체", lat)):
        if m.sum() < 5:
            continue
        db = np.degrees(np.angle(np.exp(1j * (np.arctan2(vy, vx) - np.arctan2(vyw, vxw)))))[m]
        print(f"   {lab}: SLAM |v| {np.hypot(vx,vy)[m].mean():.3f} m/s, 바퀴 |v| {np.hypot(vxw,vyw)[m].mean():.3f} m/s (배율 {np.median(np.hypot(vx,vy)[m]/np.hypot(vxw,vyw)[m]):.3f}), "
              f"β_SLAM − β_바퀴 중앙값 {np.median(db):+.2f}° [사분위 {np.percentile(db,25):+.2f}~{np.percentile(db,75):+.2f}], SLAM ω 평균 {wz[m].mean():+.3f} rad/s")
    # 바퀴별: 공칭 ω_nom = T⁻¹ [v_SLAM, ω_SLAM] 대비 실제 ω
    wn = (F.SX[None] * vx[:, None] + F.SY[None] * vy[:, None] + F.SW[None] * F.K * wz[:, None]) / F.RW
    print("   바퀴별 (옆걸음 정속): |ω| 평균 rad/s, |ω| / |ω_nom| 중앙값, λ (SLAM 기준) 중앙값")
    for i in range(4):
        m = lat & (np.abs(wn[:, i]) > 2.0)
        lam = (W[m, i] - wn[m, i]) * np.sign(wn[m, i]) / np.maximum(np.maximum(np.abs(W[m, i]), np.abs(wn[m, i])), 2.5)
        print(f"     {WH[i]}: |ω| {np.abs(W[m,i]).mean():5.2f} (공칭 {np.abs(wn[m,i]).mean():5.2f}), 비 {np.median(np.abs(W[m,i])/np.abs(wn[m,i])):.3f}, λ {np.median(lam):+.3f}")
    # 바퀴별 c_y (옆걸음 표본, c_x·c_ω 는 이전 맞춤 '전체' 값으로 빼고)
    import json
    fit = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "fit.json")))["전체"]
    print("   바퀴별 옆방향 계수 c_y (공칭 ∓25) — c_x·c_ω 는 11-20~22 맞춤 값으로 빼고 옆걸음 표본에 맞춤")
    for i, w in enumerate(WH):
        y = W[lat, i] - fit[w][0] * vx[lat] - fit[w][1] * wz[lat]
        cy = float(np.dot(vy[lat], y) / np.dot(vy[lat], vy[lat]))
        print(f"     {w}: c_y {cy:+.2f} (공칭 {F.SY[i]*25:+.0f}) → |c_y|/25 = {abs(cy)/25:.3f}")
    par = W[:, 0] + W[:, 1] - W[:, 2] - W[:, 3]
    print(f"   바퀴 parity |c| 중앙값: 옆걸음 {np.median(np.abs(par[lat])):.2f} rad/s")

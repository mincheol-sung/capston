#!/usr/bin/python3
"""② 벽 평행 시험 분석 (2026-11-22). 로봇 옆면(또는 앞면)을 곧은 벽과 평행하게 놓고 정지 녹화한 bag 에서,
앞·뒤 라이다 각각의 스캔을 실행 TF 로 base_footprint 에 옮겨 벽 직선을 RANSAC 으로 찾고, base 축(x 또는 y)에 대한 기울기를 낸다.
차체가 벽과 평행이면 그 기울기 = −(그 라이다의 yaw 오차 ε) (ε + = 반시계로 돌아감). 진행 방향 어긋남 β_SLAM − β_바퀴 = +2.67° 가
라이다 탓이면 ε = −2.67° → 벽이 +2.67° 로 보인다 (2026-11-22 정정: 처음에 −2.7° 라고 적었던 것은 부호가 틀렸다). 0° 면 바퀴·차체 쪽.
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-22_wheel_kinematics/wall_test.py bags/<날짜>/<시분초> [벽 방향: left|right|front|rear, 기본 left]"""
import math, sys
import numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import lidar_relative_yaw as L

b = sys.argv[1]; side = sys.argv[2] if len(sys.argv) > 2 else "left"
F, Rr, W, TF = L.load(b)
want = {"left": (0.0, +1), "right": (0.0, -1), "front": (90.0, +1), "rear": (90.0, -1)}[side]   # 벽 직선 방향(°), 벽이 있는 쪽
for name, scans, frame in (("앞 라이다", F, "laser_front"), ("뒤 라이다", Rr, "laser_rear")):
    P = L.cloud(scans[len(scans) // 4: 3 * len(scans) // 4], L.to_base(TF, frame), rmin=0.2, rmax=3.0)
    best = None
    for Q in L.ransac_lines(P, n_lines=6, tol=0.008, min_in=80):
        a = math.degrees(L.line_angle(Q)); a = (a - want[0] + 90) % 180 - 90          # 기준 축에 대한 기울기
        c = Q.mean(0); off = c[1] if want[0] == 0.0 else c[0]                       # 벽까지 거리 (y 또는 x)
        span = np.ptp((Q - c) @ np.array([math.cos(math.radians(want[0])), math.sin(math.radians(want[0]))]))
        if abs(a) < 15 and np.sign(off) == want[1] and span > 0.5 and (best is None or abs(off) < abs(best[1])):
            best = (a, off, span, len(Q))
    if best:
        print(f"{name}: 벽 직선 기울기 {best[0]:+.2f}° (거리 {abs(best[1]):.3f} m, 길이 {best[2]:.2f} m, 점 {best[3]})")
    else:
        print(f"{name}: {side} 쪽 벽 직선을 못 찾음")

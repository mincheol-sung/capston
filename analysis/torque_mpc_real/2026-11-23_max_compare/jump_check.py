#!/usr/bin/python3
"""첫 꼭짓점 앞 SLAM 옆 오차 81 mm / 0.1 s 가 실제 움직임인가 — 0.5 s 의 SLAM 포즈에서 출발해
 (1) 바퀴 기구학 T⁺ω (몸체 vx, vy) + 자이로 ω_z 적분 (추측항법), (2) IMU 몸체 가속도 a_y, (3) 상태 추정기 위치 를 SLAM 과 비교.
 옆 성분 = 기준 변 방향의 왼쪽 법선 (+ 안쪽)."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from load import load, ref_at
B = "/home/smoc/capston/capston/bags/2026-11-23"; bag = sys.argv[1] if len(sys.argv) > 1 else "121319"
d = load(f"{B}/{bag}"); D = d["D"]; sl, E, W, G = d["slam"], d["E"], d["W"], d["G"]; a0 = d["arm"][0]
rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=["/imu/base_link"])); A = []
while rd.has_next():
    tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp]))
    A.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.linear_acceleration.x, m.linear_acceleration.y])
A = np.array(A)
ts = np.arange(a0, a0 + 30, 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts]); tr0 = ts[np.argmax(sp > 0.02)]
r, k = 0.04, 0.1932
Tp = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-1 / k, 1 / k, -1 / k, 1 / k]]) * r / 4      # 바퀴 ω → 몸체 [vx, vy, ω]
Xr0, _ = ref_at(D, tr0 + 0.5); n0 = np.array([-math.sin(Xr0[2]), math.cos(Xr0[2])])          # 변 0 왼쪽 법선 (안쪽)
ta = tr0 + 0.5; i = np.argmin(np.abs(sl[:, 0] - ta)); ta = sl[i, 0]
p = sl[i, 1:3].copy(); psi = sl[i, 3]; dt = 0.002; tt = np.arange(ta, tr0 + 1.75, dt); DR = []
for t in tt:
    w = np.array([np.interp(t, W[:, 0], W[:, 1 + j]) for j in range(4)]); vb = Tp @ w
    psi += np.interp(t, G[:, 0], G[:, 1]) * dt
    p += dt * np.array([math.cos(psi) * vb[0] - math.sin(psi) * vb[1], math.sin(psi) * vb[0] + math.cos(psi) * vb[1]]); DR.append(p.copy())
DR = np.array(DR)
print(f"{bag}: 기준 출발 기준 시각, 0.5 s 의 SLAM 포즈에서 출발. 옆 = 변 0 의 안쪽 법선 (+ 안)")
print(" t   | SLAM 옆 [mm] | 바퀴+자이로 추측항법 옆 | 추정기 옆 | SLAM − 추측항법 | IMU a_y 0.05 s 평균 [m/s²] | 바퀴 몸체 v_y [m/s]")
for t in np.arange(0.5, 1.71, 0.05):
    tq = tr0 + t; ks = np.argmin(np.abs(sl[:, 0] - tq)); slp = np.array([np.interp(tq, sl[:, 0], sl[:, 1]), np.interp(tq, sl[:, 0], sl[:, 2])])
    dr = np.array([np.interp(tq, tt, DR[:, 0]), np.interp(tq, tt, DR[:, 1])])
    ep = np.array([np.interp(tq, E[:, 0], E[:, 1]), np.interp(tq, E[:, 0], E[:, 2])])
    base = sl[i, 1:3]
    m_ = (A[:, 0] > tq - 0.025) & (A[:, 0] < tq + 0.025)
    w = np.array([np.interp(tq, W[:, 0], W[:, 1 + j]) for j in range(4)])
    print(f"{t:4.2f} | {(slp - base) @ n0 * 1e3:+7.0f} | {(dr - base) @ n0 * 1e3:+7.0f} | {(ep - base) @ n0 * 1e3:+7.0f} | {(slp - dr) @ n0 * 1e3:+7.0f} | "
          f"{A[m_, 2].mean():+6.2f} | {(Tp @ w)[1]:+5.2f}   (SLAM 표본 {abs(sl[ks, 0] - tq) * 1e3:.0f} ms 떨어짐)")
# SLAM 원 표본 (10 Hz) 옆 성분 차분
m2 = (sl[:, 0] > tr0 + 0.5) & (sl[:, 0] < tr0 + 1.7)
q = sl[m2]; lat = (q[:, 1:3] - sl[i, 1:3]) @ n0
print("SLAM 표본별 옆 [mm]:", " ".join(f"{x * 1e3:+.0f}@{(tq - tr0):.2f}" for x, tq in zip(lat, q[:, 0])))

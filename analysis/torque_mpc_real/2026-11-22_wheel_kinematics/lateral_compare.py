#!/usr/bin/python3
"""옆걸음 판끼리 비교 (2026-11-22): 바퀴별 큰 슬립 % (추정 속도 기준 λ, |λ| > 0.5), |ω| 최대 / 공칭 최대,
'혼자 돎' 시간 (몸체 |v| < 0.05 m/s 인데 그 바퀴만 |ω| > 3 rad/s, 나머지 셋 < 1), 왼쪽·오른쪽 정속 구간 바퀴 |ω|, 제어기 오차 e_y 최대."""
import sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
WR, KG, EPS = 0.04, 0.1932, 2.5
WH = ["FL", "FR", "RL", "RR"]
print("| bag | 바퀴 | 큰 슬립 % | |ω| 최대 (공칭 최대) rad/s | 혼자 돎 s | 왼쪽 정속 |ω| | 오른쪽 정속 |ω| |")
print("|---|---|---|---|---|---|---|")
for b in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/wheel_states", "/state_estimator/odom", "/torque_mpc/status", "/dra_mppi/reference_trajectory"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while r.has_next():
        n, d, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
    S = np.array([[t, m.data[20] if len(m.data) > 20 else 0, m.data[2]] for t, m in D["/torque_mpc/status"]]); act = S[S[:, 1] == 0]
    t0, t1 = act[0, 0], act[-1, 0]
    E = np.array([[st(m.header), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for _, m in D["/state_estimator/odom"]])
    W = np.array([[st(m.header), *m.velocity] for _, m in D["/wheel_states"]])
    R = np.array(sorted((st(m.header) + m.dt, rr.vy) for _, m in D["/dra_mppi/reference_trajectory"] if m.references for rr in m.references[:1]))
    m = (E[:, 0] >= t0) & (E[:, 0] <= t1); te = E[m, 0]; vx, vy, wz = E[m, 1], E[m, 2], E[m, 3]
    w = np.stack([np.interp(te, W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    wn = np.stack([vx - vy - KG * wz, vx + vy + KG * wz, vx + vy - KG * wz, vx - vy + KG * wz], 1) / WR
    lam = (w - wn) * np.sign(wn) / np.maximum(np.maximum(np.abs(w), np.abs(wn)), EPS)
    sp = np.hypot(vx, vy); mov = sp > 0.05
    vyr = np.interp(te, R[:, 0], R[:, 1], left=0, right=0)
    left, right = (vyr > 0.28) & mov, (vyr < -0.28) & mov
    for i in range(4):
        others = np.delete(np.abs(w), i, axis=1)
        alone = (sp < 0.05) & (np.abs(w[:, i]) > 3) & (others.max(1) < 1)
        print(f"| {b.split('/')[-1]} | {WH[i]} | {100*np.nanmean(np.abs(lam[mov, i]) > 0.5):.1f} | {np.abs(w[:,i]).max():.1f} ({np.abs(wn[:,i]).max():.1f}) | "
              f"{alone.sum()*0.01:.2f} | {np.abs(w[left,i]).mean():.1f} | {np.abs(w[right,i]).mean():.1f} |")
    print(f"| {b.split('/')[-1]} | 제어기 오차 \\|e_y\\| 최대 {np.abs(act[:,2]).max()*100:.1f} cm, 추종 {t1-t0:.1f} s, 왼쪽 정속 {left.sum()*0.01:.1f} s · 오른쪽 {right.sum()*0.01:.1f} s | | | | | |")

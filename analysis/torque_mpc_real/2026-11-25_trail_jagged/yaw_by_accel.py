#!/usr/bin/python3
"""세 후보 가르기 (2026-11-25): SLAM 방향 흔들림 (SLAM Δψ − 자이로 적분, 0.5 s 이동중앙값 뺌) 을 요 각가속도 |ψ̈| × 진행 속도로.
자이로를 기준으로 하므로 바퀴 미끄럼과 무관하다.
  후보 (b) 자이로·라이다 시각 차 δ 만이면: 회전 보정 오차가 ψ̇·δ → 흔들림은 |ψ̈| 에만 달리고 제자리 회전에서도 같은 크기.
  후보 (a) 이동 예측 (바퀴 오돔 없이 등속) 이 틀어지는 것이면: 진행 속도가 0 이면 안 생긴다.
같이: 위치 흔들림 (turn_events.py 와 같은 정의) 도 같은 칸으로.
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_trail_jagged/yaw_by_accel.py bag [bag ...]   → out_yaw_by_accel.md"""
import math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__))
OFF = np.array([0.1093, 0.017]); R_W = 0.04
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
P = []
for BAG in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/tracked_pose", "/wheel_states", "/imu/base_link"]))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    S, W, G = [], [], []
    while r.has_next():
        n, d, _ = r.read_next(); m = deserialize_message(d, ty[n])
        if n == "/tracked_pose":
            p = m.pose; ps = yaw_of(p.orientation); c, s = math.cos(ps), math.sin(ps)
            S.append((st(m.header), p.position.x - (c * OFF[0] - s * OFF[1]), p.position.y - (s * OFF[0] + c * OFF[1]), ps))
        elif n == "/wheel_states":
            W.append((st(m.header), *m.velocity))
        else:
            G.append((st(m.header), m.angular_velocity.z))
    if min(len(S), len(W), len(G)) < 50: continue
    S, W, G = map(np.array, (S, W, G))
    tw = W[:, 0]; om = W[:, 1:5]
    vxw = R_W / 4 * om.sum(1); vyw = R_W / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3])
    cx = np.concatenate([[0], np.cumsum(vxw[1:] * np.diff(tw))]); cy = np.concatenate([[0], np.cumsum(vyw[1:] * np.diff(tw))])
    tg = G[:, 0]; fg = 1 / np.median(np.diff(tg)); b, a = butter(2, 5 / (fg / 2))
    wz = filtfilt(b, a, G[:, 1]); wdot = np.gradient(wz, tg)
    cg = np.concatenate([[0], np.cumsum(0.5 * (G[1:, 1] + G[:-1, 1]) * np.diff(tg))])
    ts = S[:, 0]; tm = 0.5 * (ts[1:] + ts[:-1]); ps = np.unwrap(S[:, 3])
    dtr = lambda x: np.array([x[i] - np.median(x[np.abs(tm - tm[i]) <= 0.25]) for i in range(len(x))])
    hy = np.abs(dtr(np.diff(ps) - np.diff(np.interp(ts, tg, cg))))                       # 방향 흔들림 [rad]
    ym = 0.5 * (ps[1:] + ps[:-1]); dx, dy = np.diff(S[:, 1]), np.diff(S[:, 2])
    rx = (np.cos(ym) * dx + np.sin(ym) * dy) - np.diff(np.interp(ts, tw, cx))
    ry = (-np.sin(ym) * dx + np.cos(ym) * dy) - np.diff(np.interp(ts, tw, cy))
    hp = np.hypot(dtr(rx), dtr(ry))                                                       # 위치 흔들림 [m]
    sp = np.hypot(np.interp(tm, tw, vxw), np.interp(tm, tw, vyw))
    yr = np.abs(np.interp(tm, tg, wz))
    ya = np.array([np.abs(wdot[(tg >= t - 0.1) & (tg <= t + 0.1)]).max(initial=0.0) for t in tm])
    P.append(np.c_[ya, yr, sp, hy, hp])
P = np.vstack(P)
out = ["# SLAM 방향·위치 흔들림 × 요 각가속도 × 진행 속도 (11-25 실차 판, 방향은 자이로 기준)\n",
       "칸 = 방향 흔들림 RMS ° / 위치 흔들림 RMS mm (간격 수)\n",
       "| |ψ̈| rad/s² | 제자리 (≤ 0.05 m/s) | 0.05~0.3 | 0.3~0.8 | > 0.8 m/s |", "|---|---|---|---|---|"]
for lo, hi in ((0, 2), (2, 4), (4, 8), (8, 16), (16, 999)):
    cells = []
    for vlo, vhi in ((0, 0.05), (0.05, 0.3), (0.3, 0.8), (0.8, 9)):
        m = (P[:, 0] >= lo) & (P[:, 0] < hi) & (P[:, 2] >= vlo) & (P[:, 2] < vhi)
        cells.append(f"{math.degrees(np.sqrt(np.mean(P[m, 3] ** 2))):.2f}° / {1e3 * np.sqrt(np.mean(P[m, 4] ** 2)):.1f} ({m.sum()})" if m.sum() >= 10 else "-")
    out.append(f"| {lo}~{hi} | " + " | ".join(cells) + " |")
m = (P[:, 0] >= 8) & (P[:, 2] <= 0.05)
out.append(f"\n제자리에서 |ψ̈| ≥ 8 인 간격의 요 각속도 중앙값 {np.median(P[m, 1]):.2f} rad/s, 움직이며 (> 0.3 m/s) |ψ̈| ≥ 8 인 간격 {np.median(P[(P[:, 0] >= 8) & (P[:, 2] > 0.3), 1]):.2f} rad/s")
open(os.path.join(H, "out_yaw_by_accel.md"), "w").write("\n".join(out) + "\n")
print("\n".join(out))

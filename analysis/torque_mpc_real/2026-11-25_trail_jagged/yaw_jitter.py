#!/usr/bin/python3
"""SLAM 방향 (yaw) 흔들림: SLAM 포즈 두 개 사이 Δψ − 자이로 적분 (2026-11-25). 0.5 s 이동중앙값을 빼서 (자이로 바이어스·느린 몫 제거)
요 각속도 구간별 RMS. 경로 종류 (bag 이름 두 번째 칸) 별로 모은다.
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_trail_jagged/yaw_jitter.py bag [bag ...]"""
import math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rows = {}
for BAG in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/tracked_pose", "/imu/base_link"])); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    S, G = [], []
    while r.has_next():
        n, d, _ = r.read_next(); m = deserialize_message(d, ty[n])
        if n == "/tracked_pose":
            S.append((st(m.header), yaw_of(m.pose.orientation)))
        else:
            G.append((st(m.header), m.angular_velocity.z))
    if len(S) < 50 or len(G) < 50: continue
    S, G = np.array(S), np.array(G)
    cg = np.concatenate([[0], np.cumsum(0.5 * (G[1:, 1] + G[:-1, 1]) * np.diff(G[:, 0]))])
    ts = S[:, 0]; tm = 0.5 * (ts[1:] + ts[:-1]); ps = np.unwrap(S[:, 1])
    res = np.diff(ps) - np.diff(np.interp(ts, G[:, 0], cg))
    yr = np.abs(np.diff(ps) / np.diff(ts))
    h = np.array([res[i] - np.median(res[np.abs(tm - tm[i]) <= 0.25]) for i in range(len(res))])
    kind = os.path.basename(BAG).split("_")[1]
    for lo, hi in ((0, 0.05), (0.05, 1.0), (1.0, 2.0), (2.0, 99)):
        rows.setdefault((kind, f"{lo}~{hi}"), []).append(h[(yr >= lo) & (yr < hi)])
print("| 경로 | 요 각속도 rad/s | 간격 수 | SLAM Δψ − 자이로 흔들림 RMS ° | 95 % ° |\n|---|---|---|---|---|")
for (kind, lab), hs in sorted(rows.items()):
    h = np.concatenate(hs)
    if len(h) >= 5:
        print(f"| {kind} | {lab} | {len(h)} | {math.degrees(np.sqrt(np.mean(h ** 2))):.2f} | {math.degrees(np.percentile(np.abs(h), 95)):.2f} |")

#!/usr/bin/python3
"""path following 실차 판 — /figure8/progress [t, t_ref, t_clock, t_proj, 묶음] 에서 기준 시각을 얼마나 묶었나.
  주행 구간 (0 ≤ t_ref ≤ T_path) 표본 중 묶음 +1 (기다림) · −1 (앞당김) · 0 비율, t_clock − t_ref (시계보다 늦은 기준 시각) 최대·끝,
  t_ref − t_proj (로봇 정사영보다 앞선 기준) 분포. 사용: pf_progress.py 폴더 ...  → 표 + pf_progress.npz (그림용)"""
import os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
B = "/home/smoc/capston/capston/bags/2026-11-23"; HERE = os.path.dirname(os.path.abspath(__file__))
T_PATH = 20.15
store = {}
print("| 판 | 표본 | 기다림 % | 앞당김 % | 시계대로 % | 늦음 최대 / 끝 [s] | t_ref − t_proj 중앙 / 95 % [s] |")
for bag in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/figure8/progress"])); P = []
    while rd.has_next():
        _, d, _ = rd.read_next(); P.append(list(deserialize_message(d, Float64MultiArray).data))
    P = np.array(P); store[bag] = P
    m = (P[:, 1] >= 0) & (P[:, 1] <= T_PATH)
    f = P[m, 4]; lag = P[m, 2] - P[m, 1]; ahead = (P[m, 1] - P[m, 3])[np.isfinite(P[m, 3])]
    print(f"| {bag} | {m.sum()} | {100 * np.mean(f > 0.5):.0f} | {100 * np.mean(f < -0.5):.0f} | {100 * np.mean(np.abs(f) < 0.5):.0f} | "
          f"{lag.max():.2f} / {lag[-1]:.2f} | {np.median(ahead):+.3f} / {np.percentile(ahead, 95):+.3f} |")
np.savez(os.path.join(HERE, "pf_progress.npz"), **store)

#!/usr/bin/python3
"""바퀴 오돔을 Cartographer 예측에 넣으면 슬립 때 SLAM 이 끌려가나 (2026-11-25).
같은 bag 의 재생 두 판 (기준 = 오돔 없음, 오돔 = 예측에만) 위치 차를 바퀴 슬립 크기별로 본다.
슬립 = /slip_logger/slip λ (바퀴 4 개 중 최대 |λ|, 10 Hz filtfilt). 차가 슬립과 같이 커지면 오돔이 SLAM 을 끈 것.
  /usr/bin/python3 slip_check.py bag base.csv odom.csv"""
import csv, math, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
BAG, FB, FO = sys.argv[1:4]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=["/slip_logger/slip", "/tracked_pose"])); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
L, REC = [], []
while r.has_next():
    n, d, _ = r.read_next(); m = deserialize_message(d, ty[n])
    if n == "/slip_logger/slip" and len(m.data) >= 5:
        L.append(m.data[:5])
    elif n == "/tracked_pose":
        REC.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x, m.pose.position.y))
L, REC = np.array(L), np.array(REC)
lam = np.where(np.isfinite(L[:, 1:5]), L[:, 1:5], 0.0)
b, a = butter(1, 10 / (1 / np.median(np.diff(L[:, 0])) / 2)); lf = np.abs(filtfilt(b, a, lam, axis=0)).max(1)
rd = lambda f: np.array([[float(x["stamp"]), float(x["x"]), float(x["y"])] for x in csv.DictReader(open(f))])
B, O = rd(FB), rd(FO)
t = B[:, 0]; m = (t >= O[0, 0]) & (t <= O[-1, 0]) & (t >= L[0, 0]) & (t <= L[-1, 0])
dd = np.hypot(B[m, 1] - np.interp(t[m], O[:, 0], O[:, 1]), B[m, 2] - np.interp(t[m], O[:, 0], O[:, 2]))
dr_b = np.hypot(B[m, 1] - np.interp(t[m], REC[:, 0], REC[:, 1]), B[m, 2] - np.interp(t[m], REC[:, 0], REC[:, 2]))
dr_o = np.hypot(np.interp(t[m], O[:, 0], O[:, 1]) - np.interp(t[m], REC[:, 0], REC[:, 1]), np.interp(t[m], O[:, 0], O[:, 2]) - np.interp(t[m], REC[:, 0], REC[:, 2]))
s = np.interp(t[m], L[:, 0], lf)
print("| 바퀴 슬립 max|λ̃| | 포즈 수 | 오돔 판 − 기준 판 거리 중앙값 / 최대 mm | 기준 판 − 실차 기록 중앙값 mm | 오돔 판 − 실차 기록 중앙값 mm |\n|---|---|---|---|---|")
for lo, hi in ((0, 0.1), (0.1, 0.25), (0.25, 0.5), (0.5, 9)):
    k = (s >= lo) & (s < hi)
    if k.sum() >= 3:
        print(f"| {lo}~{hi} | {k.sum()} | {1e3 * np.median(dd[k]):.1f} / {1e3 * dd[k].max():.1f} | {1e3 * np.median(dr_b[k]):.1f} | {1e3 * np.median(dr_o[k]):.1f} |")

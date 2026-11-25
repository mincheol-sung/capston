#!/usr/bin/python3
"""IMU 가속도·자이로 잡음 밀도를 bag 에서 직접 (독립 경로): Welch PSD, 정지 구간 / 주행 구간.
   밀도 √S (두쪽 PSD, Q 의 q_a 와 같은 규약: 표본 분산 = S·f_s) = √(G/2), G = 한쪽 PSD."""
import sys, numpy as np, rosbag2_py
from scipy.signal import welch
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
b = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=["/imu/base_link", "/state_estimator/odom"]))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
I, O = [], []
while r.has_next():
    n, d, _ = r.read_next(); m = deserialize_message(d, types[n]); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if n == "/imu/base_link":
        I.append((t, m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z))
    else:
        O.append((t, np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)))
I, O = np.array(I), np.array(O)
fs = 1 / np.median(np.diff(I[:, 0]))
spd = np.interp(I[:, 0], O[:, 0], O[:, 1], left=0, right=0)
def runs(mask, min_len):
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return [(a, b) for a, b in zip(idx[::2], idx[1::2]) if b - a >= min_len]
print(f"IMU {fs:.0f} Hz, {len(I)} 표본")
for name, mask in (("정지 (추정 속도 < 0.01, 출력 있는 구간)", (spd < 0.01) & (spd > 0) | ((spd == 0) & (I[:, 0] > O[0, 0]) & (I[:, 0] < O[-1, 0]))), ("주행 (> 0.2 m/s)", spd > 0.2)):
    segs = runs(mask, int(2 * fs))
    if not segs:
        print(name, "구간 없음"); continue
    out = {}
    for c, lab in ((1, "a_x"), (2, "a_y"), (3, "ω_z")):
        G = []; F = None
        for a, b_ in segs:
            F, g = welch(I[a:b_, c] - I[a:b_, c].mean(), fs=fs, nperseg=int(fs))
            G.append(g * (b_ - a))
        G = np.sum(G, 0) / sum(b_ - a for a, b_ in segs)
        dens = lambda lo, hi: float(np.sqrt(np.mean(G[(F >= lo) & (F < hi)]) / 2))
        out[lab] = (dens(1, 5), dens(5, 20), dens(20, 100), dens(100, fs / 2), float(np.sqrt(np.sum(G) * (F[1] - F[0]))))
    tot = sum(b_ - a for a, b_ in segs) / fs
    print(f"\n{name}: {len(segs)} 구간 {tot:.1f} s — 밀도 √S [단위/√Hz] 대역별, 전체 σ")
    print("| 축 | 1–5 Hz | 5–20 Hz | 20–100 Hz | 100 Hz–나이퀴스트 | σ (전 대역) |")
    for lab, v in out.items():
        print(f"| {lab} | " + " | ".join(f"{x:.4f}" for x in v[:4]) + f" | {v[4]:.3f} |")

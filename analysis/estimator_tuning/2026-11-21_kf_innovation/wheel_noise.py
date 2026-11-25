#!/usr/bin/python3
"""엔코더 속도 잡음 (/wheel_states velocity): 정지 구간 σ, 주행 구간 5–20 Hz 밀도, 그리고 parity c = ω_FL + ω_FR − ω_RL − ω_RR
   (T 의 null space: T·[1,1,−1,−1]ᵀ = 0 → 무슬립·강체면 0). 주행 중 c 의 분포."""
import sys, numpy as np, rosbag2_py
from scipy.signal import welch
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
b = sys.argv[1]
r_ = rosbag2_py.SequentialReader()
r_.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
r_.set_filter(rosbag2_py.StorageFilter(topics=["/wheel_states", "/state_estimator/odom"]))
types = {t.name: get_message(t.type) for t in r_.get_all_topics_and_types()}
W, O = [], []
while r_.has_next():
    n, d, _ = r_.read_next(); m = deserialize_message(d, types[n]); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if n == "/wheel_states":
        W.append((t, *m.velocity, all(m.fresh)))
    else:
        O.append((t, np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)))
W, O = np.array(W), np.array(O)
dt = np.diff(W[:, 0]); fs = 1 / np.median(dt)
print(f"/wheel_states {fs:.0f} Hz (간격 p50 {np.median(dt)*1e3:.1f} / p99 {np.percentile(dt,99)*1e3:.1f} / 최대 {dt.max()*1e3:.0f} ms), fresh 전부 true {W[:,5].mean()*100:.1f} %")
spd = np.interp(W[:, 0], O[:, 0], O[:, 1], left=np.nan, right=np.nan)
w = W[:, 1:5]; c = w[:, 0] + w[:, 1] - w[:, 2] - w[:, 3]
still = (spd < 0.005) & (np.abs(w).max(1) < 0.5)
mov = spd > 0.2
print(f"정지 {still.sum()/fs:.1f} s: 바퀴별 ω σ {np.round(w[still].std(0), 4)} rad/s, 바퀴별 |ω| 최대 {np.round(np.abs(w[still]).max(0), 3)}")
idx = np.flatnonzero(np.diff(np.r_[0, mov.astype(int), 0])); segs = [(a, b_) for a, b_ in zip(idx[::2], idx[1::2]) if b_ - a >= fs * 2]
if segs:
    a, b_ = max(segs, key=lambda s: s[1] - s[0])
    F, G = welch(w[a:b_] - w[a:b_].mean(0), fs=fs, nperseg=int(fs), axis=0)
    band = (F >= 5) & (F < 20)
    print(f"주행 {sum(e-s for s,e in segs)/fs:.1f} s: 5–20 Hz 밀도 √(G/2) 바퀴별 {np.round(np.sqrt(G[band].mean(0)/2), 4)} rad/s/√Hz")
    print(f"주행 parity c: RMS {np.sqrt(np.mean(c[mov]**2)):.3f} rad/s, p50 |c| {np.percentile(np.abs(c[mov]),50):.3f}, p95 {np.percentile(np.abs(c[mov]),95):.3f}, 최대 {np.abs(c[mov]).max():.2f} rad/s")
print(f"정지 parity c: RMS {np.sqrt(np.mean(c[still]**2)):.4f} rad/s")

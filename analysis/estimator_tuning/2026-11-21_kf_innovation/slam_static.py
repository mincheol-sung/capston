#!/usr/bin/python3
"""SLAM 포즈 오차가 시간 상관인가 (innovation whiteness 가정 점검). 로봇이 서 있는 구간(바퀴 |ω| 모두 < 0.3 rad/s, 3 s 이상)에서
   /tracked_pose 의 떨림 = 구간 평균을 뺀 값. σ 와 자기상관 ρ(lag), lag 1 = 한 스캔(약 0.1 s).
   같은 구간의 kf innovation ρ₁ 도 (지금 yaml 게인, 재생)."""
import math, os, sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, os.path.dirname(__file__))
import replay_tune as RT
from amr_mpc.core import config
b = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=["/tracked_pose", "/wheel_states"]))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}; P, W = [], []
while r.has_next():
    n, d, _ = r.read_next(); m = deserialize_message(d, ty[n]); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if n == "/tracked_pose":
        P.append((t, m.pose.position.x, m.pose.position.y, RT.yaw(m.pose.orientation)))
    else:
        W.append((t, np.abs(m.velocity).max()))
P, W = np.array(sorted(set(map(tuple, P)))), np.array(W)
wmax = np.interp(P[:, 0], W[:, 0], W[:, 1])
# 바퀴가 멈춘 지 0.5 s 지난 뒤부터 (감속 꼬리 빼기)
still = wmax < 0.3
for i in range(len(still)):
    if still[i] and np.any(wmax[(P[:, 0] > P[i, 0] - 0.5) & (P[:, 0] <= P[i, 0])] >= 0.3):
        still[i] = False
# SLAM 튐(초기 위치 지정·재측위)에서 자른다: 연속 두 포즈가 2 cm 또는 0.5° 넘게 다르면 그 앞뒤 1 s 를 뺀다
jump = np.r_[False, (np.hypot(np.diff(P[:, 1]), np.diff(P[:, 2])) > 0.02) | (np.abs(np.diff(np.unwrap(P[:, 3]))) > math.radians(0.5))]
for tj in P[jump, 0]:
    still &= np.abs(P[:, 0] - tj) > 1.0
print(f"SLAM 튐 {int(jump.sum())} 곳 (정지 판정에서 앞뒤 1 s 제외)")
idx = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0])); segs = [(a, e) for a, e in zip(idx[::2], idx[1::2]) if P[e - 1, 0] - P[a, 0] >= 3.0]
print(f"정지 구간 {len(segs)} 개, 합 {sum(P[e-1,0]-P[a,0] for a,e in segs):.1f} s, 포즈 {sum(e-a for a,e in segs)} 개")
L = 10
for c, lab, sc, u in ((1, "x", 1e3, "mm"), (2, "y", 1e3, "mm"), (3, "yaw", 180 / math.pi, "°")):
    num = np.zeros(L + 1); den = 0.0; dev = []
    for a, e in segs:
        x = P[a:e, c] - P[a:e, c].mean(); dev.append(x)
        for l in range(L + 1):
            num[l] += np.dot(x[:len(x) - l], x[l:])
    rho = num / num[0]
    print(f"  {lab}: 떨림 σ {np.std(np.concatenate(dev))*sc:.3f} {u} | ρ(lag) 1~5: " + " ".join(f"{v:+.2f}" for v in rho[1:6]) + f" | lag 10: {rho[10]:+.2f}")
# 같은 정지 구간의 kf innovation (지금 게인) ρ₁
ev, _ = RT.load(b); base = dict(config.section("estimator", "torque_estimator.yaml")); base["mode"] = "kf"
R, _ = RT.replay(ev, base)
inseg = np.zeros(len(R), bool)
for a, e in segs:
    inseg |= (R[:, 0] >= P[a, 0] + 0.3) & (R[:, 0] <= P[e - 1, 0])
for name, m in (("정지", inseg & (R[:, 5] > 0)), ("주행", (R[:, 4] > 0.05) & (R[:, 5] > 0))):
    e = R[m]
    rho1 = lambda x: float(np.corrcoef(x[:-1], x[1:])[0, 1])
    print(f"  kf innovation ({name}, {m.sum()} 개): ν_xy RMS {np.sqrt(np.mean(e[:,1:3]**2))*1e3:.2f} mm, ρ₁ xy {0.5*(rho1(e[:,1])+rho1(e[:,2])):+.2f}, "
          f"ν_ψ RMS {math.degrees(np.sqrt(np.mean(e[:,3]**2))):.3f}°, ρ₁ yaw {rho1(e[:,3]):+.2f}")

#!/usr/bin/python3
"""Cartographer bag 재생 결과 비교 (2026-11-25). tools/slam_replay/replay.sh 의 pose.csv 들과 bag 의 실차 /tracked_pose.
흔들림 정의는 turn_events.py·yaw_by_accel.py 와 같다: SLAM 이동 − 바퀴 이동 (위치), SLAM Δψ − 자이로 적분 (방향), 0.5 s 이동중앙값 뺌.
움직임 가르기 (자이로·바퀴): 달리며 전환 = |ψ̈| ≥ 8 rad/s² & > 0.05 m/s, 제자리 전환 = |ψ̈| ≥ 8 & ≤ 0.05, 꾸준·직진 = |ψ̈| < 2 & > 0.05.
  /usr/bin/python3 replay_eval.py bag 이름=pose.csv [이름=pose.csv ...]"""
import csv, math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
OFF = np.array([0.1093, 0.017]); R_W = 0.04
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_q = lambda x, y, z, w: math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
BAG = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=["/tracked_pose", "/wheel_states", "/imu/base_link", "/mpc/armed"]))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
REC, W, G, ARM = [], [], [], []
while r.has_next():
    n, d, tb = r.read_next(); m = deserialize_message(d, ty[n])
    if n == "/tracked_pose":
        q = m.pose.orientation; REC.append((st(m.header), m.pose.position.x, m.pose.position.y, yaw_q(q.x, q.y, q.z, q.w)))
    elif n == "/wheel_states":
        W.append((st(m.header), *m.velocity))
    elif n == "/imu/base_link":
        G.append((st(m.header), m.angular_velocity.z))
    elif m.data:
        ARM.append(tb * 1e-9)
REC, W, G = map(np.array, (REC, W, G))
tw = W[:, 0]; om = W[:, 1:5]
vxw = R_W / 4 * om.sum(1); vyw = R_W / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3])
cx = np.concatenate([[0], np.cumsum(vxw[1:] * np.diff(tw))]); cy = np.concatenate([[0], np.cumsum(vyw[1:] * np.diff(tw))])
tg = G[:, 0]; fg = 1 / np.median(np.diff(tg)); b, a = butter(2, 5 / (fg / 2))
wz = filtfilt(b, a, G[:, 1]); wdot = np.gradient(wz, tg)
cg = np.concatenate([[0], np.cumsum(0.5 * (G[1:, 1] + G[:-1, 1]) * np.diff(tg))])
t_arm = ARM[0] if ARM else REC[0, 0]


def metrics(S):
    S = S[S[:, 0] >= t_arm - 1.0]                                  # 무장 1 s 전부터 (측위 맞추기 끝난 뒤)
    c, s = np.cos(S[:, 3]), np.sin(S[:, 3])
    X = S[:, 1] - (c * OFF[0] - s * OFF[1]); Y = S[:, 2] - (s * OFF[0] + c * OFF[1])
    ts = S[:, 0]; tm = 0.5 * (ts[1:] + ts[:-1]); ps = np.unwrap(S[:, 3]); ym = 0.5 * (ps[1:] + ps[:-1])
    dx, dy = np.diff(X), np.diff(Y)
    rx = (np.cos(ym) * dx + np.sin(ym) * dy) - np.diff(np.interp(ts, tw, cx))
    ry = (-np.sin(ym) * dx + np.cos(ym) * dy) - np.diff(np.interp(ts, tw, cy))
    ryaw = np.diff(ps) - np.diff(np.interp(ts, tg, cg))
    dtr = lambda x: np.array([x[i] - np.median(x[np.abs(tm - tm[i]) <= 0.25]) for i in range(len(x))])
    hp, hy = np.hypot(dtr(rx), dtr(ry)), np.abs(dtr(ryaw))
    sp = np.hypot(np.interp(tm, tw, vxw), np.interp(tm, tw, vyw))
    ya = np.array([np.abs(wdot[(tg >= t - 0.1) & (tg <= t + 0.1)]).max(initial=0.0) for t in tm])
    cls = {"달리며 전환": (ya >= 8) & (sp > 0.05), "제자리 전환": (ya >= 8) & (sp <= 0.05), "꾸준·직진": (ya < 2) & (sp > 0.05)}
    big = int((np.hypot(np.diff(X), np.diff(Y)) > 0.15).sum())
    return {k: (m.sum(), 1e3 * np.sqrt(np.mean(hp[m] ** 2)), 100 * np.mean(hp[m] > 0.02), math.degrees(np.sqrt(np.mean(hy[m] ** 2))))
            for k, m in cls.items()}, big, (ts, X, Y)


runs = [("실차 기록", REC)]
for arg in sys.argv[2:]:
    lab, path = arg.split("=", 1)
    rows = list(csv.DictReader(open(path)))
    runs.append((lab, np.array([[float(x["stamp"]), float(x["x"]), float(x["y"]),
                                 yaw_q(float(x["qx"]), float(x["qy"]), float(x["qz"]), float(x["qw"]))] for x in rows])))
print(f"bag {os.path.basename(BAG)}  (무장 뒤)\n")
print("| 판 | 달리며 전환: 위치 RMS mm / > 2 cm % / 방향 ° (간격 수) | 제자리 전환 | 꾸준·직진 | 실차 기록과 위치 차 중앙값 mm |")
print("|---|---|---|---|---|")
ref = None
for lab, S in runs:
    mt, big, (ts, X, Y) = metrics(S)
    if ref is None:
        ref = (ts, X, Y); dd = "-"
    else:
        m = (ts >= ref[0][0]) & (ts <= ref[0][-1])
        dd = f"{1e3 * np.median(np.hypot(X[m] - np.interp(ts[m], ref[0], ref[1]), Y[m] - np.interp(ts[m], ref[0], ref[2]))):.1f}"
    cell = lambda k: f"{mt[k][1]:.1f} / {mt[k][2]:.0f} / {mt[k][3]:.2f} ({mt[k][0]})"
    print(f"| {lab} | {cell('달리며 전환')} | {cell('제자리 전환')} | {cell('꾸준·직진')} | {dd} |")

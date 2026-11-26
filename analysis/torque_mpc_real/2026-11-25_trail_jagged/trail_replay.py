#!/usr/bin/python3
"""초록 선 (= 추정기 출력) 이 Cartographer 설정에 따라 얼마나 매끈해지나 (2026-11-25).
bag 의 IMU 와 SLAM 포즈를 추정기 (PlanarStateEstimator, kf, 지금 yaml) 에 노드와 같은 순서로 넣어 다시 돌린다.
SLAM 포즈는 실차 기록 또는 Cartographer 재생 판 (pose.csv, 도착 시각 = stamp + 실차 지연 중앙값).
초록 선 흔들림 = 추정기 위치를 0.5 s 이동평균에서 진행 방향 옆으로 벗어난 거리 (compare.py 와 같은 정의), 무장 뒤 움직인 구간.
  /usr/bin/python3 trail_replay.py bag 이름=pose.csv [...]"""
import csv, math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, os.path.join(ROOT, "src/amr_basic_stack/controller/mpc"))
from amr_mpc.core import config  # noqa: E402
from amr_mpc.torque.estimator import PlanarStateEstimator  # noqa: E402
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_q = lambda x, y, z, w: math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
BAG = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=["/imu/base_link", "/tracked_pose", "/state_estimator/odom", "/mpc/armed"]))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
IMU, POSE, OD, ARM = [], [], [], []
while r.has_next():
    n, d, trx = r.read_next(); m = deserialize_message(d, ty[n])
    if n == "/imu/base_link":
        IMU.append((trx * 1e-9, 0, st(m.header), m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z))
    elif n == "/tracked_pose":
        q = m.pose.orientation; POSE.append((trx * 1e-9, 1, st(m.header), m.pose.position.x, m.pose.position.y, yaw_q(q.x, q.y, q.z, q.w)))
    elif n == "/state_estimator/odom":
        p = m.pose.pose; OD.append((st(m.header), p.position.x, p.position.y))
    elif m.data:
        ARM.append(trx * 1e-9)
lat = float(np.median([e[0] - e[2] for e in POSE]))
t_arm = ARM[0] if ARM else POSE[0][0]
cfg = dict(config.section("estimator", "torque_estimator.yaml")); cfg["mode"] = "kf"


def run(poses):
    ev = sorted(IMU + poses, key=lambda e: (e[0], e[1]))
    est = PlanarStateEstimator(cfg, pose_period=0.1); out = []; last = -1e9
    for _, kind, t, a, b, c in ev:
        if kind == 0:
            est.imu(t, a, b, c)
            if est.t is not None and getattr(est, "pose_ready", True):
                s = est.state(); out.append(s[:3] + (math.hypot(s[4], s[5]),))
        elif t > last:
            last = t; est.pose(t, a, b, c)
    return np.array(out)


def jag(A):
    A = A[(A[:, 0] >= t_arm) & (A[:, 3] > 0.1)] if A.shape[1] > 3 else A
    t, P = A[:, 0], A[:, 1:3]
    out = []
    for j in range(0, len(t), 5):                                   # 20 Hz 로 솎아 계산
        m = np.abs(t - t[j]) <= 0.25
        a, b = np.where(m)[0][[0, -1]]; u = P[b] - P[a]; L = np.linalg.norm(u)
        if L < 0.05: continue
        u /= L; rel = P[j] - P[m].mean(0); out.append(u[0] * rel[1] - u[1] * rel[0])
    out = np.abs(np.array(out))
    return f"{1e3 * np.sqrt(np.mean(out ** 2)):.1f} / {1e3 * np.percentile(out, 95):.1f} / {1e3 * out.max():.1f}"


rows = [("실차 /state_estimator/odom", None), ("다시 돌림 ← 실차 SLAM", POSE)]
for arg in sys.argv[2:]:
    lab, path = arg.split("=", 1)
    P = []
    for x in csv.DictReader(open(path)):
        ts = float(x["stamp"]); P.append((ts + lat, 1, ts, float(x["x"]), float(x["y"]), yaw_q(float(x["qx"]), float(x["qy"]), float(x["qz"]), float(x["qw"]))))
    rows.append((f"다시 돌림 ← {lab}", P))
print(f"bag {os.path.basename(BAG)} (SLAM 도착 지연 {1e3 * lat:.0f} ms)\n\n| 추정기 입력 | 초록 선 흔들림 RMS / 95 % / 최대 mm |\n|---|---|")
sp_od = None
for lab, P in rows:
    if P is None:
        A = np.array(OD); vv = run(POSE)                             # 속도 걸러내기는 다시 돌린 추정 속도로
        A = np.c_[A, np.interp(A[:, 0], vv[:, 0], vv[:, 3])]
    else:
        A = run(P)
    print(f"| {lab} | {jag(A)} |")

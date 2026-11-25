#!/usr/bin/python3
"""추정기 몸체 v_x 의 지연 (IMU 400 Hz 뒤 다시 재기, 2026-11-22). 추종 창 (/torque_mpc/status hold 0) 에서 10 ms 격자, 평균 뺀 뒤 상호상관.
   기준: 바퀴 기구학 v_x (r/4 Σω — 슬립 때 바퀴가 앞설 수 있음), SLAM 중앙 차분 v_x (앞뒤 포즈, 위상 0, 10 Hz 라 매끈해짐)."""
import math, sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
R_IMU = np.array([0.1093, 0.017])
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
for b in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/wheel_states", "/state_estimator/odom", "/tracked_pose", "/torque_mpc/status", "/imu/base_link"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while r.has_next():
        n, d, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
    S = np.array([[t, m.data[20] if len(m.data) > 20 else 0] for t, m in D["/torque_mpc/status"]]); act = S[S[:, 1] == 0, 0]
    E = np.array([[st(m.header), m.twist.twist.linear.x] for _, m in D["/state_estimator/odom"]])
    W = np.array([[st(m.header), 0.01 * sum(m.velocity)] for _, m in D["/wheel_states"]])
    P = np.array(sorted({(st(m.header), m.pose.position.x, m.pose.position.y, 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)) for _, m in D["/tracked_pose"]}))
    c, s = np.cos(P[:, 3]), np.sin(P[:, 3])
    C = P[:, 1:3] - np.stack([c * R_IMU[0] - s * R_IMU[1], s * R_IMU[0] + c * R_IMU[1]], 1)
    vw = (C[2:] - C[:-2]) / (P[2:, 0] - P[:-2, 0])[:, None]
    vsx = np.cos(P[1:-1, 3]) * vw[:, 0] + np.sin(P[1:-1, 3]) * vw[:, 1]
    imu_hz = len(D["/imu/base_link"]) / (D["/imu/base_link"][-1][0] - D["/imu/base_link"][0][0])
    tg = np.arange(act[0], act[-1], 0.01)
    e = np.interp(tg, E[:, 0], E[:, 1]); w = np.interp(tg, W[:, 0], W[:, 1]); sl = np.interp(tg, P[1:-1, 0], vsx)
    def lag(ref, sig):          # sig 가 ref 보다 늦은 시간 (+ = 늦음)
        a, bb = ref - ref.mean(), sig - sig.mean(); L = np.arange(-20, 31)
        cc = [np.dot(a[max(0, -l):len(a) - max(0, l)], bb[max(0, l):len(bb) - max(0, -l)]) for l in L]
        k = int(np.argmax(cc)); return 10 * L[k], cc[k] / math.sqrt(np.dot(a, a) * np.dot(bb, bb))
    le, re_ = lag(w, e); ls, rs = lag(sl, e); lw, rw = lag(sl, w)
    print(f"{b} (IMU {imu_hz:.0f} Hz, 창 {tg[-1]-tg[0]:.1f} s, |v_x| 최대 {np.abs(w).max():.2f} m/s): 추정 vs 바퀴 {le:+d} ms (r {re_:.2f}) | "
          f"추정 vs SLAM 차분 {ls:+d} ms (r {rs:.2f}) | 바퀴 vs SLAM 차분 {lw:+d} ms (r {rw:.2f})")

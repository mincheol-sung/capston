#!/usr/bin/python3
"""C 가 왜 못 따라가나 — 실차 bag 의 /torque_mpc/status ([t, e_x, e_y, e_θ(추정기 기준), τ×4, τ̄max, active, iters, fallback, ms, ref_age,
d̂×6, hold, stopped, U_s×4]) + /state_estimator/odom vs /tracked_pose (로봇 중심) + 기준열.
  포화 = max_i |τ_i| / τ̄_i ≥ 0.98,   앞먹임 초과 = max_i |U_s,i| / τ̄_i > 1 (기준을 따라가는 데 필요한 토크가 상한 밖)
  오차 둘: MPC 가 보는 오차 (추정기 − 기준, status) 와 SLAM 이 보는 오차 (SLAM 중심 − 기준). 둘의 차 ≈ 추정기 − SLAM."""
import importlib.util, json, math, os, sys
import numpy as np, yaml
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]; off = (est["imu_offset_x"], est["imu_offset_y"])
TB = {"new": np.array([0.1093, 0.1113, 0.1154, 0.1055]), "old": np.array([0.1093, 0.0721, 0.1154, 0.0886])}


def load(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    want = ["/torque_mpc/status", "/state_estimator/odom", "/tracked_pose", "/mpc/armed"]
    rd.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    S, E, P, A = [], [], [], []
    while rd.has_next():
        tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp]))
        if tp == "/torque_mpc/status": S.append(list(m.data))
        elif tp == "/state_estimator/odom": E.append([st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)])
        elif tp == "/tracked_pose": P.append([st(m.header), m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)])
        elif tp == "/mpc/armed": A.append((tb * 1e-9, bool(m.data)))
    return np.array([s for s in S if len(s) == 26]), np.array(E), np.array(P), A


def analyse(bag, which):
    S, E, P, A = load(f"{WS}/bags/2026-11-20/{bag}")
    t = S[:, 0]; tau = S[:, 4:8]; us = S[:, 22:26]; ex = np.hypot(S[:, 1], S[:, 2]); hold = S[:, 20] > 0.5
    run = cs.armed_mask(t, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
    tb = TB[which]
    sat = (np.abs(tau) / tb).max(1) >= 0.98; ff = (np.abs(us) / tb).max(1) > 1.0
    # 추정기 − SLAM (같은 시각, SLAM 중심)
    xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], off)
    ex_s = np.interp(P[:, 0], E[:, 0], E[:, 1]) - xc; ey_s = np.interp(P[:, 0], E[:, 0], E[:, 2]) - yc
    ok = (P[:, 0] > E[0, 0]) & (P[:, 0] < E[-1, 0]) & cs.armed_mask(P[:, 0], A)
    tr = t[run]; tr0, tr1 = tr[0], tr[-1]
    okd = ok & (P[:, 0] >= tr0) & (P[:, 0] <= tr1)
    d_es = np.hypot(ex_s[okd], ey_s[okd])
    c2 = cs.analyse(f"{WS}/bags/2026-11-20/{bag}", off, 0.04, 0.1932)
    return dict(bag=bag, which=which, sat=100 * sat[run].mean(), ff=100 * ff[run].mean(),
                e_mpc=1e3 * float(np.sqrt(np.mean(ex[run] ** 2))), e_slam=1e3 * c2["rms"],
                est_slam_rms=1e3 * float(np.sqrt(np.mean(d_es ** 2))), est_slam_mean=(1e3 * float(np.mean(ex_s[okd])), 1e3 * float(np.mean(ey_s[okd]))),
                t=(tr - tr0).tolist(), sat_t=sat[run].astype(int).tolist(), ff_t=ff[run].astype(int).tolist(),
                ratio=(np.abs(tau[run]) / tb).max(1).tolist(), ff_ratio=(np.abs(us[run]) / tb).max(1).tolist(), e_t=ex[run].tolist(),
                es_t=(P[okd, 0] - tr0).tolist(), es=d_es.tolist())


if __name__ == "__main__":
    out = [analyse(b, w) for b, w in [("171531", "old"), ("171735", "old"), ("190506", "new"), ("190642", "new")]]
    json.dump(out, open(os.path.join(HERE, "why_c.json"), "w"))
    print("| 판 | μ̂ | 토크 상한 붙은 시간 | 앞먹임 U_s 가 상한 넘는 시간 | MPC 가 보는 오차 RMS (추정기) | SLAM 이 보는 오차 RMS | 추정기 − SLAM 위치 RMS (평균 x, y) |")
    for o in out:
        print(f"| {o['bag']} | {o['which']} | {o['sat']:.0f} % | {o['ff']:.0f} % | {o['e_mpc']:.0f} mm | {o['e_slam']:.0f} mm | {o['est_slam_rms']:.0f} mm ({o['est_slam_mean'][0]:+.0f}, {o['est_slam_mean'][1]:+.0f}) |")

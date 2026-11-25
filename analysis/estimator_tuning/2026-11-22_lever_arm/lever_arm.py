#!/usr/bin/python3
"""SLAM 추적점(TF camera_gyro_frame = torque_estimator.yaml imu_offset)이 바퀴 운동학 중심에서 실제로 어디 있나 (2026-11-22).

추정기의 로봇 중심 속도 v_c = v_imu − ω × r_yaml. 실제 r = r_yaml + δ 면 v_c 에 ω × δ = (−ω δ_y, ω δ_x) 가 섞인다.
바퀴 기구학 v_w = (r/4)Σ(±ω_i) 는 운동학 중심의 속도 (슬립 없을 때). 제자리 회전 (|v| 작고 |ω| > 0.5 rad/s) 에서
  Δv_x = v_c,x − v_w,x = −δ_y ω + b_x,   Δv_y = δ_x ω + b_y   를 최소제곱으로.
고르기에 따른 치우침을 보려고 두 가지로 고른다: (A) 바퀴 |v_w| < 0.15, (B) 추정 |v_c| < 0.15. 바퀴 parity |c| > 2 rad/s (슬립) 는 뺀다.
11-19 새 하부 뒤 bag 만.  nice -n 19 ionice -c3 /usr/bin/python3 analysis/estimator_tuning/2026-11-22_lever_arm/lever_arm.py
"""
import glob, os, sys
import numpy as np, rosbag2_py, yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

R_YAML = (0.1093, 0.017)
rows = []
for meta in sorted(glob.glob("bags/2026-11-19/*/metadata.yaml") + glob.glob("bags/2026-11-2[0-2]/*/metadata.yaml")):
    d = os.path.dirname(meta)
    try:
        m = yaml.safe_load(open(meta))["rosbag2_bagfile_information"]
    except Exception:
        continue
    c = {t["topic_metadata"]["name"]: t["message_count"] for t in m["topics_with_message_count"]}
    if c.get("/state_estimator/odom", 0) < 500 or c.get("/wheel_states", 0) < 500:
        continue
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=d, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/wheel_states", "/state_estimator/odom"]))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}; W, E = [], []
    while r.has_next():
        n, dd, _ = r.read_next(); mm = deserialize_message(dd, ty[n]); t = mm.header.stamp.sec + mm.header.stamp.nanosec * 1e-9
        if n == "/wheel_states":
            W.append((t, *mm.velocity))
        else:
            E.append((t, mm.twist.twist.linear.x, mm.twist.twist.linear.y, mm.twist.twist.angular.z))
    W, E = np.array(W), np.array(E)
    if len(E) < 100:
        continue
    w = W[:, 1:5]
    vw = np.stack([0.01 * w.sum(1), 0.01 * (-w[:, 0] + w[:, 1] + w[:, 2] - w[:, 3])], 1)     # r/4 = 0.01
    par = w[:, 0] + w[:, 1] - w[:, 2] - w[:, 3]
    ok = (W[:, 0] > E[0, 0]) & (W[:, 0] < E[-1, 0])
    ve = np.stack([np.interp(W[:, 0], E[:, 0], E[:, i]) for i in (1, 2, 3)], 1)
    for t_, a, b, cc, e0, e1, wz, okk in zip(W[:, 0], vw[:, 0], vw[:, 1], par, ve[:, 0], ve[:, 1], ve[:, 2], ok):
        if okk and abs(wz) > 0.5 and abs(cc) <= 2.0:
            rows.append((os.path.basename(os.path.dirname(d)) + "/" + os.path.basename(d), t_, a, b, e0, e1, wz))
X = np.array([r_[1:] for r_ in rows]); names = np.array([r_[0] for r_ in rows])
print(f"|ω| > 0.5, parity ≤ 2 인 표본 {len(X)} 개 (bag {len(set(names))} 개)")


def fit(sel, lab):
    t, vwx, vwy, vex, vey, wz = X[sel].T
    A = np.stack([wz, np.ones_like(wz)], 1)
    kx, rx, *_ = np.linalg.lstsq(A, vex - vwx, rcond=None); ky, ry, *_ = np.linalg.lstsq(A, vey - vwy, rcond=None)
    dy, dx = -kx[0], ky[0]
    # bag 별 추정 (1 s = 100 표본 이상) — 표본끼리 상관이 커서 불확실성은 bag 사이 퍼짐으로 본다
    per = []
    for nm in sorted(set(names[sel])):
        s2 = sel & (names == nm)
        if s2.sum() >= 100:
            t2, a2, b2, c2, d2, w2 = X[s2].T
            A2 = np.stack([w2, np.ones_like(w2)], 1)
            per.append((nm, int(s2.sum()), np.linalg.lstsq(A2, d2 - b2, rcond=None)[0][0], -np.linalg.lstsq(A2, c2 - a2, rcond=None)[0][0]))
    P = np.array([[p[2], p[3]] for p in per]) if per else np.zeros((0, 2))
    print(f"\n({lab}) 표본 {sel.sum()} 개 ({sel.sum()/100:.1f} s), |ω| RMS {np.sqrt(np.mean(wz**2)):.2f} rad/s")
    print(f"   합쳐서: δ_x {dx*1e3:+.1f} mm, δ_y {dy*1e3:+.1f} mm → 실제 r = ({(R_YAML[0]+dx):.4f}, {(R_YAML[1]+dy):.4f}) m   "
          f"(절편 b_x {kx[1]*1e3:+.1f}, b_y {ky[1]*1e3:+.1f} mm/s)")
    if len(P):
        print(f"   bag 별 (1 s 이상, {len(P)} 개): δ_x 중앙값 {np.median(P[:,0])*1e3:+.1f} mm [사분위 {np.percentile(P[:,0],25)*1e3:+.1f}~{np.percentile(P[:,0],75)*1e3:+.1f}], "
              f"δ_y 중앙값 {np.median(P[:,1])*1e3:+.1f} mm [{np.percentile(P[:,1],25)*1e3:+.1f}~{np.percentile(P[:,1],75)*1e3:+.1f}]")
        for nm, n, a, b in per:
            print(f"      {nm}: {n/100:.1f} s  δ_x {a*1e3:+.1f}  δ_y {b*1e3:+.1f} mm")
    # 잔차: δ 보정 전·후 Δv RMS
    res0 = np.hypot(vex - vwx, vey - vwy)
    res1 = np.hypot(vex - vwx + dy * wz - kx[1], vey - vwy - dx * wz - ky[1])
    print(f"   Δv RMS: 지금 r {np.sqrt(np.mean(res0**2))*1e3:.1f} mm/s → δ 보정 뒤 {np.sqrt(np.mean(res1**2))*1e3:.1f} mm/s")


vw_sp = np.hypot(X[:, 1], X[:, 2]); ve_sp = np.hypot(X[:, 3], X[:, 4])
fit(vw_sp < 0.15, "A: 바퀴 |v| < 0.15 m/s")
fit(ve_sp < 0.15, "B: 추정 |v| < 0.15 m/s")
fit((vw_sp < 0.05) & (ve_sp < 0.05) | (vw_sp < 0.15) & (ve_sp < 0.15), "A∩B: 둘 다 < 0.15")

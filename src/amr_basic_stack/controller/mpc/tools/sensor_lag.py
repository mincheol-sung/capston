#!/usr/bin/python3
"""센서·추정기 신호가 얼마나 늦나 — 바퀴 엔코더(CAN, 거의 지연 없음)를 기준으로 상호상관 (2026-11-19).

2026-11-18 bag: 자이로 +15 ms, 가속도계 a_x +55 ms, 추정기 v_x +90~95 ms, SLAM 포즈 −50~+50 ms (바퀴 기준).
D455 IMU 설정(amrall imu_accel_fps:= imu_gyro_fps:= imu_unite:=)을 바꾼 판과 비교하려고 만든 도구.

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 tools/sensor_lag.py <bag 폴더> [...]        (amrcmp 처럼 amrlag)

비교 (무장 뒤 0.5 ~ 20 s, 10 ms 격자, 지연 + = 바퀴보다 늦음, ±0.4 s 탐색)
  자이로 ω_z          vs 바퀴 기구학 ω_z
  가속도계 a_x (IMU 점) vs 바퀴 v_x 의 미분 (양쪽 비슷한 중앙 이동평균 — 평활이 지연을 만들지 않게 가운데 정렬)
  추정기 v_x, ω_z     vs 바퀴 v_x, ω_z
  SLAM v_x            vs 바퀴 v_x (/tracked_pose 를 IMU 점 → 중심으로 옮겨 ±0.05 s 중앙 차분, 몸체 축)
※ 바퀴 v_x 는 슬립이 있으면 몸체 속도와 다르지만 변화 시각은 맞다 — 지연(시각)만 본다. 상관계수 r 이 낮으면 그 줄은 믿지 말 것.
"""
import argparse
import math
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "config")


def xcorr_lag(t, ref, sig_t, sig, maxlag=0.4, step=0.005):
    """ref(t) 와 sig(t + L) 의 상관이 가장 큰 L (L > 0 이면 sig 가 늦다). 반환 (L, r)."""
    best = (0.0, -2.0)
    for L in np.arange(-maxlag, maxlag + 1e-9, step):
        s = np.interp(t + L, sig_t, sig)
        if np.std(s) < 1e-12 or np.std(ref) < 1e-12:
            continue
        c = float(np.corrcoef(ref, s)[0, 1])
        if c > best[1]:
            best = (float(L), c)
    return best


def centered_mean(x, n):
    return np.convolve(x, np.ones(n) / n, mode="same")


def load(bag, r_w, k):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = ["/imu/base_link", "/wheel_states", "/state_estimator/odom", "/tracked_pose", "/mpc/armed"]
    r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
    D = {w: [] for w in want}
    q = r_w / 4.0
    while r.has_next():
        tp, data, tb = r.read_next()
        m = deserialize_message(data, get_message(types[tp]))
        if tp == "/imu/base_link":
            D[tp].append((st(m.header), m.linear_acceleration.x, m.angular_velocity.z))
        elif tp == "/wheel_states":
            w = list(m.velocity)
            D[tp].append((st(m.header), q * sum(w), q / k * (-w[0] + w[1] - w[2] + w[3])))
        elif tp == "/state_estimator/odom":
            D[tp].append((st(m.header), m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif tp == "/tracked_pose":
            D[tp].append((st(m.header), m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))
        else:
            D[tp].append((tb * 1e-9, bool(m.data)))
    return {a: np.array(b) for a, b in D.items()}


def analyse(bag, r_w, k, off):
    D = load(bag, r_w, k)
    arm = D["/mpc/armed"]
    if arm.size == 0 or not (arm[:, 1] > 0).any() or D["/wheel_states"].size == 0:
        raise RuntimeError("무장 구간이나 /wheel_states 가 없다")
    ta = arm[arm[:, 1] > 0][0, 0]
    wh = D["/wheel_states"]
    t = np.arange(ta + 0.5, min(ta + 20.0, wh[-1, 0] - 0.5), 0.01)
    wvx, wwz = np.interp(t, wh[:, 0], wh[:, 1]), np.interp(t, wh[:, 0], wh[:, 2])
    out = []
    imu = D["/imu/base_link"]
    if imu.size:
        out.append(("자이로 ω_z", xcorr_lag(t, wwz, imu[:, 0], imu[:, 2])))
        wax = np.gradient(centered_mean(wvx, 5), 0.01)
        out.append(("가속도계 a_x", xcorr_lag(t, wax, imu[:, 0], centered_mean(imu[:, 1], 9))))
    od = D["/state_estimator/odom"]
    if od.size:
        out.append(("추정기 v_x", xcorr_lag(t, wvx, od[:, 0], od[:, 1])))
        out.append(("추정기 ω_z", xcorr_lag(t, wwz, od[:, 0], od[:, 2])))
    tp = D["/tracked_pose"]
    if tp.size > 20:
        ts, ps = tp[:, 0], tp[:, 3]
        c, s = np.cos(ps), np.sin(ps)
        xc, yc = tp[:, 1] - (c * off[0] - s * off[1]), tp[:, 2] - (s * off[0] + c * off[1])
        d = lambda a: (np.interp(ts + 0.05, ts, a) - np.interp(ts - 0.05, ts, a)) / 0.1
        out.append(("SLAM v_x", xcorr_lag(t, wvx, ts, c * d(xc) + s * d(yc))))
    rate = len(imu) / (imu[-1, 0] - imu[0, 0]) if imu.size else float("nan")
    return out, rate


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bags", nargs="+")
    a = ap.parse_args()
    plant = yaml.safe_load(open(os.path.join(CFG, "torque_plant.yaml")))
    est = yaml.safe_load(open(os.path.join(CFG, "torque_estimator.yaml")))["estimator"]
    geo = plant["geometry"]
    r_w, k = geo["wheel_radius_m"], 0.5 * (geo["wheel_base_m"] + geo["robot_width_m"])
    off = (float(est.get("imu_offset_x", 0.0)), float(est.get("imu_offset_y", 0.0)))
    for bag in a.bags:
        bag = bag.rstrip("/")
        args = open(os.path.join(bag, "run_info", "args.txt")).read() if os.path.isfile(os.path.join(bag, "run_info", "args.txt")) else ""
        imu_args = " ".join(w for w in args.split() if w.startswith("imu_"))
        try:
            res, rate = analyse(bag, r_w, k, off)
        except Exception as ex:
            print(f"[{bag}] 건너뜀: {ex}")
            continue
        print(f"[{bag}]  IMU 메시지 {rate:.0f} Hz  {imu_args or '(imu_ 인자 없음 = 장치 기본, linear_interpolation)'}")
        for name, (L, c) in res:
            print(f"   {name:12s} {1e3 * L:+5.0f} ms  (r {c:.2f}){'   ← r 낮음, 믿지 말 것' if c < 0.6 else ''}")


if __name__ == "__main__":
    main()

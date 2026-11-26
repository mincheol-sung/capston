#!/usr/bin/python3
"""누가 흔들리나: SLAM · 바퀴 · IMU 세 속도의 1~4 Hz 성분을 서로 빼서 각자 잡음을 가른다 (three-cornered hat, 2026-11-25).
  SLAM v_S = SLAM 포즈 차분 (몸체 좌표), 바퀴 v_W = T ω 구간 평균, IMU v_I = ∫(a − ω×v) dt  (v 는 바퀴 값, 몸체 좌표 미분식)
  10 Hz (SLAM 간격) 로 맞춘 뒤 0.5~4 Hz 대역 → var(A−B) = σA² + σB² 세 식에서 σ 셋.
  같이: 차체 흔들림 — IMU 롤·피치 각속도 1~20 Hz RMS, 상하 가속도 1~20 Hz RMS (움직인 구간)
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_trail_jagged/three_hat.py bag [bag ...]"""
import math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
OFF = np.array([0.1093, 0.017]); R_W = 0.04
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
print("| bag | 움직인 SLAM 간격 | σ SLAM 앞/옆 mm/s | σ 바퀴 앞/옆 | σ IMU 앞/옆 | 롤 각속도 1~20 Hz RMS °/s | 피치 °/s | 상하 가속 m/s² |")
print("|---|---|---|---|---|---|---|---|")
for BAG in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/tracked_pose", "/wheel_states", "/imu/base_link"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    S, W, I = [], [], []
    while r.has_next():
        n, d, _ = r.read_next(); m = deserialize_message(d, ty[n])
        if n == "/tracked_pose":
            p = m.pose; ps = yaw_of(p.orientation); c, s = math.cos(ps), math.sin(ps)
            S.append((st(m.header), p.position.x - (c * OFF[0] - s * OFF[1]), p.position.y - (s * OFF[0] + c * OFF[1]), ps))
        elif n == "/wheel_states":
            W.append((st(m.header), *m.velocity))
        else:
            a, w = m.linear_acceleration, m.angular_velocity
            I.append((st(m.header), a.x, a.y, a.z, w.x, w.y, w.z))
    if min(len(S), len(W), len(I)) < 50:
        print(f"| {os.path.basename(BAG)} | (토픽 부족) |"); continue
    S, W, I = map(np.array, (S, W, I))
    tw = W[:, 0]; om = W[:, 1:5]
    vxw = R_W / 4 * om.sum(1); vyw = R_W / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3])
    ti = I[:, 0]; wz = I[:, 6]
    vxi_w, vyi_w = np.interp(ti, tw, vxw), np.interp(ti, tw, vyw)
    dvx = I[:, 1] + wz * vyi_w; dvy = I[:, 2] - wz * vxi_w             # 몸체 좌표 v̇ = a − ω × v
    dti = np.diff(ti, prepend=ti[0])
    cix, ciy = np.cumsum(dvx * dti), np.cumsum(dvy * dti)              # IMU 속도 (드리프트는 대역 필터로 지운다)
    ts = S[:, 0]; tm = 0.5 * (ts[1:] + ts[:-1]); dt = np.diff(ts)
    ps = np.unwrap(S[:, 3]); ym = 0.5 * (ps[1:] + ps[:-1]); dx, dy = np.diff(S[:, 1]), np.diff(S[:, 2])
    vS = np.c_[(np.cos(ym) * dx + np.sin(ym) * dy) / dt, (-np.sin(ym) * dx + np.cos(ym) * dy) / dt]
    cwx = np.concatenate([[0], np.cumsum(vxw[1:] * np.diff(tw))]); cwy = np.concatenate([[0], np.cumsum(vyw[1:] * np.diff(tw))])
    vW = np.c_[np.diff(np.interp(ts, tw, cwx)) / dt, np.diff(np.interp(ts, tw, cwy)) / dt]
    vI = np.c_[np.interp(tm, ti, cix), np.interp(tm, ti, ciy)]
    mov = np.abs(np.interp(tm, tw, vxw)) + np.abs(np.interp(tm, tw, vyw)) > 0.1
    if mov.sum() < 20:
        print(f"| {os.path.basename(BAG)} | {int(mov.sum())} | (거의 안 움직임) |"); continue
    i0, i1 = np.where(mov)[0][[0, -1]]
    seg = slice(max(i0 - 10, 0), min(i1 + 10, len(tm)))                # 움직인 구간 (앞뒤 1 s 여유)
    b, a = butter(2, [0.5 / 5, 4 / 5], btype="band")
    F = lambda x: filtfilt(b, a, x[seg], axis=0)
    fS, fW, fI = F(vS), F(vW), F(vI)
    sig = []
    for k in range(2):
        AB, AC, BC = np.var(fS[:, k] - fW[:, k]), np.var(fS[:, k] - fI[:, k]), np.var(fW[:, k] - fI[:, k])
        sig.append([math.sqrt(max(v, 0)) for v in ((AB + AC - BC) / 2, (AB + BC - AC) / 2, (AC + BC - AB) / 2)])
    # 차체 흔들림 (움직인 구간의 IMU)
    fi = 1 / np.median(np.diff(ti)); bb, aa = butter(2, [1 / (fi / 2), 20 / (fi / 2)], btype="band")
    mi = (ti >= tm[i0]) & (ti <= tm[i1])
    rr = lambda x: float(np.sqrt(np.mean(filtfilt(bb, aa, x)[mi] ** 2)))
    print(f"| {os.path.basename(BAG)} | {int(mov.sum())} | {1e3 * sig[0][0]:.0f} / {1e3 * sig[1][0]:.0f} | {1e3 * sig[0][1]:.0f} / {1e3 * sig[1][1]:.0f} | "
          f"{1e3 * sig[0][2]:.0f} / {1e3 * sig[1][2]:.0f} | {math.degrees(rr(I[:, 4])):.1f} | {math.degrees(rr(I[:, 5])):.1f} | {rr(I[:, 3]):.2f} |")

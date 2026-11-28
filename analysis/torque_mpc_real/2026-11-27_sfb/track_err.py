"""추종 오차: 기준 지평 (/dra_mppi/reference_trajectory, PlanarReferenceArray) 을 펼쳐 dense 기준을 만들고
추정기 (/state_estimator/odom) 와 header 시각으로 맞춰 위치·속도·헤딩·요 속도 오차를 낸다.
  위치 = |p_r − p| (map),  헤딩 = wrap(ψ_r − ψ),  속도 = |R(ψ_r) u_r,xy − R(ψ) v_xy| (월드에서 벡터 차),  요 속도 = ω_r − ω
  창 = 무장 중 · 기준 속력 > 0.05 m/s (기준이 움직이는 동안). 상태 피드백 오차 (status) 와 위치·헤딩을 검산.
source install/setup.bash 뒤  /usr/bin/python3 track_err.py <bag 폴더 ...>"""
import math
import os
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WANT = ("/dra_mppi/reference_trajectory", "/state_estimator/odom", "/mpc/armed", "/torque_mpc/status", "/pure_pursuit/status")


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


for path in sys.argv[1:]:
    path = path.rstrip("/")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in WANT}
    ref, od, arm, sts = {}, [], [], []
    t_first = None
    while r.has_next():
        name, data, ns = r.read_next()
        if name not in types:
            continue
        tb = ns * 1e-9
        t_first = t_first if t_first is not None else tb
        m = deserialize_message(data, types[name])
        if name == "/dra_mppi/reference_trajectory":
            t0 = stamp(m.header)
            for j, p in enumerate(m.references):
                if j * m.dt > 0.15:                        # 다음 메시지 (≈ 0.1 s) 까지만 — 뒤 메시지가 같은 시각을 덮어쓴다
                    break
                ref[round((t0 + j * m.dt) * 1000)] = (p.x, p.y, p.yaw, p.vx, p.vy, p.yaw_rate)
        elif name == "/state_estimator/odom":
            q, p, tw = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            od.append([stamp(m.header), p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z])
        elif name == "/mpc/armed":
            arm.append([tb, float(m.data)])
        else:
            sts.append([tb, *m.data[:4]])                  # [bag t, now, ex, ey, eθ]
    od = np.array(od); arm = np.array(arm); sts = np.array(sts)
    kt = np.array(sorted(ref)); R = np.array([ref[k] for k in kt]); tr = kt / 1000.0
    t_on = arm[arm[:, 1] > 0.5, 0][0]
    # 무장 시각은 bag 시각 — header 시각과의 차는 수 ms 라 그대로 쓴다
    t = od[:, 0]
    m = (t >= tr[0]) & (t <= tr[-1]) & (t >= t_on - 0.05)
    t, X = t[m], od[m]
    Rr = np.column_stack([np.interp(t, tr, R[:, i]) for i in (0, 1, 3, 4, 5)])
    yr = np.interp(t, tr, np.unwrap(R[:, 2]))
    vref = np.hypot(Rr[:, 2], Rr[:, 3])
    win = vref > 0.05
    ep = np.hypot(Rr[:, 0] - X[:, 1], Rr[:, 1] - X[:, 2])
    eh = np.degrees(wrap(yr - X[:, 3]))
    cr, sr, c, s = np.cos(yr), np.sin(yr), np.cos(X[:, 3]), np.sin(X[:, 3])
    vrw = np.column_stack([cr * Rr[:, 2] - sr * Rr[:, 3], sr * Rr[:, 2] + cr * Rr[:, 3]])
    vw = np.column_stack([c * X[:, 4] - s * X[:, 5], s * X[:, 4] + c * X[:, 5]])
    ev = np.hypot(*(vrw - vw).T)
    espd = np.hypot(*vw.T) - vref                                   # 속력 차 (음 = 느림)
    ew = Rr[:, 4] - X[:, 6]
    rms = lambda a: float(np.sqrt(np.mean(a[win] ** 2)))
    name = os.path.basename(path)
    print(f"── {name}: 기준 움직임 {win.sum() * np.median(np.diff(t)):.1f} s, 기준 속력 {vref[win].min():.2f}~{vref[win].max():.2f} m/s")
    print(f"   위치 RMS {100 * rms(ep):5.2f} cm (최대 {100 * ep[win].max():5.2f}) · 헤딩 RMS {rms(eh):4.2f}° (최대 {np.abs(eh[win]).max():5.2f}°)")
    print(f"   속도 벡터 RMS {100 * rms(ev):5.2f} cm/s (최대 {100 * ev[win].max():5.1f}) · 속력 차 RMS {100 * rms(espd):5.2f} cm/s (평균 {100 * espd[win].mean():+5.2f})"
          f" · 요 속도 RMS {rms(ew):.3f} rad/s (최대 {np.abs(ew[win]).max():.2f})")
    if len(sts):                                                    # 검산: 제어기 status 오차 (기준 − 상태, map) 와 위치·헤딩 비교
        ts_ = sts[:, 0] - (sts[0, 0] - sts[0, 1]) * 0                  # bag 시각
        ms = (sts[:, 0] >= t_on) & (sts[:, 0] <= t[win][-1] + (sts[0, 0] - sts[0, 1]))
        print(f"   검산 (제어기 status, 무장 뒤): 위치 RMS {100 * np.sqrt(np.mean(sts[ms, 2] ** 2 + sts[ms, 3] ** 2)):5.2f} cm · "
              f"헤딩 RMS {np.degrees(np.sqrt(np.mean(wrap(sts[ms, 4]) ** 2))):4.2f}°")
    np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)), name + "_trk.npz"), t=t, ep=ep, eh=eh, ev=ev, espd=espd, ew=ew,
             vref=vref, win=win, wz=X[:, 6], t_on=t_on)

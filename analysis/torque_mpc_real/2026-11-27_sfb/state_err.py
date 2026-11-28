"""MPC 상태 6 개별 부호 있는 추종 오차 (기준 − 추정기), map 좌표 — MPC 비용 Q 가 보는 것과 같은 정의.
  e = [x_r − x, y_r − y, wrap(θ_r − θ), ẋ_r − ẋ, ẏ_r − ẏ, θ̇_r − θ̇]   (ẋ, ẏ = 월드 속도: R(θ) v_body)
  덧붙여 진행·옆 방향 분해 (기준 속도 방향 t̂, 왼쪽 법선 n̂): 위치 진행 오차 = e_p·t̂ (+ = 로봇이 뒤처짐), 옆 = e_p·n̂ (+ = 로봇이 기준 오른쪽)
  창 = 무장 중 · 기준 속력 > 0.05 m/s. 기준 지평 점 j 의 시각 = header + (j+1)·dt (track_err.py 와 같음).
source install/setup.bash 뒤  /usr/bin/python3 state_err.py <bag ...>   → <이름>_se.npz"""
import math
import os
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

REF_SHIFT = +0.05
WANT = ("/dra_mppi/reference_trajectory", "/state_estimator/odom", "/mpc/armed")


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


for path in sys.argv[1:]:
    path = path.rstrip("/")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in WANT}
    ref, od, arm = {}, [], []
    while r.has_next():
        name, data, ns = r.read_next()
        if name not in types:
            continue
        m = deserialize_message(data, types[name])
        if name == "/dra_mppi/reference_trajectory":
            t0 = stamp(m.header)
            for j, p in enumerate(m.references):
                if j * m.dt > 0.15:
                    break
                ref[round((t0 + j * m.dt + REF_SHIFT) * 1000)] = (p.x, p.y, p.yaw, p.vx, p.vy, p.yaw_rate)
        elif name == "/state_estimator/odom":
            q, p, tw = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            od.append([stamp(m.header), p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z])
        else:
            arm.append([ns * 1e-9, float(m.data)])
    od = np.array(od); arm = np.array(arm)
    kt = np.array(sorted(ref)); R = np.array([ref[k] for k in kt]); tr = kt / 1000.0
    t_on = arm[arm[:, 1] > 0.5, 0][0]
    t = od[:, 0]; msk = (t >= tr[0]) & (t <= tr[-1]) & (t >= t_on - 0.05); t, X = t[msk], od[msk]
    xr, yr_, ub, vb, wr = (np.interp(t, tr, R[:, i]) for i in (0, 1, 3, 4, 5))
    th_r = np.interp(t, tr, np.unwrap(R[:, 2])); th = X[:, 3]
    vxr = np.cos(th_r) * ub - np.sin(th_r) * vb; vyr = np.sin(th_r) * ub + np.cos(th_r) * vb          # 기준 월드 속도
    vx = np.cos(th) * X[:, 4] - np.sin(th) * X[:, 5]; vy = np.sin(th) * X[:, 4] + np.cos(th) * X[:, 5]  # 추정기 월드 속도
    E = np.column_stack([xr - X[:, 1], yr_ - X[:, 2], wrap(th_r - th), vxr - vx, vyr - vy, wr - X[:, 6]])
    vref = np.hypot(vxr, vyr); win = vref > 0.05
    tx, ty = vxr / np.maximum(vref, 1e-6), vyr / np.maximum(vref, 1e-6)                              # 진행 방향 t̂, 왼쪽 n̂ = (−t_y, t_x)
    along = E[:, 0] * tx + E[:, 1] * ty; cross = -(E[:, 0] * -ty + E[:, 1] * tx)                       # cross + = 로봇이 기준 오른쪽
    v_along = E[:, 3] * tx + E[:, 4] * ty; v_cross = -(E[:, 3] * -ty + E[:, 4] * tx)
    name = os.path.basename(path)
    np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)), name + "_se.npz"), t=t - t[win][0], E=E, along=along, cross=cross,
             v_along=v_along, v_cross=v_cross, vref=vref, wr=wr, win=win)
    print(name, "ok", int(win.sum()))

#!/usr/bin/python3
"""d̂ 관측기의 한 스텝 잔차를 bag 에서 다시 계산 (torque_mpc.cpp update_disturbance 와 같은 식, reference 선형화):
  x = [p_x, p_y, θ, R(θ) v_몸체] (/state_estimator/odom, torque_mpc_node on_state 와 같음),  a = G(θ_{k−1}) τ_{k−1} + w(Ẋ_{k−1}),
  r_k = x_k − [p + Ts Ẋ + ½Ts² a ; Ẋ + Ts a]_{k−1}.  τ_{k−1} = 상태 k−1 을 받은 뒤 MPC 가 낸 τ (/torque_mpc/status, 시각으로 짝).
  몸체 좌표 (R(θ_k)ᵀ) 로 돌려: 위치·속도 잔차 표준편차 (Kalman R), skip_xy 1 mm 에 걸리는 비율, 원 위 곡률 중심 쪽 속도 잔차 / Ts 평균 (왼원·오른원)."""
import os, math, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "compare_all.py")).read()
g = {"__file__": os.path.join(HERE, "compare_all.py"), "__name__": "ca"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
cs, w, WS = g["cs"], g["w"], g["WS"]
Ts = 0.01; r, k = 0.04, 0.1932; m, Jw, Iz = 4.56, 8.9e-4, 0.06
mu = 4 * Jw / r ** 2; m_eff = m + mu; I_eff = Iz + mu * k * k
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
Minv = np.diag([1 / m_eff, 1 / m_eff, 1 / I_eff])


def read_odom(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/state_estimator/odom"]))
    out = []
    while rd.has_next():
        _, data, _ = rd.read_next(); o = deserialize_message(data, get_message(types["/state_estimator/odom"]))
        q = o.pose.pose.orientation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tw = o.twist.twist
        out.append([o.header.stamp.sec + o.header.stamp.nanosec * 1e-9, o.pose.pose.position.x, o.pose.pose.position.y, yaw,
                    tw.linear.x, tw.linear.y, tw.angular.z])
    return np.array(out)


for b in (sys.argv[1:] or ["212502", "212105"]):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path); S, E, P, A = w.load(path)
    O = read_odom(path); t = O[:, 0]; th = np.unwrap(O[:, 3])
    c, s = np.cos(th), np.sin(th); X = np.c_[O[:, 1], O[:, 2], th, c * O[:, 4] - s * O[:, 5], s * O[:, 4] + c * O[:, 5], O[:, 6]]
    ts = S[:, 0]; tau = S[:, 4:8]
    res = []
    for kk in range(1, len(t)):
        dt = t[kk] - t[kk - 1]
        if abs(dt - Ts) > 0.002:
            continue
        j = np.searchsorted(ts, t[kk - 1] + 1e-4)                # 상태 k−1 뒤 첫 MPC 출력
        if j >= len(ts) or ts[j] > t[kk]:
            continue
        x0 = X[kk - 1]; th0 = x0[2]; R3 = np.array([[math.cos(th0), -math.sin(th0), 0], [math.sin(th0), math.cos(th0), 0], [0, 0, 1]])
        a = Minv @ R3 @ F @ tau[j] + Minv @ np.array([mu * x0[5] * (-x0[4]), mu * x0[5] * x0[3], 0.0])
        pred = np.r_[x0[:3] + Ts * x0[3:] + 0.5 * Ts * Ts * a, x0[3:] + Ts * a]
        d = X[kk] - pred; d[2] = math.atan2(math.sin(d[2]), math.cos(d[2]))
        ck, sk = math.cos(X[kk, 2]), math.sin(X[kk, 2])
        pb = [ck * d[0] + sk * d[1], -sk * d[0] + ck * d[1]]; vb = [ck * d[3] + sk * d[4], -sk * d[3] + ck * d[4]]
        res.append([t[kk], pb[0], pb[1], d[2], vb[0], vb[1], d[5], max(abs(d[0]), abs(d[1]))])
    R_ = np.array(res); tr = R_[:, 0]
    rt = np.array([a_ for a_, _ in D["ref"]]); om, vr = [], []
    for x in tr:
        i = np.searchsorted(rt, x, side="right") - 1
        u = D["ref"][i][1].at(x)[1] if i >= 0 else np.zeros(3); om.append(u[2]); vr.append(math.hypot(u[0], u[1]))
    om = np.array(om); vr = np.array(vr)
    arm = cs.armed_mask(tr, A) & (vr > 0.02); circ = arm & (vr > 0.3) & (np.abs(om) > 0.3)
    print(f"== {b}  잔차 {arm.sum()} 개 (주행), 원 위 {circ.sum()}")
    print(f"   표준편차 (몸체)  위치 x·y {1e3*R_[arm,1].std():.3f}·{1e3*R_[arm,2].std():.3f} mm  θ {np.degrees(R_[arm,3].std()):.3f}°   "
          f"속도 x·y {1e3*R_[arm,4].std():.2f}·{1e3*R_[arm,5].std():.2f} mm/s  ω {np.degrees(R_[arm,6].std()):.3f}°/s")
    print(f"   skip_xy 1 mm 에 걸리는 비율 {100*np.mean(R_[arm,7] > 1e-3):.1f} %  (원 위 {100*np.mean(R_[circ,7] > 1e-3):.1f} %)")
    an = R_[:, 5] / Ts * np.sign(om)                          # 몸체 y × 곡률 부호 = 곡률 중심 쪽 (tangent)
    for nm, mm in (("왼원", circ & (om > 0)), ("오른원", circ & (om < 0))):
        keep = mm & (R_[:, 7] <= 1e-3)
        print(f"   {nm}: 속도 잔차 곡률 중심 쪽 /Ts 평균 {np.mean(an[mm]):+.3f} m/s² (skip_xy 통과분만 {np.mean(an[keep]):+.3f}),"
              f" 몸체 y /Ts {np.mean(R_[mm,5])/Ts:+.3f}, 몸체 x /Ts {np.mean(R_[mm,4])/Ts:+.3f}")

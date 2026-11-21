#!/usr/bin/python3
"""오늘 실차 bag 에서 바퀴별 헛돎 시작을 찾아 그 순간의 접지 토크로 바퀴별 μ 를 낸다 (FR 재측정 대신, 로봇을 안 움직이고).
  ω_nom,i = (T⁻¹ [v_x, v_y, ω_z])_i,  v = SLAM 중심 속도 (±0.1 s 차분, 몸체 축), ω_z = IMU 자이로 (/imu/base_link)
  슬립 s_i = ω_i − ω_nom,i [rad/s]. 헛돎 시작 = |s_i| > max(THR, 0.25|ω_nom,i|) 이 HOLD 이어진 첫 시각 (같은 바퀴 다음 사건은 0.5 s 뒤부터)
  접지 토크 τ_c,i = |τ_fb,i| − (C_i + V_i|ω_i|) − J_w|ω̇_i|  (2026-11-17_ramp/wheel_slip_onset.py 와 같은 식, 11-16 마찰표), 시작 30 ms 전 값
  μ_i = τ_c,i / ((r/√2)(m g/4))      → 그 바퀴가 그 순간 버틴 최대 = 한계의 추정 (헛돌기 직전)
  ※ SLAM 속도 잡음 ≈ 0.1 m/s = 2.5 rad/s → THR 6 rad/s. CAN 응답이 빠진(fresh false) 표본은 뺀다."""
import importlib.util, json, math, os, sys
import numpy as np, yaml
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
P_ = yaml.safe_load(open(f"{MPC}/config/torque_plant.yaml")); est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]
r = P_["geometry"]["wheel_radius_m"]; k = (P_["geometry"]["wheel_base_m"] + P_["geometry"]["robot_width_m"]) / 2
m, g, Jw = P_["body"]["mass_kg"], 9.81, P_["wheel"]["inertia_J_w_kgm2"]
C = np.array(P_["friction"]["coulomb_Nm"]); V = np.array(P_["friction"]["viscous_Nms_per_rad"])
UNIT = r / math.sqrt(2) * m * g / 4
Tinv = np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]) / r
off = (est["imu_offset_x"], est["imu_offset_y"]); THR, HOLD = 6.0, 0.03
W = ["FL", "FR", "RL", "RR"]


def load(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    want = ["/wheel_states", "/tracked_pose", "/imu/base_link", "/mpc/armed"]
    rd.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    ws, pose, gz, arm = [], [], [], []
    while rd.has_next():
        tp, data, tb = rd.read_next(); mm = deserialize_message(data, get_message(types[tp]))
        if tp == "/wheel_states":
            ws.append([st(mm.header), *mm.velocity, *mm.torque, *[float(f) for f in mm.fresh]])
        elif tp == "/tracked_pose":
            pose.append([st(mm.header), mm.pose.position.x, mm.pose.position.y, yaw(mm.pose.orientation)])
        elif tp == "/imu/base_link":
            gz.append([st(mm.header), mm.angular_velocity.z])
        elif tp == "/mpc/armed":
            arm.append((tb * 1e-9, bool(mm.data)))
    return np.array(ws), np.array(pose), np.array(gz), arm


def events(bag):
    ws, P, G, arm = load(bag)
    if len(ws) < 100 or len(P) < 20 or len(G) < 100:
        return []
    t = ws[:, 0]; om = ws[:, 1:5]; tfb = ws[:, 5:9]; fresh = ws[:, 9:13] > 0.5
    xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], off); tp = P[:, 0]; h = 0.1
    vx = (np.interp(t + h, tp, xc) - np.interp(t - h, tp, xc)) / (2 * h); vy = (np.interp(t + h, tp, yc) - np.interp(t - h, tp, yc)) / (2 * h)
    th = np.interp(t, tp, np.unwrap(P[:, 3])); c, s = np.cos(th), np.sin(th)
    bx, by = c * vx + s * vy, -s * vx + c * vy; wz = np.interp(t, G[:, 0], G[:, 1])
    nom = (Tinv @ np.stack([bx, by, wz])).T
    sl = om - nom
    ok = (t > tp[0] + h) & (t < tp[-1] - h) & cs.armed_mask(t, arm)
    dom = np.gradient(om, t, axis=0)
    out = []
    for i in range(4):
        bad = ok & fresh[:, i] & (np.abs(sl[:, i]) > np.maximum(THR, 0.25 * np.abs(nom[:, i])))
        j, last = 0, -1e9
        while j < len(t):
            if bad[j] and t[j] > last + 0.5:
                e = j
                while e < len(t) and bad[e]:
                    e += 1
                if t[min(e, len(t) - 1)] - t[j] >= HOLD:
                    p = np.searchsorted(t, t[j] - 0.03)                     # 시작 30 ms 전
                    tc = abs(tfb[p, i]) - (C[i] + V[i] * abs(om[p, i])) - Jw * abs(dom[p, i])
                    out.append(dict(bag=os.path.basename(bag), wheel=W[i], t=float(t[j]), slip_peak=float(np.abs(sl[j:e, i]).max()),
                                    dur=float(t[min(e, len(t) - 1)] - t[j]), tau_fb=float(abs(tfb[p, i])), tau_c=float(tc), mu=float(tc / UNIT),
                                    quad=int(1 if (om[p, i] >= 0) == (tfb[p, i] >= 0) else 2) if abs(om[p, i]) > 1 else 0,
                                    omega=float(om[p, i]), others=[W[q] for q in range(4) if q != i and bad[j]]))
                    last = t[min(e, len(t) - 1)]
                j = e
            else:
                j += 1
    return out


if __name__ == "__main__":
    allev = []
    for b in sys.argv[1:]:
        try:
            ev = events(b.rstrip("/"))
        except Exception as ex:
            print(f"{b}: 건너뜀 {ex}"); continue
        allev += ev
        print(f"{os.path.basename(b.rstrip('/'))}: 헛돎 사건 {len(ev)}")
    json.dump(allev, open(os.path.join(HERE, "slip_events.json"), "w"), ensure_ascii=False, indent=1)
    print("\n바퀴 | 사건 수 | μ (헛돌기 30 ms 전 접지 토크) 중앙값 [최소~최대] | 구동(+)·제동(−) 사건 수")
    for w in W:
        e = [x for x in allev if x["wheel"] == w]
        if e:
            mu = np.array([x["mu"] for x in e])
            print(f"{w} | {len(e)} | {np.median(mu):.3f} [{mu.min():.3f} ~ {mu.max():.3f}] | 구동 {sum(x['quad'] == 1 for x in e)} · 제동 {sum(x['quad'] == 2 for x in e)}")
        else:
            print(f"{w} | 0 |")

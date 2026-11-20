"""구간별 진행 방향 오차 (SLAM 중심 − 기준) 와 바퀴별 상한 닿음 (/torque_mpc/cmd_raw |τ*_i| ≥ τ̄_i − 1e-4, τ̄ = MPC 시작 로그 값). 2026-11-19"""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OFF = np.array([0.1093, 0.0173]); TB = np.array([0.1093, 0.0721, 0.1154, 0.0886])   # 바퀴별 μ̂ 상한 (11)
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/dra_mppi/reference_trajectory", "/tracked_pose", "/torque_mpc/cmd_raw"]))
    R, S, U = [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == "/dra_mppi/reference_trajectory" and m.references:
            q = m.references[0]; R.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + m.dt, q.x, q.y, q.yaw, math.hypot(q.vx, q.vy)])
        elif tp == "/tracked_pose":
            th = 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w); c, s = math.cos(th), math.sin(th)
            S.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x - (c * OFF[0] - s * OFF[1]), m.pose.position.y - (s * OFF[0] + c * OFF[1])])
        elif m.mode == 0: U.append([tr, *m.torque])
    R, S, U = map(np.array, (R, S, U)); R = R[np.argsort(R[:, 0])]
    mov = R[R[:, 4] > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
    cr = R[R[:, 4] >= 0.999 * R[:, 4].max()]; c0, c1 = cr[0, 0], cr[-1, 0]; cm = 0.5 * (c0 + c1)
    Sm = S[(S[:, 0] >= t0) & (S[:, 0] <= t1)]
    ex, ey = Sm[:, 1] - np.interp(Sm[:, 0], R[:, 0], R[:, 1]), Sm[:, 2] - np.interp(Sm[:, 0], R[:, 0], R[:, 2])
    yw = np.interp(Sm[:, 0], R[:, 0], np.unwrap(R[:, 3])); ea = 1e3 * (np.cos(yw) * ex + np.sin(yw) * ey)
    ph = [("가속", t0, c0), ("순항 앞", c0, cm), ("순항 뒤", cm, c1), ("감속", c1, t1)]
    print(f"== {path.split('/')[-1]}  진행 방향 오차 평균 [mm] (− = 뒤처짐): " +
          "  ".join(f"{n} {ea[(Sm[:,0]>=a)&(Sm[:,0]<b)].mean():+.0f}" for n, a, b in ph) + f"  | 끝 {ea[-1]:+.0f}")
    Um = U[(U[:, 0] >= t0) & (U[:, 0] <= t1)]; sat = np.abs(Um[:, 1:5]) >= TB - 1e-4
    print(f"   상한 닿음 시간 % [FL FR RL RR]: {np.round(100*sat.mean(0),0)}  아무 바퀴 {100*sat.any(1).mean():.0f} %  |  "
          + "  ".join(f"{n} 아무 바퀴 {100*sat[(Um[:,0]>=a)&(Um[:,0]<b)].any(1).mean():.0f}%" for n, a, b in ph))

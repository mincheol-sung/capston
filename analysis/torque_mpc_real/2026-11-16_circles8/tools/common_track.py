"""제어기와 상관없는 공통 잣대: 추정기 포즈 vs 기준 포즈 (기준열 첫 행, 목표 시각 = stamp + dt).
몸체 앞·옆 위치 오차, 헤딩 오차, 교차점(기준 ω 부호 바뀜) ±0.6 s 와 나머지, 바퀴 토크(제어기 순토크·모터 명령)·슬립."""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r_w, k = 0.04, (0.1897 + 0.207) / 2
Tinv = (1 / r_w) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
NET = ["/torque_mpc/cmd_raw", "/wheel_pi/cmd_raw"]
T = ["/dra_mppi/reference_trajectory", "/state_estimator/odom", "/mpc/armed", "/wheel_torque_cmd", "/wheel_states"] + NET
print("판      | 제어기 | 위치 RMS/최대 cm | 앞 RMS | 옆 RMS (평균) | 헤딩 RMS/최대° | 교차점±0.6s 위치 RMS | |τ순| 최대 N·m | |τ순|>0.101 / >0.152 시간 % | Δτ RMS | 슬립 RMS m/s")
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=[x for x in T if x in types]))
    R, O, A, M, W, N = [], [], [], [], [], {n: [] for n in NET}
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and m.references:
            q = m.references[0]; R.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + m.dt, q.x, q.y, q.yaw, q.vx, q.yaw_rate])
        elif tp == T[1]:
            O.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                      2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z])
        elif tp == T[2]: A.append((tr, m.data))
        elif tp == T[3] and m.mode == 0: M.append([tr, *m.torque])
        elif tp == T[4]: W.append([tr, *m.velocity])
        elif tp in NET and m.mode == 0: N[tp].append([tr, *m.torque])
    if len(R) < 10:   # 무장 안 한 판 (2026-11-17 131505) — 기준이 안 나와 비교할 게 없다
        print(f"{path.split('/')[-1]} | 기준 {len(R)} 행 — 건너뜀"); continue
    R, O, M, W = map(np.array, (R, O, M, W)); R = R[np.argsort(R[:, 0])]
    net_topic = max(NET, key=lambda n: len(N[n])); Nt = np.array(N[net_topic]); ctrl = "MPC" if "mpc" in net_topic else "PP"
    mov = R[np.abs(R[:, 4]) > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
    sel = (O[:, 0] >= t0) & (O[:, 0] <= t1); Os = O[sel]
    rx, ry = np.interp(Os[:, 0], R[:, 0], R[:, 1]), np.interp(Os[:, 0], R[:, 0], R[:, 2])
    ryaw = np.interp(Os[:, 0], R[:, 0], np.unwrap(R[:, 3]))
    ex, ey = Os[:, 1] - rx, Os[:, 2] - ry; c, s = np.cos(ryaw), np.sin(ryaw)
    ef, es = c * ex + s * ey, -s * ex + c * ey; e = np.hypot(ex, ey)
    eth = np.degrees(np.angle(np.exp(1j * (Os[:, 3] - ryaw))))
    w = R[:, 5]; flips = R[1:, 0][(np.sign(w[1:]) != np.sign(w[:-1])) & (np.abs(w[1:]) > 0.2) & (np.abs(w[:-1]) > 0.2)]
    near = np.zeros(len(Os), bool)
    for f in flips: near |= np.abs(Os[:, 0] - f) < 0.6
    # 토크는 수신 시각 기준이라 주행 구간을 수신 시각으로 다시 (기준 시각과 거의 같다)
    Nm = Nt[(Nt[:, 0] >= t0) & (Nt[:, 0] <= t1)]; amax = np.abs(Nm[:, 1:]).max(1)
    dtau = np.sqrt(np.mean(np.diff(Nm[:, 1:], axis=0) ** 2))
    Wm = W[(W[:, 0] >= t0) & (W[:, 0] <= t1)]
    iO = np.clip(np.searchsorted(O[:, 0], Wm[:, 0]), 0, len(O) - 1)
    vb = np.linalg.pinv(Tinv) @ Wm[:, 1:].T; slip = np.sqrt(np.mean((vb[0] - O[iO, 4]) ** 2 + (vb[1] - O[iO, 5]) ** 2))
    print(f"{path.split('/')[-1]} | {ctrl:4s}  | {100*np.sqrt(np.mean(e**2)):5.1f} / {100*e.max():5.1f}   | {100*np.sqrt(np.mean(ef**2)):4.1f}  | {100*np.sqrt(np.mean(es**2)):4.1f} ({100*es.mean():+4.1f}) "
          f"| {np.sqrt(np.mean(eth**2)):4.1f} / {np.abs(eth).max():4.1f}  | {100*np.sqrt(np.mean(e[near]**2)) if near.any() else float('nan'):5.1f}              "
          f"| {amax.max():.3f}          | {100*np.mean(amax > 0.1012):4.0f} / {100*np.mean(amax > 0.1518):4.0f}          | {dtau:.4f} | {slip:.3f}")

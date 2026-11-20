"""제어기 순토크를 몸체 힘(F_x, F_y, M_z = T_inv^T τ) + 영공간 n = [1,1,-1,-1]·τ/2 로 나누고,
기준이 요구하는 힘 F_ref = M_eff (u̇ + [-ω v_y, ω v_x, 0]) (w 항 없음, 바퀴 등가 질량 포함) 과 비교. 2026-11-17 PP vs MPC."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1897 + 0.207) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
m, Iz, mw = 4.56, 0.060, 2.225
Meff = np.diag([m + mw, m + mw, Iz + mw * k * k])
NET = ["/torque_mpc/cmd_raw", "/wheel_pi/cmd_raw"]; REF = "/dra_mppi/reference_trajectory"
print("판     | 제어기 | 순토크 RMS | 힘 성분 τ RMS (F_x/F_y/M_z 몫) | 영공간 RMS | 요구 τ RMS (F_x/F_y/M_z) | 요구 |τ| 최대 | 명령−요구 F_y RMS N")
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=[x for x in NET + [REF] if x in types]))
    R, N = [], {n: [] for n in NET}
    while rd.has_next():
        tp, d, tr = rd.read_next(); msg = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == REF and msg.references:
            q = msg.references[0]; R.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + msg.dt, q.vx, q.vy, q.yaw_rate])
        elif tp in NET and msg.mode == 0: N[tp].append([tr, *msg.torque])
    if len(R) < 10: print(path.split('/')[-1], "기준 없음"); continue
    R = np.array(R); R = R[np.argsort(R[:, 0])]
    nt = max(NET, key=lambda n: len(N[n])); Nt = np.array(N[nt]); ctrl = "MPC" if "mpc" in nt else "PP"
    mov = R[np.abs(R[:, 1]) > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
    Nm = Nt[(Nt[:, 0] >= t0) & (Nt[:, 0] <= t1)]; tau = Nm[:, 1:]
    F = tau @ Tinv                      # 행마다 T_inv^T τ
    P = np.linalg.pinv(Tinv.T)          # 힘 → 최소 노름 토크
    tf = F @ P.T; tn = tau - tf         # 힘 성분 / 영공간 성분 (직교 분해)
    # 기준 요구 힘 (기준 시각 격자, 중앙 차분 0.2 s)
    tt = R[:, 0]; u = R[:, 1:4]; h = 0.1
    ud = np.stack([(np.interp(tt + h, tt, u[:, j]) - np.interp(tt - h, tt, u[:, j])) / (2 * h) for j in range(3)], 1)
    a = ud + np.stack([-u[:, 2] * u[:, 1], u[:, 2] * u[:, 0], 0 * tt], 1)
    Fr = a @ Meff.T; sr = (tt >= t0) & (tt <= t1); tr_ = Fr[sr] @ P.T
    comp = lambda Fx: np.sqrt(np.mean(np.sum((Fx @ P.T) ** 2, 1)))
    one = lambda Fm, j: comp(Fm * (np.arange(3) == j))
    Fy_ref = np.interp(Nm[:, 0], tt, Fr[:, 1])
    print(f"{path.split('/')[-1]} | {ctrl:4s} | {np.sqrt(np.mean(np.sum(tau**2,1)/4)):.4f}   | {np.sqrt(np.mean(np.sum(tf**2,1)/4)):.4f} "
          f"({one(F,0)/2:.4f}/{one(F,1)/2:.4f}/{one(F,2)/2:.4f}) | {np.sqrt(np.mean(np.sum(tn**2,1)/4)):.4f}   "
          f"| {np.sqrt(np.mean(np.sum(tr_**2,1)/4)):.4f} ({one(Fr[sr],0)/2:.4f}/{one(Fr[sr],1)/2:.4f}/{one(Fr[sr],2)/2:.4f}) | {np.abs(tr_).max():.4f} "
          f"| {np.sqrt(np.mean((F[:,1]-Fy_ref)**2)):.2f}")

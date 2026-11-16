"""원마다 추정기 상태: SLAM 보정 잔차(몸체 앞·옆)의 평균, 가속도 바이어스 추정 b_ay. /state_estimator/status = [t, e_x, e_y, e_ψ, b_ax, b_ay, b_g, 튐, 보정, 나이]."""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
T = ["/dra_mppi/reference_trajectory", "/state_estimator/status", "/mpc/armed", "/state_estimator/odom"]
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=T))
    R, E, A, O = [], [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and m.references: q = m.references[0]; R.append([tr, q.vx, q.yaw_rate])
        elif tp == T[1]: E.append([tr, *m.data[1:7]])
        elif tp == T[2]: A.append((tr, m.data))
        else: O.append([tr, 2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)])
    R, E, O = map(np.array, (R, E, O)); arm = [a for a, v in A if v][0]; R = R[R[:, 0] > arm]
    mov = R[np.abs(R[:, 1]) > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
    w = R[:, 2]; flip = R[1:, 0][(np.sign(w[1:]) != np.sign(w[:-1])) & (np.abs(w[1:]) > 0.2) & (np.abs(w[:-1]) > 0.2)][0]
    psi = np.interp(E[:, 0], O[:, 0], np.unwrap(O[:, 1]))
    rf = np.cos(psi) * E[:, 1] + np.sin(psi) * E[:, 2]; rs = -np.sin(psi) * E[:, 1] + np.cos(psi) * E[:, 2]
    print(f"== {path.split('/')[-1]}")
    for name, msk in (("정지 (무장 전)", E[:, 0] < arm), ("첫 원 (반시계)", (E[:, 0] > t0 + 1) & (E[:, 0] < flip - 0.6)), ("둘째 원 (시계)", (E[:, 0] > flip + 0.6) & (E[:, 0] < t1 - 1))):
        print(f"   {name:14s}: SLAM 보정 잔차 평균 앞 {1000*rf[msk].mean():+5.1f} 옆 {1000*rs[msk].mean():+5.1f} mm | b_ax {E[msk,4].mean():+.3f} b_ay {E[msk,5].mean():+.3f} m/s² | b_g {E[msk,6].mean():+.4f} rad/s")

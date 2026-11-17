"""circles8: 첫 원(반시계)·두 번째 원(시계)·끝 감속 구간별 오차 (앞/옆 = 몸체 축), 그리고 오차 진동의 주된 주파수."""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
T = ["/dra_mppi/reference_trajectory", "/torque_mpc/status", "/mpc/armed", "/state_estimator/odom"]
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=T))
    R, S, A, O = [], [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and m.references: q = m.references[0]; R.append([tr, q.vx, q.yaw_rate])
        elif tp == T[1]: S.append([tr, m.data[1], m.data[2], m.data[3]])
        elif tp == T[2]: A.append((tr, m.data))
        else: O.append([tr, 2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)])
    R, S, O = map(np.array, (R, S, O)); arm = [a for a, v in A if v][0]; R = R[R[:, 0] > arm]
    mov = R[np.abs(R[:, 1]) > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
    w = R[:, 2]; flip = R[1:, 0][(np.sign(w[1:]) != np.sign(w[:-1])) & (np.abs(w[1:]) > 0.2) & (np.abs(w[:-1]) > 0.2)][0]
    S = S[(S[:, 0] >= t0) & (S[:, 0] <= t1)]
    psi = np.interp(S[:, 0], O[:, 0], np.unwrap(O[:, 1]))
    ef = np.cos(psi) * S[:, 1] + np.sin(psi) * S[:, 2]; es = -np.sin(psi) * S[:, 1] + np.cos(psi) * S[:, 2]   # 몸체 앞·옆
    print(f"== {path.split('/')[-1]}  (교차점 {flip-t0:.1f} s, 끝 {t1-t0:.1f} s)")
    for name, msk in (("첫 원 (반시계)", (S[:, 0] < flip - 0.6)), ("둘째 원 (시계)", (S[:, 0] > flip + 0.6) & (S[:, 0] < t1 - 2.0)), ("끝 2 s 감속", S[:, 0] >= t1 - 2.0)):
        print(f"   {name:14s}: 앞 RMS {100*np.sqrt(np.mean(ef[msk]**2)):4.1f} 평균 {100*ef[msk].mean():+5.1f} cm | 옆 RMS {100*np.sqrt(np.mean(es[msk]**2)):4.1f} 평균 {100*es[msk].mean():+5.1f} cm")
    # 주행 중 앞 오차의 주된 주파수 (평균 빼고 FFT)
    msk = (S[:, 0] > t0 + 1) & (S[:, 0] < t1 - 1); x = ef[msk] - ef[msk].mean(); tt = S[msk, 0]
    tu = np.arange(tt[0], tt[-1], 0.01); xu = np.interp(tu, tt, x)
    F = np.abs(np.fft.rfft(xu * np.hanning(len(xu)))); f = np.fft.rfftfreq(len(xu), 0.01)
    band = (f > 0.3) & (f < 10); fp = f[band][np.argmax(F[band])]
    print(f"   앞 오차 진동 주된 주파수 {fp:.2f} Hz")

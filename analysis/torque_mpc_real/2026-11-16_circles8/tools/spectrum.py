"""순토크의 몸체 힘 성분(F_x, F_y, M_z)에서 기준 요구 힘을 뺀 피드백 몫의 주파수 — 한계 순환이 있는지. 2026-11-17"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1897 + 0.207) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
NET = ["/torque_mpc/cmd_raw", "/wheel_pi/cmd_raw"]; ODO = "/state_estimator/odom"; REF = "/dra_mppi/reference_trajectory"
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=[x for x in NET + [REF, ODO] if x in types]))
    R, N, O = [], {n: [] for n in NET}, []
    while rd.has_next():
        tp, d, tr = rd.read_next(); msg = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == REF and msg.references:
            q = msg.references[0]; R.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + msg.dt, q.vx])
        elif tp in NET and msg.mode == 0: N[tp].append([tr, *msg.torque])
        elif tp == ODO: O.append(tr)
    R = np.array(R); nt = max(NET, key=lambda n: len(N[n])); Nt = np.array(N[nt])
    mov = R[np.abs(R[:, 1]) > 1e-3]; t0, t1 = mov[:, 0].min(), mov[:, 0].max()
    Nm = Nt[(Nt[:, 0] >= t0) & (Nt[:, 0] <= t1)]; dt = np.median(np.diff(Nm[:, 0])); O = np.array(O); do = np.median(np.diff(O))
    F = Nm[:, 1:] @ Tinv
    out = [f"{path.split('/')[-1]} {'MPC' if 'mpc' in nt else 'PP '}  명령 주기 {1e3*dt:.1f} ms  추정기 주기 {1e3*do:.1f} ms |"]
    for j, nm in enumerate(["F_x", "F_y", "M_z"]):
        x = F[:, j] - np.convolve(F[:, j], np.ones(25) / 25, "same")   # 0.5 s 이동평균 뺀 빠른 몫
        X = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2; f = np.fft.rfftfreq(len(x), dt)
        b = f > 0.5; pk = f[b][np.argmax(X[b])]
        hi = X[f > 5].sum() / X[b].sum()
        out.append(f"{nm} 빠른 몫 RMS {np.sqrt(np.mean(x**2)):.2f}, 정점 {pk:.1f} Hz, 5 Hz 위 {100*hi:.0f}%")
    print("  ".join(out))

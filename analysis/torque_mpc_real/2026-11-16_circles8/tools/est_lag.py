"""추정기 몸체 속도 vs 엔코더 속도 pinv(T_inv)·ω 의 시간 어긋남 (상호상관, 수신 시각 기준). 2026-11-17
MPC 는 추정기 상태로 루프를 닫고, PP 의 바퀴 PI 는 엔코더 /wheel_states 로 닫는다 — 둘의 지연 차이."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1897 + 0.207) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]); P = np.linalg.pinv(Tinv)
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/state_estimator/odom", "/wheel_states"]))
    O, W = [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); msg = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == "/wheel_states": W.append([tr, *msg.velocity])
        else:
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            O.append([tr, st, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
    O, W = np.array(O), np.array(W); vb = (P @ W[:, 1:].T).T
    mv = np.abs(vb[:, 0]) > 0.2; t0, t1 = W[mv, 0].min(), W[mv, 0].max()
    g = np.arange(t0, t1, 0.002); out = [path.split('/')[-1], f"추정기 stamp→수신 {1e3*np.median(O[:,0]-O[:,1]):.1f} ms |"]
    for j, nm in enumerate(["v_x", "v_y", "ω"]):
        a = np.interp(g, W[:, 0], vb[:, j]); b = np.interp(g, O[:, 0], O[:, 2 + j])
        a, b = a - np.convolve(a, np.ones(500) / 500, "same"), b - np.convolve(b, np.ones(500) / 500, "same")   # 1 s 아래 몫만
        lags = np.arange(-50, 151); c = [np.dot(a[100:-200], b[100 + L:len(b) - 200 + L]) for L in lags]
        out.append(f"{nm} 추정기가 엔코더보다 {2*lags[int(np.argmax(c))]:+d} ms")
    print("  ".join(out))

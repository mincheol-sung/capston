"""주행 bag 에서 바퀴별로 '미끄러지지 않고 실제로 땅에 전달된 토크' 의 상위값 → μ_i 하한. 2026-11-19
τ_c,i = τ_측정,i − (C_i + V_i|ω_i|)·sign(ω_i) − J_w·ω̇_i     (torque_plant.yaml 2026-11-16 마찰표, J_w 8.9e-4)
슬립 s_i = r·(ω_i − (T_inv v_est)_i)  — |s_i| < S 이고 |ω_i| > 2 rad/s (정지 마찰 구간 제외) 인 샘플만.
μ_i ≥ |τ_c,i| / ((r/√2)(mg/4))  — 이 토크를 미끄러지지 않고 냈으니 한계는 그 이상."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1897 + 0.207) / 2; Jw = 8.9e-4
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
C = np.array([0.0174, 0.0206, 0.0155, 0.0157]); V = np.array([0.001066, 0.001158, 0.001112, 0.001177])
den = r / np.sqrt(2) * 4.56 * 9.81 / 4; S = float(sys.argv[1])
for path in sys.argv[2:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=["/wheel_states", "/state_estimator/odom"]))
    W, O = [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == "/wheel_states": W.append([tr, *m.velocity, *m.torque])
        else: O.append([tr, m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z])
    W, O = np.array(W), np.array(O)
    if len(O) < 100: print(path.split('/')[-1], "추정기 없음"); continue
    t = W[:, 0]; om = W[:, 1:5]; tm = W[:, 5:9]
    od = np.stack([np.gradient(om[:, j], t) for j in range(4)], 1)
    od = np.stack([np.convolve(od[:, j], np.ones(5) / 5, "same") for j in range(4)], 1)   # 50 ms 평활
    vo = np.stack([np.interp(t, O[:, 0], O[:, j]) for j in (1, 2, 3)], 1)
    s = r * (om - vo @ Tinv.T)
    tc = tm - (C + V * np.abs(om)) * np.sign(om) - Jw * od
    out = [path.split('/')[-1]]
    for i, w in enumerate(("FL", "FR", "RL", "RR")):
        ok = (np.abs(s[:, i]) < S) & (np.abs(om[:, i]) > 2.0) & (t > O[0, 0]) & (t < O[-1, 0])
        if ok.sum() < 50: out.append(f"{w} 표본 부족"); continue
        q = np.percentile(np.abs(tc[ok, i]), 99)
        out.append(f"{w} |τ_c| 99% {q:.3f} (μ≥{q/den:.2f}, n {ok.sum()})")
    print("  ".join(out))

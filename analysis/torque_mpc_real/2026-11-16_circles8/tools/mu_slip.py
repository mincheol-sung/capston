"""바퀴마다 '바닥에 요구한 μ' 와 슬립을 짝짓는다 (mu_ramp_test 와 같은 45° 롤러 정의).
  τ_ground,i = τ_net,i − J_w·ω̇_i   (τ_net = MPC 순토크 /torque_mpc/cmd_raw — 구동계 마찰은 torque_output 이 따로 보상)
  μ_dem,i   = √2·|τ_ground,i| / (r·N),  N = m g / 4
  슬립,i    = r·(ω_i − c_i)·sign(τ_ground,i),  c = T⁻¹·v_body (추정기 몸체 속도로 본 무슬립 바퀴속도)"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k, m, g, Jw = 0.04, (0.1897 + 0.207) / 2, 4.56, 9.81, 8.9e-4
N = m * g / 4
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
T = ["/torque_mpc/cmd_raw", "/wheel_states", "/state_estimator/odom", "/mpc/armed"]
bins = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.9]
allrows = []
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=T))
    C, W, O, A = [], [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); msg = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and msg.mode == 0: C.append([tr, *msg.torque])
        elif tp == T[1]: W.append([tr, *msg.velocity])
        elif tp == T[2]: O.append([tr, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
        elif tp == T[3]: A.append((tr, msg.data))
    C, W, O = map(np.array, (C, W, O))
    arm = [a for a, v in A if v][0]
    W = W[W[:, 0] > arm]
    wd = np.gradient(W[:, 1:5], W[:, 0], axis=0)
    from scipy.ndimage import uniform_filter1d
    wd = uniform_filter1d(wd, 5, axis=0)
    iC = np.searchsorted(C[:, 0], W[:, 0]) - 1; iO = np.clip(np.searchsorted(O[:, 0], W[:, 0]), 0, len(O) - 1)
    ok = (iC >= 0) & (W[:, 0] - C[np.clip(iC, 0, None), 0] < 0.03)
    tg = C[np.clip(iC, 0, None), 1:5] - Jw * wd
    c = (Tinv @ O[iO, 1:4].T).T
    mu = np.sqrt(2) * np.abs(tg) / (r * N)
    slip = r * (W[:, 1:5] - c) * np.sign(tg)
    allrows.append((path.split('/')[-1], mu[ok].ravel(), slip[ok].ravel(), np.abs(W[ok, 1:5]).ravel() * r))
print("μ_dem 구간 | " + " | ".join(f"{n:^22s}" for n, *_ in allrows))
print("           | " + " | ".join(f"{'표본':>5s} {'상대슬립':>7s} {'90%':>6s}" for _ in allrows))
for lo, hi in zip(bins[:-1], bins[1:]):
    cells = []
    for n, mu, sl, sp in allrows:
        s = (mu >= lo) & (mu < hi)
        rel = sl[s] / np.maximum(sp[s], 0.1)            # 바퀴 표면 속도 대비 (속도에 비례하는 기구학 차이 분리)
        cells.append(f"{s.sum():5d} {np.median(rel) if s.any() else np.nan:+7.2f} {np.percentile(rel, 90) if s.sum() > 10 else np.nan:+6.2f}")
    print(f"{lo:.1f}~{hi:.1f}    | " + " | ".join(cells))

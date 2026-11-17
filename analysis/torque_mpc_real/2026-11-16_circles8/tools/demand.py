"""기준 궤적이 요구하는 바퀴 토크 (모델 그대로 따라갈 때의 목표 입력) vs 상한 τ̄.
yaw 고정이라 몸체 가속 = d/dt(기준 몸체 속도). F_body = M·a, τ = ((T⁻¹)ᵀ)⁺ F (최소 노름, 1판 식-14 와 같은 분배)."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k, m, Jw, Iz = 0.04, (0.1897 + 0.207) / 2, 4.56, 8.9e-4, 0.06
mw = 4 * Jw / r ** 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
P = np.linalg.pinv(Tinv.T)
print("판     | 속도 | τ̄       | 가정                       | 최대 필요 |τ| N·m          | τ̄ 넘는 시간 (주행 중)")
for path, tb in zip(sys.argv[1::2], map(float, sys.argv[2::2])):
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=["/dra_mppi/reference_trajectory"]))
    seen = None
    while rd.has_next():
        _, d, _ = rd.read_next(); msg = deserialize_message(d, get_message(types["/dra_mppi/reference_trajectory"]))
        rows = np.array([[q.vx, q.vy, q.yaw_rate] for q in msg.references])
        if len(rows) and np.hypot(*rows[:, :2].T).max() > 0.2 and (seen is None or len(rows) >= len(seen[1])):
            seen = (msg.dt, rows)          # 지평이 가장 긴(=주행 한가운데) 기준열 하나만 있으면 모양이 다 나오지 않으므로 아래에서 전체를 다시 모은다
    # 전체 주행 기준: 각 메시지의 첫 행을 이어 붙인다
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/dra_mppi/reference_trajectory"]))
    ts, vs = [], []
    while rd.has_next():
        _, d, tr = rd.read_next(); msg = deserialize_message(d, get_message(types["/dra_mppi/reference_trajectory"]))
        if msg.references:
            q = msg.references[0]; ts.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + msg.dt); vs.append([q.vx, q.vy, q.yaw_rate])
    ts, vs = np.array(ts), np.array(vs)
    o = np.argsort(ts); ts, vs = ts[o], vs[o]
    keep = np.r_[True, np.diff(ts) > 1e-6]; ts, vs = ts[keep], vs[keep]
    mov = np.hypot(vs[:, 0], vs[:, 1]) > 1e-3
    a = np.gradient(vs, ts, axis=0)                     # yaw 고정 → 몸체 가속 = 몸체 속도의 미분
    # 5-2 바닥 효율(M = m 기준 0.65·0.40·0.40)을 M = m_eq 기준으로 바꾸면 앞 0.65/0.672 = 0.97, 옆 0.40/0.672 = 0.60, 회전 0.40/0.407 = 0.98
    for label, Mx, Mz, ex, ey in (("m_eq (손실 없음)", m + mw, Iz + mw * k * k, 1.0, 1.0),
                                  ("m_eq + 바닥 효율 0.97/0.60", m + mw, Iz + mw * k * k, 0.97, 0.60),
                                  ("m 4.56 (오전 모델)", m, Iz, 1.0, 1.0)):
        F = np.column_stack([Mx * a[:, 0] / ex, Mx * a[:, 1] / ey, Mz * a[:, 2]])
        tau = (P @ F.T).T
        need = np.abs(tau).max(axis=1)
        over = mov & (need > tb)
        print(f"{path.split('/')[-1]} | {np.hypot(vs[:,0],vs[:,1]).max():.2f} | {tb:.4f} | {label:26s} | {need[mov].max():.4f} ({need[mov].max()/tb*100:4.0f} % of τ̄) | {100*over.sum()/mov.sum():5.1f} %")

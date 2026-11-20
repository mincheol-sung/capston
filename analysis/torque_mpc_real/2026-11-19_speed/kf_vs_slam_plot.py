"""180025 (est_mode slam) vs 180149 (kf): 엔코더 속도, 진행 방향 오차 (SLAM 중심), FR 상한 닿음 — 시간축. 2026-11-19"""
import sys, math, numpy as np, rosbag2_py, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1724 + 0.2140) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]); P = np.linalg.pinv(Tinv); OFF = np.array([0.1093, 0.0173])
TB = np.array([0.1093, 0.0721, 0.1154, 0.0886])
fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
for path, lab in zip(sys.argv[1:-1], ["est_mode slam", "est_mode kf"]):
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/dra_mppi/reference_trajectory", "/tracked_pose", "/torque_mpc/cmd_raw", "/wheel_states"]))
    R, S, U, W = [], [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == "/dra_mppi/reference_trajectory" and m.references:
            q = m.references[0]; R.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + m.dt, q.x, q.y, q.yaw, math.hypot(q.vx, q.vy)])
        elif tp == "/tracked_pose":
            th = 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w); c, s = math.cos(th), math.sin(th)
            S.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x - (c * OFF[0] - s * OFF[1]), m.pose.position.y - (s * OFF[0] + c * OFF[1])])
        elif tp == "/wheel_states": W.append([tr, *m.velocity])
        elif m.mode == 0: U.append([tr, *m.torque])
    R, S, U, W = map(np.array, (R, S, U, W)); R = R[np.argsort(R[:, 0])]
    mov = R[R[:, 4] > 1e-3]; t0 = mov[0, 0]; t1 = mov[-1, 0]
    vb = (P @ W[:, 1:5].T).T; ve = np.hypot(vb[:, 0], vb[:, 1])
    Sm = S[(S[:, 0] >= t0) & (S[:, 0] <= t1)]
    ex, ey = Sm[:, 1] - np.interp(Sm[:, 0], R[:, 0], R[:, 1]), Sm[:, 2] - np.interp(Sm[:, 0], R[:, 0], R[:, 2])
    yw = np.interp(Sm[:, 0], R[:, 0], np.unwrap(R[:, 3])); ea = 100 * (np.cos(yw) * ex + np.sin(yw) * ey)
    l, = ax[0].plot(W[:, 0] - t0, np.convolve(ve, np.ones(10) / 10, "same"), lw=0.8, label=f"{path.split('/')[-1]} {lab} encoder |v|")
    ax[0].plot(R[:, 0] - t0, R[:, 4], color=l.get_color(), ls="--", lw=1)
    ax[1].plot(Sm[:, 0] - t0, ea, color=l.get_color(), lw=1, label=lab)
    Um = U[(U[:, 0] >= t0 - 1) & (U[:, 0] <= t1 + 1)]
    ax[2].plot(Um[:, 0] - t0, np.abs(Um[:, 2]) / TB[1], color=l.get_color(), lw=0.6, label=f"{lab} |τ*_FR|/τ̄_FR")
ax[0].set_ylabel("speed [m/s]"); ax[0].legend(fontsize=8); ax[0].set_ylim(0, 1.4)
ax[1].axhline(0, color="k", lw=0.5); ax[1].set_ylabel("along-track err [cm]\n(SLAM, + ahead)"); ax[1].legend(fontsize=8)
ax[2].set_ylabel("FR |τ*| / τ̄ (0.0721)"); ax[2].set_xlabel("time since reference start [s]"); ax[2].legend(fontsize=8); ax[2].set_ylim(0, 1.1)
ax[0].set_xlim(-0.5, 13.5)
fig.tight_layout(); fig.savefig(sys.argv[-1], dpi=110)

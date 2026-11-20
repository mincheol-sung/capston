"""PP(130656) vs MPC friction(131757) vs MPC motor_only(131217): 위치 오차, F_x·M_z 명령, 포화 — 시간축 그림. 2026-11-17"""
import sys, math, numpy as np, rosbag2_py, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1897 + 0.207) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
NET = ["/torque_mpc/cmd_raw", "/wheel_pi/cmd_raw"]; REF = "/dra_mppi/reference_trajectory"; ODO = "/state_estimator/odom"
runs = [("130656", "PP (pure pursuit + wheel PI)"), ("131757", "MPC friction μ̂0.6 (τ̄ 0.152)"), ("131217", "MPC motor_only (τ̄ 1.2)")]
fig, ax = plt.subplots(3, 3, figsize=(15, 8), sharex=True)
for c, (b, name) in enumerate(runs):
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri="bags/2026-11-17/" + b, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=[x for x in NET + [REF, ODO] if x in types]))
    R, N, O = [], {n: [] for n in NET}, []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == REF and m.references:
            q = m.references[0]; R.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + m.dt, q.x, q.y, q.yaw, q.vx, q.yaw_rate])
        elif tp in NET and m.mode == 0: N[tp].append([tr, *m.torque])
        elif tp == ODO: O.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y])
    R = np.array(R); R = R[np.argsort(R[:, 0])]; O = np.array(O); nt = max(NET, key=lambda n: len(N[n])); Nt = np.array(N[nt])
    mov = R[np.abs(R[:, 4]) > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
    Os = O[(O[:, 0] >= t0) & (O[:, 0] <= t1)]
    e = 100 * np.hypot(Os[:, 1] - np.interp(Os[:, 0], R[:, 0], R[:, 1]), Os[:, 2] - np.interp(Os[:, 0], R[:, 0], R[:, 2]))
    Nm = Nt[(Nt[:, 0] >= t0) & (Nt[:, 0] <= t1)]; F = Nm[:, 1:] @ Tinv; amax = np.abs(Nm[:, 1:]).max(1)
    w = R[:, 5]; flips = R[1:, 0][(np.sign(w[1:]) != np.sign(w[:-1])) & (np.abs(w[1:]) > 0.2) & (np.abs(w[:-1]) > 0.2)]
    ax[0, c].plot(Os[:, 0] - t0, e, lw=0.8); ax[0, c].set_title(f"{b} {name}", fontsize=9); ax[0, c].set_ylabel("pos error [cm]" if c == 0 else "")
    ax[1, c].plot(Nm[:, 0] - t0, F[:, 0], lw=0.6, label="F_x [N]"); ax[1, c].plot(Nm[:, 0] - t0, F[:, 2] * 10, lw=0.6, label="M_z×10 [N·m]")
    ax[1, c].legend(fontsize=7, loc="upper right")
    ax[2, c].plot(Nm[:, 0] - t0, amax, lw=0.6); ax[2, c].axhline(0.1518, color="r", ls="--", lw=0.8); ax[2, c].set_ylabel("max|τ_i| [N·m]" if c == 0 else "")
    ax[2, c].set_xlabel("time since start [s]")
    for f in flips:
        for a in ax[:, c]: a.axvline(f - t0, color="gray", lw=0.6, ls=":")
    near = np.zeros(len(Nm), bool)
    for f in flips: near |= np.abs(Nm[:, 0] - f) < 0.6
    print(f"{b}: 포화(>0.1518) 시간 — 교차점 ±0.6 s {100*np.mean(amax[near] > 0.1518):.0f}%, 나머지 {100*np.mean(amax[~near] > 0.1518):.0f}%  (교차점 {len(flips)} 개)")
for a in ax[1, 1:]: a.set_ylim(-15, 15)
ax[1, 0].set_ylim(-15, 15); ax[2, 2].set_yscale("log")
fig.tight_layout(); fig.savefig(sys.argv[1], dpi=110)

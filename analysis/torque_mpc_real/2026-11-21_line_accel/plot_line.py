# 직선 급가속 시계열: 기준 속도 · 바퀴 기구학 속도 r·mean(ω) · IMU a_x 적분 속도 · SLAM 차분 속도 / IMU a_x / 바퀴 토크
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
out = sys.argv[1]; bags = sys.argv[2:]; r = 0.04
fig, ax = plt.subplots(3, len(bags), figsize=(5 * len(bags), 10), squeeze=False)
for j, b in enumerate(bags):
    raw = read("bags/2026-11-21/" + b, ("/imu/base_link", "/wheel_states", "/mpc/armed", "/dra_mppi/reference_trajectory", "/tracked_pose", "/state_estimator/odom"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data)
    R = np.array([[t, np.hypot(r_.vx, r_.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if t >= s0 and len(m.references) for r_ in [m.references[0]]])
    t_go = R[np.argmax(R[:, 1] > 0.01), 0]; T = lambda t: t - t_go
    I = np.array([[t, m.linear_acceleration.x] for t, m in raw["/imu/base_link"]])
    bias = I[(I[:, 0] > t_go - 1.2) & (I[:, 0] < t_go - 0.2), 1].mean(); a = np.convolve(I[:, 1] - bias, np.ones(5) / 5, "same")
    k = (I[:, 0] >= t_go - 0.05); vI = np.concatenate([[0], np.cumsum(0.5 * (a[k][1:] + a[k][:-1]) * np.diff(I[k, 0]))])
    W = np.array([[t, *m.velocity, *m.torque] for t, m in raw["/wheel_states"]])
    P = np.array([[t, m.pose.position.x, m.pose.position.y] for t, m in raw["/tracked_pose"]])
    vS = np.hypot(np.gradient(P[:, 1], P[:, 0]), np.gradient(P[:, 2], P[:, 0]))
    O = np.array([[t, m.twist.twist.linear.x] for t, m in raw["/state_estimator/odom"]])
    A = ax[0, j]; A.plot(T(R[:, 0]), R[:, 1], "k", label="기준"); A.plot(T(W[:, 0]), r * W[:, 1:5].mean(1), label="바퀴 r·mean(ω)")
    A.plot(T(I[k, 0]), vI, label="IMU 적분"); A.plot(T(O[:, 0]), O[:, 1], label="추정기 v_x"); A.plot(T(P[:, 0]), vS, ".", ms=3, label="SLAM 차분 (tracking frame)")
    A.set_xlim(-0.3, 2.0); A.set_ylim(-0.2, 1.6); A.set_title(f"{b} 속도 [m/s]"); A.legend(fontsize=7); A.grid(alpha=.3)
    A = ax[1, j]; A.plot(T(I[:, 0]), a); A.set_xlim(-0.3, 2.0); A.set_title("IMU a_x [m/s²] (바이어스 뺌)"); A.grid(alpha=.3)
    A = ax[2, j]
    for i, n in enumerate(("FL", "FR", "RL", "RR")): A.plot(T(W[:, 0]), W[:, 5 + i], label=n)
    A.set_xlim(-0.3, 2.0); A.set_title("바퀴 토크 측정 [N·m]"); A.legend(fontsize=7); A.grid(alpha=.3); A.set_xlabel("출발 뒤 [s]")
plt.rcParams["font.family"] = "DejaVu Sans"
fig.tight_layout(); fig.savefig(out, dpi=95); print(out)

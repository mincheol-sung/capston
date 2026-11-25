# A·C 3 판씩 지나간 궤적 (추정기 = RViz /torque_viz/trail 과 같은 자세, SLAM 점선) 을 /figure8/path 위에. 무장 ~ 해제 전체 (정렬 회전·정지 포함)
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
out = sys.argv[1]; groups = {"A (PP)": sys.argv[2].split(","), "C (MPC, d̂ on, μ̂×2)": sys.argv[3].split(",")}
plt.rcParams["font.family"] = ["Noto Sans CJK TC", "DejaVu Sans"]
fig, ax = plt.subplots(2, 2, figsize=(14, 11))
for col, (name, bags) in enumerate(groups.items()):
    for i, b in enumerate(bags):
        raw = read("bags/2026-11-21/" + b, ("/state_estimator/odom", "/figure8/path", "/mpc/armed", "/tracked_pose", "/dra_mppi/reference_trajectory"))
        s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
        R = [t for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1]; s1 = min(s1, R[-1] + 2.0)
        path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
        O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1])
        S = np.array([[t, m.pose.position.x, m.pose.position.y] for t, m in raw["/tracked_pose"] if s0 <= t <= s1])
        # 경로 좌표로 옮겨 출발점 = 원점 (판마다 출발 자리가 달라도 겹쳐 보이게)
        c0 = path[0]; d = path[min(20, len(path) - 1)] - c0; a = -np.arctan2(d[1], d[0]); Rm = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        P = (path - c0) @ Rm.T; Oq = (O[:, 1:] - c0) @ Rm.T; Sq = (S[:, 1:] - c0) @ Rm.T
        clr = ["C0", "C1", "C2"][i]
        for r, zoom in ((0, False), (1, True)):
            A = ax[r, col]
            if i == 0: A.plot(P[:, 0], P[:, 1], "k-", lw=1.0, label="기준 경로")
            A.plot(Oq[:, 0], Oq[:, 1], "-", color=clr, lw=1.2, label=f"{b} 추정기 (RViz)")
            A.plot(Sq[:, 0], Sq[:, 1], ":", color=clr, lw=1.0, label=f"{b} SLAM")
            A.set_aspect("equal"); A.grid(alpha=0.3)
            if zoom: A.set_xlim(0.15, 0.62); A.set_ylim(-0.32, 0.32)
    ax[0, col].set_title(name); ax[0, col].legend(fontsize=7, loc="upper left")
ax[1, 0].set_title("확대: 오른쪽 원 바깥 끝"); ax[1, 1].set_title("확대: 오른쪽 원 바깥 끝")
fig.tight_layout(); fig.savefig(out, dpi=110); print(out)

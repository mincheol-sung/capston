"""2026-11-23 실차 첫 Np 75 · Nc 40 판 그림: (왼) P1 8자 궤적 — 11-22 C (Np 50 · Nc 5) 3 판 vs 114314, (가운데) 경로 거리 시계열,
(오른) 정사각형 114214 vs 11-21 C3a 201411. 위치 = SLAM /tracked_pose 를 로봇 중심으로 (compare_slam.imu_to_center).
실행 (작업공간 루트): source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-23_horizon_real/plot.py
"""
import importlib.util, os, numpy as np, yaml, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
WS = "/home/smoc/capston/capston"; HERE = os.path.dirname(os.path.abspath(__file__)); MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False, "axes.grid": True, "grid.color": "#e6e5e0"})
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]; off = (float(est["imu_offset_x"]), float(est["imu_offset_y"]))
exec(open(f"{WS}/amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])


def p1(b):
    raw = read(f"{WS}/bags/{b}", ("/figure8/path", "/mpc/armed", "/dra_mppi/reference_trajectory", "/tracked_pose"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
    R = np.array([[t, np.hypot(r.vx, r.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    S = np.array([[t, m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for t, m in raw["/tracked_pose"] if s0 <= t <= s1])
    S = S[(np.interp(S[:, 0], R[:, 0], R[:, 1]) > 0.05) & (S[:, 0] >= R[0, 0]) & (S[:, 0] <= R[-1, 0])]
    xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off)
    d = np.sqrt(((np.c_[xc, yc][:, None, :] - path[None]) ** 2).sum(-1)).min(1)
    c0 = path[0]; v = path[5] - path[0]; a0 = np.arctan2(v[1], v[0]); Rm = np.array([[np.cos(a0), np.sin(a0)], [-np.sin(a0), np.cos(a0)]])
    return (path - c0) @ Rm.T, (np.c_[xc, yc] - c0) @ Rm.T, S[:, 0] - S[0, 0], d


fig, ax = plt.subplots(1, 3, figsize=(17, 6.2), gridspec_kw={"width_ratios": [1, 1.25, 1]})
old = ["2026-11-22/002707", "2026-11-22/002931", "2026-11-22/003146"]
for i, b in enumerate(old + ["2026-11-23/114314"]):
    P, Q, t, d = p1(b); new = i == 3
    if i == 0:
        ax[0].plot(P[:, 0], P[:, 1], "k-", lw=1, label="기준 경로")
    kw = dict(color="#eb6834", lw=1.6, zorder=3) if new else dict(color="#9a9993", lw=0.9)
    lab = f"{b[-6:]} Np 75 · Nc 40" if new else f"{b[-6:]} Np 50 · Nc 5"
    ax[0].plot(Q[:, 0], Q[:, 1], label=lab, **kw)
    ax[1].plot(t, 100 * d, label=f"{lab}  RMS {100*np.sqrt((d**2).mean()):.1f} cm", **kw)
ax[0].set_aspect("equal"); ax[0].legend(fontsize=8, loc="upper right"); ax[0].set_title("P1 8자 (C 설정), SLAM 중심")
ax[0].set_xlabel("출발 방향 [m]"); ax[0].set_ylabel("왼쪽 [m]")
ax[1].set_title("P1 경로 거리 (SLAM 중심)"); ax[1].set_xlabel("주행 시간 [s]"); ax[1].set_ylabel("경로까지 거리 [cm]"); ax[1].legend(fontsize=8)

CA = f"{WS}/analysis/torque_mpc_real/2026-11-21_square/compare_all.py"; src = open(CA).read()
g = {"__file__": CA, "__name__": "ca_head"}; exec(src[:src.index("rows = []; TRAJ = {}")], g)
sq = [("2026-11-21/201411", g["TB"], "#9a9993", "11-21 C3a 201411 (Np 50 · Nc 5, 옛 C)"),
      ("2026-11-23/114214", np.array([0.1445, 0.1184, 0.1444, 0.1859]), "#eb6834", "11-23 114214 (Np 75 · Nc 40, 새 C)")]
ax[2].plot([0, 1.5, 1.5, 0, 0], [0, 0, 1.5, 1.5, 0], "k-", lw=1, label="기준 정사각형")
for b, tb, c, lab in sq:
    r = g["run"](b, tb); ax[2].plot(r["lx"][r["win"]], r["ly"][r["win"]], color=c, lw=1.5, label=lab)
ax[2].set_aspect("equal"); ax[2].legend(fontsize=8, loc="center"); ax[2].set_title("정사각형 1.5 m (v_max 1.5, 가속 1.5), SLAM 중심")
ax[2].set_xlabel("x [m]"); ax[2].set_ylabel("y [m]")
fig.tight_layout(); out = f"{HERE}/fig_horizon_real.png"; fig.savefig(out, dpi=100); print(out)

#!/usr/bin/python3
"""실차 정사각형 (2026-11-20 17:14~17:18) — 퓨어 퍼슛 vs MPC C. tools/compare_slam.py 의 load() 로 같은 토픽·같은 잣대.
   위치 = /tracked_pose (IMU 점 → 중심), 기준 = /dra_mppi/reference_trajectory, 정사각형 로컬 좌표 = 주행 시작 순간의 기준 포즈.
   슬립 = 바퀴 기구학 몸체 속도 T ω − SLAM 중심 속도 (compare_slam 과 같은 식). 바닥은 SLAM 차분 잡음 ≈ 0.10 m/s.
   source /opt/ros/jazzy/setup.bash && source install/setup.bash && /usr/bin/python3 plot_real_square.py"""
import importlib.util, json, math, os, sys
import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
WS = "/home/smoc/capston/capston"
MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py")
cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
BAGS = [("171406", "퓨어 퍼슛, 가속 1.0 · 0.8 m/s", "#1baf7a", "--"),
        ("171640", "퓨어 퍼슛, 가속 1.5 · 1.5 m/s", "#1baf7a", "-"),
        ("171531", "MPC C, 가속 1.5 · 1.5 m/s (1)", "#eb6834", "-"),
        ("171735", "MPC C, 가속 1.5 · 1.5 m/s (2)", "#6250d6", "-")]
SIDE = 1.5
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2, REF = "#0b0b0b", "#52514e", "#a9a8a2"
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]
plant = yaml.safe_load(open(f"{MPC}/config/torque_plant.yaml"))
off = (float(est["imu_offset_x"]), float(est["imu_offset_y"]))
r_w = plant["geometry"]["wheel_radius_m"]


def contour(px, py):
    def seg(ax, ay, bx, by):
        ux, uy = bx - ax, by - ay
        t = np.clip(((px - ax) * ux + (py - ay) * uy) / (ux * ux + uy * uy), 0, 1)
        return np.hypot(px - ax - t * ux, py - ay - t * uy)
    P = [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]
    return np.min([seg(*P[i], *P[i + 1]) for i in range(4)], axis=0)


def run(bag):
    D = cs.load(f"{WS}/bags/2026-11-20/{bag}")
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], off)
    rt = np.array([a for a, _ in D["ref"]])
    rows = []
    for t, x, y, th in zip(P[:, 0], xc, yc, P[:, 3]):
        i = np.searchsorted(rt, t, side="right") - 1
        if i >= 0:
            Xr, ur = D["ref"][i][1].at(t); rows.append([t, x, y, th, *Xr, *ur])
    a = np.array(rows); t = a[:, 0]
    sp = np.hypot(a[:, 7], a[:, 8]); arm = cs.armed_mask(t, D["armed"]); mv = arm & (sp > 0.02)
    i0 = np.flatnonzero(mv)[0]; o = a[i0, 4:7]                      # 주행 시작 순간의 기준 포즈 = 출발 꼭짓점
    c, s = math.cos(o[2]), math.sin(o[2])
    loc = lambda X, Y: (c * (X - o[0]) + s * (Y - o[1]), -s * (X - o[0]) + c * (Y - o[1]))
    lx, ly = loc(a[:, 1], a[:, 2]); rx, ry = loc(a[:, 4], a[:, 5])
    win = arm & (t >= t[i0] - 0.5) & (t <= t[np.flatnonzero(mv)[-1]] + 2.0)
    d = contour(lx, ly)
    # 슬립 (compare_slam 과 같은 식)
    W = np.array(D["wheel"]); q = r_w / 4.0
    wv = np.stack([q * W[:, 1:5].sum(1), q * (-W[:, 1] + W[:, 2] + W[:, 3] - W[:, 4])], 1)
    ts, h = P[:, 0], 0.1
    sv = [(np.interp(ts + h, ts, arr) - np.interp(ts - h, ts, arr)) / (2 * h) for arr in (xc, yc)]
    cc, ss = np.cos(P[:, 3]), np.sin(P[:, 3])
    bx, by = cc * sv[0] + ss * sv[1], -ss * sv[0] + cc * sv[1]
    slip = np.hypot(np.interp(ts, W[:, 0], wv[:, 0]) - bx, np.interp(ts, W[:, 0], wv[:, 1]) - by)
    T = np.array(D["tau"]); T = T[(T[:, 1] != 1)]
    return dict(t=t - t[i0], lx=lx, ly=ly, rx=rx, ry=ry, mv=mv, win=win, contour=d, speed=np.hypot(bx, by), ts=ts - t[i0],
                slip=slip, tt=T[:, 0] - t[i0], tau=np.abs(T[:, 2:6]).max(axis=1), cs=cs.analyse(f"{WS}/bags/2026-11-20/{bag}", off, r_w, 0.1932))


R = {b: run(b) for b, *_ in BAGS}

# 그림 1: 궤적
fig, axes = plt.subplots(1, 2, figsize=(13, 6.6))
for ax, sel, title in ((axes[0], ["171640", "171531", "171735"], "같은 조건: 가속 1.5 m/s² · 최고 1.5 m/s"),
                       (axes[1], ["171406"], "퓨어 퍼슛, 가속 1.0 m/s² · 최고 0.8 m/s")):
    ax.plot([0, SIDE, SIDE, 0, 0], [0, 0, SIDE, SIDE, 0], color=REF, lw=7, alpha=0.45, solid_capstyle="round")
    for b, name, col, ls in BAGS:
        if b in sel:
            r = R[b]; w = r["win"]
            ax.plot(r["lx"][w], r["ly"][w], color=col, lw=1.9, ls="-", label=f"{name}: 윤곽 {1e3*np.sqrt(np.mean(r['contour'][r['mv']]**2)):.0f} mm")
    ax.set_aspect("equal"); ax.set_xlim(-0.45, 2.0); ax.set_ylim(-0.45, 2.0)
    ax.set_title(title, loc="left", color=INK); ax.legend(loc="center", frameon=False, fontsize=9)
    ax.set_xlabel("출발 꼭짓점 기준 x [m] (첫 변 방향)"); ax.set_ylabel("y [m]")
fig.suptitle("실차 정사각형 1.5 m · tangent · 꼭짓점 0.1 m/s — SLAM 포즈(로봇 중심) 궤적, 2026-11-20", x=0.01, ha="left", color=INK, fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(os.path.join(HERE, "fig_real_traj.png"), dpi=130); plt.close(fig)

# 그림 2: 시계열 (가속 1.5 판) — 속도 / 윤곽 오차 / 슬립 / 최대 바퀴 토크
fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
for b, name, col, ls in BAGS[1:]:
    r = R[b]; w = (r["t"] > -0.5) & (r["t"] < 9.5); ww = (r["ts"] > -0.5) & (r["ts"] < 9.5); wt = (r["tt"] > -0.5) & (r["tt"] < 9.5)
    ax[0].plot(r["ts"][ww], r["speed"][ww], color=col, lw=1.6, label=name)
    ax[1].plot(r["t"][w], 1e3 * r["contour"][w], color=col, lw=1.6)
    ax[2].plot(r["ts"][ww], r["slip"][ww], color=col, lw=1.4)
    ax[3].plot(r["tt"][wt], r["tau"][wt], color=col, lw=1.2)
r = R["171531"]; w = (r["t"] > -0.5) & (r["t"] < 9.5)
ax[0].plot(r["t"][w], np.hypot(np.gradient(r["rx"][w], r["t"][w]), np.gradient(r["ry"][w], r["t"][w])), color=INK2, lw=1, ls="--", label="기준 속도")
ax[0].set_ylabel("속도 [m/s]\n(SLAM 차분)"); ax[0].legend(loc="upper right", frameon=False, fontsize=8.5, ncol=2)
ax[1].set_ylabel("윤곽 오차 [mm]\n(정사각형 변까지)")
ax[2].set_ylabel("슬립 |T ω − v_SLAM| [m/s]"); ax[2].axhline(0.10, color=INK2, lw=0.9, ls=":")
ax[2].text(8.0, 0.115, "SLAM 차분 잡음 바닥 ≈ 0.10", color=INK2, fontsize=8.5)
ax[3].set_ylabel("최대 바퀴 토크 [N·m]\n(최종 명령, 마찰 보상 포함)"); ax[3].set_xlabel("주행 시작 뒤 시간 [s]")
ax[0].set_title("가속 1.5 m/s² · 최고 1.5 m/s 판 — 퓨어 퍼슛 1 판, MPC C 2 판", loc="left", color=INK)
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_real_timeseries.png"), dpi=130); plt.close(fig)

# 표
summ = {}
print("| 판 | 윤곽 RMS / 최대 [mm] | 시간 추종 RMS [mm] | e_t 평균 (− 뒤처짐) | e_c RMS | 요 RMS ° | 슬립 RMS x·y [m/s] | 슬립 >0.3 시간 % | τ max | Δτ RMS |")
for b, name, *_ in BAGS:
    r = R[b]; c2 = r["cs"]; mvts = (r["ts"] >= 0) & (r["ts"] <= c2["dur"])
    summ[b] = dict(name=name, contour=1e3 * float(np.sqrt(np.mean(r["contour"][r["mv"]] ** 2))), contour_max=1e3 * float(r["contour"][r["mv"]].max()),
                   slip03=100 * float(np.mean(r["slip"][mvts] > 0.3)), **c2)
    s = summ[b]
    print(f"| {name} | {s['contour']:.0f} / {s['contour_max']:.0f} | {1e3*s['rms']:.0f} | {1e3*s['et_mean']:+.0f} | {1e3*s['ec']:.0f} | {s['yaw']:.1f} | "
          f"{s['slip_x']:.3f} · {s['slip_y']:.3f} | {s['slip03']:.1f} | {s['tau_max']:.2f} | {s['dtau']:.4f} |")
json.dump(summ, open(os.path.join(HERE, "summary.json"), "w"), ensure_ascii=False, indent=1)

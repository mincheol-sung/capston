#!/usr/bin/python3
"""실차: 속도 vs 경로 위치 (정사각형 둘레를 따라 잰 거리 s, 꼭짓점 1.5·3.0·4.5 m). 기준 속도 곡선도 같은 축에."""
import importlib.util, json, math, os
import numpy as np, yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]; off = (est["imu_offset_x"], est["imu_offset_y"])
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False, "axes.edgecolor": "#b8b7b0",
                     "axes.labelcolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e6e5e0", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
SIDE = 1.5


def arc(px, py):
    """정사각형 둘레 위 가장 가까운 점의 둘레 거리 s (0 출발 꼭짓점, 반시계)."""
    d = np.stack([np.abs(py), np.abs(px - SIDE), np.abs(py - SIDE), np.abs(px)], 1); e = np.argmin(d, 1)
    a = np.clip(np.choose(e, [px, py, SIDE - px, SIDE - py]), 0, SIDE)
    return e * SIDE + a


def load(bag):
    D = cs.load(f"{WS}/bags/2026-11-20/{bag}")
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], off); ts = P[:, 0]
    rt = np.array([a for a, _ in D["ref"]]); R = []
    for t in ts:
        i = np.searchsorted(rt, t, side="right") - 1
        R.append(np.concatenate(D["ref"][i][1].at(t)) if i >= 0 else np.full(6, np.nan))
    R = np.array(R); arm = cs.armed_mask(ts, D["armed"]); vref = np.hypot(R[:, 3], R[:, 4]); mv = arm & (vref > 0.02)
    i0 = np.flatnonzero(mv)[0]; o = R[i0, :3]; c, s = math.cos(o[2]), math.sin(o[2])
    L = lambda X, Y: (c * (X - o[0]) + s * (Y - o[1]), -s * (X - o[0]) + c * (Y - o[1]))
    lx, ly = L(xc, yc); rx, ry = L(R[:, 0], R[:, 1]); h = 0.1
    v = np.hypot((np.interp(ts + h, ts, xc) - np.interp(ts - h, ts, xc)) / (2 * h), (np.interp(ts + h, ts, yc) - np.interp(ts - h, ts, yc)) / (2 * h))
    w = arm & (ts >= ts[i0]) & (ts <= ts[np.flatnonzero(mv)[-1]])
    # unwrap 은 무장 주행 구간만 (앞쪽 기준 없는 NaN 행이 들어가면 뒤가 모두 NaN 이 된다)
    s_m = np.unwrap(arc(lx[w], ly[w]) / (4 * SIDE) * 2 * np.pi) / (2 * np.pi) * 4 * SIDE
    s_r = np.unwrap(arc(rx[w], ry[w]) / (4 * SIDE) * 2 * np.pi) / (2 * np.pi) * 4 * SIDE
    return s_m - s_m[0], v[w], s_r - s_r[0], vref[w]


fig, ax = plt.subplots(figsize=(12, 4.8))
for k in (1, 2, 3):
    ax.axvline(k * SIDE, color="#a9a8a2", lw=0.9, ls=":")
    ax.text(k * SIDE + 0.03, 1.72, f"꼭짓점 {k}", color="#52514e", fontsize=8.5)
first = True
for bag, name, col in [("171640", "퓨어 퍼슛", "#1baf7a"), ("171531", "MPC C (1)", "#eb6834"), ("171735", "MPC C (2)", "#6250d6")]:
    sm, v, sr, vr = load(bag)
    if first:
        o = np.argsort(sr); ax.plot(sr[o], vr[o], color="#0b0b0b", lw=2.2, ls="--", label="기준 (변 가운데 1.5 m/s)"); first = False
    ax.plot(sm, v, color=col, lw=1.8, label=name)
ax.set_xlim(0, 4 * SIDE); ax.set_ylim(0, 1.85)
ax.set_xlabel("정사각형 둘레를 따라 잰 위치 s [m] (가장 가까운 변 위로 투영)"); ax.set_ylabel("속도 [m/s] (SLAM 차분)")
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), frameon=False, ncol=4, fontsize=9)
ax.set_title("실차 가속 1.5 m/s² · 최고 1.5 m/s — 어디서 몇 m/s 를 냈나", loc="left", color="#0b0b0b")
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_real_speed_vs_s.png"), dpi=130); print("saved")

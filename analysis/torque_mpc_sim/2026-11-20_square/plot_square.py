#!/usr/bin/python3
"""2026-11-20 정사각형 시뮬 비교 그림 (MPC B·C, 퓨어 퍼슛). 입력: runs/*.npz·*.json (square_rec.py 가 남긴 것).
  /usr/bin/python3 plot_square.py   → fig_trajectory.png, fig_timeseries_a1.0.png, fig_summary.png"""
import json, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2, REF = "#0b0b0b", "#52514e", "#a9a8a2"
SER = {"mpcB": ("MPC (B, 모터 상한)", "#2a78d6"), "mpcC": ("MPC (C, 마찰 상한)", "#eb6834"),
       "pp": ("퓨어 퍼슛 + 바퀴 PI", "#1baf7a")}
SIDE = 1.5
TAU_C_FR = 0.0721          # C 상한 최소 바퀴 (FR) [N·m] — torque_plant.yaml mu_hat_wheels 로 계산


def load(label):
    z = np.load(os.path.join(RUNS, f"{label}.npz"))
    s = json.loads(open(os.path.join(RUNS, f"{label}.json")).read().strip().splitlines()[-1])
    return z["raw"], z["tau"], z["tau_t"], s


def square_xy():
    return np.array([0, SIDE, SIDE, 0, 0]), np.array([0, 0, SIDE, SIDE, 0])


# ── 그림 1: 궤적 (전체 + 꼭짓점 확대)
fig, axes = plt.subplots(2, 2, figsize=(11, 10), gridspec_kw={"width_ratios": [1.15, 1]})
for row, (a, vp) in enumerate([("0.4", 0.78), ("1.0", 1.23)]):
    full, zoom = axes[row]
    for ax in (full, zoom):
        sx, sy = square_xy()
        ax.plot(sx, sy, color=REF, lw=6, alpha=0.45, solid_capstyle="round", zorder=1)
    for key in ("mpcB", "mpcC", "pp"):
        raw, _, _, s = load(f"{key}_a{a}")
        name, col = SER[key]
        for ax in (full, zoom):
            ax.plot(raw[:, 1], raw[:, 2], color=col, lw=2, zorder=3)
        zoom.plot([], [], color=col, lw=2, label=f"{name}  통과 {s['corner_pass_mm'][1]:.0f} mm")
    full.set_title(f"가감속 {a} m/s² · 변 가운데 최고 {vp} m/s", color=INK, loc="left", fontsize=11)
    full.set_aspect("equal"); full.set_xlim(-0.25, 1.75); full.set_ylim(-0.25, 1.75)
    full.annotate("출발", (0, 0), (0.12, -0.17), color=INK2, fontsize=9, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.6))
    full.add_patch(plt.Rectangle((SIDE - 0.25, SIDE - 0.25), 0.4, 0.4, fill=False, ec=INK2, lw=0.8, ls="--", zorder=4))
    zoom.set_aspect("equal"); zoom.set_xlim(SIDE - 0.25, SIDE + 0.15); zoom.set_ylim(SIDE - 0.25, SIDE + 0.15)
    zoom.plot([SIDE], [SIDE], marker="o", ms=8, mfc="#fcfcfb", mec=INK, mew=1.5, zorder=5)
    zoom.set_title("꼭짓점 (1.5, 1.5) 확대 — 점선 상자", color=INK, loc="left", fontsize=11)
    zoom.legend(loc="lower left", frameon=False, fontsize=9)
    for ax in (full, zoom):
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
fig.suptitle("정사각형 1.5 m · tangent 헤딩 · 꼭짓점 0.1 m/s — 시뮬 참값 궤적 (추정기 상태로 제어)", color=INK, x=0.01,
             ha="left", fontsize=13)
fig.tight_layout(rect=(0, 0, 1, 0.97))
fig.savefig(os.path.join(HERE, "fig_trajectory.png"), dpi=130)
plt.close(fig)

# ── 그림 2: a 1.0 시계열 (기준 속도 / 위치 오차 / 최대 바퀴 토크) — 축마다 한 가지 양
fig, (a0, a1, a2) = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True, gridspec_kw={"height_ratios": [1, 1.4, 1.4]})
corner_t = None
for key in ("mpcB", "mpcC", "pp"):
    raw, tau, tau_t, s = load(f"{key}_a1.0")
    name, col = SER[key]
    vr = np.hypot(raw[:, 7], raw[:, 8])
    t0 = raw[np.argmax(vr > 0.02), 0]
    t = raw[:, 0] - t0
    e = np.hypot(raw[:, 1] - raw[:, 4], raw[:, 2] - raw[:, 5])
    m = (t > -0.5) & (t < 12.5)
    if corner_t is None:
        a0.plot(t[m], vr[m], color=INK2, lw=2)
        # 꼭짓점 = 기준 속도 극소 (0.1 m/s) 시각
        mid = (t > 0.5) & (t < 9.0)
        idx = np.where(mid[1:-1] & (vr[1:-1] <= vr[:-2]) & (vr[1:-1] <= vr[2:]) & (vr[1:-1] < 0.2))[0] + 1
        corner_t = []
        for i in idx:
            if not corner_t or t[i] - corner_t[-1] > 1.0:
                corner_t.append(t[i])
    a1.plot(t[m], 1e3 * e[m], color=col, lw=2, label=name)
    tt = tau_t - t0
    mt = (tt > -0.5) & (tt < 12.5)
    a2.plot(tt[mt], np.abs(tau[mt]).max(axis=1), color=col, lw=1.6, label=name)
for ax in (a0, a1, a2):
    for c in corner_t:
        ax.axvline(c, color=REF, lw=0.9, ls=":")
a0.set_ylabel("기준 속도 [m/s]"); a0.set_title("가감속 1.0 m/s² 판 — 점선 = 꼭짓점 통과 시각", color=INK, loc="left")
a0.text(corner_t[0] + 0.1, 1.15, "꼭짓점", color=INK2, fontsize=9)
a1.set_ylabel("위치 오차 |p − p_ref| [mm]"); a1.set_ylim(0, 245); a1.legend(loc="upper left", frameon=False, ncol=3)
a2.set_ylabel("최대 바퀴 토크 [N·m]\n(최종 명령, 마찰 보상 포함)"); a2.set_xlabel("주행 시작 뒤 시간 [s]"); a2.set_ylim(0, 0.64)
a2.axhline(TAU_C_FR, color=INK2, lw=1, ls="--")
a2.text(11.45, TAU_C_FR + 0.012, f"C 순토크 상한\n최소 (FR) {TAU_C_FR}", color=INK2, fontsize=8.5, va="bottom")
a2.legend(loc="upper left", frameon=False, ncol=3)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig_timeseries_a1.0.png"), dpi=130)
plt.close(fig)

# ── 그림 3: 요약 (꼭짓점 통과 거리 평균 / 윤곽 오차 RMS) — 묶음 막대, 값 직접 표기
fig, (b0, b1) = plt.subplots(1, 2, figsize=(11, 4.2))
groups = ["0.4", "1.0"]
w = 0.26
for gi, a in enumerate(groups):
    for si, key in enumerate(("mpcB", "mpcC", "pp")):
        _, _, _, s = load(f"{key}_a{a}")
        name, col = SER[key]
        x = gi + (si - 1) * (w + 0.02)
        for ax, val in ((b0, float(np.mean(s["corner_pass_mm"]))), (b1, s["contour_rms_mm"])):
            ax.bar(x, val, width=w, color=col, label=name if gi == 0 else None, zorder=3)
            ax.text(x, val + 2, f"{val:.0f}", ha="center", va="bottom", color=INK, fontsize=9)
for ax, title in ((b0, "꼭짓점 통과 거리 (내부 꼭짓점 3 개 평균) [mm]"), (b1, "윤곽 오차 RMS (정사각형 변까지 거리, 주행 중) [mm]")):
    ax.set_xticks(range(len(groups)), [f"가감속 {g} m/s²" for g in groups])
    ax.set_title(title, color=INK, loc="left", fontsize=10.5)
    ax.grid(axis="x", visible=False)
    ax.set_ylim(0, ax.get_ylim()[1] * 1.12)
b0.legend(loc="upper left", frameon=False)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig_summary.png"), dpi=130)
plt.close(fig)
print("saved")

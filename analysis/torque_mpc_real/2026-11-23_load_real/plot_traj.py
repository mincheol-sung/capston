#!/usr/bin/python3
"""load_compare.npz → fig_traj.png: 각 판 기준 좌표 (출발 포즈) 로 옮긴 SLAM 로봇 중심 궤적 — 겹쳐 보기 + 판별 (큰 슬립 지점 표시)."""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
COL = {"C_off": "#eb6834", "C_on": "#1baf7a", "PP": "#2a78d6"}
NAME = {"C_off": "C 하중 이동 끔 (151038)", "C_on": "C 하중 이동 켬 h 0.2 (154431)", "PP": "A 퓨어 퍼슛 (151249)"}
INK, LIM = "#0b0b0b", "#d03b3b"
z = np.load("load_compare.npz"); R = json.load(open("load_compare.json"))


def local(lab, x, y):
    r0 = z[f"{lab}_ref"][0]; c0, s0 = np.cos(r0[2]), np.sin(r0[2]); dx, dy = x - r0[0], y - r0[1]
    return c0 * dx + s0 * dy, -s0 * dx + c0 * dy


fig, ax = plt.subplots(1, 4, figsize=(17, 5.2), gridspec_kw=dict(wspace=0.18))
ref = z["C_on_ref"]; rx, ry = local("C_on", ref[:, 0], ref[:, 1])
for a in ax:
    a.plot(rx, ry, color=INK, lw=0.9, ls="--", label="기준")
    a.set_aspect("equal"); a.set_xlim(-0.6, 0.6); a.set_ylim(-0.95, 0.95); a.set_xlabel("출발 헤딩 방향 [m]")
ax[0].set_ylabel("왼쪽 [m]")
for lab in ("C_off", "C_on", "PP"):
    lx, ly = local(lab, z[f"{lab}_x"], z[f"{lab}_y"])
    ax[0].plot(lx, ly, color=COL[lab], lw=1.2, label=NAME[lab])
ax[0].legend(loc="lower center", fontsize=7, frameon=False, bbox_to_anchor=(0.5, -0.02))
ax[0].set_title("(a) 겹쳐 보기", loc="left", fontsize=10, color=INK)
for k, lab in enumerate(("C_off", "C_on", "PP")):
    a = ax[k + 1]; r = R[lab]
    lx, ly = local(lab, z[f"{lab}_x"], z[f"{lab}_y"])
    a.plot(lx, ly, color=COL[lab], lw=1.4)
    mv = z[f"{lab}_moving"]; big = (np.abs(z[f"{lab}_lam"]).max(1) > 0.5) & mv
    a.plot(lx[big], ly[big], ".", color=LIM, ms=3.5, label="큰 슬립 (|λ| > 0.5)")
    a.set_title(f"({'bcd'[k]}) {NAME[lab]}\nRMS {r['e_rms']:.0f} · 최대 {r['e_max']:.0f} mm · 요 {r['h_rms']:.1f}° · 큰 슬립 {r['big']:.0f} %",
                loc="left", fontsize=9, color=INK)
    a.legend(loc="lower center", fontsize=7.5, frameon=False)
fig.suptitle("실차 2026-11-23 계단 변조 8자 (circles8 지름 0.8 m, 0.35 ↔ 1.04 m/s, accel 3.0, 2 바퀴) — SLAM 로봇 중심, 각 판 기준의 출발 포즈 좌표",
             x=0.04, y=1.03, ha="left", fontsize=11, color=INK)
fig.savefig("fig_traj.png", dpi=130, bbox_inches="tight"); print("saved")

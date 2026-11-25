#!/usr/bin/python3
"""step_compare.npz → fig_step_slip.png: 경로 (각 판 기준 좌표), 위치 오차, 바퀴별 큰 슬립 (헛돎/끌림)."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
COL = {"PP": "#2a78d6", "C": "#1baf7a"}; NAME = {"PP": "A 퓨어 퍼슛 (151249)", "C": "C MPC friction (151038)"}; INK, MUTED, LIM = "#0b0b0b", "#52514e", "#d03b3b"
z = np.load("step_compare.npz"); R = json.load(open("step_compare.json")); NM = ["FL", "FR", "RL", "RR"]
fig, ax = plt.subplots(1, 3, figsize=(15.5, 4.9), gridspec_kw=dict(width_ratios=[0.8, 1.3, 1.1], wspace=0.25))
a, b, c = ax
for ctl in ("PP", "C"):
    ref = z[f"{ctl}_ref"]; r0 = ref[0]; c0, s0 = np.cos(r0[2]), np.sin(r0[2])
    loc = lambda x, y: (c0 * (x - r0[0]) + s0 * (y - r0[1]), -s0 * (x - r0[0]) + c0 * (y - r0[1]))
    if ctl == "C":
        rx, ry = loc(ref[:, 0], ref[:, 1]); a.plot(rx, ry, color=INK, lw=0.9, ls="--", label="기준")
    lx, ly = loc(z[f"{ctl}_x"], z[f"{ctl}_y"]); a.plot(lx, ly, color=COL[ctl], lw=1.3, label=NAME[ctl])
    m = R[ctl]
    b.plot(z[f"{ctl}_t"], 1e3 * z[f"{ctl}_e"], color=COL[ctl], lw=1.2, label=f"{NAME[ctl]}  RMS {m['e_rms']:.0f} · 최대 {m['e_max']:.0f} mm · 요 RMS {m['h_rms']:.1f}°")
a.set_aspect("equal"); a.set_xlabel("출발 헤딩 방향 [m]"); a.set_ylabel("왼쪽 [m]"); a.legend(loc="lower center", fontsize=7.5, frameon=False)
a.set_title("(a) 경로 — 각 판 기준 좌표", loc="left", fontsize=10, color=INK)
b.set_xlabel("기준 출발부터 t [s]"); b.set_ylabel("위치 오차 [mm]"); b.legend(loc="upper left", fontsize=7.5, frameon=False)
b.set_title("(b) 시간 추종 오차 — circles8 지름 0.8 m, 0.35↔1.04 m/s 계단, accel 3.0", loc="left", fontsize=10, color=INK)
x = np.arange(4); wdt = 0.36
for j, ctl in enumerate(("C", "PP")):
    mv = z[f"{ctl}_moving"]; lam = z[f"{ctl}_lam"][mv]
    dr = [100 * np.mean(lam[:, i] > 0.5) for i in range(4)]; br = [100 * np.mean(lam[:, i] < -0.5) for i in range(4)]
    xx = x + (j - 0.5) * wdt
    c.bar(xx, br, width=wdt * 0.92, color=COL[ctl], alpha=0.45, edgecolor="#fcfcfb", linewidth=1)
    c.bar(xx, dr, width=wdt * 0.92, bottom=br, color=COL[ctl], edgecolor="#fcfcfb", linewidth=1, label=f"{'C' if ctl == 'C' else 'A PP'}  큰 슬립 {R[ctl]['big']:.0f} % (헛돎 {R[ctl]['drive']:.0f} · 끌림 {R[ctl]['brake']:.0f})")
c.set_xticks(x, NM); c.set_ylabel("|λ| > 0.5 시간 % (진한 = 헛돎, 옅은 = 끌림)"); c.legend(loc="upper right", fontsize=8, frameon=False)
c.set_title("(c) 바퀴별 큰 슬립 — C 의 차이는 FL 끌림 (토크 ≈ 0.04 N·m)", loc="left", fontsize=10, color=INK)
fig.suptitle("실차 2026-11-23 계단 변조 8자 — C 가 더 미끄러졌다 (25 % vs 17 %)", x=0.05, y=1.02, ha="left", fontsize=11.5, color=INK)
fig.savefig("fig_step_slip.png", dpi=130, bbox_inches="tight"); print("saved")

#!/usr/bin/python3
"""compare.npz → fig_max_compare.png (행: circles8 width 3.0 / square, 열: 경로 · 위치 오차 · 가장 큰 바퀴 슬립)."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
COL = {"PP": "#2a78d6", "C": "#1baf7a"}; NAME = {"PP": "A 퓨어 퍼슛", "C": "C MPC friction"}
INK, MUTED, LIM = "#0b0b0b", "#52514e", "#d03b3b"
z = np.load(os.path.join(HERE, "compare.npz")); R = json.load(open(os.path.join(HERE, "compare.json")))
TITLE = {"fig8": "circles8 폭 3.0 (지름 1.5 m), speed 0.80 · accel 1.40 → 0.40~1.20 m/s, 2 바퀴",
         "square": "정사각형 1.5 m, accel 1.70 · 변 최고 1.55 m/s, 꼭짓점 0.1"}
fig, ax = plt.subplots(2, 3, figsize=(15.5, 9.4), gridspec_kw=dict(width_ratios=[1.05, 1.25, 1.25], hspace=0.36, wspace=0.24))
for r, shape in enumerate(("fig8", "square")):
    a, b, c = ax[r]
    # 판마다 출발 위치가 달라 기준도 다른 곳에 있다 → 각 판을 자기 기준의 출발 포즈 (위치·헤딩) 좌표로 옮겨 겹친다
    def local(k, x, y):
        r0 = z[f"{k}_ref"][0]; c0, s0 = np.cos(r0[2]), np.sin(r0[2]); dx, dy = x - r0[0], y - r0[1]
        return c0 * dx + s0 * dy, -s0 * dx + c0 * dy
    ref = z[f"{shape}_C_ref"]; rx, ry = local(f"{shape}_C", ref[:, 0], ref[:, 1])
    a.plot(rx, ry, color=INK, lw=0.9, ls="--", label="기준 (두 판 같은 모양)")
    for ctl in ("PP", "C"):
        k = f"{shape}_{ctl}"; m = R[k]; lx, ly = local(k, z[f"{k}_x"], z[f"{k}_y"])
        a.plot(lx, ly, color=COL[ctl], lw=1.4, label=f"{NAME[ctl]} ({m['bag']})")
        b.plot(z[f"{k}_t"], 1e3 * z[f"{k}_e"], color=COL[ctl], lw=1.3,
               label=f"{NAME[ctl]}  RMS {m['e_rms']:.0f} · 최대 {m['e_max']:.0f} mm · 요 RMS {m['h_rms']:.1f}°")
        mv = z[f"{k}_moving"]; lam = np.where(mv, np.abs(z[f"{k}_lam"]).max(1), np.nan)
        c.plot(z[f"{k}_t"], lam, color=COL[ctl], lw=1.0,
               label=f"{NAME[ctl]}  큰 슬립 {m['big']:.1f} % (헛돎 {m['drive']:.1f} · 끌림 {m['brake']:.1f})")
    a.set_aspect("equal"); a.set_xlabel("출발 헤딩 방향 [m]"); a.set_ylabel("왼쪽 [m]"); a.legend(loc="best", fontsize=7.5, frameon=False)
    a.set_title(f"({'ab'[r]}1) {TITLE[shape]}\n경로 — SLAM 로봇 중심, 각 판 기준의 출발 포즈 좌표", loc="left", fontsize=9.5, color=INK)
    b.set_xlabel("기준 출발부터 t [s]"); b.set_ylabel("위치 오차 |p − p_r| [mm]"); b.legend(loc="upper left", fontsize=7.5, frameon=False)
    b.set_title(f"({'ab'[r]}2) 시간 추종 오차", loc="left", fontsize=10, color=INK)
    if shape == "square":
        corner = z["square_C_corner"]; t = z["square_C_t"]; on = None
        for i in range(len(t)):
            if corner[i] and on is None:
                on = t[i]
            if not corner[i] and on is not None:
                for A_ in (b, c):
                    A_.axvspan(on, t[i - 1], color=INK, alpha=0.07, lw=0)
                on = None
        b.text(0.02, 0.02, "회색 띠 = 꼭짓점 헤딩 회전 (마지막 = 끝 정렬)", transform=b.transAxes, fontsize=8, color=MUTED)
    c.axhline(0.5, color=LIM, lw=0.9, ls="--"); c.text(0.01, 0.53, "큰 슬립 |λ| 0.5", color=LIM, fontsize=8, transform=c.get_yaxis_transform())
    c.set_ylim(0, 2.1); c.set_xlabel("t [s]"); c.set_ylabel("max_i |λ_i| (SLAM 속도 + 자이로 기준)")
    c.legend(loc="upper right", fontsize=7.5, frameon=False)
    c.set_title(f"({'ab'[r]}3) 네 바퀴 중 가장 큰 슬립 (기준 이동 중)", loc="left", fontsize=10, color=INK)
fig.suptitle("실차 2026-11-23 speed:=max · accel:=max — A 퓨어 퍼슛 vs C MPC friction", x=0.05, y=0.985, ha="left", fontsize=11.5, color=INK)
fig.savefig(os.path.join(HERE, "fig_max_compare.png"), dpi=130, bbox_inches="tight")
print("saved")

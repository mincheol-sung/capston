#!/usr/bin/python3
"""why.npz → fig_why_pp_vs_mpc.png"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
CP, CM, INK, MUTED, LIM = "#2a78d6", "#1baf7a", "#0b0b0b", "#52514e", "#d03b3b"
z = np.load(os.path.join(HERE, "why.npz"))
P = {k[2:]: z[k] for k in z.files if k.startswith("P_")}; M = {k[2:]: z[k] for k in z.files if k.startswith("M_")}
TB = np.array([0.1230, 0.1253, 0.1298, 0.1187]); t_f = M["t"][-1]
fig, ax = plt.subplots(2, 3, figsize=(15, 8.8), gridspec_kw=dict(hspace=0.36, wspace=0.26))

a = ax[0, 0]
a.plot(P["ref_full"][:, 0], P["ref_full"][:, 1], color=INK, lw=0.9, ls="--", label="기준 (circles8, 지름 1 m × 2)")
a.plot(P["x"], P["y"], color=CP, lw=1.6, label="A 퓨어 퍼슛 (200720)")
a.plot(M["x"], M["y"], color=CM, lw=1.6, label="C MPC (200303)")
a.plot(M["x"][-1], M["y"][-1], "x", color=LIM, ms=9, mew=2); a.text(M["x"][-1] - 0.1, M["y"][-1] - 0.17, "C fault 정지 (기준과 51 cm)", fontsize=8, color=LIM)
a.plot(P["ref_full"][0, 0], P["ref_full"][0, 1], "o", color=INK, ms=4)
a.set_aspect("equal"); a.set_xlabel("x [m] (지도)"); a.set_ylabel("y [m]"); a.legend(loc="lower left", fontsize=8, frameon=False)
a.set_title("(a) 경로 — SLAM 로봇 중심", loc="left", fontsize=10, color=INK)

a = ax[0, 1]
a.plot(P["t"], 1e3 * P["e"], color=CP, lw=1.5, label="A PP"); a.plot(M["t"], 1e3 * M["e"], color=CM, lw=1.5, label="C MPC")
a.axhline(500, color=LIM, lw=0.9, ls="--"); a.text(0.05, 515, "fault 한계 50 cm", color=LIM, fontsize=8)
a.axvline(t_f, color=CM, lw=0.6, ls=":")
a.set_xlabel("기준 출발부터 t [s]"); a.set_ylabel("위치 오차 |p − p_r| [mm]"); a.legend(loc="upper right", fontsize=8, frameon=False)
a.set_title("(b) 시간 추종 오차 (SLAM 기준)", loc="left", fontsize=10, color=INK)

a = ax[0, 2]
a.plot(P["t"], P["wm"][:, 0], color=CP, lw=1.4, label="A PP  FL 측정")
a.plot(P["t"], P["wn"][:, 0], color=CP, lw=0.9, ls="--", label="A PP  FL 지면 기준 ω_nom")
a.plot(M["t"], M["wm"][:, 0], color=CM, lw=1.6, label="C MPC FL 측정")
a.plot(M["t"], M["wn"][:, 0], color=CM, lw=0.9, ls="--", label="C MPC FL 지면 기준 ω_nom")
a.axhline(0, color=MUTED, lw=0.6)
a.annotate("FL 이 거꾸로 돈다\n(λ ≈ −1.3 … −2)", (1.6, -21), (2.4, -8), fontsize=8, color=INK, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
a.set_xlabel("t [s]"); a.set_ylabel("FL 바퀴 ω [rad/s]"); a.legend(loc="upper left", fontsize=7.5, frameon=False, ncol=2)
a.set_ylim(-40, 60)
a.set_title("(c) 안쪽 앞바퀴 FL — 측정 vs 지면 속도로 계산한 값", loc="left", fontsize=10, color=INK)

a = ax[1, 0]
a.fill_between([0, 6.4], -TB.min(), TB.min(), color=CM, alpha=0.12, lw=0)
a.text(0.1, -0.33, f"음영 = C 상한 ±τ̄ ({TB.min():.2f}~{TB.max():.2f} N·m, 마찰 보상 전)", fontsize=8, color=MUTED)
a.plot(P["t"], P["tau"][:, 0], color=CP, lw=1.3, label="A PP  FL"); a.plot(P["t"], P["tau"][:, 1], color=CP, lw=1.0, ls="--", label="A PP  FR")
a.plot(M["t"], M["tau"][:, 0], color=CM, lw=1.6, label="C MPC FL"); a.plot(M["t"], M["tau"][:, 1], color=CM, lw=1.1, ls="--", label="C MPC FR")
a.set_xlabel("t [s]"); a.set_ylabel("최종 바퀴 토크 [N·m] (마찰 보상 포함)"); a.legend(loc="upper right", fontsize=7.5, frameon=False, ncol=2)
a.set_ylim(-0.4, 0.8)
a.set_title("(d) 바퀴 토크 — PP 는 C 상한의 4~6 배까지 쓴다", loc="left", fontsize=10, color=INK)

a = ax[1, 1]
a.plot(P["t"], np.abs(P["lam"]).max(1), color=CP, lw=1.1, label="A PP")
a.plot(M["t"], np.abs(M["lam"]).max(1), color=CM, lw=1.5, label="C MPC")
a.axhline(0.5, color=LIM, lw=0.9, ls="--"); a.text(3.3, 0.38, "큰 슬립 |λ| 0.5", color=LIM, fontsize=8)
a.set_ylim(0, 2.2); a.set_xlabel("t [s]"); a.set_ylabel("max_i |λ_i| (SLAM 속도 + 자이로 기준)"); a.legend(loc="upper right", fontsize=8, frameon=False)
a.set_title("(e) 네 바퀴 중 가장 큰 슬립", loc="left", fontsize=10, color=INK)

a = ax[1, 2]
import json
r = json.load(open(os.path.join(HERE, "why.json")))
labs = list(r.keys()); x = np.arange(len(labs))
dr = [r[k]["drive"] for k in labs]; br = [r[k]["brake"] for k in labs]
a.bar(x, br, width=0.55, color="#e34948", label="끌림 λ < −0.5")
a.bar(x, dr, width=0.55, bottom=br, color="#2a78d6", label="헛돎 λ > 0.5", edgecolor="#fcfcfb", linewidth=2)
for i, k in enumerate(labs):
    a.text(i, br[i] + dr[i] + 2, f"{r[k]['big']:.0f} %", ha="center", fontsize=9, color=INK)
a.set_xticks(x, ["A PP", "C MPC"]); a.set_ylim(0, 100); a.set_ylabel("달린 시간 중 큰 슬립 %"); a.legend(loc="upper left", fontsize=8, frameon=False)
a.set_title("(f) 큰 슬립 시간 (헛돎 / 끌림)", loc="left", fontsize=10, color=INK)

fig.suptitle("실차 2026-11-21 circles8 1.1 m/s · accel 1.5 — A 퓨어 퍼슛 (200720) vs C MPC friction (200303, mu_hat_wheels ×1.125 · W [20, 20, 2.5, 2, 2, 1])",
             x=0.05, y=0.985, ha="left", fontsize=11, color=INK)
fig.savefig(os.path.join(HERE, "fig_why_pp_vs_mpc.png"), dpi=130, bbox_inches="tight")
print("saved")

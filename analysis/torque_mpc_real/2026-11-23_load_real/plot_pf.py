#!/usr/bin/python3
"""실차 path following (2026-11-23 17:20, 171926_pf_c · 172102_pf_pp) vs 같은 기준 시간 추종 판 → fig_pf.png
위: 판마다 한 칸 궤적 (C 시간 추종 160416 · C path following · PP 시간 추종 161802 · PP path following), 출발 포즈 좌표, 빨간 점 = 큰 슬립.
아래: (e) 기준 시각 늦음 t_clock − t_ref, (f) 경로 이탈 vs 시간 오차 RMS, (g) 큰 슬립 % — 시간 추종은 같은 설정 판 전부 (점), path following 1 판.
자료: sweep_pf.json·sweep_pf.npz (sweep_compare.py), pf_progress.npz (pf_progress.py)."""
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
INK, LIM, MUTED = "#0b0b0b", "#d03b3b", "#52514e"
CC, CP = "#1baf7a", "#2a78d6"
R = json.load(open(os.path.join(HERE, "sweep_pf.json"))); z = np.load(os.path.join(HERE, "sweep_pf.npz")); pg = np.load(os.path.join(HERE, "pf_progress.npz"))
TRAJ = [("C020b", "C h 0.2 시간 추종 (160416, 39 °C)", CC), ("Cpf", "C h 0.2 path following (171926, 30 °C)", CC),
        ("PPb", "PP 시간 추종 (161802, 41 °C)", CP), ("PPpf", "PP path following (172102, 32 °C)", CP)]


def local(lab, x, y):
    r0 = z[f"{lab}_ref"][0]; c0, s0 = np.cos(r0[2]), np.sin(r0[2]); dx, dy = x - r0[0], y - r0[1]
    return c0 * dx + s0 * dy, -s0 * dx + c0 * dy


fig = plt.figure(figsize=(16, 11.5))
gs = fig.add_gridspec(2, 12, height_ratios=[1.45, 1], hspace=0.32, wspace=0.9)
for k, (lab, name, col) in enumerate(TRAJ):
    a = fig.add_subplot(gs[0, 3 * k:3 * k + 3]); r = R[lab]
    rx, ry = local(lab, z[f"{lab}_ref"][:, 0], z[f"{lab}_ref"][:, 1]); lx, ly = local(lab, z[f"{lab}_x"], z[f"{lab}_y"])
    a.plot(rx, ry, color=INK, lw=0.8, ls="--"); a.plot(lx, ly, color=col, lw=1.3, alpha=1.0 if "pf" in lab else 0.75)
    big = (np.abs(z[f"{lab}_lam"]).max(1) > 0.5) & z[f"{lab}_moving"]; a.plot(lx[big], ly[big], ".", color=LIM, ms=2.5)
    a.set_aspect("equal"); a.set_xlim(-0.62, 0.5); a.set_ylim(-0.97, 0.93); a.set_xlabel("출발 헤딩 방향 [m]")
    if k == 0:
        a.set_ylabel("왼쪽 [m]")
    a.set_title(f"({'abcd'[k]}) {name}\n랩 {r['T']:.2f} s · 경로 이탈 {r['e_cont_rms']:.0f}/{r['e_cont_max']:.0f} mm · 시간 오차 {r['e_rms']:.0f} mm\n"
                f"yaw {r['h_rms']:.1f}° · 큰 슬립 {r['big']:.0f} % (헛돎 {r['drive']:.0f}/끌림 {r['brake']:.0f})", loc="left", fontsize=8.5, color=INK)
# (e) 늦음
a = fig.add_subplot(gs[1, 0:5])
for bag, lab, col in (("171926_pf_c", "C path following", CC), ("172102_pf_pp", "PP path following", CP)):
    P = pg[bag]; m = P[:, 1] >= -0.5; t = P[m, 0] - P[m, 0][np.argmax(P[m, 1] >= 0)]
    a.plot(t, P[m, 2] - P[m, 1], color=col, lw=1.4, label=f"{lab} — 끝 {P[m, 2][-1] - P[m, 1][-1]:.2f} s 늦음")
a.set_xlim(0, 22); a.set_xlabel("기준 출발부터 벽시계 [s]"); a.set_ylabel("t_clock − t_ref [s]")
a.set_title("(e) 기준 시각이 시계보다 늦은 양 (= 로봇이 못 따라가 기준이 기다린 시간의 누적)", loc="left", fontsize=9.5, color=INK)
a.legend(loc="upper left", fontsize=8, frameon=False)
# (f) 경로 이탈 vs 시간 오차, (g) 큰 슬립
GROUPS = [("C 시간 추종", ["C020a", "C020b", "C020c"], CC, 0.35), ("C path following", ["Cpf"], CC, 1.0),
          ("PP 시간 추종", ["PPa", "PPb"], CP, 0.35), ("PP path following", ["PPpf"], CP, 1.0)]
for j, (key, title, unit) in enumerate((("e_cont_rms", "(f) 경로 이탈 RMS (점) · 시간 오차 RMS (×)", "mm"), ("big", "(g) 큰 슬립 % (max_i |λ_i| > 0.5)", "%"))):
    a = fig.add_subplot(gs[1, 5 + 3 * j + (1 if j else 0): 5 + 3 * j + 3 + (1 if j else 0) + (0 if j else 0)])
    for i, (name, labs, col, al) in enumerate(GROUPS):
        v = [R[l][key] for l in labs]
        a.plot([i] * len(v), v, "o", color=col, alpha=max(al, 0.6), ms=7, mfc=col if al == 1.0 else "none")
        if key == "e_cont_rms":
            a.plot([i] * len(labs), [R[l]["e_rms"] for l in labs], "x", color=col, ms=7)
    a.set_xticks(range(4), [g[0].replace(" ", "\n", 1) for g in GROUPS], fontsize=8); a.set_ylabel(unit); a.set_xlim(-0.5, 3.5)
    a.set_title(title, loc="left", fontsize=9.5, color=INK)
fig.text(0.05, 0.015, "시간 추종 판은 모터 38~41 °C (PP 151249 만 32 °C), path following 두 판은 재부팅 뒤 30~33 °C — 같은 설정에서도 온도·판에 따라 RMS 가 26 mm 까지 흔들렸다 (h 스윕).",
         fontsize=8, color=MUTED)
fig.suptitle("실차 path following (path_follow_lead 0.05 s) vs 시간 추종 — 계단 변조 8자 speed 0.69 · accel 3.0 (마찰 밖 기준), 2026-11-23",
             x=0.05, y=0.985, ha="left", fontsize=12, color=INK)
fig.savefig(os.path.join(HERE, "fig_pf.png"), dpi=120, bbox_inches="tight"); print("saved")

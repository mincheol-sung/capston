#!/usr/bin/python3
"""fig_corner1_slam_jump.png — 121319 첫 꼭짓점: SLAM·추정기·바퀴+자이로 추측항법 (0.5 s SLAM 포즈에서 출발) 경로와 안쪽 옆 위치, MPC 가 본 옆 오차."""
import math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from load import load, ref_at
HERE = os.path.dirname(os.path.abspath(__file__)); B = "/home/smoc/capston/capston/bags/2026-11-23"
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
C1, C2, C3, INK, MUTED, LIM = "#1baf7a", "#eb6834", "#6250d6", "#0b0b0b", "#52514e", "#d03b3b"   # SLAM(C 색) · 추정기 · 추측항법
d = load(f"{B}/121319"); D = d["D"]; sl, E, W, G, S = d["slam"], d["E"], d["W"], d["G"], d["S"]; a0 = d["arm"][0]
ts = np.arange(a0, a0 + 30, 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts]); tr0 = ts[np.argmax(sp > 0.02)]
r, k = 0.04, 0.1932; Tp = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-1 / k, 1 / k, -1 / k, 1 / k]]) * r / 4
Xr0, _ = ref_at(D, tr0 + 0.5); n0 = np.array([-math.sin(Xr0[2]), math.cos(Xr0[2])])
i = np.argmin(np.abs(sl[:, 0] - (tr0 + 0.5))); ta = sl[i, 0]; base = sl[i, 1:3]
p = base.copy(); psi = sl[i, 3]; tt = np.arange(ta, tr0 + 2.2, 0.002); DR = []
for t in tt:
    w = np.array([np.interp(t, W[:, 0], W[:, 1 + j]) for j in range(4)]); vb = Tp @ w
    psi += np.interp(t, G[:, 0], G[:, 1]) * 0.002
    p = p + 0.002 * np.array([math.cos(psi) * vb[0] - math.sin(psi) * vb[1], math.sin(psi) * vb[0] + math.cos(psi) * vb[1]]); DR.append(p)
DR = np.array(DR)
tq = np.arange(0.0, 2.2, 0.01) + tr0
ref = np.array([ref_at(D, t)[0][:2] for t in tq])
slp = np.stack([np.interp(tq, sl[:, 0], sl[:, 1]), np.interp(tq, sl[:, 0], sl[:, 2])], 1)
est = np.stack([np.interp(tq, E[:, 0], E[:, 1]), np.interp(tq, E[:, 0], E[:, 2])], 1)
fig, ax = plt.subplots(1, 3, figsize=(15.5, 5.2), gridspec_kw=dict(width_ratios=[1, 1.25, 1.25], wspace=0.28))
a = ax[0]
a.plot(ref[:, 0], ref[:, 1], color=INK, lw=0.9, ls="--", label="기준")
a.plot(slp[:, 0], slp[:, 1], color=C1, lw=1.6, label="SLAM (로봇 중심) — 그림의 C 경로")
m = (sl[:, 0] > tr0) & (sl[:, 0] < tr0 + 2.2); a.plot(sl[m, 1], sl[m, 2], "o", color=C1, ms=3)
a.plot(est[:, 0], est[:, 1], color=C2, lw=1.2, label="상태 추정기 (MPC 가 보는 위치)")
a.plot(DR[:, 0], DR[:, 1], color=C3, lw=1.2, label="바퀴+자이로 추측항법 (0.5 s 에서 출발)")
j = np.argmin(np.abs(sl[:, 0] - (tr0 + 1.04))); a.annotate("SLAM 81 mm 점프\n(0.94 → 1.04 s)", sl[j, 1:3], sl[j, 1:3] + np.array([0.25, 0.05]),
                                                          fontsize=8, color=LIM, arrowprops=dict(arrowstyle="-", color=LIM, lw=0.7))
a.set_aspect("equal"); a.set_xlabel("x [m]"); a.set_ylabel("y [m]"); a.legend(loc="upper right", fontsize=7.5, frameon=False)
lo = ref[(tq - tr0 > 0.3) & (tq - tr0 < 2.1)]; a.set_xlim(lo[:, 0].min() - 0.25, lo[:, 0].max() + 0.35); a.set_ylim(lo[:, 1].min() - 0.2, lo[:, 1].max() + 0.25)
a.set_title("(a) 첫 변·첫 꼭짓점 경로 (점 = SLAM 표본 10 Hz)", loc="left", fontsize=10, color=INK)

b = ax[1]; tl = tq - tr0; w = tl >= 0.5
b.plot(tl, 1e3 * ((slp - base) @ n0), color=C1, lw=1.6, label="SLAM")
b.plot(tl, 1e3 * ((est - base) @ n0), color=C2, lw=1.2, label="상태 추정기")
b.plot(tt - tr0, 1e3 * ((DR - base) @ n0), color=C3, lw=1.2, label="바퀴+자이로 추측항법")
b.axvspan(0.94, 1.04, color=LIM, alpha=0.12, lw=0); b.text(1.06, -55, "SLAM 점프", color=LIM, fontsize=8)
b.axvline(1.45, color=MUTED, lw=0.6, ls=":"); b.text(1.47, 175, "꼭짓점 헤딩 회전 시작", color=MUTED, fontsize=8)
b.set_xlim(0.5, 1.8); b.set_xlabel("기준 출발부터 t [s]"); b.set_ylabel("첫 변 안쪽 법선 방향 위치 [mm] (+ 안)")
b.legend(loc="upper left", fontsize=8, frameon=False)
b.set_title("(b) 옆 위치 — 바퀴·자이로로는 안 움직였다", loc="left", fontsize=10, color=INK)

c = ax[2]; ms = (S[:, 0] > tr0 + 0.5) & (S[:, 0] < tr0 + 1.8); tsr = S[ms, 0] - tr0
en = np.array([(-S[q, 1:3]) @ n0 for q in np.where(ms)[0]])     # status Xe = 기준 − 상태 → 부호 뒤집음
c.plot(tsr, 1e3 * en, color=C2, lw=1.4, label="MPC 가 본 옆 오차 (추정기 − 기준)")
c.annotate("이 오차를 줄이려 바깥으로 민다 →\n추측항법 옆 위치 −9 → −47 mm ((b) 보라)", (1.35, 160), (0.55, 140), fontsize=8, color=MUTED,
           arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
c.axvspan(0.94, 1.04, color=LIM, alpha=0.12, lw=0)
c.set_xlim(0.5, 1.8); c.set_xlabel("t [s]"); c.set_ylabel("옆 오차 [mm] (+ 안)"); c.legend(loc="upper left", fontsize=8, frameon=False)
c.set_title("(c) MPC 는 '안으로 17 cm' 로 보고 바깥으로 민다", loc="left", fontsize=10, color=INK)
fig.suptitle("실차 121319 (C, 정사각형 speed:=max) 첫 꼭짓점 — 안으로 파고든 것처럼 보이는 이유", x=0.05, y=1.0, ha="left", fontsize=11.5, color=INK)
fig.savefig(os.path.join(HERE, "fig_corner1_slam_jump.png"), dpi=130, bbox_inches="tight")
print("saved")

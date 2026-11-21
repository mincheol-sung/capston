#!/usr/bin/python3
"""profile.npz → fig_square_profile.png (시간 프로파일 + 경로 + 꼭짓점 1 확대)."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "axes.labelcolor": "#0b0b0b", "xtick.color": "#52514e", "ytick.color": "#52514e",
                     "axes.grid": True, "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb",
                     "axes.facecolor": "#fcfcfb", "axes.unicode_minus": False})
WC = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]          # FL FR RL RR (범주 1~4)
GRAY = "#8a8985"; INK = "#0b0b0b"; LIM = "#d03b3b"
NM = ["FL", "FR", "RL", "RR"]
z = np.load(os.path.join(HERE, "profile.npz"))
t, X, U, psi, alpha, spd, S = z["t"], z["X"], z["U"], z["psi"], z["alpha"], z["spd"], z["S"]
corner, rat_e, rat_d, tb = z["corner"], z["rat_e"], z["rat_d"], z["tb"]
T_path, T_total, t_start = float(z["T_path"]), float(z["T_total"]), z["t_start"]

# 꼭짓점 구간 [시작, 끝]
spans, on = [], None
for n in range(len(t)):
    if corner[n] and on is None:
        on = t[n]
    if not corner[n] and on is not None:
        spans.append((on, t[n - 1])); on = None

fig = plt.figure(figsize=(13, 9.2))
gs = fig.add_gridspec(5, 2, width_ratios=[1.75, 1], hspace=0.32, wspace=0.18)
axL = [fig.add_subplot(gs[i, 0]) for i in range(5)]
for a in axL[1:]:
    a.sharex(axL[0])


def shade(a):
    for s0, s1 in spans:
        a.axvspan(s0, s1, color="#cde2fb", alpha=0.55, lw=0)
    a.axvspan(T_path, T_total, color="#e4e3df", alpha=0.7, lw=0)


a = axL[0]; shade(a)
a.plot(t, spd, color=INK, lw=1.6)
for i, ts in enumerate(t_start):
    a.text(ts + 0.05, 1.62, f"변 {i}", color="#52514e", fontsize=8)
a.set_ylabel("속력 |v| [m/s]"); a.set_ylim(0, 1.75)
a.annotate("1.5 m/s (가운데)", (t_start[1] + 0.94, 1.5), (t_start[1] + 1.15, 1.25), fontsize=8, color="#52514e",
           arrowprops=dict(arrowstyle="-", color=GRAY, lw=0.6))
a.annotate("꼭짓점 0.1 m/s", (t_start[2], 0.1), (t_start[2] + 0.25, 0.42), fontsize=8, color="#52514e",
           arrowprops=dict(arrowstyle="-", color=GRAY, lw=0.6))
a.set_title("정사각형 기준  side 1.5 m · corner 0.1 m/s · accel 1.5 m/s² · v_max 1.5 m/s · tangent (yaw_blend 0.3 m)",
            loc="left", fontsize=10, color=INK)

a = axL[1]; shade(a)
a.plot(t, np.degrees(psi), color=INK, lw=1.6)
a.set_ylabel("헤딩 ψ [deg]"); a.set_yticks([0, 90, 180, 270])

a = axL[2]; shade(a)
a.plot(t, U[:, 2], color=INK, lw=1.6)
a.set_ylabel("ω [rad/s]")
a.text(T_path + 0.05, 2.2, "정렬 회전\n(서서 +90°)", fontsize=8, color="#52514e")

a = axL[3]; shade(a)
a.plot(t, alpha, color=GRAY, lw=0.9, label="해석값")
# 노드가 쓰는 0.2 s 차분의 θ̈ 은 rat_d 계산에만 들어갔으니 여기서 다시 낸다
h = 100
wpad = np.concatenate([np.repeat(U[:1, 2], h), U[:, 2], np.repeat(U[-1:, 2], h)])
a.plot(t, (wpad[2 * h:] - wpad[:-2 * h]) / 0.2, color=WC[0], lw=1.6, label="노드가 쓰는 0.2 s 차분")
a.set_ylabel("α = ω̇ [rad/s²]"); a.legend(loc="lower left", fontsize=8, frameon=False, ncol=2)

a = axL[4]; shade(a)
for i in range(4):
    a.plot(t, rat_d[:, i], color=WC[i], lw=1.4, label=f"{NM[i]} (τ̄ {tb[i]:.3f})")
a.axhline(1.0, color=LIM, lw=1.0, ls="--")
a.text(0.05, 1.08, "C 마찰 상한", color=LIM, fontsize=8)
a.set_ylabel("|U_s,i| / τ̄_i\n(0.2 s 차분)"); a.set_xlabel("t [s]   (파랑 띠 = 꼭짓점 헤딩 회전 구간, 회색 띠 = 끝 정렬 회전)")
a.legend(loc="lower right", bbox_to_anchor=(1.0, 0.98), fontsize=8, frameon=False, ncol=4)
a.set_xlim(0, T_total + 0.2)
for a in axL[:-1]:
    plt.setp(a.get_xticklabels(), visible=False)

# 경로 (속력 색)
ax = fig.add_subplot(gs[0:3, 1])
pts = X[:, :2]; segs = np.stack([pts[:-1], pts[1:]], 1)
from matplotlib.colors import ListedColormap
cm = ListedColormap(plt.get_cmap("Blues")(np.linspace(0.3, 1.0, 256)))
lc = LineCollection(segs, cmap=cm, norm=plt.Normalize(0.0, 1.5), lw=3.2)
lc.set_array(spd[:-1]); ax.add_collection(lc)
for j in range(1, 4):
    m = (np.abs(S - j * 1.5) <= 0.15) & (spd > 0)
    ax.plot(pts[m, 0], pts[m, 1], color=LIM, lw=1.0)
# 헤딩 화살표 (0.25 s 마다)
for n in range(0, int(T_path / 1e-3), 250):
    ax.annotate("", (X[n, 0] + 0.12 * np.cos(X[n, 2]), X[n, 1] + 0.12 * np.sin(X[n, 2])), (X[n, 0], X[n, 1]),
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.7, mutation_scale=7))
cb = fig.colorbar(lc, ax=ax, fraction=0.04, pad=0.02); cb.set_label("속력 [m/s]")
ax.set_aspect("equal"); ax.set_xlim(-0.3, 1.8); ax.set_ylim(-0.3, 1.8)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("경로 (화살표 = 헤딩 0.25 s 마다, 빨강 = 헤딩 회전 구간 ±0.15 m)", loc="left", fontsize=9)
ax.text(0.02, -0.22, "출발·도착", fontsize=8, color="#52514e")

# 꼭짓점 1 확대
ax = fig.add_subplot(gs[3:5, 1])
w = (t > 1.40) & (t < 2.50)
for s0, s1 in spans[:1]:
    ax.axvspan(s0, s1, color="#cde2fb", alpha=0.55, lw=0)
for i in range(4):
    ax.plot(t[w], rat_d[w, i], color=WC[i], lw=1.4, label=NM[i])
ax.plot(t[w], rat_e[w].max(1), color=GRAY, lw=0.9, ls="-", label="해석 가속을 쓸 때 (네 바퀴 최대)")
ax.axhline(1.0, color=LIM, lw=1.0, ls="--")
ax.axvline(t_start[1], color=GRAY, lw=0.6, ls=":")
ax.text(t_start[1] + 0.01, 0.08, "꼭짓점", fontsize=8, color="#52514e")
ax.set_ylim(0, 5.2); ax.set_xlabel("t [s]"); ax.set_ylabel("|U_s,i| / τ̄_i")
ax.set_title("꼭짓점 1 확대 (1.4~2.5 s)", loc="left", fontsize=9)
ax.legend(loc="upper left", fontsize=7.5, frameon=False, ncol=3)

fig.savefig(os.path.join(HERE, "fig_square_profile.png"), dpi=140, bbox_inches="tight")
print("saved")

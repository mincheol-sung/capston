#!/usr/bin/python3
"""가속 격자 (정사각형 1.5 m, 최고 1.5 m/s) — 궤적 격자 그림 + 표. 사용법: analyze_acc.py 결과폴더 그림폴더
   윤곽 오차 = 참값 위치에서 정사각형 변까지 거리 (경로 모양을 얼마나 지켰나, 시간 지연과 무관)."""
import json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUN, FIG = sys.argv[1], sys.argv[2]
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9.5, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2, REF = "#0b0b0b", "#52514e", "#a9a8a2"
SER = {"pp": ("퓨어 퍼슛 + 바퀴 PI", "#1baf7a"), "mpcB": ("MPC B (모터 상한)", "#2a78d6"), "mpcC": ("MPC C (마찰 상한)", "#eb6834")}
ACC, MU, SIDE = ["1.0", "1.5", "2.0"], ["0.3", "0.4"], 1.5


def contour(raw):
    px, py = raw[:, 1], raw[:, 2]
    def seg(ax, ay, bx, by):
        ux, uy = bx - ax, by - ay
        t = np.clip(((px - ax) * ux + (py - ay) * uy) / (ux * ux + uy * uy), 0, 1)
        return np.hypot(px - ax - t * ux, py - ay - t * uy)
    P = [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]
    d = np.min([seg(*P[i], *P[i + 1]) for i in range(4)], axis=0)
    mv = np.hypot(raw[:, 7], raw[:, 8]) > 0.02
    return float(np.sqrt(np.mean(d[mv] ** 2))), float(d[mv].max())


rows = []
fig, axes = plt.subplots(len(ACC), len(MU), figsize=(10, 14))
for i, a in enumerate(ACC):
    for j, mu in enumerate(MU):
        ax = axes[i][j]
        ax.plot([0, SIDE, SIDE, 0, 0], [0, 0, SIDE, SIDE, 0], color=REF, lw=6, alpha=0.45, solid_capstyle="round")
        need = math.sqrt(2) * float(a) / 9.81
        for c, (name, col) in SER.items():
            lab = f"{c}_a{a}_mu{mu}"
            fj = os.path.join(RUN, f"{lab}.json")
            if not os.path.exists(fj):
                continue
            s = json.loads(open(fj).read().strip().splitlines()[-1])
            raw = np.load(os.path.join(RUN, f"{lab}.npz"))["raw"]
            cr, cm = contour(raw)
            div = "track_error" in (s.get("fault") or "")
            rows.append(dict(a=a, mu=mu, c=c, track=s.get("moving_rms_mm", np.nan), contour=1e3 * cr, contour_max=1e3 * cm,
                             slip=100 * s.get("slip_frac10", np.nan), lmax=s.get("slip_max", np.nan), tau=s.get("tau_max", np.nan), div=div))
            ax.plot(raw[:, 1], raw[:, 2], color=col, lw=1.6,
                    label=f"{name}: 윤곽 {1e3*cr:.0f} mm, 슬립 {100*s.get('slip_frac10', np.nan):.0f} %" + (" (이탈)" if div else ""))
        ax.set_aspect("equal"); ax.set_xlim(-0.6, 2.1); ax.set_ylim(-0.6, 2.4)
        ax.set_title(f"가속 {a} m/s² · 바닥 μ {mu} (필요 μ {need:.2f}{' > 바닥' if need > float(mu) else ''})", loc="left", color=INK, fontsize=10)
        ax.legend(loc="upper left", frameon=False, fontsize=8)
fig.suptitle("정사각형 1.5 m · 최고 1.5 m/s · 꼭짓점 0.1 m/s · tangent — 시뮬 참값 궤적, 지금 가중치 (판마다 1 회)", x=0.01, ha="left", color=INK, fontsize=11.5)
fig.tight_layout(rect=(0, 0, 1, 0.98))
fig.savefig(os.path.join(FIG, "fig_accel_grid.png"), dpi=120)
print("| 가속 | 바닥 μ | 제어기 | 윤곽 RMS / 최대 [mm] | 추종 RMS [mm] | 슬립 >0.1 [%] | λ max | τ max |")
print("|---|---|---|---|---|---|---|---|")
for r in rows:
    print(f"| {r['a']} | {r['mu']} | {r['c']}{' (이탈)' if r['div'] else ''} | {r['contour']:.0f} / {r['contour_max']:.0f} | {r['track']:.0f} | {r['slip']:.1f} | {r['lmax']:.2f} | {r['tau']:.3f} |")
json.dump(rows, open(os.path.join(FIG, "accel_grid.json"), "w"), ensure_ascii=False, indent=1)

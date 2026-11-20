#!/usr/bin/python3
"""속도 탐색 (circles8 W2.0 tangent, 바닥 μ 0.4): PP·MPC B·C 의 슬립·위치 오차 vs 속도."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK = "#0b0b0b"
SER = {"pp": ("퓨어 퍼슛 + 바퀴 PI (엔코더)", "#1baf7a"), "mpcB": ("MPC B (모터 상한, 추정기)", "#2a78d6"),
       "mpcC": ("MPC C (마찰 상한, 추정기)", "#eb6834")}
V = [0.6, 0.7, 0.8, 0.9, 1.0, 1.1]
def get(c, v):
    return json.loads(open(os.path.join(HERE, "speed", f"{c}_v{v}.json")).read().strip().splitlines()[-1])
fig, ax = plt.subplots(1, 3, figsize=(13, 4.2))
for c, (name, col) in SER.items():
    s = [get(c, v) for v in V]
    for a, key, k in ((ax[0], "slip_frac10", 100), (ax[1], "slip_max", 1), (ax[2], "moving_rms_mm", 1)):
        y = [x[key] * k for x in s]
        a.plot(V, y, color=col, lw=2, marker="o", ms=6, mec="#fcfcfb", mew=1.5, label=name)
ax[0].set_title("|λ| > 0.1 인 주행 시간 비율 [%]", loc="left", color=INK)
ax[1].set_title("슬립률 최대 |λ| (참값)", loc="left", color=INK); ax[1].set_yscale("log")
ax[2].set_title("주행 위치 오차 RMS [mm] (참값)", loc="left", color=INK)
for a in ax:
    a.set_xlabel("순항 속도 [m/s]  (W2.0 상한 1.11)")
ax[0].legend(frameon=False, loc="upper left", fontsize=9)
fig.suptitle("circles8 W2.0 · tangent · 바닥 μ 0.4 (슬립 모델) · MPC 는 지금 가중치 (w_dudt 0.001, r_u 0.1, use_w) — 판마다 1 회",
             x=0.01, ha="left", color="#52514e", fontsize=9.5)
fig.tight_layout(rect=(0, 0, 1, 0.95))
fig.savefig(os.path.join(HERE, "fig_speed_slip.png"), dpi=130)
print("saved")

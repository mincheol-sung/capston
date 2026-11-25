#!/usr/bin/python3
"""load_compare.npz/json → fig_load_real.png: 위치 오차, 원심 가속 구간별 안쪽/바깥 슬립."""
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
INK = "#0b0b0b"
z = np.load("load_compare.npz"); R = json.load(open("load_compare.json"))
fig, ax = plt.subplots(1, 2, figsize=(14.5, 4.8), gridspec_kw=dict(width_ratios=[1.35, 1], wspace=0.22))
a = ax[0]
for lab in ("C_off", "C_on", "PP"):
    r = R[lab]; a.plot(z[f"{lab}_t"], 1e3 * z[f"{lab}_e"], color=COL[lab], lw=1.2,
                       label=f"{NAME[lab]}  RMS {r['e_rms']:.0f} · 최대 {r['e_max']:.0f} mm · 요 {r['h_rms']:.1f}° · 큰 슬립 {r['big']:.0f} %")
a.set_xlabel("기준 출발부터 t [s]"); a.set_ylabel("위치 오차 [mm]"); a.legend(loc="upper left", fontsize=7.5, frameon=False)
a.set_title("(a) 시간 추종 오차 — 실차 계단 변조 8자 (지름 0.8 m, accel 3.0)", loc="left", fontsize=10, color=INK)
b = ax[1]; keys = ["0.0-0.6", "0.6-2.2", "2.2-3.5"]; xl = ["a_y < 0.6", "0.6 ~ 2.2", "2.2 ~ 3.5 m/s²"]
w = 0.13; x = np.arange(3)
for j, lab in enumerate(("C_off", "C_on", "PP")):
    inf = [R[lab]["bins"][k]["in_f"] for k in keys]; inr = [R[lab]["bins"][k]["in_r"] for k in keys]
    b.bar(x + (2 * j - 2.5) * w, inf, width=w * 0.95, color=COL[lab], label=f"{lab} 안쪽 앞")
    b.bar(x + (2 * j - 1.5) * w, inr, width=w * 0.95, color=COL[lab], alpha=0.45, label=f"{lab} 안쪽 뒤")
b.set_xticks(x, [f"{l}\n(주행의 {R['C_on']['bins'][k]['share']:.0f} %)" for l, k in zip(xl, keys)])
b.set_ylabel("|λ| > 0.5 시간 %"); b.legend(loc="upper left", fontsize=7.5, frameon=False, ncol=3)
b.set_title("(b) 원심 가속 구간별 안쪽 바퀴 큰 슬립 (진한 = 앞, 옅은 = 뒤)", loc="left", fontsize=10, color=INK)
fig.savefig("fig_load_real.png", dpi=130, bbox_inches="tight"); print("saved")

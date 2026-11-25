#!/usr/bin/python3
"""sweep.json → fig_h_sweep.png + 평균 표 (stdout). 실차 h 스윕 2026-11-23 (계단 변조 8자, 모터 38~41 °C 판).
점 = 판 하나, 선 = h 별 평균. 151038 (h 0, 모터 30 °C — 조건이 다른 이른 판) 은 회색 ×, h 0.2 상한 고정 (가중 배분만) 은 빈 네모, PP 두 판은 띠."""
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
C, PPC, ND, OLD, INK = "#1baf7a", "#2a78d6", "#eb6834", "#9a9893", "#0b0b0b"
R = json.load(open(os.path.join(HERE, "sweep.json")))
H = {"C_h0.2": 0.2, "C_h000": 0.0, "C_h000c": 0.0, "C_h010": 0.1, "C_h010b": 0.1, "C_h015": 0.15, "C_h015b": 0.15, "C_h015c": 0.15,
     "C_h020b": 0.2, "C_h020c": 0.2, "C_h025": 0.25, "C_h025b": 0.25}
PP = [R["PP"], R["PP_ppb"]]
M = [("e_rms", "위치 RMS [mm]"), ("e_max", "위치 최대 [mm] (대개 t ≈ 18 s)"), ("h_rms", "yaw RMS [°]"),
     ("big", "큰 슬립 % (max_i |λ_i| > 0.5)"), ("lam_rms", "λ RMS (네 바퀴, 기준 이동 중)"), ("hi_in", "센 선회 (a_y > 2.2) 안쪽 뒤 |λ|>0.5 %")]
val = lambda r, k: r["big_hi_in"][1] if k == "hi_in" else r[k]
hs = sorted(set(H.values()))
fig, ax = plt.subplots(2, 3, figsize=(14.5, 7.6), gridspec_kw=dict(hspace=0.38, wspace=0.26))
print("| h | 판 수 | " + " | ".join(t for _, t in M) + " |")
means = {h: {} for h in hs}
for (k, title), a in zip(M, ax.flat):
    pv = [val(r, k) for r in PP]
    a.axhspan(min(pv), max(pv), color=PPC, alpha=0.16, lw=0); a.axhline(np.mean(pv), color=PPC, lw=1.0, ls="--")
    a.text(0.995, np.mean(pv), " PP 두 판", color=PPC, fontsize=7.5, va="bottom", ha="right", transform=a.get_yaxis_transform())
    for lab, h in H.items():
        a.plot(h, val(R[lab], k), "o", color=C, ms=5, alpha=0.75)
    for h in hs:
        means[h][k] = float(np.mean([val(R[l], k) for l, hh in H.items() if hh == h]))
    a.plot(hs, [means[h][k] for h in hs], "-", color=C, lw=1.6, label="C 하중 이동 (판 = 점, 선 = 평균)")
    a.plot(0.2, val(R["C_h020nd"], k), "s", mfc="none", mec=ND, mew=1.5, ms=7, label="h 0.2 상한 고정 (가중 배분만)")
    a.plot(0.0, val(R["C_h0"], k), "x", color=OLD, ms=7, mew=1.5, label="h 0 151038 (모터 30 °C, 이른 판)")
    a.set_xticks(hs); a.set_xlabel("무게중심 높이 h [m] (mpc_com_height)"); a.set_title(title, loc="left", fontsize=10, color=INK)
ax.flat[0].legend(loc="upper center", fontsize=7.5, frameon=False)
for h in hs:
    n = sum(1 for v in H.values() if v == h)
    print(f"| {h:.2f} | {n} | " + " | ".join(f"{means[h][k]:.3f}" if k == "lam_rms" else f"{means[h][k]:.1f}" for k, _ in M) + " |")
for nm, rr in (("PP 평균", PP),):
    print(f"| {nm} | 2 | " + " | ".join(f"{np.mean([val(r, k) for r in rr]):.3f}" if k == "lam_rms" else f"{np.mean([val(r, k) for r in rr]):.1f}" for k, _ in M) + " |")
r = R["C_h020nd"]; print("| 0.20 상한 고정 | 1 | " + " | ".join(f"{val(r, k):.3f}" if k == "lam_rms" else f"{val(r, k):.1f}" for k, _ in M) + " |")
fig.suptitle("실차 무게중심 높이 h 스윕 — 계단 변조 8자 (지름 0.8 m, speed 0.69 · accel 3.0, 0.35~1.04 m/s), 2026-11-23 · 모터 38~41 °C",
             x=0.05, y=0.985, ha="left", fontsize=11.5, color=INK)
fig.savefig(os.path.join(HERE, "fig_h_sweep.png"), dpi=130, bbox_inches="tight"); print("saved")

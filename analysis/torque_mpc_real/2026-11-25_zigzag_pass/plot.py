# results.json (analyze.py) → 궤적 (경로 좌표, 추정기 실선·SLAM 점선) 과 바닥 토크/마찰 상한 비율 시계열
import json, os, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
R = {r["name"]: r for r in json.load(open(os.path.join(HERE, "results.json"))) if "_traj" in r}
order = [("140937", "MPC (C)"), ("141123", "PP L_d 0.30"), ("141841", "PP L_d 0.10 (n 9)"), ("141521", "PP L_d 0.05"),
         ("142336", "adpp 0.10~0.20 (n 9)"), ("142026", "adpp 0.10~0.20 (n 9, 7.5 s 중단)")]
fig, ax = plt.subplots(3, len(order), figsize=(4.2 * len(order), 11), gridspec_kw=dict(height_ratios=[2.2, 1, 1]))
for j, (n, lab) in enumerate(order):
    r = R[n]; tr = r["_traj"]
    P = np.array(tr["path"]); c0 = P[0]; d = P[5] - c0; a = -np.arctan2(d[1], d[0])
    Rm = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]); f = lambda X: (np.asarray(X) - c0) @ Rm.T
    A = ax[0, j]; A.plot(*f(P).T, "k-", lw=1, label="기준 경로")
    A.plot(*f(tr["od"]).T, "-", color="C0", lw=1.1, label="추정기")
    A.plot(*f(tr["slam"]).T, ":", color="C3", lw=1.0, label="SLAM")
    A.set_aspect("equal"); A.grid(alpha=0.3); A.set_title(f"{n} {lab}", fontsize=10)
    A.text(0.02, 0.02, f"경로 거리 RMS {100*r['d_rms']:.1f} cm (SLAM {100*r['dslam_rms']:.1f})\n최대 {100*r['d_max']:.1f} cm, e_θ RMS {r['th_rms']:.0f}°",
           transform=A.transAxes, fontsize=8, va="bottom")
    if j == 0: A.legend(fontsize=7, loc="upper right")
    ts = r["_ts"]; t = np.array(ts["t"]); g = np.array(ts["r"])
    B = ax[1, j]; B.plot(t, g, lw=0.6, color="C1"); B.axhline(1.0, color="k", lw=0.8); B.axhline(1.25, color="r", lw=0.8, ls="--")
    B.set_ylim(0, 3); B.grid(alpha=0.3); B.set_title(f"max_i |P(τ−f)|_i/τ̄_i  (> 1: {100*r['g_over1']:.0f} %)", fontsize=9)
    Cx = ax[2, j]; Cx.plot(ts["te"], 100 * np.array(ts["e"]), lw=0.6, color="C2"); Cx.set_ylim(0, 40); Cx.grid(alpha=0.3)
    Cx.set_title(f"시간 기준 |e_xy| [cm]  (RMS {100*r['e_rms']:.1f})", fontsize=9); Cx.set_xlabel("기준 시각 t [s]")
fig.tight_layout(); out = os.path.join(HERE, "zigzag_pass.png"); fig.savefig(out, dpi=100); print(out)

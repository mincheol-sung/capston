#!/usr/bin/python3
"""슬립 — pass vs stop × A·B·C, 시뮬 참값 λ (/slip_logger/slip 의 참값 λ×4, ω_nom = T⁻¹ v_참값).
  λ_i = (ω_i − ω_nom,i)·sgn(ω_nom,i) / max(|ω_i|, |ω_nom,i|, ω_ε)   (slip_logger.py)
      — 구동 (rω > v) 이면 1 − v/(rω), 제동이면 rω/v − 1 = Wong (Theory of Ground Vehicles) 의 구동 slip · 제동 skid 를 max 분모로 합친 꼴.
  주행 창 (analyze.py 와 같음) 안에서:  큰 슬립 = max_i |λ_i| > 0.5 인 시간 비율,  미끄럼 = > 0.1,  λ 최대, 네 바퀴 λ RMS,
  큰 슬립 중 헛돎 (λ > 0, 구동) / 끌림 (λ < 0, 제동) 몫, 큰 슬립이 꼭짓점 0.3 m 안 / 밖 몫, 바퀴별 |λ| 최대.
출력: 표 (stdout), slip_summary.json, fig_square_slip.png."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analyze import R, CORNERS

HERE = os.path.dirname(os.path.abspath(__file__))
NM = ["FL", "FR", "RL", "RR"]


def load(lab):
    f = f"{R}/{lab}.npz"
    if not os.path.exists(f):
        return None
    z = np.load(f); raw, lam = z["raw"], z["lam"]
    t = raw[:, 0]; vr = np.hypot(raw[:, 7], raw[:, 8]); act = (vr > 0.02) | (np.abs(raw[:, 9]) > 0.02)
    t0, t1 = t[np.argmax(act)], t[len(act) - 1 - np.argmax(act[::-1])]
    m = (lam[:, 0] >= t0) & (lam[:, 0] <= t1)
    L = lam[m]; L = L[np.all(np.isfinite(L[:, 1:5]), axis=1)]
    px = np.interp(L[:, 0], t, raw[:, 1]); py = np.interp(L[:, 0], t, raw[:, 2])
    near = np.min(np.hypot(px[:, None] - CORNERS[None, :, 0], py[:, None] - CORNERS[None, :, 1]), axis=1) < 0.3
    return L[:, 0] - t0, L[:, 1:5], near


def met(lab):
    d = load(lab)
    if d is None:
        return None
    tt, lam, near = d; a = np.abs(lam); mx = a.max(1); big = mx > 0.5
    sgn = np.sign(lam[np.arange(len(lam)), a.argmax(1)])
    return dict(lab=lab, spin=100 * float(big.mean()), slip01=100 * float((mx > 0.1).mean()), lmax=float(mx.max()),
                rms=float(np.sqrt(np.mean(lam ** 2))),
                drive=100 * float((sgn[big] > 0).mean()) if big.any() else float("nan"),
                corner=100 * float(near[big].mean()) if big.any() else float("nan"),
                wheel=[float(x) for x in a.max(0)])


if __name__ == "__main__":
    rows = {}
    for mode in ("pass", "stop"):
        for g in ("PP", "B", "C"):
            rows[f"{g}_{mode}"] = [m for m in (met(f"{g}_{mode}_r{k}") for k in (1, 2)) if m]
    json.dump(rows, open(os.path.join(HERE, "slip_summary.json"), "w"), indent=1)
    f = lambda v, k, fmt="{:.1f}": "/".join(fmt.format(x[k]) for x in v)
    print("| 판 | 큰 슬립 % (max|λ|>0.5) | 미끄럼 % (>0.1) | λ 최대 | λ RMS | 큰 슬립 중 헛돎 (λ>0) 몫 % | 큰 슬립 중 꼭짓점 0.3 m 안 % | 바퀴별 |λ| 최대 (r1) FL/FR/RL/RR |")
    print("|---|---|---|---|---|---|---|---|")
    for k, v in rows.items():
        g, mode = k.split("_")
        print(f"| {'A' if g == 'PP' else g} {mode} | {f(v, 'spin')} | {f(v, 'slip01')} | {f(v, 'lmax', '{:.2f}')} | {f(v, 'rms', '{:.3f}')} "
              f"| {f(v, 'drive', '{:.0f}')} | {f(v, 'corner', '{:.0f}')} | {' / '.join(f'{x:.2f}' for x in v[0]['wheel'])} |")

    plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                         "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                         "axes.unicode_minus": False})
    COL = {"PP": "#2a78d6", "B": "#eb6834", "C": "#1baf7a"}; NAME = {"PP": "A 퓨어 퍼슛", "B": "B MPC motor_only", "C": "C MPC friction"}
    INK, LIM = "#0b0b0b", "#d03b3b"
    fig, ax = plt.subplots(2, 2, figsize=(13, 7.2), gridspec_kw=dict(width_ratios=[2.3, 1], hspace=0.38, wspace=0.2))
    for r, mode in enumerate(("pass", "stop")):
        a = ax[r, 0]
        for g in ("PP", "B", "C"):
            d = load(f"{g}_{mode}_r1")
            if d is None:
                continue
            tt, lam, _ = d
            a.plot(tt, np.abs(lam).max(1), color=COL[g], lw=1.2, label=NAME[g])
        a.axhline(0.5, color=LIM, lw=0.9, ls="--"); a.text(0.05, 0.53, "큰 슬립 |λ| 0.5", color=LIM, fontsize=8)
        a.set_ylim(0, 2.0); a.set_xlabel("주행 시작부터 t [s]"); a.set_ylabel("max_i |λ_i|  (참값)")
        a.set_title(f"꼭짓점 {mode} — 네 바퀴 중 가장 큰 |λ| (r1 판)", loc="left", fontsize=10, color=INK)
        if r == 0:
            a.legend(loc="upper right", fontsize=8.5, frameon=False, ncol=3)
        # 막대: 헛돎 % (r1, r2 평균 ± 두 판 범위)
        b = ax[r, 1]
        for i, g in enumerate(("PP", "B", "C")):
            v = rows[f"{g}_{mode}"]; vals = [x["spin"] for x in v]
            b.bar(i, np.mean(vals), color=COL[g], width=0.6)
            b.plot([i, i], [min(vals), max(vals)], color=INK, lw=1.0)
            b.text(i, np.mean(vals) + 0.6, f"{np.mean(vals):.1f} %", ha="center", fontsize=8.5, color=INK)
        b.set_xticks([0, 1, 2], ["A", "B", "C"]); b.set_ylim(0, 22); b.set_ylabel("큰 슬립 시간 % (|λ| > 0.5)")
        b.set_title(f"큰 슬립 시간 — {mode} (막대 = 2 판 평균, 선 = 범위)", loc="left", fontsize=10, color=INK)
    fig.suptitle("정사각형 1.5 m · accel 1.5 · v_max 1.5 · tangent · 시뮬 바닥 μ 0.45 — 바퀴 슬립 λ (참값, λ>0 헛돎 · λ<0 끌림)", x=0.06, y=0.98,
                 ha="left", fontsize=11, color=INK)
    fig.savefig(os.path.join(HERE, "fig_square_slip.png"), dpi=140, bbox_inches="tight")
    print("saved")

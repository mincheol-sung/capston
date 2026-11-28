"""슬립 되먹임 판 하나씩: 바퀴별 슬립률 (10 Hz filtfilt) · 슬립 되먹임 Δτ · 기준 속력 + 바퀴별 > 50 / > 25 % 막대.
신호·창은 summary.py 와 같다 (무장 중 · 추정 속력 > 0.05 m/s).
python3 -I plot_sfb_run.py <npz 폴더> <판 이름 ...>"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from scipy.signal import butter, filtfilt

HERE, RUNS = sys.argv[1], sys.argv[2:]
WHEELS = ("FL", "FR", "RL", "RR")
COL = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
INK, INK2, MUTED, GRID, SURF, OUTWIN = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb", "#efeeea"


def load(name):
    A = np.load(os.path.join(HERE, name + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(t)))), L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05)
    X = Lf[w]; big = np.abs(X) > 0.5
    F = A["sfb"]; fw = (F[:, 0] >= t_on) & (F[:, 0] <= t_off)
    on = bool(np.median(F[fw, 2]) > 0.5); scale = None
    ref = A["ref"]; vr = np.hypot(ref[:, 3], ref[:, 4])
    v_off = np.interp(t_off, ref[:, 0], vr)
    sl = A["sl"]; m = (sl[:, 0] >= t_on) & (sl[:, 0] <= t_off)
    dist = np.hypot(*np.diff(sl[m, 1:3], axis=0).T).sum()
    return dict(name=name, t=t - t_on, t_end=t_off - t_on, Lf=Lf, win=w, on=on,
                p50=100 * big.mean(0), p25=100 * (np.abs(X) > 0.25).mean(0), drive=100 * (big & (X > 0)).mean(0),
                mx50=100 * np.mean(np.abs(X).max(1) > 0.5), mx25=100 * np.mean(np.abs(X).max(1) > 0.25),
                tf=F[:, 0] - t_on, dtau=F[:, 7:11], tau_mpc=F[:, 11:15], tau_out=F[:, 15:19], fw=fw, fresh=F[fw, 3].mean(),
                tr=ref[:, 0] - t_on, vr=vr, v_off=v_off, dist=dist, n=int(w.sum()))


R = [load(n) for n in RUNS]
for r in R:
    d = r["dtau"][r["fw"]]; applied = r["tau_out"][r["fw"]] - r["tau_mpc"][r["fw"]]
    print(f"{r['name']}: 되먹임 {'켬' if r['on'] else '끔 (계산만)'} · 바퀴 값 fresh {100 * r['fresh']:.0f} % · 무장 {r['t_end']:.1f} s · SLAM 거리 {r['dist']:.2f} m"
          f" · 해제 때 기준 속력 {r['v_off']:.2f} m/s · 창 {r['n']} 행")
    print("   > 50 %  " + "  ".join(f"{WHEELS[i]} {r['p50'][i]:5.1f}" for i in range(4)) + f"   | 네 바퀴 중 최대 {r['mx50']:.1f}")
    print("   > 25 %  " + "  ".join(f"{WHEELS[i]} {r['p25'][i]:5.1f}" for i in range(4)) + f"   | 네 바퀴 중 최대 {r['mx25']:.1f}")
    print("   > 50 % 중 헛돎 몫 " + "  ".join(f"{WHEELS[i]} {100 * r['drive'][i] / r['p50'][i]:.0f} %" if r['p50'][i] > 0 else f"{WHEELS[i]} -" for i in range(4)))
    print("   Δτ RMS " + "  ".join(f"{WHEELS[i]} {np.sqrt(np.mean(d[:, i] ** 2)):.3f}" for i in range(4)) +
          f"   |Δτ| 최대 {np.abs(d).max():.3f} N·m   실제로 더해진 (낸 τ − τ_MPC) RMS {np.sqrt(np.mean(applied ** 2)):.3f}")

# ── 그림 (판마다 하나, 같은 눈금) ──
fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0])
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "figure.facecolor": SURF, "axes.facecolor": SURF})
t_max = max(r["t_end"] for r in R) + 0.8
dt_lim = max(np.abs(r["dtau"][r["fw"]]).max() for r in R) * 1.1
bar_max = max(r["p25"].max() for r in R)
bar_max = next(s for s in (10, 20, 30, 40, 50, 60, 70, 80, 100) if bar_max * 1.15 <= s)


def shade(ax, r):
    out = ~r["win"]; e = np.flatnonzero(np.diff(np.r_[0, out.astype(int), 0]))
    for a, b in zip(e[::2], e[1::2]):
        ax.axvspan(r["t"][a], r["t"][min(b, len(r["t"]) - 1)], color=OUTWIN, lw=0, zorder=0)


for r in R:
    fig = plt.figure(figsize=(13.5, 9.2), dpi=150)
    gs = fig.add_gridspec(3, 2, height_ratios=[3, 1.8, 1.0], width_ratios=[3.3, 1], hspace=0.38, wspace=0.16,
                          top=0.86, bottom=0.07, left=0.07, right=0.98)
    ax = fig.add_subplot(gs[0, 0]); shade(ax, r)
    for y, ls in ((0.25, (0, (1, 2))), (0.5, (0, (4, 3)))):
        for s in (1, -1):
            ax.axhline(s * y, color=MUTED, lw=0.8, ls=ls, zorder=1)
    ax.axhline(0, color=GRID, lw=0.8, zorder=1)
    for k in range(4):
        ax.plot(r["t"], r["Lf"][:, k], color=COL[k], lw=1.0, zorder=3, label=WHEELS[k])
    ax.set_ylim(-2.05, 2.05); ax.set_yticks([-2, -1, -0.5, 0, 0.5, 1, 2]); ax.set_yticklabels(["−2", "−1", "−0.5", "0", "0.5", "1", "2"])
    ax.set_ylabel("슬립률 (10 Hz LPF)"); ax.set_title("바퀴별 슬립률", loc="left", fontsize=10.5, color=INK, pad=5)
    ax.text(1.0, -1.85, "헛돎 ↑  /  끌림 ↓", transform=ax.get_yaxis_transform(), ha="right", fontsize=8.5, color=MUTED)

    ax2 = fig.add_subplot(gs[1, 0], sharex=ax); shade(ax2, r)
    ax2.axhline(0, color=GRID, lw=0.8)
    for k in range(4):
        ax2.plot(r["tf"], r["dtau"][:, k], color=COL[k], lw=0.9, zorder=3)
    ax2.set_ylim(-dt_lim, dt_lim); ax2.set_ylabel("Δτ [N·m]")
    ax2.set_title("슬립 되먹임 Δτ = −K_v (바퀴 몸체 속도 − 추정기 몸체 속도)" + ("  — 모터 명령에 더함" if r["on"] else "  — 계산만, 모터 명령에 안 더함"),
                  loc="left", fontsize=10.5, color=INK, pad=5)

    ax3 = fig.add_subplot(gs[2, 0], sharex=ax)
    ax3.plot(r["tr"], r["vr"], color=INK2, lw=1.0)
    ax3.set_ylabel("기준 속력 m/s"); ax3.set_ylim(0, max(1.2, r["vr"].max() * 1.15)); ax3.set_xlabel("무장 뒤 시간 [s]")
    for a in (ax, ax2, ax3):
        a.grid(axis="x", color=GRID, lw=0.6); a.set_xlim(-1.0, t_max)
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    plt.setp(ax.get_xticklabels(), visible=False); plt.setp(ax2.get_xticklabels(), visible=False)

    axb = fig.add_subplot(gs[:, 1])
    x = np.arange(4)
    axb.bar(x, r["p25"], width=0.62, color=[c + "40" for c in COL], edgecolor=COL, linewidth=1.0, zorder=2)
    axb.bar(x, r["p50"], width=0.62, color=COL, edgecolor=SURF, linewidth=1.0, zorder=3)
    for i in range(4):
        axb.text(x[i], r["p25"][i] + bar_max * 0.012, f"{r['p25'][i]:.1f}", ha="center", va="bottom", fontsize=8.5, color=INK2)
        axb.text(x[i], r["p50"][i] / 2 if r["p50"][i] > bar_max * 0.06 else r["p50"][i] + bar_max * 0.012, f"{r['p50'][i]:.1f}",
                 ha="center", va="center" if r["p50"][i] > bar_max * 0.06 else "bottom", fontsize=9,
                 color="#ffffff" if r["p50"][i] > bar_max * 0.06 else INK, zorder=4)
    axb.set_xticks(x); axb.set_xticklabels(WHEELS); axb.set_ylim(0, bar_max); axb.set_ylabel("창 안 시간 중 %")
    axb.grid(axis="y", color=GRID, lw=0.6, zorder=0)
    for s in ("top", "right"):
        axb.spines[s].set_visible(False)
    axb.set_title("|슬립률| > 50 % (진한) · > 25 % (옅은)", loc="left", fontsize=10.5, color=INK, pad=5)
    axb.text(0.02, 0.98, f"네 바퀴 중 최대\n> 50 %: {r['mx50']:.1f} %\n> 25 %: {r['mx25']:.1f} %", transform=axb.transAxes, va="top",
             fontsize=9.5, color=INK2)

    tag = "슬립 되먹임 " + ("켬" if r["on"] else "끔 (이름은 on, 인자 slip_fb 빠짐)")
    fig.text(0.07, 0.985, f"{r['name']} — circles8 (width 2.0 · speed 1.0 · accel 1.0) · MPC · {tag}", fontsize=12, color=INK, va="top")
    fig.text(0.07, 0.955, "limit_mode motor_only (바닥 마찰 상한 없음) · 슬립률 10 Hz 1차 Butterworth filtfilt · 창 = 무장 중 · 추정 속력 > 0.05 m/s",
             fontsize=9, color=MUTED, va="top")
    h = [Line2D([], [], color=c, lw=1.6) for c in COL] + [Line2D([], [], color=MUTED, lw=0.8, ls=(0, (1, 2))),
                                                          Line2D([], [], color=MUTED, lw=0.8, ls=(0, (4, 3))), Line2D([], [], color=OUTWIN, lw=6)]
    fig.legend(h, list(WHEELS) + ["±0.25", "±0.5", "창 밖 (무장 전·뒤, 속력 ≤ 0.05 m/s)"], loc="upper left", bbox_to_anchor=(0.065, 0.935),
               ncol=7, frameon=False, fontsize=9.5, labelcolor=INK2, handlelength=1.8, columnspacing=1.4)
    out = os.path.join(HERE, f"slip_{r['name']}.png")
    fig.savefig(out, facecolor=SURF)
    print("저장:", out)

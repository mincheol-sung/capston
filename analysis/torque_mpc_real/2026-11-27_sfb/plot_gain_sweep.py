"""슬립 되먹임 배율 (lqr K_ω 배율) 대비 슬립·추종·토크 — circles8 1.0/1.0, crossing_blend 0.0, 11-27.
계열 ① motor_only (17:37~17:59 배율 훑기 + 되먹임 끔 162411), ② friction (오후 판). PP 두 판은 가로 띠.
python3 -I plot_gain_sweep.py <npz 폴더>"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from scipy.signal import butter, filtfilt

HERE = sys.argv[1]
SERIES = [
    ("MPC · motor_only", "#2a78d6", "o", "-", [(0.0, "162411_sfb_on_r2"), (1.3, "175559_circle1"), (1.4, "175853_circle1"), (1.5, "173707_circle1"),
                                             (1.5, "175358_circle1"), (1.7, "175505_circle1"), (2.5, "175248_circle1"), (5.0, "175124_circle1")]),
    ("MPC · friction", "#1baf7a", "s", "--", [(0.0, "165139_sfbw_lqr025_r3"), (0.25, "164829_sfbw_lqr025_r1"), (1.0, "165723_sfbw_lqr1_r1"),
                                             (2.0, "171400_circle1"), (2.0, "171546_circle1")]),
]
PP = ["165935_pp_r1", "173001_pp_r1"]
W = ("FL", "FR", "RL", "RR")
INK, INK2, MUTED, GRID, SURF, PPC = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb", "#eda100"


def metrics(name):
    A = np.load(os.path.join(HERE, name + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(t)))), L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05); X = np.abs(Lf[w])
    mm = (A["mot"][:, 0] >= t_on) & (A["mot"][:, 0] <= t_off); M = A["mot"][mm, 2:6]
    ref = A["ref"]; vr = np.hypot(ref[:, 3], ref[:, 4]); v_off = np.interp(t_off, ref[:, 0], vr)
    T = np.load(os.path.join(HERE, name + "_trk.npz")); tw = T["win"]
    rms = lambda a: float(np.sqrt(np.mean(a[tw] ** 2)))
    dt_app = np.nan
    if "sfb" in A.files and len(A["sfb"]):
        F = A["sfb"]; fm = (F[:, 0] >= t_on) & (F[:, 0] <= t_off)
        dt_app = float(np.sqrt(np.mean((F[fm, 15:19] - F[fm, 11:15]) ** 2)))
    return dict(p50=100 * (X > 0.5).mean(0), m50=100 * np.mean(X.max(1) > 0.5), m25=100 * np.mean(X.max(1) > 0.25),
                pos=100 * rms(T["ep"]), head=rms(T["eh"]), vel=100 * rms(T["ev"]), yaw=rms(T["ew"]),
                dtick=1e3 * float(np.sqrt(np.mean(np.diff(M, axis=0) ** 2))), tmax=float(np.abs(M).max()), dtau=1e3 * dt_app if np.isfinite(dt_app) else np.nan,
                v_off=v_off, armed=t_off - t_on)


D = [(lab, col, mk, ls, [(g, n, metrics(n)) for g, n in runs]) for lab, col, mk, ls, runs in SERIES]
P = [metrics(n) for n in PP]
print(f"{'계열':16s} {'배율':>4s} {'판':22s} {'>50 최대':>7s} {'FL':>5s} {'FR':>5s} {'RL':>5s} {'RR':>5s} {'위치':>6s} {'속도':>6s} {'헤딩':>5s} {'요속':>6s} {'틱Δτ':>6s} {'더한Δτ':>6s} {'해제때 v_ref':>8s}")
for lab, col, mk, ls, rs in D:
    for g, n, m in rs:
        print(f"{lab:16s} {g:4.2f} {n:22s} {m['m50']:7.1f} " + " ".join(f"{m['p50'][i]:5.1f}" for i in range(4)) +
              f" {m['pos']:6.2f} {m['vel']:6.2f} {m['head']:5.2f} {m['yaw']:6.3f} {m['dtick']:6.1f} {m['dtau']:6.1f} {m['v_off']:8.2f}")
for n, m in zip(PP, P):
    print(f"{'PP':16s} {'-':>4s} {n:22s} {m['m50']:7.1f} " + " ".join(f"{m['p50'][i]:5.1f}" for i in range(4)) +
          f" {m['pos']:6.2f} {m['vel']:6.2f} {m['head']:5.2f} {m['yaw']:6.3f} {m['dtick']:6.1f} {'':>6s} {m['v_off']:8.2f}")

# ── 그림 ──
fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0])
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "figure.facecolor": SURF, "axes.facecolor": SURF})
PANELS = [("m50", "네 바퀴 중 최대 |슬립률| > 50 % [시간 %]", 70, "{:.0f}"),
          ("p50_FL", "FL (앞 왼) > 50 % [시간 %]", 35, "{:.0f}"), ("p50_FR", "FR (앞 오) > 50 % [시간 %]", 35, "{:.0f}"),
          ("p50_RR", "RR (뒤 오) > 50 % [시간 %]", 35, "{:.0f}"),
          ("pos", "위치 RMS [cm]", 20, "{:.1f}"), ("vel", "속도 벡터 RMS [cm/s]", 40, "{:.0f}"), ("head", "헤딩 RMS [°]", 6, "{:.1f}"),
          ("yaw", "요 속도 RMS [rad/s]", 0.6, "{:.2f}"), ("dtick", "틱간 모터 명령 변화 RMS [mN·m]", 80, "{:.0f}")]
fig, axes = plt.subplots(3, 3, figsize=(15, 12), dpi=150, gridspec_kw=dict(hspace=0.42, wspace=0.25))
XMAX = 5.3


def val(m, key):
    return m["p50"][W.index(key[4:])] if key.startswith("p50_") else m[key]


for ax, (key, title, ymax, fmt) in zip(axes.ravel(), PANELS):
    pv = [val(m, key) for m in P]
    ax.axhspan(min(pv), max(pv), color=PPC, alpha=0.22, lw=0, zorder=0)
    ax.axhline(np.mean(pv), color=PPC, lw=1.2, zorder=1)
    for lab, col, mk, ls, rs in D:
        gs_ = sorted(set(g for g, _, _ in rs))
        mean = [np.mean([val(m, key) for g, _, m in rs if g == gg]) for gg in gs_]
        ax.plot(gs_, np.minimum(mean, ymax), color=col, ls=ls, lw=1.4, zorder=2)
        for g, n, m in rs:
            v = val(m, key)
            if v > ymax:
                ax.annotate(f"{fmt.format(v)} ↑", (g, ymax), xytext=(0, -12), textcoords="offset points", ha="center", fontsize=8.5, color=col)
                ax.plot(g, ymax, marker="^", color=col, ms=7, zorder=3)
            else:
                ax.plot(g, v, marker=mk, color=col, ms=6.5, mec=SURF, mew=1.0, zorder=3)
    ax.set_xlim(-0.2, XMAX); ax.set_ylim(0, ymax); ax.set_xticks([0, 0.25, 1, 1.5, 2, 2.5, 5]); ax.set_xticklabels(["0", ".25", "1", "1.5", "2", "2.5", "5"])
    ax.grid(color=GRID, lw=0.6, zorder=0)
    for s_ in ("top", "right"):
        ax.spines[s_].set_visible(False)
    ax.set_title(title, loc="left", fontsize=10.5, color=INK, pad=6)
for ax in axes[-1]:
    ax.set_xlabel("슬립 되먹임 배율 (lqr K_ω × 배율, 1 = 대각 0.0162 N·m/(rad/s))")
h = [Line2D([], [], color=c, ls=ls, marker=mk, lw=1.4, ms=6.5) for _, c, mk, ls, _ in SERIES] + [Line2D([], [], color=PPC, lw=6, alpha=0.4)]
fig.text(0.06, 0.985, "슬립 되먹임 배율 훑기 — circles8 width 2.0 · speed 1.0 · accel 1.0 · crossing_blend 0.0 · 11-27, 판마다 한 점 (같은 배율 두 판은 선이 평균)",
         fontsize=11.5, color=INK, va="top")
fig.text(0.06, 0.962, "슬립률 10 Hz 1차 Butterworth filtfilt, 창 = 무장 중 · 추정 속력 > 0.05 m/s.  추종 오차 = 기준 지평을 펼친 100 Hz 기준 − 추정기, 창 = 기준 속력 > 0.05 m/s "
         "(제어기 status 와 같은 값). 0 = 되먹임 끔", fontsize=8.5, color=MUTED, va="top")
fig.legend(h, [s[0] for s in SERIES] + ["PP 두 판 (16:59 · 17:30) 범위 · 평균"], loc="upper left", bbox_to_anchor=(0.055, 0.945), ncol=3, frameon=False,
           fontsize=10, labelcolor=INK2)
fig.subplots_adjust(top=0.9, bottom=0.06, left=0.06, right=0.98)
out = os.path.join(HERE, "slip_fb_gain_sweep.png")
fig.savefig(out, facecolor=SURF)
print("저장:", out)

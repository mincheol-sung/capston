"""MPC (+ 슬립 되먹임) vs PP — 같은 기준 (circles8 2 바퀴 · speed 0.7 ± 0.5 변조): 바퀴별 슬립 + 추종 오차 (위치·속도·헤딩·요 속도) + 시계열.
슬립은 *.npz (read_bags), 추종 오차는 *_trk.npz (track_err.py, 기준 지평을 펼친 dense 기준 − 추정기, 창 = 기준 움직이는 동안).
python3 -I plot_mpc_pp_track.py <npz 폴더>"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
from scipy.signal import butter, filtfilt

HERE = sys.argv[1]
RUNS = [("MPC + 되먹임 lqr 1.5 (174149, motor_only)", "174149_circle1", "#2a78d6"), ("PP 고정 L_d 0.3 m (174419)", "174419_pp_r1", "#eda100")]
W = ("FL", "FR", "RL", "RR")
INK, INK2, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb"


def slip(name):
    A = np.load(os.path.join(HERE, name + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(t)))), L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5])); wz = np.interp(t, od[:, 0], od[:, 6])
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05); X = np.abs(Lf[w])
    rr_r = 100 * np.mean(np.abs(Lf[w & (wz < -0.7), 3]) > 0.5); rl_l = 100 * np.mean(np.abs(Lf[w & (wz > 0.7), 2]) > 0.5)
    rr25 = 100 * np.mean(np.abs(Lf[w & (wz < -0.7), 3]) > 0.25); rl25 = 100 * np.mean(np.abs(Lf[w & (wz > 0.7), 2]) > 0.25)
    return dict(p50=100 * (X > 0.5).mean(0), p25=100 * (X > 0.25).mean(0), m50=100 * np.mean(X.max(1) > 0.5), m25=100 * np.mean(X.max(1) > 0.25),
                rr_r=rr_r, rl_l=rl_l, rr25=rr25, rl25=rl25)


R = []
for lab, nm, col in RUNS:
    s = slip(nm); T = np.load(os.path.join(HERE, nm + "_trk.npz")); w = T["win"]
    rms = lambda a: float(np.sqrt(np.mean(a[w] ** 2)))
    tr = dict(pos=100 * rms(T["ep"]), pos_max=100 * T["ep"][w].max(), head=rms(T["eh"]), head_max=np.abs(T["eh"][w]).max(),
              vel=100 * rms(T["ev"]), vel_max=100 * T["ev"][w].max(), spd=100 * rms(T["espd"]), yaw=rms(T["ew"]), yaw_max=np.abs(T["ew"][w]).max())
    R.append((lab, nm, col, s, tr, T))
    print(f"{lab}")
    print("   > 50 % " + " ".join(f"{W[i]} {s['p50'][i]:4.1f}" for i in range(4)) + f" | 최대 {s['m50']:.1f}   > 25 % " +
          " ".join(f"{W[i]} {s['p25'][i]:4.1f}" for i in range(4)) + f" | 최대 {s['m25']:.1f}")
    print(f"   원별: RR @ 오른쪽 원 > 50 % {s['rr_r']:.1f} (> 25 % {s['rr25']:.1f}) · RL @ 왼쪽 원 > 50 % {s['rl_l']:.1f} (> 25 % {s['rl25']:.1f})")
    print(f"   위치 RMS {tr['pos']:.2f} cm (최대 {tr['pos_max']:.2f}) · 속도 벡터 RMS {tr['vel']:.2f} cm/s (최대 {tr['vel_max']:.1f}) · 속력 차 RMS {tr['spd']:.2f} cm/s"
          f" · 헤딩 RMS {tr['head']:.2f}° (최대 {tr['head_max']:.2f}) · 요 속도 RMS {tr['yaw']:.3f} rad/s (최대 {tr['yaw_max']:.2f})")

fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0])
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "figure.facecolor": SURF, "axes.facecolor": SURF})
fig = plt.figure(figsize=(14, 12), dpi=150)
gs = fig.add_gridspec(3, 4, height_ratios=[1.2, 1.0, 1.25], hspace=0.55, wspace=0.35, top=0.9, bottom=0.05, left=0.06, right=0.98)


def clean(ax):
    for s_ in ("top", "right"):
        ax.spines[s_].set_visible(False)
    ax.grid(axis="y", color=GRID, lw=0.6, zorder=0)


# ① 바퀴별 슬립
for k, (key, title, ymax) in enumerate((("p50", "|슬립률| > 50 % 인 시간 비율", 10), ("p25", "|슬립률| > 25 % 인 시간 비율 (보조)", 30))):
    ax = fig.add_subplot(gs[0, 2 * k:2 * k + 2]); x = np.arange(4); bw = 0.36
    for j, (lab, nm, col, s, tr, T) in enumerate(R):
        xs = x + (j - 0.5) * (bw + 0.02)
        ax.bar(xs, s[key], width=bw, color=col, edgecolor=SURF, zorder=2)
        for i in range(4):
            ax.text(xs[i], s[key][i] + ymax * 0.015, f"{s[key][i]:.1f}", ha="center", va="bottom", fontsize=8.5, color=INK2)
    ax.set_xticks(x); ax.set_xticklabels(["FL (앞 왼)", "FR (앞 오)", "RL (뒤 왼)", "RR (뒤 오)"]); ax.set_ylim(0, ymax); ax.set_ylabel("창 안 시간 중 %")
    clean(ax); ax.set_title(title + f"   (네 바퀴 중 최대: " + " / ".join(f"{r_[3]['m50' if key == 'p50' else 'm25']:.1f}" for r_ in R) + ")",
                            loc="left", fontsize=10.5, color=INK, pad=6)

# ② 추종 오차 RMS
mets = [("pos", "pos_max", "위치 RMS [cm]"), ("vel", "vel_max", "속도 벡터 RMS [cm/s]"), ("head", "head_max", "헤딩 RMS [°]"), ("yaw", "yaw_max", "요 속도 RMS [rad/s]")]
for k, (key, kmax, title) in enumerate(mets):
    ax = fig.add_subplot(gs[1, k])
    vals = [r_[4][key] for r_ in R]
    ax.bar(range(len(R)), vals, width=0.6, color=[r_[2] for r_ in R], edgecolor=SURF, zorder=2)
    top = max(vals) * 1.35
    for j, r_ in enumerate(R):
        fmt = "{:.3f}" if key == "yaw" else "{:.2f}"
        ax.text(j, vals[j] + top * 0.02, fmt.format(vals[j]) + f"\n(최대 {r_[4][kmax]:.{2 if key == 'yaw' else 1}f})", ha="center", va="bottom", fontsize=8.5, color=INK2)
    ax.set_ylim(0, top); ax.set_xticks([]); clean(ax); ax.set_title(title, loc="left", fontsize=10.5, color=INK, pad=6)

# ③ 시계열: 위치 오차·헤딩 오차 + 기준 속력
for k, (key, lab_y, sc) in enumerate((("ep", "위치 오차 [cm]", 100.0), ("eh", "헤딩 오차 [°]", 1.0))):
    ax = fig.add_subplot(gs[2, 2 * k:2 * k + 2])
    for lab, nm, col, s, tr, T in R:
        tt = T["t"] - T["t"][T["win"]][0]
        ax.plot(tt, sc * T[key] if key == "ep" else T[key], color=col, lw=1.0)
    T0 = R[0][5]; tt0 = T0["t"] - T0["t"][T0["win"]][0]
    ax2 = ax.twinx(); ax2.plot(tt0, T0["vref"], color=MUTED, lw=0.9, ls=(0, (3, 2))); ax2.set_ylim(0, 3.2); ax2.set_yticks([0, 0.5, 1.0])
    ax2.set_ylabel("기준 속력 m/s (점선)", color=MUTED); ax2.tick_params(colors=MUTED)
    for s_ in ("top",):
        ax.spines[s_].set_visible(False); ax2.spines[s_].set_visible(False)
    ax.grid(axis="x", color=GRID, lw=0.6); ax.set_xlim(-0.5, tt0[T0["win"]][-1] + 0.5); ax.set_xlabel("기준 출발 뒤 시간 [s]"); ax.set_ylabel(lab_y)
    ax.set_title(lab_y.split(" [")[0] + " 시계열 (기준 − 추정기)", loc="left", fontsize=10.5, color=INK, pad=6)

fig.text(0.06, 0.985, "같은 기준: circles8 width 2.0 · 2 바퀴 · speed 0.7 ± 0.5 (sin 2 주기, 위상 −90°) · accel 1.0 · crossing_blend 0.0 — 각 한 판",
         fontsize=11.5, color=INK, va="top")
fig.text(0.06, 0.962, "슬립률 10 Hz 1차 Butterworth filtfilt, 창 = 무장 중 · 추정 속력 > 0.05 m/s.  추종 오차: 기준 지평을 펼친 100 Hz 기준 − 추정기 (/state_estimator/odom), "
         "창 = 기준 속력 > 0.05 m/s. 위치·헤딩은 제어기 status 와 같은 값", fontsize=8.5, color=MUTED, va="top")
fig.legend([Patch(color=c) for _, _, c in RUNS], [l for l, _, _ in RUNS], loc="upper left", bbox_to_anchor=(0.055, 0.945), ncol=2, frameon=False,
           fontsize=10, labelcolor=INK2)
out = os.path.join(HERE, "mpc_pp_slip_track_174149_174419.png")
fig.savefig(out, facecolor=SURF)
print("저장:", out)

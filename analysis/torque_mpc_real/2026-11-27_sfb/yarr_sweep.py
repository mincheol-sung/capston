"""11-27 19:24~19:44 yarr_mpc 배율 훑기 (friction · circles8 2.0 · 1.0/1.0 · lqr · blend 0.0) — 배율별 슬립·추종·토크 + 판 검사.
배율은 bag 이름이 아니라 run_info/args.txt 에서 읽는다. npz 는 read_bags.py, *_trk.npz 는 track_err.py.
python3 -I yarr_sweep.py <npz 폴더> <bag 날짜 폴더> [제외할 판 ...]"""
import os
import re
import sys
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from scipy.signal import butter, filtfilt

HERE, BAGS = sys.argv[1], sys.argv[2]
EXCL = set(sys.argv[3:])
W = ("FL", "FR", "RL", "RR")
r_w = 0.04
REF_OFF = "165139_sfbw_lqr025_r3"            # 같은 friction · blend 0.0 · 되먹임 끔 (16:51, 한 판) — 참고선
INK, INK2, MUTED, GRID, SURF, C1, OFFC = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb", "#1baf7a", "#8a8984"


def args_of(name):
    a = open(os.path.join(BAGS, name, "run_info", "args.txt")).read()
    g = lambda k, d="": (re.search(rf"\b{k}:=(\S+)", a) or [None, d])[1]
    return dict(scale=float(g("slip_fb_scale", "nan")), limit=g("limit_mode", "-"), sfb=g("slip_fb", "-"), blend=g("crossing_blend_m", "0.0"))


def metrics(name):
    A = np.load(os.path.join(HERE, name + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(t)))), L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05); X = np.abs(Lf[w]); big = X.max(1) > 0.5
    spin = 100 * np.mean((Lf[w][big] > 0.5).any(1)) if big.any() else np.nan       # > 50 % 순간 중 헛돎 (λ > 0.5) 이 낀 몫
    mm = (A["mot"][:, 0] >= t_on) & (A["mot"][:, 0] <= t_off); M = A["mot"][mm, 2:6]
    ref = A["ref"]; vr = np.hypot(ref[:, 3], ref[:, 4]); v_off = np.interp(t_off, ref[:, 0], vr)
    ws = A["ws"]; wm = (ws[:, 0] >= t_on) & (ws[:, 0] <= t_off); temp = float(np.nanmean(ws[wm, 13:17])) if ws.shape[1] >= 17 else np.nan
    sl = A["sl"]; slm = (sl[:, 0] >= t_on - 1) & (sl[:, 0] <= t_off); jump = float(np.hypot(*np.diff(sl[slm, 1:3], axis=0).T).max()) if slm.sum() > 2 else np.nan
    T = np.load(os.path.join(HERE, name + "_trk.npz")); tw = T["win"]
    rms = lambda a: float(np.sqrt(np.mean(a[tw] ** 2)))
    dtau = vslip = np.nan
    if "sfb" in A.files and len(A["sfb"]):
        F = A["sfb"]; fm = (F[:, 0] >= t_on) & (F[:, 0] <= t_off) & (np.interp(F[:, 0], od[:, 0], np.hypot(od[:, 4], od[:, 5])) > 0.05)
        dtau = 1e3 * float(np.sqrt(np.mean((F[fm, 15:19] - F[fm, 11:15]) ** 2)))
        if F.shape[1] >= 23:
            vslip = 100 * r_w * float(np.sqrt(np.mean(F[fm, 19:23] ** 2)))              # 미끄럼 속도 r·e_ω RMS [cm/s] (네 바퀴 묶어)
    return dict(p50=100 * (X > 0.5).mean(0), m50=100 * big.mean(), m25=100 * np.mean(X.max(1) > 0.25), lam=100 * float(X.max(1).mean()), spin=spin,
                vslip=vslip, pos=100 * rms(T["ep"]), pos_max=100 * float(T["ep"][tw].max()), head=rms(T["eh"]), vel=100 * rms(T["ev"]), yaw=rms(T["ew"]),
                dtick=1e3 * float(np.sqrt(np.mean(np.diff(M, axis=0) ** 2))), dtau=dtau, v_off=v_off, armed=t_off - t_on,
                ref_s=float(tw.sum() * np.median(np.diff(T["t"]))), n_arm=int(len(arm)), temp=temp, jump=100 * jump)


names = sorted(os.path.basename(p) for p in glob.glob(os.path.join(BAGS, "*_yarr_mpc")))
R = []
print(f"{'판':16s} {'배율':>4s} {'상한':8s} {'무장s':>5s} {'기준s':>5s} {'해제v':>5s} {'온도':>5s} {'SLAM튐':>6s} | {'>50최대':>6s} {'>25최대':>6s} "
      f"{'평균λ':>5s} {'미끄럼':>6s} {'헛돎몫':>5s} " + " ".join(f"{w:>5s}" for w in W) + f" | {'위치':>5s} {'최대':>5s} {'헤딩':>4s} {'속도':>5s} {'요속':>5s} | {'틱Δτ':>5s} {'더한Δτ':>6s}")
for n in names:
    a = args_of(n); m = metrics(n)
    bad = n in EXCL or n.split("_")[0] in EXCL
    R.append((n, a, m, bad))
    print(f"{n:16s} {a['scale']:4.1f} {a['limit']:8s} {m['armed']:5.1f} {m['ref_s']:5.1f} {m['v_off']:5.2f} {m['temp']:5.1f} {m['jump']:6.1f} | {m['m50']:6.1f} {m['m25']:6.1f} "
          f"{m['lam']:5.1f} {m['vslip']:6.2f} {m['spin']:5.0f} " + " ".join(f"{m['p50'][i]:5.1f}" for i in range(4)) +
          f" | {m['pos']:5.2f} {m['pos_max']:5.1f} {m['head']:4.2f} {m['vel']:5.2f} {m['yaw']:5.3f} | {m['dtick']:5.1f} {m['dtau']:6.1f}" + ("   ← 제외" if bad else ""))
off = metrics(REF_OFF) if os.path.exists(os.path.join(HERE, REF_OFF + ".npz")) else None

KEYS = [("m50", "{:.1f}"), ("m25", "{:.1f}"), ("lam", "{:.1f}"), ("vslip", "{:.2f}"), ("pos", "{:.2f}"), ("head", "{:.2f}"), ("vel", "{:.2f}"), ("yaw", "{:.3f}"), ("dtick", "{:.1f}")]
scales = sorted(set(a["scale"] for _, a, _, b in R if not b))
print("\n배율별 평균 (제외 판 뺌)   [최소 ~ 최대]")
for s in scales:
    ms = [m for _, a, m, b in R if not b and a["scale"] == s]
    print(f"  {s:.1f} (n={len(ms)}) " + "  ".join(f"{k} {fmt.format(np.mean([m[k] for m in ms]))} [{fmt.format(min(m[k] for m in ms))}~{fmt.format(max(m[k] for m in ms))}]" for k, fmt in KEYS))
if off:
    print(f"  참고 되먹임 끔 {REF_OFF}: " + "  ".join(f"{k} {fmt.format(off[k])}" for k, fmt in KEYS))

from scipy import stats
V = [(a["scale"], m) for _, a, m, b in R if not b]
print(f"\n배율에 대한 직선 회귀 (판 {len(V)} 개, 기울기 = 배율 0.1 당 변화, p = 기울기 0 가설)   + Kruskal–Wallis (배율 6 묶음 차이)")
for k, fmt in KEYS:
    x = np.array([s for s, _ in V]); y = np.array([m[k] for _, m in V]); lr = stats.linregress(x, y)
    kw = stats.kruskal(*[y[x == s] for s in scales])
    print(f"  {k:6s} 1.0→1.5 적합 {fmt.format(lr.intercept + lr.slope * 1.0)} → {fmt.format(lr.intercept + lr.slope * 1.5)}   기울기 {0.1 * lr.slope:+.3f}/0.1 "
          f"(95 % ±{0.1 * 1.96 * lr.stderr:.3f})  p {lr.pvalue:.3f}  R² {lr.rvalue ** 2:.2f}   KW p {kw.pvalue:.3f}")
rd = {}                                                                          # 바퀴 = 1.0 부터 다시 시작하는 묶음 (오름차순으로 돌림)
k_ = -1
for n, a, m, b in R:
    if a["scale"] == 1.0:
        k_ += 1
    if not b and k_ >= 0:
        rd.setdefault(k_, []).append((a["scale"], m))
print("\n바퀴별 (매 바퀴 1.0→1.5 오름차순) — 같은 바퀴 안 1.0 대비 차이: vslip [cm/s] · pos [cm] · m50 [%]")
for k_, lst in rd.items():
    base = [m for s, m in lst if s == 1.0][0]
    print(f"  바퀴 {k_ + 1}: " + "  ".join(f"{s:.1f} {m['vslip'] - base['vslip']:+.2f}/{m['pos'] - base['pos']:+.2f}/{m['m50'] - base['m50']:+.1f}" for s, m in lst))

from scipy import stats
V = [(a["scale"], m) for _, a, m, b in R if not b]
print(f"\n배율에 대한 직선 회귀 (판 {len(V)} 개, 기울기 = 배율 0.1 당 변화, p = 기울기 0 가설)   + Kruskal–Wallis (배율 6 묶음 차이)")
for k, fmt in KEYS:
    x = np.array([s for s, _ in V]); y = np.array([m[k] for _, m in V]); lr = stats.linregress(x, y)
    kw = stats.kruskal(*[y[x == s] for s in scales])
    print(f"  {k:6s} 1.0→1.5 적합 {fmt.format(lr.intercept + lr.slope * 1.0)} → {fmt.format(lr.intercept + lr.slope * 1.5)}   기울기 {0.1 * lr.slope:+.3f}/0.1 "
          f"(95 % ±{0.1 * 1.96 * lr.stderr:.3f})  p {lr.pvalue:.3f}  R² {lr.rvalue ** 2:.2f}   KW p {kw.pvalue:.3f}")
rd = {}                                                                          # 바퀴 = 1.0 부터 다시 시작하는 묶음 (오름차순으로 돌림)
k_ = -1
for n, a, m, b in R:
    if a["scale"] == 1.0:
        k_ += 1
    if not b and k_ >= 0:
        rd.setdefault(k_, []).append((a["scale"], m))
print("\n바퀴별 (매 바퀴 1.0→1.5 오름차순) — 같은 바퀴 안 1.0 대비 차이: vslip [cm/s] · pos [cm] · m50 [%]")
for k_, lst in rd.items():
    base = [m for s, m in lst if s == 1.0][0]
    print(f"  바퀴 {k_ + 1}: " + "  ".join(f"{s:.1f} {m['vslip'] - base['vslip']:+.2f}/{m['pos'] - base['pos']:+.2f}/{m['m50'] - base['m50']:+.1f}" for s, m in lst))

# ── 그림 ──
fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0])
    plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "figure.facecolor": SURF, "axes.facecolor": SURF})
PANELS = [("m50", "네 바퀴 중 최대 |슬립률| > 50 % [시간 %]"), ("m25", "네 바퀴 중 최대 |슬립률| > 25 % [시간 %]"),
          ("lam", "네 바퀴 중 최대 |슬립률| 평균 [%]"), ("vslip", "미끄럼 속도 r·(ω − ω_nom) RMS [cm/s]"),
          ("pos", "위치 RMS [cm]"), ("head", "헤딩 RMS [°]"), ("vel", "속도 벡터 RMS [cm/s]"), ("yaw", "요 속도 RMS [rad/s]"),
          ("dtick", "틱간 모터 명령 변화 RMS [mN·m]")]
fig, axes = plt.subplots(3, 3, figsize=(15, 12), dpi=150, gridspec_kw=dict(hspace=0.42, wspace=0.25))
for ax, (key, title) in zip(axes.ravel(), PANELS):
    vals_all = []
    for s in scales:
        v = [m[key] for _, a, m, b in R if not b and a["scale"] == s]; vals_all += v
        ax.plot([s] * len(v), v, "o", color=C1, ms=6, mec=SURF, mew=1.0, alpha=0.75, zorder=3)
    mean = [np.mean([m[key] for _, a, m, b in R if not b and a["scale"] == s]) for s in scales]
    ax.plot(scales, mean, color=INK, lw=1.6, marker="_", ms=16, mew=2.2, zorder=4)
    for _, a, m, b in R:
        if b and np.isfinite(m[key]):
            ax.plot(a["scale"], m[key], "x", color="#d6453d", ms=7, mew=1.6, zorder=3); vals_all.append(m[key])
    top = max(vals_all) * 1.15
    if off and np.isfinite(off[key]):
        if off[key] <= top * 1.6:
            top = max(top, off[key] * 1.1)
            ax.axhline(off[key], color=OFFC, lw=1.1, ls=(0, (4, 3)), zorder=1)
        else:
            ax.annotate(f"끔 {off[key]:.0f} ↑", (1.0, top), xytext=(2, -12), textcoords="offset points", fontsize=8.5, color=OFFC)
    ax.set_ylim(0, top); ax.set_xlim(0.94, 1.56); ax.set_xticks(scales)
    ax.grid(color=GRID, lw=0.6, zorder=0)
    for s_ in ("top", "right"):
        ax.spines[s_].set_visible(False)
    ax.set_title(title, loc="left", fontsize=10.5, color=INK, pad=6)
for ax in axes[-1]:
    ax.set_xlabel("슬립 되먹임 배율 (lqr K_ω × 배율, 1 = 대각 0.0162 N·m/(rad/s))")
h = [Line2D([], [], color=C1, marker="o", ls="", ms=6), Line2D([], [], color=INK, lw=1.6, marker="_", ms=14, mew=2), Line2D([], [], color="#d6453d", marker="x", ls="", ms=7, mew=1.6),
     Line2D([], [], color=OFFC, lw=1.1, ls=(0, (4, 3)))]
fig.text(0.06, 0.985, "슬립 되먹임 배율 훑기 — friction 상한 · circles8 width 2.0 · speed 1.0 · accel 1.0 · crossing_blend 0.0 · 11-27 19:24~19:44 (순서 섞어 3 바퀴)",
         fontsize=11.5, color=INK, va="top")
fig.text(0.06, 0.962, "슬립률 10 Hz 1차 Butterworth filtfilt, 창 = 무장 중 · 추정 속력 > 0.05 m/s.  추종 오차 = 기준 지평을 펼친 100 Hz 기준 − 추정기, 창 = 기준 속력 > 0.05 m/s. "
         "배율은 run_info/args.txt 에서 읽음", fontsize=8.5, color=MUTED, va="top")
fig.legend(h, ["판 하나", "배율별 평균", "제외한 판 (중단·이상)", f"되먹임 끔 참고 ({REF_OFF[:6]}, 16:51 한 판)"], loc="upper left", bbox_to_anchor=(0.055, 0.945), ncol=4,
           frameon=False, fontsize=10, labelcolor=INK2)
fig.subplots_adjust(top=0.9, bottom=0.06, left=0.06, right=0.98)
out = os.path.join(HERE, "yarr_scale_sweep.png")
fig.savefig(out, facecolor=SURF)
print("저장:", out)

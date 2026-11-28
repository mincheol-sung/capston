"""q_laps.py 결과 모아 보기: 후보 (mpc_W) 별 판·바퀴 지표, 평균, 바퀴/판 편차 분해, 기준 대비 차이 (Welch · 시간상 가까운 기준 판과 짝), 판정 기준, 그림.
판정 기준 (2026-11-27 사용자와 정함): 위치 RMS 개선 (3 판 평균 차가 2.8σ 넘거나 짝 비교가 모두 같은 방향), 큰 슬립 +2 %p 이내,
미끄럼 속도 +0.6 cm/s 이내, 모터 명령 떨림 25 mN·m 이하.
python3 -I q_report.py <그림 이름> <rows.npy ...> [--exclude 판이름 ...]"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
argv = sys.argv[1:]
EXCL = set(argv[argv.index("--exclude") + 1:]) if "--exclude" in argv else set()
argv = argv[:argv.index("--exclude")] if "--exclude" in argv else argv
FIG, FILES = argv[0], argv[1:]
rows = [r for f in FILES for r in np.load(f, allow_pickle=True) if r[0] not in EXCL and r[0].split("_")[0] not in EXCL]
BASE = "기준(yaml)"
cands = [BASE] + sorted({r[1] for r in rows} - {BASE})
LAB = {BASE: "기준 20,20,10,2,2,1"}
runs = {}
for n, W, lap, m in rows:
    runs.setdefault((W, n), {})[lap] = m
order = sorted({n for (_, n) in runs}, key=lambda n: runs[[k for k in runs if k[1] == n][0]][0]["t0"])
KEYS = [("pos", "위치 RMS [cm]", "{:.2f}"), ("along", "진행 늦음 평균 [cm]", "{:+.2f}"), ("cross_r", "옆 RMS [cm]", "{:.2f}"), ("head", "헤딩 RMS [°]", "{:.2f}"),
        ("vel", "속도 RMS [cm/s]", "{:.2f}"), ("yaw", "요 속도 RMS [rad/s]", "{:.3f}"), ("m50", "큰 슬립 >50 % [%]", "{:.1f}"), ("m25", "중간 슬립 >25 % [%]", "{:.1f}"),
        ("vslip", "미끄럼 속도 RMS [cm/s]", "{:.2f}"), ("dtick", "모터 명령 떨림 [mN·m]", "{:.1f}")]
GUARD = {"m50": 2.0, "vslip": 0.6, "dtick_abs": 25.0}

print("① 판별 (바퀴 0 = 판 전체, 2~N = 첫 바퀴 뺀 것)")
print(f"  {'순서':>3s} {'판':16s} {'후보':26s} {'구간':6s} {'길이s':>5s} {'튐':>2s} " + " ".join(f"{k:>7s}" for k, _, _ in KEYS))
for i, n in enumerate(order):
    W = [k for k in runs if k[1] == n][0][0]; d = runs[(W, n)]
    for lap in sorted(d, key=lambda x: (x != 0, x != -1, x)):
        m = d[lap]; seg = "판 전체" if lap == 0 else ("2~N" if lap == -1 else f"{lap} 바퀴")
        print(f"  {i + 1:3d} {n:16s} {LAB.get(W, W):26s} {seg:6s} {m['dur']:5.1f} {m['jumps']:2d} " + " ".join(f"{f.format(m[k]):>7s}" for k, _, f in KEYS)
              + ("" if m["done"] else "  ← 끝까지 못 감"))


def per_run(W, key, lap=0):
    return np.array([runs[(W, n)][lap][key] for n in order if (W, n) in runs and lap in runs[(W, n)]])


for lap, title in ((0, "판 전체"), (-1, "첫 바퀴 뺀 2~N 바퀴")):
    print(f"\n② 후보 평균 ({title}) — 판 평균의 평균, 괄호 = 기준 대비 차 · Welch p")
    for W in cands:
        line = f"  {LAB.get(W, W):26s} n={len(per_run(W, 'pos', lap))} "
        for k, _, f in KEYS:
            v = per_run(W, k, lap)
            if not len(v):
                continue
            s = f"{k} {f.format(v.mean())}"
            if W != BASE and len(per_run(BASE, k, lap)) >= 2 and len(v) >= 2:
                b = per_run(BASE, k, lap); s += f"({v.mean() - b.mean():+.2f} p{stats.ttest_ind(v, b, equal_var=False).pvalue:.2f})"
            line += s + "  "
        print(line)

print("\n③ 편차 분해 (바퀴 1..N 지표로): 바퀴끼리 σ (판 안) · 판끼리 σ (판 평균에서 바퀴 몫 뺀 것)")
for k in ("pos", "head", "vel", "vslip", "m25"):
    within, between = [], []
    for W in cands:
        lap_ids = sorted({l for (w, n) in runs if w == W for l in runs[(w, n)] if l >= 1})
        if len(lap_ids) < 2:
            continue
        M = np.array([[runs[(W, n)][l][k] for l in lap_ids] for n in order if (W, n) in runs and all(l in runs[(W, n)] for l in lap_ids)])
        if len(M) < 2:
            continue
        Mc = M - M.mean(0)                                                # 바퀴 번호별 평균 (첫 바퀴는 가속이 껴서 다르다) 을 빼고
        within.append(((Mc - Mc.mean(1, keepdims=True)) ** 2).sum() / (M.shape[0] * (M.shape[1] - 1)))
        between.append(np.var(Mc.mean(1), ddof=1) - within[-1] / M.shape[1])
    if within:
        print(f"  {k:6s} 바퀴끼리 σ {np.sqrt(np.mean(within)):.3f} · 판끼리 σ {np.sqrt(max(np.mean(between), 0.0)):.3f}")

print("\n④ 짝 비교: 시간상 가장 가까운 기준 판 대비 차 (판마다)")
bases = [n for n in order if (BASE, n) in runs]
for W in cands[1:]:
    diffs = []
    for n in order:
        if (W, n) not in runs or not bases:
            continue
        b = min(bases, key=lambda bn: abs(runs[(BASE, bn)][0]["t0"] - runs[(W, n)][0]["t0"]))
        diffs.append({k: runs[(W, n)][0][k] - runs[(BASE, b)][0][k] for k, _, _ in KEYS})
    print(f"  {LAB.get(W, W):26s} " + "  ".join(f"{k} " + "/".join(f"{d[k]:+.2f}" for d in diffs) for k in ("pos", "head", "vel", "m50", "vslip", "dtick")))
    v = per_run(W, "pos"); b = per_run(BASE, "pos")
    if len(v) >= 2 and len(b) >= 2:
        ok_pos = all(d["pos"] < 0 for d in diffs) or (b.mean() - v.mean()) > 2.8 * np.sqrt(np.var(v, ddof=1) / len(v) + np.var(b, ddof=1) / len(b))
        g_m50 = per_run(W, "m50").mean() - per_run(BASE, "m50").mean() <= GUARD["m50"]
        g_vs = per_run(W, "vslip").mean() - per_run(BASE, "vslip").mean() <= GUARD["vslip"]
        g_dt = per_run(W, "dtick").mean() <= GUARD["dtick_abs"]
        print(f"     → 위치 개선 {'예' if ok_pos else '아니오'} · 큰 슬립 {'지킴' if g_m50 else '넘음'} · 미끄럼 속도 {'지킴' if g_vs else '넘음'} · 떨림 {'지킴' if g_dt else '넘음'}")

# ── 그림 ──
fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
INK, INK2, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb"
COL = ["#52514e", "#2a78d6", "#1baf7a", "#eda100", "#d6453d", "#7a5bd6"]
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                     "figure.facecolor": SURF, "axes.facecolor": SURF})
fig, axes = plt.subplots(2, 5, figsize=(18, 8.5), dpi=150, gridspec_kw=dict(hspace=0.45, wspace=0.32))
for ax, (k, title, f) in zip(axes.ravel(), KEYS):
    for i, W in enumerate(cands):
        c = COL[i % len(COL)]
        for n in order:
            if (W, n) not in runs:
                continue
            laps_ = [l for l in runs[(W, n)] if l >= 1]
            if len(laps_) > 1:
                ax.plot([i + 0.18] * len(laps_), [runs[(W, n)][l][k] for l in laps_], "o", ms=3.5, color=c, alpha=0.35, zorder=2)
            ax.plot(i, runs[(W, n)][0][k], "o", ms=7, color=c, mec=SURF, mew=1.0, zorder=3)
        v = per_run(W, k)
        if len(v):
            ax.plot([i - 0.25, i + 0.25], [v.mean()] * 2, color=INK, lw=2.0, zorder=4)
    ax.set_xticks(range(len(cands))); ax.set_xticklabels([LAB.get(W, W).replace(" ", "\n", 1) for W in cands], fontsize=8)
    ax.set_title(title, loc="left", fontsize=10.5, color=INK, pad=6); ax.grid(axis="y", color=GRID, lw=0.6)
    for s_ in ("top", "right"):
        ax.spines[s_].set_visible(False)
    lo, hi = ax.get_ylim(); ax.set_ylim(min(0, lo), hi * 1.08)
fig.text(0.04, 0.985, "Q 후보 비교 — circles8 width 2.0 · 0.8 m/s · accel 1.0 · friction · 슬립 되먹임 1.0 · R 고정", fontsize=12, color=INK, va="top")
fig.text(0.04, 0.955, "큰 점 = 판 전체, 옅은 점 = 바퀴별, 검은 막대 = 후보 평균. 추종 오차 = 기준 − 추정기 (map), 슬립률 10 Hz filtfilt", fontsize=8.5, color=MUTED, va="top")
fig.subplots_adjust(top=0.9, bottom=0.08, left=0.04, right=0.99)
out = os.path.join(HERE, FIG + ".png"); fig.savefig(out, facecolor=SURF); print("저장:", out)

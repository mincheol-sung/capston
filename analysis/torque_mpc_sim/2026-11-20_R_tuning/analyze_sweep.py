#!/usr/bin/python3
"""R 스윕 결과 → 표 + 그림. 사용법: analyze_sweep.py 결과폴더 그림폴더 접두어 [Δτ 기준 라벨 접미어]
  라벨 꼴: <limit>_wd<w_dudt>_ru<r_u>  (limit = motor_only | friction)
  최적 = Δτ RMS ≤ 지금 설정(wd 0.001, ru 0.1)의 Δτ RMS 인 판 중 주행 위치 RMS 최소 (실차 떨림 없음이 확인된 수준을 넘지 않게)."""
import glob, json, math, os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUN, FIG, PRE = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "savefig.facecolor": "#fcfcfb"})
INK, INK2 = "#0b0b0b", "#52514e"
LIM = {"motor_only": ("MPC B (모터 상한)", "#2a78d6"), "friction": ("MPC C (마찰 상한)", "#eb6834")}
WD = [0.0001, 0.0003, 0.001, 0.003, 0.01]
RU = [0.01, 0.1, 1, 10]
CUR = (0.001, 0.1)

rows = {}
for f in glob.glob(os.path.join(RUN, "*.json")):
    if "fault" in os.path.basename(f):
        continue
    m = re.match(r"(motor_only|friction)_wd([0-9.e-]+)_ru([0-9.e-]+)\.json", os.path.basename(f))
    if not m:
        continue
    try:
        s = json.loads(open(f).read().strip().splitlines()[-1])
    except Exception:
        continue
    if s.get("fault") or "moving_rms_mm" not in s:
        continue
    rows[(m.group(1), float(m.group(2)), float(m.group(3)))] = s

METRICS = [("moving_rms_mm", "주행 위치 오차 RMS [mm]", "{:.1f}"), ("yaw_rms_deg", "요 오차 RMS [°]", "{:.2f}"),
           ("dtau_rms", "틱간 Δτ RMS [N·m]", "{:.4f}"), ("slip_frac10", "|λ|>0.1 시간 비율 [%]", "{:.1f}")]

best = {}
for lim in LIM:
    cur = rows.get((lim, *CUR))
    if cur is None:
        continue
    cap = cur["dtau_rms"]
    ok = [(k, s) for k, s in rows.items() if k[0] == lim and s["dtau_rms"] <= cap * 1.0001]
    k, s = min(ok, key=lambda ks: ks[1]["moving_rms_mm"])
    best[lim] = (k, s, cur)

# 표
with open(os.path.join(FIG, f"{PRE}_table.md"), "w") as fo:
    for lim in LIM:
        fo.write(f"\n### {LIM[lim][0]}\n\n| w_dudt \\ r_u | " + " | ".join(str(r) for r in RU) + " |\n|" + "---|" * (len(RU) + 1) + "\n")
        for wd in WD:
            cells = []
            for ru in RU:
                s = rows.get((lim, wd, ru))
                cells.append("—" if s is None else f"{s['moving_rms_mm']:.1f} mm · {s['yaw_rms_deg']:.2f}° · Δτ {s['dtau_rms']:.4f}"
                             + (f" · 슬립 {100*s.get('slip_frac10', float('nan')):.1f}%" if "slip_frac10" in s else ""))
            fo.write(f"| {wd:g} (r_Δ {wd/1e-4:g}) | " + " | ".join(cells) + " |\n")
        if lim in best:
            k, s, cur = best[lim]
            fo.write(f"\n최적 (Δτ ≤ 지금 {cur['dtau_rms']:.4f}): w_dudt {k[1]:g}, r_u {k[2]:g} → {s['moving_rms_mm']:.1f} mm "
                     f"(지금 {cur['moving_rms_mm']:.1f}), 요 {s['yaw_rms_deg']:.2f}° (지금 {cur['yaw_rms_deg']:.2f}°)\n")
print(open(os.path.join(FIG, f"{PRE}_table.md")).read())

# 그림 A: 격자 히트맵 (행 = 지표, 열 = B·C). 순차 색 한 색상 (밝음 → 어두움 = 나쁨)
mets = [m for m in METRICS if any(m[0] in s for s in rows.values())]
fig, axes = plt.subplots(len(mets), 2, figsize=(10, 2.6 * len(mets) + 0.6), squeeze=False)
for j, lim in enumerate(LIM):
    for i, (key, title, fmt) in enumerate(mets):
        ax = axes[i][j]
        Z = np.full((len(WD), len(RU)), np.nan)
        for a, wd in enumerate(WD):
            for b, ru in enumerate(RU):
                s = rows.get((lim, wd, ru))
                if s is not None and key in s:
                    Z[a, b] = s[key] * (100 if key == "slip_frac10" else 1)
        vals = np.concatenate([np.array([s[key] * (100 if key == "slip_frac10" else 1)
                                         for k, s in rows.items() if key in s])])
        ax.imshow(Z, cmap="Blues", vmin=np.nanmin(vals), vmax=np.nanmax(vals), aspect="auto")
        for a in range(len(WD)):
            for b in range(len(RU)):
                if np.isfinite(Z[a, b]):
                    dark = (Z[a, b] - np.nanmin(vals)) / (np.nanmax(vals) - np.nanmin(vals) + 1e-12) > 0.55
                    ax.text(b, a, fmt.format(Z[a, b]), ha="center", va="center", fontsize=8.5,
                            color="#fcfcfb" if dark else INK)
        cb, ca = RU.index(CUR[1]), WD.index(CUR[0])
        ax.add_patch(plt.Rectangle((cb - 0.5, ca - 0.5), 1, 1, fill=False, ec=INK, lw=1.6, ls="--"))
        if lim in best:
            k = best[lim][0]
            ax.add_patch(plt.Rectangle((RU.index(k[2]) - 0.5, WD.index(k[1]) - 0.5), 1, 1, fill=False, ec="#e34948", lw=2.2))
        ax.set_xticks(range(len(RU)), [f"{r:g}" for r in RU]); ax.set_yticks(range(len(WD)), [f"{w:g}" for w in WD])
        ax.set_xlabel("r_u (입력 편차 벌점)") if i == len(mets) - 1 else None
        ax.set_ylabel("w_dudt (변화율 벌점)") if j == 0 else None
        ax.set_title(f"{LIM[lim][0]} — {title}", color=INK, loc="left", fontsize=9.5)
fig.suptitle("점선 칸 = 지금 설정 (w_dudt 0.001, r_u 0.1), 빨간 칸 = Δτ ≤ 지금 중 위치 오차 최소", color=INK2, x=0.01, ha="left", fontsize=9.5)
fig.tight_layout(rect=(0, 0, 1, 0.98))
fig.savefig(os.path.join(FIG, f"{PRE}_heatmap.png"), dpi=130)
plt.close(fig)

# 그림 B: 맞바꿈 (Δτ RMS ↔ 위치 RMS)
fig, ax = plt.subplots(figsize=(8.5, 5.2))
ax.grid(True, color="#e6e5e0", lw=0.6)
for lim, (name, col) in LIM.items():
    pts = [(s["dtau_rms"], s["moving_rms_mm"], k) for k, s in rows.items() if k[0] == lim]
    if not pts:
        continue
    x, y, _ = zip(*pts)
    ax.scatter(x, y, s=46, color=col, label=name, zorder=3, edgecolors="#fcfcfb", linewidths=1.5)
    if lim in best:
        k, s, cur = best[lim]
        ax.scatter([cur["dtau_rms"]], [cur["moving_rms_mm"]], s=160, facecolors="none", edgecolors=INK, lw=1.4, zorder=4)
        ax.annotate(f"지금 ({name[:5]})", (cur["dtau_rms"], cur["moving_rms_mm"]), (8, 6), textcoords="offset points", color=INK2, fontsize=8.5)
        ax.scatter([s["dtau_rms"]], [s["moving_rms_mm"]], s=200, marker="*", color=col, edgecolors=INK, lw=0.8, zorder=5)
        ax.annotate(f"최적 w_dudt {k[1]:g}, r_u {k[2]:g}", (s["dtau_rms"], s["moving_rms_mm"]), (8, -12), textcoords="offset points",
                    color=INK, fontsize=8.5)
        ax.axvline(cur["dtau_rms"], color=col, lw=0.8, ls=":")
ax.set_xscale("log")
ax.set_xlabel("틱간 Δτ RMS [N·m] (클수록 떨림, 점선 = 지금 설정 = 실차 떨림 없음 확인 수준)")
ax.set_ylabel("주행 위치 오차 RMS [mm] (시뮬 참값 기준)")
ax.legend(frameon=False, loc="upper right")
ax.set_title("R 스윕 맞바꿈 — 각 점 = (w_dudt, r_u) 한 판", color=INK, loc="left")
fig.tight_layout()
fig.savefig(os.path.join(FIG, f"{PRE}_tradeoff.png"), dpi=130)
print("saved")

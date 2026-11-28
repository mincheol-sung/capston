"""튜닝 뒤 (220501_tuned_mu060) 세 경로 × 세 제어기 주행 경로 겹쳐 그리기.
경로 = SLAM 로봇 중심 (seq_s1.metrics 의 _sx·_sy, 창 = 무장 · 기준 속력 > 0.05), 기준 = 같은 창의 기준열 x·y. 복귀 다리 (rotate) 는 뺀다.
  paths_overlay.png : 경로마다 한 칸, 세 제어기 6 판씩 겹침
  paths_grid.png    : 경로 × 제어기 3 × 3, 칸마다 6 판 겹침 (평균 위치 RMS 를 제목에)
source install/setup.bash 뒤 /usr/bin/python3 plot_paths.py"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "2026-11-28_c_sfb_q4"))
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools")
import seq_s1 as S  # noqa: E402
from bag_runs import runs  # noqa: E402

BAG = "/home/smoc/capston/capston/bags/2026-11-28/220501_tuned_mu060"
SHAPES = [("circles8", "s1 circles8 (width 2.0 · 1.0 m/s · accel 1.0)"), ("zigzag", "s2 zigzag (5 × 2.0 m · accel 1.15)"),
          ("polyline", "s3 polyline (최고 1.5 m/s · accel 1.5)")]
CTRL = [("mpc", r"C (MPC, $\hat{\mu}$ 0.60 · 배율 1.2)", "#2a78d6", "-"), ("pp", "PP", "#eb6834", "--"), ("adpp", "ADPP", "#1baf7a", ":")]
INK, INK2, GRID, REF = "#1f2328", "#57606a", "#d0d7de", "#8c959f"

fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})

rows = json.load(open(os.path.join(HERE, "rows.json")))
pos = {(m["shape"], m["ctrl"]): [] for m in rows}
for m in rows:
    if m["period"] == "후":
        pos[(m["shape"], m["ctrl"])].append(m["pos"])

RS, D = runs(BAG), S.read(BAG)
tracks = {}
for r in RS:
    sq = r.get("seq") or {}
    shp = (r.get("ref") or {}).get("shape")
    if not r["armed"] or sq.get("kind") == "return" or shp not in dict(SHAPES):
        continue
    (t_on, t_off), = r["armed"][:1]
    m = S.metrics(D, t_on, t_off)
    R = m["_R"]; ts = m["_trR"]
    w = m["_win"]; tw = m["_ts"][w] + t_on
    k = (ts >= tw[0]) & (ts <= tw[-1])
    tracks.setdefault((shp, r["controller"]), []).append(dict(x=m["_sx"], y=m["_sy"], rx=R[k, 0], ry=R[k, 1], run=r["run"], dir=sq.get("dir")))
print({k: len(v) for k, v in tracks.items()})


def draw_ref(ax, shp):
    t = tracks[(shp, "mpc")][0]
    ax.plot(t["rx"], t["ry"], color=REF, lw=7, alpha=0.30, solid_capstyle="round", zorder=1, label="기준 경로")


def style(ax):
    ax.set_aspect("equal", adjustable="datalim"); ax.grid(color=GRID, lw=0.5)
    ax.set_xlabel("x [m] (map)"); ax.set_ylabel("y [m] (map)")


# 1) 경로마다 세 제어기 겹침
fig, axs = plt.subplots(1, 3, figsize=(17, 6.2))
for ax, (shp, title) in zip(axs, SHAPES):
    draw_ref(ax, shp)
    for c, lab, col, ls in CTRL:
        for j, t in enumerate(tracks.get((shp, c), [])):
            ax.plot(t["x"], t["y"], color=col, ls=ls, lw=1.3, alpha=0.75, zorder=3,
                    label=f"{lab} — 위치 RMS {np.mean(pos[(shp, c)]):.1f} cm" if j == 0 else None)
    style(ax); ax.set_title(title, loc="left", fontsize=11, color=INK)
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.12), fontsize=8.5, frameon=False)
fig.text(0.01, 0.99, "튜닝 뒤 세 경로 × 세 제어기 (220501_tuned_mu060, 제어기마다 6 판 겹침, SLAM 로봇 중심)", fontsize=12, color=INK, va="top")
fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(os.path.join(HERE, "paths_overlay.png"), dpi=130); plt.close(fig)

# 2) 경로 × 제어기
fig, axs = plt.subplots(3, 3, figsize=(16, 15))
for i, (shp, title) in enumerate(SHAPES):
    lims = None
    for j, (c, lab, col, ls) in enumerate(CTRL):
        ax = axs[i, j]
        draw_ref(ax, shp)
        for t in tracks.get((shp, c), []):
            ax.plot(t["x"], t["y"], color=col, lw=1.1, alpha=0.8, zorder=3)
        style(ax)
        p = pos[(shp, c)]
        ax.set_title(f"{title.split(' (')[0]} · {lab.split(' (')[0]} — {len(p)} 판, 위치 RMS {np.mean(p):.1f} cm (최대 판 {np.max(p):.1f})",
                     loc="left", fontsize=10, color=INK)
        if j == 0:
            lims = (ax.get_xlim(), ax.get_ylim())
        else:
            ax.set_xlim(*lims[0]); ax.set_ylim(*lims[1])
fig.text(0.01, 0.995, "경로 × 제어기 — 칸마다 6 판 겹침 (회색 굵은 선 = 기준). 색: 파랑 C · 주황 PP · 청록 ADPP", fontsize=12, color=INK, va="top")
fig.tight_layout(rect=(0, 0, 1, 0.975)); fig.savefig(os.path.join(HERE, "paths_grid.png"), dpi=120); plt.close(fig)
print("그림:", os.path.join(HERE, "paths_overlay.png"), os.path.join(HERE, "paths_grid.png"))

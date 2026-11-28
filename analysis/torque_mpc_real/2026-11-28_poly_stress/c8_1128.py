"""11-28 원 8 자 세 제어기 한 판씩 (반지름 0.5 m · 1.0 m/s · 가속 1.0 · blend 0 · 1 바퀴) + 11-27 같은 조건 판.
지표: circ_run.py analyze (위치·안쪽 치우침·진행·헤딩·슬립률 %) + circ_slipvel.py one (미끄럼 속도) + CPU 유휴.
그림: c8_1128_paths.png (세 제어기 경로를 나란히, 큰 미끄럼 > 0.2 m/s 위치 표시).
source install/setup.bash 뒤 /usr/bin/python3 c8_1128.py → c8_1128_out.txt, 표 출력"""
import os, re, sys, warnings
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ARGV = sys.argv[:]; sys.argv = sys.argv[:1]
warnings.filterwarnings("ignore")
src = open(os.path.join(HERE, "circ_run.py")).read().split("fp = [f for f in font_manager")[0]
exec(compile(src, "circ_run_head", "exec"))
sv = open(os.path.join(HERE, "circ_slipvel.py")).read().split("out = open(")[0]
NS = {"__file__": os.path.join(HERE, "circ_slipvel.py")}; exec(compile(sv, "circ_slipvel_head", "exec"), NS)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

BB = "/home/smoc/capston/capston/bags/"
RUNS = [("MPC 11-28 (W ×4 · 되먹임 1.0)", "2026-11-28/170146_c8_mpc_r1"), ("PP 11-28", "2026-11-28/170339_c8_pp_r1"), ("ADPP 11-28", "2026-11-28/170436_c8_adpp_r1"),
        ("MPC 11-27 (W ×1 · 되먹임 1.0)", "2026-11-27/192519_yarr_mpc"), ("MPC 11-27 (W ×1 · 되먹임 1.0)", "2026-11-27/193234_yarr_mpc"),
        ("MPC 11-27 (W ×1 · 되먹임 1.0)", "2026-11-27/193851_yarr_mpc"), ("PP 11-27", "2026-11-27/165935_pp_r1"), ("PP 11-27", "2026-11-27/173001_pp_r1"),
        ("ADPP 11-27", "2026-11-27/173512_pp_r1")]


def cpu_idle(path):
    f = os.path.join(path, "run_info", "mpstat.txt")
    v = [float(l.split()[-1]) for l in open(f) if re.search(r"\sall\s", l) and not l.split()[-1].startswith("%")] if os.path.exists(f) else []
    return float(np.median(v[2:])) if len(v) > 3 else np.nan


out = open(os.path.join(HERE, "c8_1128_out.txt"), "w")
rows = []
for lab, rel in RUNS:
    p = BB + rel; r = analyze(p); s = NS["one"](p); r.update(sv_rms=s["rms"], sv_p20=s["p20"], sv_spin=s["spin"], cpu=cpu_idle(p), lab=lab)
    rows.append(r)
    print(f"{rel}: {lab} · 끝까지 {r['done']} · fault {r['faults'] or '없음'} · CPU 유휴 {r['cpu']:.0f} % · 위치 SLAM {r['slam_pos']:.2f} (추정기 {r['pos']:.2f}, 최대 {r['pos_max']:.1f})"
          f" · 안쪽 {r['inside']:+.2f} · 진행 {r['along']:+.2f} · 헤딩 {r['head']:.2f} · 큰 슬립 {r['m50']:.1f} · 중간 {r['m25']:.1f} · 미끄럼 속도 RMS {s['rms']:.1f} · >0.2 {s['p20']:.1f} · 헛돎 {s['spin']:.0f}", file=out)
out.close()
print("| 판 | 제어기 | 위치 SLAM cm | 위치 최대 | 진행 평균 | 안쪽 치우침 | 헤딩 ° | 큰 슬립 % | 중간 슬립 % | 미끄럼 속도 RMS cm/s | > 0.2 m/s % | CPU 유휴 % |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|")
for (lab, rel), r in zip(RUNS, rows):
    print(f"| {rel.split('/')[1][:6]} | {lab} | {r['slam_pos']:.1f} | {r['pos_max']:.1f} | {r['along']:+.1f} | {r['inside']:+.1f} | {r['head']:.1f} | {r['m50']:.1f} | {r['m25']:.1f} | "
          f"{r['sv_rms']:.1f} | {r['sv_p20']:.1f} | {r['cpu']:.0f} |")

fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": "#d0d7de"})
fig, axs = plt.subplots(1, 3, figsize=(15, 5.6))
COL = ["#0969da", "#bf8700", "#8250df"]
for ax, r, c in zip(axs, rows[:3], COL):
    w = r["_win"]
    ax.plot(r["_R"][:, 0], r["_R"][:, 1], color="#8c959f", lw=6, alpha=0.35, solid_capstyle="round", label="기준")
    ax.plot(r["_X"][w, 1], r["_X"][w, 2], color=c, lw=1.3, label="추정기")
    ax.plot(r["_sx"], r["_sy"], color="#1a7f37", lw=0.8, ls="--", label="SLAM 중심")
    ax.set_aspect("equal"); ax.grid(color="#d0d7de", lw=0.5)
    ax.set_title(f"{r['lab']}\n위치 {r['slam_pos']:.1f} cm · 큰 슬립 {r['m50']:.1f} % · 미끄럼 속도 {r['sv_rms']:.1f} cm/s", loc="left", fontsize=10)
axs[0].legend(fontsize=8.5, frameon=False, loc="lower left")
fig.text(0.01, 0.99, "11-28 원 8 자 — 반지름 0.5 m · 1.0 m/s · 가속 1.0 · blend 0 · 1 바퀴 (옆 힘 요구 109 %), 세 제어기 한 판씩", fontsize=11, va="top")
fig.tight_layout(rect=(0, 0, 1, 0.94)); fig.savefig(os.path.join(HERE, "c8_1128_paths.png"), dpi=130); plt.close(fig)
print("그림:", os.path.join(HERE, "c8_1128_paths.png"))

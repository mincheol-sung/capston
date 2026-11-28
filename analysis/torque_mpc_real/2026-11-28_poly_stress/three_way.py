"""PP · 어댑티브 PP · MPC 를 같은 잣대로 — 11-26 저녁 같은 세션에서 번갈아 돈 세 시나리오 + 11-28 새 기본값 MPC (꺾은선).
source install/setup.bash 뒤  /usr/bin/python3 three_way.py   → three_way_out.txt (판별), 표 출력, three_way.png
지표 정의는 poly_run.py analyze 그대로 (추정기·SLAM 중심 위치 오차, 진행·옆, 헤딩, 10 Hz LPF 슬립률, 꼭짓점 통과 속력, 모터 명령 떨림).
슬립 되먹임 토픽 (미끄럼 속도) 은 MPC 만 있어 여기서는 안 쓴다."""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.argv = sys.argv[:1]
exec(compile(open(os.path.join(HERE, "poly_run.py")).read().split("# ── 글꼴·색")[0], "poly_run_head", "exec"))
import warnings
warnings.filterwarnings("ignore")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

B26, B28 = "/home/smoc/capston/capston/bags/2026-11-26", "/home/smoc/capston/capston/bags/2026-11-28"
SCEN = {
    "꺾은선 1.5/1.5 통과 (스트레스)": {
        "MPC": ["201259_s2pl_mpc_r1", "201604_s2pl_mpc_r2", "201944_s2pl_mpc_r3"],
        "PP": ["201404_s2pl_pp_r1", "201704_s2pl_pp_r2", "202049_s2pl_pp_r3"],
        "ADPP": ["201501_s2pl_adpp_r1", "201804_s2pl_adpp_r2", "202150_s2pl_adpp_r3"],
        "MPC 11-28": ["155209_stress15", "160322"]},
    "원 8 자 R 0.5 · 1.0 m/s · blend 0.6": {
        "MPC": ["184758_s2_mpc_r1", "184937_s2_mpc_r2", "185209_s2_mpc_r3", "185937_s2_mpc_r4", "190248_s2_mpc_r5", "190352_s2_mpc_r6"],
        "PP": ["185321_s2_pp_r1", "185437_s2_pp_r2", "185532_s2_pp_r3"],
        "ADPP": ["185636_s2_adpp_r1", "185736_s2_adpp_r2", "185834_s2_adpp_r3"]},
    "지그재그 30° · 9 변 통과": {
        "MPC": ["190531_s2zz_mpc_r1", "191148_s2zz_mpc_r2", "200634_s2zz_mpc_r3"],
        "PP": ["190718_s2zz_pp_r1", "195945_s2zz_pp_r2", "200815_s2zz_pp_r3"],
        "ADPP": ["190854_s2zz_adpp_r1", "200337_s2zz_adpp_r2", "200952_s2zz_adpp_r3"]},
}
KEYS = [("pos", "위치 RMS 추정기 cm"), ("slam_pos", "위치 RMS SLAM 중심 cm"), ("pos_max", "위치 최대 cm"), ("along_r", "진행 RMS cm"),
        ("cross_r", "옆 RMS cm"), ("head", "헤딩 RMS °"), ("m50", "큰 슬립 > 0.5 %"), ("m25", "중간 슬립 > 0.25 %"),
        ("vcorner", "꼭짓점 통과 속력 m/s"), ("dtick", "모터 명령 떨림 mN·m")]
out = open(os.path.join(HERE, "three_way_out.txt"), "w")
res = {}
for sc, groups in SCEN.items():
    res[sc] = {}
    for g, names in groups.items():
        rows = []
        for n in names:
            p = os.path.join(B28 if g.endswith("11-28") else B26, n)
            try:
                r = analyze(p)
            except Exception as e:
                print(f"{n}: 실패 {type(e).__name__} {e}", file=out); continue
            rows.append(r)
            print(f"[{sc} · {g}] {n}: 무장 {r['armed']:.1f} s · 끝까지 {'예' if r['done'] else '아니오'} · fault {r['faults'] or '없음'} · "
                  + " · ".join(f"{lab} {r[k]:.2f}" for k, lab in KEYS), file=out)
        res[sc][g] = rows
out.close()


def agg(rows, k):
    v = np.array([r[k] for r in rows], float); v = v[np.isfinite(v)]
    return (v.mean(), v.min(), v.max(), len(v)) if len(v) else (np.nan, np.nan, np.nan, 0)


for sc, groups in res.items():
    print(f"\n## {sc}")
    gs = list(groups)
    print("| 지표 | " + " | ".join(f"{g} ({len(groups[g])} 판)" for g in gs) + " |")
    print("|---|" + "---|" * len(gs))
    for k, lab in KEYS:
        cells = []
        for g in gs:
            m, lo, hi, n = agg(groups[g], k)
            cells.append("-" if n == 0 else (f"{m:.2f} ({lo:.2f}~{hi:.2f})" if k == "vcorner" else f"{m:.1f} ({lo:.1f}~{hi:.1f})"))
        print(f"| {lab} | " + " | ".join(cells) + " |")
    print("| 끝까지 / fault | " + " | ".join(f"{sum(r['done'] for r in groups[g])}/{len(groups[g])} · {sum(bool(r['faults']) for r in groups[g])}" for g in gs) + " |")

# 그림: 시나리오 × (위치 RMS SLAM, 헤딩 RMS, 큰 슬립) 막대 + 판 점
fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": "#d0d7de"})
COL = {"MPC": "#0969da", "PP": "#bf8700", "ADPP": "#8250df", "MPC 11-28": "#54aeff"}
show = [("slam_pos", "위치 RMS (SLAM 중심) [cm]"), ("head", "헤딩 RMS [°]"), ("m50", "큰 슬립 (> 0.5) 시간 [%]")]
fig, axs = plt.subplots(len(res), len(show), figsize=(13, 3.2 * len(res)))
for i, (sc, groups) in enumerate(res.items()):
    for j, (k, lab) in enumerate(show):
        ax = axs[i, j]; gs = list(groups)
        for x, g in enumerate(gs):
            v = [r[k] for r in groups[g] if np.isfinite(r[k])]
            ax.bar(x, np.mean(v) if v else 0, color=COL[g], width=0.62, alpha=0.85)
            ax.scatter([x] * len(v), v, color="#1f2328", s=10, zorder=3)
            ax.text(x, (np.mean(v) if v else 0), f"{np.mean(v):.1f}" if v else "-", ha="center", va="bottom", fontsize=8, color="#57606a")
        ax.set_xticks(range(len(gs))); ax.set_xticklabels(gs, fontsize=8.5); ax.grid(axis="y", color="#d0d7de", lw=0.5)
        if i == 0:
            ax.set_title(lab, loc="left", fontsize=10.5)
        if j == 0:
            ax.set_ylabel(sc, fontsize=9)
fig.text(0.01, 0.995, "세 제어기 — 11-26 저녁 같은 세션 번갈아 (막대 = 평균, 점 = 판). 'MPC 11-28' = 새 기본값 (위치 ×4 · 슬립 되먹임 1.0)", fontsize=10.5, va="top")
fig.tight_layout(rect=(0, 0, 1, 0.975)); fig.savefig(os.path.join(HERE, "three_way.png"), dpi=130); plt.close(fig)
print("\n그림:", os.path.join(HERE, "three_way.png"), "· 판별:", os.path.join(HERE, "three_way_out.txt"))

"""2026-11-28 밤 — 원 · zigzag · polyline 세 경로 공통 μ̂ × 되먹임 배율 고르기.
후보 (μ̂, 배율): (0.33, 1.0) 지금 기본값 · (0.50, 1.1) · (0.60, 1.2) · (0.70, 1.2). 비교: PP · ADPP.
판: 원 = 오늘 S1 모든 판 (grid1.BAGS + 204627_grid2), zigzag = 211522_zz_2 · 205948_zz · 194358_seq_sfbq4 (data 다리만, 모양 zigzag),
    polyline = 213115_poly (같은 세션 PP 3 · ADPP 2 · 조건마다 MPC 3).
지표: 위치 RMS (SLAM 로봇 중심) · slip velocity (네 바퀴 최대 r(ω − ω_nom) 의 RMS — 정식 용어 아님, 이 프로젝트 지표) · 큰 슬립 % (|λ| > 0.5 시간).
비 = MPC 조건 평균 / min(PP 평균, ADPP 평균) — 1 보다 작으면 둘 다보다 낫다.
고르는 법 두 가지: (가) 세 경로 × 세 지표 비의 평균, (나) 가장 나쁜 비 (minimax — 어느 경로에서도 크게 지지 않는 것).
nice -n 19 /usr/bin/python3 final3.py → final3_out.txt, final3.png"""
import os
import sys

import numpy as np
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import grid1 as G1  # noqa: E402
import mu055 as M  # noqa: E402
import scen as SC  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

B = "/home/smoc/capston/capston/bags/2026-11-28/"
CAND = [(0.33, 1.0), (0.50, 1.1), (0.60, 1.2), (0.70, 1.2)]
MET = [("pos", "위치 RMS cm"), ("sv", "slip velocity cm/s"), ("m50", "큰 슬립 %")]
EXTRA = [("pos_max", "위치 최대 cm"), ("head", "헤딩 RMS °"), ("dtick", "떨림 mN·m")]


def key(m):
    return (round(m["mu"], 2), round(m["sc"], 2)) if m["ctrl"] == "MPC" else m["ctrl"]


def load():
    sc = {}
    sc["원"] = [m for tag, b in G1.BAGS + [("grid2", B + "204627_grid2")] for m in M.rows_of(tag, b)]
    SC.SHAPE = "zigzag"
    sc["zigzag"] = [m for b in ("211522_zz_2", "205948_zz", "194358_seq_sfbq4") for m in SC.rows_seq(b, B + b)]
    SC.SHAPE = "polyline"
    sc["polyline"] = SC.rows_seq("poly", B + "213115_poly")
    return sc


if __name__ == "__main__":
    sc = load()
    out = open(os.path.join(HERE, "final3_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    R = {}
    for s, rows in sc.items():
        G = {}
        for m in rows:
            G.setdefault(key(m), []).append(m)
        R[s] = G
        P(f"\n## {s} — 판 수: " + " · ".join(f"{k if isinstance(k, str) else f'MPC {k[0]:.2f}·{k[1]:g}'} {len(v)}" for k, v in sorted(G.items(), key=str) if (k in CAND or isinstance(k, str))))
        P("| 조건 | 판 | " + " | ".join(lab for _, lab in MET + EXTRA) + " |")
        P("|---|---|" + "---|" * (len(MET) + len(EXTRA)))
        for k in CAND + ["PP", "ADPP"]:
            v = G.get(k, [])
            if not v:
                continue
            nm = f"MPC μ̂ {k[0]:.2f} · {k[1]:g}" if isinstance(k, tuple) else k
            P(f"| {nm} | {len(v)} | " + " | ".join(f"{np.nanmean([m[q] for m in v]):.1f}" for q, _ in MET + EXTRA) + " |")
    P("\n## 비 = MPC / min(PP, ADPP) (경로별 · 지표별), 괄호 = 더 좋은 쪽 기준값과 Welch p")
    score = {}
    for k in CAND:
        ratios = []
        P(f"\n### MPC μ̂ {k[0]:.2f} · 배율 {k[1]:g}")
        for s in sc:
            G = R[s]; v = G.get(k, [])
            cells = []
            for q, lab in MET:
                base = min(("PP", "ADPP"), key=lambda r: np.mean([m[q] for m in G[r]]))
                x = np.array([m[q] for m in v]); y = np.array([m[q] for m in G[base]])
                r = x.mean() / y.mean(); ratios.append((r, s, lab))
                p = stats.ttest_ind(x, y, equal_var=False).pvalue if len(x) > 1 and len(y) > 1 else np.nan
                cells.append(f"{lab.split()[0]} {x.mean():.1f}/{y.mean():.1f} ({base}) = {r:.2f} (p {p:.3f})")
            P(f"- {s}: " + " · ".join(cells))
        rr = np.array([r for r, _, _ in ratios]); worst = max(ratios)
        score[k] = (rr.mean(), worst[0], worst[1], worst[2], int((rr < 1).sum()))
        P(f"- 평균 비 {rr.mean():.3f} · 가장 나쁜 비 {worst[0]:.2f} ({worst[1]} {worst[2]}) · 9 칸 중 1 보다 작은 칸 {int((rr < 1).sum())}")
    P("\n## 순위")
    for lab, idx in (("(가) 평균 비", 0), ("(나) 가장 나쁜 비 (minimax)", 1)):
        P(f"- {lab}: " + " < ".join(f"μ̂ {k[0]:.2f}·{k[1]:g} ({score[k][idx]:.3f})" for k in sorted(score, key=lambda k: score[k][idx])))
    out.close()

    fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
    if fp:
        font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
    INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
    COL = {CAND[0]: "#86b6ef", CAND[1]: "#1baf7a", CAND[2]: "#2a78d6", CAND[3]: "#6250d6", "PP": "#eb6834", "ADPP": "#e87ba4"}
    LAB = {CAND[0]: "MPC 0.33·1.0 (지금 기본)", CAND[1]: "MPC 0.50·1.1", CAND[2]: "MPC 0.60·1.2", CAND[3]: "MPC 0.70·1.2", "PP": "PP", "ADPP": "ADPP"}
    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    for j, s in enumerate(sc):
        G = R[s]
        for i, (q, ttl) in enumerate((("sv", "slip velocity cm/s"), ("m50", "큰 슬립 %"))):
            ax = axs[i, j]
            for k in CAND + ["PP", "ADPP"]:
                v = G.get(k, [])
                if not v:
                    continue
                x, y = [m[q] for m in v], [m["pos"] for m in v]
                ax.scatter(x, y, s=34, color=COL[k], alpha=0.55, zorder=2)
                ax.scatter([np.mean(x)], [np.mean(y)], s=190, marker="X", color=COL[k], edgecolors="white", linewidths=1.4, zorder=3, label=LAB[k])
            ax.set_xlabel(f"{ttl} (왼쪽 = 좋음)"); ax.set_ylabel("위치 RMS cm (아래 = 좋음)"); ax.grid(color=GRID, lw=0.5)
            ax.set_title(f"{s} — 위치 대 {ttl}", loc="left", fontsize=10.5, color=INK)
            if s == "polyline":
                ax.set_xscale("log")
    axs[0, 0].legend(fontsize=8.5, frameon=False, loc="upper left")
    fig.text(0.01, 0.995, "2026-11-28 세 경로 mu_hat × 되먹임 배율 — 작은 점 = 판, X = 조건 평균 (polyline 가로축은 로그: PP 첫 판 slip v 121)", fontsize=10.5, color=INK, va="top")
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(os.path.join(HERE, "final3.png"), dpi=130); plt.close(fig)
    print("그림:", os.path.join(HERE, "final3.png"))

"""2026-11-28 192447_mu041 — S1 원 (반지름 0.5 · 1.0 m/s · blend 0.6) MPC μ̂ 0.41 ↔ 0.33 번갈아 3 판씩 + PP 1 판.
출발점 A = 183517 seq 와 같음 (--anchor-pose). 비교 묶음: 183517 seq MPC 5 판 (μ̂ 0.33) · PP 5 판.
지표: seq_s1.metrics (위치·진행·안쪽·헤딩·미끄럼 속도·큰 슬립 %·떨림) + sfb_modes.one (바닥 토크 τ_g = P(τ − f) 상한 붙음, mode 별 미끄럼·Δτ)
+ seq_s1_cause.body_force (최고 속력 구간 Jᵀτ 진행·구심). τ̄ 는 판의 μ̂ 로 다시 잡는다 (τ̄ ∝ μ̂).
nice -n 19 /usr/bin/python3 mu041.py → mu041_out.txt, mu041.png"""
import os
import sys

import numpy as np
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import seq_s1 as S  # noqa: E402
import seq_s1_cause as C  # noqa: E402
import sfb_modes as SM  # noqa: E402
from bag_runs import runs  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

BAG = S.B + "2026-11-28/192447_mu041"
TBAR33 = SM.TBAR.copy()


def run_rows(bag, tag):
    RS = runs(bag); D = S.read(bag); F = C.read_sfb(bag); CAP = SM.read_cap(bag); J, _ = C.fit_J(D); out = []
    for r in RS:
        (t_on, t_off), = r["armed"]; c = r["controller"].upper(); mu = (r["mpc"] or {}).get("mu_hat", 0.33)
        m = S.metrics(D, t_on, t_off)
        SM.TBAR = TBAR33 * mu / 0.33
        if c == "MPC":
            e = SM.one(D, F, t_on, t_off, CAP); bf = C.body_force(F, J, m, t_on)
        else:
            e = SM.one(D, None, t_on, t_off); bf = None
        m.update(ctrl=c, mu=mu, run=r["run"], tag=tag, sat=e["sat"], r50=e["ratio50"], r95=e["ratio95"], ew=e["ew"], dt=e["dt"], bf=bf)
        out.append(m)
    SM.TBAR = TBAR33
    return out


if __name__ == "__main__":
    out = open(os.path.join(HERE, "mu041_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    rows = run_rows(BAG, "mu041") + run_rows(S.SEQ, "seq 18:35")
    P("| 묶음 | 판 | 제어기 | μ̂ | 모터 °C | 출발 cm | 위치 RMS cm | 최대 | 진행 (+뒤) | 안쪽 | 옆 RMS | 헤딩 ° | 미끄럼 속도 cm/s | >0.2 m/s % | 큰 슬립 % | 헛돎 % | 떨림 mN·m | 바닥 상한 붙음 % | 구심 힘 τ_MPC / Δτ / 낸 τ N |")
    P("|" + "---|" * 19)
    for m in rows:
        bf = m["bf"]
        P(f"| {m['tag']} | {m['run']} | {m['ctrl']} | {m['mu']:.2f} | {m['temp']:.1f} | {m['start']:.1f} | {m['pos']:.1f} | {m['pos_max']:.1f} | {m['along']:+.1f} | {m['inside']:+.1f} | "
          f"{m['cross_r']:.1f} | {m['head']:.1f} | {m['sv']:.1f} | {m['sv20']:.1f} | {m['m50']:.1f} | {m['spin']:.0f} | {m['dtick']:.1f} | "
          + (f"{m['sat']:.0f} | {bf['mpc'][1]:+.2f} / {bf['dtau'][1]:+.2f} / {bf['out'][1]:+.2f} |" if bf else "- | - |"))

    G = {"MPC μ̂ 0.41 (mu041)": [m for m in rows if m["tag"] == "mu041" and m["ctrl"] == "MPC" and m["mu"] > 0.4],
         "MPC μ̂ 0.33 (mu041)": [m for m in rows if m["tag"] == "mu041" and m["ctrl"] == "MPC" and m["mu"] < 0.4],
         "MPC μ̂ 0.33 (seq 18:35)": [m for m in rows if m["tag"] != "mu041" and m["ctrl"] == "MPC"],
         "PP (mu041)": [m for m in rows if m["tag"] == "mu041" and m["ctrl"] == "PP"],
         "PP (seq 18:35)": [m for m in rows if m["tag"] != "mu041" and m["ctrl"] == "PP"]}
    KEYS = [("pos", "위치 RMS cm"), ("pos_max", "위치 최대 cm"), ("along", "진행 cm (+뒤)"), ("inside", "안쪽 cm"), ("cross_r", "옆 RMS cm"), ("head", "헤딩 RMS °"),
            ("sv", "미끄럼 속도 cm/s"), ("sv20", "미끄럼 > 0.2 m/s %"), ("m50", "큰 슬립 %"), ("spin", "큰 슬립 중 헛돎 %"), ("dtick", "떨림 mN·m"), ("sat", "바닥 상한 붙음 %"), ("temp", "모터 °C")]
    P("\n## 묶음 평균 (최소~최대)")
    P("| 지표 | " + " | ".join(f"{k} (n {len(v)})" for k, v in G.items()) + " |")
    P("|---|" + "---|" * len(G))
    for k, lab in KEYS:
        cells = []
        for v in G.values():
            a = np.array([m[k] for m in v])
            cells.append(f"{a.mean():.1f}" + (f" ({a.min():.1f}~{a.max():.1f})" if len(a) > 1 else ""))
        P(f"| {lab} | " + " | ".join(cells) + " |")
    A, B = G["MPC μ̂ 0.41 (mu041)"], G["MPC μ̂ 0.33 (mu041)"]
    P("\n## μ̂ 0.41 − 0.33 (같은 bag, 번갈아) — 이웃한 짝 (판 1−2, 3−4, 5−6) 차이와 Welch t (n 3 대 3)")
    for k, lab in KEYS[:11]:
        d = [a[k] - b[k] for a, b in zip(A, B)]
        p = stats.ttest_ind([m[k] for m in A], [m[k] for m in B], equal_var=False).pvalue
        sep = "안 겹침" if (max(m[k] for m in A) < min(m[k] for m in B) or min(m[k] for m in A) > max(m[k] for m in B)) else "겹침"
        P(f"- {lab}: 짝 차이 " + ", ".join(f"{x:+.1f}" for x in d) + f" · 평균 {np.mean([m[k] for m in A]):.1f} vs {np.mean([m[k] for m in B]):.1f} (p {p:.3f}, 범위 {sep})")
    P("\n## mode 별 (최고 속력 구간) — 미끄럼 RMS cm/s 앞 · 옆 · 요 · null / Δτ RMS mN·m / 구심 힘 N (τ_MPC · Δτ · 낸 τ)")
    for k in ("MPC μ̂ 0.41 (mu041)", "MPC μ̂ 0.33 (mu041)", "MPC μ̂ 0.33 (seq 18:35)"):
        v = G[k]
        ew = np.mean([[m["ew"][q] for q in SM.MODES] for m in v], 0); dt = np.mean([[m["dt"][q] for q in SM.MODES] for m in v], 0)
        bf = np.mean([[m["bf"]["mpc"][1], m["bf"]["dtau"][1], m["bf"]["out"][1], m["bf"]["mpc"][0], m["bf"]["out"][0]] for m in v], 0)
        P(f"- {k}: 미끄럼 " + " · ".join(f"{x:.1f}" for x in ew) + " | Δτ " + " · ".join(f"{x:.0f}" for x in dt)
          + f" | 구심 {bf[0]:+.2f} · {bf[1]:+.2f} · {bf[2]:+.2f} (요구 m v²/R 9.12) | 진행 방향 τ_MPC {bf[3]:+.2f} → 낸 τ {bf[4]:+.2f}")
    out.close()

    # 그림: 판별 점 (위치 · 진행 · 미끄럼 속도 · 큰 슬립) + 시간축 진행 오차
    fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
    if fp:
        font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
    INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
    COL = ["#6250d6", "#2a78d6", "#86b6ef", "#eb6834", "#f2a07f"]
    fig, axs = plt.subplots(1, 4, figsize=(15, 4.6))
    for ax, (k, lab) in zip(axs, [("pos", "위치 RMS [cm]"), ("along", "진행 평균 [cm] (+ 뒤처짐)"), ("sv", "미끄럼 속도 RMS [cm/s]"), ("m50", "큰 슬립 [%]")]):
        for i, (g, v) in enumerate(G.items()):
            y = [m[k] for m in v]
            ax.scatter(np.full(len(y), i), y, s=50, color=COL[i], zorder=3)
            ax.plot([i - 0.25, i + 0.25], [np.mean(y)] * 2, color=INK, lw=1.4)
        ax.set_xticks(range(len(G))); ax.set_xticklabels(["MPC\n0.41", "MPC\n0.33", "MPC 0.33\n18:35", "PP", "PP\n18:35"], fontsize=8.5)
        ax.set_title(lab, loc="left", fontsize=10, color=INK); ax.grid(axis="y", color=GRID, lw=0.5)
    fig.text(0.01, 0.99, "S1 원 (반지름 0.5 m · 1.0 m/s · blend 0.6) — mu_hat 0.41 ↔ 0.33 번갈아 3 판씩 (19:24 bag mu041) + 18:35 seq 판, 가로 막대 = 평균, 숫자 = mu_hat", fontsize=10.5, color=INK, va="top")
    fig.tight_layout(rect=(0, 0, 1, 0.92)); fig.savefig(os.path.join(HERE, "mu041.png"), dpi=130); plt.close(fig)
    print("그림:", os.path.join(HERE, "mu041.png"))

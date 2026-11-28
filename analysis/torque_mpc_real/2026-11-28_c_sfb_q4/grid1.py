"""S1 원 (반지름 0.5 · 1.0 m/s · blend 0.6) — μ̂ × 되먹임 배율 조합 효과. 2026-11-28 첫 묶음 (202908_grid, 12 조합 1 판씩) +
같은 날 같은 경로 MPC 판 (183517 seq · 192447_mu041 · 200153) 합침. 지표는 mu055.rows_of 와 같다.
1) 조합별 평균 표. 2) 2 차 다항 회귀 y = b0 + b1 μ + b2 s + b3 μ s + b4 μ² + b5 s² (μ = 파라미터 μ̂, s = 배율), 남는 차 표준편차.
3) 회귀로 예측한 격자에서 PP 를 모두 이기는 영역 (위치 < PP · slip velocity < PP · 큰 슬립 < PP · 떨림 ≤ PP).
nice -n 19 /usr/bin/python3 grid1.py → grid1_out.txt, grid1.png"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mu055 as M  # noqa: E402
import seq_s1 as S  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

BAGS = [("grid", S.B + "2026-11-28/202908_grid"), ("20:01", S.B + "2026-11-28/200153"), ("20:00", S.B + "2026-11-28/200047"),
        ("19:24", S.B + "2026-11-28/192447_mu041"), ("18:35", S.SEQ)]
KEYS = [("pos", "위치 RMS cm"), ("sv", "slip velocity cm/s"), ("m50", "큰 슬립 %"), ("dtick", "떨림 mN·m"), ("head", "헤딩 RMS °"), ("sat", "상한 붙음 %")]


def X_of(mu, s):
    mu, s = np.asarray(mu, float), np.asarray(s, float)
    return np.column_stack([np.ones_like(mu), mu, s, mu * s, mu ** 2, s ** 2])


if __name__ == "__main__":
    rows = [m for tag, b in BAGS for m in M.rows_of(tag, b)]
    mpc = [m for m in rows if m["ctrl"] == "MPC"]; pp = [m for m in rows if m["ctrl"] == "PP"]
    out = open(os.path.join(HERE, "grid1_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    ppm = {k: float(np.mean([m[k] for m in pp])) for k, _ in KEYS[:5]}
    P(f"MPC {len(mpc)} 판 · PP {len(pp)} 판. PP 평균: " + " · ".join(f"{lab} {ppm[k]:.1f}" for k, lab in KEYS[:5]))
    P("\n## 1) 조합별 평균 (판 수)")
    P("| μ̂ | 배율 | 판 | " + " | ".join(lab for _, lab in KEYS) + " |")
    P("|---|---|---|" + "---|" * len(KEYS))
    G = {}
    for m in mpc:
        G.setdefault((round(m["mu"], 2), round(m["sc"], 2)), []).append(m)
    for k in sorted(G):
        v = G[k]
        P(f"| {k[0]:.2f} | {k[1]:g} | {len(v)} | " + " | ".join(f"{np.mean([m[q] for m in v]):.1f}" + (f" ({min(m[q] for m in v):.1f}~{max(m[q] for m in v):.1f})" if len(v) > 1 else "")
                                                         for q, _ in KEYS) + " |")
    mu = np.array([m["mu"] for m in mpc]); s = np.array([m["sc"] for m in mpc]); X = X_of(mu, s)
    P("\n## 2) 2 차 회귀 (계수 [1, μ, s, μ·s, μ², s²]) · R² · 남는 차 표준편차 · 같은 조합 반복의 표준편차")
    fits = {}
    for k, lab in KEYS[:4]:
        y = np.array([m[k] for m in mpc]); b, *_ = np.linalg.lstsq(X, y, rcond=None); r = y - X @ b
        rep = [np.std([m[k] for m in v], ddof=1) for v in G.values() if len(v) > 1]
        fits[k] = b
        P(f"- {lab}: b = [{', '.join(f'{x:+.2f}' for x in b)}] · R² {1 - r.var() / y.var():.2f} · 남는 차 σ {r.std(ddof=6):.2f} · 반복 σ (가중 없음 평균) {np.mean(rep):.2f}")
    # 3) 격자 예측
    MU = np.round(np.arange(0.40, 0.851, 0.05), 2); SC = np.round(np.arange(0.8, 1.41, 0.1), 2)
    P("\n## 3) 회귀 예측 격자 — 칸 = 위치 / slip velocity / 큰 슬립 / 떨림. ★ = 넷 다 PP 이하 (위치 < PP−1 cm 여유)")
    P("| μ̂ \\ 배율 | " + " | ".join(f"{x:g}" for x in SC) + " |")
    P("|---|" + "---|" * len(SC))
    best = []
    for a in MU:
        cells = []
        for c in SC:
            pr = {k: float(X_of([a], [c]) @ fits[k]) for k in fits}
            ok = pr["pos"] < ppm["pos"] - 1 and pr["sv"] < ppm["sv"] and pr["m50"] < ppm["m50"] and pr["dtick"] <= ppm["dtick"]
            if ok:
                best.append((pr["pos"] / ppm["pos"] + pr["sv"] / ppm["sv"] + pr["m50"] / ppm["m50"], a, c, pr))
            inside = (0.33 <= a <= 0.80) and (0.7 <= c <= 1.4)
            cells.append(("★" if ok else "") + f"{pr['pos']:.1f}/{pr['sv']:.1f}/{pr['m50']:.0f}/{pr['dtick']:.0f}" + ("" if inside else " (밖)"))
        P(f"| {a:.2f} | " + " | ".join(cells) + " |")
    P("\n★ 칸 중 (위치/PP + slip v/PP + 큰 슬립/PP) 합이 작은 순 8 개:")
    for sc_, a, c, pr in sorted(best)[:8]:
        P(f"- μ̂ {a:.2f} · 배율 {c:g}: 위치 {pr['pos']:.1f} · slip v {pr['sv']:.1f} · 큰 슬립 {pr['m50']:.1f} · 떨림 {pr['dtick']:.1f} (합 {sc_:.2f})")
    out.close()

    fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
    if fp:
        font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
    INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
    fig, axs = plt.subplots(1, 4, figsize=(17, 4.4))
    mm, ss = np.meshgrid(np.linspace(0.33, 0.80, 60), np.linspace(0.7, 1.4, 60))
    for ax, (k, lab) in zip(axs, KEYS[:4]):
        Z = (X_of(mm.ravel(), ss.ravel()) @ fits[k]).reshape(mm.shape)
        cs = ax.contourf(mm, ss, Z, levels=12, cmap="Blues_r" if k != "dtick" else "Blues_r")
        ax.contour(mm, ss, Z, levels=[ppm[k]], colors="#eb6834", linewidths=1.8)
        ax.scatter(mu, s, s=16, color="white", edgecolors=INK, linewidths=0.6, zorder=3)
        fig.colorbar(cs, ax=ax, shrink=0.85)
        ax.set_title(f"{lab} (주황 선 = PP {ppm[k]:.1f})", loc="left", fontsize=10, color=INK)
        ax.set_xlabel("mu_hat (파라미터)"); ax.set_ylabel("되먹임 배율")
    fig.text(0.01, 0.99, "S1 원 MPC 30 판 2 차 회귀 — 진할수록 작음 (좋음), 흰 점 = 돈 판", fontsize=10.5, color=INK, va="top")
    fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig(os.path.join(HERE, "grid1.png"), dpi=130); plt.close(fig)
    print("그림:", os.path.join(HERE, "grid1.png"))

"""2026-11-28 20:46 grid2 — S1 원 두 번째 묶음: (0.50, 1.1) · (0.70, 1.2) · (0.60, 1.2) · (0.55, 1.0) 를 3 판씩 번갈아 + PP 앞뒤.
녹화 중 bag 은 건드리지 않고 복사본 (scratchpad grid2_snap, reindex) 을 읽는다 — 경로는 인자로 받는다.
1) grid2 안에서만 (같은 세션) 조건별 표 + PP. 2) 오늘 S1 모든 판 합친 조건별 표. 3) 후보 넷 대 PP·ADPP Welch t.
nice -n 19 /usr/bin/python3 grid2.py <grid2 bag 경로> → grid2_out.txt"""
import os
import sys

import numpy as np
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import grid1 as G1  # noqa: E402
import mu055 as M  # noqa: E402

KEYS = [("pos", "위치 RMS cm"), ("pos_max", "위치 최대 cm"), ("sv", "slip velocity cm/s"), ("m50", "큰 슬립 %"), ("dtick", "떨림 mN·m"), ("head", "헤딩 RMS °"), ("sat", "상한 붙음 %")]
CAND = [(0.50, 1.1), (0.55, 1.0), (0.60, 1.2), (0.70, 1.2)]


def table(P, rows, title):
    P(f"\n## {title}")
    G = {}
    for m in rows:
        k = (round(m["mu"], 2), round(m["sc"], 2)) if m["ctrl"] == "MPC" else (m["ctrl"], None)
        G.setdefault(k, []).append(m)
    order = sorted([k for k in G if k[1] is not None]) + [k for k in (("PP", None), ("ADPP", None)) if k in G]
    P("| 조건 | 판 | " + " | ".join(lab for _, lab in KEYS) + " |")
    P("|---|---|" + "---|" * len(KEYS))
    for k in order:
        v = G[k]; name = f"MPC μ̂ {k[0]:.2f} · {k[1]:g}" if k[1] is not None else k[0]
        cells = []
        for q, _ in KEYS:
            a = np.array([m[q] for m in v], float)
            cells.append("-" if not np.isfinite(a).any() else f"{np.nanmean(a):.1f}" + (f" ({np.nanmin(a):.1f}~{np.nanmax(a):.1f})" if len(a) > 1 else ""))
        P(f"| {name} | {len(v)} | " + " | ".join(cells) + " |")
    return G


if __name__ == "__main__":
    snap = sys.argv[1]
    out = open(os.path.join(HERE, "grid2_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    g2 = M.rows_of("grid2", snap)
    P(f"grid2 판 {len(g2)} 개 (복사본 {snap}) · 모터 잰/명령 |τ| {min(m['tq'] for m in g2):.2f}~{max(m['tq'] for m in g2):.2f} · 모터 에러 로그 {sum(m['err'] for m in g2)} · 모터 °C {min(m['temp'] for m in g2):.1f}~{max(m['temp'] for m in g2):.1f}")
    for m in g2:
        P(f"  판 {m['run']:2d} {m['ctrl']:4s} μ̂ {m['mu']:.2f} · {m['sc']:g} · 무장 {m['armed']:.1f} s | 위치 {m['pos']:.1f} · slip v {m['sv']:.1f} · 큰 슬립 {m['m50']:.1f} · 떨림 {m['dtick']:.1f} · 헤딩 {m['head']:.1f}")
    Gs = table(P, g2, "1) grid2 안에서만 (같은 세션, 번갈아)")
    allr = [m for tag, b in G1.BAGS for m in M.rows_of(tag, b)] + g2
    Ga = table(P, allr, "2) 오늘 S1 원 모든 판 합침 (18:35 · 19:24 · 20:00 · 20:01 · grid · grid2)")
    P("\n## 3) 후보 넷 대 PP · ADPP (2) 의 판들, Welch t — 음수 = MPC 가 작음 (좋음)")
    for c in CAND:
        A = Ga.get(c, [])
        for ref in ("PP", "ADPP"):
            Bv = Ga.get((ref, None), [])
            if len(A) > 1 and len(Bv) > 1:
                s = []
                for q, lab in KEYS[:6]:
                    x, y = np.array([m[q] for m in A]), np.array([m[q] for m in Bv])
                    s.append(f"{lab} {x.mean() - y.mean():+.1f} (p {stats.ttest_ind(x, y, equal_var=False).pvalue:.3f})")
                P(f"- μ̂ {c[0]:.2f} · {c[1]:g} (n {len(A)}) − {ref} (n {len(Bv)}): " + " · ".join(s))
    out.close()

#!/usr/bin/python3
"""직선 구간 (꼭짓점 0.3 m 밖) 의 yaw 오차가 일정한 어긋남인가: SLAM 요 − 기준 요 의 부호 평균·표준편차, 추정기 요 − SLAM 요 평균."""
import os, math
import numpy as np
src = open("analyze.py").read(); g = {"__file__": os.path.abspath("analyze.py"), "__name__": "h"}
exec(src[:src.index("R = {b: run(b, tb)")], g)
run, cs, CORNERS, w = g["run"], g["cs"], g["CORNERS"], g["w"]
wrap = lambda a: np.arctan2(np.sin(a), np.cos(a))
print("| 판 | 직선 yaw 오차 평균 ± 표준편차 [°] | 변별 평균 [°] | 추정기 − SLAM 요 평균 [°] |")
for lab, bag, tb in [("C2_0", "2026-11-21/190553", 1), ("C2b", "2026-11-21/195636", 1), ("C1a", "2026-11-21/192220", 1),
                     ("P2a", "2026-11-21/192323", None), ("P2b", "2026-11-21/194225", None), ("C3a", "2026-11-21/201411", 1), ("C2a", "2026-11-21/192457", 1)]:
    r = run(bag, np.ones(4) if tb else None); path = f"{g['WS']}/bags/{bag}"; D = cs.load(path)
    P = np.array(D["pose"]); rt = np.array([a for a, _ in D["ref"]])
    ok = np.searchsorted(rt, P[:, 0], side="right") - 1 >= 0; P = P[ok]
    ye = np.degrees(wrap(np.array([th - D["ref"][np.searchsorted(rt, t, side="right") - 1][1].at(t)[0][2] for t, th in zip(P[:, 0], P[:, 3])])))
    mv = r["mv"]; near = np.min(np.hypot(r["lx"][:, None] - CORNERS[None, :, 0], r["ly"][:, None] - CORNERS[None, :, 1]), axis=1) < 0.3
    st = mv & ~near
    # 변 번호: 로컬 좌표에서 가장 가까운 변
    side = np.argmin(np.c_[np.abs(r["ly"]), np.abs(r["lx"] - 1.5), np.abs(r["ly"] - 1.5), np.abs(r["lx"])], axis=1)
    per = " / ".join(f"{ye[st & (side == k)].mean():+.1f}" for k in range(4) if (st & (side == k)).any())
    S, E, _, A = w.load(path)
    eo = np.degrees(wrap(np.interp(P[:, 0], E[:, 0], np.unwrap(E[:, 3])) - P[:, 3]))
    print(f"| {lab} | {ye[st].mean():+.1f} ± {ye[st].std():.1f} | {per} | {eo[st].mean():+.2f} |")

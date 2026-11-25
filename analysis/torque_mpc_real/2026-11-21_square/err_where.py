#!/usr/bin/python3
"""C2 (Q11=Q22=20) 세 판 + P2 두 판: contour·yaw 오차가 꼭짓점 (0.3 m 안) 과 직선 중 어디서 나나. analyze.py run() 의 로컬 좌표 + SLAM 요 − 기준 요."""
import os, math
import numpy as np
src = open("analyze.py").read(); g = {"__file__": os.path.abspath("analyze.py"), "__name__": "h"}
exec(src[:src.index("R = {b: run(b, tb)")], g)
run, cs, CORNERS = g["run"], g["cs"], g["CORNERS"]
TB = np.array([0.1230, 0.1253, 0.1298, 0.1187])
print("| 판 | 꼭짓점 시간 몫 | contour RMS 꼭짓점 / 직선 [mm] | contour² 중 꼭짓점 몫 | yaw RMS 꼭짓점 / 직선 [°] | yaw² 중 꼭짓점 몫 |")
for lab, bag, tb in [("C2_0", "2026-11-21/190553", TB), ("C2a", "2026-11-21/192457", TB), ("C2b", "2026-11-21/195636", TB),
                     ("P2a", "2026-11-21/192323", None), ("P2b", "2026-11-21/194225", None), ("C3a", "2026-11-21/201411", TB)]:
    r = run(bag, tb); D = cs.load(f"{g['WS']}/bags/{bag}")
    P = np.array(D["pose"]); rt = np.array([a for a, _ in D["ref"]])
    yerr = []
    for t, th in zip(P[:, 0], P[:, 3]):
        i = np.searchsorted(rt, t, side="right") - 1
        if i >= 0:
            yerr.append(math.atan2(math.sin(th - D["ref"][i][1].at(t)[0][2]), math.cos(th - D["ref"][i][1].at(t)[0][2])))
    yerr = np.degrees(np.array(yerr))[: len(r["t"])]
    mv = r["mv"]; d = r["contour"]
    near = np.min(np.hypot(r["lx"][:, None] - CORNERS[None, :, 0], r["ly"][:, None] - CORNERS[None, :, 1]), axis=1) < 0.3
    c, s_ = mv & near, mv & ~near
    rms = lambda a, m: float(np.sqrt(np.mean(a[m] ** 2)))
    print(f"| {lab} | {100*c.sum()/mv.sum():.0f} % | {1e3*rms(d,c):.0f} / {1e3*rms(d,s_):.0f} | {100*np.sum(d[c]**2)/np.sum(d[mv]**2):.0f} % | "
          f"{rms(yerr,c):.1f} / {rms(yerr,s_):.1f} | {100*np.sum(yerr[c]**2)/np.sum(yerr[mv]**2):.0f} % |")

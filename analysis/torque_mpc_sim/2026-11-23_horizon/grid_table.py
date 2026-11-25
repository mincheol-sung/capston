#!/usr/bin/python3
"""grid_raw.txt (grid.cpp 출력) → 장면·속도별 표. 장면 0 외란 없음 · 1 실차 외란 (d_x −0.155, d_y +0.085 m/s²) + d̂ · 2 같은 외란, d̂ 끔. C 상한."""
import re
rows = []
for l in open("grid_raw.txt"):
    p = l.split(); d = dict(Np=int(p[0]), Nc=int(p[1]), sc=int(p[2]), v=float(p[3]))
    for k, v in re.findall(r"(s_l|s_r|pathRMS|timeRMS|dtau|tmax|sat) ([+\-\d.]+)", l):
        d[k] = float(v)
    rows.append(d)
names = {0: "외란 없음", 1: "실차 외란 + d̂", 2: "실차 외란, d̂ 끔"}
for sc in (0, 1, 2):
    for v in (0.61, 0.80):
        print(f"\n== {names[sc]}  v {v}:  Np  Nc | s (두 원 평균 안쪽) · 경로 RMS · 시간 RMS [mm] · Δτ RMS · 포화 %")
        for d in sorted([r for r in rows if r["sc"] == sc and r["v"] == v], key=lambda d: (d["Np"], d["Nc"])):
            print(f"  {d['Np']:4d} {d['Nc']:3d} | {(d['s_l']+d['s_r'])/2:+6.2f}  {d['pathRMS']:6.2f}  {d['timeRMS']:6.2f}  {d['dtau']:.5f}  {d['sat']:5.1f}")

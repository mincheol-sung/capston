#!/usr/bin/python3
"""바퀴별 코스트다운 비교 (2026-11-16): 토크 0 (MIT kp = kd = 0, 자유 회전) 구간의 감속으로 마찰을 잰다.
  J·ω̇ = −τ_f(ω)  →  τ_f(ω) = −J·ω̇   (J = 같은 날 계단 차분 측정의 바퀴별 중앙값)
마찰 모양을 가정하지 않고 같은 ω 에서 바퀴끼리 바로 비교한다. 서는 시간(ω 15 → 2 rad/s)도 낸다.

  측정:  ak40_bench.py steps --id N --taus 0.09,0.06 --hold 5                         # 워밍업
         ak40_bench.py steps --id N --taus 0.08,0.06,0.06,0 --hold 1.5 --csv coast_N_k.csv  # k = 1..3
  분석:  /usr/bin/python3 coast_analysis.py coast_*.csv
"""
import csv
import glob
import os
import sys

import numpy as np

J = {1: 9.76e-4, 2: 9.35e-4, 3: 7.3e-4, 4: 9.12e-4}          # 2026-11-16_jw 계단 차분 중앙값
NAME = {1: "FL", 2: "FR", 3: "RL", 4: "RR"}
LEVELS = (5.0, 10.0, 15.0, 20.0)


def coast_segment(path):
    rows = [r for r in csv.DictReader(open(path)) if r["mode"] == "torque"]
    t = np.array([float(r["t"]) for r in rows]); v = np.abs(np.array([float(r["v"]) for r in rows]))
    tc = np.array([float(r["tau_cmd"]) for r in rows])
    i0 = np.flatnonzero(tc == 0.0)
    if len(i0) == 0:
        return None
    sel = slice(i0[0] + 2, i0[-1] + 1)                          # 명령 지연 2 샘플은 뺀다
    return t[sel] - t[i0[0]], v[sel]


def main():
    files = sorted({f for p in (sys.argv[1:] or ["coast_*.csv"]) for f in glob.glob(p)})
    by = {}
    for f in files:
        wid = int(os.path.basename(f).split("_")[1])
        seg = coast_segment(f)
        if seg is not None:
            by.setdefault(wid, []).append(seg)
    print("바퀴 | 판 | 시작 ω  | 15→2 rad/s 서는 시간 s | τ_f = −J·ω̇ [mN·m] @ " + " / ".join(f"{w:.0f}" for w in LEVELS) + " rad/s")
    for wid in sorted(by):
        stops, tau = [], {w: [] for w in LEVELS}
        for tt, vv in by[wid]:
            if vv.max() > 15 and vv.min() < 2:
                t15 = np.interp(15, vv[::-1], tt[::-1]); t2 = np.interp(2, vv[::-1], tt[::-1]); stops.append(t2 - t15)
            for w in LEVELS:                                    # ω 가 w 근처(±2.5)인 구간을 직선으로 맞춘 기울기
                m = np.abs(vv - w) < 2.5
                if m.sum() >= 4:
                    slope = np.polyfit(tt[m], vv[m], 1)[0]
                    tau[w].append(-J[wid] * slope)
        cells = " / ".join(f"{1e3*np.median(tau[w]):5.1f}" if tau[w] else "  -  " for w in LEVELS)
        print(f" {NAME[wid]}  | {len(by[wid])}  | {np.median([v.max() for _, v in by[wid]]):5.1f}  | "
              f"{np.median(stops) if stops else float('nan'):.3f}                 | {cells}")


if __name__ == "__main__":
    main()

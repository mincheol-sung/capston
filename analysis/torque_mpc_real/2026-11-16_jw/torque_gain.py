"""코스트다운(전류 0)의 감속으로 기계 마찰 f(ω) = J·|ω̇| 를 구하고, 같은 바퀴가 τ_명령 0.06 에서 머문 정상 속도 ω_ss 에서
f(ω_ss) / 0.06 = 명령 대비 실제로 바퀴에 나간 토크의 비 k 를 낸다 (정상 상태: k·τ_명령 = f(ω_ss)).
J 는 네 바퀴 같은 부품이라 공통 8.77e-4 (계단 차분 전체 중앙)로 둔다. 전압 한계(≈ 42 rad/s)에 닿은 바퀴는 k 의 하한만 나온다."""
import csv, glob, numpy as np
J = 8.77e-4
NAME = {1: "FL", 2: "FR", 3: "RL", 4: "RR"}
WSS = {1: 42.11, 2: 28.81, 3: 42.04, 4: 31.34}           # 워밍업 0.06 N·m 5 s 의 마지막 1 s 평균 (벤치 출력)
REPLY = {1: 0.0555, 2: 0.0577, 3: 0.0568, 4: 0.0586}     # 같은 구간 응답 토크
print("바퀴 | 0.06 N·m 정상 ω | 코스트 감속 @ω_ss rad/s² | 기계 마찰 f(ω_ss) N·m | k = f/τ_명령 | 응답 토크 / 명령")
for wid in (1, 2, 3, 4):
    pts = []
    for f in sorted(glob.glob(f"coast_{wid}_*.csv")):
        rows = [r for r in csv.DictReader(open(f)) if r["mode"] == "torque"]
        t = np.array([float(r["t"]) for r in rows]); v = np.abs(np.array([float(r["v"]) for r in rows])); tc = np.array([float(r["tau_cmd"]) for r in rows])
        i0 = np.flatnonzero(tc == 0.0)[0] + 2
        tt, vv = t[i0:], v[i0:]
        m = (vv > 3) & (vv < 45)
        c = np.polyfit(tt[m] - tt[0], vv[m], 3)              # ω(t) 3 차 → 감속을 ω 의 함수로
        d = np.polyder(c)
        for s in np.linspace(0, (tt[m] - tt[0]).max(), 200):
            pts.append((np.polyval(c, s), -np.polyval(d, s)))
    P = np.array(pts); P = P[np.argsort(P[:, 0])]
    w = WSS[wid]
    lim = w > 41
    near = np.abs(P[:, 0] - w) < 2.0
    dec = np.median(P[near, 1]) if near.any() else (np.median(P[P[:, 0] > P[:, 0].max() - 3, 1]) if True else np.nan)
    tag = " (코스트 시작 속도 근처 값)" if not near.any() else ""
    f = J * dec
    print(f" {NAME[wid]}  | {w:5.1f}{'(전압 한계)' if lim else '':10s} | {dec:6.1f}{tag:20s} | {f:.4f}                 | {f/0.06:.2f}{' 이상' if lim else '':5s}  | {REPLY[wid]/0.06:.3f}")

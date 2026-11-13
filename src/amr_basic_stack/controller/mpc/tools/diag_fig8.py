#!/usr/bin/env python3
"""8자 주행 CSV 진단 — 실차에서 8자가 안 나올 때 원인을 가른다.
   python3 diag_fig8.py fig8_real_YYYYmmdd_HHMMSS.csv
"""
import sys, math
import numpy as np

K_GEOM, V_LIMIT = 0.19835, 1.20
R = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
t, x, y, th = R["t"] - R["t"][0], R["x"], R["y"], R["th"]
xr, yr, thr = R["xr"], R["yr"], R["thr"]
e = np.hypot(R["xe"], R["ye"]); the = np.abs(R["the"])
print(f"{sys.argv[1]}   {len(R)} 행,  {t[-1]:.1f} s,  주기 {np.median(np.diff(t))*1000:.0f} ms\n")

print("① 기준열이 8자였나  (기준 xr,yr 의 모양)")
print(f"   경계상자  가로 {xr.max()-xr.min():.3f} m   세로 {yr.max()-yr.min():.3f} m")
sx = np.sign(xr - xr[0]); flips = int((np.diff(sx[sx != 0]) != 0).sum())
print(f"   기동점 좌우 교차 {flips} 회  (8자면 두 잎을 오가므로 ≥1, 원이면 0)")
print(f"   → {'기준열은 8자다' if xr.max()-xr.min() > 0.5 and flips >= 1 else '★ 기준열부터 8자가 아니다 — 노드/파라미터 문제'}\n")

print("② MPC 가 기준을 따라갔나  (odom 기준 오차)")
print(f"   위치오차  RMS {np.sqrt(np.mean(e**2))*100:5.2f} cm   최대 {e.max()*100:6.2f} cm")
print(f"   헤딩오차  RMS {np.degrees(np.sqrt(np.mean(the**2))):5.2f} °    최대 {np.degrees(the.max()):6.2f} °")
ok2 = e.max() < 0.10
print(f"   → {'MPC 는 따라갔다 (odom 상으로는)' if ok2 else '★ MPC 가 못 따라갔다'}\n")

print("③ 명령이 포화했나")
load = R["load"] / V_LIMIT
print(f"   부하 평균 {load.mean()*100:3.0f} %  최대 {load.max()*100:3.0f} %   "
      f"95 % 초과 {int((load > 0.95).sum())} 스텝 ({(load>0.95).mean()*100:.1f} %)")
print(f"   QP 폴백 {int(R['fallback'].sum())} 스텝,  step 최대 {R['step_ms'].max():.1f} ms (주기 40)")
print(f"   → {'여유 있다' if load.max() < 0.9 else '★ 포화 — 속도를 낮춰라'}\n")

print("④ 기준열이 끊겼나")
print(f"   기준 나이 평균 {R['ref_age'].mean():.3f} s  최대 {R['ref_age'].max():.3f} s  (timeout 0.5)")
gap = np.diff(t); print(f"   틱 누락  최대 간격 {gap.max()*1000:.0f} ms,  100 ms 초과 {int((gap>0.1).sum())} 회")
m = t < t[-1] - 1.0                       # 종료 직전 마지막 발행은 timeout 까지 늙는다 — 제외
print(f"   → {'정상' if R['ref_age'][m].max() < 0.45 and gap.max() < 0.1 else '★ 끊김/지연 있음'}\n")

print("⑤ 명령 vs 실측 속도  (구동계가 명령을 냈나 — odom 신뢰도의 1차 근거)")
for i, ax in enumerate(("vx", "vy", "wz")):
    u, v = R[("ux", "uy", "uw")[i]], R[ax]
    if np.abs(u).max() < 1e-3:
        print(f"   {ax:3s} 명령 없음"); continue
    lag = int(np.argmax([np.corrcoef(u[:-20], np.roll(v, -k)[:-20])[0,1] for k in range(20)]))
    gain = np.polyfit(u, v, 1)[0]
    print(f"   {ax:3s} |u|max {np.abs(u).max():5.2f}  |v|max {np.abs(v).max():5.2f}  "
          f"이득 {gain:4.2f}  지연 {lag*40} ms")
print("   → 이득이 1 에서 크게 벗어나면 구동계/캘리브레이션. 특히 wz.\n")

print("⑥ 폐합 (실제로 제자리에 왔나 — odom 기준)")
print(f"   시작 ({x[0]:+.3f}, {y[0]:+.3f}, {math.degrees(th[0]):+.1f}°)")
print(f"   끝   ({x[-1]:+.3f}, {y[-1]:+.3f}, {math.degrees(th[-1]):+.1f}°)")
print(f"   차   {math.hypot(x[-1]-x[0], y[-1]-y[0])*100:.1f} cm,  {math.degrees(th[-1]-th[0]):+.1f}°")
print("   ★ odom 은 제자리인데 로봇이 실제로 제자리가 아니면 → odom 이 원인 (슬립/IMU)\n")

print("⑦ 언제 깨졌나  (2 s 구간별) — 첫 줄부터 나쁘면 초기조건/배선, 중간부터면 발산")
print(f"   {'t[s]':>8} {'오차cm':>7} {'부하%':>6} {'|u|vx':>6} {'|v|vx':>6} {'|u|wz':>6} {'|v|wz':>6} {'이득vx':>6} {'QP':>4}")
for lo in np.arange(0, t[-1], 2.0):
    m = (t >= lo) & (t < lo + 2.0)
    if m.sum() < 5:
        continue
    uvx, vvx = R["ux"][m], R["vx"][m]
    g = float(np.polyfit(uvx, vvx, 1)[0]) if np.std(uvx) > 1e-3 else float("nan")
    print(f"   {lo:5.0f}-{lo+2:<3.0f} {e[m].max()*100:6.1f} {R['load'][m].max()/V_LIMIT*100:5.0f} "
          f"{np.abs(uvx).max():6.2f} {np.abs(vvx).max():6.2f} "
          f"{np.abs(R['uw'][m]).max():6.2f} {np.abs(R['wz'][m]).max():6.2f} {g:6.2f} "
          f"{int(R['qp_iters'][m].max()):4d}")
print()

W, H = 61, 21
sx_, sy_ = max(np.abs(x).max(), np.abs(xr).max(), 1e-3), max(np.abs(y).max(), np.abs(yr).max(), 1e-3)
g = [[' ']*W for _ in range(H)]
for P, ch in ((np.c_[xr, yr], '.'), (np.c_[x, y], '#')):
    for px, py in P:
        c = int(round((px/sx_*0.5+0.5)*(W-1))); r = int(round((0.5-py/sy_*0.5)*(H-1)))
        if 0 <= c < W and 0 <= r < H: g[r][c] = ch
print(f"기준 '.'  실주행 '#'   (가로 ±{sx_:.2f} m, 세로 ±{sy_:.2f} m — 축척 다름)")
print('\n'.join(''.join(r) for r in g))

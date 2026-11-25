# 확인 주행 (check_run.npz) 에서 외란 추정이 마찰의 어느 만큼을 맡았나:  /usr/bin/python3 dhat_check.py
# /torque_mpc/status 14–19 는 d̂ 의 "한 스텝 상태 증분" [½Ts²·a ; Ts·a] (몸체 좌표) 다 — 가속도는 17–19 를 Ts 로 나눈 값.
import os
import numpy as np
A = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_run.npz"))
st, cap, ws = A["st"], A["cap"], A["ws"]; t_on = float(A["t_on"])
Ts, r, m_eff = 0.01, 0.04, 4.56 + 4 * 8.9e-4 / 0.04**2
D = st[:, 1:]; n = min(len(D), len(cap)); t = st[:n, 0] - t_on
ad, tau, us, tb = D[:n, 17:20] / Ts, D[:n, 4:8], D[:n, 22:26], cap[:n, 16:20]
om = np.interp(st[:n, 0], ws[:, 0], ws[:, 5:9].mean(1))
print(f"d̂ 최대 크기: 앞 {np.abs(ad[:,0]).max():.2f} · 옆 {np.abs(ad[:,1]).max():.2f} m/s², 회전 {np.abs(ad[:,2]).max():.2f} rad/s² (상한 0.5 · 2.0)")
print("| 무장 뒤 s | 바퀴 속도 rad/s | d̂ 앞 · 옆 m/s² · 회전 rad/s² | d̂ 앞 성분 → 바퀴당 mN·m | 모델 마찰 | 목표 입력 | 실제 명령 | 명령 − 목표 |")
print("|---|---|---|---|---|---|---|---|")
for a in range(0, 16):
    s = (t >= a) & (t < a + 1)
    if s.any():
        print(f"| {a}~{a+1} | {om[s].mean():.2f} | {ad[s,0].mean():+.3f} · {ad[s,1].mean():+.3f} · {ad[s,2].mean():+.3f} | {-m_eff*ad[s,0].mean()*r/4*1e3:+.1f} | {tb[s].mean()*1e3:+.1f} | {us[s].mean()*1e3:+.1f} | {tau[s].mean()*1e3:+.1f} | {(tau[s].mean()-us[s].mean())*1e3:+.1f} |")
for lab, a, b in (("전진 등속", 4.3, 5.6), ("후진 등속", 8.4, 9.7)):
    s = (t >= a) & (t <= b); sg = np.sign(om[s].mean())
    fm, tq, dh = tb[s].mean() * sg, tau[s].mean() * sg, -m_eff * ad[s, 0].mean() * r / 4 * sg
    print(f"{lab}: 실제 명령 {tq*1e3:.1f} = 모델 마찰 {fm*1e3:.1f} + 외란 추정 몫 {dh*1e3:.1f} + 나머지 {(tq-fm-dh)*1e3:.1f} mN·m/바퀴 → 모델이 못 맞춘 {(tq-fm)*1e3:.1f} 중 외란 추정이 {100*dh/(tq-fm):.0f} %")
